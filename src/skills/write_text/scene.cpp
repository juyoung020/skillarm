#include "scene.hpp"

#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace wt {

#ifndef SKILLARM_ROOT
#define SKILLARM_ROOT "."
#endif

std::string skillarm_root() {
  if (const char* e = std::getenv("SKILLARM_ROOT")) return e;
  return SKILLARM_ROOT;
}

Scene::~Scene() {
  if (m) mj_deleteModel(m);
}

namespace {
void set_quat(double dst[4], const M3& R) {
  const auto q = quat_of(R);
  for (int i = 0; i < 4; ++i) dst[i] = q[size_t(i)];
}

mjsBody* body_or_throw(mjSpec* s, const char* name) {
  mjsBody* b = mjs_findBody(s, name);
  if (!b) throw std::runtime_error(std::string("body missing in robot MJCF: ") + name);
  return b;
}

void add_pencil_geoms(mjSpec* spec, mjsBody* b, const PencilGeom& g, bool collide) {
  // convex hexagonal prism + cone, tip at the origin, +z towards the back end; vertices at 30 + 60 k deg -> a flat
  // faces +x
  std::vector<float> v;
  const double R = g.circumradius();
  auto ring = [&](double r, double z) {
    for (int k = 0; k < 6; ++k) {
      const double a = rad(30.0 + 60.0 * k);
      v.push_back(float(r * std::cos(a)));
      v.push_back(float(r * std::sin(a)));
      v.push_back(float(z));
    }
  };
  ring(g.lead_r, 0.0015);
  ring(R, g.cone);
  ring(R, g.length);
  mjsMesh* mesh = mjs_addMesh(spec, nullptr);
  mjs_setName(mesh->element, "pencil_hull");
  mjs_setFloat(mesh->uservert, v.data(), int(v.size()));

  mjsGeom* pg = mjs_addGeom(b, nullptr);
  mjs_setName(pg->element, "pencil");
  pg->type = mjGEOM_MESH;
  mjs_setString(pg->meshname, "pencil_hull");
  pg->density = g.density;
  pg->contype = collide ? 1 : 0;
  pg->conaffinity = collide ? 1 : 0;
  pg->condim = 4;
  pg->priority = 1;  // its friction (not the pads' 1.5) decides pencil contacts
  pg->friction[0] = g.friction; pg->friction[1] = 0.002; pg->friction[2] = 0.0001;
  pg->rgba[0] = 0.95f; pg->rgba[1] = 0.75f; pg->rgba[2] = 0.1f; pg->rgba[3] = 1.0f;

  mjsGeom* tg = mjs_addGeom(b, nullptr);
  mjs_setName(tg->element, "pencil_tip");
  tg->type = mjGEOM_SPHERE;
  tg->size[0] = g.tip_r;
  tg->pos[2] = g.tip_r;
  tg->density = 2000;
  tg->contype = collide ? 1 : 0;
  tg->conaffinity = collide ? 1 : 0;
  tg->condim = 3;
  tg->priority = 2;
  tg->friction[0] = g.tip_friction; tg->friction[1] = 0.001; tg->friction[2] = 0.0001;
  tg->rgba[0] = 0.15f; tg->rgba[1] = 0.15f; tg->rgba[2] = 0.15f; tg->rgba[3] = 1.0f;
}
}  // namespace

Pose pencil_on_desk(const PencilGeom& g, double desk_z, double cx, double cy, double tip_yaw) {
  // pencil +z (tip -> back) points opposite to the tip direction; +x (a flat normal) points down onto the desk
  const V3 tipdir{std::cos(tip_yaw), std::sin(tip_yaw), 0};
  const V3 z = -tipdir, x{0, 0, -1};
  const V3 y = cross(z, x);
  Pose p;
  p.R = M3::from_cols(x, y, z);
  const double half_flat = 0.5 * g.flat;
  // centre of the hex prism part is at length/2 along +z from the tip (close enough for the pose input)
  p.p = V3{cx, cy, desk_z + half_flat} + tipdir * (0.5 * g.length);
  return p;
}

void build_scene(const SceneConfig& c, Scene& S) {
  const std::string root = skillarm_root();
  const std::string robot = c.robot_xml.empty() ? root + "/models/so101/so101_task_robot.xml" : c.robot_xml;
  char err[2048] = {};
  mjSpec* spec = mj_parseXML(robot.c_str(), nullptr, err, sizeof err);
  if (!spec) throw std::runtime_error(std::string("mj_parseXML: ") + err);

  spec->option.timestep = 0.002;
  spec->option.integrator = mjINT_IMPLICITFAST;
  spec->option.cone = mjCONE_ELLIPTIC;
  spec->option.impratio = 10;
  if (c.render) {
    spec->visual.global.offwidth = 1280;
    spec->visual.global.offheight = 960;
    spec->visual.headlight.diffuse[0] = spec->visual.headlight.diffuse[1] = spec->visual.headlight.diffuse[2] = 0.6f;
    spec->visual.headlight.ambient[0] = spec->visual.headlight.ambient[1] = spec->visual.headlight.ambient[2] = 0.35f;
  }
  mjsBody* world = mjs_findBody(spec, "world");
  if (!world) throw std::runtime_error("no world body");

  // desk
  mjsGeom* desk = mjs_addGeom(world, nullptr);
  mjs_setName(desk->element, "desk");
  desk->type = mjGEOM_PLANE;
  desk->size[0] = 0; desk->size[1] = 0; desk->size[2] = 0.05;
  desk->pos[2] = c.desk_z;
  desk->contype = 1; desk->conaffinity = 1;
  desk->friction[0] = 0.6;
  desk->rgba[0] = 0.62f; desk->rgba[1] = 0.55f; desk->rgba[2] = 0.47f; desk->rgba[3] = 1;

  if (c.with_paper) {
    mjsGeom* pp = mjs_addGeom(world, nullptr);
    mjs_setName(pp->element, "paper");
    pp->type = mjGEOM_BOX;
    pp->size[0] = 0.5 * c.paper.w; pp->size[1] = 0.5 * c.paper.h; pp->size[2] = 0.5 * c.paper.thick;
    pp->pos[0] = c.paper.cx; pp->pos[1] = c.paper.cy; pp->pos[2] = c.desk_z + 0.5 * c.paper.thick;
    set_quat(pp->quat, rot_z(c.paper.yaw));
    pp->contype = 1; pp->conaffinity = 1;
    pp->friction[0] = 0.5;
    pp->solref[0] = -c.paper.underlay_stiffness;
    pp->solref[1] = -c.paper.underlay_damping;
    pp->solimp[0] = 0.99; pp->solimp[1] = 0.99; pp->solimp[2] = 0.001;
    pp->rgba[0] = 0.97f; pp->rgba[1] = 0.97f; pp->rgba[2] = 0.95f; pp->rgba[3] = 1;
  }

  mjsBody* gripper = body_or_throw(spec, "gripper");
  if (c.mount) {
    const std::string stl = c.mount_stl.empty() ? root + "/models/so101/accessories/c270_wrist_mount.stl" : c.mount_stl;
    mjsMesh* mm = mjs_addMesh(spec, nullptr);
    mjs_setName(mm->element, "wrist_mount_hull");
    mjs_setString(mm->file, stl.c_str());
    mm->scale[0] = mm->scale[1] = mm->scale[2] = 0.001;
    mjsGeom* mg = mjs_addGeom(gripper, nullptr);
    mjs_setName(mg->element, "wrist_mount");
    mg->type = mjGEOM_MESH;
    mjs_setString(mg->meshname, "wrist_mount_hull");
    mg->contype = 1; mg->conaffinity = 0; mg->group = 3;
    mg->density = 0;  // the mount's mass is already in the measured gripper inertia (approx.)
    mg->mass = 0;
    mg->rgba[0] = 0.2f; mg->rgba[1] = 0.2f; mg->rgba[2] = 0.22f; mg->rgba[3] = 1;
  }

  if (c.mode == PencilMode::kFree) {
    mjsBody* pb = mjs_addBody(world, nullptr);
    mjs_setName(pb->element, "pencil");
    pb->pos[0] = c.pencil_world.p.x; pb->pos[1] = c.pencil_world.p.y; pb->pos[2] = c.pencil_world.p.z;
    set_quat(pb->quat, c.pencil_world.R);
    mjsJoint* fj = mjs_addFreeJoint(pb);
    mjs_setName(fj->element, "pencil_free");
    add_pencil_geoms(spec, pb, c.pencil, true);
  } else if (c.mode == PencilMode::kAttached) {
    mjsBody* pb = mjs_addBody(gripper, nullptr);
    mjs_setName(pb->element, "pencil");
    pb->pos[0] = c.pencil_in_gripper.p.x; pb->pos[1] = c.pencil_in_gripper.p.y; pb->pos[2] = c.pencil_in_gripper.p.z;
    set_quat(pb->quat, c.pencil_in_gripper.R);
    add_pencil_geoms(spec, pb, c.pencil, true);
  }

  if (c.render) {
    mjsLight* l = mjs_addLight(world, nullptr);
    l->pos[0] = 0.2; l->pos[2] = 1.5; l->dir[2] = -1;
    l->type = mjLIGHT_DIRECTIONAL;
    l->diffuse[0] = l->diffuse[1] = l->diffuse[2] = 0.5f;
    mjsCamera* cam = mjs_addCamera(world, nullptr);
    mjs_setName(cam->element, "overview");
    cam->pos[0] = 0.62; cam->pos[1] = -0.42; cam->pos[2] = 0.42;
    cam->alt.type = mjORIENTATION_XYAXES;
    // look at (0.2, 0.0, 0.0)
    {
      const V3 eye{0.62, -0.42, 0.42}, at{0.2, 0.02, 0.0};
      const V3 f = normalize(at - eye), x = normalize(cross(f, V3{0, 0, 1})), y = cross(x, f);
      cam->alt.xyaxes[0] = x.x; cam->alt.xyaxes[1] = x.y; cam->alt.xyaxes[2] = x.z;
      cam->alt.xyaxes[3] = y.x; cam->alt.xyaxes[4] = y.y; cam->alt.xyaxes[5] = y.z;
    }
    cam->fovy = 45;
    mjsCamera* top = mjs_addCamera(world, nullptr);
    mjs_setName(top->element, "paper_top");
    top->pos[0] = c.paper.cx - 0.12; top->pos[1] = c.paper.cy; top->pos[2] = 0.55;
    top->alt.type = mjORIENTATION_XYAXES;
    {
      const V3 eye{c.paper.cx - 0.12, c.paper.cy, 0.55}, at{c.paper.cx, c.paper.cy, c.desk_z};
      const V3 f = normalize(at - eye), x = normalize(cross(f, V3{1, 0, 0}) * -1.0), y = cross(x, f);
      top->alt.xyaxes[0] = x.x; top->alt.xyaxes[1] = x.y; top->alt.xyaxes[2] = x.z;
      top->alt.xyaxes[3] = y.x; top->alt.xyaxes[4] = y.y; top->alt.xyaxes[5] = y.z;
    }
    top->fovy = 40;
  }

  mjModel* m = mj_compile(spec, nullptr);
  if (!m) {
    std::string e = mjs_getError(spec);
    mj_deleteSpec(spec);
    throw std::runtime_error("mj_compile: " + e);
  }
  mj_deleteSpec(spec);
  if (S.m) mj_deleteModel(S.m);
  S.m = m;

  const char* jn[6] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
  for (int j = 0; j < 6; ++j) {
    const int id = mj_name2id(m, mjOBJ_JOINT, jn[j]);
    if (id < 0) throw std::runtime_error(std::string("joint missing: ") + jn[j]);
    S.qadr[j] = m->jnt_qposadr[id];
    S.dadr[j] = m->jnt_dofadr[id];
    S.lo[j] = m->jnt_range[2 * id];
    S.hi[j] = m->jnt_range[2 * id + 1];
    S.act[j] = mj_name2id(m, mjOBJ_ACTUATOR, jn[j]);
  }
  if (S.act[5] >= 0) {
    m->actuator_forcerange[2 * S.act[5]] = -c.grip_torque;
    m->actuator_forcerange[2 * S.act[5] + 1] = c.grip_torque;
  }
  S.gripper_body = mj_name2id(m, mjOBJ_BODY, "gripper");
  S.site = mj_name2id(m, mjOBJ_SITE, "gripperframe");
  S.pencil_body = mj_name2id(m, mjOBJ_BODY, "pencil");
  S.pencil_geom = mj_name2id(m, mjOBJ_GEOM, "pencil");
  S.tip_geom = mj_name2id(m, mjOBJ_GEOM, "pencil_tip");
  S.paper_geom = mj_name2id(m, mjOBJ_GEOM, "paper");
  S.desk_geom = mj_name2id(m, mjOBJ_GEOM, "desk");
  S.mount_geom = mj_name2id(m, mjOBJ_GEOM, "wrist_mount");
  S.fixed_pad = mj_name2id(m, mjOBJ_GEOM, "fixed_jaw_pad");
  S.moving_pad = mj_name2id(m, mjOBJ_GEOM, "moving_jaw_pad");
  const int fj = mj_name2id(m, mjOBJ_JOINT, "pencil_free");
  if (fj >= 0) { S.pencil_free_qadr = m->jnt_qposadr[fj]; S.pencil_free_dadr = m->jnt_dofadr[fj]; }

  const char* links[7] = {"base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"};
  S.link_geoms.assign(7, {});
  S.arm_geoms.clear();
  for (int i = 0; i < 7; ++i) {
    const int b = mj_name2id(m, mjOBJ_BODY, links[i]);
    if (b < 0) continue;
    for (int g = 0; g < m->ngeom; ++g) {
      if (m->geom_bodyid[g] != b) continue;
      const bool coll = m->geom_contype[g] != 0 || (i == 0 && m->geom_type[g] == mjGEOM_MESH && m->geom_group[g] == 2);
      if (!coll) continue;
      S.link_geoms[size_t(i)].push_back(g);
      if (i > 0) S.arm_geoms.push_back(g);
    }
  }
}

}  // namespace wt
