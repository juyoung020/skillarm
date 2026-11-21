#include "sim/so101_model.hpp"

#include <limits>
#include <stdexcept>

#include "core/stl.hpp"

namespace skillarm {

So101Model::So101Model(const std::string& scene_xml) {
  char err[1024] = {};
  m_ = mj_loadXML(scene_xml.c_str(), nullptr, err, sizeof err);
  if (!m_) throw std::runtime_error(std::string("mj_loadXML: ") + err);
  d_ = mj_makeData(m_);
  for (int j = 0; j < kJoints; ++j) {
    const int id = mj_name2id(m_, mjOBJ_JOINT, kJointNames[j]);
    if (id < 0) throw std::runtime_error(std::string("joint not in model: ") + kJointNames[j]);
    qadr_[j] = m_->jnt_qposadr[id];
  }
  const int base = mj_name2id(m_, mjOBJ_BODY, "base");
  for (int g = 0; g < m_->ngeom; ++g) {
    const int b = m_->geom_bodyid[g];
    if (b != 0 && b != base && m_->geom_type[g] == mjGEOM_MESH) arm_geoms_.push_back(g);
  }
}

So101Model::~So101Model() {
  mj_deleteData(d_);
  mj_deleteModel(m_);
}

void So101Model::attach_mesh_mm(const std::string& body, const std::string& stl_path, int stride) {
  const int b = mj_name2id(m_, mjOBJ_BODY, body.c_str());
  if (b < 0) throw std::runtime_error("body not in model: " + body);
  const auto all = load_stl_vertices(stl_path);
  Attach a{b, {}, "wrist_mount"};
  for (size_t i = 0; i < all.size(); i += size_t(stride))
    a.pts.push_back({all[i].x / 1000.0, all[i].y / 1000.0, all[i].z / 1000.0});
  attach_.push_back(std::move(a));
}

void So101Model::set_joints(const Joints& q) {
  for (int j = 0; j < kJoints; ++j) d_->qpos[qadr_[j]] = q[j];
  mj_kinematics(m_, d_);
}

Clearance So101Model::clearance() const {
  Clearance c{std::numeric_limits<double>::infinity(), {}};
  for (int g : arm_geoms_) {
    const int mesh = m_->geom_dataid[g];
    const int v0 = m_->mesh_vertadr[mesh], nv = m_->mesh_vertnum[mesh];
    const mjtNum* R = d_->geom_xmat + 9 * g;  // row-major world-from-geom rotation
    const mjtNum zrow0 = R[6], zrow1 = R[7], zrow2 = R[8];
    const mjtNum tz = d_->geom_xpos[3 * g + 2];
    for (int i = 0; i < nv; ++i) {
      const float* v = m_->mesh_vert + 3 * (v0 + i);
      const double z = zrow0 * v[0] + zrow1 * v[1] + zrow2 * v[2] + tz;
      if (z < c.z) { c.z = z; c.where = mj_id2name(m_, mjOBJ_BODY, m_->geom_bodyid[g]); }
    }
  }
  for (const auto& a : attach_) {
    const mjtNum* R = d_->xmat + 9 * a.body;
    const mjtNum tz = d_->xpos[3 * a.body + 2];
    for (const auto& p : a.pts) {
      const double z = R[6] * p.x + R[7] * p.y + R[8] * p.z + tz;
      if (z < c.z) { c.z = z; c.where = a.name; }
    }
  }
  return c;
}

}  // namespace skillarm
