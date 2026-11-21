#include "api/api_impl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace skillarm {
using nlohmann::json;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxOpening = 0.07;  // m, SO-101 jaws fully open (approx.)

json jp(const Pose& p) { return {{"position", p.position}, {"quaternion_wxyz", p.quaternion_wxyz}, {"frame", p.frame}}; }
json je(const ApiError& e) {
  if (e.code.empty()) return nullptr;
  return {{"code", e.code}, {"message", e.message}, {"recoverable", e.recoverable}};
}
json jx(const ExecutionResult& r) {
  return {{"ok", r.ok}, {"status", r.status}, {"steps_executed", r.steps_executed},
          {"final_position_error_m", r.final_position_error_m}, {"final_joint_error_rad", r.final_joint_error_rad}};
}
double max_abs_diff(const std::vector<double>& a, const std::vector<double>& b) {
  double m = 0;
  for (size_t i = 0; i < std::min(a.size(), b.size()); ++i) m = std::max(m, std::fabs(a[i] - b[i]));
  return m;
}
double dist(const Vec3d& a, const Vec3d& b) {
  return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}
Quat quat_z(double yaw) { return {std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)}; }
void quat_to_mat(const Quat& q, double R[9]) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - z * w);     R[2] = 2 * (x * z + y * w);
  R[3] = 2 * (x * y + z * w);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - x * w);
  R[6] = 2 * (x * z - y * w);     R[7] = 2 * (y * z + x * w);     R[8] = 1 - 2 * (x * x + y * y);
}
// Symmetric 3x3 eigen-decomposition (Jacobi). Columns of V are eigenvectors, eigenvalues in w (descending).
void eig3(double A[3][3], double w[3], double V[3][3]) {
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) V[i][j] = i == j;
  for (int sweep = 0; sweep < 50; ++sweep) {
    double off = std::fabs(A[0][1]) + std::fabs(A[0][2]) + std::fabs(A[1][2]);
    if (off < 1e-15) break;
    for (int p = 0; p < 2; ++p)
      for (int q = p + 1; q < 3; ++q) {
        if (std::fabs(A[p][q]) < 1e-18) continue;
        const double th = 0.5 * std::atan2(2 * A[p][q], A[q][q] - A[p][p]);
        const double c = std::cos(th), s = std::sin(th);
        for (int k = 0; k < 3; ++k) {
          const double akp = A[k][p], akq = A[k][q];
          A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
          const double apk = A[p][k], aqk = A[q][k];
          A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < 3; ++k) {
          const double vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
        }
      }
  }
  int idx[3] = {0, 1, 2};
  std::sort(idx, idx + 3, [&](int a, int b) { return A[a][a] > A[b][b]; });
  double Vs[3][3];
  for (int c = 0; c < 3; ++c) {
    w[c] = A[idx[c]][idx[c]];
    for (int r = 0; r < 3; ++r) Vs[r][c] = V[r][idx[c]];
  }
  std::memcpy(V, Vs, sizeof Vs);
}
}  // namespace

ApiImpl::ApiImpl(EnvironmentAdapter& env, Recorder& rec, Segmenter* seg)
    : env_(env), rec_(rec), seg_(seg), kin_(std::make_unique<Kinematics>(env.planning_model())) {
  check_data_ = mj_makeData(env.planning_model());
  const auto st = env_.robot_state();
  cmd_arm_ = st.joint_positions;
  cmd_grip_ = st.gripper_position;
  down_ = kin_->fk(env_.home()).quaternion_wxyz;
  keyframe_("reset");
}

ApiImpl::~ApiImpl() { mj_deleteData(check_data_); }

void ApiImpl::keyframe_(const std::string& label) {
  last_obs_ = env_.observe();
  rec_.keyframe(label, steps_, last_obs_.cameras);
}

bool ApiImpl::tick_(const std::vector<double>& arm, double grip) {
  if (terminated_ || truncated_) return false;
  const auto t0 = std::chrono::steady_clock::now();
  const auto out = env_.step(arm, grip);
  rec_.time_bucket("adapter_step", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  cmd_arm_ = arm;
  cmd_grip_ = grip;
  terminated_ = out.terminated;
  truncated_ = out.truncated;
  const auto st = env_.robot_state();
  rec_.step({{"step_index", steps_}, {"simulator_time_s", env_.sim_time()},
             {"action", {{"arm", arm}, {"gripper", grip}}},
             {"robot_state", {{"joint_positions", st.joint_positions}, {"gripper_position", st.gripper_position}}},
             {"ok", out.ok}, {"terminated", out.terminated}, {"truncated", out.truncated}});
  ++steps_;
  return !(terminated_ || truncated_);
}

TaskContext ApiImpl::get_task_context() {
  const int sp = rec_.span_start("get_task_context", "api");
  auto c = env_.task_context();
  rec_.span_end(sp, true, json::object(), {{"task_id", c.task_id}, {"language", c.language}});
  return c;
}

Observation ApiImpl::get_observation() {
  const int sp = rec_.span_start("get_observation", "api");
  last_obs_ = env_.observe();
  json cams = json::array();
  for (const auto& c : last_obs_.cameras) cams.push_back({{"name", c.name}, {"width", c.rgb.width}, {"height", c.rgb.height}});
  rec_.span_end(sp, true, json::object(), {{"cameras", cams}});
  return last_obs_;
}

RobotState ApiImpl::get_robot_state() {
  const int sp = rec_.span_start("get_robot_state", "api");
  auto s = env_.robot_state();
  rec_.span_end(sp, true, json::object(),
                {{"joint_positions", s.joint_positions}, {"gripper_position", s.gripper_position},
                 {"end_effector_pose", jp(s.end_effector_pose)}});
  return s;
}

bool ApiImpl::camera_(const std::string& name, CameraObservation& out) {
  if (last_obs_.cameras.empty()) last_obs_ = env_.observe();
  for (const auto& c : last_obs_.cameras)
    if (name.empty() || c.name == name) { out = c; return true; }
  return false;
}

SegmentationSet ApiImpl::segment_text(const std::string& camera_name, const std::string& text) {
  const int sp = rec_.span_start("segment_text", "api");
  SegmentationSet r;
  CameraObservation cam;
  last_obs_ = env_.observe();
  if (!seg_) r.error = {"PROVIDER_UNAVAILABLE", "no segmentation provider is loaded", false};
  else if (!camera_(camera_name, cam)) r.error = {"NOT_FOUND", "camera '" + camera_name + "' not published", false};
  else {
    r = seg_->segment_text(cam, text);
    rec_.overlay(r, cam);
  }
  json items = json::array();
  for (const auto& s : r.items) items.push_back({{"label", s.label}, {"score", s.score}, {"box_xyxy", s.box_xyxy}});
  rec_.span_end(sp, r.ok, {{"camera_name", camera_name}, {"text", text}}, {{"ok", r.ok}, {"items", items}}, je(r.error));
  return r;
}

SegmentationSet ApiImpl::segment_points(const std::string& camera_name, const std::vector<std::array<double, 2>>& pts) {
  const int sp = rec_.span_start("segment_points", "api");
  SegmentationSet r;
  CameraObservation cam;
  last_obs_ = env_.observe();
  if (!seg_) r.error = {"PROVIDER_UNAVAILABLE", "no segmentation provider is loaded", false};
  else if (!camera_(camera_name, cam)) r.error = {"NOT_FOUND", "camera '" + camera_name + "' not published", false};
  else r = seg_->segment_points(cam, pts);
  rec_.span_end(sp, r.ok, {{"camera_name", camera_name}, {"points_px", pts}}, {{"ok", r.ok}, {"count", r.items.size()}},
                je(r.error));
  return r;
}

PointCloud ApiImpl::mask_to_point_cloud(const Segmentation& mask, const std::string& camera_name) {
  const int sp = rec_.span_start("mask_to_point_cloud", "api");
  PointCloud pc;
  CameraObservation cam;
  if (camera_(camera_name.empty() ? mask.camera_name : camera_name, cam) && !cam.depth_m.empty()) {
    const double fx = cam.intrinsics[0], cx = cam.intrinsics[2], fy = cam.intrinsics[4], cy = cam.intrinsics[5];
    double R[9];
    quat_to_mat(cam.camera_pose.quaternion_wxyz, R);
    const auto& t = cam.camera_pose.position;
    for (int v = 0; v < mask.height; ++v)
      for (int u = 0; u < mask.width; ++u) {
        if (!mask.mask[size_t(v * mask.width + u)]) continue;
        const double d = cam.depth_m[size_t(v * cam.rgb.width + u)];
        if (!(d > 0)) continue;
        const double x = (u - cx) * d / fx, y = (v - cy) * d / fy, z = d;
        pc.points.push_back({R[0] * x + R[1] * y + R[2] * z + t[0], R[3] * x + R[4] * y + R[5] * z + t[1],
                             R[6] * x + R[7] * y + R[8] * z + t[2]});
      }
  }
  rec_.span_end(sp, !pc.points.empty(), {{"camera_name", camera_name}, {"mask_pixels",
                std::accumulate(mask.mask.begin(), mask.mask.end(), 0)}}, {{"point_count", pc.points.size()}});
  return pc;
}

PointCloud ApiImpl::crop_point_cloud(const PointCloud& pc, const Vec3d& lo, const Vec3d& hi) {
  const int sp = rec_.span_start("crop_point_cloud", "api");
  PointCloud out;
  out.frame = pc.frame;
  for (const auto& p : pc.points)
    if (p[0] >= lo[0] && p[0] <= hi[0] && p[1] >= lo[1] && p[1] <= hi[1] && p[2] >= lo[2] && p[2] <= hi[2])
      out.points.push_back(p);
  rec_.span_end(sp, true, {{"input_points", pc.points.size()}, {"lower", lo}, {"upper", hi}},
                {{"point_count", out.points.size()}});
  return out;
}

// PCA oriented bounding box, deterministic: eigenvector sign fixed so its dominant component is positive,
// right-handed basis (ASPIRE geometry.py:316-355).
ObjectGeometry ApiImpl::estimate_geometry(const PointCloud& pc) {
  const int sp = rec_.span_start("estimate_geometry", "api");
  ObjectGeometry g;
  g.point_count = int(pc.points.size());
  g.pose.frame = pc.frame;
  if (pc.points.size() >= 3) {
    Vec3d c{0, 0, 0};
    for (const auto& p : pc.points) for (int i = 0; i < 3; ++i) c[size_t(i)] += p[size_t(i)];
    for (auto& x : c) x /= double(pc.points.size());
    double A[3][3] = {};
    for (const auto& p : pc.points)
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) A[i][j] += (p[size_t(i)] - c[size_t(i)]) * (p[size_t(j)] - c[size_t(j)]);
    double w[3], V[3][3];
    eig3(A, w, V);
    for (int k = 0; k < 3; ++k) {
      int dom = 0;
      for (int r = 1; r < 3; ++r) if (std::fabs(V[r][k]) > std::fabs(V[dom][k])) dom = r;
      if (V[dom][k] < 0) for (int r = 0; r < 3; ++r) V[r][k] = -V[r][k];
    }
    // right-handed: third = first x second
    V[0][2] = V[1][0] * V[2][1] - V[2][0] * V[1][1];
    V[1][2] = V[2][0] * V[0][1] - V[0][0] * V[2][1];
    V[2][2] = V[0][0] * V[1][1] - V[1][0] * V[0][1];
    double lo[3] = {1e9, 1e9, 1e9}, hi[3] = {-1e9, -1e9, -1e9};
    for (const auto& p : pc.points)
      for (int k = 0; k < 3; ++k) {
        const double s = (p[0] - c[0]) * V[0][k] + (p[1] - c[1]) * V[1][k] + (p[2] - c[2]) * V[2][k];
        lo[k] = std::min(lo[k], s); hi[k] = std::max(hi[k], s);
      }
    for (int k = 0; k < 3; ++k) {
      g.extents[size_t(k)] = hi[k] - lo[k];
      const double mid = (hi[k] + lo[k]) / 2;
      for (int r = 0; r < 3; ++r) c[size_t(r)] += V[r][k] * mid;
    }
    g.pose.position = c;
    double R[9] = {V[0][0], V[0][1], V[0][2], V[1][0], V[1][1], V[1][2], V[2][0], V[2][1], V[2][2]};
    mjtNum q[4];
    mju_mat2Quat(q, R);
    g.pose.quaternion_wxyz = {q[0], q[1], q[2], q[3]};
  }
  rec_.span_end(sp, g.point_count >= 3, {{"point_count", g.point_count}},
                {{"pose", jp(g.pose)}, {"extents", g.extents}});
  return g;
}

// Geometric top-down grasps for the SO-101 jaws: approach straight down through the box centre, jaws closing across
// the narrower horizontal side. Contact-GraspNet is trained for the Panda hand and does not transfer.
GraspSet ApiImpl::generate_grasps(const std::string& camera_name, const Segmentation& mask, int max_candidates) {
  const int sp = rec_.span_start("generate_grasps", "api");
  GraspSet gs;
  const auto pc = mask_to_point_cloud(mask, camera_name);
  if (pc.points.size() < 10) {
    gs.error = {"NOT_FOUND", "too few depth points under the mask", true};
  } else {
    const auto g = estimate_geometry(pc);
    double R[9];
    quat_to_mat(g.pose.quaternion_wxyz, R);
    // horizontal box axes (least vertical two)
    std::vector<std::pair<double, int>> axes;
    for (int k = 0; k < 3; ++k) axes.push_back({std::fabs(R[6 + k]), k});
    std::sort(axes.begin(), axes.end());
    // SO-101 jaws close along the tool frame's +z axis and the tool centre point lies on the fixed jaw's inner face
    // (measured from the meshes, see `skillarm jaw-geom`). So: rotate the downward home orientation about z until +z is
    // perpendicular to the chosen box side, then shift the tool centre back by half the object width.
    double Rh[9];
    quat_to_mat(down_, Rh);
    const double a0 = std::atan2(Rh[5], Rh[2]);   // closing axis angle at yaw 0 (third column, xy)
    const double radial = std::atan2(g.pose.position[1], g.pose.position[0]);
    for (int pick = 0; pick < 2; ++pick) {
      const int k = axes[size_t(pick)].second;     // close across this box axis
      const double width = g.extents[size_t(k)];
      if (width > kMaxOpening - 0.005) continue;
      const double ax = std::atan2(R[3 + k], R[0 + k]);
      // of the two opposite closing directions keep the one whose yaw is nearest the radial direction (reachability)
      double yaw = 0, best = 1e9;
      for (double flip : {0.0, kPi}) {
        const double cand = ax + flip - a0;
        const double d = std::fabs(std::remainder(cand + a0 - radial, 2 * kPi));
        if (d < best) { best = d; yaw = cand; }
      }
      GraspCandidate c;
      c.pose.quaternion_wxyz = quat_mul(quat_z(yaw), down_);
      double Rc[9];
      quat_to_mat(c.pose.quaternion_wxyz, Rc);
      const double half = width / 2 + 0.002;
      c.pose.position = {g.pose.position[0] - Rc[2] * half, g.pose.position[1] - Rc[5] * half,
                         std::max(g.pose.position[2] - g.extents[2] * 0.25, 0.008)};
      c.width_m = width;
      c.approach_distance_m = 0.04;  // top-down is reachable only up to ~6 cm above the desk
      c.score = (1.0 - width / kMaxOpening) * (pick == 0 ? 1.0 : 0.8);
      gs.grasps.push_back(c);
    }
    std::sort(gs.grasps.begin(), gs.grasps.end(), [](auto& a, auto& b) { return a.score > b.score; });
    if (int(gs.grasps.size()) > max_candidates) gs.grasps.resize(size_t(max_candidates));
    gs.ok = !gs.grasps.empty();
    if (!gs.ok) gs.error = {"NOT_FOUND", "object wider than the jaw opening", true};
  }
  json gj = json::array();
  for (const auto& c : gs.grasps) gj.push_back({{"pose", jp(c.pose)}, {"score", c.score}, {"width_m", c.width_m}});
  rec_.span_end(sp, gs.ok, {{"camera_name", camera_name}}, {{"grasps", gj}}, je(gs.error));
  return gs;
}

IKResult ApiImpl::solve_ik(const Pose& target, const std::vector<double>& seed) {
  const int sp = rec_.span_start("solve_ik", "api");
  const auto r = kin_->ik(target, seed.empty() ? env_.robot_state().joint_positions : seed);
  rec_.span_end(sp, r.ok, {{"target", jp(target)}},
                {{"ok", r.ok}, {"joint_positions", r.joint_positions}, {"position_error_m", r.position_error_m},
                 {"orientation_error_rad", r.orientation_error_rad}}, je(r.error));
  return r;
}

std::string ApiImpl::desk_check_(const Trajectory& t) {
  const mjModel* m = env_.planning_model();
  const char* names[5] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};
  const int base = mj_name2id(m, mjOBJ_BODY, "base");
  const int grip = mj_name2id(m, mjOBJ_BODY, "gripper");
  const int jaw = mj_name2id(m, mjOBJ_BODY, "moving_jaw_so101_v1");
  mj_resetData(m, check_data_);
  for (size_t w = 0; w < t.joint_positions.size(); w += std::max<size_t>(1, t.joint_positions.size() / 40)) {
    for (int j = 0; j < 5; ++j)
      check_data_->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, names[j])]] = t.joint_positions[w][size_t(j)];
    mj_kinematics(m, check_data_);
    for (int g = 0; g < m->ngeom; ++g) {
      const int b = m->geom_bodyid[g];
      if (m->geom_type[g] != mjGEOM_MESH || m->body_rootid[b] != m->body_rootid[base] || b == base) continue;
      const bool tool = (b == grip || b == jaw);
      const int mesh = m->geom_dataid[g];
      const mjtNum* R = check_data_->geom_xmat + 9 * g;
      double zmin = 1e9;
      for (int i = 0; i < m->mesh_vertnum[mesh]; ++i) {
        const float* v = m->mesh_vert + 3 * (m->mesh_vertadr[mesh] + i);
        zmin = std::min(zmin, R[6] * v[0] + R[7] * v[1] + R[8] * v[2] + check_data_->geom_xpos[3 * g + 2]);
      }
      if (zmin < (tool ? -0.002 : 0.005))
        return std::string(mj_id2name(m, mjOBJ_BODY, b)) + " would go " + std::to_string(int(zmin * 1000)) +
               " mm into the desk at waypoint " + std::to_string(w);
    }
  }
  return "";
}

PlanResult ApiImpl::plan_joints_(const std::vector<double>& goal, const MotionStrategy& s) {
  PlanResult r;
  if (goal.size() != kArmDof) { r.error = {"INVALID_ARGUMENT", "expected 5 arm joint values", false}; return r; }
  if (s.trajectory_planner != "interpolation") {
    r.error = {"UNSUPPORTED", "trajectory_planner '" + s.trajectory_planner + "' is not available yet", false};
    return r;
  }
  std::vector<double> g = goal;
  for (int j = 0; j < kArmDof; ++j) g[size_t(j)] = std::clamp(g[size_t(j)], kin_->lower()[size_t(j)], kin_->upper()[size_t(j)]);
  const auto start = cmd_arm_;
  const double step = std::clamp(s.max_joint_step_rad, 0.005, 0.2);
  const int n = std::max(1, int(std::ceil(max_abs_diff(start, g) / step)));
  for (int i = 1; i <= n; ++i) {
    std::vector<double> q(kArmDof);
    for (int j = 0; j < kArmDof; ++j) q[size_t(j)] = start[size_t(j)] + (g[size_t(j)] - start[size_t(j)]) * i / n;
    r.trajectory.joint_positions.push_back(q);
  }
  r.trajectory.dt_s = env_.control_dt();
  const auto clash = desk_check_(r.trajectory);
  if (!clash.empty()) { r.error = {"PLAN_COLLISION", clash, true}; return r; }
  r.ok = true;
  return r;
}

PlanResult ApiImpl::plan_motion_joints(const std::vector<double>& goal, const MotionStrategy& s) {
  const int sp = rec_.span_start("plan_motion", "api");
  auto r = plan_joints_(goal, s);
  rec_.span_end(sp, r.ok, {{"goal_joints", goal}, {"planner", s.trajectory_planner}},
                {{"ok", r.ok}, {"waypoints", r.trajectory.joint_positions.size()}}, je(r.error));
  return r;
}

PlanResult ApiImpl::plan_motion(const Pose& goal, const MotionStrategy& s) {
  const int sp = rec_.span_start("plan_motion", "api");
  PlanResult r;
  const auto ik = kin_->ik(goal, cmd_arm_);
  if (!ik.ok) r.error = ik.error;
  else r = plan_joints_(ik.joint_positions, s);
  rec_.span_end(sp, r.ok, {{"goal", jp(goal)}, {"planner", s.trajectory_planner}},
                {{"ok", r.ok}, {"waypoints", r.trajectory.joint_positions.size()}}, je(r.error));
  return r;
}

ExecutionResult ApiImpl::exec_(const Trajectory& t) {
  ExecutionResult r;
  if (t.joint_positions.empty()) { r.ok = true; r.status = "completed"; return r; }
  // stale trajectory: must start from where the arm is commanded now (ASPIRE: 0.02 rad)
  if (t.joint_positions.size() > 1 && max_abs_diff(t.joint_positions.front(), cmd_arm_) > 0.06) {
    r.status = "failed";
    r.error = {"STALE_TRAJECTORY", "trajectory does not start at the current arm configuration; re-plan", true};
    return r;
  }
  for (size_t i = 0; i < t.joint_positions.size(); ++i) {
    const double g = t.gripper_positions.size() == t.joint_positions.size() ? t.gripper_positions[i] : cmd_grip_;
    if (!tick_(t.joint_positions[i], g)) { r.status = terminated_ ? "terminated" : "step_limit"; break; }
    ++r.steps_executed;
  }
  // settle
  const auto& goal = t.joint_positions.back();
  for (int k = 0; k < 20 && r.status.empty(); ++k) {
    if (max_abs_diff(env_.robot_state().joint_positions, goal) < 0.01) break;
    if (!tick_(goal, cmd_grip_)) { r.status = terminated_ ? "terminated" : "step_limit"; break; }
    ++r.steps_executed;
  }
  r.final_joint_error_rad = max_abs_diff(env_.robot_state().joint_positions, goal);
  if (r.status.empty()) {
    r.ok = r.final_joint_error_rad < 0.05;
    r.status = r.ok ? "completed" : "blocked";
    if (!r.ok) r.error = {"BLOCKED", "arm stopped " + std::to_string(r.final_joint_error_rad) + " rad short of the goal (contact?)", true};
  } else {
    r.error = {"EPISODE_ENDED", "episode ended during execution", false};
  }
  return r;
}

ExecutionResult ApiImpl::execute_trajectory(const Trajectory& t) {
  const MotionFrames frames(*this, "execute_trajectory");  // ASPIRE: keyframes before and after every motion tool
  const int sp = rec_.span_start("execute_trajectory", "api");
  auto r = exec_(t);
  rec_.span_end(sp, r.ok, {{"waypoints", t.joint_positions.size()}}, jx(r), je(r.error));
  return r;
}

ExecutionResult ApiImpl::gripper_(double pos, bool contact_aware) {
  ExecutionResult r;
  pos = std::clamp(pos, 0.0, 1.0);
  double prev = env_.robot_state().gripper_position;
  int still = 0;
  for (int k = 0; k < 45; ++k) {
    if (!tick_(cmd_arm_, pos)) { r.status = "step_limit"; r.error = {"EPISODE_ENDED", "episode ended", false}; return r; }
    ++r.steps_executed;
    const double g = env_.robot_state().gripper_position;
    if (std::fabs(g - pos) < 0.02) break;
    still = std::fabs(g - prev) < 0.002 ? still + 1 : 0;
    prev = g;
    if (contact_aware && still >= 3) break;  // jaws stopped on an object
  }
  r.ok = true;
  r.status = "completed";
  return r;
}

ExecutionResult ApiImpl::set_gripper(double position) {
  const MotionFrames frames(*this, "set_gripper");  // ASPIRE: keyframes before and after every motion tool
  const int sp = rec_.span_start("set_gripper", "api");
  auto r = gripper_(position, false);
  rec_.span_end(sp, r.ok, {{"position", position}}, jx(r), je(r.error));
  return r;
}

LocalizationResult ApiImpl::localize_object(const std::string& query, const std::string& camera_name) {
  const int sp = rec_.span_start("localize_object", "api");
  LocalizationResult r;
  auto segs = segment_text(camera_name, query);
  if (!segs.ok || segs.items.empty()) {
    r.error = segs.error.code.empty() ? ApiError{"NOT_FOUND", "no segmentation for '" + query + "'", true} : segs.error;
  } else {
    r.segmentation = segs.items.front();
    r.point_cloud = mask_to_point_cloud(r.segmentation, r.segmentation.camera_name);
    if (r.point_cloud.points.size() < 10) r.error = {"NOT_FOUND", "mask has too few depth points", true};
    else { r.geometry = estimate_geometry(r.point_cloud); r.ok = true; }
  }
  rec_.span_end(sp, r.ok, {{"query", query}, {"camera_name", camera_name}},
                {{"ok", r.ok}, {"pose", jp(r.geometry.pose)}, {"extents", r.geometry.extents},
                 {"point_count", r.geometry.point_count}}, je(r.error));
  return r;
}

const GraspCandidate* ApiImpl::select_grasp(const GraspSet& grasps, const std::string& strategy) {
  const int sp = rec_.span_start("select_grasp", "api");
  const GraspCandidate* best = nullptr;
  for (const auto& g : grasps.grasps) {
    if (strategy == "top_down") {
      // approach = tool axis that points down at home; check it is within ~37 deg of straight down
      const double ang = quat_angle(quat_mul(quat_conj(g.pose.quaternion_wxyz), down_), {1, 0, 0, 0});
      (void)ang;
      double R[9], Rh[9];
      quat_to_mat(g.pose.quaternion_wxyz, R);
      quat_to_mat(down_, Rh);
      // find home's downward column and compare the same column of the grasp
      int col = 0;
      for (int c = 1; c < 3; ++c) if (Rh[6 + c] < Rh[6 + col]) col = c;
      if (-R[6 + col] < 0.8) continue;
    }
    if (!best || g.score > best->score) best = &g;
  }
  rec_.span_end(sp, best != nullptr, {{"strategy", strategy}, {"candidates", grasps.grasps.size()}},
                {{"selected", best ? jp(best->pose) : json(nullptr)}});
  return best;
}

ExecutionResult ApiImpl::move_to_joints(const std::vector<double>& target, double tolerance, int max_steps) {
  const MotionFrames frames(*this, "move_to_joints");  // ASPIRE: keyframes before and after every motion tool
  const int sp = rec_.span_start("move_to_joints", "api");
  ExecutionResult r;
  auto plan = plan_joints_(target, {});
  if (!plan.ok) { r.error = plan.error; r.status = "failed"; }
  else {
    if (int(plan.trajectory.joint_positions.size()) > max_steps) plan.trajectory.joint_positions.resize(size_t(max_steps));
    r = exec_(plan.trajectory);
    r.final_joint_error_rad = max_abs_diff(env_.robot_state().joint_positions, target);
    if (r.ok && r.final_joint_error_rad > tolerance) {
      r.ok = false;
      r.status = "tolerance_not_met";
      r.error = {"TOLERANCE_NOT_MET", "joint error " + std::to_string(r.final_joint_error_rad) + " rad", true};
    }
  }
  rec_.span_end(sp, r.ok, {{"target", target}, {"tolerance", tolerance}, {"max_steps", max_steps}}, jx(r), je(r.error));
  return r;
}

ExecutionResult ApiImpl::move_to_pose(const Pose& target, double tolerance, int max_steps, const MotionStrategy& s) {
  const MotionFrames frames(*this, "move_to_pose");  // ASPIRE: keyframes before and after every motion tool
  const int sp = rec_.span_start("move_to_pose", "api");
  ExecutionResult r;
  const auto ik = kin_->ik(target, cmd_arm_);
  if (!ik.ok) { r.error = ik.error; r.status = "failed"; }
  else {
    auto plan = plan_joints_(ik.joint_positions, s);
    if (!plan.ok) { r.error = plan.error; r.status = "failed"; }
    else {
      if (int(plan.trajectory.joint_positions.size()) > max_steps) plan.trajectory.joint_positions.resize(size_t(max_steps));
      r = exec_(plan.trajectory);
      // arrival check: position within tolerance, orientation within 10 deg (unless position-only)
      const auto st = env_.robot_state();
      r.final_position_error_m = dist(st.end_effector_pose.position, target.position);
      const bool pos_only = target.quaternion_wxyz == Quat{0, 0, 0, 0};
      const double ori = pos_only ? 0 : quat_angle(st.end_effector_pose.quaternion_wxyz, target.quaternion_wxyz);
      if (r.ok && (r.final_position_error_m > tolerance || ori > 10 * kPi / 180)) {
        r.ok = false;
        r.status = "tolerance_not_met";
        r.error = {"TOLERANCE_NOT_MET", "arrived " + std::to_string(r.final_position_error_m * 1000).substr(0, 5) +
                   " mm / " + std::to_string(ori * 180 / kPi).substr(0, 5) + " deg from the target", true};
      }
    }
  }
  rec_.span_end(sp, r.ok, {{"target", jp(target)}, {"tolerance", tolerance}, {"max_steps", max_steps}}, jx(r), je(r.error));
  return r;
}

ExecutionResult ApiImpl::open_gripper() {
  const MotionFrames frames(*this, "open_gripper");  // ASPIRE: keyframes before and after every motion tool
  const int sp = rec_.span_start("open_gripper", "api");
  auto r = gripper_(1.0, false);
  rec_.span_end(sp, r.ok, json::object(), jx(r), je(r.error));
  return r;
}

ExecutionResult ApiImpl::close_gripper() {
  const MotionFrames frames(*this, "close_gripper");  // ASPIRE: keyframes before and after every motion tool
  const int sp = rec_.span_start("close_gripper", "api");
  auto r = gripper_(0.0, true);
  rec_.span_end(sp, r.ok, json::object(), {{"ok", r.ok}, {"gripper_position", env_.robot_state().gripper_position}}, je(r.error));
  return r;
}

ExecutionResult ApiImpl::go_home() {
  const MotionFrames frames(*this, "go_home");  // ASPIRE: keyframes before and after every motion tool
  const int sp = rec_.span_start("go_home", "api");
  ExecutionResult r;
  for (int attempt = 0; attempt < 4; ++attempt) {
    auto plan = plan_joints_(env_.home(), {});
    if (!plan.ok) { r.error = plan.error; r.status = "failed"; break; }
    r = exec_(plan.trajectory);
    if (r.ok) break;
  }
  rec_.span_end(sp, r.ok, json::object(), jx(r), je(r.error));
  return r;
}

void ApiImpl::report(const std::string& key, double value) { result_[key] = value; }
void ApiImpl::report(const std::string& key, const std::string& value) { result_[key] = value; }
void ApiImpl::log(const std::string& line) { rec_.stdout_line(line); }

}  // namespace skillarm
