#include "real/real_so101.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace skillarm {
using nlohmann::json;
using clock_t_ = std::chrono::steady_clock;

namespace {
constexpr double kStepRad = 2.0 * 3.14159265358979323846 / 4096.0;
}

RealSo101::RealSo101(const std::string& host, int port, const std::string& calibration_json,
                     const std::string& scene_xml, RealTask task, std::function<bool()> judge, RealCamera camera)
    : arm_(host, port),
      depth_(camera.depth),
      cal_(Calibration::load(calibration_json)),
      task_(std::move(task)),
      judge_(std::move(judge)) {
  char err[1024] = {};
  m_ = mj_loadXML(scene_xml.c_str(), nullptr, err, sizeof err);
  if (!m_) throw std::runtime_error(std::string("mj_loadXML: ") + err);
  d_ = mj_makeData(m_);
  for (int j = 0; j < kJoints; ++j) {
    const int id = mj_name2id(m_, mjOBJ_JOINT, kJointNames[j]);
    if (id < 0) throw std::runtime_error(std::string("missing joint ") + kJointNames[j]);
    qadr_[j] = m_->jnt_qposadr[id];
  }
  const int ga = mj_name2id(m_, mjOBJ_ACTUATOR, "gripper");
  grip_lo_ = m_->actuator_ctrlrange[2 * ga];
  grip_hi_ = m_->actuator_ctrlrange[2 * ga + 1];
  site_ = mj_name2id(m_, mjOBJ_SITE, "gripperframe");

  // Same home rule as the simulator: zero pose with the wrist bent whichever way lowers the tool.
  double best_z = 1e9, best_w = 0;
  for (double w : {1.5707963267948966, -1.5707963267948966}) {
    mj_resetData(m_, d_);
    d_->qpos[qadr_[3]] = w;
    mj_kinematics(m_, d_);
    if (d_->site_xpos[3 * site_ + 2] < best_z) { best_z = d_->site_xpos[3 * site_ + 2]; best_w = w; }
  }
  home_ = {0.0, 0.0, 0.0, best_w, 0.0};

  const json info = arm_.call({{"cmd", "info"}});
  for (const auto& j : info.at("joints"))
    if (!j.value("ok", false)) throw std::runtime_error("servo " + std::to_string(j.value("id", 0)) + " not responding");
  refresh();

  if (!camera.host.empty()) {
    camera_ = std::make_unique<CameraClient>(camera.host, camera.port);
    for (int g = 0; g < m_->ngeom; ++g)  // the self view shows the robot only (geom colour; collisions unchanged)
      if (m_->geom_bodyid[g] == 0) m_->geom_rgba[4 * g + 3] = 0;
    self_view_ = std::make_unique<SimCamera>(m_, "wrist", 640, 480);
  }
}

RealSo101::~RealSo101() {
  try { arm_.call({{"cmd", "stop"}}); } catch (...) {}
  mj_deleteData(d_);
  mj_deleteModel(m_);
}

void RealSo101::refresh() {
  const json s = arm_.call({{"cmd", "state"}});
  const auto& pos = s.at("pos");
  const auto& spd = s.at("speed");
  for (int j = 0; j < kJoints; ++j) {
    const int p = pos.at(size_t(j)).get<int>();
    if (p < 0) throw std::runtime_error(std::string("joint ") + kJointNames[j] + " lost");
    q_[size_t(j)] = cal_.to_rad(j, p);
    qd_[size_t(j)] = spd.at(size_t(j)).get<int>() * kStepRad;
  }
  const std::string f = s.value("fault", "");
  if (!f.empty()) fault_ = f;
}

TaskContext RealSo101::reset(int seed) {
  (void)seed;  // the operator sets the real scene; the seed only labels the trial
  fault_.clear();
  judged_ = false;
  refresh();
  if (!fault_.empty()) throw std::runtime_error("servo fault latched: " + fault_ + " (check the arm, then clear)");
  t0_ = next_tick_ = clock_t_::now();
  ctx_ = {};
  ctx_.task_id = task_.id;
  ctx_.task_name = task_.name;
  ctx_.language = task_.language;
  ctx_.episode_max_steps = task_.max_steps;
  ctx_.episode_steps_used = 0;
  return ctx_;
}

RobotState RealSo101::robot_state() {
  RobotState s;
  for (int j = 0; j < 5; ++j) {
    s.joint_positions.push_back(q_[size_t(j)]);
    s.joint_velocities.push_back(qd_[size_t(j)]);
    s.joint_names.push_back(kJointNames[j]);
  }
  s.gripper_position = std::clamp((q_[5] - grip_lo_) / (grip_hi_ - grip_lo_), 0.0, 1.0);
  for (int j = 0; j < kJoints; ++j) d_->qpos[qadr_[j]] = q_[size_t(j)];
  mj_kinematics(m_, d_);
  for (int i = 0; i < 3; ++i) s.end_effector_pose.position[size_t(i)] = d_->site_xpos[3 * site_ + i];
  mjtNum qt[4];
  mju_mat2Quat(qt, d_->site_xmat + 9 * site_);
  s.end_effector_pose.quaternion_wxyz = {qt[0], qt[1], qt[2], qt[3]};
  s.timestamp_s = sim_time();
  return s;
}

Observation RealSo101::observe() {
  Observation o;
  if (camera_) refresh();  // the frame and the joint angles should belong together
  o.robot_state = robot_state();  // also leaves the kinematics of the current angles in d_
  o.timestamp_s = sim_time();
  if (camera_) {
    mj_camlight(m_, d_);
    const CameraObservation self = self_view_->capture(m_, d_);  // pose, intrinsics and the robot's own pixels
    CameraObservation c;
    camera_->grab(c);
    if (c.rgb.width == self.rgb.width && c.rgb.height == self.rgb.height) {
      c.intrinsics = self.intrinsics;
      c.camera_pose = self.camera_pose;
      if (depth_ && depth_->estimate(c))
        for (size_t i = 0; i < c.depth_m.size(); ++i)
          if (self.depth_m[i] > 0) c.depth_m[i] = 0;  // the gripper itself: too near for the depth network
    }
    c.timestamp_s = o.timestamp_s;
    o.cameras.push_back(std::move(c));
  }
  return o;
}

StepOutcome RealSo101::step(const std::vector<double>& arm_target, double gripper) {
  StepOutcome out;
  if (!fault_.empty()) { out.ok = false; out.terminated = true; return out; }
  json targets = json::object();
  for (int j = 0; j < 5; ++j) {
    const double lo = m_->jnt_range[2 * mj_name2id(m_, mjOBJ_JOINT, kJointNames[j])];
    const double hi = m_->jnt_range[2 * mj_name2id(m_, mjOBJ_JOINT, kJointNames[j]) + 1];
    targets[std::to_string(cal_.joint[j].id)] = cal_.to_pos(j, std::clamp(arm_target[size_t(j)], lo, hi));
  }
  const double grip_rad = grip_lo_ + std::clamp(gripper, 0.0, 1.0) * (grip_hi_ - grip_lo_);
  targets[std::to_string(cal_.joint[5].id)] = cal_.to_pos(5, grip_rad);
  try {
    arm_.call({{"cmd", "servo"}, {"targets", targets}});
  } catch (const std::exception& e) {  // refused (fault latched, joint lost) or connection gone
    fault_ = e.what();
    out.ok = false;
    out.terminated = true;
    return out;
  }
  // hold the 30 Hz cadence against wall-clock time (no drift accumulation)
  next_tick_ += std::chrono::duration_cast<clock_t_::duration>(std::chrono::duration<double>(control_dt()));
  const auto now = clock_t_::now();
  if (next_tick_ > now) std::this_thread::sleep_until(next_tick_);
  else next_tick_ = now;
  refresh();
  ++ctx_.episode_steps_used;
  if (!fault_.empty()) { out.ok = false; out.terminated = true; }
  out.truncated = ctx_.episode_steps_used >= task_.max_steps;
  return out;
}

bool RealSo101::task_success() {
  if (!judged_) {
    arm_.call({{"cmd", "stop"}});  // hold before the operator looks
    success_ = judge_ ? judge_() : false;
    judged_ = true;
  }
  return success_;
}

double RealSo101::sim_time() const { return std::chrono::duration<double>(clock_t_::now() - t0_).count(); }

}  // namespace skillarm
