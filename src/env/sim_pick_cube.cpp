#include "env/sim_pick_cube.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace skillarm {
namespace {
const char* kNames[6] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
constexpr double kPi = 3.14159265358979323846;
}  // namespace

SimPickCube::SimPickCube(const std::string& task_xml, int max_steps) : max_steps_(max_steps) {
  char err[1024] = {};
  m_ = mj_loadXML(task_xml.c_str(), nullptr, err, sizeof err);
  if (!m_) throw std::runtime_error(std::string("mj_loadXML: ") + err);
  d_ = mj_makeData(m_);
  for (int j = 0; j < 6; ++j) {
    const int id = mj_name2id(m_, mjOBJ_JOINT, kNames[j]);
    const int a = mj_name2id(m_, mjOBJ_ACTUATOR, kNames[j]);
    if (id < 0 || a < 0) throw std::runtime_error(std::string("missing joint/actuator ") + kNames[j]);
    qadr_[j] = m_->jnt_qposadr[id];
    dadr_[j] = m_->jnt_dofadr[id];
    act_[j] = a;
  }
  grip_lo_ = m_->actuator_ctrlrange[2 * act_[5]];
  grip_hi_ = m_->actuator_ctrlrange[2 * act_[5] + 1];
  cube_body_ = mj_name2id(m_, mjOBJ_BODY, "cube");
  cube_q_ = m_->jnt_qposadr[mj_name2id(m_, mjOBJ_JOINT, "cube")];
  cube_z0_ = m_->geom_size[3 * mj_name2id(m_, mjOBJ_GEOM, "cube") + 2];

  // Home: upper arm vertical, forearm horizontal, wrist bent so the gripper points down (pick the sign that
  // lowers the tool), jaws open. Chosen from the model, not hand-tuned numbers.
  const int site = mj_name2id(m_, mjOBJ_SITE, "gripperframe");
  double best_z = 1e9, best_w = 0;
  for (double w : {kPi / 2, -kPi / 2}) {
    mj_resetData(m_, d_);
    d_->qpos[qadr_[3]] = w;
    mj_kinematics(m_, d_);
    if (d_->site_xpos[3 * site + 2] < best_z) { best_z = d_->site_xpos[3 * site + 2]; best_w = w; }
  }
  home_ = {0.0, 0.0, 0.0, best_w, 0.0};
  reset(0);
}

SimPickCube::~SimPickCube() {
  mj_deleteData(d_);
  mj_deleteModel(m_);
}

TaskContext SimPickCube::reset(int seed) {
  mj_resetData(m_, d_);
  for (int j = 0; j < 5; ++j) d_->qpos[qadr_[j]] = home_[size_t(j)];
  d_->qpos[qadr_[5]] = grip_hi_;
  for (int j = 0; j < 5; ++j) d_->ctrl[act_[j]] = home_[size_t(j)];
  d_->ctrl[act_[5]] = grip_hi_;

  // Cube in front of the arm: "front" = direction of the tool at the zero pose.
  mj_kinematics(m_, d_);
  const int site = mj_name2id(m_, mjOBJ_SITE, "gripperframe");
  const double fx = d_->site_xpos[3 * site], fy = d_->site_xpos[3 * site + 1];
  const double front = std::atan2(fy, fx);
  std::mt19937 rng(unsigned(seed) * 2654435761u + 12345u);
  const double r = std::uniform_real_distribution<double>(0.16, 0.25)(rng);
  const double a = front + std::uniform_real_distribution<double>(-0.6, 0.6)(rng);
  const double yaw = std::uniform_real_distribution<double>(-kPi / 4, kPi / 4)(rng);
  d_->qpos[cube_q_ + 0] = r * std::cos(a);
  d_->qpos[cube_q_ + 1] = r * std::sin(a);
  d_->qpos[cube_q_ + 2] = cube_z0_;
  d_->qpos[cube_q_ + 3] = std::cos(yaw / 2);
  d_->qpos[cube_q_ + 4] = 0;
  d_->qpos[cube_q_ + 5] = 0;
  d_->qpos[cube_q_ + 6] = std::sin(yaw / 2);
  mj_forward(m_, d_);
  for (int i = 0; i < 100; ++i) mj_step(m_, d_);  // settle

  ctx_ = {};
  ctx_.task_id = "pick_cube";
  ctx_.task_name = "pick up the red cube";
  ctx_.language = "Pick up the red cube from the desk and lift it at least 5 cm.";
  ctx_.episode_max_steps = max_steps_;
  ctx_.episode_steps_used = 0;
  return ctx_;
}

RobotState SimPickCube::robot_state() {
  RobotState s;
  for (int j = 0; j < 5; ++j) {
    s.joint_positions.push_back(d_->qpos[qadr_[j]]);
    s.joint_velocities.push_back(d_->qvel[dadr_[j]]);
    s.joint_names.push_back(kNames[j]);
  }
  s.gripper_position = std::clamp((d_->qpos[qadr_[5]] - grip_lo_) / (grip_hi_ - grip_lo_), 0.0, 1.0);
  const int site = mj_name2id(m_, mjOBJ_SITE, "gripperframe");
  for (int i = 0; i < 3; ++i) s.end_effector_pose.position[size_t(i)] = d_->site_xpos[3 * site + i];
  mjtNum q[4];
  mju_mat2Quat(q, d_->site_xmat + 9 * site);
  s.end_effector_pose.quaternion_wxyz = {q[0], q[1], q[2], q[3]};
  s.timestamp_s = d_->time;
  return s;
}

Observation SimPickCube::observe() {
  Observation o;
  o.robot_state = robot_state();
  o.timestamp_s = d_->time;
  // "scene" camera = where the C270 sits on the real desk; 640x480 like the real stream
  if (!camera_ && !camera_failed_) {
    try {
      camera_ = std::make_unique<SimCamera>(m_, "scene", 640, 480);
    } catch (const std::exception& e) {
      camera_failed_ = true;
      std::fprintf(stderr, "sim camera unavailable: %s\n", e.what());
    }
  }
  if (camera_) o.cameras.push_back(camera_->capture(m_, d_));
  return o;
}

StepOutcome SimPickCube::step(const std::vector<double>& arm_target, double gripper) {
  for (int j = 0; j < 5; ++j) {
    const double lo = m_->actuator_ctrlrange[2 * act_[j]], hi = m_->actuator_ctrlrange[2 * act_[j] + 1];
    d_->ctrl[act_[j]] = std::clamp(arm_target[size_t(j)], lo, hi);
  }
  d_->ctrl[act_[5]] = grip_lo_ + std::clamp(gripper, 0.0, 1.0) * (grip_hi_ - grip_lo_);
  const int sub = std::max(1, int(std::lround(control_dt() / m_->opt.timestep)));
  for (int i = 0; i < sub; ++i) mj_step(m_, d_);
  ++ctx_.episode_steps_used;
  StepOutcome out;
  out.truncated = ctx_.episode_steps_used >= max_steps_;
  return out;
}

Vec3d SimPickCube::cube_position() const {
  return {d_->xpos[3 * cube_body_], d_->xpos[3 * cube_body_ + 1], d_->xpos[3 * cube_body_ + 2]};
}

void SimPickCube::debug_contacts() const {
  auto bname = [&](int g) { return mj_id2name(m_, mjOBJ_BODY, m_->geom_bodyid[g]); };
  for (int i = 0; i < d_->ncon; ++i) {
    const auto& c = d_->contact[i];
    std::printf("   contact %s <-> %s depth %.4f at (%.3f %.3f %.3f)\n", bname(c.geom[0]), bname(c.geom[1]), -c.dist,
                c.pos[0], c.pos[1], c.pos[2]);
  }
  const int site = mj_name2id(m_, mjOBJ_SITE, "gripperframe");
  const int jaw = mj_name2id(m_, mjOBJ_BODY, "moving_jaw_so101_v1");
  const auto c = cube_position();
  std::printf("   tcp (%.3f %.3f %.3f) jaw body (%.3f %.3f %.3f) cube (%.3f %.3f %.3f) gripper q %.3f\n",
              d_->site_xpos[3 * site], d_->site_xpos[3 * site + 1], d_->site_xpos[3 * site + 2], d_->xpos[3 * jaw],
              d_->xpos[3 * jaw + 1], d_->xpos[3 * jaw + 2], c[0], c[1], c[2], d_->qpos[qadr_[5]]);
}

double SimPickCube::cube_yaw() const {
  const mjtNum* q = d_->xquat + 4 * cube_body_;
  return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

bool SimPickCube::task_success() { return cube_position()[2] > cube_z0_ + 0.05; }

}  // namespace skillarm
