// Simulated SO-101 pick-up task in MuJoCo (models/so101/task_pick_cube.xml).
#pragma once
#include <memory>
#include <random>

#include "env/adapter.hpp"
#include "sim/sim_camera.hpp"

namespace skillarm {

class SimPickCube final : public EnvironmentAdapter {
 public:
  explicit SimPickCube(const std::string& task_xml, int max_steps = 1500);
  ~SimPickCube() override;

  std::string embodiment() const override { return "so101_sim"; }
  TaskContext reset(int seed) override;
  TaskContext task_context() const override { return ctx_; }
  RobotState robot_state() override;
  Observation observe() override;
  StepOutcome step(const std::vector<double>& arm_target, double gripper) override;
  double control_dt() const override { return 1.0 / 30.0; }
  std::vector<double> home() const override { return home_; }
  const mjModel* planning_model() const override { return m_; }
  bool task_success() override;
  double sim_time() const override { return d_->time; }

  // Host-only access for tests / ground-truth evaluation.
  Vec3d cube_position() const;
  double cube_yaw() const;
  void debug_contacts() const;  // host-only: print current contacts and jaw positions

 private:
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int qadr_[6] = {}, dadr_[6] = {}, act_[6] = {};
  int cube_q_ = 0, cube_body_ = -1;
  double grip_lo_ = 0, grip_hi_ = 0;
  std::vector<double> home_;
  TaskContext ctx_;
  int max_steps_;
  double cube_z0_ = 0.015;
  std::unique_ptr<SimCamera> camera_;  // created on the first observe()
  bool camera_failed_ = false;
};

}  // namespace skillarm
