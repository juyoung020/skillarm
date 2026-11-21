// The real SO-101 behind the Pi servo server (servo/server.cpp), as an EnvironmentAdapter.
// Control: every step() sends the joint targets in the server's continuous "servo" mode and waits for the next
// 30 Hz tick; the server rate-limits goals (45 steps per 20 ms), clamps them to the EEPROM limits and latches a
// fault when a joint is blocked or overloaded. Kinematics (end-effector pose, IK, desk checks) use the MuJoCo model
// of the same arm with the desk plane at z = 0.
#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "core/calibration.hpp"
#include "env/adapter.hpp"
#include "real/arm_client.hpp"

namespace skillarm {

struct RealTask {
  std::string id = "real_pick_cube";
  std::string name = "pick up the red cube";
  std::string language = "Pick up the red cube from the desk and lift it at least 5 cm.";
  int max_steps = 1500;
};

class RealSo101 final : public EnvironmentAdapter {
 public:
  // judge: host-side success label (the real arm has no ground truth); called once by task_success().
  RealSo101(const std::string& host, int port, const std::string& calibration_json, const std::string& scene_xml,
            RealTask task, std::function<bool()> judge);
  ~RealSo101() override;

  std::string embodiment() const override { return "so101_real"; }
  TaskContext reset(int seed) override;  // reads the arm; the real scene is not re-randomised
  TaskContext task_context() const override { return ctx_; }
  RobotState robot_state() override;
  Observation observe() override;
  StepOutcome step(const std::vector<double>& arm_target, double gripper) override;
  double control_dt() const override { return 1.0 / 30.0; }
  std::vector<double> home() const override { return home_; }
  const mjModel* planning_model() const override { return m_; }
  bool task_success() override;
  double sim_time() const override;  // wall-clock seconds since reset

  const std::string& fault() const { return fault_; }

 private:
  void refresh();  // state request -> cached joint angles

  ArmClient arm_;
  Calibration cal_;
  RealTask task_;
  std::function<bool()> judge_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int qadr_[6] = {}, site_ = -1;
  double grip_lo_ = 0, grip_hi_ = 0;
  std::vector<double> home_;
  TaskContext ctx_;
  Joints q_{}, qd_{};  // rad, rad/s (index 5 = gripper)
  std::string fault_;
  std::chrono::steady_clock::time_point t0_, next_tick_;
  bool judged_ = false, success_ = false;
};

}  // namespace skillarm
