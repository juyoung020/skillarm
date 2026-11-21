// Environment adapter: the boundary between the harness and a robot (simulated or real).
// Mirrors ASPIRE's EnvironmentAdapter protocol (docs/01_aspire_analysis.md §6), reduced to one arm.
#pragma once
#include <mujoco/mujoco.h>

#include <string>
#include <vector>

#include "skillarm/api.hpp"

namespace skillarm {

struct StepOutcome { bool ok = true; bool terminated = false; bool truncated = false; };

class EnvironmentAdapter {
 public:
  virtual ~EnvironmentAdapter() = default;
  virtual std::string embodiment() const = 0;       // "so101_sim" | "so101_real"
  virtual TaskContext reset(int seed) = 0;
  virtual TaskContext task_context() const = 0;
  virtual RobotState robot_state() = 0;
  virtual Observation observe() = 0;                // cameras may be empty when no renderer is available
  // One control tick: drive the arm towards arm_target (rad, 5 values) and the gripper towards 0..1.
  virtual StepOutcome step(const std::vector<double>& arm_target, double gripper) = 0;
  virtual double control_dt() const = 0;
  virtual std::vector<double> home() const = 0;
  virtual const mjModel* planning_model() const = 0;  // kinematic model for IK / collision checks
  // Host-only: never exposed to the program.
  virtual bool task_success() = 0;
  virtual double sim_time() const = 0;
};

}  // namespace skillarm
