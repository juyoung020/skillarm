// Implementation of the agent API on top of an EnvironmentAdapter, kinematics, perception and the recorder.
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "env/adapter.hpp"
#include "kin/kinematics.hpp"
#include "api/segmenter.hpp"
#include "record/recorder.hpp"
#include "skillarm/api.hpp"

namespace skillarm {

class ApiImpl final : public Api {
 public:
  ApiImpl(EnvironmentAdapter& env, Recorder& rec, Segmenter* seg = nullptr);
  ~ApiImpl() override;

  TaskContext get_task_context() override;
  Observation get_observation() override;
  RobotState get_robot_state() override;
  SegmentationSet segment_text(const std::string& camera_name, const std::string& text) override;
  SegmentationSet segment_points(const std::string& camera_name, const std::vector<std::array<double, 2>>& points_px) override;
  PointCloud mask_to_point_cloud(const Segmentation& mask, const std::string& camera_name) override;
  PointCloud crop_point_cloud(const PointCloud& pc, const Vec3d& lower, const Vec3d& upper) override;
  ObjectGeometry estimate_geometry(const PointCloud& pc) override;
  GraspSet generate_grasps(const std::string& camera_name, const Segmentation& mask, int max_candidates) override;
  IKResult solve_ik(const Pose& target, const std::vector<double>& seed_joints) override;
  PlanResult plan_motion(const Pose& goal, const MotionStrategy& strategy) override;
  PlanResult plan_motion_joints(const std::vector<double>& goal, const MotionStrategy& strategy) override;
  ExecutionResult execute_trajectory(const Trajectory& traj) override;
  ExecutionResult set_gripper(double position) override;
  LocalizationResult localize_object(const std::string& query, const std::string& camera_name) override;
  const GraspCandidate* select_grasp(const GraspSet& grasps, const std::string& strategy) override;
  ExecutionResult move_to_joints(const std::vector<double>& target, double tolerance, int max_steps) override;
  ExecutionResult move_to_pose(const Pose& target, double tolerance, int max_steps, const MotionStrategy& strategy) override;
  ExecutionResult open_gripper() override;
  ExecutionResult close_gripper() override;
  ExecutionResult go_home() override;
  void report(const std::string& key, double value) override;
  void report(const std::string& key, const std::string& value) override;
  void log(const std::string& line) override;

  const nlohmann::json& program_result() const { return result_; }
  bool episode_over() const { return terminated_ || truncated_; }
  bool terminated() const { return terminated_; }
  bool truncated() const { return truncated_; }
  int steps() const { return steps_; }

 private:
  // internals without their own trace span
  PlanResult plan_joints_(const std::vector<double>& goal, const MotionStrategy& s);
  ExecutionResult exec_(const Trajectory& t);
  ExecutionResult gripper_(double pos, bool contact_aware);
  bool tick_(const std::vector<double>& arm, double grip);  // one env step + record; false if episode ended
  bool camera_(const std::string& name, CameraObservation& out);
  std::string desk_check_(const Trajectory& t);              // "" if the path keeps clear of the desk

  EnvironmentAdapter& env_;
  Recorder& rec_;
  Segmenter* seg_;
  std::unique_ptr<Kinematics> kin_;
  mjData* check_data_ = nullptr;
  std::vector<double> cmd_arm_;
  double cmd_grip_ = 1.0;
  Quat down_{1, 0, 0, 0};          // tool orientation at home (gripper pointing down), yaw 0
  nlohmann::json result_ = nlohmann::json::object();
  bool terminated_ = false, truncated_ = false;
  int steps_ = 0;
  Observation last_obs_;
};

}  // namespace skillarm
