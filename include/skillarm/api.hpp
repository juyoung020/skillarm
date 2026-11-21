// skillarm agent API — the only header a robot program may include (besides a few standard ones).
//
// A program is one C++ file:
//
//     #include "skillarm/api.hpp"
//     SKILLARM_PROGRAM(api) {
//       auto cube = api.localize_object("red cube");
//       ...
//       api.report("lifted", ok);
//     }
//
// It is compiled into a DLL by the harness and run once per seed. Conventions follow NVIDIA ASPIRE:
// metres, radians, quaternions (w, x, y, z), camera frames in OpenCV convention (+x right, +y down, +z forward),
// gripper 0 = closed, 1 = open. Expected failures come back as ok == false with an ApiError, never as exceptions.
#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace skillarm {

using Vec3d = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w, x, y, z

struct ApiError {
  std::string code;     // IK_UNREACHABLE, PLAN_FAILED, NOT_FOUND, PROVIDER_UNAVAILABLE, STALE_TRAJECTORY, ...
  std::string message;
  bool recoverable = true;
};

struct Pose {
  Vec3d position{0, 0, 0};
  Quat quaternion_wxyz{1, 0, 0, 0};
  std::string frame = "robot_base";
};

struct RobotState {
  std::vector<double> joint_positions;   // arm joints (5 for SO-101), rad
  std::vector<double> joint_velocities;  // rad/s
  std::vector<std::string> joint_names;
  double gripper_position = 0;           // 0 closed .. 1 open
  Pose end_effector_pose;                // tool centre point in base_frame
  std::string base_frame = "robot_base";
  double timestamp_s = 0;
};

struct TaskContext {
  std::string task_id, task_name, language;  // language = the only statement of the goal
  int episode_max_steps = 0, episode_steps_used = 0;
};

struct Image {
  int width = 0, height = 0;
  std::vector<uint8_t> rgb;     // width*height*3
};

struct CameraObservation {
  std::string name;
  Image rgb;
  std::vector<float> depth_m;   // width*height, metres, 0 = unknown
  std::array<double, 9> intrinsics{};  // fx 0 cx / 0 fy cy / 0 0 1
  Pose camera_pose;             // base <- camera (OpenCV camera frame)
  double timestamp_s = 0;
};

struct Observation {
  std::vector<CameraObservation> cameras;
  RobotState robot_state;
  double timestamp_s = 0;
};

struct Segmentation {
  int width = 0, height = 0;
  std::vector<uint8_t> mask;    // width*height, 1 = object
  std::string label;
  double score = 0;
  std::array<int, 4> box_xyxy{};
  std::string camera_name;
};
struct SegmentationSet { bool ok = false; std::vector<Segmentation> items; ApiError error; };

struct PointCloud { std::vector<Vec3d> points; std::string frame = "robot_base"; };

struct ObjectGeometry {
  Pose pose;            // oriented bounding box centre and axes
  Vec3d extents{};      // full side lengths along the box axes
  int point_count = 0;
};

struct LocalizationResult {
  bool ok = false;
  ObjectGeometry geometry;
  PointCloud point_cloud;
  Segmentation segmentation;
  ApiError error;
};

struct GraspCandidate {
  Pose pose;                   // approach direction = pose z axis
  double score = 0, width_m = 0, approach_distance_m = 0.08;
};
struct GraspSet { bool ok = false; std::vector<GraspCandidate> grasps; ApiError error; };

struct IKResult {
  bool ok = false;
  std::vector<double> joint_positions;
  double position_error_m = 0, orientation_error_rad = 0;
  ApiError error;
};

struct Trajectory {
  std::vector<std::vector<double>> joint_positions;  // N x dof
  std::vector<double> gripper_positions;             // N (may be empty)
  double dt_s = 0.02;
  std::string planner = "interpolation";
  bool collision_aware = false;
};
struct PlanResult { bool ok = false; Trajectory trajectory; ApiError error; };

struct ExecutionResult {
  bool ok = false;
  int steps_executed = 0;
  std::string status;                 // completed, tolerance_not_met, step_limit, terminated, blocked, failed
  double final_position_error_m = 0;  // for pose targets
  double final_joint_error_rad = 0;   // for joint targets
  ApiError error;
};

struct MotionStrategy {
  std::string trajectory_planner = "interpolation";  // interpolation | rrt (collision-aware)
  double max_joint_step_rad = 0.05;
};

class Api {
 public:
  virtual ~Api() = default;
  // --- atomic tools (ASPIRE names) ---
  virtual TaskContext get_task_context() = 0;
  virtual Observation get_observation() = 0;
  virtual RobotState get_robot_state() = 0;
  virtual SegmentationSet segment_text(const std::string& camera_name, const std::string& text) = 0;
  virtual SegmentationSet segment_points(const std::string& camera_name, const std::vector<std::array<double, 2>>& points_px) = 0;
  virtual PointCloud mask_to_point_cloud(const Segmentation& mask, const std::string& camera_name) = 0;
  virtual PointCloud crop_point_cloud(const PointCloud& pc, const Vec3d& lower, const Vec3d& upper) = 0;
  virtual ObjectGeometry estimate_geometry(const PointCloud& pc) = 0;
  virtual GraspSet generate_grasps(const std::string& camera_name, const Segmentation& mask, int max_candidates = 5) = 0;
  virtual IKResult solve_ik(const Pose& target, const std::vector<double>& seed_joints = {}) = 0;
  virtual PlanResult plan_motion(const Pose& goal, const MotionStrategy& strategy = {}) = 0;
  virtual PlanResult plan_motion_joints(const std::vector<double>& goal, const MotionStrategy& strategy = {}) = 0;
  virtual ExecutionResult execute_trajectory(const Trajectory& traj) = 0;
  virtual ExecutionResult set_gripper(double position) = 0;
  // --- high-level tools ---
  virtual LocalizationResult localize_object(const std::string& query, const std::string& camera_name = "") = 0;
  virtual const GraspCandidate* select_grasp(const GraspSet& grasps, const std::string& strategy = "top_down") = 0;
  virtual ExecutionResult move_to_joints(const std::vector<double>& target, double tolerance = 0.01, int max_steps = 120) = 0;
  virtual ExecutionResult move_to_pose(const Pose& target, double tolerance = 0.01, int max_steps = 120, const MotionStrategy& strategy = {}) = 0;
  virtual ExecutionResult open_gripper() = 0;
  virtual ExecutionResult close_gripper() = 0;
  virtual ExecutionResult go_home() = 0;
  // --- program output ---
  virtual void report(const std::string& key, double value) = 0;
  virtual void report(const std::string& key, const std::string& value) = 0;
  virtual void log(const std::string& line) = 0;  // goes to logs/program.stdout
};

}  // namespace skillarm

#define SKILLARM_PROGRAM(api) \
  extern "C" __declspec(dllexport) void skillarm_program(skillarm::Api& api)
