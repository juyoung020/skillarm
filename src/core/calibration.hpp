// Servo steps <-> model joint angles.
// Calibration = LeRobot / so101_new_calib definition: 0 rad is the midpoint of each joint's mechanical range,
// stored on the servo as Homing_Offset so that the midpoint reads 2047. 4096 steps per turn.
#pragma once
#include <string>
#include "skillarm/types.hpp"

namespace skillarm {

struct JointCal { int id = 0; int range_min = 0; int range_max = 4095; int homing_offset = 0; };

struct Calibration {
  JointCal joint[kJoints];
  // Gripper maps its measured range onto the model range [-10 deg, 100 deg] (so101_new_calib gripper limits).
  static Calibration load(const std::string& json_path);
  double to_rad(int j, int32_t pos) const;
  int32_t to_pos(int j, double rad) const;
};

}  // namespace skillarm
