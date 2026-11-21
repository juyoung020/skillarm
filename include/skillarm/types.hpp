// Common types. Units follow ASPIRE: metres, radians, quaternions (w, x, y, z).
#pragma once
#include <array>
#include <cstdint>

namespace skillarm {

constexpr int kJoints = 6;  // shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
inline constexpr const char* kJointNames[kJoints] = {
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};

using Joints = std::array<double, kJoints>;      // radians (model convention)
using ServoPos = std::array<int32_t, kJoints>;   // raw servo steps, 2047 = range midpoint

struct Vec3 { double x = 0, y = 0, z = 0; };

}  // namespace skillarm
