// Forward / inverse kinematics of the SO-101 arm on its official MJCF (5 arm joints + gripper).
// IK: damped least squares on the "gripperframe" site Jacobian, many seeds, accepted only after an FK check
// (same rule as ASPIRE's YAM mink IK: never report success the forward kinematics does not confirm).
#pragma once
#include <mujoco/mujoco.h>

#include <string>
#include <vector>

#include "skillarm/api.hpp"

namespace skillarm {

constexpr int kArmDof = 5;

struct IkOptions {
  double position_tol_m = 0.002;
  double orientation_tol_rad = 10.0 * 3.14159265358979323846 / 180.0;
  bool position_only = false;   // also selected by passing quaternion (0,0,0,0)
  int seeds = 24;
  int iterations = 150;
  unsigned rng_seed = 1;
};

class Kinematics {
 public:
  explicit Kinematics(const mjModel* m);  // borrows the model; owns its own mjData
  ~Kinematics();
  Kinematics(const Kinematics&) = delete;
  Kinematics& operator=(const Kinematics&) = delete;

  Pose fk(const std::vector<double>& arm_q) const;          // tool centre point in the base frame
  IKResult ik(const Pose& target, const std::vector<double>& seed, const IkOptions& o = {}) const;
  const std::vector<double>& lower() const { return lo_; }
  const std::vector<double>& upper() const { return hi_; }

 private:
  void set(const std::vector<double>& q) const;
  const mjModel* m_;
  mjData* d_;
  int site_ = -1, base_body_ = -1;
  int qadr_[kArmDof] = {}, dadr_[kArmDof] = {};
  std::vector<double> lo_, hi_;
};

// quaternion helpers (w, x, y, z)
Quat quat_mul(const Quat& a, const Quat& b);
Quat quat_conj(const Quat& q);
double quat_angle(const Quat& a, const Quat& b);  // rotation angle between two orientations, rad

}  // namespace skillarm
