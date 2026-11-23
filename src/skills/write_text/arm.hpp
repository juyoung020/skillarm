// Kinematics, IK and clearance checks for the write_text skill on a compiled Scene (own mjData per instance).
//
// IK: weighted damped least squares over the 5 arm joints with a point target on the gripper body and up to two
// axis targets (gripper-body axis -> world direction). Used for
//   * the top-down pencil grasp: site point + approach (site x) down + closing axis (site z) across the pencil;
//   * writing: pencil tip point + pencil axis direction (5 constraints for 5 joints).
// Every solution is checked with forward kinematics before it is reported ok.
#pragma once
#include <mujoco/mujoco.h>

#include <string>
#include <vector>

#include "scene.hpp"
#include "wt_math.hpp"

namespace wt {

struct AxisTarget {
  V3 local;   // gripper-body frame
  V3 world;   // desired world direction
  double w = 0.05;  // metres per unit of direction error
};

struct IkTask {
  V3 point_local;  // gripper-body frame
  V3 target;       // world
  AxisTarget axes[2];
  int n_axes = 0;
  double pos_tol = 0.0005;
  double axis_tol = rad(2.0);
};

struct IkResult {
  bool ok = false;
  Q5 q{};
  double pos_err = 1, axis_err[2] = {0, 0};
  double margin = 0;  // smallest distance to a joint limit (rad)
};

struct Clear {
  double desk = 1;      // arm (excl. jaw pads) to desk/paper, m
  double pads = 1;      // jaw pads to desk/paper
  double self = 1;      // non-adjacent links
  double pencil_arm = 1;   // pencil to arm links (excl. the jaws that hold it)
  double pencil_desk = 1;  // pencil hull (not the tip sphere) to desk/paper
  std::string self_pair;
};

class Arm {
 public:
  explicit Arm(const Scene& s);
  ~Arm();
  Arm(const Arm&) = delete;
  Arm& operator=(const Arm&) = delete;

  const Scene& scene() const { return S_; }
  void set(const Q5& q, double grip = 0.0);  // kinematics + comPos
  Pose body_pose() const;                    // world <- gripper body (after set)
  Pose site_pose() const;                    // world <- gripperframe site
  Pose site_in_body() const { return site_in_body_; }
  Pose fk_body(const Q5& q) { set(q); return body_pose(); }

  IkResult ik(const IkTask& t, const Q5& seed, int iters = 60);
  // several seeds (seed first, then standard poses, then random), best FK-verified solution
  IkResult ik_multi(const IkTask& t, const Q5& seed, int n_random = 12, unsigned rng = 1);

  // clearance at q (pencil terms only if the scene has an attached pencil)
  Clear clearance(const Q5& q, double grip, double cap = 0.05);
  double joint_margin(const Q5& q) const;

  mjData* data() { return d_; }

 private:
  const Scene& S_;
  mjData* d_;
  Pose site_in_body_;
  std::vector<double> jacp_, jacr_;
  std::vector<std::array<int, 2>> self_pairs_;
  std::vector<std::string> self_names_;
};

// standard seeds
Q5 home_q();

}  // namespace wt
