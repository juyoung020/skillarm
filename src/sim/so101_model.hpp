// SO-101 in MuJoCo: load the official MJCF, set joint angles, forward kinematics, desk clearance.
#pragma once
#include <mujoco/mujoco.h>

#include <string>
#include <utility>
#include <vector>

#include "skillarm/types.hpp"

namespace skillarm {

struct Clearance {
  double z = 0;        // lowest point above the desk plane (z = 0), metres
  std::string where;   // body (or "wrist_mount") that owns the lowest point
};

class So101Model {
 public:
  explicit So101Model(const std::string& scene_xml);
  ~So101Model();
  So101Model(const So101Model&) = delete;
  So101Model& operator=(const So101Model&) = delete;

  // Attach an STL (millimetres) rigidly to a body frame — e.g. the C270 wrist mount on "gripper".
  void attach_mesh_mm(const std::string& body, const std::string& stl_path, int stride = 1);

  void set_joints(const Joints& q);  // radians, runs mj_kinematics
  Clearance clearance() const;       // arm bodies (not world, not base) + attachments

  const mjModel* model() const { return m_; }
  const mjData* data() const { return d_; }

 private:
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int qadr_[kJoints] = {};
  std::vector<int> arm_geoms_;
  struct Attach { int body; std::vector<Vec3> pts; std::string name; };
  std::vector<Attach> attach_;
};

}  // namespace skillarm
