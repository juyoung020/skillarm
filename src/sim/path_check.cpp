#include "sim/path_check.hpp"

#include <cstdio>
#include <memory>

#include "sim/so101_model.hpp"

#ifndef SKILLARM_ROOT
#error "SKILLARM_ROOT must point at the repository root"
#endif

namespace skillarm {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kStepRad = 2.0 * kPi / 4096.0;
double raw_rad(int pos) { return (pos - 2047) * kStepRad; }

So101Model& model() {
  static std::unique_ptr<So101Model> m;
  if (!m) {
    const std::string root = SKILLARM_ROOT;
    m = std::make_unique<So101Model>(root + "/models/so101/scene.xml");
    // every 7th STL vertex: same sampling as the Python reference (docs/ref_path_check.py)
    m->attach_mesh_mm("gripper", root + "/models/so101/accessories/c270_wrist_mount.stl", 7);
  }
  return *m;
}
}  // namespace

PathVerdict check_path(const ServoPos& start, const std::vector<std::pair<int, int>>& steps, double min_mm) {
  auto& sim = model();
  Joints q{};
  for (int j = 0; j < kJoints; ++j) q[j] = raw_rad(start[j]);
  q[5] = 0.0;  // gripper opening barely changes clearance
  PathVerdict v;
  for (const auto& [jn, target] : steps) {
    const int j = jn - 1;
    const double a = q[j], b = raw_rad(target);
    for (int k = 0; k < 30; ++k) {
      q[j] = a + (b - a) * (k / 29.0);
      sim.set_joints(q);
      const auto c = sim.clearance();
      const double mm = c.z * 1000.0;
      if (mm < v.lowest_mm) {
        v.lowest_mm = mm;
        v.where = c.where;
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s %+.0f deg", kJointNames[j], q[j] * 180.0 / kPi);
        v.at = buf;
      }
      if (mm < min_mm) {
        v.safe = false;
        return v;
      }
    }
  }
  return v;
}

}  // namespace skillarm
