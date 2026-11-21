// skillarm CLI.
//   skillarm path-check "<6 servo positions>" "<joint:target> ..."
//     Simulates moving one joint at a time (1-based joint numbers, servo steps) and reports the lowest point of
//     the arm + wrist mount above the desk. Exit 0 = safe (>= 10 mm), 1 = would hit the desk.
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "core/calibration.hpp"
#include "sim/so101_model.hpp"

using namespace skillarm;

namespace {
constexpr double kStepRad = 2.0 * 3.14159265358979323846 / 4096.0;
double raw_rad(int pos) { return (pos - 2047) * kStepRad; }
double deg(double r) { return r * 180.0 / 3.14159265358979323846; }

int path_check(const std::string& start_s, const std::vector<std::string>& steps_s) {
  const std::string root = SKILLARM_ROOT;
  So101Model sim(root + "/models/so101/scene.xml");
  // Same sampling as the Python reference (every 7th STL vertex) so results compare 1:1.
  sim.attach_mesh_mm("gripper", root + "/models/so101/accessories/c270_wrist_mount.stl", 7);

  Joints q{};
  std::istringstream in(start_s);
  for (int j = 0; j < kJoints; ++j) {
    int p;
    in >> p;
    q[j] = raw_rad(p);
  }
  q[5] = 0.0;
  Clearance worst{1e9, {}};
  std::string worst_at;
  for (const auto& st : steps_s) {
    const int j = std::stoi(st.substr(0, st.find(':'))) - 1;
    const double a = q[j], b = raw_rad(std::stoi(st.substr(st.find(':') + 1)));
    for (int k = 0; k < 30; ++k) {
      q[j] = a + (b - a) * (k / 29.0);
      sim.set_joints(q);
      const auto c = sim.clearance();
      if (c.z < worst.z) {
        worst = c;
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s %+.0f deg", kJointNames[j], deg(q[j]));
        worst_at = buf;
      }
      if (c.z < 0.010) {
        std::printf("UNSAFE: moving %s to %+.0f deg puts %s at %+.0f mm above the desk\n", kJointNames[j],
                    deg(q[j]), c.where.c_str(), c.z * 1000);
        return 1;
      }
    }
  }
  std::printf("SAFE: lowest point %+.0f mm (%s, %s)\n", worst.z * 1000, worst.where.c_str(), worst_at.c_str());
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string cmd = argc > 1 ? argv[1] : "";
    if (cmd == "path-check" && argc >= 3) {
      std::vector<std::string> steps(argv + 3, argv + argc);
      return path_check(argv[2], steps);
    }
    std::fprintf(stderr, "usage: skillarm path-check \"<6 servo positions>\" <joint:target> ...\n");
    return 2;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 3;
  }
}
