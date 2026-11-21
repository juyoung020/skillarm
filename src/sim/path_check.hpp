// Desk-clearance check for a joint-by-joint move, simulated in MuJoCo with the wrist camera mount attached.
#pragma once
#include <string>
#include <utility>
#include <vector>

#include "skillarm/types.hpp"

namespace skillarm {

struct PathVerdict {
  bool safe = true;
  double lowest_mm = 1e9;
  std::string where;   // body owning the lowest point
  std::string at;      // joint and angle where it happened
};

// start: current servo positions; steps: (joint 1..6, target servo position), executed in order.
PathVerdict check_path(const ServoPos& start, const std::vector<std::pair<int, int>>& steps, double min_mm = 10.0);

}  // namespace skillarm
