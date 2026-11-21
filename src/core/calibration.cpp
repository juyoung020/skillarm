#include "core/calibration.hpp"

#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace skillarm {
namespace {
constexpr double kStepRad = 2.0 * 3.14159265358979323846 / 4096.0;
constexpr double kGripMin = -0.17453297762778586, kGripMax = 1.7453291995659765;  // so101_new_calib.xml

int field(const std::string& block, const char* key) {
  std::smatch m;
  std::regex re(std::string("\"") + key + "\"\\s*:\\s*(-?\\d+)");
  if (!std::regex_search(block, m, re)) throw std::runtime_error(std::string("calibration: missing ") + key);
  return std::stoi(m[1]);
}
}  // namespace

Calibration Calibration::load(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string s = ss.str();
  Calibration c;
  for (int j = 0; j < kJoints; ++j) {
    const auto at = s.find(std::string("\"") + kJointNames[j] + "\"");
    if (at == std::string::npos) throw std::runtime_error(std::string("calibration: no joint ") + kJointNames[j]);
    const std::string block = s.substr(at, s.find('}', at) - at);
    c.joint[j] = {field(block, "id"), field(block, "range_min"), field(block, "range_max"), field(block, "homing_offset")};
  }
  return c;
}

double Calibration::to_rad(int j, int32_t pos) const {
  if (j == kJoints - 1) {
    const auto& g = joint[j];
    double t = double(pos - g.range_min) / double(g.range_max - g.range_min);
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    return kGripMin + (kGripMax - kGripMin) * t;
  }
  return (pos - 2047) * kStepRad;
}

int32_t Calibration::to_pos(int j, double rad) const {
  if (j == kJoints - 1) {
    const auto& g = joint[j];
    const double t = (rad - kGripMin) / (kGripMax - kGripMin);
    return int32_t(std::lround(g.range_min + t * (g.range_max - g.range_min)));
  }
  return int32_t(std::lround(rad / kStepRad)) + 2047;
}

}  // namespace skillarm
