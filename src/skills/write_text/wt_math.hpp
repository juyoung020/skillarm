// Small 3-D math for the write_text skill (row-major 3x3, metres, radians). Header-only.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace wt {

constexpr double kPi = 3.14159265358979323846;
inline double deg(double r) { return r * 180.0 / kPi; }
inline double rad(double d) { return d * kPi / 180.0; }

struct V3 {
  double x = 0, y = 0, z = 0;
  V3() = default;
  V3(double a, double b, double c) : x(a), y(b), z(c) {}
  double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
  double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator-(V3 a) { return {-a.x, -a.y, -a.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline V3 operator*(double s, V3 a) { return a * s; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double norm(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 normalize(V3 a) {
  const double n = norm(a);
  return n > 1e-15 ? a * (1.0 / n) : V3{0, 0, 0};
}
inline double angle_between(V3 a, V3 b) { return std::atan2(norm(cross(a, b)), dot(a, b)); }

struct M3 {  // row-major
  double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static M3 from_cols(V3 c0, V3 c1, V3 c2) {
    M3 r;
    r.m[0] = c0.x; r.m[3] = c0.y; r.m[6] = c0.z;
    r.m[1] = c1.x; r.m[4] = c1.y; r.m[7] = c1.z;
    r.m[2] = c2.x; r.m[5] = c2.y; r.m[8] = c2.z;
    return r;
  }
  static M3 from_mj(const double* p) {  // MuJoCo xmat (row-major)
    M3 r;
    for (int i = 0; i < 9; ++i) r.m[i] = p[i];
    return r;
  }
  V3 col(int j) const { return {m[j], m[3 + j], m[6 + j]}; }
  M3 T() const {
    M3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) r.m[3 * i + j] = m[3 * j + i];
    return r;
  }
};
inline V3 operator*(const M3& a, V3 v) {
  return {a.m[0] * v.x + a.m[1] * v.y + a.m[2] * v.z, a.m[3] * v.x + a.m[4] * v.y + a.m[5] * v.z,
          a.m[6] * v.x + a.m[7] * v.y + a.m[8] * v.z};
}
inline M3 operator*(const M3& a, const M3& b) {
  M3 r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double s = 0;
      for (int k = 0; k < 3; ++k) s += a.m[3 * i + k] * b.m[3 * k + j];
      r.m[3 * i + j] = s;
    }
  return r;
}
inline M3 rot_z(double a) {
  M3 r;
  const double c = std::cos(a), s = std::sin(a);
  r.m[0] = c; r.m[1] = -s; r.m[3] = s; r.m[4] = c;
  return r;
}
inline M3 rot_axis(V3 u, double a) {  // Rodrigues
  u = normalize(u);
  const double c = std::cos(a), s = std::sin(a), t = 1 - c;
  M3 r;
  r.m[0] = c + u.x * u.x * t;       r.m[1] = u.x * u.y * t - u.z * s; r.m[2] = u.x * u.z * t + u.y * s;
  r.m[3] = u.y * u.x * t + u.z * s; r.m[4] = c + u.y * u.y * t;       r.m[5] = u.y * u.z * t - u.x * s;
  r.m[6] = u.z * u.x * t - u.y * s; r.m[7] = u.z * u.y * t + u.x * s; r.m[8] = c + u.z * u.z * t;
  return r;
}
inline double rot_angle(const M3& a, const M3& b) {  // angle of a^T b
  const M3 r = a.T() * b;
  const double c = std::clamp((r.m[0] + r.m[4] + r.m[8] - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(c);
}

struct Pose {  // world <- frame
  V3 p;
  M3 R;
};
inline Pose operator*(const Pose& a, const Pose& b) { return {a.p + a.R * b.p, a.R * b.R}; }
inline Pose inverse(const Pose& a) {
  const M3 rt = a.R.T();
  return {-(rt * a.p), rt};
}
inline V3 xform(const Pose& a, V3 v) { return a.p + a.R * v; }

// quaternion (w,x,y,z) <-> M3
inline std::array<double, 4> quat_of(const M3& R) {
  const double* m = R.m;
  std::array<double, 4> q;
  const double tr = m[0] + m[4] + m[8];
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    q = {0.25 * s, (m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s};
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2;
    q = {(m[7] - m[5]) / s, 0.25 * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s};
  } else if (m[4] > m[8]) {
    const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2;
    q = {(m[2] - m[6]) / s, (m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s};
  } else {
    const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2;
    q = {(m[3] - m[1]) / s, (m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25 * s};
  }
  return q;
}

// ---- SO-101 servo conventions (skillarm servo server) ----
constexpr double kTickRad = 2.0 * kPi / 4096.0;
inline double tick_to_rad(double t) { return (t - 2047.0) * kTickRad; }
inline int rad_to_tick(double r) { return int(std::lround(r / kTickRad + 2047.0)); }
// gripper: ticks [1333, 2768] <-> [-0.17453, 1.74533] rad (linear)
constexpr double kGripLoTick = 1333, kGripHiTick = 2768, kGripLoRad = -0.17453, kGripHiRad = 1.74533;
inline double grip_tick_to_rad(double t) {
  return kGripLoRad + (t - kGripLoTick) * (kGripHiRad - kGripLoRad) / (kGripHiTick - kGripLoTick);
}
inline int grip_rad_to_tick(double r) {
  return int(std::lround(kGripLoTick + (r - kGripLoRad) * (kGripHiTick - kGripLoTick) / (kGripHiRad - kGripLoRad)));
}

using Q5 = std::array<double, 5>;
using Q6 = std::array<double, 6>;  // 5 arm joints + gripper (rad)

}  // namespace wt
