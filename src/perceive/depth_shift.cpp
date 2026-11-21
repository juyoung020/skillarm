#include "perceive/depth_shift.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace skillarm {

double recover_shift(const float* P, const uint8_t* valid, int w, int h, const std::array<double, 9>& K) {
  const double fx = K[0], cx = K[2], fy = K[4], cy = K[5];
  // subsample to ~4096 valid pixels
  std::vector<int> idx;
  int count = 0;
  for (int i = 0; i < w * h; ++i) count += valid[i] != 0;
  if (count < 16) return 0;
  const int stride = std::max(1, count / 4096);
  for (int i = 0, k = 0; i < w * h; ++i)
    if (valid[i] && (k++ % stride) == 0) idx.push_back(i);
  double zmin = std::numeric_limits<double>::max(), zmax = -zmin;
  std::vector<double> zs;
  for (int i : idx) {
    zmin = std::min(zmin, double(P[3 * i + 2]));
    zmax = std::max(zmax, double(P[3 * i + 2]));
    zs.push_back(P[3 * i + 2]);
  }
  std::nth_element(zs.begin(), zs.begin() + ptrdiff_t(zs.size() / 2), zs.end());
  const double zmed = zs[zs.size() / 2];
  auto cost = [&](double t) {
    double c = 0;
    for (int i : idx) {
      const double z = P[3 * i + 2] + t;
      if (z <= 1e-6) return std::numeric_limits<double>::infinity();
      const double u = (i % w) + 0.5, v = (i / w) + 0.5;  // pixel centres, as MoGe's normalized view-plane uv
      const double du = fx * P[3 * i] / z + cx - u, dv = fy * P[3 * i + 1] / z + cy - v;
      c += du * du + dv * dv;
    }
    return c;
  };
  // coarse scan over t with z + t > 0 for every sample, then golden-section refinement around the best
  const double lo = -zmin + 1e-4 * std::max(1.0, std::fabs(zmed));
  // the scene depth range bounds how far the nearest point can sit beyond the camera
  const double span = 20.0 * std::max(zmax - zmin, 0.05 * std::max(1.0, std::fabs(zmed)));
  constexpr int kScan = 200;
  double best_t = 0, best_c = std::numeric_limits<double>::infinity();
  for (int k = 0; k <= kScan; ++k) {
    const double t = lo + span * std::pow(double(k) / kScan, 2.0);  // denser near the lower bound
    const double c = cost(t);
    if (c < best_c) { best_c = c; best_t = t; }
  }
  const double step = span / kScan * 4;
  double a = std::max(lo, best_t - step), b = best_t + step;
  const double g = 0.6180339887498949;
  double x1 = b - g * (b - a), x2 = a + g * (b - a), f1 = cost(x1), f2 = cost(x2);
  for (int it = 0; it < 60; ++it) {
    if (f1 < f2) { b = x2; x2 = x1; f2 = f1; x1 = b - g * (b - a); f1 = cost(x1); }
    else { a = x1; x1 = x2; f1 = f2; x2 = a + g * (b - a); f2 = cost(x2); }
  }
  return 0.5 * (a + b);
}

}  // namespace skillarm
