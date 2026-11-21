#include "perceive/depth_shift.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace skillarm {
namespace {
struct Samples {
  std::vector<int> idx;  // ~4096 valid pixels
  double zmin = 0, zmax = 0, zmed = 0;
};

bool sample(const float* P, const uint8_t* valid, int w, int h, Samples& s) {
  int count = 0;
  for (int i = 0; i < w * h; ++i) count += valid[i] != 0;
  if (count < 16) return false;
  const int stride = std::max(1, count / 4096);
  for (int i = 0, k = 0; i < w * h; ++i)
    if (valid[i] && (k++ % stride) == 0) s.idx.push_back(i);
  std::vector<double> zs;
  for (int i : s.idx) zs.push_back(P[3 * i + 2]);
  s.zmin = *std::min_element(zs.begin(), zs.end());
  s.zmax = *std::max_element(zs.begin(), zs.end());
  std::nth_element(zs.begin(), zs.begin() + ptrdiff_t(zs.size() / 2), zs.end());
  s.zmed = zs[zs.size() / 2];
  return true;
}

// coarse scan over t with z + t > 0 for every sample, then golden-section refinement around the best
double search(const Samples& s, const std::function<double(double)>& cost) {
  const double lo = -s.zmin + 1e-4 * std::max(1.0, std::fabs(s.zmed));
  // the scene depth range bounds how far the nearest point can sit beyond the camera
  const double span = 20.0 * std::max(s.zmax - s.zmin, 0.05 * std::max(1.0, std::fabs(s.zmed)));
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
}  // namespace

double recover_shift(const float* P, const uint8_t* valid, int w, int h, const std::array<double, 9>& K) {
  const double fx = K[0], cx = K[2], fy = K[4], cy = K[5];
  Samples s;
  if (!sample(P, valid, w, h, s)) return 0;
  return search(s, [&](double t) {
    double c = 0;
    for (int i : s.idx) {
      const double z = P[3 * i + 2] + t;
      if (z <= 1e-6) return std::numeric_limits<double>::infinity();
      const double u = (i % w) + 0.5, v = (i / w) + 0.5;  // pixel centres, as MoGe's normalized view-plane uv
      const double du = fx * P[3 * i] / z + cx - u, dv = fy * P[3 * i + 1] / z + cy - v;
      c += du * du + dv * dv;
    }
    return c;
  });
}

std::array<double, 2> recover_focal_shift(const float* P, const uint8_t* valid, int w, int h) {
  Samples s;
  if (!sample(P, valid, w, h, s)) return {0, 0};
  const double cx = 0.5 * w, cy = 0.5 * h;
  // residual of the best focal f = sum(p.q) / sum(p.p), p = (X, Y) / (Z + t), q = pixel - centre
  auto fit = [&](double t, double& f) {
    double pq = 0, pp = 0, qq = 0;
    for (int i : s.idx) {
      const double z = P[3 * i + 2] + t;
      if (z <= 1e-6) return std::numeric_limits<double>::infinity();
      const double pu = P[3 * i] / z, pv = P[3 * i + 1] / z;
      const double qu = (i % w) + 0.5 - cx, qv = (i / w) + 0.5 - cy;
      pq += pu * qu + pv * qv;
      pp += pu * pu + pv * pv;
      qq += qu * qu + qv * qv;
    }
    f = pq / pp;
    return qq - pq * pq / pp;
  };
  double f = 0;
  const double t = search(s, [&](double t) { double ff; return fit(t, ff); });
  fit(t, f);
  return {f, t};
}

}  // namespace skillarm
