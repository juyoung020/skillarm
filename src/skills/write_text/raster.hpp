// Tiny RGB raster for previews and trace images: anti-aliased thick lines (distance to segment), dots, PNG out.
// Labels are drawn with the skill's own stroke font.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "font.hpp"

namespace wt {

struct Rgb {
  uint8_t r = 0, g = 0, b = 0;
};

class Image {
 public:
  Image(int w, int h, Rgb bg = {255, 255, 255}) : w_(w), h_(h), px_(size_t(w) * size_t(h) * 3) {
    for (size_t i = 0; i < size_t(w) * size_t(h); ++i) { px_[3 * i] = bg.r; px_[3 * i + 1] = bg.g; px_[3 * i + 2] = bg.b; }
  }
  int w() const { return w_; }
  int h() const { return h_; }
  void blend(int x, int y, Rgb c, double a) {
    if (x < 0 || y < 0 || x >= w_ || y >= h_ || a <= 0) return;
    a = std::min(a, 1.0);
    uint8_t* p = &px_[(size_t(y) * size_t(w_) + size_t(x)) * 3];
    p[0] = uint8_t(p[0] + (c.r - p[0]) * a);
    p[1] = uint8_t(p[1] + (c.g - p[1]) * a);
    p[2] = uint8_t(p[2] + (c.b - p[2]) * a);
  }
  // segment with radius rad (pixels), anti-aliased over one pixel
  void line(double x0, double y0, double x1, double y1, double width, Rgb c, double alpha = 1.0) {
    const double r = 0.5 * width;
    const int xa = int(std::floor(std::min(x0, x1) - r - 1)), xb = int(std::ceil(std::max(x0, x1) + r + 1));
    const int ya = int(std::floor(std::min(y0, y1) - r - 1)), yb = int(std::ceil(std::max(y0, y1) + r + 1));
    const double dx = x1 - x0, dy = y1 - y0, l2 = dx * dx + dy * dy;
    for (int y = std::max(0, ya); y <= std::min(h_ - 1, yb); ++y)
      for (int x = std::max(0, xa); x <= std::min(w_ - 1, xb); ++x) {
        const double px = x + 0.5, py = y + 0.5;
        double t = l2 > 0 ? ((px - x0) * dx + (py - y0) * dy) / l2 : 0;
        t = std::clamp(t, 0.0, 1.0);
        const double d = std::hypot(px - (x0 + t * dx), py - (y0 + t * dy));
        const double cov = std::clamp(r + 0.5 - d, 0.0, 1.0);
        if (cov > 0) blend(x, y, c, cov * alpha);
      }
  }
  void dashed(double x0, double y0, double x1, double y1, double width, Rgb c, double dash = 6, double alpha = 1.0) {
    const double L = std::hypot(x1 - x0, y1 - y0);
    if (L < 1e-9) return;
    for (double s = 0; s < L; s += 2 * dash) {
      const double e = std::min(L, s + dash);
      line(x0 + (x1 - x0) * s / L, y0 + (y1 - y0) * s / L, x0 + (x1 - x0) * e / L, y0 + (y1 - y0) * e / L, width, c, alpha);
    }
  }
  void dot(double x, double y, double radius, Rgb c, double alpha = 1.0) { line(x, y, x, y, 2 * radius, c, alpha); }
  void rect(double x0, double y0, double x1, double y1, double width, Rgb c) {
    line(x0, y0, x1, y0, width, c); line(x1, y0, x1, y1, width, c);
    line(x1, y1, x0, y1, width, c); line(x0, y1, x0, y0, width, c);
  }
  void fill(int x0, int y0, int x1, int y1, Rgb c) {
    for (int y = std::max(0, y0); y < std::min(h_, y1); ++y)
      for (int x = std::max(0, x0); x < std::min(w_, x1); ++x) blend(x, y, c, 1.0);
  }
  // label with the stroke font; (x, y) = left baseline, size = cap height in pixels
  void text(double x, double y, const std::string& s, double size, Rgb c, double width = 1.4) {
    LayoutOptions o;
    o.size_m = size;
    const TextLayout L = layout_text(s, o);
    const double ox = x - L.minx, oy = y;  // layout is ink-centred: re-anchor at its left edge, baseline ~ y
    const double base = L.glyphs.empty() ? 0 : L.glyphs[0].y0;
    for (const auto& st : L.strokes)
      for (size_t k = 1; k < st.size(); ++k)
        line(ox + st[k - 1].x, oy - (st[k - 1].y - base), ox + st[k].x, oy - (st[k].y - base), width, c);
  }
  bool save_png(const std::string& path) const;
  const std::vector<uint8_t>& pixels() const { return px_; }
  uint8_t* data() { return px_.data(); }

 private:
  int w_, h_;
  std::vector<uint8_t> px_;
};

}  // namespace wt
