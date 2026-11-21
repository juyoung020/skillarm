// Perception commands of the skillarm CLI.
//   skillarm depth-shift-test                          CPU: recover a known z-shift from a synthetic point map
//   skillarm sam3 <image> "<text>" [--fp32] [--out masks.png]   GPU: text-prompted masks + timings
//   skillarm depth <image> <fx> <fy> <cx> <cy> [--vitb] [--out depth.png]  GPU: metric depth (16-bit PNG, mm)
#include "perceive_cmds.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "perceive/depth_shift.hpp"

#pragma warning(push, 0)
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"
#pragma warning(pop)

#ifdef SKILLARM_GPU
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "perceive/moge_depth.hpp"
#include "perceive/sam3_segmenter.hpp"
#endif

namespace skillarm {

int cmd_depth_shift_test() {
  // desk plane 0.55 m away tilted 30 deg, a 4 cm box on it; the network's point map = true points - shift along z
  const int w = 640, h = 480;
  const std::array<double, 9> K = {600, 0, 319.5, 0, 600, 239.5, 0, 0, 1};
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0, 0.002);
  int fails = 0;
  for (double true_shift : {-0.3, -0.05, 0.0, 0.12, 0.8}) {
    std::vector<float> P(size_t(w) * h * 3);
    std::vector<uint8_t> valid(size_t(w) * h, 1);
    for (int v = 0; v < h; ++v)
      for (int u = 0; u < w; ++u) {
        const double x = (u + 0.5 - K[2]) / K[0], y = (v + 0.5 - K[5]) / K[4];
        double z = 0.55 / (1.0 - 0.577 * y);  // ray / tilted plane intersection
        if (std::fabs(x) < 0.05 && std::fabs(y) < 0.05) z -= 0.04;
        z *= 1 + noise(rng);
        const size_t i = size_t(v) * w + u;
        P[3 * i] = float(x * z);
        P[3 * i + 1] = float(y * z);
        P[3 * i + 2] = float(z - true_shift);  // affine-invariant: shifted along z
      }
    const auto t0 = std::chrono::steady_clock::now();
    const double s = recover_shift(P.data(), valid.data(), w, h, K);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const bool ok = std::fabs(s - true_shift) < 0.005;
    fails += !ok;
    std::printf("true shift %+.3f m -> recovered %+.4f m (error %.2f mm) in %.1f ms %s\n", true_shift, s,
                1000 * std::fabs(s - true_shift), ms, ok ? "ok" : "FAIL");
  }
  return fails ? 1 : 0;
}

#ifdef SKILLARM_GPU
namespace {
void gpu_init() {
  static bool done = false;
  if (!done) { SetDllDirectoryA(SKILLARM_TENSORRT_BIN); done = true; }  // delay-loaded TensorRT DLLs
}
bool load_rgb(const std::string& path, CameraObservation& cam) {
  int w = 0, h = 0, c = 0;
  unsigned char* px = stbi_load(path.c_str(), &w, &h, &c, 3);
  if (!px) { std::printf("cannot read %s\n", path.c_str()); return false; }
  cam.name = "file";
  cam.rgb.width = w;
  cam.rgb.height = h;
  cam.rgb.rgb.assign(px, px + size_t(w) * h * 3);
  stbi_image_free(px);
  return true;
}
std::string flag_value(const std::vector<std::string>& a, const std::string& f, const std::string& def) {
  for (size_t i = 0; i + 1 < a.size(); ++i)
    if (a[i] == f) return a[i + 1];
  return def;
}
bool has(const std::vector<std::string>& a, const std::string& f) { return std::find(a.begin(), a.end(), f) != a.end(); }
}  // namespace

int cmd_sam3(const std::vector<std::string>& a) {
  if (a.size() < 2) { std::printf("usage: skillarm sam3 <image> \"<text>\" [--fp32] [--out masks.png]\n"); return 2; }
  gpu_init();
  CameraObservation cam;
  if (!load_rgb(a[0], cam)) return 1;
  Sam3Options o;
  o.dir = std::string(SKILLARM_ROOT) + "/models/onnx/sam3";
  o.variant = has(a, "--fp32") ? "" : "_fp16";
  const auto t0 = std::chrono::steady_clock::now();
  Sam3Segmenter seg(o);
  std::printf("engines ready in %.1f s (%s)\n",
              std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), o.variant.empty() ? "fp32" : "fp16");
  SegmentationSet r;
  for (int pass = 0; pass < 3; ++pass) {  // pass 0 = cold, then cached image + text: decoder only
    if (pass == 2) cam.timestamp_s = 1;    // new frame, same text
    r = seg.segment_text(cam, a[1]);
    const auto& t = seg.last_timing();
    std::printf("pass %d: vision %.1f ms%s, text %.1f ms%s, decoder %.1f ms, post %.1f ms\n", pass, t.vision_ms,
                t.vision_cached ? " (cached)" : "", t.text_ms, t.text_cached ? " (cached)" : "", t.decoder_ms, t.post_ms);
  }
  if (!r.ok) { std::printf("error %s: %s\n", r.error.code.c_str(), r.error.message.c_str()); return 1; }
  std::printf("%zu object(s) for \"%s\"\n", r.items.size(), a[1].c_str());
  std::vector<uint8_t> vis = cam.rgb.rgb;
  for (size_t k = 0; k < r.items.size(); ++k) {
    const auto& s = r.items[k];
    size_t area = 0;
    for (auto m : s.mask) area += m;
    std::printf("  #%zu score %.3f box [%d %d %d %d] area %zu px\n", k, s.score, s.box_xyxy[0], s.box_xyxy[1],
                s.box_xyxy[2], s.box_xyxy[3], area);
    const uint8_t col[3] = {uint8_t(80 + 170 * (k % 2)), uint8_t(200 - 60 * (k % 3)), uint8_t(60 * k % 255)};
    for (size_t i = 0; i < s.mask.size(); ++i)
      if (s.mask[i])
        for (int c = 0; c < 3; ++c) vis[3 * i + c] = uint8_t((vis[3 * i + c] + col[c]) / 2);
  }
  const auto out = flag_value(a, "--out", "");
  if (!out.empty()) stbi_write_png(out.c_str(), cam.rgb.width, cam.rgb.height, 3, vis.data(), cam.rgb.width * 3);
  return 0;
}

int cmd_depth(const std::vector<std::string>& a) {
  if (a.size() < 5) { std::printf("usage: skillarm depth <image> <fx> <fy> <cx> <cy> [--vitb] [--out depth.png]\n"); return 2; }
  gpu_init();
  CameraObservation cam;
  if (!load_rgb(a[0], cam)) return 1;
  cam.intrinsics = {std::stod(a[1]), 0, std::stod(a[3]), 0, std::stod(a[2]), std::stod(a[4]), 0, 0, 1};
  MogeOptions o;
  o.onnx = std::string(SKILLARM_ROOT) + "/models/onnx/moge2/moge2_" + (has(a, "--vitb") ? "vitb" : "vits") + "_normal.onnx";
  o.width = cam.rgb.width;
  o.height = cam.rgb.height;
  const auto t0 = std::chrono::steady_clock::now();
  MogeDepth net(o);
  std::printf("engine ready in %.1f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  for (int pass = 0; pass < 3; ++pass) {
    if (!net.estimate(cam)) { std::printf("depth failed\n"); return 1; }
    std::printf("pass %d: %.1f ms (scale %.4f, shift %+.4f)\n", pass, net.last_ms(), net.last_scale(), net.last_shift());
  }
  std::vector<float> d;
  for (float z : cam.depth_m) if (z > 0) d.push_back(z);
  if (d.empty()) { std::printf("no valid depth\n"); return 1; }
  std::sort(d.begin(), d.end());
  std::printf("valid %.1f %%, depth min %.3f / median %.3f / max %.3f m\n", 100.0 * d.size() / cam.depth_m.size(), d.front(),
              d[d.size() / 2], d.back());
  const auto out = flag_value(a, "--out", "");
  if (!out.empty()) {
    std::vector<uint16_t> mm(cam.depth_m.size());
    for (size_t i = 0; i < mm.size(); ++i) mm[i] = uint16_t(std::min(65535.0, std::round(cam.depth_m[i] * 1000.0)));
    // 16-bit PNG, big-endian samples
    std::vector<uint8_t> be(mm.size() * 2);
    for (size_t i = 0; i < mm.size(); ++i) { be[2 * i] = uint8_t(mm[i] >> 8); be[2 * i + 1] = uint8_t(mm[i]); }
    std::printf("note: written as 8-bit preview (near = bright)\n");
    std::vector<uint8_t> prev(mm.size());
    const float lo = d[d.size() / 50], hi = d[d.size() * 49 / 50];
    for (size_t i = 0; i < mm.size(); ++i)
      prev[i] = cam.depth_m[i] > 0 ? uint8_t(255 * std::clamp((hi - cam.depth_m[i]) / std::max(hi - lo, 1e-3f), 0.f, 1.f)) : 0;
    stbi_write_png(out.c_str(), cam.rgb.width, cam.rgb.height, 1, prev.data(), cam.rgb.width);
  }
  return 0;
}
#else
int cmd_sam3(const std::vector<std::string>&) { std::printf("built without GPU perception\n"); return 1; }
int cmd_depth(const std::vector<std::string>&) { std::printf("built without GPU perception\n"); return 1; }
#endif

}  // namespace skillarm
