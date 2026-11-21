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
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "env/sim_pick_cube.hpp"
#include "perceive/depth_shift.hpp"
#include "runner/runner.hpp"

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
  if (a.empty()) {
    std::printf("usage: skillarm depth <image> [<fx> <fy> <cx> <cy>] [--vitb] [--out depth.png]  (no intrinsics: focal estimated)\n");
    return 2;
  }
  gpu_init();
  CameraObservation cam;
  if (!load_rgb(a[0], cam)) return 1;
  const bool K = a.size() >= 5 && a[1].rfind("--", 0) != 0;
  if (K) cam.intrinsics = {std::stod(a[1]), 0, std::stod(a[3]), 0, std::stod(a[2]), std::stod(a[4]), 0, 0, 1};
  MogeOptions o;
  o.onnx = std::string(SKILLARM_ROOT) + "/models/onnx/moge2/moge2_" + (has(a, "--vitb") ? "vitb" : "vits") + "_normal.onnx";
  o.width = cam.rgb.width;
  o.height = cam.rgb.height;
  const auto t0 = std::chrono::steady_clock::now();
  MogeDepth net(o);
  std::printf("engine ready in %.1f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  for (int pass = 0; pass < 3; ++pass) {
    if (!net.estimate(cam)) { std::printf("depth failed\n"); return 1; }
    std::printf("pass %d: %.1f ms (scale %.4f, shift %+.4f, focal %.1f px%s)\n", pass, net.last_ms(), net.last_scale(),
                net.last_shift(), cam.intrinsics[0], K ? "" : " estimated");
    if (!K && pass < 2) cam.intrinsics[0] = 0;  // estimate again on the next pass
  }
  std::vector<float> d;
  for (float z : cam.depth_m) if (z > 0) d.push_back(z);
  if (d.empty()) { std::printf("no valid depth\n"); return 1; }
  std::sort(d.begin(), d.end());
  std::printf("valid %.1f %%, depth min %.3f / median %.3f / max %.3f m\n", 100.0 * d.size() / cam.depth_m.size(), d.front(),
              d[d.size() / 2], d.back());
  // --roi x0 y0 x1 y1 (repeatable): least-squares plane z = a x + b y + c through the box's 3-D points, reported as
  // mean depth and RMS distance to the plane -- flat surfaces (wall, screen) should come out flat
  for (size_t i = 0; i + 4 < a.size(); ++i) {
    if (a[i] != "--roi") continue;
    const int x0 = std::stoi(a[i + 1]), y0 = std::stoi(a[i + 2]), x1 = std::stoi(a[i + 3]), y1 = std::stoi(a[i + 4]);
    const double fx = cam.intrinsics[0], fy = cam.intrinsics[4], cx = cam.intrinsics[2], cy = cam.intrinsics[5];
    double S[3][3] = {}, r[3] = {}, zsum = 0;
    std::vector<std::array<double, 3>> pts;
    for (int v = y0; v < y1; ++v)
      for (int u = x0; u < x1; ++u) {
        const double z = cam.depth_m[size_t(v) * cam.rgb.width + u];
        if (z <= 0) continue;
        const double x = (u - cx) * z / fx, y = (v - cy) * z / fy, row[3] = {x, y, 1};
        for (int p = 0; p < 3; ++p) {
          for (int q = 0; q < 3; ++q) S[p][q] += row[p] * row[q];
          r[p] += row[p] * z;
        }
        pts.push_back({x, y, z});
        zsum += z;
      }
    if (pts.size() < 10) continue;
    // solve S k = r (3x3, Cramer)
    auto det = [](double m[3][3]) {
      return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
             m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    const double D = det(S);
    double k[3];
    for (int c = 0; c < 3; ++c) {
      double M[3][3];
      for (int p = 0; p < 3; ++p)
        for (int q = 0; q < 3; ++q) M[p][q] = q == c ? r[p] : S[p][q];
      k[c] = det(M) / D;
    }
    const double nrm = std::sqrt(k[0] * k[0] + k[1] * k[1] + 1);
    double ss = 0;
    for (const auto& p : pts) {
      const double e = (k[0] * p[0] + k[1] * p[1] + k[2] - p[2]) / nrm;
      ss += e * e;
    }
    std::printf("roi [%d %d %d %d]: mean depth %.3f m, plane RMS %.1f mm, tilt from the image plane %.1f deg\n", x0, y0, x1,
                y1, zsum / double(pts.size()), 1000 * std::sqrt(ss / double(pts.size())),
                std::acos(1 / nrm) * 180 / 3.14159265358979);
  }
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

namespace {
struct Backproject {
  double R[9], t[3], fx, fy, cx, cy;
  explicit Backproject(const CameraObservation& c) {
    const mjtNum q[4] = {c.camera_pose.quaternion_wxyz[0], c.camera_pose.quaternion_wxyz[1],
                         c.camera_pose.quaternion_wxyz[2], c.camera_pose.quaternion_wxyz[3]};
    mju_quat2Mat(R, q);
    for (int i = 0; i < 3; ++i) t[i] = c.camera_pose.position[size_t(i)];
    fx = c.intrinsics[0], fy = c.intrinsics[4], cx = c.intrinsics[2], cy = c.intrinsics[5];
  }
  std::array<double, 3> world(double u, double v, double z) const {
    const double x = (u - cx) * z / fx, y = (v - cy) * z / fy;
    return {R[0] * x + R[1] * y + R[2] * z + t[0], R[3] * x + R[4] * y + R[5] * z + t[1],
            R[6] * x + R[7] * y + R[8] * z + t[2]};
  }
  std::array<double, 2> pixel(const Vec3d& p) const {  // world -> image
    const double d[3] = {p[0] - t[0], p[1] - t[1], p[2] - t[2]};
    const double x = R[0] * d[0] + R[3] * d[1] + R[6] * d[2], y = R[1] * d[0] + R[4] * d[1] + R[7] * d[2],
                 z = R[2] * d[0] + R[5] * d[1] + R[8] * d[2];
    return {fx * x / z + cx, fy * y / z + cy};
  }
};
double dist3(const std::array<double, 3>& a, const Vec3d& b) {
  return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}
// centroid of the masked pixels back-projected with the given depth map
bool mask_centroid(const Backproject& bp, const std::vector<uint8_t>& mask, const std::vector<float>& depth, int w,
                   std::array<double, 3>& c) {
  c = {0, 0, 0};
  size_t n = 0;
  for (size_t i = 0; i < mask.size(); ++i)
    if (mask[i] && depth[i] > 0) {
      const auto p = bp.world(double(i % size_t(w)), double(i / size_t(w)), depth[i]);
      for (int k = 0; k < 3; ++k) c[size_t(k)] += p[size_t(k)];
      ++n;
    }
  if (!n) return false;
  for (auto& v : c) v /= double(n);
  return true;
}
}  // namespace

// skillarm sim-perceive [seed] [--out dir] [--net-scale]   (--net-scale: MoGe metric scale instead of the desk plane)
//   renderer: depth at the cube pixel lands on the cube, a floor pixel lands on z = 0
//   SAM3 (GPU): "red cube" mask -> 3-D centroid with rendered depth vs ground truth
//   MoGe-2 (GPU): depth from the RGB alone vs rendered depth, and the cube centroid with MoGe depth
int cmd_sim_perceive(const std::vector<std::string>& a) {
  int seed = 0;
  std::string out;
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i] == "--out" && i + 1 < a.size()) out = a[++i];
    else if (a[i] == "--net-scale") continue;
    else seed = std::stoi(a[i]);
  }
  SimPickCube env(std::string(SKILLARM_ROOT) + "/models/so101/task_pick_cube.xml");
  env.reset(seed);
  auto t0 = std::chrono::steady_clock::now();
  const Observation obs = env.observe();
  const double render_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (obs.cameras.empty()) { std::printf("no camera rendered\n"); return 1; }
  const auto& cam = obs.cameras[0];
  const int w = cam.rgb.width, h = cam.rgb.height;
  t0 = std::chrono::steady_clock::now();
  (void)env.observe();
  const double render2_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::printf("camera %s %dx%d, fx %.1f, first frame %.1f ms (context), next %.1f ms\n", cam.name.c_str(), w, h,
              cam.intrinsics[0], render_ms, render2_ms);
  if (!out.empty()) {
    std::filesystem::create_directories(out);
    stbi_write_png((out + "/rgb.png").c_str(), w, h, 3, cam.rgb.rgb.data(), w * 3);
  }

  int fails = 0;
  const Backproject bp(cam);
  const Vec3d cube = env.cube_position();
  const auto px = bp.pixel(cube);
  const int cu = int(std::lround(px[0])), cv = int(std::lround(px[1]));
  if (cu < 0 || cv < 0 || cu >= w || cv >= h) { std::printf("cube outside the image\n"); return 1; }
  const auto on_cube = bp.world(cu, cv, cam.depth_m[size_t(cv) * w + cu]);
  const double e_cube = dist3(on_cube, cube);
  // something in front of the cube (the gripper) hides its centre pixel: the hit is nearer than the cube
  const double cube_depth = std::sqrt((cube[0] - bp.t[0]) * (cube[0] - bp.t[0]) + (cube[1] - bp.t[1]) * (cube[1] - bp.t[1]) +
                                      (cube[2] - bp.t[2]) * (cube[2] - bp.t[2]));
  const double hit_depth = dist3(on_cube, {bp.t[0], bp.t[1], bp.t[2]});
  const bool occluded = !(e_cube < 0.03) && hit_depth < cube_depth - 0.03;
  const bool ok_cube = e_cube < 0.03 || occluded;  // a surface point of a 3 cm cube
  fails += !ok_cube;
  std::printf("renderer: cube pixel (%d, %d) back-projects %.1f mm from the cube centre %s\n", cu, cv, 1000 * e_cube,
              occluded ? "(occluded by the arm, skipped)" : ok_cube ? "ok" : "FAIL");
  const auto floor = bp.world(10, h - 10, cam.depth_m[size_t(h - 10) * w + 10]);
  const bool ok_floor = std::fabs(floor[2]) < 0.002;
  fails += !ok_floor;
  std::printf("renderer: bottom-left pixel back-projects to z = %+.2f mm %s\n", 1000 * floor[2], ok_floor ? "ok" : "FAIL");

#ifdef SKILLARM_GPU
  gpu_init();
  Sam3Options so;
  so.dir = std::string(SKILLARM_ROOT) + "/models/onnx/sam3";
  so.variant = "_fp16";
  Sam3Segmenter seg(so);
  auto r = seg.segment_text(cam, "red cube");
  r = seg.segment_text(cam, "red cube");  // warm
  if (!r.ok || r.items.empty()) { std::printf("SAM3: no cube found\n"); return 1; }
  const auto& m = r.items[0];
  std::array<double, 3> c{};
  mask_centroid(bp, m.mask, cam.depth_m, w, c);
  const double e_seg = dist3(c, cube);
  const bool ok_seg = e_seg < 0.02;
  fails += !ok_seg;
  std::printf("SAM3: score %.3f, mask centroid (rendered depth) %.1f mm from the cube centre %s\n", m.score,
              1000 * e_seg, ok_seg ? "ok" : "FAIL");

  MogeOptions mo;
  mo.onnx = std::string(SKILLARM_ROOT) + "/models/onnx/moge2/moge2_vits_normal.onnx";
  mo.width = w;
  mo.height = h;
  if (std::find(a.begin(), a.end(), "--net-scale") == a.end()) mo.desk_z = 0;  // desk = z 0 in the base frame
  MogeDepth net(mo);
  CameraObservation est = cam;
  est.depth_m.clear();
  net.estimate(est);
  net.estimate(est);  // warm
  std::vector<double> rel;
  for (size_t i = 0; i < est.depth_m.size(); ++i)
    if (est.depth_m[i] > 0 && cam.depth_m[i] > 0) rel.push_back(std::fabs(est.depth_m[i] - cam.depth_m[i]) / cam.depth_m[i]);
  std::sort(rel.begin(), rel.end());
  const double med = rel.empty() ? 1 : rel[rel.size() / 2], p90 = rel.empty() ? 1 : rel[rel.size() * 9 / 10];
  {  // shape only: error left after the best single scale factor
    std::vector<double> ratio, rel2;
    for (size_t i = 0; i < est.depth_m.size(); ++i)
      if (est.depth_m[i] > 0 && cam.depth_m[i] > 0) ratio.push_back(cam.depth_m[i] / est.depth_m[i]);
    std::nth_element(ratio.begin(), ratio.begin() + ptrdiff_t(ratio.size() / 2), ratio.end());
    const double k = ratio.empty() ? 1 : ratio[ratio.size() / 2];
    for (size_t i = 0; i < est.depth_m.size(); ++i)
      if (est.depth_m[i] > 0 && cam.depth_m[i] > 0) rel2.push_back(std::fabs(k * est.depth_m[i] - cam.depth_m[i]) / cam.depth_m[i]);
    std::sort(rel2.begin(), rel2.end());
    if (!rel2.empty())
      std::printf("MoGe-2 shape: true/estimated scale %.3f, error after rescaling median %.1f %% / p90 %.1f %%\n", k,
                  100 * rel2[rel2.size() / 2], 100 * rel2[rel2.size() * 9 / 10]);
  }
  std::array<double, 3> cm{};
  const bool have = mask_centroid(bp, m.mask, est.depth_m, w, cm);
  const double e_moge = have ? dist3(cm, cube) : 1;
  const bool ok_moge = med < 0.05 && e_moge < 0.04;  // desk-anchored metric depth, cube found within 4 cm
  fails += !ok_moge;
  std::printf("MoGe-2: %.1f ms, depth error median %.1f %% / p90 %.1f %%, cube centroid %.1f mm off %s\n", net.last_ms(),
              100 * med, 100 * p90, 1000 * e_moge, ok_moge ? "ok" : "FAIL");
  if (!out.empty()) {
    std::vector<uint8_t> vis = cam.rgb.rgb;
    for (size_t i = 0; i < m.mask.size(); ++i)
      if (m.mask[i]) { vis[3 * i] = 40; vis[3 * i + 1] = 220; vis[3 * i + 2] = 40; }
    stbi_write_png((out + "/sam3.png").c_str(), w, h, 3, vis.data(), w * 3);
    std::vector<uint8_t> err(size_t(w) * h, 0);  // white = 10 % error or more
    for (size_t i = 0; i < err.size(); ++i)
      if (est.depth_m[i] > 0 && cam.depth_m[i] > 0)
        err[i] = uint8_t(std::min(255.0, 2550.0 * std::fabs(est.depth_m[i] - cam.depth_m[i]) / cam.depth_m[i]));
    stbi_write_png((out + "/moge_error.png").c_str(), w, h, 1, err.data(), w);
  }
#else
  std::printf("SAM3 / MoGe-2 checks need the GPU build\n");
#endif
  return fails ? 1 : 0;
}


void register_perception() {
#ifdef SKILLARM_GPU
  set_segmenter_factory([]() -> std::unique_ptr<Segmenter> {
    gpu_init();
    Sam3Options o;
    o.dir = std::string(SKILLARM_ROOT) + "/models/onnx/sam3";
    o.variant = "_fp16";  // same masks as fp32 on the cube scenes, about 4x faster
    return std::make_unique<Sam3Segmenter>(o);
  });
#endif
}
}  // namespace skillarm
