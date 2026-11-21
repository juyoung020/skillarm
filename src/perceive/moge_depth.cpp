#include "perceive/moge_depth.hpp"

#include <NvInfer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <vector>

#include "perceive/depth_shift.hpp"
#include "perceive/kernels.hpp"
#include "perceive/onnx_bake.hpp"

namespace skillarm {
namespace {
void ck(cudaError_t e, const char* w) {
  if (e != cudaSuccess) throw std::runtime_error(std::string(w) + ": " + cudaGetErrorString(e));
}
float half_to_float(uint16_t h) {
  const uint32_t s = (h & 0x8000u) << 16, e = (h >> 10) & 0x1F, m = h & 0x3FF;
  if (e == 0) return (s ? -1.f : 1.f) * std::ldexp(float(m), -24);
  const uint32_t f = e == 31 ? (s | 0x7F800000u | (m << 13)) : (s | ((e + 112) << 23) | (m << 13));
  float out;
  std::memcpy(&out, &f, 4);
  return out;
}
std::vector<float> download_float(const TensorInfo& t, cudaStream_t s) {
  std::vector<uint8_t> raw(t.bytes);
  ck(cudaMemcpyAsync(raw.data(), t.dev, t.bytes, cudaMemcpyDeviceToHost, s), "download");
  ck(cudaStreamSynchronize(s), "sync");
  const bool half = t.dtype == int(nvinfer1::DataType::kHALF);
  const size_t n = t.bytes / (half ? 2 : 4);
  std::vector<float> v(n);
  if (half) for (size_t i = 0; i < n; ++i) v[i] = half_to_float(reinterpret_cast<const uint16_t*>(raw.data())[i]);
  else std::memcpy(v.data(), raw.data(), t.bytes);
  return v;
}
}  // namespace

MogeDepth::MogeDepth(MogeOptions o) : o_(std::move(o)) {
  ck(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "stream");
  // num_tokens becomes a constant in a side copy of the ONNX so TensorRT can fold the resize sizes derived from it
  const std::string baked = o_.onnx.substr(0, o_.onnx.size() - 5) + ".tokens" + std::to_string(o_.num_tokens) + ".onnx";
  namespace fs = std::filesystem;
  if (!fs::exists(baked) || fs::last_write_time(baked) < fs::last_write_time(o_.onnx))
    onnx_bake_int64_input(o_.onnx, baked, "num_tokens", {o_.num_tokens});
  net_ = std::make_unique<TrtEngine>(baked,
                                     std::map<std::string, std::vector<int64_t>>{{"image", {1, 3, o_.height, o_.width}}});
  for (const auto& t : net_->tensors())  // vits export: "scale", vitb export: "metric_scale"
    if (!t.input && t.name.find("scale") != std::string::npos) scale_name_ = t.name;
  if (scale_name_.empty()) throw std::runtime_error(o_.onnx + ": no metric scale output");
  ck(cudaMalloc(reinterpret_cast<void**>(&d_rgb_), size_t(o_.width) * o_.height * 3), "cudaMalloc rgb");
}

MogeDepth::~MogeDepth() {
  net_.reset();
  cudaFree(d_rgb_);
  if (stream_) cudaStreamDestroy(stream_);
}

// Metric scale from the desk: with the camera pose known, every pixel's ray meets the plane z = desk_z at a known
// depth. Most of the image is desk, so the median ratio (plane depth / relative depth), refined on the pixels that
// agree within 3 %, is the scale; robot and objects stand above the plane and fall out as outliers.
double MogeDepth::desk_scale(const CameraObservation& cam, const std::vector<float>& points,
                             const std::vector<uint8_t>& valid) const {
  const int w = cam.rgb.width, h = cam.rgb.height;
  const auto& q = cam.camera_pose.quaternion_wxyz;
  const double a = q[0], b = q[1], c = q[2], d = q[3];
  const double r20 = 2 * (b * d - a * c), r21 = 2 * (c * d + a * b), r22 = 1 - 2 * (b * b + c * c);  // world z row
  const double tz = cam.camera_pose.position[2];
  const double fx = cam.intrinsics[0], fy = cam.intrinsics[4], cx = cam.intrinsics[2], cy = cam.intrinsics[5];
  std::vector<double> ratio, plane;
  std::vector<size_t> idx;
  for (int v = 0; v < h; v += 2)
    for (int u = 0; u < w; u += 2) {
      const size_t i = size_t(v) * w + u;
      if (!valid[i]) continue;
      const double dz = r20 * (u - cx) / fx + r21 * (v - cy) / fy + r22;  // world z of the ray per unit depth
      const double t = (o_.desk_z - tz) / dz;
      const double z = points[3 * i + 2] + shift_;
      if (dz >= -1e-6 || t <= 0 || z <= 0) continue;
      ratio.push_back(t / z);
      plane.push_back(t);
      idx.push_back(i);
    }
  desk_fraction_ = 0;
  if (ratio.size() < 100) return 0;
  auto median = [](std::vector<double> v) {
    std::nth_element(v.begin(), v.begin() + ptrdiff_t(v.size() / 2), v.end());
    return v[v.size() / 2];
  };
  // densest ratio (log histogram, 1 % bins) instead of the median: the desk may be a minority of the frame
  std::vector<double> lr(ratio.size());
  for (size_t k = 0; k < ratio.size(); ++k) lr[k] = std::log(ratio[k]);
  const double lo = *std::min_element(lr.begin(), lr.end()), bin = 0.01;
  std::vector<int> hist(size_t((*std::max_element(lr.begin(), lr.end()) - lo) / bin) + 3, 0);
  for (double v : lr) ++hist[size_t((v - lo) / bin)];
  size_t best = 0;
  int best_n = 0;
  for (size_t k = 0; k + 2 < hist.size(); ++k)  // 3-bin window
    if (hist[k] + hist[k + 1] + hist[k + 2] > best_n) { best_n = hist[k] + hist[k + 1] + hist[k + 2]; best = k + 1; }
  const double s_mode = std::exp(lo + (double(best) + 0.5) * bin);
  auto refine = [&](double s, size_t& inliers) {
    inliers = 0;
    for (int it = 0; it < 3; ++it) {
      std::vector<double> in;
      for (size_t k = 0; k < ratio.size(); ++k)
        if (std::fabs(s * (points[3 * idx[k] + 2] + shift_) - plane[k]) < 0.03 * plane[k]) in.push_back(ratio[k]);
      inliers = in.size();
      if (in.size() < 100) break;
      s = median(in);
    }
    return s;
  };
  // the median start suits a frame that is mostly desk (it measured better on the sim cube); the densest ratio is
  // the fallback when the desk is a minority. Accept only when enough of the frame agrees with one plane scale.
  const double sampled = double(w / 2) * double(h / 2);
  size_t in_med = 0, in_mode = 0;
  const double s_med = refine(median(ratio), in_med);
  if (double(in_med) / sampled >= 0.05) {
    desk_fraction_ = double(in_med) / sampled;
    return s_med;
  }
  const double s2 = refine(s_mode, in_mode);
  desk_fraction_ = double(in_mode) / sampled;
  return desk_fraction_ >= 0.05 ? s2 : 0;
}

bool MogeDepth::estimate(CameraObservation& cam) {
  const int w = cam.rgb.width, h = cam.rgb.height;
  if (w != o_.width || h != o_.height || cam.rgb.rgb.size() != size_t(w) * h * 3) return false;
  const auto t0 = std::chrono::steady_clock::now();
  ck(cudaMemcpyAsync(d_rgb_, cam.rgb.rgb.data(), cam.rgb.rgb.size(), cudaMemcpyHostToDevice, stream_), "upload");
  auto& in = net_->tensor("image");
  const float mean[3] = {0, 0, 0}, sd[3] = {1, 1, 1};  // [0, 1] RGB; the network normalises internally
  gpu::resize_normalize(d_rgb_, w, h, in.dev, in.dtype == int(nvinfer1::DataType::kHALF), w, h, mean, sd, stream_);
  net_->run(stream_);
  const auto points = download_float(net_->tensor("points"), stream_);  // [1, h, w, 3]
  const auto mask = download_float(net_->tensor("mask"), stream_);      // [1, h, w], sigmoid
  scale_ = download_float(net_->tensor(scale_name_), stream_).at(0);
  std::vector<uint8_t> valid(size_t(w) * h);
  for (size_t i = 0; i < valid.size(); ++i) valid[i] = mask[i] > 0.5f && std::isfinite(points[3 * i + 2]);
  if (cam.intrinsics[0] > 0) {
    shift_ = recover_shift(points.data(), valid.data(), w, h, cam.intrinsics);
  } else {  // uncalibrated camera: focal from the point map too, written back into the observation
    const auto fs = recover_focal_shift(points.data(), valid.data(), w, h);
    shift_ = fs[1];
    cam.intrinsics = {fs[0], 0, 0.5 * w - 0.5, 0, fs[0], 0.5 * h - 0.5, 0, 0, 1};
  }
  if (std::isfinite(o_.desk_z)) {
    const double s = desk_scale(cam, points, valid);
    if (s > 0) scale_ = s;
  }
  cam.depth_m.assign(size_t(w) * h, 0.f);
  for (size_t i = 0; i < valid.size(); ++i)
    if (valid[i]) {
      const double z = (points[3 * i + 2] + shift_) * scale_;
      cam.depth_m[i] = z > 0 ? float(z) : 0.f;
    }
  last_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return true;
}

}  // namespace skillarm
