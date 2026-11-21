#include "perceive/moge_depth.hpp"

#include <NvInfer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include "perceive/depth_shift.hpp"
#include "perceive/kernels.hpp"

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
  net_ = std::make_unique<TrtEngine>(o_.onnx,
                                     std::map<std::string, std::vector<int64_t>>{{"image", {1, 3, o_.height, o_.width}}},
                                     std::map<std::string, std::vector<int64_t>>{{"num_tokens", {o_.num_tokens}}});
  auto& nt = net_->tensor("num_tokens");
  if (nt.dev) {  // a plain device input in this export: upload once
    const int64_t v = o_.num_tokens;
    const int32_t v32 = int32_t(v);
    ck(cudaMemcpy(nt.dev, nt.bytes == 8 ? static_cast<const void*>(&v) : static_cast<const void*>(&v32), nt.bytes,
                  cudaMemcpyHostToDevice), "num_tokens");
  }
  ck(cudaMalloc(reinterpret_cast<void**>(&d_rgb_), size_t(o_.width) * o_.height * 3), "cudaMalloc rgb");
}

MogeDepth::~MogeDepth() {
  net_.reset();
  cudaFree(d_rgb_);
  if (stream_) cudaStreamDestroy(stream_);
}

bool MogeDepth::estimate(CameraObservation& cam) {
  const int w = cam.rgb.width, h = cam.rgb.height;
  if (w != o_.width || h != o_.height || cam.rgb.rgb.size() != size_t(w) * h * 3 || cam.intrinsics[0] <= 0) return false;
  const auto t0 = std::chrono::steady_clock::now();
  ck(cudaMemcpyAsync(d_rgb_, cam.rgb.rgb.data(), cam.rgb.rgb.size(), cudaMemcpyHostToDevice, stream_), "upload");
  auto& in = net_->tensor("image");
  const float mean[3] = {0, 0, 0}, sd[3] = {1, 1, 1};  // [0, 1] RGB; the network normalises internally
  gpu::resize_normalize(d_rgb_, w, h, in.dev, in.dtype == int(nvinfer1::DataType::kHALF), w, h, mean, sd, stream_);
  net_->run(stream_);
  const auto points = download_float(net_->tensor("points"), stream_);  // [1, h, w, 3]
  const auto mask = download_float(net_->tensor("mask"), stream_);      // [1, h, w], sigmoid
  scale_ = download_float(net_->tensor("scale"), stream_).at(0);
  std::vector<uint8_t> valid(size_t(w) * h);
  for (size_t i = 0; i < valid.size(); ++i) valid[i] = mask[i] > 0.5f && std::isfinite(points[3 * i + 2]);
  shift_ = recover_shift(points.data(), valid.data(), w, h, cam.intrinsics);
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
