#include "perceive/sam3_segmenter.hpp"

#include <NvInfer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <numeric>
#include <stdexcept>

#include "perceive/kernels.hpp"

namespace skillarm {
namespace {
constexpr int kSize = 1008;   // vision encoder input side
constexpr int kTextLen = 32;  // CLIP context used by the export

void ck(cudaError_t e, const char* w) {
  if (e != cudaSuccess) throw std::runtime_error(std::string(w) + ": " + cudaGetErrorString(e));
}
bool is_half(const TensorInfo& t) { return t.dtype == int(nvinfer1::DataType::kHALF); }
double ms_since(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}
float to_float(const std::vector<uint8_t>& raw, size_t i, bool half) {
  if (!half) return reinterpret_cast<const float*>(raw.data())[i];
  const uint16_t h = reinterpret_cast<const uint16_t*>(raw.data())[i];  // IEEE half -> float
  const uint32_t s = (h & 0x8000u) << 16, e = (h >> 10) & 0x1F, m = h & 0x3FF;
  if (e == 0) return (s ? -1.f : 1.f) * std::ldexp(float(m), -24);  // zero / subnormal
  const uint32_t f = e == 31 ? (s | 0x7F800000u | (m << 13)) : (s | ((e + 112) << 23) | (m << 13));
  float out;
  std::memcpy(&out, &f, 4);
  return out;
}
std::vector<uint8_t> download(const TensorInfo& t, cudaStream_t s) {
  std::vector<uint8_t> v(t.bytes);
  ck(cudaMemcpyAsync(v.data(), t.dev, t.bytes, cudaMemcpyDeviceToHost, s), "download");
  return v;
}
double sigmoid(double x) { return 1.0 / (1.0 + std::exp(-x)); }
}  // namespace

Sam3Segmenter::Sam3Segmenter(Sam3Options o) : o_(std::move(o)), tok_(o_.dir + "/tokenizer.json") {
  ck(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "stream");
  const std::string v = o_.variant;
  vision_ = std::make_unique<TrtEngine>(o_.dir + "/vision_encoder" + v + ".onnx",
                                        std::map<std::string, std::vector<int64_t>>{{"pixel_values", {1, 3, kSize, kSize}}});
  text_ = std::make_unique<TrtEngine>(o_.dir + "/text_encoder" + v + ".onnx",
                                      std::map<std::string, std::vector<int64_t>>{{"input_ids", {1, kTextLen}},
                                                                                  {"attention_mask", {1, kTextLen}}});
  decoder_ = std::make_unique<TrtEngine>(o_.dir + "/prompt_decoder" + v + ".onnx",
                                         std::map<std::string, std::vector<int64_t>>{{"attention_mask", {1, kTextLen}}});
  // zero-copy chaining: encoder outputs are the decoder inputs
  for (const char* n : {"fpn_hidden_state_0", "fpn_hidden_state_1", "fpn_hidden_state_2"}) {
    auto& out = vision_->tensor(n);
    auto& in = decoder_->tensor(n);
    if (out.dtype != in.dtype || out.bytes != in.bytes)
      throw std::runtime_error(std::string("SAM3 ") + n + ": encoder output and decoder input differ in type/shape");
    decoder_->bind(n, out.dev);
  }
  auto& tf = text_->tensor("text_features");
  if (tf.bytes != decoder_->tensor("text_features").bytes) throw std::runtime_error("SAM3 text_features size mismatch");
  decoder_->bind("text_features", tf.dev);
  decoder_->bind("attention_mask", text_->tensor("attention_mask").dev);
  ck(cudaMalloc(reinterpret_cast<void**>(&d_sel_), sizeof(int) * 256), "cudaMalloc sel");
}

Sam3Segmenter::~Sam3Segmenter() {
  for (auto& [k, c] : text_cache_) cudaFree(c.features);
  cudaFree(d_rgb_);
  cudaFree(d_masks_);
  cudaFree(d_sel_);
  if (stream_) cudaStreamDestroy(stream_);
}

void Sam3Segmenter::encode_image(const CameraObservation& cam) {
  const std::string key = cam.name + "@" + std::to_string(cam.timestamp_s) + "#" + std::to_string(cam.rgb.width) + "x" +
                          std::to_string(cam.rgb.height);
  timing_.vision_cached = key == image_key_;
  if (timing_.vision_cached) return;
  const auto t0 = std::chrono::steady_clock::now();
  const size_t n = cam.rgb.rgb.size();
  if (n > rgb_cap_) {
    cudaFree(d_rgb_);
    ck(cudaMalloc(reinterpret_cast<void**>(&d_rgb_), n), "cudaMalloc rgb");
    rgb_cap_ = n;
  }
  ck(cudaMemcpyAsync(d_rgb_, cam.rgb.rgb.data(), n, cudaMemcpyHostToDevice, stream_), "upload rgb");
  auto& in = vision_->tensor("pixel_values");
  const float mean[3] = {0.5f, 0.5f, 0.5f}, sd[3] = {0.5f, 0.5f, 0.5f};  // processor_config.json
  gpu::resize_normalize(d_rgb_, cam.rgb.width, cam.rgb.height, in.dev, is_half(in), kSize, kSize, mean, sd, stream_);
  vision_->run(stream_);
  ck(cudaStreamSynchronize(stream_), "vision");
  timing_.vision_ms = ms_since(t0);
  image_key_ = key;
}

void Sam3Segmenter::encode_text(const std::string& text) {
  auto& tf = text_->tensor("text_features");
  auto& am = text_->tensor("attention_mask");
  timing_.text_cached = text == text_key_;
  if (timing_.text_cached) return;  // features already in the decoder's input buffers
  const auto t0 = std::chrono::steady_clock::now();
  auto it = text_cache_.find(text);
  if (it != text_cache_.end()) {
    timing_.text_cached = true;
    ck(cudaMemcpyAsync(tf.dev, it->second.features, tf.bytes, cudaMemcpyDeviceToDevice, stream_), "text cache");
    ck(cudaMemcpyAsync(am.dev, it->second.mask.data(), am.bytes, cudaMemcpyHostToDevice, stream_), "mask");
  } else {
    const auto e = tok_.encode(text, kTextLen);
    ck(cudaMemcpyAsync(text_->tensor("input_ids").dev, e.ids.data(), sizeof(int64_t) * kTextLen, cudaMemcpyHostToDevice,
                       stream_), "ids");
    ck(cudaMemcpyAsync(am.dev, e.attention_mask.data(), sizeof(int64_t) * kTextLen, cudaMemcpyHostToDevice, stream_), "mask");
    text_->run(stream_);
    TextCache c;
    c.mask = e.attention_mask;
    ck(cudaMalloc(reinterpret_cast<void**>(&c.features), tf.bytes), "cudaMalloc text");
    ck(cudaMemcpyAsync(c.features, tf.dev, tf.bytes, cudaMemcpyDeviceToDevice, stream_), "text copy");
    text_cache_.emplace(text, c);
  }
  ck(cudaStreamSynchronize(stream_), "text");
  timing_.text_ms = ms_since(t0);
  text_key_ = text;
}

SegmentationSet Sam3Segmenter::segment_text(const CameraObservation& cam, const std::string& text) {
  SegmentationSet out;
  if (cam.rgb.width <= 0 || cam.rgb.rgb.size() != size_t(cam.rgb.width) * cam.rgb.height * 3) {
    out.error = {"INVALID_INPUT", "camera " + cam.name + " has no RGB image", false};
    return out;
  }
  timing_ = {};
  encode_image(cam);
  encode_text(text);
  auto t0 = std::chrono::steady_clock::now();
  decoder_->run(stream_);
  const auto& L = decoder_->tensor("pred_logits");
  const auto& P = decoder_->tensor("presence_logits");
  const auto& B = decoder_->tensor("pred_boxes");
  const auto& M = decoder_->tensor("pred_masks");
  const auto lr = download(L, stream_), pr = download(P, stream_), br = download(B, stream_);
  ck(cudaStreamSynchronize(stream_), "decoder");
  timing_.decoder_ms = ms_since(t0);
  t0 = std::chrono::steady_clock::now();

  const int nq = int(L.shape.back());
  const double presence = sigmoid(to_float(pr, 0, is_half(P)));
  std::vector<std::pair<double, int>> cand;
  for (int q = 0; q < nq; ++q) {
    const double s = sigmoid(to_float(lr, size_t(q), is_half(L))) * presence;
    if (s > o_.score_threshold) cand.push_back({s, q});
  }
  std::sort(cand.rbegin(), cand.rend());
  if (int(cand.size()) > o_.max_items) cand.resize(size_t(o_.max_items));
  out.ok = true;
  if (cand.empty()) return out;

  const int w = cam.rgb.width, h = cam.rgb.height, n = int(cand.size());
  std::vector<int> sel(static_cast<size_t>(n));
  for (int k = 0; k < n; ++k) sel[size_t(k)] = cand[size_t(k)].second;
  const size_t need = size_t(n) * w * h;
  if (need > masks_cap_) {
    cudaFree(d_masks_);
    ck(cudaMalloc(reinterpret_cast<void**>(&d_masks_), need), "cudaMalloc masks");
    masks_cap_ = need;
  }
  ck(cudaMemcpyAsync(d_sel_, sel.data(), sizeof(int) * size_t(n), cudaMemcpyHostToDevice, stream_), "sel");
  const int mh = int(M.shape[M.shape.size() - 2]), mw = int(M.shape.back());
  gpu::masks_from_logits(M.dev, is_half(M), d_sel_, n, mw, mh, d_masks_, w, h, stream_);
  std::vector<uint8_t> masks(need);
  ck(cudaMemcpyAsync(masks.data(), d_masks_, need, cudaMemcpyDeviceToHost, stream_), "masks");
  ck(cudaStreamSynchronize(stream_), "post");
  for (int k = 0; k < n; ++k) {
    Segmentation s;
    s.width = w;
    s.height = h;
    s.mask.assign(masks.begin() + ptrdiff_t(size_t(k) * w * h), masks.begin() + ptrdiff_t(size_t(k + 1) * w * h));
    s.label = text;
    s.score = cand[size_t(k)].first;
    const int q = sel[size_t(k)];
    const bool hb = is_half(B);
    s.box_xyxy = {int(std::lround(to_float(br, size_t(q) * 4 + 0, hb) * w)), int(std::lround(to_float(br, size_t(q) * 4 + 1, hb) * h)),
                  int(std::lround(to_float(br, size_t(q) * 4 + 2, hb) * w)), int(std::lround(to_float(br, size_t(q) * 4 + 3, hb) * h))};
    s.camera_name = cam.name;
    out.items.push_back(std::move(s));
  }
  timing_.post_ms = ms_since(t0);
  return out;
}

SegmentationSet Sam3Segmenter::segment_points(const CameraObservation&, const std::vector<std::array<double, 2>>&) {
  SegmentationSet out;
  out.error = {"PROVIDER_UNAVAILABLE", "SAM3 export has text prompts only; use segment_text", false};
  return out;
}

}  // namespace skillarm
