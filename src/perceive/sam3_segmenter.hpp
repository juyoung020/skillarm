// SAM3 text-prompted segmentation on TensorRT (tepirale/sam3-ONNX: vision_encoder, text_encoder, prompt_decoder).
// Per call: the image is resized/normalised on the GPU straight into the vision encoder input; the encoder's three
// FPN outputs are the decoder's inputs (same device buffers, no copies). The vision encoder runs once per frame and
// the text encoder once per distinct prompt (both cached). Score per query = sigmoid(logit) * sigmoid(presence).
#pragma once
#include <map>
#include <memory>
#include <string>

#include "api/segmenter.hpp"
#include "perceive/clip_tokenizer.hpp"
#include "perceive/trt_engine.hpp"

namespace skillarm {

struct Sam3Options {
  std::string dir;               // folder with the ONNX files and tokenizer.json
  std::string variant = "_fp16"; // "" (fp32) or "_fp16"
  double score_threshold = 0.5;
  int max_items = 16;
};

class Sam3Segmenter final : public Segmenter {
 public:
  explicit Sam3Segmenter(Sam3Options o);
  ~Sam3Segmenter() override;
  SegmentationSet segment_text(const CameraObservation& cam, const std::string& text) override;
  // this export has no point/box prompt inputs
  SegmentationSet segment_points(const CameraObservation& cam, const std::vector<std::array<double, 2>>& pts) override;

  struct Timing { double vision_ms = 0, text_ms = 0, decoder_ms = 0, post_ms = 0; bool vision_cached = false, text_cached = false; };
  const Timing& last_timing() const { return timing_; }

 private:
  void encode_image(const CameraObservation& cam);
  void encode_text(const std::string& text);

  Sam3Options o_;
  ClipTokenizer tok_;
  std::unique_ptr<TrtEngine> vision_, text_, decoder_;
  cudaStream_t stream_ = nullptr;
  uint8_t* d_rgb_ = nullptr;   size_t rgb_cap_ = 0;
  uint8_t* d_masks_ = nullptr; size_t masks_cap_ = 0;
  int* d_sel_ = nullptr;
  std::string image_key_, text_key_;
  struct TextCache { void* features = nullptr; std::vector<int64_t> mask; };
  std::map<std::string, TextCache> text_cache_;
  Timing timing_;
};

}  // namespace skillarm
