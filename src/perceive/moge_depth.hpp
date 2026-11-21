// Monocular metric depth with MoGe-2 (Ruicheng/moge-2-vit{s,b}-normal-onnx) on TensorRT.
// The network predicts a point map that is correct up to a shift along z, plus a validity mask and a metric scale.
// As in MoGe's infer() with a known field of view, the shift is recovered from the camera intrinsics by minimising
// the reprojection error of the point map; depth = (z + shift) * scale.
#pragma once
#include <memory>
#include <string>

#include "api/segmenter.hpp"
#include "perceive/trt_engine.hpp"

namespace skillarm {

struct MogeOptions {
  std::string onnx;       // models/onnx/moge2/moge2_vits_normal.onnx
  int width = 640, height = 480;  // camera resolution (the engine is built for it)
  int num_tokens = 2500;  // MoGe resolution level 9 (1200..2500)
};

class MogeDepth final : public DepthEstimator {
 public:
  explicit MogeDepth(MogeOptions o);
  ~MogeDepth() override;
  bool estimate(CameraObservation& cam) override;
  double last_ms() const { return last_ms_; }
  double last_shift() const { return shift_; }
  double last_scale() const { return scale_; }

 private:
  MogeOptions o_;
  std::unique_ptr<TrtEngine> net_;
  cudaStream_t stream_ = nullptr;
  uint8_t* d_rgb_ = nullptr;
  double last_ms_ = 0, shift_ = 0, scale_ = 1;
};

}  // namespace skillarm
