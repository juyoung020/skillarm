// Perception provider interfaces used by the agent API. Optional: without them, the perception tools report
// PROVIDER_UNAVAILABLE (segmenter) or fall back to the camera's own depth (depth estimator).
#pragma once
#include <array>
#include <string>
#include <vector>

#include "skillarm/api.hpp"

namespace skillarm {

class Segmenter {
 public:
  virtual ~Segmenter() = default;
  virtual SegmentationSet segment_text(const CameraObservation& cam, const std::string& text) = 0;
  virtual SegmentationSet segment_points(const CameraObservation& cam, const std::vector<std::array<double, 2>>& pts) = 0;
};

// Monocular metric depth: fills cam.depth_m (metres, 0 = unknown) from cam.rgb and cam.intrinsics. With
// intrinsics[0] <= 0 (uncalibrated camera) the focal length is estimated too and written into cam.intrinsics.
class DepthEstimator {
 public:
  virtual ~DepthEstimator() = default;
  virtual bool estimate(CameraObservation& cam) = 0;
};

}  // namespace skillarm
