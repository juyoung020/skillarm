// Recovering the z-shift (and, when the camera is not calibrated, the focal length) of an affine-invariant point map
// (MoGe), as MoGe's own infer() does.
#pragma once
#include <array>
#include <cstdint>

namespace skillarm {

// Shift t minimising sum |K * (X, Y) / (Z + t) - (u, v)|^2 over valid pixels (pixel centres u + 0.5, v + 0.5).
// points_hwc: w*h*3 floats in the OpenCV camera frame; valid: w*h flags.
double recover_shift(const float* points_hwc, const uint8_t* valid, int w, int h, const std::array<double, 9>& K);

// Unknown focal: square pixels, principal point at the image centre. For each shift the best focal has a closed form
// (least squares), so the search stays one-dimensional. Returns {focal_px, shift}.
std::array<double, 2> recover_focal_shift(const float* points_hwc, const uint8_t* valid, int w, int h);

}  // namespace skillarm
