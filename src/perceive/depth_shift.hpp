// Recovering the z-shift of an affine-invariant point map (MoGe) from known camera intrinsics.
#pragma once
#include <array>
#include <cstdint>

namespace skillarm {

// Shift t minimising sum |K * (X, Y) / (Z + t) - (u, v)|^2 over valid pixels (pixel centres u + 0.5, v + 0.5).
// points_hwc: w*h*3 floats in the OpenCV camera frame; valid: w*h flags.
double recover_shift(const float* points_hwc, const uint8_t* valid, int w, int h, const std::array<double, 9>& K);

}  // namespace skillarm
