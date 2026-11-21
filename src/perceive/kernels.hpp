// CUDA pre/post-processing kernels (kernels.cu). All pointers are device pointers; work is queued on stream s.
#pragma once
#include <cuda_runtime_api.h>

#include <cstdint>

namespace skillarm::gpu {

// RGB8 HWC image -> network input CHW (float or half), bilinear resize, (v/255 - mean)/std.
void resize_normalize(const uint8_t* src, int sw, int sh, void* dst, bool half, int dw, int dh, const float mean[3],
                      const float std_[3], cudaStream_t s);
// Selected mask logits [sel[k]] (mw x mh each) -> n binary masks at w x h.
void masks_from_logits(const void* logits, bool half, const int* sel, int n, int mw, int mh, uint8_t* out, int w, int h,
                       cudaStream_t s);

}  // namespace skillarm::gpu
