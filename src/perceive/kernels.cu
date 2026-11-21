// CUDA pre/post-processing for the perception networks. One thread per output element, no host round trips.
#include <cuda_fp16.h>

#include "perceive/kernels.hpp"

namespace skillarm::gpu {
namespace {

template <typename T> __device__ inline T to_t(float v);
template <> __device__ inline float to_t<float>(float v) { return v; }
template <> __device__ inline __half to_t<__half>(float v) { return __float2half(v); }
template <typename T> __device__ inline float to_f(T v);
template <> __device__ inline float to_f<float>(float v) { return v; }
template <> __device__ inline float to_f<__half>(__half v) { return __half2float(v); }

// RGB8 HWC (sw x sh) -> planar CHW (dw x dh), bilinear with half-pixel centres (PIL / torch align_corners=False),
// then (v / 255 - mean) / std per channel.
template <typename T>
__global__ void k_resize_normalize(const uint8_t* __restrict__ src, int sw, int sh, T* __restrict__ dst, int dw, int dh,
                                   float3 mean, float3 inv_std) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= dw || y >= dh) return;
  const float fx = fminf(fmaxf((x + 0.5f) * sw / dw - 0.5f, 0.f), sw - 1.f);
  const float fy = fminf(fmaxf((y + 0.5f) * sh / dh - 0.5f, 0.f), sh - 1.f);
  const int x0 = int(fx), y0 = int(fy), x1 = min(x0 + 1, sw - 1), y1 = min(y0 + 1, sh - 1);
  const float ax = fx - x0, ay = fy - y0;
  const uint8_t* p00 = src + (size_t(y0) * sw + x0) * 3;
  const uint8_t* p01 = src + (size_t(y0) * sw + x1) * 3;
  const uint8_t* p10 = src + (size_t(y1) * sw + x0) * 3;
  const uint8_t* p11 = src + (size_t(y1) * sw + x1) * 3;
  const size_t plane = size_t(dw) * dh, o = size_t(y) * dw + x;
  const float m[3] = {mean.x, mean.y, mean.z}, s[3] = {inv_std.x, inv_std.y, inv_std.z};
#pragma unroll
  for (int c = 0; c < 3; ++c) {
    const float v = (1 - ay) * ((1 - ax) * p00[c] + ax * p01[c]) + ay * ((1 - ax) * p10[c] + ax * p11[c]);
    dst[c * plane + o] = to_t<T>((v * (1.f / 255.f) - m[c]) * s[c]);
  }
}

// mask logits (n masks of mw x mh) -> uint8 masks at image size, bilinear on the logits, logit > 0 <=> sigmoid > 0.5
template <typename T>
__global__ void k_masks(const T* __restrict__ logits, const int* __restrict__ sel, int mw, int mh, uint8_t* __restrict__ out,
                        int w, int h) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y, k = blockIdx.z;
  if (x >= w || y >= h) return;
  const T* L = logits + size_t(sel[k]) * mw * mh;
  const float fx = fminf(fmaxf((x + 0.5f) * mw / w - 0.5f, 0.f), mw - 1.f);
  const float fy = fminf(fmaxf((y + 0.5f) * mh / h - 0.5f, 0.f), mh - 1.f);
  const int x0 = int(fx), y0 = int(fy), x1 = min(x0 + 1, mw - 1), y1 = min(y0 + 1, mh - 1);
  const float ax = fx - x0, ay = fy - y0;
  const float v = (1 - ay) * ((1 - ax) * to_f(L[y0 * mw + x0]) + ax * to_f(L[y0 * mw + x1])) +
                  ay * ((1 - ax) * to_f(L[y1 * mw + x0]) + ax * to_f(L[y1 * mw + x1]));
  out[size_t(k) * w * h + size_t(y) * w + x] = v > 0.f ? 1 : 0;
}

dim3 grid2(int w, int h, int z = 1) { return dim3((w + 15) / 16, (h + 15) / 16, z); }

}  // namespace

void resize_normalize(const uint8_t* src, int sw, int sh, void* dst, bool half, int dw, int dh, const float mean[3],
                      const float std_[3], cudaStream_t s) {
  const float3 m{mean[0], mean[1], mean[2]}, is{1.f / std_[0], 1.f / std_[1], 1.f / std_[2]};
  if (half) k_resize_normalize<__half><<<grid2(dw, dh), dim3(16, 16), 0, s>>>(src, sw, sh, static_cast<__half*>(dst), dw, dh, m, is);
  else k_resize_normalize<float><<<grid2(dw, dh), dim3(16, 16), 0, s>>>(src, sw, sh, static_cast<float*>(dst), dw, dh, m, is);
}

void masks_from_logits(const void* logits, bool half, const int* sel, int n, int mw, int mh, uint8_t* out, int w, int h,
                       cudaStream_t s) {
  if (n <= 0) return;
  if (half) k_masks<__half><<<grid2(w, h, n), dim3(16, 16), 0, s>>>(static_cast<const __half*>(logits), sel, mw, mh, out, w, h);
  else k_masks<float><<<grid2(w, h, n), dim3(16, 16), 0, s>>>(static_cast<const float*>(logits), sel, mw, mh, out, w, h);
}

}  // namespace skillarm::gpu
