// Turn a graph input of an ONNX model into a constant (initializer), editing the protobuf wire format directly.
// Used for MoGe-2's num_tokens: TensorRT cannot fold the resize sizes computed from it while it is an input,
// but folds them once it is a constant.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace skillarm {

// Writes out_path = in_path with input `name` replaced by an int64 initializer holding `values`
// with the rank the input declares (scalar, or 1-D of values.size()). Throws if the model or the input is not found.
void onnx_bake_int64_input(const std::string& in_path, const std::string& out_path, const std::string& name,
                           const std::vector<int64_t>& values);

}  // namespace skillarm
