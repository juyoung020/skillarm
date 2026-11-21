// One TensorRT engine built from an ONNX graph, with device buffers for every input/output tensor.
// TensorRT 11 networks are strongly typed: the ONNX graph's own types (fp32 / fp16) decide the precision.
// The serialized engine is cached next to the ONNX file (<onnx>.sm<NN>.engine) and rebuilt when the ONNX is newer.
#pragma once
#include <cuda_runtime_api.h>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace nvinfer1 { class IRuntime; class ICudaEngine; class IExecutionContext; }

namespace skillarm {

struct TensorInfo {
  std::string name;
  bool input = false;
  int dtype = 0;               // nvinfer1::DataType value
  std::vector<int64_t> shape;  // resolved (dynamic dims fixed by the profile)
  size_t bytes = 0;
  void* dev = nullptr;         // owned buffer, or an external one after bind()
};

class TrtEngine {
 public:
  // input_shapes: full shape per input name (dynamic dims not given default to 1). shape_values: values of shape
  // tensor inputs (e.g. MoGe-2 num_tokens), fixed at build time and fed from host memory at run time.
  TrtEngine(const std::string& onnx_path, const std::map<std::string, std::vector<int64_t>>& input_shapes = {},
            const std::map<std::string, std::vector<int64_t>>& shape_values = {}, size_t workspace_mb = 2048);
  ~TrtEngine();
  TrtEngine(const TrtEngine&) = delete;
  TrtEngine& operator=(const TrtEngine&) = delete;

  TensorInfo& tensor(const std::string& name);
  const std::vector<TensorInfo>& tensors() const { return io_; }
  // use another buffer for this tensor (e.g. the previous engine's output) — no copy between engines
  void bind(const std::string& name, void* dev);
  void run(cudaStream_t stream);
  double build_seconds() const { return build_s_; }

 private:
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> ctx_;
  std::vector<TensorInfo> io_;
  std::vector<void*> owned_;
  std::map<std::string, std::vector<int64_t>> host_values_;
  double build_s_ = 0;
};

size_t trt_dtype_size(int dtype);

}  // namespace skillarm
