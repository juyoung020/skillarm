#include "perceive/trt_engine.hpp"

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace skillarm {
namespace fs = std::filesystem;

namespace {
class Logger final : public nvinfer1::ILogger {
 public:
  void log(Severity s, const char* msg) noexcept override {
    if (s <= Severity::kWARNING) std::fprintf(stderr, "[TensorRT] %s\n", msg);
  }
};
Logger g_log;

void check(cudaError_t e, const char* what) {
  if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
}

std::string engine_path(const std::string& onnx, const std::map<std::string, std::vector<int64_t>>& shapes,
                        const std::map<std::string, std::vector<int64_t>>& values) {
  int dev = 0;
  cudaDeviceProp p{};
  check(cudaGetDevice(&dev), "cudaGetDevice");
  check(cudaGetDeviceProperties(&p, dev), "cudaGetDeviceProperties");
  std::string sig;  // fixed shapes are baked into the engine, so they are part of its file name
  for (const auto* m : {&shapes, &values})
    for (const auto& [k, v] : *m) {
      sig += "_" + k;
      for (auto x : v) sig += "-" + std::to_string(x);
    }
  uint32_t h = 2166136261u;  // FNV-1a
  for (unsigned char c : sig) h = (h ^ c) * 16777619u;
  char hex[20];
  std::snprintf(hex, sizeof hex, "%08x", unsigned(h));
  return onnx + "." + hex + ".sm" + std::to_string(p.major * 10 + p.minor) + ".trt" + std::to_string(NV_TENSORRT_MAJOR) + "." +
         std::to_string(NV_TENSORRT_MINOR) + ".engine";
}
}  // namespace

size_t trt_dtype_size(int dtype) {
  switch (nvinfer1::DataType(dtype)) {
    case nvinfer1::DataType::kFLOAT: case nvinfer1::DataType::kINT32: return 4;
    case nvinfer1::DataType::kHALF: case nvinfer1::DataType::kBF16: return 2;
    case nvinfer1::DataType::kINT64: return 8;
    default: return 1;
  }
}

TrtEngine::TrtEngine(const std::string& onnx, const std::map<std::string, std::vector<int64_t>>& shapes,
                     const std::map<std::string, std::vector<int64_t>>& values, size_t workspace_mb)
    : host_values_(values) {
  runtime_.reset(nvinfer1::createInferRuntime(g_log));
  const std::string cache = engine_path(onnx, shapes, values);
  std::vector<char> blob;
  if (fs::exists(cache) && fs::last_write_time(cache) >= fs::last_write_time(onnx)) {
    std::ifstream f(cache, std::ios::binary);
    blob.assign(std::istreambuf_iterator<char>(f), {});
  } else {
    const auto t0 = std::chrono::steady_clock::now();
    std::unique_ptr<nvinfer1::IBuilder> b(nvinfer1::createInferBuilder(g_log));
    std::unique_ptr<nvinfer1::INetworkDefinition> net(b->createNetworkV2(0));
    std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*net, g_log));
    if (!parser->parseFromFile(onnx.c_str(), int(nvinfer1::ILogger::Severity::kWARNING)))
      throw std::runtime_error("TensorRT cannot parse " + onnx);
    std::unique_ptr<nvinfer1::IBuilderConfig> cfg(b->createBuilderConfig());
    cfg->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_mb << 20);
    auto* prof = b->createOptimizationProfile();
    for (int i = 0; i < net->getNbInputs(); ++i) {
      auto* in = net->getInput(i);
      auto d = in->getDimensions();
      const auto it = shapes.find(in->getName());
      for (int k = 0; k < d.nbDims; ++k)
        if (d.d[k] < 0) d.d[k] = (it != shapes.end() && size_t(k) < it->second.size()) ? it->second[size_t(k)] : 1;
      if (it != shapes.end() && it->second.size() == size_t(d.nbDims))
        for (int k = 0; k < d.nbDims; ++k) d.d[k] = it->second[size_t(k)];
      if (in->isShapeTensor()) {
        const auto v = values.find(in->getName());
        if (v == values.end()) throw std::runtime_error(std::string("shape tensor input ") + in->getName() + " needs a value");
        for (auto sel : {nvinfer1::OptProfileSelector::kMIN, nvinfer1::OptProfileSelector::kOPT,
                         nvinfer1::OptProfileSelector::kMAX})
          prof->setShapeValuesV2(in->getName(), sel, v->second.data(), int32_t(v->second.size()));
      }
      for (auto sel : {nvinfer1::OptProfileSelector::kMIN, nvinfer1::OptProfileSelector::kOPT,
                       nvinfer1::OptProfileSelector::kMAX})
        prof->setDimensions(in->getName(), sel, d);
    }
    cfg->addOptimizationProfile(prof);
    std::unique_ptr<nvinfer1::IHostMemory> plan(b->buildSerializedNetwork(*net, *cfg));
    if (!plan) throw std::runtime_error("TensorRT build failed for " + onnx);
    blob.assign(static_cast<const char*>(plan->data()), static_cast<const char*>(plan->data()) + plan->size());
    std::ofstream(cache, std::ios::binary).write(blob.data(), std::streamsize(blob.size()));
    build_s_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
  if (!engine_) throw std::runtime_error("cannot load engine " + cache);
  ctx_.reset(engine_->createExecutionContext());

  for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
    TensorInfo t;
    t.name = engine_->getIOTensorName(i);
    t.input = engine_->getTensorIOMode(t.name.c_str()) == nvinfer1::TensorIOMode::kINPUT;
    t.dtype = int(engine_->getTensorDataType(t.name.c_str()));
    if (t.input) {
      auto d = engine_->getProfileShape(t.name.c_str(), 0, nvinfer1::OptProfileSelector::kOPT);
      ctx_->setInputShape(t.name.c_str(), d);
      if (engine_->isShapeInferenceIO(t.name.c_str())) {  // shape tensor: read from host memory, set before
        auto& v = host_values_.at(t.name);              // output shapes are queried
        t.shape = {int64_t(v.size())};
        t.bytes = v.size() * sizeof(int64_t);
        ctx_->setTensorAddress(t.name.c_str(), v.data());
      }
    }
    io_.push_back(t);
  }
  for (auto& t : io_) {
    if (t.input && engine_->isShapeInferenceIO(t.name.c_str())) continue;
    const auto d = ctx_->getTensorShape(t.name.c_str());
    size_t n = 1;
    for (int k = 0; k < d.nbDims; ++k) { t.shape.push_back(d.d[k]); n *= size_t(std::max<int64_t>(d.d[k], 1)); }
    t.bytes = n * trt_dtype_size(t.dtype);
    check(cudaMalloc(&t.dev, t.bytes), "cudaMalloc");
    owned_.push_back(t.dev);
    ctx_->setTensorAddress(t.name.c_str(), t.dev);
  }
}

TrtEngine::~TrtEngine() {
  ctx_.reset();
  engine_.reset();
  for (void* p : owned_) cudaFree(p);
}

TensorInfo& TrtEngine::tensor(const std::string& name) {
  for (auto& t : io_)
    if (t.name == name) return t;
  throw std::runtime_error("engine has no tensor " + name);
}

void TrtEngine::bind(const std::string& name, void* dev) {
  auto& t = tensor(name);
  t.dev = dev;
  ctx_->setTensorAddress(name.c_str(), dev);
}

void TrtEngine::run(cudaStream_t stream) {
  if (!ctx_->enqueueV3(stream)) throw std::runtime_error("TensorRT enqueue failed");
}

}  // namespace skillarm
