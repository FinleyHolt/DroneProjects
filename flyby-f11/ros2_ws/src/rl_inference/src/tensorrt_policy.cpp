/**
 * @file tensorrt_policy.cpp
 * @brief Implementation of TensorRT policy inference
 */

#include "rl_inference/tensorrt_policy.hpp"

#include <fstream>
#include <iostream>
#include <cstring>

// TensorRT headers
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

namespace rl_inference
{

// TensorRT logger
class TRTLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    // Suppress info-level messages
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

static TRTLogger g_logger;

TensorRTPolicy::TensorRTPolicy(const PolicyConfig & config)
: config_(config)
{
  host_output_.resize(config_.action_dim);
}

TensorRTPolicy::~TensorRTPolicy()
{
  freeBuffers();

  if (context_) {
    delete context_;
  }
  if (engine_) {
    delete engine_;
  }
  if (runtime_) {
    delete runtime_;
  }
}

TensorRTPolicy::TensorRTPolicy(TensorRTPolicy && other) noexcept
: config_(std::move(other.config_)),
  stats_(std::move(other.stats_)),
  runtime_(other.runtime_),
  engine_(other.engine_),
  context_(other.context_),
  device_input_(other.device_input_),
  device_output_(other.device_output_),
  host_output_(std::move(other.host_output_)),
  input_binding_idx_(other.input_binding_idx_),
  output_binding_idx_(other.output_binding_idx_),
  initialized_(other.initialized_)
{
  other.runtime_ = nullptr;
  other.engine_ = nullptr;
  other.context_ = nullptr;
  other.device_input_ = nullptr;
  other.device_output_ = nullptr;
  other.initialized_ = false;
}

TensorRTPolicy & TensorRTPolicy::operator=(TensorRTPolicy && other) noexcept
{
  if (this != &other) {
    freeBuffers();
    if (context_) {
      delete context_;
    }
    if (engine_) {
      delete engine_;
    }
    if (runtime_) {
      delete runtime_;
    }

    config_ = std::move(other.config_);
    stats_ = std::move(other.stats_);
    runtime_ = other.runtime_;
    engine_ = other.engine_;
    context_ = other.context_;
    device_input_ = other.device_input_;
    device_output_ = other.device_output_;
    host_output_ = std::move(other.host_output_);
    input_binding_idx_ = other.input_binding_idx_;
    output_binding_idx_ = other.output_binding_idx_;
    initialized_ = other.initialized_;

    other.runtime_ = nullptr;
    other.engine_ = nullptr;
    other.context_ = nullptr;
    other.device_input_ = nullptr;
    other.device_output_ = nullptr;
    other.initialized_ = false;
  }
  return *this;
}

bool TensorRTPolicy::loadEngine(const std::string & engine_path)
{
  // Read engine file
  std::ifstream file(engine_path, std::ios::binary);
  if (!file.good()) {
    std::cerr << "Failed to open engine file: " << engine_path << std::endl;
    return false;
  }

  file.seekg(0, std::ios::end);
  size_t file_size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<char> engine_data(file_size);
  file.read(engine_data.data(), file_size);
  file.close();

  // Create runtime
  runtime_ = nvinfer1::createInferRuntime(g_logger);
  if (!runtime_) {
    std::cerr << "Failed to create TensorRT runtime" << std::endl;
    return false;
  }

  // Deserialize engine
  engine_ = runtime_->deserializeCudaEngine(engine_data.data(), file_size);
  if (!engine_) {
    std::cerr << "Failed to deserialize TensorRT engine" << std::endl;
    return false;
  }

  // Create execution context
  context_ = engine_->createExecutionContext();
  if (!context_) {
    std::cerr << "Failed to create TensorRT execution context" << std::endl;
    return false;
  }

  // Get binding indices
  input_binding_idx_ = engine_->getBindingIndex("observation");
  if (input_binding_idx_ < 0) {
    // Try alternative names
    input_binding_idx_ = engine_->getBindingIndex("input");
    if (input_binding_idx_ < 0) {
      input_binding_idx_ = 0;  // Assume first binding is input
    }
  }

  output_binding_idx_ = engine_->getBindingIndex("action");
  if (output_binding_idx_ < 0) {
    output_binding_idx_ = engine_->getBindingIndex("output");
    if (output_binding_idx_ < 0) {
      output_binding_idx_ = 1;  // Assume second binding is output
    }
  }

  // Initialize CUDA buffers
  if (!initializeBuffers()) {
    std::cerr << "Failed to initialize CUDA buffers" << std::endl;
    return false;
  }

  initialized_ = true;
  std::cout << "TensorRT policy loaded successfully from: " << engine_path << std::endl;
  std::cout << "  Input binding: " << input_binding_idx_
            << " (dim: " << config_.observation_dim << ")" << std::endl;
  std::cout << "  Output binding: " << output_binding_idx_
            << " (dim: " << config_.action_dim << ")" << std::endl;

  return true;
}

bool TensorRTPolicy::initializeBuffers()
{
  // Allocate device memory for input
  size_t input_size = config_.observation_dim * sizeof(float);
  if (cudaMalloc(&device_input_, input_size) != cudaSuccess) {
    std::cerr << "Failed to allocate CUDA memory for input" << std::endl;
    return false;
  }

  // Allocate device memory for output
  size_t output_size = config_.action_dim * sizeof(float);
  if (cudaMalloc(&device_output_, output_size) != cudaSuccess) {
    std::cerr << "Failed to allocate CUDA memory for output" << std::endl;
    cudaFree(device_input_);
    device_input_ = nullptr;
    return false;
  }

  return true;
}

void TensorRTPolicy::freeBuffers()
{
  if (device_input_) {
    cudaFree(device_input_);
    device_input_ = nullptr;
  }
  if (device_output_) {
    cudaFree(device_output_);
    device_output_ = nullptr;
  }
}

bool TensorRTPolicy::infer(const std::vector<float> & observation, std::vector<float> & action)
{
  if (!isReady()) {
    std::cerr << "Policy not ready for inference" << std::endl;
    return false;
  }

  if (static_cast<int>(observation.size()) != config_.observation_dim) {
    std::cerr << "Invalid observation size: " << observation.size()
              << " (expected: " << config_.observation_dim << ")" << std::endl;
    return false;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  // Copy input to device
  cudaMemcpy(device_input_, observation.data(),
    config_.observation_dim * sizeof(float), cudaMemcpyHostToDevice);

  // Set up bindings
  void * bindings[2];
  bindings[input_binding_idx_] = device_input_;
  bindings[output_binding_idx_] = device_output_;

  // Run inference
  bool success = context_->executeV2(bindings);
  if (!success) {
    std::cerr << "TensorRT inference failed" << std::endl;
    return false;
  }

  // Copy output back to host
  cudaMemcpy(host_output_.data(), device_output_,
    config_.action_dim * sizeof(float), cudaMemcpyDeviceToHost);

  auto end_time = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  // Update statistics
  stats_.last_inference_ms = elapsed_ms;
  stats_.inference_count++;
  stats_.avg_inference_ms = (stats_.avg_inference_ms * (stats_.inference_count - 1) + elapsed_ms) /
    stats_.inference_count;
  stats_.max_inference_ms = std::max(stats_.max_inference_ms, elapsed_ms);

  // Return action
  action = host_output_;
  return true;
}

bool TensorRTPolicy::inferBatch(
  const std::vector<std::vector<float>> & observations,
  std::vector<std::vector<float>> & actions)
{
  actions.clear();
  actions.reserve(observations.size());

  for (const auto & obs : observations) {
    std::vector<float> action;
    if (!infer(obs, action)) {
      return false;
    }
    actions.push_back(std::move(action));
  }

  return true;
}

void TensorRTPolicy::resetStats()
{
  stats_ = InferenceStats();
}

std::unique_ptr<TensorRTPolicy> createPolicy(
  const std::string & engine_path,
  PolicyType policy_type)
{
  PolicyConfig config;
  config.engine_path = engine_path;
  config.policy_type = policy_type;

  switch (policy_type) {
    case PolicyType::SEARCH:
      config.observation_dim = 40;
      config.action_dim = 4;
      break;
    case PolicyType::DWELL:
      config.observation_dim = 25;
      config.action_dim = 6;
      break;
  }

  auto policy = std::make_unique<TensorRTPolicy>(config);
  if (!policy->loadEngine(engine_path)) {
    return nullptr;
  }

  return policy;
}

bool buildEngineFromONNX(
  const std::string & onnx_path,
  const std::string & engine_path,
  bool use_fp16)
{
  // Create builder
  auto builder = nvinfer1::createInferBuilder(g_logger);
  if (!builder) {
    std::cerr << "Failed to create TensorRT builder" << std::endl;
    return false;
  }

  // Create network with explicit batch
  const auto explicitBatch = 1U << static_cast<uint32_t>(
    nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = builder->createNetworkV2(explicitBatch);
  if (!network) {
    std::cerr << "Failed to create TensorRT network" << std::endl;
    delete builder;
    return false;
  }

  // Create ONNX parser
  auto parser = nvonnxparser::createParser(*network, g_logger);
  if (!parser) {
    std::cerr << "Failed to create ONNX parser" << std::endl;
    delete network;
    delete builder;
    return false;
  }

  // Parse ONNX file
  if (!parser->parseFromFile(onnx_path.c_str(),
      static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
  {
    std::cerr << "Failed to parse ONNX file: " << onnx_path << std::endl;
    for (int i = 0; i < parser->getNbErrors(); ++i) {
      std::cerr << "  " << parser->getError(i)->desc() << std::endl;
    }
    delete parser;
    delete network;
    delete builder;
    return false;
  }

  // Create builder config
  auto config = builder->createBuilderConfig();
  if (!config) {
    std::cerr << "Failed to create builder config" << std::endl;
    delete parser;
    delete network;
    delete builder;
    return false;
  }

  // Set workspace size
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1 << 28);  // 256 MB

  // Enable FP16 if requested and supported
  if (use_fp16 && builder->platformHasFastFp16()) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
    std::cout << "FP16 mode enabled" << std::endl;
  }

  // Build engine
  std::cout << "Building TensorRT engine (this may take a while)..." << std::endl;
  auto serialized_engine = builder->buildSerializedNetwork(*network, *config);
  if (!serialized_engine) {
    std::cerr << "Failed to build TensorRT engine" << std::endl;
    delete config;
    delete parser;
    delete network;
    delete builder;
    return false;
  }

  // Write engine to file
  std::ofstream file(engine_path, std::ios::binary);
  if (!file.good()) {
    std::cerr << "Failed to open output file: " << engine_path << std::endl;
    delete serialized_engine;
    delete config;
    delete parser;
    delete network;
    delete builder;
    return false;
  }

  file.write(static_cast<const char *>(serialized_engine->data()), serialized_engine->size());
  file.close();

  std::cout << "TensorRT engine saved to: " << engine_path << std::endl;

  // Cleanup
  delete serialized_engine;
  delete config;
  delete parser;
  delete network;
  delete builder;

  return true;
}

}  // namespace rl_inference
