/**
 * @file tensorrt_policy.hpp
 * @brief TensorRT-based policy inference for RL policies
 *
 * Loads TensorRT engines exported from PyTorch → ONNX → TensorRT
 * and performs inference with minimal latency (<2ms on Jetson Orin NX).
 */

#ifndef RL_INFERENCE__TENSORRT_POLICY_HPP_
#define RL_INFERENCE__TENSORRT_POLICY_HPP_

#include <string>
#include <vector>
#include <memory>
#include <chrono>

// Forward declarations for TensorRT types (avoid including heavy headers)
namespace nvinfer1 {
class IRuntime;
class ICudaEngine;
class IExecutionContext;
}

namespace rl_inference
{

/**
 * @brief Policy type enumeration
 */
enum class PolicyType
{
  SEARCH,  // SearchPolicy: 40-dim obs, 4-dim action
  DWELL    // DwellPolicy: 25-dim obs, 6-dim action
};

/**
 * @brief Configuration for TensorRT policy
 */
struct PolicyConfig
{
  std::string engine_path;         // Path to .engine file
  PolicyType policy_type{PolicyType::SEARCH};
  int observation_dim{40};         // Observation space dimension
  int action_dim{4};               // Action space dimension
  bool use_fp16{true};             // Use FP16 inference
  int device_id{0};                // CUDA device ID
  size_t workspace_size{1 << 28};  // 256 MB workspace
};

/**
 * @brief Inference statistics
 */
struct InferenceStats
{
  double last_inference_ms{0.0};
  double avg_inference_ms{0.0};
  double max_inference_ms{0.0};
  size_t inference_count{0};
};

/**
 * @brief TensorRT-based policy for RL inference
 *
 * Loads a TensorRT engine and performs batched inference.
 * Optimized for low-latency execution on Jetson Orin NX.
 */
class TensorRTPolicy
{
public:
  /**
   * @brief Constructor
   * @param config Policy configuration
   */
  explicit TensorRTPolicy(const PolicyConfig & config);

  /**
   * @brief Destructor - releases TensorRT resources
   */
  ~TensorRTPolicy();

  // Non-copyable
  TensorRTPolicy(const TensorRTPolicy &) = delete;
  TensorRTPolicy & operator=(const TensorRTPolicy &) = delete;

  // Movable
  TensorRTPolicy(TensorRTPolicy && other) noexcept;
  TensorRTPolicy & operator=(TensorRTPolicy && other) noexcept;

  /**
   * @brief Load TensorRT engine from file
   * @param engine_path Path to .engine file
   * @return True if successful
   */
  bool loadEngine(const std::string & engine_path);

  /**
   * @brief Check if policy is ready for inference
   */
  bool isReady() const { return engine_ != nullptr && context_ != nullptr; }

  /**
   * @brief Run inference on observation vector
   * @param observation Input observation vector [obs_dim]
   * @param action Output action vector [action_dim]
   * @return True if successful
   */
  bool infer(const std::vector<float> & observation, std::vector<float> & action);

  /**
   * @brief Run batched inference
   * @param observations Input observations [batch_size, obs_dim]
   * @param actions Output actions [batch_size, action_dim]
   * @return True if successful
   */
  bool inferBatch(
    const std::vector<std::vector<float>> & observations,
    std::vector<std::vector<float>> & actions);

  /**
   * @brief Get policy configuration
   */
  const PolicyConfig & getConfig() const { return config_; }

  /**
   * @brief Get inference statistics
   */
  const InferenceStats & getStats() const { return stats_; }

  /**
   * @brief Reset inference statistics
   */
  void resetStats();

  /**
   * @brief Get observation dimension
   */
  int getObservationDim() const { return config_.observation_dim; }

  /**
   * @brief Get action dimension
   */
  int getActionDim() const { return config_.action_dim; }

private:
  /**
   * @brief Initialize CUDA buffers for inference
   */
  bool initializeBuffers();

  /**
   * @brief Free CUDA buffers
   */
  void freeBuffers();

  PolicyConfig config_;
  InferenceStats stats_;

  // TensorRT components (using raw pointers with custom deleter)
  nvinfer1::IRuntime * runtime_{nullptr};
  nvinfer1::ICudaEngine * engine_{nullptr};
  nvinfer1::IExecutionContext * context_{nullptr};

  // CUDA buffers
  void * device_input_{nullptr};
  void * device_output_{nullptr};
  std::vector<float> host_output_;

  // Binding indices
  int input_binding_idx_{-1};
  int output_binding_idx_{-1};

  bool initialized_{false};
};

/**
 * @brief Factory function to create policy from engine file
 * @param engine_path Path to TensorRT engine
 * @param policy_type Type of policy (determines obs/action dims)
 * @return Unique pointer to policy, or nullptr on failure
 */
std::unique_ptr<TensorRTPolicy> createPolicy(
  const std::string & engine_path,
  PolicyType policy_type);

/**
 * @brief Build TensorRT engine from ONNX file
 *
 * Utility function for converting ONNX models to TensorRT engines.
 * Should be run offline, not during deployment.
 *
 * @param onnx_path Path to ONNX model
 * @param engine_path Output path for TensorRT engine
 * @param use_fp16 Enable FP16 optimization
 * @return True if successful
 */
bool buildEngineFromONNX(
  const std::string & onnx_path,
  const std::string & engine_path,
  bool use_fp16 = true);

}  // namespace rl_inference

#endif  // RL_INFERENCE__TENSORRT_POLICY_HPP_
