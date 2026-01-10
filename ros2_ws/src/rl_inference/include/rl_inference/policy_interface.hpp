/**
 * @file policy_interface.hpp
 * @brief ROS2 interface for RL policy inference
 *
 * Provides ROS2 service/topic interfaces for the BT nodes to query
 * RL policies. Handles observation construction and action publishing.
 */

#ifndef RL_INFERENCE__POLICY_INTERFACE_HPP_
#define RL_INFERENCE__POLICY_INTERFACE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "flyby_msgs/msg/uav_state.hpp"
#include "flyby_msgs/msg/rl_policy_context.hpp"

#include "rl_inference/tensorrt_policy.hpp"

namespace rl_inference
{

/**
 * @brief ROS2 interface for policy inference
 *
 * Subscribes to UAV state and policy context, runs inference,
 * and publishes action commands.
 */
class PolicyInterface
{
public:
  using StateCallback = std::function<void(const flyby_msgs::msg::UAVState::SharedPtr)>;
  using ContextCallback = std::function<void(const flyby_msgs::msg::RLPolicyContext::SharedPtr)>;
  using ActionCallback = std::function<void(const geometry_msgs::msg::TwistStamped &)>;

  /**
   * @brief Constructor
   * @param node ROS2 node for subscriptions/publications
   * @param policy TensorRT policy for inference
   * @param policy_name Name for topic namespacing
   */
  PolicyInterface(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<TensorRTPolicy> policy,
    const std::string & policy_name);

  /**
   * @brief Check if policy is active and producing actions
   */
  bool isActive() const { return is_active_; }

  /**
   * @brief Activate policy inference
   */
  void activate();

  /**
   * @brief Deactivate policy inference
   */
  void deactivate();

  /**
   * @brief Get last computed action
   */
  const geometry_msgs::msg::TwistStamped & getLastAction() const { return last_action_; }

  /**
   * @brief Get inference latency in milliseconds
   */
  double getLatencyMs() const { return policy_->getStats().last_inference_ms; }

  /**
   * @brief Set callback for action updates
   */
  void setActionCallback(ActionCallback callback) { action_callback_ = callback; }

private:
  /**
   * @brief Construct observation vector from state and context
   */
  std::vector<float> constructObservation(
    const flyby_msgs::msg::UAVState & state,
    const flyby_msgs::msg::RLPolicyContext & context);

  /**
   * @brief Convert action vector to TwistStamped message
   */
  geometry_msgs::msg::TwistStamped actionToTwist(
    const std::vector<float> & action,
    const std_msgs::msg::Header & header);

  /**
   * @brief State callback
   */
  void onState(const flyby_msgs::msg::UAVState::SharedPtr msg);

  /**
   * @brief Context callback
   */
  void onContext(const flyby_msgs::msg::RLPolicyContext::SharedPtr msg);

  /**
   * @brief Run inference and publish action
   */
  void runInference();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<TensorRTPolicy> policy_;
  std::string policy_name_;

  // Subscriptions
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp::Subscription<flyby_msgs::msg::RLPolicyContext>::SharedPtr context_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr action_pub_;

  // State
  flyby_msgs::msg::UAVState::SharedPtr last_state_;
  flyby_msgs::msg::RLPolicyContext::SharedPtr last_context_;
  geometry_msgs::msg::TwistStamped last_action_;
  bool is_active_{false};

  // Callback
  ActionCallback action_callback_;
};

/**
 * @brief Search policy observation builder
 *
 * Constructs 40-dim observation vector for SearchPolicy:
 * - Drone state (12): pos, vel, rot_matrix, ang_vel
 * - Coverage features (15): coverage %, frontier centroids, uncovered ratio
 * - Mission state (3): battery, time, coverage
 * - Context (10): home direction, targets, exploration direction
 */
class SearchObservationBuilder
{
public:
  static constexpr int OBS_DIM = 40;

  /**
   * @brief Build observation from state and context
   */
  static std::vector<float> build(
    const flyby_msgs::msg::UAVState & state,
    const flyby_msgs::msg::RLPolicyContext & context);
};

/**
 * @brief Dwell policy observation builder
 *
 * Constructs 25-dim observation vector for DwellPolicy:
 * - Target relative state (10): relative pos, vel, image pos
 * - Image features (8): bounding box, center offset, quality
 * - UAV state (5): vel, alt, yaw rate
 * - Gimbal state (2): pan, tilt
 */
class DwellObservationBuilder
{
public:
  static constexpr int OBS_DIM = 25;

  /**
   * @brief Build observation from state and context
   */
  static std::vector<float> build(
    const flyby_msgs::msg::UAVState & state,
    const flyby_msgs::msg::RLPolicyContext & context);
};

}  // namespace rl_inference

#endif  // RL_INFERENCE__POLICY_INTERFACE_HPP_
