#ifndef BEHAVIOR_TREES__BT_NODES__RL_ACTION_NODES_HPP_
#define BEHAVIOR_TREES__BT_NODES__RL_ACTION_NODES_HPP_

#include <string>
#include <memory>
#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include "flyby_msgs/msg/uav_state.hpp"
#include "flyby_msgs/msg/rl_policy_context.hpp"
#include "flyby_msgs/msg/target_of_interest.hpp"
#include "flyby_msgs/msg/area_of_operations.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace behavior_trees
{

/**
 * @brief Base class for RL policy action nodes
 *
 * Provides common infrastructure for executing RL policies:
 * - State subscription and observation construction
 * - Velocity command publishing
 * - Termination condition monitoring
 * - Fallback triggering on policy failure
 */
class RLPolicyActionBase : public BT::StatefulActionNode
{
public:
  RLPolicyActionBase(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  virtual ~RLPolicyActionBase() = default;

  static BT::PortsList providedBasePorts();

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<flyby_msgs::msg::RLPolicyContext>::SharedPtr context_pub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr policy_action_sub_;

  // State
  flyby_msgs::msg::UAVState current_state_;
  geometry_msgs::msg::TwistStamped latest_policy_action_;
  std::atomic<bool> state_received_{false};
  std::atomic<bool> action_received_{false};

  // Timing
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_action_time_;
  std::chrono::milliseconds action_timeout_{200};  // Fallback if no action for 200ms
  std::chrono::milliseconds max_execution_time_{300000};  // 5 minutes max

  // Policy parameters
  std::string policy_name_;
  double max_velocity_{5.0};
  double max_yaw_rate_{0.5};

  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);
  void policyActionCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  virtual flyby_msgs::msg::RLPolicyContext buildContext() = 0;
  virtual bool checkTerminationConditions() = 0;
  void publishContext();
  void publishVelocityCommand(const geometry_msgs::msg::TwistStamped & cmd);
  bool isPolicyTimedOut();
};

/**
 * @brief Search Policy action node
 *
 * Executes the trained SearchPolicy for area coverage.
 * Constructs 40-dim observation and publishes 4-dim action (velocity + heading).
 *
 * Terminates when:
 * - Coverage target reached
 * - Battery reserve threshold hit
 * - Policy timeout
 * - External preemption
 */
class SearchPolicyAction : public RLPolicyActionBase
{
public:
  SearchPolicyAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  flyby_msgs::msg::RLPolicyContext buildContext() override;
  bool checkTerminationConditions() override;

private:
  // Search-specific state
  flyby_msgs::msg::AreaOfOperations search_area_;
  double coverage_target_{0.85};
  double current_coverage_{0.0};
  int targets_detected_{0};

  rclcpp::Subscription<flyby_msgs::msg::AreaOfOperations>::SharedPtr ao_sub_;

  void aoCallback(const flyby_msgs::msg::AreaOfOperations::SharedPtr msg);
};

/**
 * @brief Dwell Policy action node
 *
 * Executes the trained DwellPolicy for target tracking with 2-axis gimbal.
 * Constructs 25-dim observation and publishes 6-dim action (velocity + gimbal).
 *
 * Terminates when:
 * - Dwell time requirement met
 * - Target lost for extended period
 * - Battery reserve threshold hit
 * - Policy timeout
 * - External preemption
 */
class DwellPolicyAction : public RLPolicyActionBase
{
public:
  DwellPolicyAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  flyby_msgs::msg::RLPolicyContext buildContext() override;
  bool checkTerminationConditions() override;

private:
  // Dwell-specific state
  flyby_msgs::msg::TargetOfInterest current_target_;
  double required_dwell_time_{10.0};  // seconds
  double accumulated_dwell_time_{0.0};
  double track_confidence_{0.0};
  double track_lost_threshold_{0.3};
  std::chrono::steady_clock::time_point track_lost_time_;
  std::chrono::milliseconds max_track_lost_duration_{5000};  // 5 seconds

  // Gimbal state
  double gimbal_pan_{0.0};
  double gimbal_tilt_{0.0};
  double gimbal_pan_rate_limit_{0.785};   // 45 deg/s
  double gimbal_tilt_rate_limit_{0.785};  // 45 deg/s

  rclcpp::Subscription<flyby_msgs::msg::TargetOfInterest>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_cmd_pub_;

  void targetCallback(const flyby_msgs::msg::TargetOfInterest::SharedPtr msg);
  void publishGimbalCommand(double pan_rate, double tilt_rate);
};

/**
 * @brief Register all RL action nodes with the BT factory
 */
void registerRLActionNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node);

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES__BT_NODES__RL_ACTION_NODES_HPP_
