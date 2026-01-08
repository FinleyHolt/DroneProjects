#include "behavior_trees/bt_nodes/rl_action_nodes.hpp"

#include <cmath>

namespace behavior_trees
{

// =============================================================================
// RLPolicyActionBase
// =============================================================================

RLPolicyActionBase::RLPolicyActionBase(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10);

  context_pub_ = node_->create_publisher<flyby_msgs::msg::RLPolicyContext>(
    "rl_policy_context", 10);

  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "uav_state", 10,
    std::bind(&RLPolicyActionBase::stateCallback, this, std::placeholders::_1));

  // Subscribe to policy actions from rl_inference node
  policy_action_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "rl_policy_action", 10,
    std::bind(&RLPolicyActionBase::policyActionCallback, this, std::placeholders::_1));

  // Get parameters
  getInput("max_velocity", max_velocity_);
  getInput("max_yaw_rate", max_yaw_rate_);

  int timeout_ms = 200;
  getInput("action_timeout_ms", timeout_ms);
  action_timeout_ = std::chrono::milliseconds(timeout_ms);

  int max_time_ms = 300000;
  getInput("max_execution_time_ms", max_time_ms);
  max_execution_time_ = std::chrono::milliseconds(max_time_ms);
}

BT::PortsList RLPolicyActionBase::providedBasePorts()
{
  return {
    BT::InputPort<double>("max_velocity", 5.0, "Maximum velocity (m/s)"),
    BT::InputPort<double>("max_yaw_rate", 0.5, "Maximum yaw rate (rad/s)"),
    BT::InputPort<int>("action_timeout_ms", 200, "Action timeout (ms)"),
    BT::InputPort<int>("max_execution_time_ms", 300000, "Max execution time (ms)"),
    BT::OutputPort<std::string>("termination_reason", "Reason for termination"),
  };
}

void RLPolicyActionBase::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  current_state_ = *msg;
  state_received_ = true;
}

void RLPolicyActionBase::policyActionCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_policy_action_ = *msg;
  action_received_ = true;
  last_action_time_ = std::chrono::steady_clock::now();
}

void RLPolicyActionBase::publishContext()
{
  auto context = buildContext();
  context.header.stamp = node_->now();
  context_pub_->publish(context);
}

void RLPolicyActionBase::publishVelocityCommand(const geometry_msgs::msg::TwistStamped & cmd)
{
  // Clamp velocities to safety limits
  geometry_msgs::msg::TwistStamped safe_cmd = cmd;
  safe_cmd.header.stamp = node_->now();

  double linear_speed = std::sqrt(
    cmd.twist.linear.x * cmd.twist.linear.x +
    cmd.twist.linear.y * cmd.twist.linear.y);

  if (linear_speed > max_velocity_) {
    double scale = max_velocity_ / linear_speed;
    safe_cmd.twist.linear.x *= scale;
    safe_cmd.twist.linear.y *= scale;
  }

  safe_cmd.twist.linear.z = std::clamp(cmd.twist.linear.z, -2.0, 2.0);
  safe_cmd.twist.angular.z = std::clamp(cmd.twist.angular.z, -max_yaw_rate_, max_yaw_rate_);

  cmd_vel_pub_->publish(safe_cmd);
}

bool RLPolicyActionBase::isPolicyTimedOut()
{
  auto now = std::chrono::steady_clock::now();

  // Check if we've received an action recently
  if (action_received_) {
    auto elapsed_since_action = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_action_time_);

    if (elapsed_since_action > action_timeout_) {
      RCLCPP_WARN(node_->get_logger(),
        "Policy action timeout: %ld ms since last action",
        elapsed_since_action.count());
      return true;
    }
  }

  // Check total execution time
  auto total_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - start_time_);

  if (total_elapsed > max_execution_time_) {
    RCLCPP_WARN(node_->get_logger(),
      "Policy max execution time exceeded: %ld ms",
      total_elapsed.count());
    return true;
  }

  return false;
}

// =============================================================================
// SearchPolicyAction
// =============================================================================

SearchPolicyAction::SearchPolicyAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: RLPolicyActionBase(name, config, node)
{
  policy_name_ = "search";

  ao_sub_ = node_->create_subscription<flyby_msgs::msg::AreaOfOperations>(
    "area_of_operations", 10,
    std::bind(&SearchPolicyAction::aoCallback, this, std::placeholders::_1));

  getInput("coverage_target", coverage_target_);
}

BT::PortsList SearchPolicyAction::providedPorts()
{
  auto ports = RLPolicyActionBase::providedBasePorts();
  ports.insert(BT::InputPort<double>("coverage_target", 0.85, "Coverage target percentage"));
  ports.insert(BT::OutputPort<double>("coverage_achieved", "Final coverage percentage"));
  ports.insert(BT::OutputPort<int>("targets_detected", "Number of targets detected"));
  return ports;
}

BT::NodeStatus SearchPolicyAction::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  last_action_time_ = start_time_;
  state_received_ = false;
  action_received_ = false;
  current_coverage_ = 0.0;
  targets_detected_ = 0;

  RCLCPP_INFO(node_->get_logger(),
    "SearchPolicyAction: Starting search policy with %.0f%% coverage target",
    coverage_target_ * 100.0);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchPolicyAction::onRunning()
{
  // Wait for state
  if (!state_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "SearchPolicyAction: Waiting for UAV state...");
    return BT::NodeStatus::RUNNING;
  }

  // Check termination conditions
  if (checkTerminationConditions()) {
    setOutput("coverage_achieved", current_coverage_);
    setOutput("targets_detected", targets_detected_);
    return BT::NodeStatus::SUCCESS;
  }

  // Check for policy timeout (triggers fallback)
  if (isPolicyTimedOut()) {
    setOutput("termination_reason", "policy_timeout");
    return BT::NodeStatus::FAILURE;
  }

  // Publish context for RL inference node
  publishContext();

  // If we have a policy action, execute it
  if (action_received_) {
    publishVelocityCommand(latest_policy_action_);
  }

  return BT::NodeStatus::RUNNING;
}

void SearchPolicyAction::onHalted()
{
  // Stop the vehicle
  geometry_msgs::msg::TwistStamped stop_cmd;
  stop_cmd.header.stamp = node_->now();
  cmd_vel_pub_->publish(stop_cmd);

  // Publish inactive context
  auto context = buildContext();
  context.is_active = false;
  context_pub_->publish(context);

  RCLCPP_INFO(node_->get_logger(),
    "SearchPolicyAction: Halted with %.1f%% coverage, %d targets",
    current_coverage_ * 100.0, targets_detected_);
}

flyby_msgs::msg::RLPolicyContext SearchPolicyAction::buildContext()
{
  flyby_msgs::msg::RLPolicyContext context;
  context.header.stamp = node_->now();
  context.policy_type = "search";
  context.is_active = true;

  // UAV state
  context.current_pose.header = current_state_.header;
  context.current_pose.pose.position = current_state_.position;
  context.current_pose.pose.orientation = current_state_.orientation;

  geometry_msgs::msg::TwistStamped vel;
  vel.twist.linear = current_state_.velocity;
  context.current_velocity = vel;

  context.battery_pct = current_state_.battery_percentage;

  // Search context
  context.search_area = search_area_;
  context.coverage_pct = current_coverage_;
  context.targets_detected = targets_detected_;

  // Constraints
  context.max_velocity_mps = max_velocity_;

  return context;
}

bool SearchPolicyAction::checkTerminationConditions()
{
  // Coverage target reached
  if (current_coverage_ >= coverage_target_) {
    setOutput("termination_reason", "coverage_target_reached");
    RCLCPP_INFO(node_->get_logger(),
      "SearchPolicyAction: Coverage target reached (%.1f%%)",
      current_coverage_ * 100.0);
    return true;
  }

  // Battery check (would be done via ontology in production)
  if (current_state_.battery_percentage < 25.0) {
    setOutput("termination_reason", "battery_reserve");
    RCLCPP_WARN(node_->get_logger(),
      "SearchPolicyAction: Battery reserve reached (%.1f%%)",
      current_state_.battery_percentage);
    return true;
  }

  return false;
}

void SearchPolicyAction::aoCallback(const flyby_msgs::msg::AreaOfOperations::SharedPtr msg)
{
  search_area_ = *msg;
  current_coverage_ = msg->current_coverage_pct / 100.0;  // Convert to [0, 1]
}

// =============================================================================
// DwellPolicyAction
// =============================================================================

DwellPolicyAction::DwellPolicyAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: RLPolicyActionBase(name, config, node)
{
  policy_name_ = "dwell";

  target_sub_ = node_->create_subscription<flyby_msgs::msg::TargetOfInterest>(
    "current_target", 10,
    std::bind(&DwellPolicyAction::targetCallback, this, std::placeholders::_1));

  gimbal_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "gimbal_command", 10);

  getInput("required_dwell_time", required_dwell_time_);
  getInput("track_lost_threshold", track_lost_threshold_);

  int max_lost_ms = 5000;
  getInput("max_track_lost_duration_ms", max_lost_ms);
  max_track_lost_duration_ = std::chrono::milliseconds(max_lost_ms);
}

BT::PortsList DwellPolicyAction::providedPorts()
{
  auto ports = RLPolicyActionBase::providedBasePorts();
  ports.insert(BT::InputPort<double>("required_dwell_time", 10.0, "Required dwell time (s)"));
  ports.insert(BT::InputPort<double>("track_lost_threshold", 0.3, "Track lost confidence threshold"));
  ports.insert(BT::InputPort<int>("max_track_lost_duration_ms", 5000, "Max track lost duration (ms)"));
  ports.insert(BT::OutputPort<double>("dwell_time_achieved", "Actual dwell time (s)"));
  ports.insert(BT::OutputPort<std::string>("target_id", "ID of tracked target"));
  return ports;
}

BT::NodeStatus DwellPolicyAction::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  last_action_time_ = start_time_;
  track_lost_time_ = start_time_;
  state_received_ = false;
  action_received_ = false;
  accumulated_dwell_time_ = 0.0;
  track_confidence_ = 0.0;

  RCLCPP_INFO(node_->get_logger(),
    "DwellPolicyAction: Starting dwell policy with %.1fs required",
    required_dwell_time_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DwellPolicyAction::onRunning()
{
  // Wait for state
  if (!state_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "DwellPolicyAction: Waiting for UAV state...");
    return BT::NodeStatus::RUNNING;
  }

  // Check termination conditions
  if (checkTerminationConditions()) {
    setOutput("dwell_time_achieved", accumulated_dwell_time_);
    setOutput("target_id", current_target_.target_id);
    return BT::NodeStatus::SUCCESS;
  }

  // Check for policy timeout (triggers fallback)
  if (isPolicyTimedOut()) {
    setOutput("termination_reason", "policy_timeout");
    setOutput("dwell_time_achieved", accumulated_dwell_time_);
    return BT::NodeStatus::FAILURE;
  }

  // Publish context for RL inference node
  publishContext();

  // If we have a policy action, execute it
  if (action_received_) {
    // Extract UAV velocity (first 4 dims of 6-dim action)
    publishVelocityCommand(latest_policy_action_);

    // Extract gimbal command (last 2 dims: pan_rate, tilt_rate)
    // Note: In actual implementation, the rl_inference node would parse
    // the 6-dim action and publish separate gimbal commands
  }

  // Accumulate dwell time if tracking
  if (track_confidence_ > track_lost_threshold_) {
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_action_time_);
    accumulated_dwell_time_ += dt.count() / 1000.0;
  }

  return BT::NodeStatus::RUNNING;
}

void DwellPolicyAction::onHalted()
{
  // Stop the vehicle
  geometry_msgs::msg::TwistStamped stop_cmd;
  stop_cmd.header.stamp = node_->now();
  cmd_vel_pub_->publish(stop_cmd);

  // Zero gimbal rates
  publishGimbalCommand(0.0, 0.0);

  // Publish inactive context
  auto context = buildContext();
  context.is_active = false;
  context_pub_->publish(context);

  RCLCPP_INFO(node_->get_logger(),
    "DwellPolicyAction: Halted with %.1fs dwell time",
    accumulated_dwell_time_);
}

flyby_msgs::msg::RLPolicyContext DwellPolicyAction::buildContext()
{
  flyby_msgs::msg::RLPolicyContext context;
  context.header.stamp = node_->now();
  context.policy_type = "dwell";
  context.is_active = true;

  // UAV state
  context.current_pose.header = current_state_.header;
  context.current_pose.pose.position = current_state_.position;
  context.current_pose.pose.orientation = current_state_.orientation;

  geometry_msgs::msg::TwistStamped vel;
  vel.twist.linear = current_state_.velocity;
  context.current_velocity = vel;

  context.battery_pct = current_state_.battery_percentage;

  // Gimbal state
  context.gimbal_pan_rad = gimbal_pan_;
  context.gimbal_tilt_rad = gimbal_tilt_;
  context.gimbal_pan_rate_limit = gimbal_pan_rate_limit_;
  context.gimbal_tilt_rate_limit = gimbal_tilt_rate_limit_;

  // Dwell context
  context.current_target = current_target_;
  context.track_confidence = track_confidence_;
  context.dwell_time_remaining_sec = std::max(0.0, required_dwell_time_ - accumulated_dwell_time_);

  // Constraints
  context.max_velocity_mps = max_velocity_;

  return context;
}

bool DwellPolicyAction::checkTerminationConditions()
{
  // Dwell time requirement met
  if (accumulated_dwell_time_ >= required_dwell_time_) {
    setOutput("termination_reason", "dwell_complete");
    RCLCPP_INFO(node_->get_logger(),
      "DwellPolicyAction: Dwell time requirement met (%.1fs)",
      accumulated_dwell_time_);
    return true;
  }

  // Track lost for too long
  if (track_confidence_ < track_lost_threshold_) {
    auto now = std::chrono::steady_clock::now();
    auto lost_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - track_lost_time_);

    if (lost_duration > max_track_lost_duration_) {
      setOutput("termination_reason", "target_lost");
      RCLCPP_WARN(node_->get_logger(),
        "DwellPolicyAction: Target lost for %ld ms",
        lost_duration.count());
      return true;
    }
  } else {
    // Reset lost timer when tracking
    track_lost_time_ = std::chrono::steady_clock::now();
  }

  // Battery check
  if (current_state_.battery_percentage < 25.0) {
    setOutput("termination_reason", "battery_reserve");
    RCLCPP_WARN(node_->get_logger(),
      "DwellPolicyAction: Battery reserve reached (%.1f%%)",
      current_state_.battery_percentage);
    return true;
  }

  return false;
}

void DwellPolicyAction::targetCallback(const flyby_msgs::msg::TargetOfInterest::SharedPtr msg)
{
  current_target_ = *msg;
  track_confidence_ = msg->confidence;
  accumulated_dwell_time_ = msg->accumulated_dwell_time_sec;
}

void DwellPolicyAction::publishGimbalCommand(double pan_rate, double tilt_rate)
{
  geometry_msgs::msg::Vector3Stamped cmd;
  cmd.header.stamp = node_->now();
  cmd.vector.x = std::clamp(pan_rate, -gimbal_pan_rate_limit_, gimbal_pan_rate_limit_);
  cmd.vector.y = std::clamp(tilt_rate, -gimbal_tilt_rate_limit_, gimbal_tilt_rate_limit_);
  cmd.vector.z = 0.0;  // Reserved
  gimbal_cmd_pub_->publish(cmd);
}

// =============================================================================
// Registration
// =============================================================================

void registerRLActionNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node)
{
  factory.registerBuilder<SearchPolicyAction>(
    "SearchPolicy",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SearchPolicyAction>(name, config, node);
    });

  factory.registerBuilder<DwellPolicyAction>(
    "DwellPolicy",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<DwellPolicyAction>(name, config, node);
    });
}

}  // namespace behavior_trees
