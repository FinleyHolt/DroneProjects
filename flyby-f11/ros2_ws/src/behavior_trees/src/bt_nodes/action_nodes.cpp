#include "behavior_trees/bt_nodes/action_nodes.hpp"

#include <cmath>

namespace behavior_trees
{

// =============================================================================
// TransitAction
// =============================================================================

TransitAction::TransitAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10);

  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "uav_state", 10,
    std::bind(&TransitAction::stateCallback, this, std::placeholders::_1));

  // Get parameters from ports
  getInput("lookahead_distance", lookahead_distance_);
  getInput("goal_tolerance", goal_tolerance_);
  getInput("max_linear_velocity", max_linear_velocity_);
  getInput("max_yaw_rate", max_yaw_rate_);
}

BT::PortsList TransitAction::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
    BT::InputPort<double>("lookahead_distance", 5.0, "Lookahead distance (m)"),
    BT::InputPort<double>("goal_tolerance", 2.0, "Goal tolerance (m)"),
    BT::InputPort<double>("max_linear_velocity", 5.0, "Max linear velocity (m/s)"),
    BT::InputPort<double>("max_yaw_rate", 0.5, "Max yaw rate (rad/s)"),
    BT::OutputPort<bool>("goal_reached", "True when goal is reached"),
  };
}

BT::NodeStatus TransitAction::onStart()
{
  // Get path from blackboard
  if (!getInput("path", current_path_)) {
    RCLCPP_ERROR(node_->get_logger(), "TransitAction: No path provided");
    return BT::NodeStatus::FAILURE;
  }

  if (current_path_.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "TransitAction: Empty path");
    return BT::NodeStatus::SUCCESS;  // Nothing to do
  }

  current_waypoint_idx_ = 0;
  pose_received_ = false;

  RCLCPP_INFO(node_->get_logger(),
    "TransitAction: Starting transit with %zu waypoints",
    current_path_.poses.size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TransitAction::onRunning()
{
  if (!pose_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "TransitAction: Waiting for pose...");
    return BT::NodeStatus::RUNNING;
  }

  // Check if we've reached the goal
  if (isGoalReached()) {
    RCLCPP_INFO(node_->get_logger(), "TransitAction: Goal reached");
    setOutput("goal_reached", true);

    // Stop the vehicle
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = node_->now();
    cmd_vel_pub_->publish(stop_cmd);

    return BT::NodeStatus::SUCCESS;
  }

  // Compute and publish pure pursuit command
  auto cmd = computePurePursuitCommand();
  cmd_vel_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;
}

void TransitAction::onHalted()
{
  // Stop the vehicle
  geometry_msgs::msg::TwistStamped stop_cmd;
  stop_cmd.header.stamp = node_->now();
  cmd_vel_pub_->publish(stop_cmd);

  RCLCPP_INFO(node_->get_logger(), "TransitAction: Halted");
}

void TransitAction::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose.position.x = msg->position.x;
  current_pose_.pose.position.y = msg->position.y;
  current_pose_.pose.position.z = msg->position.z;
  current_pose_.pose.orientation = msg->orientation;
  pose_received_ = true;
}

geometry_msgs::msg::TwistStamped TransitAction::computePurePursuitCommand()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();

  if (current_path_.poses.empty()) {
    return cmd;
  }

  // Find the closest waypoint
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = current_waypoint_idx_;

  for (size_t i = current_waypoint_idx_; i < current_path_.poses.size(); ++i) {
    double dx = current_path_.poses[i].pose.position.x - current_pose_.pose.position.x;
    double dy = current_path_.poses[i].pose.position.y - current_pose_.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // Find lookahead point
  size_t lookahead_idx = closest_idx;
  double accumulated_dist = 0.0;

  for (size_t i = closest_idx; i < current_path_.poses.size() - 1; ++i) {
    double dx = current_path_.poses[i + 1].pose.position.x -
                current_path_.poses[i].pose.position.x;
    double dy = current_path_.poses[i + 1].pose.position.y -
                current_path_.poses[i].pose.position.y;
    accumulated_dist += std::sqrt(dx * dx + dy * dy);

    if (accumulated_dist >= lookahead_distance_) {
      lookahead_idx = i + 1;
      break;
    }
    lookahead_idx = i + 1;
  }

  // Get lookahead point
  const auto & lookahead = current_path_.poses[lookahead_idx].pose.position;

  // Compute heading to lookahead point
  double dx = lookahead.x - current_pose_.pose.position.x;
  double dy = lookahead.y - current_pose_.pose.position.y;
  double dz = lookahead.z - current_pose_.pose.position.z;

  double target_yaw = std::atan2(dy, dx);

  // Get current yaw from quaternion
  double siny_cosp = 2.0 * (current_pose_.pose.orientation.w * current_pose_.pose.orientation.z +
                            current_pose_.pose.orientation.x * current_pose_.pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (current_pose_.pose.orientation.y * current_pose_.pose.orientation.y +
                                   current_pose_.pose.orientation.z * current_pose_.pose.orientation.z);
  double current_yaw = std::atan2(siny_cosp, cosy_cosp);

  // Yaw error (normalized to [-pi, pi])
  double yaw_error = target_yaw - current_yaw;
  while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

  // Proportional yaw rate control
  double yaw_rate = std::clamp(yaw_error * 1.0, -max_yaw_rate_, max_yaw_rate_);

  // Linear velocity (reduced when turning)
  double turn_factor = 1.0 - std::min(std::abs(yaw_error) / M_PI, 1.0) * 0.5;
  double horizontal_dist = std::sqrt(dx * dx + dy * dy);
  double speed = std::min(horizontal_dist * 0.5, max_linear_velocity_) * turn_factor;

  // Convert to body frame velocity
  cmd.twist.linear.x = speed * std::cos(yaw_error);
  cmd.twist.linear.y = speed * std::sin(yaw_error);
  cmd.twist.linear.z = std::clamp(dz * 0.5, -2.0, 2.0);  // PID-like altitude control
  cmd.twist.angular.z = yaw_rate;

  // Update current waypoint index
  current_waypoint_idx_ = closest_idx;

  return cmd;
}

bool TransitAction::isGoalReached()
{
  if (current_path_.poses.empty()) return true;

  const auto & goal = current_path_.poses.back().pose.position;
  double dx = goal.x - current_pose_.pose.position.x;
  double dy = goal.y - current_pose_.pose.position.y;
  double dz = goal.z - current_pose_.pose.position.z;

  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  return dist < goal_tolerance_;
}

// =============================================================================
// LoiterAction
// =============================================================================

LoiterAction::LoiterAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10);

  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "uav_state", 10,
    std::bind(&LoiterAction::stateCallback, this, std::placeholders::_1));

  getInput("altitude", target_altitude_);
}

BT::PortsList LoiterAction::providedPorts()
{
  return {
    BT::InputPort<double>("altitude", 50.0, "Loiter altitude (m AGL)"),
  };
}

BT::NodeStatus LoiterAction::onStart()
{
  position_initialized_ = false;
  RCLCPP_INFO(node_->get_logger(), "LoiterAction: Starting loiter at %.1f m", target_altitude_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LoiterAction::onRunning()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();

  if (!position_initialized_) {
    // First time - capture current position
    if (!loiter_position_.header.frame_id.empty()) {
      position_initialized_ = true;
    }
  }

  if (position_initialized_) {
    // Position hold with altitude control
    double dx = loiter_position_.pose.position.x -
                loiter_position_.pose.position.x;  // Always 0 for position hold
    double dy = loiter_position_.pose.position.y -
                loiter_position_.pose.position.y;  // Always 0 for position hold

    // Altitude control
    // Note: In real implementation, this would use actual current altitude
    cmd.twist.linear.z = 0.0;  // Maintain current altitude
  }

  cmd_vel_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;  // Loiter runs indefinitely until halted
}

void LoiterAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "LoiterAction: Halted");
}

void LoiterAction::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  if (!position_initialized_) {
    loiter_position_.header = msg->header;
    loiter_position_.pose.position.x = msg->position.x;
    loiter_position_.pose.position.y = msg->position.y;
    loiter_position_.pose.position.z = msg->position.z;
    loiter_position_.pose.orientation = msg->orientation;
  }
}

// =============================================================================
// LandAction
// =============================================================================

LandAction::LandAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10);

  behavior_cmd_pub_ = node_->create_publisher<flyby_msgs::msg::BehaviorCommand>(
    "behavior_command", 10);

  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "uav_state", 10,
    std::bind(&LandAction::stateCallback, this, std::placeholders::_1));

  getInput("descent_rate", descent_rate_);
  getInput("landing_altitude", landing_altitude_);
}

BT::PortsList LandAction::providedPorts()
{
  return {
    BT::InputPort<double>("descent_rate", 1.0, "Descent rate (m/s)"),
    BT::InputPort<double>("landing_altitude", 0.5, "Altitude to consider landed (m)"),
  };
}

BT::NodeStatus LandAction::onStart()
{
  is_landed_ = false;
  RCLCPP_INFO(node_->get_logger(), "LandAction: Starting landing sequence");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandAction::onRunning()
{
  if (is_landed_) {
    RCLCPP_INFO(node_->get_logger(), "LandAction: Landed successfully");

    // Send disarm command
    flyby_msgs::msg::BehaviorCommand disarm_cmd;
    disarm_cmd.header.stamp = node_->now();
    disarm_cmd.behavior = flyby_msgs::msg::BehaviorCommand::DISARM;
    behavior_cmd_pub_->publish(disarm_cmd);

    return BT::NodeStatus::SUCCESS;
  }

  // Check if landed
  if (current_altitude_ < landing_altitude_) {
    is_landed_ = true;
    // Stop descent
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = node_->now();
    cmd_vel_pub_->publish(stop_cmd);
    return BT::NodeStatus::RUNNING;  // One more tick to return success
  }

  // Command descent
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.twist.linear.z = -descent_rate_;
  cmd_vel_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;
}

void LandAction::onHalted()
{
  // Stop descent
  geometry_msgs::msg::TwistStamped stop_cmd;
  stop_cmd.header.stamp = node_->now();
  cmd_vel_pub_->publish(stop_cmd);

  RCLCPP_INFO(node_->get_logger(), "LandAction: Halted");
}

void LandAction::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  current_altitude_ = msg->position.z;
}

// =============================================================================
// LawnmowerPatternAction
// =============================================================================

LawnmowerPatternAction::LawnmowerPatternAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10);

  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "uav_state", 10,
    std::bind(&LawnmowerPatternAction::stateCallback, this, std::placeholders::_1));

  getInput("track_spacing", track_spacing_);
  getInput("search_altitude", search_altitude_);
  getInput("search_speed", search_speed_);
  getInput("track_coverage", track_coverage_);
}

BT::PortsList LawnmowerPatternAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Polygon>("area", "Area to cover"),
    BT::InputPort<double>("track_spacing", 30.0, "Spacing between tracks (m)"),
    BT::InputPort<double>("search_altitude", 50.0, "Search altitude (m AGL)"),
    BT::InputPort<double>("search_speed", 5.0, "Search speed (m/s)"),
    BT::InputPort<bool>("track_coverage", true, "Track coverage percentage"),
    BT::OutputPort<double>("coverage_pct", "Current coverage percentage"),
  };
}

BT::NodeStatus LawnmowerPatternAction::onStart()
{
  geometry_msgs::msg::Polygon area;
  if (!getInput("area", area)) {
    RCLCPP_ERROR(node_->get_logger(), "LawnmowerPatternAction: No area provided");
    return BT::NodeStatus::FAILURE;
  }

  generateWaypoints(area);
  current_waypoint_idx_ = 0;
  pose_received_ = false;

  RCLCPP_INFO(node_->get_logger(),
    "LawnmowerPatternAction: Generated %zu waypoints", waypoints_.size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LawnmowerPatternAction::onRunning()
{
  if (!pose_received_) {
    return BT::NodeStatus::RUNNING;
  }

  if (current_waypoint_idx_ >= waypoints_.size()) {
    RCLCPP_INFO(node_->get_logger(), "LawnmowerPatternAction: Pattern complete");
    setOutput("coverage_pct", 100.0);
    return BT::NodeStatus::SUCCESS;
  }

  // Navigate to current waypoint
  const auto & target = waypoints_[current_waypoint_idx_];

  double dx = target.x - current_pose_.pose.position.x;
  double dy = target.y - current_pose_.pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  // Check if we've reached current waypoint
  if (dist < 3.0) {  // 3m tolerance
    current_waypoint_idx_++;

    // Update coverage estimate
    double coverage = (static_cast<double>(current_waypoint_idx_) /
                       static_cast<double>(waypoints_.size())) * 100.0;
    setOutput("coverage_pct", coverage);

    return BT::NodeStatus::RUNNING;
  }

  // Compute velocity command
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();

  double target_yaw = std::atan2(dy, dx);

  // Get current yaw
  double siny_cosp = 2.0 * (current_pose_.pose.orientation.w * current_pose_.pose.orientation.z +
                            current_pose_.pose.orientation.x * current_pose_.pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (current_pose_.pose.orientation.y * current_pose_.pose.orientation.y +
                                   current_pose_.pose.orientation.z * current_pose_.pose.orientation.z);
  double current_yaw = std::atan2(siny_cosp, cosy_cosp);

  double yaw_error = target_yaw - current_yaw;
  while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

  // Simple P control
  cmd.twist.linear.x = std::min(dist * 0.5, search_speed_) * std::cos(yaw_error);
  cmd.twist.linear.y = std::min(dist * 0.5, search_speed_) * std::sin(yaw_error);
  cmd.twist.linear.z = (search_altitude_ - current_pose_.pose.position.z) * 0.5;
  cmd.twist.angular.z = std::clamp(yaw_error * 1.0, -0.5, 0.5);

  cmd_vel_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;
}

void LawnmowerPatternAction::onHalted()
{
  geometry_msgs::msg::TwistStamped stop_cmd;
  stop_cmd.header.stamp = node_->now();
  cmd_vel_pub_->publish(stop_cmd);

  RCLCPP_INFO(node_->get_logger(), "LawnmowerPatternAction: Halted");
}

void LawnmowerPatternAction::generateWaypoints(const geometry_msgs::msg::Polygon & area)
{
  waypoints_.clear();

  if (area.points.size() < 3) {
    RCLCPP_WARN(node_->get_logger(), "Area has fewer than 3 points");
    return;
  }

  // Find bounding box
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & p : area.points) {
    min_x = std::min(min_x, static_cast<double>(p.x));
    max_x = std::max(max_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }

  // Generate boustrophedon pattern
  bool forward = true;
  for (double y = min_y; y <= max_y; y += track_spacing_) {
    if (forward) {
      geometry_msgs::msg::Point p1, p2;
      p1.x = min_x;
      p1.y = y;
      p1.z = search_altitude_;
      p2.x = max_x;
      p2.y = y;
      p2.z = search_altitude_;
      waypoints_.push_back(p1);
      waypoints_.push_back(p2);
    } else {
      geometry_msgs::msg::Point p1, p2;
      p1.x = max_x;
      p1.y = y;
      p1.z = search_altitude_;
      p2.x = min_x;
      p2.y = y;
      p2.z = search_altitude_;
      waypoints_.push_back(p1);
      waypoints_.push_back(p2);
    }
    forward = !forward;
  }
}

void LawnmowerPatternAction::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose.position.x = msg->position.x;
  current_pose_.pose.position.y = msg->position.y;
  current_pose_.pose.position.z = msg->position.z;
  current_pose_.pose.orientation = msg->orientation;
  pose_received_ = true;
}

// =============================================================================
// OrbitPOIAction
// =============================================================================

OrbitPOIAction::OrbitPOIAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10);

  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "uav_state", 10,
    std::bind(&OrbitPOIAction::stateCallback, this, std::placeholders::_1));

  getInput("radius", orbit_radius_);
  getInput("speed", orbit_speed_);
  getInput("altitude", orbit_altitude_);
  getInput("gimbal_track", gimbal_track_);
  getInput("clockwise", clockwise_);
}

BT::PortsList OrbitPOIAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Point>("target", "Target position to orbit"),
    BT::InputPort<double>("radius", 30.0, "Orbit radius (m)"),
    BT::InputPort<double>("speed", 5.0, "Orbit speed (m/s)"),
    BT::InputPort<double>("altitude", 50.0, "Orbit altitude (m AGL)"),
    BT::InputPort<bool>("gimbal_track", true, "Track target with gimbal"),
    BT::InputPort<bool>("clockwise", true, "Orbit direction"),
  };
}

BT::NodeStatus OrbitPOIAction::onStart()
{
  if (!getInput("target", target_position_)) {
    RCLCPP_ERROR(node_->get_logger(), "OrbitPOIAction: No target provided");
    return BT::NodeStatus::FAILURE;
  }

  pose_received_ = false;
  current_angle_ = 0.0;

  RCLCPP_INFO(node_->get_logger(),
    "OrbitPOIAction: Starting orbit around (%.1f, %.1f) at %.1f m radius",
    target_position_.x, target_position_.y, orbit_radius_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OrbitPOIAction::onRunning()
{
  if (!pose_received_) {
    return BT::NodeStatus::RUNNING;
  }

  auto cmd = computeOrbitCommand();
  cmd_vel_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;  // Orbit runs indefinitely until halted
}

void OrbitPOIAction::onHalted()
{
  geometry_msgs::msg::TwistStamped stop_cmd;
  stop_cmd.header.stamp = node_->now();
  cmd_vel_pub_->publish(stop_cmd);

  RCLCPP_INFO(node_->get_logger(), "OrbitPOIAction: Halted");
}

void OrbitPOIAction::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose.position.x = msg->position.x;
  current_pose_.pose.position.y = msg->position.y;
  current_pose_.pose.position.z = msg->position.z;
  current_pose_.pose.orientation = msg->orientation;
  pose_received_ = true;
}

geometry_msgs::msg::TwistStamped OrbitPOIAction::computeOrbitCommand()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();

  // Current position relative to target
  double dx = current_pose_.pose.position.x - target_position_.x;
  double dy = current_pose_.pose.position.y - target_position_.y;
  double current_radius = std::sqrt(dx * dx + dy * dy);
  double current_angle = std::atan2(dy, dx);

  // Radius correction (move towards desired radius)
  double radius_error = orbit_radius_ - current_radius;
  double radial_vel = std::clamp(radius_error * 0.5, -2.0, 2.0);

  // Tangential velocity for orbit
  double angular_rate = orbit_speed_ / orbit_radius_;
  if (!clockwise_) {
    angular_rate = -angular_rate;
  }

  // Convert to global frame velocities
  double tangent_x = -std::sin(current_angle) * (clockwise_ ? 1 : -1);
  double tangent_y = std::cos(current_angle) * (clockwise_ ? 1 : -1);
  double radial_x = std::cos(current_angle);
  double radial_y = std::sin(current_angle);

  cmd.twist.linear.x = tangent_x * orbit_speed_ + radial_x * radial_vel;
  cmd.twist.linear.y = tangent_y * orbit_speed_ + radial_y * radial_vel;
  cmd.twist.linear.z = (orbit_altitude_ - current_pose_.pose.position.z) * 0.5;

  // Yaw to point at target (if gimbal tracking)
  if (gimbal_track_) {
    double target_yaw = std::atan2(-dy, -dx);  // Point towards target

    // Get current yaw
    double siny_cosp = 2.0 * (current_pose_.pose.orientation.w * current_pose_.pose.orientation.z +
                              current_pose_.pose.orientation.x * current_pose_.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (current_pose_.pose.orientation.y * current_pose_.pose.orientation.y +
                                     current_pose_.pose.orientation.z * current_pose_.pose.orientation.z);
    double current_yaw = std::atan2(siny_cosp, cosy_cosp);

    double yaw_error = target_yaw - current_yaw;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    cmd.twist.angular.z = std::clamp(yaw_error * 1.0, -0.5, 0.5);
  }

  return cmd;
}

// =============================================================================
// Registration
// =============================================================================

void registerActionNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node)
{
  factory.registerBuilder<TransitAction>(
    "Transit",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<TransitAction>(name, config, node);
    });

  factory.registerBuilder<LoiterAction>(
    "Loiter",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<LoiterAction>(name, config, node);
    });

  factory.registerBuilder<LandAction>(
    "Land",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<LandAction>(name, config, node);
    });

  factory.registerBuilder<LawnmowerPatternAction>(
    "LawnmowerPattern",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<LawnmowerPatternAction>(name, config, node);
    });

  factory.registerBuilder<OrbitPOIAction>(
    "OrbitPOI",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<OrbitPOIAction>(name, config, node);
    });
}

}  // namespace behavior_trees
