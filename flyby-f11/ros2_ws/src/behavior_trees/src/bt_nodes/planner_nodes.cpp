/**
 * @file planner_nodes.cpp
 * @brief Implementation of path planning BT nodes
 *
 * Provides BT nodes for requesting paths from the global planner
 * and utility nodes for getting poses and waypoints.
 */

#include "behavior_trees/bt_nodes/planner_nodes.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

namespace behavior_trees
{

// =============================================================================
// PlanPathNode Implementation
// =============================================================================

PlanPathNode::PlanPathNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosActionNode<PlanPathAction>(name, config, params)
{
}

BT::PortsList PlanPathNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("start", "Start pose"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Goal pose"),
    BT::InputPort<double>("timeout_sec", 2.0, "Planning timeout in seconds"),
    BT::OutputPort<nav_msgs::msg::Path>("planned_path", "Resulting planned path"),
  };
}

bool PlanPathNode::setGoal(Goal & goal)
{
  // Get start pose from blackboard or input port
  auto start_result = getInput<geometry_msgs::msg::PoseStamped>("start");
  if (!start_result) {
    // Try to get from blackboard key
    auto bb_start = config().blackboard->get<geometry_msgs::msg::PoseStamped>("current_pose");
    goal.start = bb_start;
  } else {
    goal.start = start_result.value();
  }

  // Get goal pose
  auto goal_result = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_result) {
    RCLCPP_ERROR(node_->get_logger(), "PlanPathNode: goal pose not provided");
    return false;
  }
  goal.goal = goal_result.value();

  // Get timeout
  double timeout = 2.0;
  getInput("timeout_sec", timeout);
  goal.planning_timeout_sec = timeout;

  // Get NFZ and threat zones from blackboard (populated by mission tasking)
  try {
    auto tasking = config().blackboard->get<flyby_msgs::msg::MissionTasking>("mission_tasking");
    goal.no_fly_zones = tasking.no_fly_zones;

    // Convert ThreatZone boundaries to Polygon
    for (const auto & threat : tasking.threat_zones) {
      goal.threat_zones.push_back(threat.boundary);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "PlanPathNode: No mission tasking on blackboard: %s",
      e.what());
  }

  RCLCPP_INFO(node_->get_logger(), "PlanPathNode: Planning from (%.1f, %.1f) to (%.1f, %.1f)",
    goal.start.pose.position.x, goal.start.pose.position.y,
    goal.goal.pose.position.x, goal.goal.pose.position.y);

  return true;
}

BT::NodeStatus PlanPathNode::onResultReceived(const WrappedResult & result)
{
  if (result.result->success) {
    setOutput("planned_path", result.result->path);
    config().blackboard->set("planned_path", result.result->path);

    RCLCPP_INFO(node_->get_logger(), "PlanPathNode: Path found with %zu waypoints",
      result.result->path.poses.size());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(node_->get_logger(), "PlanPathNode: Planning failed: %s",
    result.result->error_message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlanPathNode::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_DEBUG(node_->get_logger(), "PlanPathNode: %d iterations, best cost: %.2f, elapsed: %.2fs",
    feedback->iterations_completed, feedback->best_cost_so_far, feedback->elapsed_time_sec);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanPathNode::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "PlanPathNode: Action failed with error code %d",
    static_cast<int>(error));
  return BT::NodeStatus::FAILURE;
}

// =============================================================================
// GetCurrentPoseNode Implementation
// =============================================================================

GetCurrentPoseNode::GetCurrentPoseNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "/flyby/uav_state", 10,
    std::bind(&GetCurrentPoseNode::stateCallback, this, std::placeholders::_1));
}

BT::PortsList GetCurrentPoseNode::providedPorts()
{
  return {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Current UAV pose"),
  };
}

void GetCurrentPoseNode::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose.position.x = msg->position.x;
  current_pose_.pose.position.y = msg->position.y;
  current_pose_.pose.position.z = msg->altitude;
  current_pose_.pose.orientation = msg->orientation;
  pose_received_ = true;
}

BT::NodeStatus GetCurrentPoseNode::tick()
{
  if (!pose_received_) {
    RCLCPP_WARN(node_->get_logger(), "GetCurrentPoseNode: No pose received yet");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("pose", current_pose_);
  config().blackboard->set("current_pose", current_pose_);
  return BT::NodeStatus::SUCCESS;
}

// =============================================================================
// GetHomeBaseNode Implementation
// =============================================================================

GetHomeBaseNode::GetHomeBaseNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
  tasking_sub_ = node_->create_subscription<flyby_msgs::msg::MissionTasking>(
    "/flyby/mission_tasking", 10,
    std::bind(&GetHomeBaseNode::taskingCallback, this, std::placeholders::_1));
}

BT::PortsList GetHomeBaseNode::providedPorts()
{
  return {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("home_base", "Home base pose"),
  };
}

void GetHomeBaseNode::taskingCallback(const flyby_msgs::msg::MissionTasking::SharedPtr msg)
{
  home_base_ = msg->home_base;
  tasking_received_ = true;
}

BT::NodeStatus GetHomeBaseNode::tick()
{
  if (!tasking_received_) {
    RCLCPP_WARN(node_->get_logger(), "GetHomeBaseNode: No mission tasking received");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("home_base", home_base_);
  config().blackboard->set("home_base", home_base_);
  return BT::NodeStatus::SUCCESS;
}

// =============================================================================
// GetAOEntryNode Implementation
// =============================================================================

GetAOEntryNode::GetAOEntryNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
  tasking_sub_ = node_->create_subscription<flyby_msgs::msg::MissionTasking>(
    "/flyby/mission_tasking", 10,
    std::bind(&GetAOEntryNode::taskingCallback, this, std::placeholders::_1));

  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "/flyby/uav_state", 10,
    std::bind(&GetAOEntryNode::stateCallback, this, std::placeholders::_1));
}

BT::PortsList GetAOEntryNode::providedPorts()
{
  return {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("ao_entry", "AO entry point"),
  };
}

void GetAOEntryNode::taskingCallback(const flyby_msgs::msg::MissionTasking::SharedPtr msg)
{
  current_tasking_ = *msg;
  tasking_received_ = true;
}

void GetAOEntryNode::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose.position.x = msg->position.x;
  current_pose_.pose.position.y = msg->position.y;
  current_pose_.pose.position.z = msg->altitude;
  current_pose_.pose.orientation = msg->orientation;
  pose_received_ = true;
}

geometry_msgs::msg::PoseStamped GetAOEntryNode::computeNearestEntryPoint()
{
  geometry_msgs::msg::PoseStamped entry_point;
  entry_point.header.frame_id = "map";
  entry_point.header.stamp = node_->now();

  const auto & ao = current_tasking_.area_of_operations;

  // If suggested entry points exist, find nearest
  if (!ao.entry_points.empty()) {
    double min_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point nearest;

    for (const auto & ep : ao.entry_points) {
      double dx = ep.x - current_pose_.pose.position.x;
      double dy = ep.y - current_pose_.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
        nearest = ep;
      }
    }

    entry_point.pose.position = nearest;
    entry_point.pose.position.z = ao.recommended_altitude_m > 0 ?
      ao.recommended_altitude_m : ao.min_altitude_m;
  } else {
    // No entry points defined - find nearest vertex of AO boundary
    double min_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point32 nearest;

    for (const auto & vertex : ao.boundary.points) {
      double dx = vertex.x - current_pose_.pose.position.x;
      double dy = vertex.y - current_pose_.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
        nearest = vertex;
      }
    }

    entry_point.pose.position.x = nearest.x;
    entry_point.pose.position.y = nearest.y;
    entry_point.pose.position.z = ao.recommended_altitude_m > 0 ?
      ao.recommended_altitude_m : ao.min_altitude_m;
  }

  // Set orientation to face into AO (toward centroid)
  double cx = 0.0, cy = 0.0;
  for (const auto & vertex : ao.boundary.points) {
    cx += vertex.x;
    cy += vertex.y;
  }
  if (!ao.boundary.points.empty()) {
    cx /= ao.boundary.points.size();
    cy /= ao.boundary.points.size();
  }

  double yaw = std::atan2(cy - entry_point.pose.position.y,
    cx - entry_point.pose.position.x);

  // Convert yaw to quaternion (z-axis rotation only)
  entry_point.pose.orientation.x = 0.0;
  entry_point.pose.orientation.y = 0.0;
  entry_point.pose.orientation.z = std::sin(yaw / 2.0);
  entry_point.pose.orientation.w = std::cos(yaw / 2.0);

  return entry_point;
}

BT::NodeStatus GetAOEntryNode::tick()
{
  if (!tasking_received_) {
    RCLCPP_WARN(node_->get_logger(), "GetAOEntryNode: No mission tasking received");
    return BT::NodeStatus::FAILURE;
  }
  if (!pose_received_) {
    RCLCPP_WARN(node_->get_logger(), "GetAOEntryNode: No pose received");
    return BT::NodeStatus::FAILURE;
  }

  auto entry = computeNearestEntryPoint();
  setOutput("ao_entry", entry);
  config().blackboard->set("ao_entry", entry);

  RCLCPP_INFO(node_->get_logger(), "GetAOEntryNode: Entry point at (%.1f, %.1f, %.1f)",
    entry.pose.position.x, entry.pose.position.y, entry.pose.position.z);

  return BT::NodeStatus::SUCCESS;
}

// =============================================================================
// GetSafeWaypointNode Implementation
// =============================================================================

GetSafeWaypointNode::GetSafeWaypointNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
  state_sub_ = node_->create_subscription<flyby_msgs::msg::UAVState>(
    "/flyby/uav_state", 10,
    std::bind(&GetSafeWaypointNode::stateCallback, this, std::placeholders::_1));

  tasking_sub_ = node_->create_subscription<flyby_msgs::msg::MissionTasking>(
    "/flyby/mission_tasking", 10,
    std::bind(&GetSafeWaypointNode::taskingCallback, this, std::placeholders::_1));
}

BT::PortsList GetSafeWaypointNode::providedPorts()
{
  return {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("safe_waypoint", "Safe waypoint outside NFZ"),
  };
}

void GetSafeWaypointNode::stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose.position.x = msg->position.x;
  current_pose_.pose.position.y = msg->position.y;
  current_pose_.pose.position.z = msg->altitude;
  current_pose_.pose.orientation = msg->orientation;
  pose_received_ = true;
}

void GetSafeWaypointNode::taskingCallback(const flyby_msgs::msg::MissionTasking::SharedPtr msg)
{
  nfz_polygons_ = msg->no_fly_zones;
}

bool GetSafeWaypointNode::isPointInPolygon(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Polygon & polygon)
{
  // Ray casting algorithm for point-in-polygon test
  int n = polygon.points.size();
  if (n < 3) {
    return false;
  }

  int crossings = 0;
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;

    double xi = polygon.points[i].x;
    double yi = polygon.points[i].y;
    double xj = polygon.points[j].x;
    double yj = polygon.points[j].y;

    if ((yi > point.y) != (yj > point.y)) {
      double x_intersect = (xj - xi) * (point.y - yi) / (yj - yi) + xi;
      if (point.x < x_intersect) {
        crossings++;
      }
    }
  }

  return (crossings % 2) == 1;
}

geometry_msgs::msg::PoseStamped GetSafeWaypointNode::computeSafeWaypoint()
{
  geometry_msgs::msg::PoseStamped safe_wp;
  safe_wp.header.frame_id = "map";
  safe_wp.header.stamp = node_->now();

  // Check which NFZ we're currently in
  const geometry_msgs::msg::Polygon * violating_nfz = nullptr;
  for (const auto & nfz : nfz_polygons_) {
    if (isPointInPolygon(current_pose_.pose.position, nfz)) {
      violating_nfz = &nfz;
      break;
    }
  }

  if (violating_nfz == nullptr) {
    // Not in any NFZ - return current position (shouldn't happen)
    RCLCPP_WARN(node_->get_logger(), "GetSafeWaypointNode: Not in NFZ, returning current position");
    return current_pose_;
  }

  // Find nearest point on NFZ boundary and move outside
  double min_dist = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_boundary;
  geometry_msgs::msg::Point outward_normal;

  int n = violating_nfz->points.size();
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;

    // Project current position onto edge
    double ax = violating_nfz->points[i].x;
    double ay = violating_nfz->points[i].y;
    double bx = violating_nfz->points[j].x;
    double by = violating_nfz->points[j].y;

    double abx = bx - ax;
    double aby = by - ay;
    double apx = current_pose_.pose.position.x - ax;
    double apy = current_pose_.pose.position.y - ay;

    double ab_sq = abx * abx + aby * aby;
    double t = std::clamp((apx * abx + apy * aby) / ab_sq, 0.0, 1.0);

    double px = ax + t * abx;
    double py = ay + t * aby;

    double dist = std::sqrt(
      std::pow(current_pose_.pose.position.x - px, 2) +
      std::pow(current_pose_.pose.position.y - py, 2));

    if (dist < min_dist) {
      min_dist = dist;
      nearest_boundary.x = px;
      nearest_boundary.y = py;

      // Compute outward normal (perpendicular to edge, pointing outward)
      double nx = -aby;
      double ny = abx;
      double norm = std::sqrt(nx * nx + ny * ny);
      if (norm > 1e-6) {
        nx /= norm;
        ny /= norm;
      }

      // Ensure normal points outward (away from polygon interior)
      double cx = 0.0, cy = 0.0;
      for (const auto & pt : violating_nfz->points) {
        cx += pt.x;
        cy += pt.y;
      }
      cx /= n;
      cy /= n;

      double to_center_x = cx - px;
      double to_center_y = cy - py;
      if (nx * to_center_x + ny * to_center_y > 0) {
        // Normal points inward, flip it
        nx = -nx;
        ny = -ny;
      }

      outward_normal.x = nx;
      outward_normal.y = ny;
    }
  }

  // Move 10m outside the NFZ boundary
  const double safety_margin = 10.0;
  safe_wp.pose.position.x = nearest_boundary.x + outward_normal.x * safety_margin;
  safe_wp.pose.position.y = nearest_boundary.y + outward_normal.y * safety_margin;
  safe_wp.pose.position.z = current_pose_.pose.position.z;

  // Orient toward the safe waypoint
  double dx = safe_wp.pose.position.x - current_pose_.pose.position.x;
  double dy = safe_wp.pose.position.y - current_pose_.pose.position.y;
  double yaw = std::atan2(dy, dx);

  safe_wp.pose.orientation.x = 0.0;
  safe_wp.pose.orientation.y = 0.0;
  safe_wp.pose.orientation.z = std::sin(yaw / 2.0);
  safe_wp.pose.orientation.w = std::cos(yaw / 2.0);

  RCLCPP_INFO(node_->get_logger(), "GetSafeWaypointNode: Safe waypoint at (%.1f, %.1f)",
    safe_wp.pose.position.x, safe_wp.pose.position.y);

  return safe_wp;
}

BT::NodeStatus GetSafeWaypointNode::tick()
{
  if (!pose_received_) {
    RCLCPP_WARN(node_->get_logger(), "GetSafeWaypointNode: No pose received");
    return BT::NodeStatus::FAILURE;
  }
  if (nfz_polygons_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "GetSafeWaypointNode: No NFZ polygons defined");
    return BT::NodeStatus::FAILURE;
  }

  auto safe_wp = computeSafeWaypoint();
  setOutput("safe_waypoint", safe_wp);
  config().blackboard->set("safe_waypoint", safe_wp);

  return BT::NodeStatus::SUCCESS;
}

// =============================================================================
// Registration
// =============================================================================

void registerPlannerNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  const BT::RosNodeParams & params)
{
  // PlanPath action node (uses ROS2 action server)
  factory.registerNodeType<PlanPathNode>("PlanPath", params);

  // Utility sync nodes (need explicit node pointer for subscriptions)
  factory.registerBuilder<GetCurrentPoseNode>(
    "GetCurrentPose",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<GetCurrentPoseNode>(name, config, node);
    });

  factory.registerBuilder<GetHomeBaseNode>(
    "GetHomeBase",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<GetHomeBaseNode>(name, config, node);
    });

  factory.registerBuilder<GetAOEntryNode>(
    "GetAOEntry",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<GetAOEntryNode>(name, config, node);
    });

  factory.registerBuilder<GetSafeWaypointNode>(
    "GetSafeWaypoint",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<GetSafeWaypointNode>(name, config, node);
    });
}

}  // namespace behavior_trees
