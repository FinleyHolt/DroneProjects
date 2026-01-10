/**
 * @file global_planner_node.cpp
 * @brief ROS2 Action Server for global path planning
 *
 * Provides PlanPath action server for BT integration.
 * Uses Informed RRT* via OMPL for optimal path planning.
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "flyby_msgs/action/plan_path.hpp"
#include "flyby_msgs/msg/mission_tasking.hpp"

#include "path_planning/global_planner.hpp"
#include "path_planning/nfz_manager.hpp"

using namespace std::chrono_literals;
using PlanPathAction = flyby_msgs::action::PlanPath;
using GoalHandlePlanPath = rclcpp_action::ServerGoalHandle<PlanPathAction>;

namespace path_planning
{

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner")
  {
    // Declare parameters
    this->declare_parameter<double>("planning_timeout_sec", 2.0);
    this->declare_parameter<double>("goal_tolerance", 2.0);
    this->declare_parameter<double>("range", 50.0);
    this->declare_parameter<bool>("simplify_path", true);
    this->declare_parameter<int>("simplification_iterations", 10);
    this->declare_parameter<double>("x_min", -1000.0);
    this->declare_parameter<double>("x_max", 1000.0);
    this->declare_parameter<double>("y_min", -1000.0);
    this->declare_parameter<double>("y_max", 1000.0);
    this->declare_parameter<double>("z_min", 20.0);
    this->declare_parameter<double>("z_max", 120.0);

    // Initialize NFZ manager
    nfz_manager_ = std::make_shared<NFZManager>();

    // Initialize planner with configuration
    PlannerConfig config;
    config.planning_timeout_sec = this->get_parameter("planning_timeout_sec").as_double();
    config.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    config.range = this->get_parameter("range").as_double();
    config.simplify_path = this->get_parameter("simplify_path").as_bool();
    config.simplification_iterations = this->get_parameter("simplification_iterations").as_int();
    config.x_min = this->get_parameter("x_min").as_double();
    config.x_max = this->get_parameter("x_max").as_double();
    config.y_min = this->get_parameter("y_min").as_double();
    config.y_max = this->get_parameter("y_max").as_double();
    config.z_min = this->get_parameter("z_min").as_double();
    config.z_max = this->get_parameter("z_max").as_double();

    nfz_manager_->setAltitudeBounds(config.z_min, config.z_max);

    planner_ = std::make_unique<GlobalPlanner>(config);
    planner_->setNFZManager(nfz_manager_);

    // Create action server
    action_server_ = rclcpp_action::create_server<PlanPathAction>(
      this,
      "/flyby/plan_path",
      std::bind(&GlobalPlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GlobalPlannerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&GlobalPlannerNode::handleAccepted, this, std::placeholders::_1));

    // Subscribe to mission tasking for NFZ updates
    tasking_sub_ = this->create_subscription<flyby_msgs::msg::MissionTasking>(
      "/flyby/mission_tasking", 10,
      std::bind(&GlobalPlannerNode::taskingCallback, this, std::placeholders::_1));

    // Publisher for visualization
    path_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/flyby/planned_path_viz", 10);

    RCLCPP_INFO(this->get_logger(), "Global Planner Node initialized");
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanPathAction::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received planning request from (%.1f, %.1f) to (%.1f, %.1f)",
      goal->start.pose.position.x, goal->start.pose.position.y,
      goal->goal.pose.position.x, goal->goal.pose.position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePlanPath> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Planning request canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
  {
    // Execute in a separate thread to not block the action server
    std::thread{std::bind(&GlobalPlannerNode::execute, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanPathAction::Result>();
    auto feedback = std::make_shared<PlanPathAction::Feedback>();

    // Update NFZs from goal
    if (!goal->no_fly_zones.empty()) {
      nfz_manager_->setNFZs(goal->no_fly_zones);
      RCLCPP_INFO(this->get_logger(), "Updated %zu NFZs from goal",
        goal->no_fly_zones.size());
    }

    // Use timeout from goal if specified
    double timeout = goal->planning_timeout_sec > 0 ?
      goal->planning_timeout_sec : planner_->getConfig().planning_timeout_sec;

    // Start planning
    auto start_time = this->now();
    feedback->elapsed_time_sec = 0.0;
    feedback->iterations_completed = 0;
    feedback->best_cost_so_far = std::numeric_limits<double>::infinity();
    goal_handle->publish_feedback(feedback);

    // Plan
    PlanningResult planning_result = planner_->plan(goal->start, goal->goal, timeout);

    // Check if canceled
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->error_message = "Planning canceled";
      goal_handle->canceled(result);
      return;
    }

    // Populate result
    result->success = planning_result.success;
    result->path = planning_result.path;
    result->error_message = planning_result.error_message;

    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Path found: %zu waypoints, length: %.1f m, time: %.2f s",
        result->path.poses.size(), planning_result.path_length, planning_result.planning_time_sec);

      // Publish visualization
      publishPathVisualization(result->path);

      goal_handle->succeed(result);
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed: %s", result->error_message.c_str());
      goal_handle->abort(result);
    }
  }

  void taskingCallback(const flyby_msgs::msg::MissionTasking::SharedPtr msg)
  {
    // Update NFZs from mission tasking
    nfz_manager_->setNFZs(msg->no_fly_zones);
    nfz_manager_->setThreatZones(msg->threat_zones);

    // Update bounds from AO if available
    const auto & ao = msg->area_of_operations;
    if (!ao.boundary.points.empty()) {
      // Compute bounding box
      double x_min = std::numeric_limits<double>::max();
      double x_max = std::numeric_limits<double>::lowest();
      double y_min = std::numeric_limits<double>::max();
      double y_max = std::numeric_limits<double>::lowest();

      for (const auto & pt : ao.boundary.points) {
        x_min = std::min(x_min, static_cast<double>(pt.x));
        x_max = std::max(x_max, static_cast<double>(pt.x));
        y_min = std::min(y_min, static_cast<double>(pt.y));
        y_max = std::max(y_max, static_cast<double>(pt.y));
      }

      // Add margin
      double margin = 200.0;
      planner_->setBounds(
        x_min - margin, x_max + margin,
        y_min - margin, y_max + margin,
        ao.min_altitude_m, ao.max_altitude_m);
    }

    RCLCPP_INFO(this->get_logger(), "Updated %zu NFZs, %zu threat zones from mission tasking",
      nfz_manager_->getNFZCount(), nfz_manager_->getThreatZoneCount());
  }

  void publishPathVisualization(const nav_msgs::msg::Path & path)
  {
    visualization_msgs::msg::MarkerArray markers;

    // Path line
    visualization_msgs::msg::Marker line_marker;
    line_marker.header = path.header;
    line_marker.header.stamp = this->now();
    line_marker.ns = "planned_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.5;
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    for (const auto & pose : path.poses) {
      line_marker.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(line_marker);

    // Waypoint spheres
    for (size_t i = 0; i < path.poses.size(); ++i) {
      visualization_msgs::msg::Marker sphere;
      sphere.header = path.header;
      sphere.header.stamp = this->now();
      sphere.ns = "waypoints";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = path.poses[i].pose;
      sphere.scale.x = 1.0;
      sphere.scale.y = 1.0;
      sphere.scale.z = 1.0;
      sphere.color.r = 1.0;
      sphere.color.g = 0.5;
      sphere.color.b = 0.0;
      sphere.color.a = 0.8;
      markers.markers.push_back(sphere);
    }

    path_viz_pub_->publish(markers);
  }

  std::shared_ptr<NFZManager> nfz_manager_;
  std::unique_ptr<GlobalPlanner> planner_;

  rclcpp_action::Server<PlanPathAction>::SharedPtr action_server_;
  rclcpp::Subscription<flyby_msgs::msg::MissionTasking>::SharedPtr tasking_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_viz_pub_;
};

}  // namespace path_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_planning::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
