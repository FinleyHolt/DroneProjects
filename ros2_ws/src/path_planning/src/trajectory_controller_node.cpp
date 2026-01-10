/**
 * @file trajectory_controller_node.cpp
 * @brief ROS2 Node for trajectory following
 *
 * Subscribes to planned paths and UAV state, publishes velocity commands.
 * Uses Pure Pursuit for XY and PID for altitude control.
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "flyby_msgs/msg/uav_state.hpp"
#include "nav_msgs/msg/path.hpp"

#include "path_planning/trajectory_controller.hpp"

using namespace std::chrono_literals;

namespace path_planning
{

class TrajectoryControllerNode : public rclcpp::Node
{
public:
  TrajectoryControllerNode()
  : Node("trajectory_controller")
  {
    // Declare parameters
    this->declare_parameter<double>("control_rate_hz", 50.0);
    this->declare_parameter<double>("lookahead_distance", 5.0);
    this->declare_parameter<double>("min_lookahead", 2.0);
    this->declare_parameter<double>("max_lookahead", 15.0);
    this->declare_parameter<double>("max_linear_velocity", 10.0);
    this->declare_parameter<double>("min_linear_velocity", 1.0);
    this->declare_parameter<double>("max_angular_velocity", 0.5);
    this->declare_parameter<double>("altitude_kp", 1.0);
    this->declare_parameter<double>("altitude_ki", 0.1);
    this->declare_parameter<double>("altitude_kd", 0.2);
    this->declare_parameter<double>("xy_goal_tolerance", 2.0);
    this->declare_parameter<double>("z_goal_tolerance", 1.0);

    // Initialize controller with configuration
    TrajectoryControllerConfig config;
    config.lookahead_distance = this->get_parameter("lookahead_distance").as_double();
    config.min_lookahead = this->get_parameter("min_lookahead").as_double();
    config.max_lookahead = this->get_parameter("max_lookahead").as_double();
    config.max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
    config.min_linear_velocity = this->get_parameter("min_linear_velocity").as_double();
    config.max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();
    config.altitude_kp = this->get_parameter("altitude_kp").as_double();
    config.altitude_ki = this->get_parameter("altitude_ki").as_double();
    config.altitude_kd = this->get_parameter("altitude_kd").as_double();
    config.xy_goal_tolerance = this->get_parameter("xy_goal_tolerance").as_double();
    config.z_goal_tolerance = this->get_parameter("z_goal_tolerance").as_double();

    controller_ = std::make_unique<TrajectoryController>(config);

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/flyby/cmd_vel", 10);
    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/flyby/goal_reached", 10);
    lookahead_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/flyby/lookahead_point", 10);

    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/flyby/planned_path", 10,
      std::bind(&TrajectoryControllerNode::pathCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<flyby_msgs::msg::UAVState>(
      "/flyby/uav_state", 10,
      std::bind(&TrajectoryControllerNode::stateCallback, this, std::placeholders::_1));

    // Control timer
    double control_rate = this->get_parameter("control_rate_hz").as_double();
    control_period_sec_ = 1.0 / control_rate;
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(control_period_sec_),
      std::bind(&TrajectoryControllerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory Controller Node initialized at %.1f Hz",
      control_rate);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path");
      controller_->clearPath();
      return;
    }

    controller_->setPath(*msg);
    RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints", msg->poses.size());
  }

  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
  {
    current_pose_.header = msg->header;
    current_pose_.pose.position.x = msg->position.x;
    current_pose_.pose.position.y = msg->position.y;
    current_pose_.pose.position.z = msg->altitude;
    current_pose_.pose.orientation = msg->orientation;
    pose_received_ = true;
  }

  void controlLoop()
  {
    if (!pose_received_ || !controller_->hasPath()) {
      // Publish zero velocity if no path or pose
      if (controller_->hasPath()) {
        // We have a path but no pose - log warning
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Waiting for UAV state");
      }
      return;
    }

    // Update controller
    auto output = controller_->update(current_pose_, control_period_sec_);

    // Publish velocity command
    geometry_msgs::msg::TwistStamped cmd_msg;
    cmd_msg.header.stamp = this->now();
    cmd_msg.header.frame_id = "base_link";
    cmd_msg.twist = output.cmd_vel;
    cmd_vel_pub_->publish(cmd_msg);

    // Publish goal reached status
    std_msgs::msg::Bool goal_msg;
    goal_msg.data = output.goal_reached;
    goal_reached_pub_->publish(goal_msg);

    // Publish lookahead visualization
    publishLookaheadVisualization();

    // Log progress periodically
    static int log_counter = 0;
    if (++log_counter % 50 == 0) {  // Every ~1 second at 50Hz
      RCLCPP_DEBUG(this->get_logger(),
        "Distance to goal: %.1f m, waypoint: %d, CTE: %.2f m",
        output.distance_to_goal, output.current_waypoint_idx, output.cross_track_error);
    }

    // Handle goal reached
    if (output.goal_reached && !goal_reached_logged_) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      goal_reached_logged_ = true;
      controller_->clearPath();
    } else if (!output.goal_reached) {
      goal_reached_logged_ = false;
    }
  }

  void publishLookaheadVisualization()
  {
    const auto & path = controller_->getPath();
    if (path.poses.empty()) {
      return;
    }

    int wp_idx = controller_->getCurrentWaypointIndex();
    if (wp_idx < 0 || wp_idx >= static_cast<int>(path.poses.size())) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = path.header.frame_id;
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = path.poses[wp_idx].pose;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    lookahead_viz_pub_->publish(marker);
  }

  std::unique_ptr<TrajectoryController> controller_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_viz_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
  double control_period_sec_{0.02};

  // State
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_received_{false};
  bool goal_reached_logged_{false};
};

}  // namespace path_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_planning::TrajectoryControllerNode>());
  rclcpp::shutdown();
  return 0;
}
