/**
 * @file local_avoidance_node.cpp
 * @brief ROS 2 node for 3D-VFH+ local obstacle avoidance
 *
 * Subscribes to depth point clouds and UAV state, maintains OctoMap,
 * runs VFH+ algorithm, and publishes avoidance waypoints.
 *
 * Topics:
 *   Subscribed:
 *     /depth/points (sensor_msgs/PointCloud2) - Depth point cloud
 *     /uav/state (flyby_msgs/UAVState) - UAV state
 *     /path_planning/current_goal (geometry_msgs/PoseStamped) - Current navigation goal
 *
 *   Published:
 *     /avoidance/waypoint_override (geometry_msgs/PoseStamped) - Modified waypoint
 *     /avoidance/obstacle_map (flyby_msgs/ObstacleMap) - Obstacle map status
 *     /avoidance/octomap (octomap_msgs/Octomap) - Visualization
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "flyby_msgs/msg/uav_state.hpp"
#include "flyby_msgs/msg/obstacle_map.hpp"

#include "local_avoidance/octomap_manager.hpp"
#include "local_avoidance/vfh3d.hpp"

using namespace std::chrono_literals;

namespace local_avoidance
{

class LocalAvoidanceNode : public rclcpp::Node
{
public:
  LocalAvoidanceNode()
  : Node("local_avoidance_node")
  {
    // Declare parameters
    declare_parameters();

    // Get parameters
    auto octomap_config = get_octomap_config();
    auto vfh_config = get_vfh_config();

    update_rate_ = this->get_parameter("update_rate").as_double();
    override_timeout_ = this->get_parameter("override_timeout").as_double();
    publish_octomap_ = this->get_parameter("publish_octomap").as_bool();
    publish_markers_ = this->get_parameter("publish_markers").as_bool();

    // Initialize components
    octomap_manager_ = std::make_shared<OctomapManager>(octomap_config);
    vfh_ = std::make_unique<VFH3D>(octomap_manager_, vfh_config);

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->get_parameter("pointcloud_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&LocalAvoidanceNode::pointcloud_callback, this, std::placeholders::_1));

    uav_state_sub_ = this->create_subscription<flyby_msgs::msg::UAVState>(
      "/uav/state",
      rclcpp::SensorDataQoS(),
      std::bind(&LocalAvoidanceNode::uav_state_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/path_planning/current_goal",
      10,
      std::bind(&LocalAvoidanceNode::goal_callback, this, std::placeholders::_1));

    // Publishers
    waypoint_override_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/avoidance/waypoint_override", 10);

    obstacle_map_pub_ = this->create_publisher<flyby_msgs::msg::ObstacleMap>(
      "/avoidance/obstacle_map", 10);

    if (publish_octomap_) {
      octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
        "/avoidance/octomap", 10);
    }

    if (publish_markers_) {
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/avoidance/markers", 10);
    }

    // Timer for VFH+ update
    auto period = std::chrono::duration<double>(1.0 / update_rate_);
    update_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalAvoidanceNode::update_callback, this));

    // Prune timer
    prune_timer_ = this->create_wall_timer(
      5s,
      std::bind(&LocalAvoidanceNode::prune_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "Local Avoidance Node initialized:\n"
      "  Update rate: %.1f Hz\n"
      "  Safety radius: %.1f m\n"
      "  Critical radius: %.1f m\n"
      "  Lookahead: %.1f m",
      update_rate_,
      vfh_config.safety_radius,
      vfh_config.critical_radius,
      vfh_config.lookahead_distance);
  }

private:
  void declare_parameters()
  {
    // Node parameters
    this->declare_parameter("update_rate", 20.0);
    this->declare_parameter("override_timeout", 0.5);
    this->declare_parameter("publish_octomap", true);
    this->declare_parameter("publish_markers", true);
    this->declare_parameter("pointcloud_topic", "/depth/points");
    this->declare_parameter("body_frame", "base_link");
    this->declare_parameter("map_frame", "odom");

    // OctoMap parameters
    this->declare_parameter("octomap.resolution", 0.5);
    this->declare_parameter("octomap.max_range", 50.0);
    this->declare_parameter("octomap.hit_probability", 0.7);
    this->declare_parameter("octomap.miss_probability", 0.4);

    // VFH parameters
    this->declare_parameter("vfh.azimuth_bins", 36);
    this->declare_parameter("vfh.elevation_bins", 18);
    this->declare_parameter("vfh.safety_radius", 5.0);
    this->declare_parameter("vfh.critical_radius", 2.0);
    this->declare_parameter("vfh.lookahead_distance", 20.0);
    this->declare_parameter("vfh.goal_weight", 5.0);
    this->declare_parameter("vfh.heading_weight", 2.0);
    this->declare_parameter("vfh.clearance_weight", 1.0);
  }

  OctomapConfig get_octomap_config()
  {
    OctomapConfig config;
    config.resolution = this->get_parameter("octomap.resolution").as_double();
    config.max_range = this->get_parameter("octomap.max_range").as_double();
    config.hit_probability = this->get_parameter("octomap.hit_probability").as_double();
    config.miss_probability = this->get_parameter("octomap.miss_probability").as_double();
    return config;
  }

  VFH3DConfig get_vfh_config()
  {
    VFH3DConfig config;
    config.azimuth_bins = this->get_parameter("vfh.azimuth_bins").as_int();
    config.elevation_bins = this->get_parameter("vfh.elevation_bins").as_int();
    config.safety_radius = this->get_parameter("vfh.safety_radius").as_double();
    config.critical_radius = this->get_parameter("vfh.critical_radius").as_double();
    config.lookahead_distance = this->get_parameter("vfh.lookahead_distance").as_double();
    config.goal_weight = this->get_parameter("vfh.goal_weight").as_double();
    config.heading_weight = this->get_parameter("vfh.heading_weight").as_double();
    config.clearance_weight = this->get_parameter("vfh.clearance_weight").as_double();
    return config;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!has_uav_state_) {
      return;
    }

    // Insert point cloud into OctoMap
    geometry_msgs::msg::Pose sensor_pose;
    sensor_pose.position = current_position_;
    sensor_pose.orientation = current_orientation_;

    octomap_manager_->insertPointCloud(*msg, sensor_pose);
    last_pointcloud_time_ = this->now();
  }

  void uav_state_callback(const flyby_msgs::msg::UAVState::SharedPtr msg)
  {
    current_position_ = msg->position;
    current_orientation_ = msg->orientation;
    current_heading_ = get_yaw_from_quaternion(msg->orientation);
    has_uav_state_ = true;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_goal_ = msg->pose.position;
    has_goal_ = true;
  }

  void update_callback()
  {
    if (!has_uav_state_ || !has_goal_) {
      return;
    }

    auto start = this->now();

    // Run VFH+ algorithm
    auto result = vfh_->computeAvoidance(current_position_, current_heading_, current_goal_);

    auto processing_time = (this->now() - start).seconds() * 1000;

    // Publish obstacle map status
    publish_obstacle_map(result, processing_time);

    // Handle result
    if (result.emergency_stop) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Emergency stop! Closest obstacle: %.2f m", result.closest_obstacle);
      publish_stop_override();
    } else if (result.path_blocked) {
      // Publish modified waypoint
      publish_avoidance_waypoint(result);
    }

    // Publish visualization
    if (publish_octomap_ && octomap_pub_) {
      publish_octomap();
    }

    if (publish_markers_ && marker_pub_) {
      publish_markers(result);
    }
  }

  void prune_callback()
  {
    octomap_manager_->prune();
  }

  void publish_obstacle_map(const VFHResult & result, double processing_time)
  {
    flyby_msgs::msg::ObstacleMap msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_parameter("body_frame").as_string();

    msg.closest_obstacle_distance = result.closest_obstacle;
    msg.obstacle_detected = result.closest_obstacle < vfh_->getConfig().lookahead_distance;
    msg.path_blocked = result.path_blocked;
    msg.recommended_heading = result.recommended_yaw;
    msg.recommended_pitch = result.recommended_pitch;
    msg.recommendation_confidence = result.confidence;

    msg.goal_direction = result.goal_direction;
    msg.goal_distance = result.goal_distance;

    msg.histogram_azimuth_bins = vfh_->getConfig().azimuth_bins;
    msg.histogram_elevation_bins = vfh_->getConfig().elevation_bins;
    msg.blocked_bins = result.blocked_bins;
    msg.candidate_directions = result.candidate_directions;

    // Safety level
    if (result.emergency_stop) {
      msg.safety_level = flyby_msgs::msg::ObstacleMap::SAFETY_CRITICAL;
    } else if (result.closest_obstacle < vfh_->getConfig().safety_radius) {
      msg.safety_level = flyby_msgs::msg::ObstacleMap::SAFETY_WARNING;
    } else if (result.path_blocked) {
      msg.safety_level = flyby_msgs::msg::ObstacleMap::SAFETY_CAUTION;
    } else {
      msg.safety_level = flyby_msgs::msg::ObstacleMap::SAFETY_CLEAR;
    }

    msg.octomap_resolution = octomap_manager_->getConfig().resolution;
    msg.octomap_size = static_cast<int>(octomap_manager_->getNumOccupiedVoxels());
    msg.processing_time_ms = processing_time;

    obstacle_map_pub_->publish(msg);
  }

  void publish_avoidance_waypoint(const VFHResult & result)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_parameter("body_frame").as_string();

    // Compute waypoint in avoidance direction
    double avoidance_heading = current_heading_ + result.recommended_yaw;
    double waypoint_distance = 10.0;  // Place waypoint 10m ahead in avoidance direction

    msg.pose.position.x = current_position_.x +
      waypoint_distance * std::cos(avoidance_heading) * std::cos(result.recommended_pitch);
    msg.pose.position.y = current_position_.y +
      waypoint_distance * std::sin(avoidance_heading) * std::cos(result.recommended_pitch);
    msg.pose.position.z = current_position_.z +
      waypoint_distance * std::sin(result.recommended_pitch);

    // Orientation toward waypoint
    msg.pose.orientation = create_quaternion_from_yaw(avoidance_heading);

    waypoint_override_pub_->publish(msg);
  }

  void publish_stop_override()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_parameter("body_frame").as_string();

    // Current position (hover in place)
    msg.pose.position = current_position_;
    msg.pose.orientation = current_orientation_;

    waypoint_override_pub_->publish(msg);
  }

  void publish_octomap()
  {
    octomap_msgs::msg::Octomap msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_parameter("map_frame").as_string();

    auto octree = octomap_manager_->getOctree();
    if (octree) {
      octomap_msgs::fullMapToMsg(*octree, msg);
      octomap_pub_->publish(msg);
    }
  }

  void publish_markers(const VFHResult & result)
  {
    visualization_msgs::msg::MarkerArray markers;

    // Closest obstacle marker
    if (result.closest_obstacle < vfh_->getConfig().lookahead_distance) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = this->get_parameter("body_frame").as_string();
      marker.ns = "closest_obstacle";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Position at closest obstacle distance in goal direction
      marker.pose.position.x = current_position_.x +
        result.closest_obstacle * result.goal_direction.x;
      marker.pose.position.y = current_position_.y +
        result.closest_obstacle * result.goal_direction.y;
      marker.pose.position.z = current_position_.z +
        result.closest_obstacle * result.goal_direction.z;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = marker.scale.y = marker.scale.z = 1.0;

      // Color based on safety level
      if (result.emergency_stop) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else if (result.path_blocked) {
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
      } else {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }
      marker.color.a = 0.8;

      markers.markers.push_back(marker);
    }

    marker_pub_->publish(markers);
  }

  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    // Yaw from quaternion
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  geometry_msgs::msg::Quaternion create_quaternion_from_yaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2);
    q.w = std::cos(yaw / 2);
    return q;
  }

  // Components
  std::shared_ptr<OctomapManager> octomap_manager_;
  std::unique_ptr<VFH3D> vfh_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr uav_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_override_pub_;
  rclcpp::Publisher<flyby_msgs::msg::ObstacleMap>::SharedPtr obstacle_map_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr prune_timer_;

  // State
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Quaternion current_orientation_;
  double current_heading_{0.0};
  geometry_msgs::msg::Point current_goal_;
  bool has_uav_state_{false};
  bool has_goal_{false};
  rclcpp::Time last_pointcloud_time_;

  // Parameters
  double update_rate_;
  double override_timeout_;
  bool publish_octomap_;
  bool publish_markers_;
};

}  // namespace local_avoidance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_avoidance::LocalAvoidanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
