#ifndef BEHAVIOR_TREES__BT_NODES__ACTION_NODES_HPP_
#define BEHAVIOR_TREES__BT_NODES__ACTION_NODES_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"

#include "flyby_msgs/msg/uav_state.hpp"
#include "flyby_msgs/msg/behavior_command.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace behavior_trees
{

/**
 * @brief Transit action node - follows a planned path
 *
 * Executes path following using Pure Pursuit for XY and PID for altitude.
 * Returns RUNNING while traversing, SUCCESS when goal reached, FAILURE on error.
 */
class TransitAction : public BT::StatefulActionNode
{
public:
  TransitAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;

  nav_msgs::msg::Path current_path_;
  size_t current_waypoint_idx_{0};
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_received_{false};

  // Pure pursuit parameters
  double lookahead_distance_{5.0};  // meters
  double goal_tolerance_{2.0};      // meters
  double max_linear_velocity_{5.0}; // m/s
  double max_yaw_rate_{0.5};        // rad/s

  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);
  geometry_msgs::msg::TwistStamped computePurePursuitCommand();
  bool isGoalReached();
};

/**
 * @brief Loiter action node - hover at current position
 *
 * Maintains position at specified altitude. Used as safe default behavior.
 */
class LoiterAction : public BT::StatefulActionNode
{
public:
  LoiterAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;

  geometry_msgs::msg::PoseStamped loiter_position_;
  double target_altitude_{50.0};  // meters AGL
  bool position_initialized_{false};

  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);
};

/**
 * @brief Land action node - execute landing sequence
 *
 * Descends to ground and disarms. Returns SUCCESS when landed.
 */
class LandAction : public BT::StatefulActionNode
{
public:
  LandAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<flyby_msgs::msg::BehaviorCommand>::SharedPtr behavior_cmd_pub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;

  double descent_rate_{1.0};       // m/s
  double landing_altitude_{0.5};   // meters - consider landed below this
  double current_altitude_{0.0};
  bool is_landed_{false};

  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);
};

/**
 * @brief Lawnmower pattern action - classical fallback for search
 *
 * Executes a lawnmower (boustrophedon) pattern over the specified area.
 * Tracks coverage and reports progress.
 */
class LawnmowerPatternAction : public BT::StatefulActionNode
{
public:
  LawnmowerPatternAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;

  // Pattern parameters
  std::vector<geometry_msgs::msg::Point> waypoints_;
  size_t current_waypoint_idx_{0};
  double track_spacing_{30.0};  // meters between tracks
  double search_altitude_{50.0};
  double search_speed_{5.0};
  bool track_coverage_{true};

  void generateWaypoints(const geometry_msgs::msg::Polygon & area);
  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);

  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_received_{false};
};

/**
 * @brief Orbit POI action - classical fallback for dwell
 *
 * Executes a circular orbit around the target with gimbal tracking.
 */
class OrbitPOIAction : public BT::StatefulActionNode
{
public:
  OrbitPOIAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;

  // Orbit parameters
  geometry_msgs::msg::Point target_position_;
  double orbit_radius_{30.0};      // meters
  double orbit_speed_{5.0};        // m/s
  double orbit_altitude_{50.0};    // meters AGL
  bool gimbal_track_{true};
  bool clockwise_{true};

  // State
  double current_angle_{0.0};
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_received_{false};

  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);
  geometry_msgs::msg::TwistStamped computeOrbitCommand();
};

/**
 * @brief Register all action nodes with the BT factory
 */
void registerActionNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node);

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES__BT_NODES__ACTION_NODES_HPP_
