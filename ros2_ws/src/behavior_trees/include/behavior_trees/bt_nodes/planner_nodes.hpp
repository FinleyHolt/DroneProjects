#ifndef BEHAVIOR_TREES__BT_NODES__PLANNER_NODES_HPP_
#define BEHAVIOR_TREES__BT_NODES__PLANNER_NODES_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "flyby_msgs/action/plan_path.hpp"
#include "flyby_msgs/msg/mission_tasking.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "nav_msgs/msg/path.hpp"

namespace behavior_trees
{

using PlanPathAction = flyby_msgs::action::PlanPath;

/**
 * @brief Plan Path action node - requests path from global planner
 *
 * Sends a PlanPath action goal to the path planning service and waits
 * for the result. Writes the resulting path to the blackboard.
 */
class PlanPathNode : public BT::RosActionNode<PlanPathAction>
{
public:
  PlanPathNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult & result) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

private:
  double default_timeout_{2.0};  // seconds
};

/**
 * @brief Get current pose and write to blackboard
 *
 * Utility node to fetch current UAV pose and make it available
 * for other nodes via the blackboard.
 */
class GetCurrentPoseNode : public BT::SyncActionNode
{
public:
  GetCurrentPoseNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_received_{false};

  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);
};

/**
 * @brief Get home base pose from mission tasking
 *
 * Utility node to fetch home base location from current mission tasking.
 */
class GetHomeBaseNode : public BT::SyncActionNode
{
public:
  GetHomeBaseNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<flyby_msgs::msg::MissionTasking>::SharedPtr tasking_sub_;
  geometry_msgs::msg::PoseStamped home_base_;
  bool tasking_received_{false};

  void taskingCallback(const flyby_msgs::msg::MissionTasking::SharedPtr msg);
};

/**
 * @brief Get AO entry point from mission tasking
 *
 * Utility node to compute optimal entry point to the area of operations.
 */
class GetAOEntryNode : public BT::SyncActionNode
{
public:
  GetAOEntryNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<flyby_msgs::msg::MissionTasking>::SharedPtr tasking_sub_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;

  flyby_msgs::msg::MissionTasking current_tasking_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool tasking_received_{false};
  bool pose_received_{false};

  void taskingCallback(const flyby_msgs::msg::MissionTasking::SharedPtr msg);
  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);
  geometry_msgs::msg::PoseStamped computeNearestEntryPoint();
};

/**
 * @brief Get safe waypoint for NFZ recovery
 *
 * Computes the nearest safe waypoint when an NFZ violation is detected.
 */
class GetSafeWaypointNode : public BT::SyncActionNode
{
public:
  GetSafeWaypointNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp::Subscription<flyby_msgs::msg::MissionTasking>::SharedPtr tasking_sub_;

  geometry_msgs::msg::PoseStamped current_pose_;
  std::vector<geometry_msgs::msg::Polygon> nfz_polygons_;
  bool pose_received_{false};

  void stateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg);
  void taskingCallback(const flyby_msgs::msg::MissionTasking::SharedPtr msg);
  geometry_msgs::msg::PoseStamped computeSafeWaypoint();
  bool isPointInPolygon(const geometry_msgs::msg::Point & point,
                        const geometry_msgs::msg::Polygon & polygon);
};

/**
 * @brief Register all planner nodes with the BT factory
 */
void registerPlannerNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  const BT::RosNodeParams & params);

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES__BT_NODES__PLANNER_NODES_HPP_
