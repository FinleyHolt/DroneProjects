#ifndef BEHAVIOR_TREES__BT_NODES__CONDITION_NODES_HPP_
#define BEHAVIOR_TREES__BT_NODES__CONDITION_NODES_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/bt_service_node.hpp"

#include "flyby_msgs/srv/ontology_query.hpp"
#include "flyby_msgs/msg/uav_state.hpp"
#include "flyby_msgs/msg/mission_tasking.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace behavior_trees
{

/**
 * @brief Base class for ontology condition nodes
 *
 * Queries the ontology service to evaluate conditions. Caches results
 * for a configurable duration to avoid excessive service calls.
 */
class OntologyConditionNode : public BT::RosServiceNode<flyby_msgs::srv::OntologyQuery>
{
public:
  OntologyConditionNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr & request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string query_type_;
  std::chrono::steady_clock::time_point last_query_time_;
  bool cached_result_{false};
  std::chrono::milliseconds cache_duration_{100};  // 100ms cache
};

/**
 * @brief Check if battery is below RTB reserve threshold
 *
 * Returns SUCCESS if battery is below reserve (should trigger RTB),
 * FAILURE otherwise.
 */
class BatteryBelowReserveCondition : public OntologyConditionNode
{
public:
  BatteryBelowReserveCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
};

/**
 * @brief Check if battery is above RTB reserve threshold
 *
 * Returns SUCCESS if battery is above reserve (can continue mission),
 * FAILURE otherwise.
 */
class BatteryAboveReserveCondition : public OntologyConditionNode
{
public:
  BatteryAboveReserveCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
};

/**
 * @brief Check if there is valid mission tasking
 *
 * Returns SUCCESS if valid tasking is available,
 * FAILURE otherwise.
 */
class HasValidTaskingCondition : public OntologyConditionNode
{
public:
  HasValidTaskingCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
};

/**
 * @brief Check if current position violates any NFZ
 *
 * Returns SUCCESS if in violation (should trigger recovery),
 * FAILURE if position is safe.
 */
class NFZViolationCondition : public OntologyConditionNode
{
public:
  NFZViolationCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
};

/**
 * @brief Check if a target has been acquired for dwell
 *
 * Returns SUCCESS if target is acquired and ready for dwell,
 * FAILURE otherwise.
 */
class TargetAcquiredCondition : public OntologyConditionNode
{
public:
  TargetAcquiredCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
};

/**
 * @brief Check if RL search policy execution is allowed
 *
 * Returns SUCCESS if search policy can be executed,
 * FAILURE if conditions prevent RL execution.
 */
class RLSearchAllowedCondition : public OntologyConditionNode
{
public:
  RLSearchAllowedCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
};

/**
 * @brief Check if RL dwell policy execution is allowed
 *
 * Returns SUCCESS if dwell policy can be executed,
 * FAILURE if conditions prevent RL execution.
 */
class RLDwellAllowedCondition : public OntologyConditionNode
{
public:
  RLDwellAllowedCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
};

/**
 * @brief Register all condition nodes with the BT factory
 */
void registerConditionNodes(
  BT::BehaviorTreeFactory & factory,
  const BT::RosNodeParams & params);

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES__BT_NODES__CONDITION_NODES_HPP_
