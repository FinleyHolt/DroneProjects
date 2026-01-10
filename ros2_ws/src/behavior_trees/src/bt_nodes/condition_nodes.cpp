#include "behavior_trees/bt_nodes/condition_nodes.hpp"

namespace behavior_trees
{

// =============================================================================
// OntologyConditionNode (Base Class)
// =============================================================================

OntologyConditionNode::OntologyConditionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosServiceNode<flyby_msgs::srv::OntologyQuery>(name, config, params)
{
  // Get query type from port or derived class
  getInput("query_type", query_type_);

  // Get cache duration from port (optional)
  int cache_ms = 100;
  getInput("cache_duration_ms", cache_ms);
  cache_duration_ = std::chrono::milliseconds(cache_ms);
}

BT::PortsList OntologyConditionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("query_type", "Type of ontology query"),
    BT::InputPort<std::string>("action_name", "", "Action being queried (optional)"),
    BT::InputPort<int>("cache_duration_ms", 100, "Result cache duration in ms"),
    BT::InputPort<geometry_msgs::msg::Point>("current_position", "Current UAV position"),
    BT::OutputPort<std::string>("explanation", "Explanation of query result"),
  };
}

bool OntologyConditionNode::setRequest(Request::SharedPtr & request)
{
  // Check cache first
  auto now = std::chrono::steady_clock::now();
  if (now - last_query_time_ < cache_duration_) {
    // Use cached result - skip service call
    return false;
  }

  request->query_type = query_type_;

  std::string action_name;
  if (getInput("action_name", action_name)) {
    request->action_name = action_name;
  }

  geometry_msgs::msg::Point position;
  if (getInput("current_position", position)) {
    request->current_position = position;
  }

  return true;
}

BT::NodeStatus OntologyConditionNode::onResponseReceived(
  const Response::SharedPtr & response)
{
  // Cache the result
  last_query_time_ = std::chrono::steady_clock::now();
  cached_result_ = response->result;

  // Set output port
  setOutput("explanation", response->explanation);

  // Return SUCCESS if condition is satisfied, FAILURE otherwise
  return response->result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus OntologyConditionNode::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_WARN(
    node_->get_logger(),
    "Ontology query '%s' failed with error code %d, using cached result: %s",
    query_type_.c_str(),
    static_cast<int>(error),
    cached_result_ ? "true" : "false");

  // On failure, use cached result if available
  return cached_result_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// =============================================================================
// BatteryBelowReserveCondition
// =============================================================================

BatteryBelowReserveCondition::BatteryBelowReserveCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: OntologyConditionNode(name, config, params)
{
  query_type_ = "battery_below_reserve";
}

BT::PortsList BatteryBelowReserveCondition::providedPorts()
{
  auto ports = OntologyConditionNode::providedPorts();
  // Add battery-specific ports
  ports.insert(BT::InputPort<double>("reserve_threshold", 20.0, "Battery reserve threshold %"));
  return ports;
}

// =============================================================================
// BatteryAboveReserveCondition
// =============================================================================

BatteryAboveReserveCondition::BatteryAboveReserveCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: OntologyConditionNode(name, config, params)
{
  query_type_ = "battery_above_reserve";
}

BT::PortsList BatteryAboveReserveCondition::providedPorts()
{
  auto ports = OntologyConditionNode::providedPorts();
  ports.insert(BT::InputPort<double>("reserve_threshold", 20.0, "Battery reserve threshold %"));
  return ports;
}

// =============================================================================
// HasValidTaskingCondition
// =============================================================================

HasValidTaskingCondition::HasValidTaskingCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: OntologyConditionNode(name, config, params)
{
  query_type_ = "has_valid_tasking";
}

BT::PortsList HasValidTaskingCondition::providedPorts()
{
  return OntologyConditionNode::providedPorts();
}

// =============================================================================
// NFZViolationCondition
// =============================================================================

NFZViolationCondition::NFZViolationCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: OntologyConditionNode(name, config, params)
{
  query_type_ = "nfz_violation";
}

BT::PortsList NFZViolationCondition::providedPorts()
{
  auto ports = OntologyConditionNode::providedPorts();
  ports.insert(BT::InputPort<double>("safety_margin", 10.0, "NFZ safety margin in meters"));
  return ports;
}

// =============================================================================
// TargetAcquiredCondition
// =============================================================================

TargetAcquiredCondition::TargetAcquiredCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: OntologyConditionNode(name, config, params)
{
  query_type_ = "target_acquired";
}

BT::PortsList TargetAcquiredCondition::providedPorts()
{
  auto ports = OntologyConditionNode::providedPorts();
  ports.insert(BT::InputPort<double>("min_confidence", 0.5, "Minimum detection confidence"));
  ports.insert(BT::OutputPort<std::string>("target_id", "ID of acquired target"));
  return ports;
}

// =============================================================================
// RLSearchAllowedCondition
// =============================================================================

RLSearchAllowedCondition::RLSearchAllowedCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: OntologyConditionNode(name, config, params)
{
  query_type_ = "rl_search_allowed";
}

BT::PortsList RLSearchAllowedCondition::providedPorts()
{
  return OntologyConditionNode::providedPorts();
}

// =============================================================================
// RLDwellAllowedCondition
// =============================================================================

RLDwellAllowedCondition::RLDwellAllowedCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: OntologyConditionNode(name, config, params)
{
  query_type_ = "rl_dwell_allowed";
}

BT::PortsList RLDwellAllowedCondition::providedPorts()
{
  return OntologyConditionNode::providedPorts();
}

// =============================================================================
// Registration
// =============================================================================

void registerConditionNodes(
  BT::BehaviorTreeFactory & factory,
  const BT::RosNodeParams & params)
{
  factory.registerNodeType<BatteryBelowReserveCondition>(
    "BatteryBelowReserve", params);

  factory.registerNodeType<BatteryAboveReserveCondition>(
    "BatteryAboveReserve", params);

  factory.registerNodeType<HasValidTaskingCondition>(
    "HasValidTasking", params);

  factory.registerNodeType<NFZViolationCondition>(
    "NFZViolation", params);

  factory.registerNodeType<TargetAcquiredCondition>(
    "TargetAcquired", params);

  factory.registerNodeType<RLSearchAllowedCondition>(
    "RLSearchAllowed", params);

  factory.registerNodeType<RLDwellAllowedCondition>(
    "RLDwellAllowed", params);

  // Generic ontology condition for custom queries
  factory.registerNodeType<OntologyConditionNode>(
    "OntologyCondition", params);
}

}  // namespace behavior_trees
