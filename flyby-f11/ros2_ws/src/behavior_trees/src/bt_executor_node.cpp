/**
 * @file bt_executor_node.cpp
 * @brief Main Behavior Tree executor node for ISR missions
 *
 * Ticks the behavior tree at 50Hz (20ms period) to ensure responsive
 * control. Handles tree loading, blackboard setup, and node registration.
 */

#include <chrono>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_ros2/ros_node_params.hpp"

#include "behavior_trees/bt_nodes/condition_nodes.hpp"
#include "behavior_trees/bt_nodes/action_nodes.hpp"
#include "behavior_trees/bt_nodes/rl_action_nodes.hpp"
#include "behavior_trees/bt_nodes/planner_nodes.hpp"

#include "flyby_msgs/msg/uav_state.hpp"
#include "flyby_msgs/msg/mission_tasking.hpp"

using namespace std::chrono_literals;

namespace behavior_trees
{

class BTExecutorNode : public rclcpp::Node
{
public:
  BTExecutorNode()
  : Node("bt_executor")
  {
    // Declare parameters
    this->declare_parameter<std::string>("tree_file", "");
    this->declare_parameter<double>("tick_rate_hz", 50.0);
    this->declare_parameter<bool>("enable_logging", false);
    this->declare_parameter<std::string>("ontology_service", "/flyby/ontology_query");
    this->declare_parameter<std::string>("plan_path_action", "/flyby/plan_path");

    // Get parameters
    tree_file_ = this->get_parameter("tree_file").as_string();
    tick_rate_hz_ = this->get_parameter("tick_rate_hz").as_double();
    enable_logging_ = this->get_parameter("enable_logging").as_bool();

    if (tree_file_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No tree_file parameter specified");
      throw std::runtime_error("tree_file parameter required");
    }

    // Setup ROS node params for BT nodes
    ros_params_.nh = this->shared_from_this();
    ros_params_.default_port_value = this->get_parameter("ontology_service").as_string();
    ros_params_.server_timeout = std::chrono::milliseconds(1000);
    ros_params_.wait_for_server_timeout = std::chrono::milliseconds(500);

    // Create subscriptions for blackboard population
    uav_state_sub_ = this->create_subscription<flyby_msgs::msg::UAVState>(
      "/flyby/uav_state", 10,
      std::bind(&BTExecutorNode::uavStateCallback, this, std::placeholders::_1));

    mission_tasking_sub_ = this->create_subscription<flyby_msgs::msg::MissionTasking>(
      "/flyby/mission_tasking", 10,
      std::bind(&BTExecutorNode::missionTaskingCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "BT Executor initializing...");
  }

  void initialize()
  {
    // Register all BT node types
    registerAllNodes();

    // Load the behavior tree
    loadTree();

    // Setup logging if enabled
    if (enable_logging_) {
      logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
    }

    // Calculate tick period
    auto tick_period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / tick_rate_hz_));

    RCLCPP_INFO(this->get_logger(), "Starting BT execution at %.1f Hz (period: %ld ms)",
      tick_rate_hz_, tick_period_ms.count());

    // Create tick timer
    tick_timer_ = this->create_wall_timer(
      tick_period_ms,
      std::bind(&BTExecutorNode::tickTree, this));
  }

private:
  void registerAllNodes()
  {
    RCLCPP_INFO(this->get_logger(), "Registering BT nodes...");

    // Get node pointer for nodes that need direct ROS access
    auto node_ptr = this->shared_from_this();

    // Register condition nodes (use ontology service)
    registerConditionNodes(factory_, ros_params_);

    // Register action nodes
    registerActionNodes(factory_, node_ptr);

    // Register RL action nodes
    registerRLActionNodes(factory_, node_ptr);

    // Register planner nodes
    registerPlannerNodes(factory_, node_ptr, ros_params_);

    RCLCPP_INFO(this->get_logger(), "All BT nodes registered");
  }

  void loadTree()
  {
    RCLCPP_INFO(this->get_logger(), "Loading behavior tree from: %s", tree_file_.c_str());

    // Check file exists
    std::ifstream file(tree_file_);
    if (!file.good()) {
      RCLCPP_ERROR(this->get_logger(), "Tree file not found: %s", tree_file_.c_str());
      throw std::runtime_error("Tree file not found");
    }

    // Create the tree
    tree_ = factory_.createTreeFromFile(tree_file_);

    // Get blackboard for population
    blackboard_ = tree_.rootBlackboard();

    // Initialize blackboard with default values
    initializeBlackboard();

    RCLCPP_INFO(this->get_logger(), "Behavior tree loaded successfully");
  }

  void initializeBlackboard()
  {
    // Initialize pose to origin
    geometry_msgs::msg::PoseStamped default_pose;
    default_pose.header.frame_id = "map";
    default_pose.pose.position.x = 0.0;
    default_pose.pose.position.y = 0.0;
    default_pose.pose.position.z = 0.0;
    default_pose.pose.orientation.w = 1.0;
    blackboard_->set("current_pose", default_pose);

    // Initialize empty path
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = "map";
    blackboard_->set("planned_path", empty_path);

    // Initialize battery percentage
    blackboard_->set("battery_pct", 100.0);

    // Initialize mission active flag
    blackboard_->set("mission_active", false);

    RCLCPP_DEBUG(this->get_logger(), "Blackboard initialized with defaults");
  }

  void uavStateCallback(const flyby_msgs::msg::UAVState::SharedPtr msg)
  {
    if (!blackboard_) {
      return;
    }

    // Update current pose on blackboard
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position.x = msg->position.x;
    pose.pose.position.y = msg->position.y;
    pose.pose.position.z = msg->altitude;
    pose.pose.orientation = msg->orientation;
    blackboard_->set("current_pose", pose);

    // Update battery
    blackboard_->set("battery_pct", static_cast<double>(msg->battery_percent));

    // Update other useful state
    blackboard_->set("armed", msg->armed);
    blackboard_->set("in_air", msg->in_air);
    blackboard_->set("comms_ok", msg->comms_ok);
    blackboard_->set("localization_ok", msg->localization_ok);
  }

  void missionTaskingCallback(const flyby_msgs::msg::MissionTasking::SharedPtr msg)
  {
    if (!blackboard_) {
      return;
    }

    // Store full mission tasking on blackboard
    blackboard_->set("mission_tasking", *msg);
    blackboard_->set("mission_active", msg->is_active);

    // Store derived values for convenience
    blackboard_->set("home_base", msg->home_base);

    // Compute AO bounds as simplified min/max for quick checks
    if (!msg->area_of_operations.boundary.points.empty()) {
      const auto & ao = msg->area_of_operations;
      blackboard_->set("ao_bounds", ao.boundary);
      blackboard_->set("ao_min_alt", ao.min_altitude_m);
      blackboard_->set("ao_max_alt", ao.max_altitude_m);
    }

    RCLCPP_INFO(this->get_logger(), "Mission tasking received: %s (active: %s)",
      msg->mission_id.c_str(), msg->is_active ? "yes" : "no");
  }

  void tickTree()
  {
    // Track timing for performance monitoring
    auto start_time = this->now();

    // Tick the tree
    BT::NodeStatus status = tree_.tickOnce();

    auto elapsed = (this->now() - start_time).nanoseconds() / 1e6;  // ms

    // Log status changes
    if (status != last_status_) {
      const char * status_str = (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" :
        (status == BT::NodeStatus::FAILURE) ? "FAILURE" :
        (status == BT::NodeStatus::RUNNING) ? "RUNNING" : "IDLE";

      RCLCPP_INFO(this->get_logger(), "BT status changed: %s", status_str);
      last_status_ = status;
    }

    // Warn if tick takes too long
    double budget_ms = 1000.0 / tick_rate_hz_;
    if (elapsed > budget_ms * 0.8) {
      RCLCPP_WARN(this->get_logger(), "BT tick took %.2f ms (budget: %.2f ms)",
        elapsed, budget_ms);
    }

    // Publish tick statistics periodically
    tick_count_++;
    if (tick_count_ % 500 == 0) {
      RCLCPP_INFO(this->get_logger(), "BT tick #%zu, last tick: %.2f ms",
        tick_count_, elapsed);
    }
  }

  // Parameters
  std::string tree_file_;
  double tick_rate_hz_{50.0};
  bool enable_logging_{false};

  // BT components
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  BT::RosNodeParams ros_params_;
  std::unique_ptr<BT::StdCoutLogger> logger_;

  // ROS subscriptions
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr uav_state_sub_;
  rclcpp::Subscription<flyby_msgs::msg::MissionTasking>::SharedPtr mission_tasking_sub_;

  // Tick timer
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // State tracking
  BT::NodeStatus last_status_{BT::NodeStatus::IDLE};
  size_t tick_count_{0};
};

}  // namespace behavior_trees

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<behavior_trees::BTExecutorNode>();

  // Initialize after construction to allow shared_from_this()
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
