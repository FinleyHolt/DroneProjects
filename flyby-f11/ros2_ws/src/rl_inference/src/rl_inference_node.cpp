/**
 * @file rl_inference_node.cpp
 * @brief ROS2 node for RL policy inference
 *
 * Provides inference services for SearchPolicy and DwellPolicy.
 * Subscribes to UAV state and policy context, publishes actions.
 */

#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "flyby_msgs/msg/uav_state.hpp"
#include "flyby_msgs/msg/rl_policy_context.hpp"

#include "rl_inference/tensorrt_policy.hpp"
#include "rl_inference/policy_interface.hpp"

using namespace std::chrono_literals;

namespace rl_inference
{

class RLInferenceNode : public rclcpp::Node
{
public:
  RLInferenceNode()
  : Node("rl_inference")
  {
    // Declare parameters
    this->declare_parameter<std::string>("search_policy_engine", "");
    this->declare_parameter<std::string>("dwell_policy_engine", "");
    this->declare_parameter<double>("max_inference_rate_hz", 50.0);
    this->declare_parameter<double>("action_timeout_ms", 200.0);

    // Get parameters
    search_engine_path_ = this->get_parameter("search_policy_engine").as_string();
    dwell_engine_path_ = this->get_parameter("dwell_policy_engine").as_string();
    max_inference_rate_ = this->get_parameter("max_inference_rate_hz").as_double();
    action_timeout_ms_ = this->get_parameter("action_timeout_ms").as_double();

    // Load policies if paths provided
    if (!search_engine_path_.empty()) {
      search_policy_ = createPolicy(search_engine_path_, PolicyType::SEARCH);
      if (search_policy_) {
        RCLCPP_INFO(this->get_logger(), "SearchPolicy loaded from: %s",
          search_engine_path_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load SearchPolicy");
      }
    }

    if (!dwell_engine_path_.empty()) {
      dwell_policy_ = createPolicy(dwell_engine_path_, PolicyType::DWELL);
      if (dwell_policy_) {
        RCLCPP_INFO(this->get_logger(), "DwellPolicy loaded from: %s",
          dwell_engine_path_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load DwellPolicy");
      }
    }

    // Subscriptions
    state_sub_ = this->create_subscription<flyby_msgs::msg::UAVState>(
      "/flyby/uav_state", 10,
      std::bind(&RLInferenceNode::onState, this, std::placeholders::_1));

    context_sub_ = this->create_subscription<flyby_msgs::msg::RLPolicyContext>(
      "/flyby/rl_context", 10,
      std::bind(&RLInferenceNode::onContext, this, std::placeholders::_1));

    // Publishers
    search_action_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/flyby/rl/search_action", 10);

    dwell_action_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/flyby/rl/dwell_action", 10);

    // Inference timer
    auto inference_period = std::chrono::duration<double>(1.0 / max_inference_rate_);
    inference_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(inference_period),
      std::bind(&RLInferenceNode::inferenceCallback, this));

    RCLCPP_INFO(this->get_logger(), "RL Inference Node initialized at %.1f Hz",
      max_inference_rate_);
  }

private:
  void onState(const flyby_msgs::msg::UAVState::SharedPtr msg)
  {
    last_state_ = msg;
  }

  void onContext(const flyby_msgs::msg::RLPolicyContext::SharedPtr msg)
  {
    last_context_ = msg;
  }

  void inferenceCallback()
  {
    if (!last_state_ || !last_context_) {
      return;
    }

    // Check if context indicates active policy
    if (!last_context_->is_active) {
      return;
    }

    // Determine which policy to run based on policy_type in context
    if (last_context_->policy_type == "search" && search_policy_ && search_policy_->isReady()) {
      runSearchInference();
    } else if (last_context_->policy_type == "dwell" && dwell_policy_ && dwell_policy_->isReady()) {
      runDwellInference();
    }
  }

  void runSearchInference()
  {
    // Build observation for search policy (40 dims)
    auto obs = buildSearchObservation();

    // Run inference
    std::vector<float> action;
    if (!search_policy_->infer(obs, action)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "SearchPolicy inference failed");
      return;
    }

    // Convert to TwistStamped (4 DOF: vx, vy, vz, yaw_rate)
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    // Scale actions from [-1, 1] to velocity units
    cmd.twist.linear.x = action[0] * 8.0;   // max 8 m/s forward
    cmd.twist.linear.y = action[1] * 8.0;   // max 8 m/s lateral
    cmd.twist.linear.z = action[2] * 3.0;   // max 3 m/s vertical
    cmd.twist.angular.z = action[3] * 0.5;  // max 0.5 rad/s yaw

    search_action_pub_->publish(cmd);

    // Log stats periodically
    static int search_count = 0;
    if (++search_count % 250 == 0) {
      RCLCPP_INFO(this->get_logger(), "SearchPolicy: %.2f ms (avg: %.2f ms)",
        search_policy_->getStats().last_inference_ms,
        search_policy_->getStats().avg_inference_ms);
    }
  }

  void runDwellInference()
  {
    // Build observation for dwell policy (25 dims)
    auto obs = buildDwellObservation();

    // Run inference
    std::vector<float> action;
    if (!dwell_policy_->infer(obs, action)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "DwellPolicy inference failed");
      return;
    }

    // Convert to TwistStamped (6 DOF: vx, vy, vz, yaw_rate, gimbal_pan, gimbal_tilt)
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    // Scale actions from [-1, 1] to velocity units
    cmd.twist.linear.x = action[0] * 5.0;    // max 5 m/s forward (slower for tracking)
    cmd.twist.linear.y = action[1] * 5.0;    // max 5 m/s lateral
    cmd.twist.linear.z = action[2] * 2.0;    // max 2 m/s vertical
    cmd.twist.angular.z = action[3] * 0.3;   // max 0.3 rad/s yaw (slower)

    // Gimbal commands encoded in angular x and y (repurposed)
    // These will be interpreted by gimbal controller
    cmd.twist.angular.x = action[4] * 0.785;  // gimbal pan rate (45 deg/s max)
    cmd.twist.angular.y = action[5] * 0.785;  // gimbal tilt rate (45 deg/s max)

    dwell_action_pub_->publish(cmd);

    // Log stats periodically
    static int dwell_count = 0;
    if (++dwell_count % 250 == 0) {
      RCLCPP_INFO(this->get_logger(), "DwellPolicy: %.2f ms (avg: %.2f ms)",
        dwell_policy_->getStats().last_inference_ms,
        dwell_policy_->getStats().avg_inference_ms);
    }
  }

  std::vector<float> buildSearchObservation()
  {
    // Build 40-dim observation for SearchPolicy
    std::vector<float> obs(40, 0.0f);

    const auto & state = *last_state_;
    const auto & ctx = *last_context_;

    // Drone state (0-17)
    // Position (normalized by 500m)
    obs[0] = state.position.x / 500.0f;
    obs[1] = state.position.y / 500.0f;
    obs[2] = state.altitude / 500.0f;

    // Velocity (normalized by 20 m/s)
    obs[3] = state.velocity.x / 20.0f;
    obs[4] = state.velocity.y / 20.0f;
    obs[5] = state.velocity.z / 20.0f;

    // Rotation matrix from quaternion (9 elements)
    float qx = state.orientation.x;
    float qy = state.orientation.y;
    float qz = state.orientation.z;
    float qw = state.orientation.w;

    obs[6] = 1 - 2 * (qy * qy + qz * qz);
    obs[7] = 2 * (qx * qy - qz * qw);
    obs[8] = 2 * (qx * qz + qy * qw);
    obs[9] = 2 * (qx * qy + qz * qw);
    obs[10] = 1 - 2 * (qx * qx + qz * qz);
    obs[11] = 2 * (qy * qz - qx * qw);
    obs[12] = 2 * (qx * qz - qy * qw);
    obs[13] = 2 * (qy * qz + qx * qw);
    obs[14] = 1 - 2 * (qx * qx + qy * qy);

    // Angular velocity (normalized by 10 rad/s)
    obs[15] = state.angular_velocity.x / 10.0f;
    obs[16] = state.angular_velocity.y / 10.0f;
    obs[17] = state.angular_velocity.z / 10.0f;

    // Coverage (18)
    obs[18] = ctx.coverage_pct / 100.0f;

    // Frontier centroids (19-28) - from search_area
    // Simplified: use search area bounds as proxy
    if (!ctx.frontier_points.empty()) {
      for (size_t i = 0; i < std::min(size_t(5), ctx.frontier_points.size()); ++i) {
        obs[19 + i * 2] = ctx.frontier_points[i].x / 200.0f;
        obs[20 + i * 2] = ctx.frontier_points[i].y / 200.0f;
      }
    }

    // Nearby uncovered ratio (29) - approximated
    obs[29] = 1.0f - ctx.coverage_pct / 100.0f;

    // Battery (30)
    obs[30] = ctx.battery_pct / 100.0f;

    // Mission time ratio (31)
    obs[31] = ctx.flight_time_sec / 300.0f;  // Assume 5 min max

    // Home direction (32-33)
    float home_dx = -state.position.x;
    float home_dy = -state.position.y;
    float home_dist = std::sqrt(home_dx * home_dx + home_dy * home_dy);
    if (home_dist > 1e-6) {
      obs[32] = home_dx / home_dist;
      obs[33] = home_dy / home_dist;
    }

    // Home distance (34)
    obs[34] = home_dist / 500.0f;

    // Targets detected ratio (35)
    obs[35] = static_cast<float>(ctx.targets_detected) / 10.0f;

    // Exploration direction (36-37) - toward uncovered area
    // Simplified: away from current position
    obs[36] = -obs[32];
    obs[37] = -obs[33];

    // Velocity features (38-39)
    obs[38] = std::sqrt(obs[3] * obs[3] + obs[4] * obs[4]) * 20.0f / 10.0f;  // Speed
    obs[39] = std::cos(state.yaw);  // Heading alignment

    return obs;
  }

  std::vector<float> buildDwellObservation()
  {
    // Build 25-dim observation for DwellPolicy
    std::vector<float> obs(25, 0.0f);

    const auto & state = *last_state_;
    const auto & ctx = *last_context_;

    // Target relative position (0-2)
    float rel_x = ctx.current_target.position.pose.position.x - state.position.x;
    float rel_y = ctx.current_target.position.pose.position.y - state.position.y;
    float rel_z = ctx.current_target.position.pose.position.z - state.altitude;
    obs[0] = rel_x / 100.0f;
    obs[1] = rel_y / 100.0f;
    obs[2] = rel_z / 100.0f;

    // Target relative velocity (3-5)
    obs[3] = (ctx.current_target.velocity.x - state.velocity.x) / 10.0f;
    obs[4] = (ctx.current_target.velocity.y - state.velocity.y) / 10.0f;
    obs[5] = (ctx.current_target.velocity.z - state.velocity.z) / 10.0f;

    // Target image position (6-7) - normalized [-1, 1]
    obs[6] = ctx.target_image_position.x;  // Assumed already normalized
    obs[7] = ctx.target_image_position.y;

    // Target distance and angle (8-9)
    float target_dist = std::sqrt(rel_x * rel_x + rel_y * rel_y + rel_z * rel_z);
    obs[8] = target_dist / 100.0f;
    obs[9] = std::atan2(rel_y, rel_x) / M_PI;

    // Drone velocity (10-12)
    obs[10] = state.velocity.x / 10.0f;
    obs[11] = state.velocity.y / 10.0f;
    obs[12] = state.velocity.z / 10.0f;

    // Drone altitude (13)
    obs[13] = state.altitude / 100.0f;

    // Drone yaw rate (14)
    obs[14] = state.angular_velocity.z / 2.0f;

    // Gimbal angles (15-16)
    obs[15] = ctx.gimbal_pan_rad / M_PI;
    obs[16] = ctx.gimbal_tilt_rad / M_PI;

    // Previous gimbal command (17-18) - stored from last iteration
    obs[17] = last_gimbal_cmd_[0];
    obs[18] = last_gimbal_cmd_[1];

    // Track quality (19)
    obs[19] = ctx.track_confidence;

    // Target in frame (20)
    obs[20] = (ctx.track_confidence > 0.1f) ? 1.0f : 0.0f;

    // Dwell progress (21)
    float dwell_progress = 1.0f - (ctx.dwell_time_remaining_sec /
      ctx.current_target.required_dwell_time_sec);
    obs[21] = std::clamp(dwell_progress, 0.0f, 1.0f);

    // Time since good track (22) - approximated from confidence
    obs[22] = (1.0f - ctx.track_confidence) * 5.0f / 5.0f;

    // Distance to optimal tracking position (23-24)
    float optimal_dist = 50.0f;  // meters
    float dist_error = target_dist - optimal_dist;
    obs[23] = dist_error / 50.0f;
    obs[24] = 0.0f;  // Lateral error (simplified)

    return obs;
  }

  // Policy engines
  std::unique_ptr<TensorRTPolicy> search_policy_;
  std::unique_ptr<TensorRTPolicy> dwell_policy_;

  // Parameters
  std::string search_engine_path_;
  std::string dwell_engine_path_;
  double max_inference_rate_{50.0};
  double action_timeout_ms_{200.0};

  // Subscriptions
  rclcpp::Subscription<flyby_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp::Subscription<flyby_msgs::msg::RLPolicyContext>::SharedPtr context_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr search_action_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr dwell_action_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr inference_timer_;

  // State
  flyby_msgs::msg::UAVState::SharedPtr last_state_;
  flyby_msgs::msg::RLPolicyContext::SharedPtr last_context_;
  float last_gimbal_cmd_[2] = {0.0f, 0.0f};
};

}  // namespace rl_inference

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rl_inference::RLInferenceNode>());
  rclcpp::shutdown();
  return 0;
}
