/**
 * @file trajectory_controller.hpp
 * @brief Trajectory following controller using Pure Pursuit + PID
 *
 * Implements path following for UAV navigation:
 * - Pure Pursuit for XY tracking
 * - PID for altitude control
 * - Rate limiting for smooth motion
 */

#ifndef PATH_PLANNING__TRAJECTORY_CONTROLLER_HPP_
#define PATH_PLANNING__TRAJECTORY_CONTROLLER_HPP_

#include <vector>
#include <memory>
#include <chrono>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace path_planning
{

/**
 * @brief Configuration for the trajectory controller
 */
struct TrajectoryControllerConfig
{
  // Pure Pursuit parameters
  double lookahead_distance{5.0};     // Lookahead distance (meters)
  double min_lookahead{2.0};          // Minimum lookahead (meters)
  double max_lookahead{15.0};         // Maximum lookahead (meters)
  double lookahead_speed_gain{0.5};   // Lookahead scaling with speed

  // Velocity limits
  double max_linear_velocity{10.0};   // Max linear velocity (m/s)
  double min_linear_velocity{1.0};    // Min linear velocity (m/s)
  double max_angular_velocity{0.5};   // Max yaw rate (rad/s)

  // PID gains for altitude
  double altitude_kp{1.0};
  double altitude_ki{0.1};
  double altitude_kd{0.2};
  double altitude_max_velocity{3.0};  // Max vertical velocity (m/s)

  // Goal tolerance
  double xy_goal_tolerance{2.0};      // XY distance to goal (meters)
  double z_goal_tolerance{1.0};       // Altitude tolerance (meters)
  double yaw_goal_tolerance{0.1};     // Yaw tolerance (radians)

  // Rate limiting
  double max_linear_accel{2.0};       // Max linear acceleration (m/s^2)
  double max_angular_accel{1.0};      // Max angular acceleration (rad/s^2)
};

/**
 * @brief Result of trajectory controller update
 */
struct ControllerOutput
{
  geometry_msgs::msg::Twist cmd_vel;
  bool goal_reached{false};
  double distance_to_goal{0.0};
  int current_waypoint_idx{0};
  double cross_track_error{0.0};
  bool using_avoidance_override{false};  // True when following avoidance waypoint
};

/**
 * @brief Avoidance override from local obstacle avoidance
 */
struct AvoidanceOverride
{
  geometry_msgs::msg::PoseStamped waypoint;
  std::chrono::steady_clock::time_point timestamp;
  double timeout_seconds{0.5};  // Override expires after this duration
  bool active{false};
};

/**
 * @brief Pure Pursuit + PID trajectory controller
 *
 * Follows a path using Pure Pursuit for XY and PID for altitude.
 * Provides smooth velocity commands with rate limiting.
 */
class TrajectoryController
{
public:
  /**
   * @brief Constructor with configuration
   * @param config Controller configuration
   */
  explicit TrajectoryController(const TrajectoryControllerConfig & config = TrajectoryControllerConfig());

  /**
   * @brief Set the path to follow
   * @param path Navigation path
   */
  void setPath(const nav_msgs::msg::Path & path);

  /**
   * @brief Clear the current path
   */
  void clearPath();

  /**
   * @brief Check if a path is currently set
   */
  bool hasPath() const { return !path_.poses.empty(); }

  /**
   * @brief Update controller and compute velocity command
   * @param current_pose Current UAV pose
   * @param dt Time since last update (seconds)
   * @return Controller output with velocity command
   */
  ControllerOutput update(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double dt);

  /**
   * @brief Reset controller state (PID integrators, etc.)
   */
  void reset();

  /**
   * @brief Get current configuration
   */
  const TrajectoryControllerConfig & getConfig() const { return config_; }

  /**
   * @brief Update configuration
   */
  void setConfig(const TrajectoryControllerConfig & config);

  /**
   * @brief Get the current path
   */
  const nav_msgs::msg::Path & getPath() const { return path_; }

  /**
   * @brief Get current waypoint index
   */
  int getCurrentWaypointIndex() const { return current_waypoint_idx_; }

  /**
   * @brief Set avoidance override waypoint
   * @param waypoint Override waypoint from local avoidance
   * @param timeout_seconds How long the override is valid (default 0.5s)
   *
   * When an avoidance override is active, the controller will navigate
   * toward this waypoint instead of following the global path.
   */
  void setAvoidanceOverride(
    const geometry_msgs::msg::PoseStamped & waypoint,
    double timeout_seconds = 0.5);

  /**
   * @brief Clear the avoidance override
   */
  void clearAvoidanceOverride();

  /**
   * @brief Check if avoidance override is currently active
   */
  bool hasActiveAvoidanceOverride() const;

private:
  /**
   * @brief Find the lookahead point on the path
   * @param current_pose Current pose
   * @param lookahead_dist Lookahead distance
   * @return Lookahead point, or nullopt if at end of path
   */
  std::optional<geometry_msgs::msg::Point> findLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double lookahead_dist);

  /**
   * @brief Compute Pure Pursuit steering
   * @param current_pose Current pose
   * @param lookahead_point Target point
   * @return Angular velocity command
   */
  double computePurePursuitSteering(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Point & lookahead_point);

  /**
   * @brief Compute PID altitude control
   * @param current_alt Current altitude
   * @param target_alt Target altitude
   * @param dt Time step
   * @return Vertical velocity command
   */
  double computeAltitudePID(double current_alt, double target_alt, double dt);

  /**
   * @brief Apply rate limiting to velocity
   * @param cmd Current command
   * @param dt Time step
   * @return Rate-limited velocity
   */
  geometry_msgs::msg::Twist applyRateLimiting(
    const geometry_msgs::msg::Twist & cmd,
    double dt);

  /**
   * @brief Compute distance to final goal
   */
  double distanceToGoal(const geometry_msgs::msg::PoseStamped & current_pose);

  /**
   * @brief Get yaw from quaternion
   */
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  double normalizeAngle(double angle);

  TrajectoryControllerConfig config_;
  nav_msgs::msg::Path path_;

  // Controller state
  int current_waypoint_idx_{0};
  geometry_msgs::msg::Twist last_cmd_;

  // PID state for altitude
  double altitude_error_integral_{0.0};
  double altitude_error_prev_{0.0};

  // Avoidance override state
  AvoidanceOverride avoidance_override_;
};

}  // namespace path_planning

#endif  // PATH_PLANNING__TRAJECTORY_CONTROLLER_HPP_
