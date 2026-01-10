/**
 * @file trajectory_controller.cpp
 * @brief Implementation of Pure Pursuit + PID trajectory controller
 */

#include "path_planning/trajectory_controller.hpp"

#include <cmath>
#include <algorithm>
#include <optional>

namespace path_planning
{

TrajectoryController::TrajectoryController(const TrajectoryControllerConfig & config)
: config_(config)
{
}

void TrajectoryController::setPath(const nav_msgs::msg::Path & path)
{
  path_ = path;
  current_waypoint_idx_ = 0;
  reset();
}

void TrajectoryController::clearPath()
{
  path_.poses.clear();
  current_waypoint_idx_ = 0;
  reset();
}

void TrajectoryController::reset()
{
  altitude_error_integral_ = 0.0;
  altitude_error_prev_ = 0.0;
  last_cmd_ = geometry_msgs::msg::Twist();
  avoidance_override_.active = false;
}

void TrajectoryController::setAvoidanceOverride(
  const geometry_msgs::msg::PoseStamped & waypoint,
  double timeout_seconds)
{
  avoidance_override_.waypoint = waypoint;
  avoidance_override_.timestamp = std::chrono::steady_clock::now();
  avoidance_override_.timeout_seconds = timeout_seconds;
  avoidance_override_.active = true;
}

void TrajectoryController::clearAvoidanceOverride()
{
  avoidance_override_.active = false;
}

bool TrajectoryController::hasActiveAvoidanceOverride() const
{
  if (!avoidance_override_.active) {
    return false;
  }

  // Check if override has expired
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration<double>(now - avoidance_override_.timestamp).count();
  return elapsed < avoidance_override_.timeout_seconds;
}

void TrajectoryController::setConfig(const TrajectoryControllerConfig & config)
{
  config_ = config;
}

double TrajectoryController::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  // Extract yaw from quaternion
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double TrajectoryController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double TrajectoryController::distanceToGoal(const geometry_msgs::msg::PoseStamped & current_pose)
{
  if (path_.poses.empty()) {
    return 0.0;
  }

  const auto & goal = path_.poses.back().pose.position;
  const auto & pos = current_pose.pose.position;

  return std::sqrt(
    std::pow(goal.x - pos.x, 2) +
    std::pow(goal.y - pos.y, 2) +
    std::pow(goal.z - pos.z, 2));
}

std::optional<geometry_msgs::msg::Point> TrajectoryController::findLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double lookahead_dist)
{
  if (path_.poses.empty()) {
    return std::nullopt;
  }

  const auto & pos = current_pose.pose.position;

  // Find the closest point on the path
  double min_dist = std::numeric_limits<double>::max();
  int closest_idx = current_waypoint_idx_;

  for (size_t i = current_waypoint_idx_; i < path_.poses.size(); ++i) {
    const auto & wp = path_.poses[i].pose.position;
    double dist = std::sqrt(
      std::pow(wp.x - pos.x, 2) +
      std::pow(wp.y - pos.y, 2));

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = static_cast<int>(i);
    }
  }

  // Update current waypoint index (don't go backwards)
  current_waypoint_idx_ = std::max(current_waypoint_idx_, closest_idx);

  // Find lookahead point
  double accumulated_dist = 0.0;
  for (size_t i = current_waypoint_idx_; i < path_.poses.size() - 1; ++i) {
    const auto & p1 = path_.poses[i].pose.position;
    const auto & p2 = path_.poses[i + 1].pose.position;

    double segment_length = std::sqrt(
      std::pow(p2.x - p1.x, 2) +
      std::pow(p2.y - p1.y, 2));

    if (accumulated_dist + segment_length >= lookahead_dist) {
      // Interpolate along this segment
      double remaining = lookahead_dist - accumulated_dist;
      double t = remaining / segment_length;
      t = std::clamp(t, 0.0, 1.0);

      geometry_msgs::msg::Point lookahead;
      lookahead.x = p1.x + t * (p2.x - p1.x);
      lookahead.y = p1.y + t * (p2.y - p1.y);
      lookahead.z = p1.z + t * (p2.z - p1.z);
      return lookahead;
    }

    accumulated_dist += segment_length;
  }

  // Return last point if lookahead extends beyond path
  return path_.poses.back().pose.position;
}

double TrajectoryController::computePurePursuitSteering(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Point & lookahead_point)
{
  const auto & pos = current_pose.pose.position;
  double current_yaw = getYawFromQuaternion(current_pose.pose.orientation);

  // Compute vector to lookahead point
  double dx = lookahead_point.x - pos.x;
  double dy = lookahead_point.y - pos.y;

  // Compute desired heading
  double desired_yaw = std::atan2(dy, dx);

  // Compute heading error
  double yaw_error = normalizeAngle(desired_yaw - current_yaw);

  // Pure pursuit curvature
  double L = std::sqrt(dx * dx + dy * dy);
  if (L < 0.01) {
    return 0.0;
  }

  // Angular velocity proportional to heading error
  double angular_vel = 2.0 * yaw_error;

  // Clamp to limits
  return std::clamp(angular_vel, -config_.max_angular_velocity, config_.max_angular_velocity);
}

double TrajectoryController::computeAltitudePID(
  double current_alt, double target_alt, double dt)
{
  double error = target_alt - current_alt;

  // Proportional
  double p_term = config_.altitude_kp * error;

  // Integral (with anti-windup)
  altitude_error_integral_ += error * dt;
  double max_integral = config_.altitude_max_velocity / config_.altitude_ki;
  altitude_error_integral_ = std::clamp(altitude_error_integral_, -max_integral, max_integral);
  double i_term = config_.altitude_ki * altitude_error_integral_;

  // Derivative
  double d_term = 0.0;
  if (dt > 0.001) {
    d_term = config_.altitude_kd * (error - altitude_error_prev_) / dt;
  }
  altitude_error_prev_ = error;

  // Total output
  double output = p_term + i_term + d_term;
  return std::clamp(output, -config_.altitude_max_velocity, config_.altitude_max_velocity);
}

geometry_msgs::msg::Twist TrajectoryController::applyRateLimiting(
  const geometry_msgs::msg::Twist & cmd, double dt)
{
  geometry_msgs::msg::Twist limited = cmd;

  if (dt < 0.001) {
    return limited;
  }

  // Limit linear acceleration
  double current_speed = std::sqrt(
    std::pow(last_cmd_.linear.x, 2) +
    std::pow(last_cmd_.linear.y, 2));
  double target_speed = std::sqrt(
    std::pow(cmd.linear.x, 2) +
    std::pow(cmd.linear.y, 2));

  double max_speed_change = config_.max_linear_accel * dt;
  if (std::abs(target_speed - current_speed) > max_speed_change) {
    double scale = (target_speed > current_speed) ?
      (current_speed + max_speed_change) / target_speed :
      (current_speed - max_speed_change) / target_speed;
    limited.linear.x = cmd.linear.x * scale;
    limited.linear.y = cmd.linear.y * scale;
  }

  // Limit vertical acceleration
  double z_change = cmd.linear.z - last_cmd_.linear.z;
  double max_z_change = config_.max_linear_accel * dt;
  if (std::abs(z_change) > max_z_change) {
    limited.linear.z = last_cmd_.linear.z + std::copysign(max_z_change, z_change);
  }

  // Limit angular acceleration
  double angular_change = cmd.angular.z - last_cmd_.angular.z;
  double max_angular_change = config_.max_angular_accel * dt;
  if (std::abs(angular_change) > max_angular_change) {
    limited.angular.z = last_cmd_.angular.z + std::copysign(max_angular_change, angular_change);
  }

  return limited;
}

ControllerOutput TrajectoryController::update(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double dt)
{
  ControllerOutput output;

  // Check for avoidance override first
  if (hasActiveAvoidanceOverride()) {
    output.using_avoidance_override = true;

    const auto & override_pos = avoidance_override_.waypoint.pose.position;
    const auto & pos = current_pose.pose.position;

    // Navigate toward override waypoint
    double dx = override_pos.x - pos.x;
    double dy = override_pos.y - pos.y;
    double dz = override_pos.z - pos.z;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < config_.xy_goal_tolerance) {
      // Reached override waypoint, clear it
      avoidance_override_.active = false;
    } else {
      // Compute velocity toward override waypoint
      double target_yaw = std::atan2(dy, dx);
      double current_yaw = getYawFromQuaternion(current_pose.pose.orientation);
      double yaw_error = normalizeAngle(target_yaw - current_yaw);

      // Slower speed during avoidance maneuvers
      double linear_vel = std::min(config_.max_linear_velocity * 0.7, dist);
      linear_vel = std::clamp(linear_vel, config_.min_linear_velocity, config_.max_linear_velocity);

      // Body frame velocities
      output.cmd_vel.linear.x = linear_vel * std::cos(yaw_error);
      output.cmd_vel.linear.y = linear_vel * std::sin(yaw_error);
      output.cmd_vel.linear.z = computeAltitudePID(pos.z, override_pos.z, dt);
      output.cmd_vel.angular.z = std::clamp(2.0 * yaw_error,
        -config_.max_angular_velocity, config_.max_angular_velocity);

      // Apply rate limiting
      output.cmd_vel = applyRateLimiting(output.cmd_vel, dt);
      last_cmd_ = output.cmd_vel;

      return output;
    }
  }

  if (path_.poses.empty()) {
    output.cmd_vel = geometry_msgs::msg::Twist();
    output.goal_reached = true;
    return output;
  }

  const auto & pos = current_pose.pose.position;
  output.distance_to_goal = distanceToGoal(current_pose);
  output.current_waypoint_idx = current_waypoint_idx_;

  // Check if goal reached
  double xy_dist = std::sqrt(
    std::pow(path_.poses.back().pose.position.x - pos.x, 2) +
    std::pow(path_.poses.back().pose.position.y - pos.y, 2));
  double z_dist = std::abs(path_.poses.back().pose.position.z - pos.z);

  if (xy_dist < config_.xy_goal_tolerance && z_dist < config_.z_goal_tolerance) {
    output.cmd_vel = geometry_msgs::msg::Twist();
    output.goal_reached = true;
    last_cmd_ = output.cmd_vel;
    return output;
  }

  // Compute adaptive lookahead based on speed
  double current_speed = std::sqrt(
    std::pow(last_cmd_.linear.x, 2) +
    std::pow(last_cmd_.linear.y, 2));
  double lookahead = config_.lookahead_distance + config_.lookahead_speed_gain * current_speed;
  lookahead = std::clamp(lookahead, config_.min_lookahead, config_.max_lookahead);

  // Find lookahead point
  auto lookahead_point = findLookaheadPoint(current_pose, lookahead);
  if (!lookahead_point) {
    output.cmd_vel = geometry_msgs::msg::Twist();
    output.goal_reached = true;
    last_cmd_ = output.cmd_vel;
    return output;
  }

  // Compute cross-track error (distance to path)
  if (current_waypoint_idx_ < static_cast<int>(path_.poses.size())) {
    const auto & nearest = path_.poses[current_waypoint_idx_].pose.position;
    output.cross_track_error = std::sqrt(
      std::pow(nearest.x - pos.x, 2) +
      std::pow(nearest.y - pos.y, 2));
  }

  // Pure Pursuit for XY
  double angular_vel = computePurePursuitSteering(current_pose, *lookahead_point);

  // Compute forward velocity (slow down near goal and during turns)
  double dist_factor = std::min(1.0, xy_dist / (3.0 * config_.xy_goal_tolerance));
  double turn_factor = std::max(0.3, 1.0 - std::abs(angular_vel) / config_.max_angular_velocity);
  double linear_vel = config_.max_linear_velocity * dist_factor * turn_factor;
  linear_vel = std::clamp(linear_vel, config_.min_linear_velocity, config_.max_linear_velocity);

  // Convert to body frame velocity
  double current_yaw = getYawFromQuaternion(current_pose.pose.orientation);
  double dx = lookahead_point->x - pos.x;
  double dy = lookahead_point->y - pos.y;
  double target_yaw = std::atan2(dy, dx);

  // Body frame velocities (forward and lateral)
  output.cmd_vel.linear.x = linear_vel * std::cos(target_yaw - current_yaw);
  output.cmd_vel.linear.y = linear_vel * std::sin(target_yaw - current_yaw);

  // PID for altitude
  double target_alt = lookahead_point->z;
  output.cmd_vel.linear.z = computeAltitudePID(pos.z, target_alt, dt);

  // Angular velocity
  output.cmd_vel.angular.z = angular_vel;

  // Apply rate limiting
  output.cmd_vel = applyRateLimiting(output.cmd_vel, dt);

  // Store for next iteration
  last_cmd_ = output.cmd_vel;

  return output;
}

}  // namespace path_planning
