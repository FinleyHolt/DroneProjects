/**
 * @file corridor_detector.cpp
 * @brief Implementation of corridor detection and path centering
 */

#include "local_avoidance/corridor_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_avoidance
{

CorridorDetector::CorridorDetector(
  std::shared_ptr<OctomapManager> octomap_manager,
  const CorridorConfig & config)
: octomap_(octomap_manager),
  config_(config)
{
}

CorridorInfo CorridorDetector::detectCorridor(
  const geometry_msgs::msg::Point & position,
  double heading,
  double safety_buffer,
  double critical_buffer,
  double lookahead)
{
  CorridorInfo info;

  if (!config_.enabled || !octomap_) {
    return info;
  }

  // Get perpendicular directions (left and right of heading)
  double left_azimuth, right_azimuth;
  computePerpendicularDirections(heading, left_azimuth, right_azimuth);

  // Search range for side obstacles (slightly wider than safety buffer)
  double side_search_range = safety_buffer * 2.0;

  // Sample multiple points along the path to get robust corridor detection
  double min_left_dist = std::numeric_limits<double>::max();
  double min_right_dist = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point closest_left, closest_right;

  auto forward_dir = azimuthToDirection(heading);

  for (int i = 0; i < config_.num_samples; ++i) {
    double sample_dist = i * config_.sample_interval;
    if (sample_dist > lookahead) {
      break;
    }

    // Sample position along path
    geometry_msgs::msg::Point sample_pos;
    sample_pos.x = position.x + forward_dir.x * sample_dist;
    sample_pos.y = position.y + forward_dir.y * sample_dist;
    sample_pos.z = position.z;

    // Cast rays left and right from this sample point
    double left_dist = findObstacleDistance(sample_pos, left_azimuth, side_search_range);
    double right_dist = findObstacleDistance(sample_pos, right_azimuth, side_search_range);

    // Track minimum distances (narrowest point of corridor)
    if (left_dist < min_left_dist) {
      min_left_dist = left_dist;
      auto left_dir = azimuthToDirection(left_azimuth);
      closest_left.x = sample_pos.x + left_dir.x * left_dist;
      closest_left.y = sample_pos.y + left_dir.y * left_dist;
      closest_left.z = sample_pos.z;
    }

    if (right_dist < min_right_dist) {
      min_right_dist = right_dist;
      auto right_dir = azimuthToDirection(right_azimuth);
      closest_right.x = sample_pos.x + right_dir.x * right_dist;
      closest_right.y = sample_pos.y + right_dir.y * right_dist;
      closest_right.z = sample_pos.z;
    }
  }

  // Store obstacle info
  info.left_obstacle = closest_left;
  info.right_obstacle = closest_right;
  info.left_distance = min_left_dist;
  info.right_distance = min_right_dist;

  // Calculate gap width
  info.gap_width = min_left_dist + min_right_dist;

  // Determine if this qualifies as a corridor:
  // - Both sides have obstacles within the safety buffer range
  // - Gap is narrow enough that buffers overlap (< 2 * safety_buffer)
  bool left_obstacle_present = (min_left_dist < side_search_range);
  bool right_obstacle_present = (min_right_dist < side_search_range);
  bool buffers_overlap = (info.gap_width < 2.0 * safety_buffer);

  info.is_corridor = left_obstacle_present && right_obstacle_present && buffers_overlap;

  if (info.is_corridor) {
    // Check if passable
    double min_required_gap = 2.0 * critical_buffer * config_.min_gap_ratio;
    info.passable = (info.gap_width >= min_required_gap);
    info.clearance_margin = info.gap_width - 2.0 * critical_buffer;

    // Compute center offset (how far off-center we are)
    // Positive center_offset means we should move right (away from left obstacle)
    // Negative center_offset means we should move left (away from right obstacle)
    info.center_offset = (min_right_dist - min_left_dist) / 2.0;

    // Compute optimal center point (midpoint between obstacles)
    info.optimal_center.x = (closest_left.x + closest_right.x) / 2.0;
    info.optimal_center.y = (closest_left.y + closest_right.y) / 2.0;
    info.optimal_center.z = position.z;

    // Compute azimuth toward the optimal center from current position
    double dx = info.optimal_center.x - position.x;
    double dy = info.optimal_center.y - position.y;

    if (std::hypot(dx, dy) > 0.1) {  // Avoid division by near-zero
      info.optimal_azimuth = std::atan2(dy, dx);
    } else {
      // Already at center, maintain current heading
      info.optimal_azimuth = heading;
    }
  } else {
    // Not a corridor - still set passable = true
    info.passable = true;
    info.clearance_margin = std::min(min_left_dist, min_right_dist);
  }

  return info;
}

bool CorridorDetector::isPassable(double gap_width, double critical_buffer) const
{
  return gap_width >= (2.0 * critical_buffer * config_.min_gap_ratio);
}

double CorridorDetector::computeCenteringScore(
  double candidate_azimuth,
  const CorridorInfo & corridor,
  const geometry_msgs::msg::Point & current_position) const
{
  if (!corridor.is_corridor) {
    return 0.0;  // No centering bonus outside corridors
  }

  // Compute angle between candidate direction and direction to center
  double dx = corridor.optimal_center.x - current_position.x;
  double dy = corridor.optimal_center.y - current_position.y;
  double dist_to_center = std::hypot(dx, dy);

  if (dist_to_center < 0.1) {
    // Already centered, all directions equally good for centering
    return 1.0;
  }

  double azimuth_to_center = std::atan2(dy, dx);
  double angle_diff = std::abs(normalizeAngle(candidate_azimuth - azimuth_to_center));

  // Score: 1.0 when pointing at center, 0.0 when pointing away
  double base_score = 1.0 - (angle_diff / M_PI);

  // Boost score based on corridor narrowness (narrower = more important to center)
  // At gap_width = critical_buffer * 2, narrowness = 1.0
  // At gap_width = safety_buffer * 2, narrowness approaches 0
  double narrowness_factor = 1.0;
  if (corridor.gap_width > 0) {
    // More off-center in a narrow corridor = higher urgency
    double off_center_ratio = std::abs(corridor.center_offset) /
      (corridor.gap_width / 2.0);
    narrowness_factor = 1.0 + std::min(off_center_ratio, 1.0);
  }

  return base_score * narrowness_factor;
}

void CorridorDetector::setConfig(const CorridorConfig & config)
{
  config_ = config;
}

double CorridorDetector::findObstacleDistance(
  const geometry_msgs::msg::Point & position,
  double azimuth,
  double max_range) const
{
  auto direction = azimuthToDirection(azimuth);
  return octomap_->castRay(position, direction, max_range);
}

void CorridorDetector::computePerpendicularDirections(
  double heading,
  double & left_azimuth,
  double & right_azimuth) const
{
  // Left is +90 degrees from heading
  left_azimuth = normalizeAngle(heading + M_PI / 2.0);
  // Right is -90 degrees from heading
  right_azimuth = normalizeAngle(heading - M_PI / 2.0);
}

double CorridorDetector::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::Point CorridorDetector::azimuthToDirection(double azimuth) const
{
  geometry_msgs::msg::Point dir;
  dir.x = std::cos(azimuth);
  dir.y = std::sin(azimuth);
  dir.z = 0.0;
  return dir;
}

}  // namespace local_avoidance
