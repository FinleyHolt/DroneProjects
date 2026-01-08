/**
 * @file vfh3d.cpp
 * @brief Implementation of 3D Vector Field Histogram (VFH+)
 */

#include "local_avoidance/vfh3d.hpp"

#include <cmath>
#include <algorithm>
#include <numeric>

namespace local_avoidance
{

VFH3D::VFH3D(
  std::shared_ptr<OctomapManager> octomap_manager,
  const VFH3DConfig & config)
: octomap_(octomap_manager), config_(config)
{
  // Initialize histogram with correct size
  const int total_bins = config_.azimuth_bins * config_.elevation_bins;
  histogram_.resize(total_bins);

  // Pre-compute bin centers
  for (int i = 0; i < total_bins; ++i) {
    double az, el;
    fromBinIndex(i, az, el);
    histogram_[i].azimuth = az;
    histogram_[i].elevation = el;
  }
}

VFHResult VFH3D::computeAvoidance(
  const geometry_msgs::msg::Point & current_position,
  double current_heading,
  const geometry_msgs::msg::Point & goal_position)
{
  VFHResult result;

  // Compute goal direction
  double dx = goal_position.x - current_position.x;
  double dy = goal_position.y - current_position.y;
  double dz = goal_position.z - current_position.z;
  result.goal_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (result.goal_distance < 0.1) {
    // Already at goal
    result.confidence = 1.0;
    return result;
  }

  // Goal direction (body frame: X forward, Y left, Z up)
  double goal_azimuth = std::atan2(dy, dx);
  double goal_elevation = std::atan2(dz, std::sqrt(dx * dx + dy * dy));

  result.goal_direction.x = dx / result.goal_distance;
  result.goal_direction.y = dy / result.goal_distance;
  result.goal_direction.z = dz / result.goal_distance;

  // Build and process histogram
  buildHistogram(current_position);
  smoothHistogram();
  applyThreshold();

  // Count blocked bins
  result.blocked_bins = std::count_if(
    histogram_.begin(), histogram_.end(),
    [](const HistogramBin & bin) { return bin.blocked; });

  // Find closest obstacle
  for (const auto & bin : histogram_) {
    if (bin.min_distance < result.closest_obstacle) {
      result.closest_obstacle = bin.min_distance;
    }
  }

  // Check for emergency stop
  if (result.closest_obstacle < config_.critical_radius) {
    result.emergency_stop = true;
    result.confidence = 1.0;
    return result;
  }

  // Check if goal direction is blocked
  int goal_bin = toBinIndex(goal_azimuth, goal_elevation);
  if (goal_bin >= 0 && goal_bin < static_cast<int>(histogram_.size())) {
    result.path_blocked = histogram_[goal_bin].blocked;
  }

  // Find candidate directions
  std::vector<int> candidates = findCandidates(goal_azimuth, goal_elevation);
  result.candidate_directions = static_cast<int>(candidates.size());

  if (candidates.empty()) {
    // No safe direction found - recommend stopping
    result.emergency_stop = true;
    result.confidence = 0.5;
    return result;
  }

  // Score candidates and find best
  double best_score = -std::numeric_limits<double>::max();
  int best_idx = candidates[0];

  for (int idx : candidates) {
    double score = scoreCandidate(idx, goal_azimuth, goal_elevation, current_heading);
    if (score > best_score) {
      best_score = score;
      best_idx = idx;
    }
  }

  // Get recommended direction
  double recommended_az, recommended_el;
  fromBinIndex(best_idx, recommended_az, recommended_el);

  // Apply hysteresis to prevent oscillation
  if (last_recommended_azimuth_.has_value()) {
    double az_diff = std::abs(normalizeAngle(recommended_az - last_recommended_azimuth_.value()));
    if (az_diff < config_.hysteresis_threshold) {
      recommended_az = last_recommended_azimuth_.value();
    }
  }

  // Compute heading change (relative to current heading)
  result.recommended_yaw = normalizeAngle(recommended_az - current_heading);
  result.recommended_pitch = std::clamp(recommended_el, -config_.max_pitch, config_.max_pitch);

  // Confidence based on clearance and candidate quality
  double clearance_factor = std::min(1.0, result.closest_obstacle / config_.safety_radius);
  double candidate_factor = std::min(1.0, result.candidate_directions / 10.0);
  result.confidence = 0.5 * clearance_factor + 0.5 * candidate_factor;

  // Update hysteresis state
  last_recommended_azimuth_ = recommended_az;
  last_recommended_elevation_ = recommended_el;

  return result;
}

void VFH3D::buildHistogram(const geometry_msgs::msg::Point & position)
{
  // Reset histogram
  for (auto & bin : histogram_) {
    bin.density = 0.0;
    bin.blocked = false;
    bin.min_distance = std::numeric_limits<double>::max();
  }

  // Get occupied voxels within lookahead sphere
  auto obstacles = octomap_->getOccupiedVoxels(position, config_.lookahead_distance);

  if (obstacles.empty()) {
    return;
  }

  // Populate histogram from obstacles
  for (const auto & obstacle : obstacles) {
    double dx = obstacle.x - position.x;
    double dy = obstacle.y - position.y;
    double dz = obstacle.z - position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 0.1) {
      continue;  // Skip obstacles at our position
    }

    // Compute direction to obstacle
    double azimuth = std::atan2(dy, dx);
    double elevation = std::atan2(dz, std::sqrt(dx * dx + dy * dy));

    // Find corresponding bin
    int bin_idx = toBinIndex(azimuth, elevation);
    if (bin_idx < 0 || bin_idx >= static_cast<int>(histogram_.size())) {
      continue;
    }

    // Update bin
    // Density increases inversely with distance (closer = more dangerous)
    double contribution = 1.0 - (distance / config_.lookahead_distance);
    contribution = std::max(0.0, contribution);
    histogram_[bin_idx].density += contribution;

    // Track minimum distance
    if (distance < histogram_[bin_idx].min_distance) {
      histogram_[bin_idx].min_distance = distance;
    }
  }

  // Normalize density by number of expected obstacles
  double max_density = 0.0;
  for (const auto & bin : histogram_) {
    if (bin.density > max_density) {
      max_density = bin.density;
    }
  }

  if (max_density > 0.01) {
    for (auto & bin : histogram_) {
      bin.density /= max_density;
    }
  }
}

void VFH3D::smoothHistogram()
{
  if (config_.histogram_smoothing <= 0) {
    return;
  }

  std::vector<double> smoothed(histogram_.size(), 0.0);
  const int k = config_.histogram_smoothing;

  // Gaussian-like weights (simplified)
  std::vector<double> weights(2 * k + 1);
  double sum = 0.0;
  for (int i = -k; i <= k; ++i) {
    weights[i + k] = std::exp(-0.5 * i * i / (k * k / 2.0));
    sum += weights[i + k];
  }
  for (auto & w : weights) {
    w /= sum;
  }

  // Apply 1D smoothing in azimuth direction
  for (int el = 0; el < config_.elevation_bins; ++el) {
    for (int az = 0; az < config_.azimuth_bins; ++az) {
      double value = 0.0;
      for (int di = -k; di <= k; ++di) {
        int az_i = (az + di + config_.azimuth_bins) % config_.azimuth_bins;
        int idx = el * config_.azimuth_bins + az_i;
        value += histogram_[idx].density * weights[di + k];
      }
      smoothed[el * config_.azimuth_bins + az] = value;
    }
  }

  // Copy back
  for (size_t i = 0; i < histogram_.size(); ++i) {
    histogram_[i].density = smoothed[i];
  }
}

void VFH3D::applyThreshold()
{
  for (auto & bin : histogram_) {
    bin.blocked = bin.density > config_.obstacle_density_threshold ||
      bin.min_distance < config_.safety_radius;
  }
}

std::vector<int> VFH3D::findCandidates(double goal_azimuth, double goal_elevation)
{
  std::vector<int> candidates;

  // First, check if goal direction is free
  int goal_bin = toBinIndex(goal_azimuth, goal_elevation);
  if (goal_bin >= 0 && goal_bin < static_cast<int>(histogram_.size()) &&
    !histogram_[goal_bin].blocked)
  {
    candidates.push_back(goal_bin);
  }

  // Find all free bins
  for (size_t i = 0; i < histogram_.size(); ++i) {
    if (!histogram_[i].blocked && static_cast<int>(i) != goal_bin) {
      // Only consider bins within reasonable angle from goal
      double angle = angleBetween(
        histogram_[i].azimuth, histogram_[i].elevation,
        goal_azimuth, goal_elevation);

      // Allow up to 90 degrees from goal direction
      if (angle < M_PI / 2) {
        candidates.push_back(i);
      }
    }
  }

  // If still no candidates, expand search to all free bins
  if (candidates.empty()) {
    for (size_t i = 0; i < histogram_.size(); ++i) {
      if (!histogram_[i].blocked) {
        candidates.push_back(i);
      }
    }
  }

  return candidates;
}

double VFH3D::scoreCandidate(
  int bin_idx,
  double goal_azimuth,
  double goal_elevation,
  double current_heading)
{
  const auto & bin = histogram_[bin_idx];

  // Goal alignment score (higher when pointing toward goal)
  double angle_to_goal = angleBetween(bin.azimuth, bin.elevation, goal_azimuth, goal_elevation);
  double goal_score = config_.goal_weight * (1.0 - angle_to_goal / M_PI);

  // Heading continuity score (higher when maintaining current heading)
  double heading_diff = std::abs(normalizeAngle(bin.azimuth - current_heading));
  double heading_score = config_.heading_weight * (1.0 - heading_diff / M_PI);

  // Clearance score (higher when more clearance)
  double clearance_score = 0.0;
  if (bin.min_distance < config_.lookahead_distance) {
    clearance_score = config_.clearance_weight * (bin.min_distance / config_.lookahead_distance);
  } else {
    clearance_score = config_.clearance_weight;
  }

  // Elevation penalty (prefer horizontal movement)
  double elevation_penalty = config_.elevation_weight * std::abs(bin.elevation) / config_.max_pitch;

  return goal_score + heading_score + clearance_score - elevation_penalty;
}

int VFH3D::toBinIndex(double azimuth, double elevation) const
{
  // Normalize azimuth to [0, 2*pi)
  double az_norm = azimuth;
  while (az_norm < 0) {az_norm += 2 * M_PI;}
  while (az_norm >= 2 * M_PI) {az_norm -= 2 * M_PI;}

  // Clamp elevation to [-pi/2, pi/2]
  double el_clamp = std::clamp(elevation, -M_PI / 2 + 0.01, M_PI / 2 - 0.01);

  // Map elevation from [-pi/2, pi/2] to [0, elevation_bins)
  double el_norm = (el_clamp + M_PI / 2) / M_PI;  // [0, 1]

  int az_bin = static_cast<int>(az_norm / (2 * M_PI) * config_.azimuth_bins);
  int el_bin = static_cast<int>(el_norm * config_.elevation_bins);

  // Clamp to valid range
  az_bin = std::clamp(az_bin, 0, config_.azimuth_bins - 1);
  el_bin = std::clamp(el_bin, 0, config_.elevation_bins - 1);

  return el_bin * config_.azimuth_bins + az_bin;
}

void VFH3D::fromBinIndex(int idx, double & azimuth, double & elevation) const
{
  int az_bin = idx % config_.azimuth_bins;
  int el_bin = idx / config_.azimuth_bins;

  // Bin center
  azimuth = (az_bin + 0.5) / config_.azimuth_bins * 2 * M_PI;
  // Convert back to [-pi, pi]
  if (azimuth > M_PI) {
    azimuth -= 2 * M_PI;
  }

  // Elevation: [0, elevation_bins) -> [-pi/2, pi/2]
  double el_norm = (el_bin + 0.5) / config_.elevation_bins;  // (0, 1)
  elevation = el_norm * M_PI - M_PI / 2;  // [-pi/2, pi/2]
}

double VFH3D::normalizeAngle(double angle) const
{
  while (angle > M_PI) {angle -= 2 * M_PI;}
  while (angle < -M_PI) {angle += 2 * M_PI;}
  return angle;
}

double VFH3D::angleBetween(double az1, double el1, double az2, double el2) const
{
  // Convert to unit vectors and compute dot product
  double x1 = std::cos(el1) * std::cos(az1);
  double y1 = std::cos(el1) * std::sin(az1);
  double z1 = std::sin(el1);

  double x2 = std::cos(el2) * std::cos(az2);
  double y2 = std::cos(el2) * std::sin(az2);
  double z2 = std::sin(el2);

  double dot = x1 * x2 + y1 * y2 + z1 * z2;
  dot = std::clamp(dot, -1.0, 1.0);

  return std::acos(dot);
}

void VFH3D::setConfig(const VFH3DConfig & config)
{
  // Check if histogram size changed
  if (config.azimuth_bins != config_.azimuth_bins ||
    config.elevation_bins != config_.elevation_bins)
  {
    const int total_bins = config.azimuth_bins * config.elevation_bins;
    histogram_.resize(total_bins);

    // Recompute bin centers
    for (int i = 0; i < total_bins; ++i) {
      double az, el;
      config_ = config;  // Temporarily set for fromBinIndex
      fromBinIndex(i, az, el);
      histogram_[i].azimuth = az;
      histogram_[i].elevation = el;
    }
  }

  config_ = config;
}

void VFH3D::reset()
{
  last_recommended_azimuth_.reset();
  last_recommended_elevation_.reset();

  for (auto & bin : histogram_) {
    bin.density = 0.0;
    bin.blocked = false;
    bin.min_distance = std::numeric_limits<double>::max();
  }
}

}  // namespace local_avoidance
