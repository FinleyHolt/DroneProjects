/**
 * @file vfh3d.hpp
 * @brief 3D Vector Field Histogram (VFH+) for aerial obstacle avoidance
 *
 * Implements 3D-VFH+ algorithm for reactive obstacle avoidance on UAVs.
 * Uses spherical histogram to represent obstacle density in all directions,
 * then selects the best direction toward the goal through free space.
 *
 * Reference:
 * - Vanneste et al., "3DVFH+: Real-Time Three-Dimensional Obstacle Avoidance
 *   Using an Octomap", MORSE 2014
 * - Based on PX4-Avoidance local_planner implementation
 */

#ifndef LOCAL_AVOIDANCE__VFH3D_HPP_
#define LOCAL_AVOIDANCE__VFH3D_HPP_

#include <memory>
#include <vector>
#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "local_avoidance/octomap_manager.hpp"

namespace local_avoidance
{

/**
 * @brief Configuration for 3D-VFH+ algorithm
 */
struct VFH3DConfig
{
  // Histogram parameters
  int azimuth_bins{36};             // Number of horizontal bins (10 deg each)
  int elevation_bins{18};           // Number of vertical bins (10 deg each)

  // Safety parameters
  double safety_radius{5.0};        // Minimum clearance from obstacles (meters)
  double critical_radius{2.0};      // Emergency stop distance (meters)
  double lookahead_distance{20.0};  // How far ahead to check for obstacles (meters)

  // Cost function weights
  double goal_weight{5.0};          // Weight for goal direction alignment
  double heading_weight{2.0};       // Weight for current heading continuity
  double clearance_weight{1.0};     // Weight for obstacle clearance
  double elevation_weight{1.0};     // Penalty for vertical movement

  // Histogram thresholds
  double obstacle_density_threshold{0.3};  // Density above this = blocked
  double free_space_threshold{0.1};        // Density below this = definitely free

  // Smoothing
  int histogram_smoothing{2};       // Gaussian smoothing kernel half-width
  double hysteresis_threshold{0.2}; // Prevent direction oscillation

  // Limits
  double max_pitch{0.5};            // Maximum pitch angle (radians, ~30 deg)
  double preferred_altitude_agl{30.0};  // Preferred altitude AGL (meters)
};

/**
 * @brief Result of VFH+ direction selection
 */
struct VFHResult
{
  // Recommended direction change
  double recommended_yaw{0.0};      // Recommended yaw change (radians)
  double recommended_pitch{0.0};    // Recommended pitch change (radians)

  // Confidence and state
  double confidence{0.0};           // Confidence in recommendation [0-1]
  bool path_blocked{false};         // True if goal direction is blocked
  bool emergency_stop{false};       // True if obstacle within critical radius

  // Diagnostics
  int candidate_directions{0};      // Number of viable directions found
  int blocked_bins{0};              // Number of blocked histogram bins
  double closest_obstacle{std::numeric_limits<double>::max()};

  // Goal info (input echoed back)
  geometry_msgs::msg::Point goal_direction;
  double goal_distance{0.0};
};

/**
 * @brief Histogram bin representing obstacle density in a direction
 */
struct HistogramBin
{
  double azimuth{0.0};              // Center azimuth angle (radians)
  double elevation{0.0};            // Center elevation angle (radians)
  double density{0.0};              // Obstacle density [0-1]
  bool blocked{false};              // True if density > threshold
  double min_distance{std::numeric_limits<double>::max()};  // Closest obstacle in this direction
};

/**
 * @brief 3D Vector Field Histogram for obstacle avoidance
 *
 * Algorithm steps:
 * 1. Query OctoMap for obstacles within lookahead sphere
 * 2. Build spherical histogram of obstacle density
 * 3. Apply smoothing and threshold to get blocked/free bins
 * 4. Find candidate directions (free bins toward goal)
 * 5. Score candidates by goal alignment, heading continuity, clearance
 * 6. Return best direction (or signal emergency stop)
 */
class VFH3D
{
public:
  /**
   * @brief Constructor
   * @param octomap_manager Shared pointer to OctoMap manager
   * @param config VFH configuration
   */
  VFH3D(
    std::shared_ptr<OctomapManager> octomap_manager,
    const VFH3DConfig & config = VFH3DConfig());

  /**
   * @brief Compute avoidance direction
   * @param current_position Current UAV position (body frame origin)
   * @param current_heading Current UAV heading (radians)
   * @param goal_position Goal position (same frame as current_position)
   * @return VFHResult with recommended heading change
   */
  VFHResult computeAvoidance(
    const geometry_msgs::msg::Point & current_position,
    double current_heading,
    const geometry_msgs::msg::Point & goal_position);

  /**
   * @brief Get current histogram (for visualization)
   * @return Vector of histogram bins
   */
  const std::vector<HistogramBin> & getHistogram() const { return histogram_; }

  /**
   * @brief Get configuration
   */
  const VFH3DConfig & getConfig() const { return config_; }

  /**
   * @brief Update configuration
   */
  void setConfig(const VFH3DConfig & config);

  /**
   * @brief Reset state (clear history for hysteresis)
   */
  void reset();

private:
  /**
   * @brief Build histogram from OctoMap
   * @param position Query position
   */
  void buildHistogram(const geometry_msgs::msg::Point & position);

  /**
   * @brief Smooth histogram with Gaussian kernel
   */
  void smoothHistogram();

  /**
   * @brief Apply threshold to mark blocked bins
   */
  void applyThreshold();

  /**
   * @brief Find candidate directions toward goal
   * @param goal_azimuth Azimuth to goal
   * @param goal_elevation Elevation to goal
   * @return Indices of candidate bins
   */
  std::vector<int> findCandidates(double goal_azimuth, double goal_elevation);

  /**
   * @brief Score a candidate direction
   * @param bin_idx Histogram bin index
   * @param goal_azimuth Azimuth to goal
   * @param goal_elevation Elevation to goal
   * @param current_heading Current heading
   * @return Score (higher = better)
   */
  double scoreCandidate(
    int bin_idx,
    double goal_azimuth,
    double goal_elevation,
    double current_heading);

  /**
   * @brief Convert (azimuth, elevation) to bin index
   */
  int toBinIndex(double azimuth, double elevation) const;

  /**
   * @brief Convert bin index to (azimuth, elevation)
   */
  void fromBinIndex(int idx, double & azimuth, double & elevation) const;

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  double normalizeAngle(double angle) const;

  /**
   * @brief Compute angle between two directions
   */
  double angleBetween(double az1, double el1, double az2, double el2) const;

  std::shared_ptr<OctomapManager> octomap_;
  VFH3DConfig config_;
  std::vector<HistogramBin> histogram_;

  // State for hysteresis
  std::optional<double> last_recommended_azimuth_;
  std::optional<double> last_recommended_elevation_;
};

}  // namespace local_avoidance

#endif  // LOCAL_AVOIDANCE__VFH3D_HPP_
