/**
 * @file corridor_detector.hpp
 * @brief Corridor detection and path centering for obstacle avoidance
 *
 * Detects when the vehicle is navigating through a corridor (obstacles on
 * both sides with overlapping safety margins) and computes the optimal
 * centered path to maximize clearance from both sides.
 */

#ifndef LOCAL_AVOIDANCE__CORRIDOR_DETECTOR_HPP_
#define LOCAL_AVOIDANCE__CORRIDOR_DETECTOR_HPP_

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "local_avoidance/octomap_manager.hpp"

namespace local_avoidance
{

/**
 * @brief Configuration for corridor detection
 */
struct CorridorConfig
{
  bool enabled{true};                  // Enable corridor detection
  double scan_width_rad{1.047};        // Angular width to scan (radians, ~60 deg)
  double min_gap_ratio{1.5};           // Min gap = ratio * critical_buffer
  double centering_weight{3.0};        // Weight for centering in scoring
  int num_samples{5};                  // Number of samples along path
  double sample_interval{2.0};         // Distance between samples (meters)
};

/**
 * @brief Result of corridor detection
 */
struct CorridorInfo
{
  bool is_corridor{false};             // True if corridor detected
  double gap_width{0.0};               // Distance between obstacles (meters)
  double center_offset{0.0};           // How far off-center current position is (meters)
                                       // Positive = need to move right, Negative = move left

  // Obstacle positions
  geometry_msgs::msg::Point left_obstacle;   // Closest obstacle on left
  geometry_msgs::msg::Point right_obstacle;  // Closest obstacle on right
  double left_distance{0.0};                 // Distance to left obstacle
  double right_distance{0.0};                // Distance to right obstacle

  // Computed optimal path
  geometry_msgs::msg::Point optimal_center;  // Point on centered path
  double optimal_azimuth{0.0};               // Azimuth toward center (radians)

  // Status
  bool passable{true};                 // False if gap < minimum required
  double clearance_margin{0.0};        // Gap - 2*critical_buffer (negative if too narrow)
};

/**
 * @brief Detects corridor situations and computes centered paths
 *
 * A corridor exists when:
 * 1. Obstacles are detected on both sides of the intended path
 * 2. The gap between them is less than 2x the safety buffer (margins overlap)
 * 3. The gap is still greater than 2x the critical buffer (passable)
 *
 * When a corridor is detected, the optimal path is computed to maintain
 * equal distance from both obstacles ("split the difference").
 */
class CorridorDetector
{
public:
  /**
   * @brief Constructor
   * @param octomap_manager Shared pointer to OctoMap manager
   * @param config Corridor detection configuration
   */
  CorridorDetector(
    std::shared_ptr<OctomapManager> octomap_manager,
    const CorridorConfig & config = CorridorConfig());

  /**
   * @brief Detect if current position/heading is in a corridor
   *
   * @param position Current UAV position
   * @param heading Current heading direction (azimuth in radians)
   * @param safety_buffer Current velocity-scaled safety buffer
   * @param critical_buffer Current velocity-scaled critical buffer
   * @param lookahead How far ahead to scan (meters)
   * @return CorridorInfo with detection results
   */
  CorridorInfo detectCorridor(
    const geometry_msgs::msg::Point & position,
    double heading,
    double safety_buffer,
    double critical_buffer,
    double lookahead);

  /**
   * @brief Check if a corridor gap is passable
   * @param gap_width Width of the corridor
   * @param critical_buffer Minimum required clearance per side
   * @return true if gap >= 2 * critical_buffer
   */
  bool isPassable(double gap_width, double critical_buffer) const;

  /**
   * @brief Compute centering score for a candidate direction
   *
   * Higher score for directions that lead toward corridor center.
   *
   * @param candidate_azimuth Azimuth of candidate direction (radians)
   * @param corridor Corridor info from detectCorridor()
   * @param current_position Current UAV position
   * @return Centering score [0, 1] (higher = better centered)
   */
  double computeCenteringScore(
    double candidate_azimuth,
    const CorridorInfo & corridor,
    const geometry_msgs::msg::Point & current_position) const;

  /**
   * @brief Get current configuration
   */
  const CorridorConfig & getConfig() const { return config_; }

  /**
   * @brief Update configuration
   */
  void setConfig(const CorridorConfig & config);

private:
  /**
   * @brief Find obstacle in a specific direction
   * @param position Start position
   * @param azimuth Direction to search (radians)
   * @param max_range Maximum search distance
   * @return Distance to obstacle, or max_range if none
   */
  double findObstacleDistance(
    const geometry_msgs::msg::Point & position,
    double azimuth,
    double max_range) const;

  /**
   * @brief Compute perpendicular directions from heading
   * @param heading Forward heading (radians)
   * @param[out] left_azimuth Azimuth to left (heading + pi/2)
   * @param[out] right_azimuth Azimuth to right (heading - pi/2)
   */
  void computePerpendicularDirections(
    double heading,
    double & left_azimuth,
    double & right_azimuth) const;

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  double normalizeAngle(double angle) const;

  /**
   * @brief Convert azimuth to direction vector (XY plane)
   */
  geometry_msgs::msg::Point azimuthToDirection(double azimuth) const;

  std::shared_ptr<OctomapManager> octomap_;
  CorridorConfig config_;
};

}  // namespace local_avoidance

#endif  // LOCAL_AVOIDANCE__CORRIDOR_DETECTOR_HPP_
