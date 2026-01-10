/**
 * @file octomap_manager.hpp
 * @brief OctoMap-based 3D obstacle map management
 *
 * Maintains an OctoMap representation of the environment from depth point clouds.
 * Handles incremental updates, ray casting, and obstacle queries.
 */

#ifndef LOCAL_AVOIDANCE__OCTOMAP_MANAGER_HPP_
#define LOCAL_AVOIDANCE__OCTOMAP_MANAGER_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace local_avoidance
{

/**
 * @brief Configuration for the OctoMap manager
 */
struct OctomapConfig
{
  double resolution{0.5};           // Voxel resolution (meters)
  double max_range{50.0};           // Maximum sensor range for ray casting (meters)
  double hit_probability{0.7};      // Probability increase for occupied voxels
  double miss_probability{0.4};     // Probability decrease for free voxels
  double occupancy_threshold{0.5};  // Threshold for considering voxel occupied
  double clamping_threshold_min{0.1192};  // Min log-odds clamping
  double clamping_threshold_max{0.971};   // Max log-odds clamping

  // Map bounds (body-centered)
  double x_min{-50.0};
  double x_max{50.0};
  double y_min{-50.0};
  double y_max{50.0};
  double z_min{-20.0};
  double z_max{30.0};

  // Pruning parameters
  double prune_frequency{5.0};      // How often to prune the tree (Hz)
  bool enable_pruning{true};
};

/**
 * @brief Query result for obstacle checking
 */
struct ObstacleQuery
{
  bool occupied{false};
  double distance{std::numeric_limits<double>::max()};
  geometry_msgs::msg::Point closest_point;
  int voxels_checked{0};
};

/**
 * @brief Manages OctoMap for 3D obstacle representation
 *
 * Thread-safe wrapper around OctoMap that handles:
 * - Point cloud insertion with ray casting
 * - Obstacle queries (nearest obstacle, ray intersection)
 * - Map maintenance (pruning, bounding)
 */
class OctomapManager
{
public:
  /**
   * @brief Constructor
   * @param config Configuration parameters
   */
  explicit OctomapManager(const OctomapConfig & config = OctomapConfig());

  /**
   * @brief Destructor
   */
  ~OctomapManager() = default;

  /**
   * @brief Insert point cloud into the map
   * @param cloud Point cloud in sensor frame
   * @param sensor_origin Origin of the sensor in map frame
   * @param sensor_orientation Orientation of the sensor in map frame
   */
  void insertPointCloud(
    const std::vector<geometry_msgs::msg::Point> & cloud,
    const geometry_msgs::msg::Point & sensor_origin);

  /**
   * @brief Insert point cloud from ROS message
   * @param cloud_msg PointCloud2 message
   * @param sensor_pose Sensor pose in map frame
   */
  void insertPointCloud(
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    const geometry_msgs::msg::Pose & sensor_pose);

  /**
   * @brief Check if a point is occupied
   * @param point Point to check (map frame)
   * @return true if occupied
   */
  bool isOccupied(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief Find nearest obstacle to a point
   * @param point Query point (map frame)
   * @param max_distance Maximum search distance
   * @return ObstacleQuery result
   */
  ObstacleQuery findNearestObstacle(
    const geometry_msgs::msg::Point & point,
    double max_distance = 10.0) const;

  /**
   * @brief Cast ray and find first obstacle intersection
   * @param origin Ray origin
   * @param direction Ray direction (unit vector)
   * @param max_range Maximum ray length
   * @return Distance to first obstacle, or max_range if none
   */
  double castRay(
    const geometry_msgs::msg::Point & origin,
    const geometry_msgs::msg::Point & direction,
    double max_range = 50.0) const;

  /**
   * @brief Get all occupied voxels within a sphere
   * @param center Sphere center
   * @param radius Sphere radius
   * @return Vector of occupied voxel centers
   */
  std::vector<geometry_msgs::msg::Point> getOccupiedVoxels(
    const geometry_msgs::msg::Point & center,
    double radius) const;

  /**
   * @brief Clear the entire map
   */
  void clear();

  /**
   * @brief Prune the octree to reduce memory
   */
  void prune();

  /**
   * @brief Get the raw OctoMap (for visualization)
   */
  std::shared_ptr<const octomap::OcTree> getOctree() const;

  /**
   * @brief Get number of occupied voxels
   */
  size_t getNumOccupiedVoxels() const;

  /**
   * @brief Get memory usage in bytes
   */
  size_t getMemoryUsage() const;

  /**
   * @brief Get current configuration
   */
  const OctomapConfig & getConfig() const { return config_; }

  /**
   * @brief Update configuration (resolution change requires rebuild)
   */
  void setConfig(const OctomapConfig & config);

private:
  /**
   * @brief Check if point is within map bounds
   */
  bool isInBounds(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief Convert geometry_msgs::Point to octomap::point3d
   */
  octomap::point3d toOctomap(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief Convert octomap::point3d to geometry_msgs::Point
   */
  geometry_msgs::msg::Point fromOctomap(const octomap::point3d & point) const;

  OctomapConfig config_;
  std::shared_ptr<octomap::OcTree> octree_;
  mutable std::mutex mutex_;
};

}  // namespace local_avoidance

#endif  // LOCAL_AVOIDANCE__OCTOMAP_MANAGER_HPP_
