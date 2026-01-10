/**
 * @file octomap_manager.cpp
 * @brief Implementation of OctoMap-based 3D obstacle map management
 */

#include "local_avoidance/octomap_manager.hpp"

#include <cmath>
#include <algorithm>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace local_avoidance
{

OctomapManager::OctomapManager(const OctomapConfig & config)
: config_(config)
{
  octree_ = std::make_shared<octomap::OcTree>(config_.resolution);

  // Set probability parameters
  octree_->setProbHit(config_.hit_probability);
  octree_->setProbMiss(config_.miss_probability);
  octree_->setClampingThresMin(config_.clamping_threshold_min);
  octree_->setClampingThresMax(config_.clamping_threshold_max);
  octree_->setOccupancyThres(config_.occupancy_threshold);
}

void OctomapManager::insertPointCloud(
  const std::vector<geometry_msgs::msg::Point> & cloud,
  const geometry_msgs::msg::Point & sensor_origin)
{
  std::lock_guard<std::mutex> lock(mutex_);

  octomap::point3d origin = toOctomap(sensor_origin);
  octomap::Pointcloud octocloud;

  for (const auto & point : cloud) {
    if (!isInBounds(point)) {
      continue;
    }
    octocloud.push_back(toOctomap(point));
  }

  // Insert with ray casting (marks free space along rays)
  octree_->insertPointCloud(octocloud, origin, config_.max_range);
}

void OctomapManager::insertPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud_msg,
  const geometry_msgs::msg::Pose & sensor_pose)
{
  std::lock_guard<std::mutex> lock(mutex_);

  octomap::point3d origin(
    sensor_pose.position.x,
    sensor_pose.position.y,
    sensor_pose.position.z
  );

  octomap::Pointcloud octocloud;

  // Iterate through PointCloud2
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    // Skip invalid points
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    // Check bounds
    if (x < config_.x_min || x > config_.x_max ||
      y < config_.y_min || y > config_.y_max ||
      z < config_.z_min || z > config_.z_max)
    {
      continue;
    }

    octocloud.push_back(octomap::point3d(x, y, z));
  }

  // Insert with ray casting
  octree_->insertPointCloud(octocloud, origin, config_.max_range);
}

bool OctomapManager::isOccupied(const geometry_msgs::msg::Point & point) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  octomap::OcTreeNode * node = octree_->search(toOctomap(point));
  if (node == nullptr) {
    return false;  // Unknown = not occupied
  }
  return octree_->isNodeOccupied(node);
}

ObstacleQuery OctomapManager::findNearestObstacle(
  const geometry_msgs::msg::Point & point,
  double max_distance) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  ObstacleQuery result;
  result.distance = max_distance;

  octomap::point3d query = toOctomap(point);

  // Iterate through occupied leaf nodes
  for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
    if (!octree_->isNodeOccupied(*it)) {
      continue;
    }

    result.voxels_checked++;

    octomap::point3d voxel_center = it.getCoordinate();
    double dist = query.distance(voxel_center);

    if (dist < result.distance) {
      result.distance = dist;
      result.closest_point = fromOctomap(voxel_center);
      result.occupied = true;
    }
  }

  return result;
}

double OctomapManager::castRay(
  const geometry_msgs::msg::Point & origin,
  const geometry_msgs::msg::Point & direction,
  double max_range) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  octomap::point3d ray_origin = toOctomap(origin);
  octomap::point3d ray_direction(direction.x, direction.y, direction.z);

  // Normalize direction
  double mag = ray_direction.norm();
  if (mag < 1e-6) {
    return max_range;
  }
  ray_direction = ray_direction * (1.0 / mag);

  octomap::point3d end_point;
  bool hit = octree_->castRay(ray_origin, ray_direction, end_point, true, max_range);

  if (hit) {
    return ray_origin.distance(end_point);
  }
  return max_range;
}

std::vector<geometry_msgs::msg::Point> OctomapManager::getOccupiedVoxels(
  const geometry_msgs::msg::Point & center,
  double radius) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<geometry_msgs::msg::Point> result;
  octomap::point3d query = toOctomap(center);

  // Define bounding box
  octomap::point3d min_pt(
    center.x - radius,
    center.y - radius,
    center.z - radius
  );
  octomap::point3d max_pt(
    center.x + radius,
    center.y + radius,
    center.z + radius
  );

  for (auto it = octree_->begin_leafs_bbx(min_pt, max_pt);
    it != octree_->end_leafs_bbx(); ++it)
  {
    if (!octree_->isNodeOccupied(*it)) {
      continue;
    }

    octomap::point3d voxel_center = it.getCoordinate();
    if (query.distance(voxel_center) <= radius) {
      result.push_back(fromOctomap(voxel_center));
    }
  }

  return result;
}

void OctomapManager::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  octree_->clear();
}

void OctomapManager::prune()
{
  std::lock_guard<std::mutex> lock(mutex_);
  octree_->prune();
}

std::shared_ptr<const octomap::OcTree> OctomapManager::getOctree() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return octree_;
}

size_t OctomapManager::getNumOccupiedVoxels() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  size_t count = 0;
  for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
    if (octree_->isNodeOccupied(*it)) {
      count++;
    }
  }
  return count;
}

size_t OctomapManager::getMemoryUsage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return octree_->memoryUsage();
}

void OctomapManager::setConfig(const OctomapConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // If resolution changed, need to rebuild
  if (std::abs(config.resolution - config_.resolution) > 1e-6) {
    octree_ = std::make_shared<octomap::OcTree>(config.resolution);
  }

  config_ = config;

  // Update probability parameters
  octree_->setProbHit(config_.hit_probability);
  octree_->setProbMiss(config_.miss_probability);
  octree_->setClampingThresMin(config_.clamping_threshold_min);
  octree_->setClampingThresMax(config_.clamping_threshold_max);
  octree_->setOccupancyThres(config_.occupancy_threshold);
}

bool OctomapManager::isInBounds(const geometry_msgs::msg::Point & point) const
{
  return point.x >= config_.x_min && point.x <= config_.x_max &&
         point.y >= config_.y_min && point.y <= config_.y_max &&
         point.z >= config_.z_min && point.z <= config_.z_max;
}

octomap::point3d OctomapManager::toOctomap(const geometry_msgs::msg::Point & point) const
{
  return octomap::point3d(point.x, point.y, point.z);
}

geometry_msgs::msg::Point OctomapManager::fromOctomap(const octomap::point3d & point) const
{
  geometry_msgs::msg::Point result;
  result.x = point.x();
  result.y = point.y();
  result.z = point.z();
  return result;
}

}  // namespace local_avoidance
