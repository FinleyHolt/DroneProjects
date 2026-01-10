/**
 * @file test_octomap_manager.cpp
 * @brief Unit tests for OctoMap manager
 */

#include <gtest/gtest.h>
#include <memory>
#include <cmath>

#include "local_avoidance/octomap_manager.hpp"

using namespace local_avoidance;

class OctomapManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    OctomapConfig config;
    config.resolution = 0.5;
    config.max_range = 50.0;

    manager_ = std::make_unique<OctomapManager>(config);
  }

  std::unique_ptr<OctomapManager> manager_;
};

TEST_F(OctomapManagerTest, InitializationCreatesEmptyMap)
{
  EXPECT_EQ(manager_->getNumOccupiedVoxels(), 0u);
}

TEST_F(OctomapManagerTest, InsertPointCloudAddsVoxels)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p;
  p.x = 10.0;
  p.y = 0.0;
  p.z = 0.0;
  points.push_back(p);

  manager_->insertPointCloud(points, origin);

  EXPECT_GT(manager_->getNumOccupiedVoxels(), 0u);
}

TEST_F(OctomapManagerTest, IsOccupiedReturnsTrueForInsertedPoints)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  geometry_msgs::msg::Point obstacle;
  obstacle.x = 10.0;
  obstacle.y = 0.0;
  obstacle.z = 0.0;

  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(obstacle);

  manager_->insertPointCloud(points, origin);

  EXPECT_TRUE(manager_->isOccupied(obstacle));
}

TEST_F(OctomapManagerTest, IsOccupiedReturnsFalseForEmptySpace)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  geometry_msgs::msg::Point obstacle;
  obstacle.x = 10.0;
  obstacle.y = 0.0;
  obstacle.z = 0.0;

  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(obstacle);

  manager_->insertPointCloud(points, origin);

  // Check point far from obstacle
  geometry_msgs::msg::Point empty;
  empty.x = 20.0;
  empty.y = 20.0;
  empty.z = 20.0;

  EXPECT_FALSE(manager_->isOccupied(empty));
}

TEST_F(OctomapManagerTest, CastRayFindsObstacle)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  // Insert obstacle at x=10
  geometry_msgs::msg::Point obstacle;
  obstacle.x = 10.0;
  obstacle.y = 0.0;
  obstacle.z = 0.0;

  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(obstacle);

  manager_->insertPointCloud(points, origin);

  // Cast ray in +X direction
  geometry_msgs::msg::Point direction;
  direction.x = 1.0;
  direction.y = 0.0;
  direction.z = 0.0;

  double distance = manager_->castRay(origin, direction, 50.0);

  // Should hit obstacle around 10 meters
  EXPECT_LT(distance, 15.0);
  EXPECT_GT(distance, 5.0);
}

TEST_F(OctomapManagerTest, CastRayReturnsMaxRangeWhenNoObstacle)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  geometry_msgs::msg::Point direction;
  direction.x = 1.0;
  direction.y = 0.0;
  direction.z = 0.0;

  double max_range = 50.0;
  double distance = manager_->castRay(origin, direction, max_range);

  EXPECT_DOUBLE_EQ(distance, max_range);
}

TEST_F(OctomapManagerTest, FindNearestObstacleReturnsClosest)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  // Insert two obstacles
  std::vector<geometry_msgs::msg::Point> points;

  geometry_msgs::msg::Point near;
  near.x = 5.0;
  near.y = 0.0;
  near.z = 0.0;
  points.push_back(near);

  geometry_msgs::msg::Point far;
  far.x = 15.0;
  far.y = 0.0;
  far.z = 0.0;
  points.push_back(far);

  manager_->insertPointCloud(points, origin);

  auto result = manager_->findNearestObstacle(origin, 20.0);

  EXPECT_TRUE(result.occupied);
  EXPECT_LT(result.distance, 7.0);  // Should find the 5m obstacle
}

TEST_F(OctomapManagerTest, GetOccupiedVoxelsReturnsSphereContents)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  // Insert obstacles at various distances
  std::vector<geometry_msgs::msg::Point> points;

  // Close obstacle
  geometry_msgs::msg::Point p1;
  p1.x = 3.0;
  p1.y = 0.0;
  p1.z = 0.0;
  points.push_back(p1);

  // Far obstacle (outside query radius)
  geometry_msgs::msg::Point p2;
  p2.x = 20.0;
  p2.y = 0.0;
  p2.z = 0.0;
  points.push_back(p2);

  manager_->insertPointCloud(points, origin);

  // Query with radius 10
  auto voxels = manager_->getOccupiedVoxels(origin, 10.0);

  // Should only find the close obstacle
  EXPECT_GE(voxels.size(), 1u);

  bool found_close = false;
  for (const auto & v : voxels) {
    double dist = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    EXPECT_LE(dist, 10.0);
    if (std::abs(v.x - 3.0) < 1.0) {
      found_close = true;
    }
  }
  EXPECT_TRUE(found_close);
}

TEST_F(OctomapManagerTest, ClearRemovesAllVoxels)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p;
  p.x = 10.0;
  p.y = 0.0;
  p.z = 0.0;
  points.push_back(p);

  manager_->insertPointCloud(points, origin);
  EXPECT_GT(manager_->getNumOccupiedVoxels(), 0u);

  manager_->clear();
  EXPECT_EQ(manager_->getNumOccupiedVoxels(), 0u);
}

TEST_F(OctomapManagerTest, PruneReducesMemory)
{
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  // Insert dense point cloud
  std::vector<geometry_msgs::msg::Point> points;
  for (double x = 0; x < 10; x += 0.1) {
    for (double y = 0; y < 10; y += 0.1) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = 0.0;
      points.push_back(p);
    }
  }

  manager_->insertPointCloud(points, origin);
  size_t memory_before = manager_->getMemoryUsage();

  manager_->prune();
  size_t memory_after = manager_->getMemoryUsage();

  // Memory should decrease or stay same after pruning
  EXPECT_LE(memory_after, memory_before);
}

TEST_F(OctomapManagerTest, ConfigurationCanBeRetrieved)
{
  const auto & config = manager_->getConfig();

  EXPECT_DOUBLE_EQ(config.resolution, 0.5);
  EXPECT_DOUBLE_EQ(config.max_range, 50.0);
}

TEST_F(OctomapManagerTest, OutOfBoundsPointsAreIgnored)
{
  OctomapConfig config;
  config.resolution = 0.5;
  config.x_min = -10.0;
  config.x_max = 10.0;
  config.y_min = -10.0;
  config.y_max = 10.0;
  config.z_min = -10.0;
  config.z_max = 10.0;

  auto bounded_manager = std::make_unique<OctomapManager>(config);

  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  std::vector<geometry_msgs::msg::Point> points;

  // Point inside bounds
  geometry_msgs::msg::Point inside;
  inside.x = 5.0;
  inside.y = 0.0;
  inside.z = 0.0;
  points.push_back(inside);

  // Point outside bounds
  geometry_msgs::msg::Point outside;
  outside.x = 100.0;  // Way outside x_max
  outside.y = 0.0;
  outside.z = 0.0;
  points.push_back(outside);

  bounded_manager->insertPointCloud(points, origin);

  // Only inside point should be present
  EXPECT_TRUE(bounded_manager->isOccupied(inside));
  EXPECT_FALSE(bounded_manager->isOccupied(outside));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
