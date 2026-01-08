/**
 * @file test_vfh3d.cpp
 * @brief Unit tests for 3D-VFH+ algorithm
 */

#include <gtest/gtest.h>
#include <memory>
#include <cmath>

#include "local_avoidance/vfh3d.hpp"
#include "local_avoidance/octomap_manager.hpp"

using namespace local_avoidance;

class VFH3DTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    OctomapConfig octo_config;
    octo_config.resolution = 0.5;
    octomap_manager_ = std::make_shared<OctomapManager>(octo_config);

    VFH3DConfig vfh_config;
    vfh_config.azimuth_bins = 36;
    vfh_config.elevation_bins = 18;
    vfh_config.safety_radius = 5.0;
    vfh_config.critical_radius = 2.0;
    vfh_config.lookahead_distance = 20.0;

    vfh_ = std::make_unique<VFH3D>(octomap_manager_, vfh_config);
  }

  std::shared_ptr<OctomapManager> octomap_manager_;
  std::unique_ptr<VFH3D> vfh_;
};

TEST_F(VFH3DTest, EmptyMapReturnsGoalDirection)
{
  // No obstacles in map
  geometry_msgs::msg::Point current_pos;
  current_pos.x = 0.0;
  current_pos.y = 0.0;
  current_pos.z = 30.0;

  geometry_msgs::msg::Point goal_pos;
  goal_pos.x = 100.0;
  goal_pos.y = 0.0;
  goal_pos.z = 30.0;

  double current_heading = 0.0;  // Facing +X

  auto result = vfh_->computeAvoidance(current_pos, current_heading, goal_pos);

  // Should recommend straight ahead (toward goal)
  EXPECT_FALSE(result.emergency_stop);
  EXPECT_FALSE(result.path_blocked);
  EXPECT_NEAR(result.recommended_yaw, 0.0, 0.2);
  EXPECT_GT(result.confidence, 0.5);
}

TEST_F(VFH3DTest, ObstacleInPathTriggersAvoidance)
{
  // Add obstacle directly in path (within lookahead_distance of 20m)
  geometry_msgs::msg::Point current_pos;
  current_pos.x = 0.0;
  current_pos.y = 0.0;
  current_pos.z = 30.0;

  // Insert obstacle points at ~10m in front (within lookahead range)
  std::vector<geometry_msgs::msg::Point> obstacle_points;
  for (double x = 8.0; x <= 12.0; x += 0.5) {
    for (double y = -3.0; y <= 3.0; y += 0.5) {
      for (double z = 27.0; z <= 33.0; z += 0.5) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        obstacle_points.push_back(p);
      }
    }
  }
  octomap_manager_->insertPointCloud(obstacle_points, current_pos);

  geometry_msgs::msg::Point goal_pos;
  goal_pos.x = 100.0;
  goal_pos.y = 0.0;
  goal_pos.z = 30.0;

  double current_heading = 0.0;

  auto result = vfh_->computeAvoidance(current_pos, current_heading, goal_pos);

  // Should detect path blocked and recommend avoidance
  EXPECT_TRUE(result.path_blocked);
  EXPECT_FALSE(result.emergency_stop);
  // Should recommend turning left or right
  EXPECT_NE(result.recommended_yaw, 0.0);
}

TEST_F(VFH3DTest, CloseObstacleTriggersEmergencyStop)
{
  geometry_msgs::msg::Point current_pos;
  current_pos.x = 0.0;
  current_pos.y = 0.0;
  current_pos.z = 30.0;

  // Insert obstacle very close
  std::vector<geometry_msgs::msg::Point> obstacle_points;
  for (double x = 1.0; x <= 1.5; x += 0.25) {
    for (double y = -0.5; y <= 0.5; y += 0.25) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = 30.0;
      obstacle_points.push_back(p);
    }
  }
  octomap_manager_->insertPointCloud(obstacle_points, current_pos);

  geometry_msgs::msg::Point goal_pos;
  goal_pos.x = 100.0;
  goal_pos.y = 0.0;
  goal_pos.z = 30.0;

  auto result = vfh_->computeAvoidance(current_pos, 0.0, goal_pos);

  // Should trigger emergency stop
  EXPECT_TRUE(result.emergency_stop);
  EXPECT_LT(result.closest_obstacle, 2.0);
}

TEST_F(VFH3DTest, HistogramHasCorrectSize)
{
  const auto & histogram = vfh_->getHistogram();
  const auto & config = vfh_->getConfig();

  EXPECT_EQ(histogram.size(),
    static_cast<size_t>(config.azimuth_bins * config.elevation_bins));
}

TEST_F(VFH3DTest, ResetClearsState)
{
  // Add some obstacle and compute
  geometry_msgs::msg::Point pos;
  pos.x = 0.0;
  pos.y = 0.0;
  pos.z = 0.0;

  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p;
  p.x = 10.0;
  p.y = 0.0;
  p.z = 0.0;
  points.push_back(p);
  octomap_manager_->insertPointCloud(points, pos);

  geometry_msgs::msg::Point goal;
  goal.x = 100.0;
  goal.y = 0.0;
  goal.z = 0.0;

  vfh_->computeAvoidance(pos, 0.0, goal);

  // Reset
  vfh_->reset();

  // Histogram should be cleared
  const auto & histogram = vfh_->getHistogram();
  for (const auto & bin : histogram) {
    EXPECT_DOUBLE_EQ(bin.density, 0.0);
    EXPECT_FALSE(bin.blocked);
  }
}

TEST_F(VFH3DTest, GoalDistanceCalculatedCorrectly)
{
  geometry_msgs::msg::Point current_pos;
  current_pos.x = 0.0;
  current_pos.y = 0.0;
  current_pos.z = 0.0;

  geometry_msgs::msg::Point goal_pos;
  goal_pos.x = 30.0;
  goal_pos.y = 40.0;
  goal_pos.z = 0.0;

  auto result = vfh_->computeAvoidance(current_pos, 0.0, goal_pos);

  // Distance should be 50 (3-4-5 triangle scaled by 10)
  EXPECT_NEAR(result.goal_distance, 50.0, 0.01);
}

TEST_F(VFH3DTest, GoalDirectionCalculatedCorrectly)
{
  geometry_msgs::msg::Point current_pos;
  current_pos.x = 0.0;
  current_pos.y = 0.0;
  current_pos.z = 0.0;

  geometry_msgs::msg::Point goal_pos;
  goal_pos.x = 100.0;
  goal_pos.y = 0.0;
  goal_pos.z = 0.0;

  auto result = vfh_->computeAvoidance(current_pos, 0.0, goal_pos);

  // Goal direction should be +X
  EXPECT_NEAR(result.goal_direction.x, 1.0, 0.01);
  EXPECT_NEAR(result.goal_direction.y, 0.0, 0.01);
  EXPECT_NEAR(result.goal_direction.z, 0.0, 0.01);
}

TEST_F(VFH3DTest, ConfigurationCanBeUpdated)
{
  VFH3DConfig new_config = vfh_->getConfig();
  new_config.safety_radius = 10.0;
  new_config.critical_radius = 5.0;

  vfh_->setConfig(new_config);

  EXPECT_DOUBLE_EQ(vfh_->getConfig().safety_radius, 10.0);
  EXPECT_DOUBLE_EQ(vfh_->getConfig().critical_radius, 5.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
