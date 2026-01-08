/**
 * @file global_planner.hpp
 * @brief OMPL-based global path planner for 3D UAV navigation
 *
 * Implements Informed RRT* for optimal path planning with NFZ avoidance
 * and threat zone cost integration.
 */

#ifndef PATH_PLANNING__GLOBAL_PLANNER_HPP_
#define PATH_PLANNING__GLOBAL_PLANNER_HPP_

#include <memory>
#include <vector>
#include <optional>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "path_planning/nfz_manager.hpp"

namespace path_planning
{

/**
 * @brief Planning result with path and metadata
 */
struct PlanningResult
{
  nav_msgs::msg::Path path;
  bool success{false};
  double path_length{0.0};
  double planning_time_sec{0.0};
  int iterations{0};
  std::string error_message;
};

/**
 * @brief Configuration for the global planner
 */
struct PlannerConfig
{
  // State space bounds (meters)
  double x_min{-1000.0};
  double x_max{1000.0};
  double y_min{-1000.0};
  double y_max{1000.0};
  double z_min{20.0};
  double z_max{120.0};

  // Planning parameters
  double planning_timeout_sec{2.0};
  double goal_tolerance{2.0};
  double range{50.0};  // RRT range parameter

  // Path simplification
  bool simplify_path{true};
  int simplification_iterations{10};

  // Threat cost weight
  double threat_cost_weight{1.0};
};

/**
 * @brief OMPL-based global planner with Informed RRT*
 *
 * Provides 3D path planning for UAV navigation while avoiding NFZs
 * and minimizing exposure to threat zones.
 */
class GlobalPlanner
{
public:
  /**
   * @brief Constructor with configuration
   * @param config Planner configuration
   */
  explicit GlobalPlanner(const PlannerConfig & config = PlannerConfig());

  /**
   * @brief Set NFZ manager for collision checking
   * @param nfz_manager Shared pointer to NFZ manager
   */
  void setNFZManager(std::shared_ptr<NFZManager> nfz_manager);

  /**
   * @brief Update planning state space bounds
   * @param x_min, x_max X bounds (meters)
   * @param y_min, y_max Y bounds (meters)
   * @param z_min, z_max Z bounds (meters, altitude)
   */
  void setBounds(
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max);

  /**
   * @brief Plan a path from start to goal
   * @param start Start pose
   * @param goal Goal pose
   * @param timeout_sec Planning timeout in seconds
   * @return Planning result with path if successful
   */
  PlanningResult plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double timeout_sec = -1.0);

  /**
   * @brief Get current configuration
   */
  const PlannerConfig & getConfig() const { return config_; }

  /**
   * @brief Update configuration
   */
  void setConfig(const PlannerConfig & config);

private:
  /**
   * @brief Initialize OMPL state space and planner
   */
  void initializeOMPL();

  /**
   * @brief State validity checker callback for OMPL
   */
  bool isStateValid(const ompl::base::State * state) const;

  /**
   * @brief Convert OMPL path to ROS nav_msgs::Path
   */
  nav_msgs::msg::Path convertToROSPath(
    const ompl::geometric::PathGeometric & ompl_path,
    const std::string & frame_id) const;

  /**
   * @brief Simplify the path using OMPL path simplifier
   */
  void simplifyPath(ompl::geometric::PathGeometric & path);

  PlannerConfig config_;
  std::shared_ptr<NFZManager> nfz_manager_;

  // OMPL components
  std::shared_ptr<ompl::base::RealVectorStateSpace> state_space_;
  std::shared_ptr<ompl::base::SpaceInformation> space_info_;
  std::shared_ptr<ompl::base::ProblemDefinition> problem_def_;
  std::shared_ptr<ompl::geometric::InformedRRTstar> planner_;
  std::shared_ptr<ompl::geometric::PathSimplifier> simplifier_;

  bool initialized_{false};
};

}  // namespace path_planning

#endif  // PATH_PLANNING__GLOBAL_PLANNER_HPP_
