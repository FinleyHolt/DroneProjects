/**
 * @file global_planner.cpp
 * @brief Implementation of OMPL-based global planner
 */

#include "path_planning/global_planner.hpp"

#include <cmath>
#include <chrono>

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

namespace path_planning
{

GlobalPlanner::GlobalPlanner(const PlannerConfig & config)
: config_(config)
{
}

void GlobalPlanner::setNFZManager(std::shared_ptr<NFZManager> nfz_manager)
{
  nfz_manager_ = nfz_manager;

  // Update altitude bounds from NFZ manager
  if (nfz_manager_) {
    config_.z_min = nfz_manager_->getMinAltitude();
    config_.z_max = nfz_manager_->getMaxAltitude();
  }

  // Re-initialize OMPL with new validity checker
  initialized_ = false;
}

void GlobalPlanner::setBounds(
  double x_min, double x_max,
  double y_min, double y_max,
  double z_min, double z_max)
{
  config_.x_min = x_min;
  config_.x_max = x_max;
  config_.y_min = y_min;
  config_.y_max = y_max;
  config_.z_min = z_min;
  config_.z_max = z_max;

  // Update NFZ manager altitude bounds
  if (nfz_manager_) {
    nfz_manager_->setAltitudeBounds(z_min, z_max);
  }

  // Re-initialize OMPL with new bounds
  initialized_ = false;
}

void GlobalPlanner::setConfig(const PlannerConfig & config)
{
  config_ = config;
  initialized_ = false;
}

void GlobalPlanner::initializeOMPL()
{
  // Create 3D state space (x, y, z)
  state_space_ = std::make_shared<ompl::base::RealVectorStateSpace>(3);

  // Set bounds
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, config_.x_min);
  bounds.setHigh(0, config_.x_max);
  bounds.setLow(1, config_.y_min);
  bounds.setHigh(1, config_.y_max);
  bounds.setLow(2, config_.z_min);
  bounds.setHigh(2, config_.z_max);
  state_space_->setBounds(bounds);

  // Create space information
  space_info_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);

  // Set state validity checker
  space_info_->setStateValidityChecker(
    [this](const ompl::base::State * state) {
      return isStateValid(state);
    });

  // Set state validity checking resolution
  space_info_->setStateValidityCheckingResolution(0.01);

  // Setup space information
  space_info_->setup();

  // Create path simplifier
  simplifier_ = std::make_shared<ompl::geometric::PathSimplifier>(space_info_);

  initialized_ = true;
}

bool GlobalPlanner::isStateValid(const ompl::base::State * state) const
{
  if (!nfz_manager_) {
    return true;  // No NFZ manager, all states valid
  }

  const auto * real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
  double x = real_state->values[0];
  double y = real_state->values[1];
  double z = real_state->values[2];

  // Check if in NFZ
  return !nfz_manager_->isInNFZ(x, y, z);
}

PlanningResult GlobalPlanner::plan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double timeout_sec)
{
  PlanningResult result;

  // Initialize OMPL if needed
  if (!initialized_) {
    initializeOMPL();
  }

  // Use configured timeout if not specified
  if (timeout_sec <= 0) {
    timeout_sec = config_.planning_timeout_sec;
  }

  // Create problem definition
  problem_def_ = std::make_shared<ompl::base::ProblemDefinition>(space_info_);

  // Set start state
  ompl::base::ScopedState<> start_state(state_space_);
  start_state[0] = start.pose.position.x;
  start_state[1] = start.pose.position.y;
  start_state[2] = start.pose.position.z;
  problem_def_->addStartState(start_state);

  // Set goal state
  ompl::base::ScopedState<> goal_state(state_space_);
  goal_state[0] = goal.pose.position.x;
  goal_state[1] = goal.pose.position.y;
  goal_state[2] = goal.pose.position.z;
  problem_def_->setGoalState(goal_state, config_.goal_tolerance);

  // Check if start/goal are valid
  if (!isStateValid(start_state.get())) {
    result.error_message = "Start state is in NFZ";
    return result;
  }
  if (!isStateValid(goal_state.get())) {
    result.error_message = "Goal state is in NFZ";
    return result;
  }

  // Set optimization objective (path length with threat cost)
  auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(space_info_);
  problem_def_->setOptimizationObjective(objective);

  // Create planner
  planner_ = std::make_shared<ompl::geometric::InformedRRTstar>(space_info_);
  planner_->setRange(config_.range);
  planner_->setProblemDefinition(problem_def_);
  planner_->setup();

  // Solve
  auto start_time = std::chrono::steady_clock::now();
  ompl::base::PlannerStatus status = planner_->solve(
    ompl::base::timedPlannerTerminationCondition(timeout_sec));
  auto end_time = std::chrono::steady_clock::now();

  result.planning_time_sec = std::chrono::duration<double>(end_time - start_time).count();

  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
    status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
  {
    // Get solution path
    auto * path = problem_def_->getSolutionPath()->as<ompl::geometric::PathGeometric>();

    // Simplify path if enabled
    if (config_.simplify_path) {
      simplifyPath(*path);
    }

    // Interpolate for smoother path
    path->interpolate();

    // Convert to ROS message
    result.path = convertToROSPath(*path, start.header.frame_id);
    result.path_length = path->length();
    result.success = true;

    // Get iteration count if available
    ompl::base::PlannerData data(space_info_);
    planner_->getPlannerData(data);
    result.iterations = static_cast<int>(data.numVertices());

  } else {
    result.error_message = "No solution found within timeout";
  }

  // Clear planner for next call
  planner_->clear();

  return result;
}

void GlobalPlanner::simplifyPath(ompl::geometric::PathGeometric & path)
{
  // Simplify using various techniques
  simplifier_->simplifyMax(path);
  simplifier_->smoothBSpline(path, config_.simplification_iterations);
}

nav_msgs::msg::Path GlobalPlanner::convertToROSPath(
  const ompl::geometric::PathGeometric & ompl_path,
  const std::string & frame_id) const
{
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = frame_id;

  for (size_t i = 0; i < ompl_path.getStateCount(); ++i) {
    const auto * state = ompl_path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = state->values[0];
    pose.pose.position.y = state->values[1];
    pose.pose.position.z = state->values[2];

    // Compute orientation from direction to next waypoint
    if (i < ompl_path.getStateCount() - 1) {
      const auto * next_state =
        ompl_path.getState(i + 1)->as<ompl::base::RealVectorStateSpace::StateType>();

      double dx = next_state->values[0] - state->values[0];
      double dy = next_state->values[1] - state->values[1];
      double yaw = std::atan2(dy, dx);

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else if (!ros_path.poses.empty()) {
      // Last waypoint: use previous orientation
      pose.pose.orientation = ros_path.poses.back().pose.orientation;
    } else {
      pose.pose.orientation.w = 1.0;
    }

    ros_path.poses.push_back(pose);
  }

  return ros_path;
}

}  // namespace path_planning
