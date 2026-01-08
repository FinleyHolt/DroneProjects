cmake_minimum_required(VERSION 3.8)
project(behavior_trees)

# C++17 required for BehaviorTree.CPP v4.x
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(flyby_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  behaviortree_cpp
  behaviortree_ros2
  flyby_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Include directories
include_directories(include)

# ==============================================================================
# BT Node Libraries
# ==============================================================================

# Condition nodes library
add_library(bt_condition_nodes SHARED
  src/bt_nodes/condition_nodes.cpp
)
ament_target_dependencies(bt_condition_nodes ${DEPENDENCIES})

# Action nodes library
add_library(bt_action_nodes SHARED
  src/bt_nodes/action_nodes.cpp
)
ament_target_dependencies(bt_action_nodes ${DEPENDENCIES})

# RL policy action nodes library
add_library(bt_rl_action_nodes SHARED
  src/bt_nodes/rl_action_nodes.cpp
)
ament_target_dependencies(bt_rl_action_nodes ${DEPENDENCIES})

# Planner nodes library
add_library(bt_planner_nodes SHARED
  src/bt_nodes/planner_nodes.cpp
)
ament_target_dependencies(bt_planner_nodes ${DEPENDENCIES})

# ==============================================================================
# Main Executor Node
# ==============================================================================

add_executable(bt_executor_node
  src/bt_executor_node.cpp
)
target_link_libraries(bt_executor_node
  bt_condition_nodes
  bt_action_nodes
  bt_rl_action_nodes
  bt_planner_nodes
)
ament_target_dependencies(bt_executor_node ${DEPENDENCIES})

# ==============================================================================
# Install
# ==============================================================================

# Install libraries
install(TARGETS
  bt_condition_nodes
  bt_action_nodes
  bt_rl_action_nodes
  bt_planner_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  bt_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install behavior tree XML files
install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ==============================================================================
# Testing
# ==============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add unit tests
  # ament_add_gtest(test_condition_nodes test/test_condition_nodes.cpp)
  # target_link_libraries(test_condition_nodes bt_condition_nodes)
  # ament_target_dependencies(test_condition_nodes ${DEPENDENCIES})
endif()

ament_package()
