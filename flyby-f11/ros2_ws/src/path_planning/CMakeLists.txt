cmake_minimum_required(VERSION 3.8)
project(path_planning)

# C++17 for modern features
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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(flyby_msgs REQUIRED)

# Find OMPL
find_package(ompl REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  flyby_msgs
)

# Include directories
include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

# ==============================================================================
# Libraries
# ==============================================================================

# NFZ Manager library
add_library(nfz_manager SHARED
  src/nfz_manager.cpp
)
ament_target_dependencies(nfz_manager ${DEPENDENCIES})

# Global Planner library
add_library(global_planner SHARED
  src/global_planner.cpp
)
target_link_libraries(global_planner
  nfz_manager
  ${OMPL_LIBRARIES}
)
ament_target_dependencies(global_planner ${DEPENDENCIES})

# Trajectory Controller library
add_library(trajectory_controller SHARED
  src/trajectory_controller.cpp
)
ament_target_dependencies(trajectory_controller ${DEPENDENCIES})

# ==============================================================================
# Nodes
# ==============================================================================

# Global Planner Node (ROS2 Action Server)
add_executable(global_planner_node
  src/global_planner_node.cpp
)
target_link_libraries(global_planner_node
  global_planner
  nfz_manager
  ${OMPL_LIBRARIES}
)
ament_target_dependencies(global_planner_node ${DEPENDENCIES})

# Trajectory Controller Node
add_executable(trajectory_controller_node
  src/trajectory_controller_node.cpp
)
target_link_libraries(trajectory_controller_node
  trajectory_controller
)
ament_target_dependencies(trajectory_controller_node ${DEPENDENCIES})

# ==============================================================================
# Install
# ==============================================================================

# Install libraries
install(TARGETS
  nfz_manager
  global_planner
  trajectory_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  global_planner_node
  trajectory_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
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
endif()

ament_package()
