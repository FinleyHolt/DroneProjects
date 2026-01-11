cmake_minimum_required(VERSION 3.8)
project(local_avoidance)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(flyby_msgs REQUIRED)

# Find OctoMap
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common filters)

set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  flyby_msgs
  octomap_msgs
)

# Include directories
include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ==============================================================================
# Libraries
# ==============================================================================

# OctoMap Manager library
add_library(octomap_manager SHARED
  src/octomap_manager.cpp
)
target_link_libraries(octomap_manager
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(octomap_manager ${DEPENDENCIES})

# Corridor detector library
add_library(corridor_detector SHARED
  src/corridor_detector.cpp
)
target_link_libraries(corridor_detector
  octomap_manager
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(corridor_detector ${DEPENDENCIES})

# 3D-VFH+ library
add_library(vfh3d SHARED
  src/vfh3d.cpp
)
target_link_libraries(vfh3d
  octomap_manager
  corridor_detector
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(vfh3d ${DEPENDENCIES})

# ==============================================================================
# Nodes
# ==============================================================================

# Local Avoidance Node
add_executable(local_avoidance_node
  src/local_avoidance_node.cpp
)
target_link_libraries(local_avoidance_node
  vfh3d
  octomap_manager
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(local_avoidance_node ${DEPENDENCIES})

# ==============================================================================
# Install
# ==============================================================================

# Install libraries
install(TARGETS
  octomap_manager
  corridor_detector
  vfh3d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  local_avoidance_node
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
# Export for downstream packages
# ==============================================================================

ament_export_include_directories(include)
ament_export_libraries(octomap_manager corridor_detector vfh3d)
ament_export_dependencies(${DEPENDENCIES} octomap)

# ==============================================================================
# Testing
# ==============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(test_vfh3d test/test_vfh3d.cpp)
  target_link_libraries(test_vfh3d vfh3d octomap_manager ${OCTOMAP_LIBRARIES})
  ament_target_dependencies(test_vfh3d ${DEPENDENCIES})

  ament_add_gtest(test_octomap_manager test/test_octomap_manager.cpp)
  target_link_libraries(test_octomap_manager octomap_manager ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
  ament_target_dependencies(test_octomap_manager ${DEPENDENCIES})
endif()

ament_package()
