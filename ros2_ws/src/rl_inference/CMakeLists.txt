cmake_minimum_required(VERSION 3.8)
project(rl_inference)

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
find_package(sensor_msgs REQUIRED)
find_package(flyby_msgs REQUIRED)

# Find CUDA and TensorRT
find_package(CUDA)

# TensorRT paths (typical Jetson installation)
set(TENSORRT_ROOT "/usr/lib/aarch64-linux-gnu" CACHE PATH "TensorRT root directory")
find_library(TENSORRT_LIBRARY nvinfer HINTS ${TENSORRT_ROOT})
find_library(TENSORRT_ONNXPARSER nvonnxparser HINTS ${TENSORRT_ROOT})
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS /usr/include/aarch64-linux-gnu /usr/include)

set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  flyby_msgs
)

# Include directories
include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
)

# ==============================================================================
# Libraries
# ==============================================================================

# TensorRT Policy library
add_library(tensorrt_policy SHARED
  src/tensorrt_policy.cpp
)
target_link_libraries(tensorrt_policy
  ${TENSORRT_LIBRARY}
  ${TENSORRT_ONNXPARSER}
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(tensorrt_policy ${DEPENDENCIES})

# ==============================================================================
# Nodes
# ==============================================================================

# RL Inference Node
add_executable(rl_inference_node
  src/rl_inference_node.cpp
)
target_link_libraries(rl_inference_node
  tensorrt_policy
)
ament_target_dependencies(rl_inference_node ${DEPENDENCIES})

# ==============================================================================
# Install
# ==============================================================================

# Install libraries
install(TARGETS
  tensorrt_policy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  rl_inference_node
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

# Install model files (TensorRT engines)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
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
