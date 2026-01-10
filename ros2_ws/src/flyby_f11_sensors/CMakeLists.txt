cmake_minimum_required(VERSION 3.8)
project(flyby_f11_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalCommand.msg"
  "msg/GimbalState.msg"
  "msg/ISRCameraState.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Install Python package (the flyby_f11_sensors directory)
install(DIRECTORY
  flyby_f11_sensors/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables
install(PROGRAMS
  flyby_f11_sensors/gimbal_controller_node.py
  flyby_f11_sensors/isr_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
