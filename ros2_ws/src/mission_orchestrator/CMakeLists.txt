cmake_minimum_required(VERSION 3.8)
project(mission_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate interfaces (messages, services, actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/Waypoint.msg"
  "msg/MissionPlan.msg"
  "msg/MissionStatus.msg"
  "msg/SafetyState.msg"
  # Services
  "srv/VerifyMission.srv"
  "srv/StartMission.srv"
  # Actions
  "action/ExecuteMission.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  mission_orchestrator/mission_manager_node.py
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

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mission_state_machine test/test_mission_state_machine.py)
  ament_add_pytest_test(test_mission_lifecycle test/test_mission_lifecycle.py)
endif()

ament_package()
