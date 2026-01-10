cmake_minimum_required(VERSION 3.8)
project(vampire_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReasoningQuery.msg"
  "msg/ReasoningResult.msg"
  "srv/VampireQuery.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

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

# Install executables
install(PROGRAMS
  vampire_bridge/vampire_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_cache_manager test/test_cache_manager.py)
  ament_add_pytest_test(test_query_builder test/test_query_builder.py)
  ament_add_pytest_test(test_result_parser test/test_result_parser.py)
endif()

ament_package()
