# Custom ROS 2 Messages for Flyby-F11

## Overview

This guide covers custom message, service, and action definitions for the Flyby-F11 autonomy stack. These interfaces bridge the perception pipeline to the Prolog reasoning engine.

## Package Structure

```
flyby_f11_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── PerceptionObject.msg
│   ├── PerceptionObjectArray.msg
│   ├── BoundingBox.msg
│   ├── SpatialRelation.msg
│   ├── SceneGraph.msg
│   └── MissionStatus.msg
├── srv/
│   ├── PrologQuery.srv
│   ├── PrologAssert.srv
│   ├── PrologRetract.srv
│   └── ValidateKnowledgeBase.srv
└── action/
    ├── ExecuteReasoning.action
    └── ExecuteMission.action
```

## Message Definitions

### PerceptionObject.msg

Single object detection from vision pipeline.

```
# Header with timestamp
std_msgs/Header header

# Unique tracking ID (persistent across frames)
uint32 track_id

# Object class name (e.g., 'person', 'car', 'building')
string class_name

# Detection confidence [0.0, 1.0]
float32 confidence

# 2D bounding box in image coordinates
BoundingBox bbox

# 3D position in world frame (from depth + pose estimation)
geometry_msgs/Point world_position

# Object velocity (if available from tracking)
geometry_msgs/Vector3 velocity

# Additional attributes (color, size, etc.)
string[] attributes
```

### BoundingBox.msg

2D bounding box in image coordinates.

```
# Top-left corner
float32 x_min
float32 y_min

# Bottom-right corner
float32 x_max
float32 y_max

# Box dimensions (computed)
float32 width
float32 height

# Normalized coordinates [0.0, 1.0] (optional)
float32 x_min_norm
float32 y_min_norm
float32 x_max_norm
float32 y_max_norm
```

### PerceptionObjectArray.msg

Array of object detections from a single perception frame.

```
# Header with timestamp
std_msgs/Header header

# Frame ID (camera frame)
string frame_id

# Array of detected objects
PerceptionObject[] objects

# Total number of detections (before filtering)
uint32 total_detections

# Number of tracked objects
uint32 num_tracked
```

### SpatialRelation.msg

Spatial relationship between two objects.

```
# Relation type (e.g., 'left_of', 'above', 'near', 'far')
string relation_type

# Subject object ID
string subject_id

# Object object ID
string object_id

# Confidence in relation [0.0, 1.0]
float32 confidence

# Distance (if applicable)
float32 distance

# Direction vector (if applicable)
geometry_msgs/Vector3 direction
```

### SceneGraph.msg

Complete scene understanding with objects and relations.

```
# Header with timestamp
std_msgs/Header header

# All objects in scene
PerceptionObject[] objects

# Spatial relations between objects
SpatialRelation[] relations

# Scene context (e.g., 'indoor', 'outdoor', 'urban')
string scene_context

# Scene confidence [0.0, 1.0]
float32 confidence

# Additional metadata
string[] tags
```

### MissionStatus.msg

Current mission execution state.

```
# Header with timestamp
std_msgs/Header header

# Mission ID
string mission_id

# Current mission state
# Possible values: IDLE, PLANNING, EXECUTING, PAUSED, COMPLETED, FAILED
string state

# Current goal description
string current_goal

# Progress [0.0, 1.0]
float32 progress

# Estimated time remaining (seconds)
float32 estimated_time_remaining

# Active constraints
string[] active_constraints

# Error message (if state is FAILED)
string error_message
```

## Service Definitions

### PrologQuery.srv

Query the Prolog knowledge base.

```
# Request
string query

---

# Response
bool success
string[] results
int32 num_results
string error_message
```

**Usage Example**:
```python
# Create service client
client = node.create_client(PrologQuery, '/reasoning/query')

# Send request
request = PrologQuery.Request()
request.query = "detected_object(ID, person, _, _, _)"
future = client.call_async(request)

# Process response
rclpy.spin_until_future_complete(node, future)
response = future.result()

if response.success:
    print(f"Found {response.num_results} results:")
    for result in response.results:
        print(f"  {result}")
```

### PrologAssert.srv

Assert facts to the Prolog knowledge base.

```
# Request
string[] facts

---

# Response
bool success
int32 num_asserted
string error_message
```

**Usage Example**:
```python
request = PrologAssert.Request()
request.facts = [
    "detected_object(obj_1, person, [100, 200], 0.85, 123456)",
    "detected_object(obj_2, car, [300, 400], 0.92, 123456)"
]

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

if future.result().success:
    print(f"Asserted {future.result().num_asserted} facts")
```

### PrologRetract.srv

Retract facts from the Prolog knowledge base.

```
# Request
string pattern

---

# Response
bool success
int32 num_retracted
string error_message
```

**Usage Example**:
```python
request = PrologRetract.Request()
request.pattern = "detected_object(_, _, _, _, _)"  # Retract all objects

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

if future.result().success:
    print(f"Retracted {future.result().num_retracted} facts")
```

### ValidateKnowledgeBase.srv

Validate consistency of the Prolog knowledge base.

```
# Request
bool deep_validation  # Perform deep semantic validation

---

# Response
bool is_valid
string[] errors
string[] warnings
int32 num_facts
int32 num_rules
```

## Action Definitions

### ExecuteReasoning.action

Execute complex reasoning task.

```
# Goal
string task_name
string query
string[] parameters

---

# Result
bool success
string[] solutions
int32 num_solutions
string error_message
float32 execution_time

---

# Feedback
string status
float32 progress
string current_step
```

**Usage Example**:
```python
from rclpy.action import ActionClient
from flyby_f11_interfaces.action import ExecuteReasoning

# Create action client
action_client = ActionClient(node, ExecuteReasoning, '/reasoning/execute')

# Wait for server
action_client.wait_for_server()

# Send goal
goal = ExecuteReasoning.Goal()
goal.task_name = 'find_landing_zone'
goal.query = "safe_landing_zone(Zone, Constraints)"
goal.parameters = ['min_area:10', 'max_slope:15']

send_goal_future = action_client.send_goal_async(
    goal,
    feedback_callback=feedback_callback
)

# Handle result
rclpy.spin_until_future_complete(node, send_goal_future)
goal_handle = send_goal_future.result()

result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future)
result = result_future.result().result

if result.success:
    print(f"Found {result.num_solutions} solutions in {result.execution_time}s")
```

### ExecuteMission.action

Execute mission with symbolic goals.

```
# Goal
string mission_type
string[] goals
string[] constraints
float32 timeout

---

# Result
bool success
string final_state
string[] completed_goals
string[] failed_goals
float32 execution_time
string error_message

---

# Feedback
string current_goal
float32 progress
string state
geometry_msgs/Pose current_position
string[] active_constraints
```

**Usage Example**:
```python
from flyby_f11_interfaces.action import ExecuteMission

# Create action client
action_client = ActionClient(node, ExecuteMission, '/mission/execute')

# Send goal
goal = ExecuteMission.Goal()
goal.mission_type = 'search_and_rescue'
goal.goals = [
    'search_area(zone_1)',
    'identify_targets(person)',
    'report_findings(base_station)'
]
goal.constraints = [
    'maintain_altitude(10)',
    'avoid_obstacles(true)',
    'battery_reserve(20)'
]
goal.timeout = 600.0  # 10 minutes

send_goal_future = action_client.send_goal_async(
    goal,
    feedback_callback=lambda feedback: print(
        f"Progress: {feedback.feedback.progress:.1%} | "
        f"Goal: {feedback.feedback.current_goal}"
    )
)
```

## Package Configuration

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>flyby_f11_interfaces</name>
  <version>0.1.0</version>
  <description>Custom ROS 2 interfaces for Flyby-F11 autonomy stack</description>
  <maintainer email="finley@example.com">Finley Holt</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rosidl_default_generators</depend>
  <depend>rosidl_default_runtime</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>action_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(flyby_f11_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/BoundingBox.msg"
  "msg/PerceptionObject.msg"
  "msg/PerceptionObjectArray.msg"
  "msg/SpatialRelation.msg"
  "msg/SceneGraph.msg"
  "msg/MissionStatus.msg"

  # Services
  "srv/PrologQuery.srv"
  "srv/PrologAssert.srv"
  "srv/PrologRetract.srv"
  "srv/ValidateKnowledgeBase.srv"

  # Actions
  "action/ExecuteReasoning.action"
  "action/ExecuteMission.action"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    action_msgs
)

ament_package()
```

## Building the Package

```bash
# Navigate to workspace
cd /home/finley/Github/DroneProjects/flyby-f11/ros2_ws

# Build interfaces package
colcon build --packages-select flyby_f11_interfaces

# Source workspace
source install/setup.bash

# Verify interfaces
ros2 interface list | grep flyby_f11

# Show message definition
ros2 interface show flyby_f11_interfaces/msg/PerceptionObject
```

## Using Custom Interfaces

### In Python Nodes

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import custom messages
from flyby_f11_interfaces.msg import (
    PerceptionObject,
    PerceptionObjectArray,
    BoundingBox
)

class PerceptionPublisher(Node):
    def __init__(self):
        super().__init__('perception_publisher')

        # Create publisher
        self.publisher = self.create_publisher(
            PerceptionObjectArray,
            '/perception/objects',
            10
        )

        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_detections)

    def publish_detections(self):
        """Publish object detections."""
        msg = PerceptionObjectArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        # Create detection
        obj = PerceptionObject()
        obj.track_id = 1
        obj.class_name = 'person'
        obj.confidence = 0.85

        # Bounding box
        bbox = BoundingBox()
        bbox.x_min = 100.0
        bbox.y_min = 200.0
        bbox.x_max = 150.0
        bbox.y_max = 250.0
        bbox.width = 50.0
        bbox.height = 50.0
        obj.bbox = bbox

        # Add to array
        msg.objects.append(obj)
        msg.total_detections = 1
        msg.num_tracked = 1

        # Publish
        self.publisher.publish(msg)
```

### In C++ Nodes

```cpp
#include <rclcpp/rclcpp.hpp>
#include <flyby_f11_interfaces/msg/perception_object_array.hpp>

class PerceptionSubscriber : public rclcpp::Node
{
public:
  PerceptionSubscriber()
  : Node("perception_subscriber")
  {
    subscription_ = this->create_subscription<
      flyby_f11_interfaces::msg::PerceptionObjectArray>(
      "/perception/objects",
      10,
      std::bind(&PerceptionSubscriber::callback, this, std::placeholders::_1));
  }

private:
  void callback(const flyby_f11_interfaces::msg::PerceptionObjectArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received %zu objects", msg->objects.size());

    for (const auto& obj : msg->objects) {
      RCLCPP_INFO(
        this->get_logger(),
        "Object %u: %s (confidence: %.2f)",
        obj.track_id,
        obj.class_name.c_str(),
        obj.confidence);
    }
  }

  rclcpp::Subscription<flyby_f11_interfaces::msg::PerceptionObjectArray>::SharedPtr subscription_;
};
```

## Message Design Best Practices

### 1. Use Standard Messages When Possible

Prefer standard ROS 2 messages (geometry_msgs, sensor_msgs) for common data types:

```
# Good: Reuse standard messages
geometry_msgs/Point world_position

# Avoid: Custom position message
float32 x
float32 y
float32 z
```

### 2. Include Timestamps

Always include header with timestamp for temporal reasoning:

```
std_msgs/Header header  # Contains timestamp and frame_id
```

### 3. Provide Context

Include metadata for better semantic understanding:

```
# Object detection with context
string class_name         # What it is
float32 confidence        # How certain
string[] attributes       # Additional properties
geometry_msgs/Vector3 velocity  # Temporal context
```

### 4. Design for Extensibility

Use arrays for variable-length data:

```
# Flexible scene representation
PerceptionObject[] objects
SpatialRelation[] relations
string[] tags
```

### 5. Validation Fields

Include validation and quality metrics:

```
bool is_valid
float32 confidence
int32 detection_count
string error_message
```

## Testing Custom Interfaces

### Publishing Test Data

```bash
# Publish test message
ros2 topic pub /perception/objects flyby_f11_interfaces/msg/PerceptionObjectArray \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera'}, \
    objects: [{track_id: 1, class_name: 'person', confidence: 0.85}], \
    total_detections: 1, num_tracked: 1}"

# Echo topic
ros2 topic echo /perception/objects
```

### Calling Test Services

```bash
# Call Prolog query service
ros2 service call /reasoning/query flyby_f11_interfaces/srv/PrologQuery \
  "{query: 'detected_object(ID, person, _, _, _)'}"
```

### Testing Actions

```bash
# Send action goal
ros2 action send_goal /reasoning/execute flyby_f11_interfaces/action/ExecuteReasoning \
  "{task_name: 'test_reasoning', query: 'test_query(X)', parameters: []}"
```

## References

- ROS 2 Custom Interfaces Tutorial: `/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/interfaces/custom_interfaces.html`
- ROS 2 Actions Tutorial: `/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/tutorials/action_servers.html`
- Standard ROS 2 Messages: https://docs.ros.org/en/humble/p/common_interfaces/
