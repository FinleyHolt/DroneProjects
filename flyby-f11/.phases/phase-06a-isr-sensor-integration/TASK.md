# Phase 6a: ISR Sensor Integration

## Overview

Create ROS 2 packages for the Flyby F-11's ISR sensor payloads, including the Gremsy VIO gimbal/camera system, Ouster LiDAR, and thermal sensors. This phase establishes the sensor interfaces required for perception grounding and RL training.

## Human Description

The F-11 uses mission-configurable ISR (Intelligence, Surveillance, Reconnaissance) payloads rather than fixed sensors like the project-drone's RealSense cameras. This phase creates:

1. **flyby_f11_sensors** - ROS 2 drivers/interfaces for ISR payloads
2. **flyby_f11_bringup** - Launch files for different payload configurations
3. **Gazebo sensor bridges** - Connect simulation sensors to ROS 2 topics

The Gazebo simulation models already exist (f11_isr_camera, f11_lidar, f11_multispectral). This phase bridges them to ROS 2.

## AI Agent Instructions

### Prerequisites
- Phase 6 completed (mission orchestration)
- Gazebo simulation models exist in `simulation/models/`
- Understanding of ROS 2 message types and Gazebo-ROS bridges

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. Create flyby_f11_sensors Package

**Location:** `ros2_ws/src/flyby_f11_sensors/`

```bash
cd ros2_ws/src
ros2 pkg create flyby_f11_sensors --build-type ament_python \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

**Package Structure:**
```
flyby_f11_sensors/
├── package.xml
├── setup.py
├── setup.cfg
├── flyby_f11_sensors/
│   ├── __init__.py
│   ├── gimbal_controller_node.py
│   ├── isr_camera_node.py
│   ├── lidar_interface_node.py
│   └── thermal_processor_node.py
├── msg/
│   ├── GimbalCommand.msg
│   ├── GimbalState.msg
│   └── ISRCameraState.msg
├── config/
│   ├── gimbal_params.yaml
│   └── camera_params.yaml
├── launch/
│   ├── isr_camera.launch.py
│   ├── lidar.launch.py
│   └── sensors_all.launch.py
└── test/
    └── test_gimbal_controller.py
```

#### 2. Implement GimbalControllerNode

```python
# flyby_f11_sensors/gimbal_controller_node.py
"""
3-axis gimbal controller for F-11 ISR camera payload.
Interfaces with Gazebo joint controllers in simulation.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
from flyby_f11_sensors.msg import GimbalCommand, GimbalState
import math

class GimbalControllerNode(Node):
    def __init__(self):
        super().__init__('gimbal_controller')

        # Gimbal limits from ontology (isr_extensions.kif)
        self.yaw_limits = (-6.02, 6.02)    # ±345 degrees
        self.pitch_limits = (-2.09, 2.09)  # ±120 degrees
        self.roll_limits = (-0.785, 0.785) # ±45 degrees

        # Current state
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0

        # Subscribe to gimbal commands
        self.cmd_sub = self.create_subscription(
            GimbalCommand,
            '/f11/gimbal/command',
            self.command_callback,
            10
        )

        # Publish to Gazebo joint controllers
        self.yaw_pub = self.create_publisher(
            Float64, '/f11_isr/gimbal/yaw', 10)
        self.pitch_pub = self.create_publisher(
            Float64, '/f11_isr/gimbal/pitch', 10)
        self.roll_pub = self.create_publisher(
            Float64, '/f11_isr/gimbal/roll', 10)

        # Publish gimbal state
        self.state_pub = self.create_publisher(
            GimbalState, '/f11/gimbal/state', 10)

        # State publishing timer (50 Hz)
        self.timer = self.create_timer(0.02, self.publish_state)

        self.get_logger().info('Gimbal controller initialized')
        self.get_logger().info(f'Limits: yaw={self.yaw_limits}, pitch={self.pitch_limits}, roll={self.roll_limits}')

    def command_callback(self, msg: GimbalCommand):
        # Clamp to limits
        yaw = max(self.yaw_limits[0], min(self.yaw_limits[1], msg.yaw))
        pitch = max(self.pitch_limits[0], min(self.pitch_limits[1], msg.pitch))
        roll = max(self.roll_limits[0], min(self.roll_limits[1], msg.roll))

        # Warn if commanded outside limits
        if yaw != msg.yaw or pitch != msg.pitch or roll != msg.roll:
            self.get_logger().warn(f'Gimbal command clamped to limits')

        # Publish to Gazebo
        self.yaw_pub.publish(Float64(data=yaw))
        self.pitch_pub.publish(Float64(data=pitch))
        self.roll_pub.publish(Float64(data=roll))

        # Update internal state
        self.current_yaw = yaw
        self.current_pitch = pitch
        self.current_roll = roll

    def publish_state(self):
        state = GimbalState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.yaw = self.current_yaw
        state.pitch = self.current_pitch
        state.roll = self.current_roll
        state.yaw_limits = list(self.yaw_limits)
        state.pitch_limits = list(self.pitch_limits)
        state.roll_limits = list(self.roll_limits)
        self.state_pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3. Create Message Definitions

```
# msg/GimbalCommand.msg
# Gimbal angle command (radians)
float64 yaw    # -6.02 to +6.02 rad
float64 pitch  # -2.09 to +2.09 rad
float64 roll   # -0.785 to +0.785 rad
```

```
# msg/GimbalState.msg
# Current gimbal state
std_msgs/Header header
float64 yaw
float64 pitch
float64 roll
float64[2] yaw_limits
float64[2] pitch_limits
float64[2] roll_limits
```

```
# msg/ISRCameraState.msg
# ISR camera state
std_msgs/Header header
bool camera_active
string mode           # "rgb", "thermal", "hybrid"
float64 zoom_level    # 1.0 to 20.0
float64 exposure
```

#### 4. Create flyby_f11_bringup Package

**Location:** `ros2_ws/src/flyby_f11_bringup/`

```bash
cd ros2_ws/src
ros2 pkg create flyby_f11_bringup --build-type ament_python \
  --dependencies rclpy launch_ros flyby_f11_sensors vampire_bridge mission_orchestrator
```

**Launch Files:**

```python
# launch/simulation.launch.py
"""Launch F-11 simulation with ISR camera payload."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('flyby_f11_bringup')

    return LaunchDescription([
        # Gazebo with F-11 world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'flyby_training.world'],
            output='screen'
        ),

        # Spawn F-11 with ISR camera
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-name', 'f11_isr',
                 '-file', 'f11_isr_camera'],
            output='screen'
        ),

        # Gimbal controller
        Node(
            package='flyby_f11_sensors',
            executable='gimbal_controller_node',
            name='gimbal_controller',
            output='screen'
        ),

        # Gazebo-ROS bridges for sensors
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/f11_isr/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/f11_isr/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ],
            output='screen'
        ),
    ])
```

#### 5. Create Gazebo-ROS Sensor Bridges

Configure bridges for all ISR sensor data:

```yaml
# config/gz_bridges.yaml
- ros_topic_name: "/f11/camera/image_raw"
  gz_topic_name: "/world/flyby_training/model/f11_isr/link/camera_link/sensor/isr_camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/f11/imu"
  gz_topic_name: "/world/flyby_training/model/f11_isr/link/base_link/sensor/imu/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/f11/gps"
  gz_topic_name: "/world/flyby_training/model/f11_isr/link/base_link/sensor/navsat/navsat"
  ros_type_name: "sensor_msgs/msg/NavSatFix"
  gz_type_name: "gz.msgs.NavSat"
  direction: GZ_TO_ROS
```

#### 6. Integration Testing

Create tests to verify sensor data flows correctly:

```python
# test/test_gimbal_controller.py
import pytest
import rclpy
from rclpy.node import Node
from flyby_f11_sensors.msg import GimbalCommand, GimbalState
import time

def test_gimbal_limits():
    """Verify gimbal respects ontology-defined limits."""
    rclpy.init()
    node = rclpy.create_node('test_gimbal')

    # Subscribe to state
    received_states = []
    node.create_subscription(
        GimbalState,
        '/f11/gimbal/state',
        lambda msg: received_states.append(msg),
        10
    )

    # Publish command exceeding limits
    pub = node.create_publisher(GimbalCommand, '/f11/gimbal/command', 10)
    cmd = GimbalCommand()
    cmd.yaw = 10.0  # Exceeds limit of 6.02
    cmd.pitch = 0.0
    cmd.roll = 0.0
    pub.publish(cmd)

    # Spin and wait
    rclpy.spin_once(node, timeout_sec=0.5)

    # Verify state is clamped
    assert len(received_states) > 0
    assert received_states[-1].yaw <= 6.02

    node.destroy_node()
    rclpy.shutdown()
```

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `flyby_f11_sensors` package builds successfully
- [ ] `flyby_f11_bringup` package builds successfully
- [ ] GimbalControllerNode enforces ontology limits
- [ ] Camera images published to `/f11/camera/image_raw`
- [ ] IMU data published to `/f11/imu`
- [ ] GPS data published to `/f11/gps`
- [ ] Gimbal commands accepted on `/f11/gimbal/command`
- [ ] Integration tests pass

### Verification

Run automated verification:
```bash
bash .phases/phase-06a-isr-sensor-integration/verification.sh
```

### Common Pitfalls

- **Gazebo topic names**: Must match exactly with model.sdf sensor names
- **Coordinate frames**: Ensure TF tree is correctly configured
- **Message types**: Use correct Gazebo-ROS type mappings
- **Gimbal limits**: Must match isr_extensions.kif values

### References

- [Gazebo-ROS Bridge](https://gazebosim.org/docs/harmonic/ros2_integration)
- [F-11 ISR Extensions](../../ontology/planning_mode/isr_extensions.kif)
- [F-11 Simulation Models](../../simulation/models/)

### Dependencies
See `dependencies.json` - requires Phase 6 completion.

### Next Phase
After completion, proceed to Phase 6b: Simulation & Training Environment
