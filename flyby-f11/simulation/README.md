# Flyby F-11 Simulation Environment

This directory contains the Gazebo simulation environment for training the hierarchical RL agents.

## Directory Structure

```
simulation/
├── worlds/               # Gazebo world files (.sdf)
│   └── training_world.sdf
├── models/               # Custom Gazebo models
│   ├── flyby_f11/       # UAV model
│   └── obstacles/       # Training obstacles
├── configs/             # ArduPilot SITL configuration
│   ├── arducopter.parm  # Vehicle parameters
│   └── locations.txt    # Spawn locations
├── config/              # ROS integration configuration
│   └── ros_gz_bridge.yaml  # ROS-Gazebo topic bridge config
├── scripts/             # Simulation utilities
│   ├── reset_episode.py # Episode reset for RL training
│   ├── spawn_vehicle.sh # Vehicle spawning utility
│   └── start_ros_gz_bridge.sh  # ROS-Gazebo bridge launcher
├── launch/              # ROS 2 launch files
│   ├── training.launch.py
│   └── ros_gz_bridge.launch.py  # ROS-Gazebo bridge launch
├── Containerfile.sitl   # ArduPilot SITL + Gazebo container
└── README.md
```

## Quick Start

### Start Simulation Stack
```bash
cd flyby-f11
podman-compose --profile simulation up
```

### Enter Container for Development
```bash
podman exec -it flyby-f11-sim-dev bash
```

### Run Training Environment
```bash
# Inside container
source /opt/ros/humble/setup.bash
ros2 launch flyby_f11_simulation training.launch.py
```

## Training World Features

The training world (`training_world.sdf`) includes:

1. **Open Navigation Area**: 200m x 200m clear space for basic flight
2. **Obstacle Field**: Static obstacles for avoidance training
3. **No-Fly Zones**: Defined regions matching ontology NFZ format
4. **Geofence Boundary**: 250m x 250m operational limit
5. **Landing Zones**: Designated safe landing areas

### World Coordinates

- **Origin**: (0, 0, 0) - Center of training area
- **Geofence**: (-125, -125) to (125, 125) meters
- **NFZ 1**: Cylindrical zone at (50, 50) radius 20m
- **NFZ 2**: Rectangular zone from (80, -40) to (100, -20)
- **Obstacle Field**: Scattered obstacles in region (-50, -50) to (50, 50)

## Episode Management

### Reset Episode
```python
from flyby_f11_simulation import EpisodeManager

manager = EpisodeManager()
manager.reset(spawn_position=[0, 0, 10])  # x, y, z in meters
```

### Configurable Start Positions
```yaml
spawn_positions:
  - name: "center"
    position: [0, 0, 10]
  - name: "corner_ne"
    position: [100, 100, 15]
  - name: "near_nfz"
    position: [30, 40, 10]
```

## Integration with Ontology

The simulation environment exports state to the vampire_bridge:

1. **UAV Position**: Published to `/flyby/state/position`
2. **NFZ Status**: Published to `/flyby/environment/nfz_status`
3. **Geofence Status**: Published to `/flyby/environment/geofence_status`

These topics feed into the StateGroundingNode for ontology queries.

## ArduPilot SITL Configuration

Vehicle parameters optimized for RL training:
- Simplified dynamics for faster simulation
- Disabled GPS wait for instant arming
- Reduced sensor noise for consistent training
- Fast physics update rate (400Hz SITL, 1000Hz Gazebo)

## ROS GZ Bridge

The ROS GZ Bridge connects Gazebo simulation topics to ROS 2 for sensor data and control commands.

### Start the Bridge

```bash
# Option 1: Using the launch file
ros2 launch flyby_f11_simulation ros_gz_bridge.launch.py

# Option 2: Using the convenience script
./scripts/start_ros_gz_bridge.sh

# Option 3: Direct ros2 run command
ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:=/simulation/config/ros_gz_bridge.yaml
```

### Bridged Topics

| ROS 2 Topic | Direction | Message Type | Description |
|-------------|-----------|--------------|-------------|
| `/f11/camera/image_raw` | GZ -> ROS | `sensor_msgs/msg/Image` | 1920x1080 RGB camera |
| `/f11/camera/camera_info` | GZ -> ROS | `sensor_msgs/msg/CameraInfo` | Camera intrinsics |
| `/f11/imu` | GZ -> ROS | `sensor_msgs/msg/Imu` | IMU at 250 Hz |
| `/f11/gps` | GZ -> ROS | `sensor_msgs/msg/NavSatFix` | GPS at 10 Hz |
| `/f11/gimbal/pitch_cmd` | ROS -> GZ | `std_msgs/msg/Float64` | Gimbal pitch (rad) |
| `/f11/gimbal/yaw_cmd` | ROS -> GZ | `std_msgs/msg/Float64` | Gimbal yaw (rad) |
| `/f11/gimbal/roll_cmd` | ROS -> GZ | `std_msgs/msg/Float64` | Gimbal roll (rad) |

### Gimbal Control Example

```bash
# Pitch gimbal down by 0.5 radians
ros2 topic pub /f11/gimbal/pitch_cmd std_msgs/msg/Float64 "{data: 0.5}"

# Yaw gimbal right by 0.3 radians
ros2 topic pub /f11/gimbal/yaw_cmd std_msgs/msg/Float64 "{data: 0.3}"
```

### Verify Bridge Operation

```bash
# List all F-11 topics
ros2 topic list | grep f11

# Echo camera info
ros2 topic echo /f11/camera/camera_info --once

# Monitor IMU data
ros2 topic hz /f11/imu
```

## Native Gazebo Gimbal Control

The F-11 ISR camera model includes 3-axis gimbal joint position controllers for direct Gazebo control (bypassing ROS).

### Native Control Topics

Publish `gz.msgs.Double` messages to control gimbal angles (radians):

| Topic | Joint | Axis | Limits |
|-------|-------|------|--------|
| `/f11_isr/gimbal/yaw` | gimbal_yaw_joint | Z | +/- 6.02 rad (+/- 345 deg) |
| `/f11_isr/gimbal/pitch` | gimbal_pitch_joint | Y | +/- 2.09 rad (+/- 120 deg) |
| `/f11_isr/gimbal/roll` | camera_joint | X | +/- 0.785 rad (+/- 45 deg) |

### Command Line Control (Native Gazebo)

```bash
# Inside container - set yaw to 1 radian (~57 degrees)
gz topic -t /f11_isr/gimbal/yaw -m gz.msgs.Double -p 'data: 1.0'

# Set pitch to look down 0.5 radians (~29 degrees)
gz topic -t /f11_isr/gimbal/pitch -m gz.msgs.Double -p 'data: 0.5'

# Set roll to 0 (level)
gz topic -t /f11_isr/gimbal/roll -m gz.msgs.Double -p 'data: 0.0'
```

### Python Control Example (Native Gazebo)

```python
import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

node = Node()
yaw_pub = node.advertise("/f11_isr/gimbal/yaw", Double)
pitch_pub = node.advertise("/f11_isr/gimbal/pitch", Double)

# Command 30 degrees pitch down
msg = Double()
msg.data = 0.524  # radians
pitch_pub.publish(msg)
```

### PID Tuning

The gimbal controllers use PID gains tuned for fast response with minimal overshoot:

- **P gain**: 10.0
- **I gain**: 0.1
- **D gain**: 0.5
- **I limits**: +/- 1.0
- **Command limits**: +/- 10.0

Adjust in `models/f11_isr_camera/model.sdf` if response needs tuning.

### Gimbal Test Script

Verify gimbal operation with the included test script:

```bash
podman exec flyby-f11-isr python3 /simulation/test_gimbal_control.py
```

This captures images at various gimbal positions to `/simulation/gimbal_test_output/`.

## Performance Notes

- **Headless Mode**: Use `HEADLESS=true` for training (no GUI)
- **Parallel Instances**: Support for 8 parallel SITL instances
- **Real-time Factor**: Target 2-4x real-time for training throughput
