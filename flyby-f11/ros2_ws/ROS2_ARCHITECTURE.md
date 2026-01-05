# Flyby F-11 ROS 2 Architecture

This document describes the ROS 2 architecture for the Flyby F-11 UAV autonomy stack, designed for simulation-to-deployment parity between Isaac Sim and real hardware.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Isaac Sim / Real Hardware                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────┐     ┌─────────────────────────────────────────────┐   │
│  │  Camera Source   │     │              Flight Controller              │   │
│  │  (Isaac Sim or   │     │          (PX4 SITL or Real PX4)            │   │
│  │   RealSense)     │     └─────────────────────────────────────────────┘   │
│  └────────┬─────────┘                         ▲                             │
│           │                                   │ /cmd_vel                    │
│           ▼                                   │                             │
├───────────────────────────────────────────────┴─────────────────────────────┤
│                              ROS 2 Layer                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  /camera/image_raw                  /perception/detections                  │
│       │                                    │                                 │
│       ▼                                    ▼                                 │
│  ┌─────────────────┐              ┌─────────────────┐                       │
│  │ yolo_detector   │──────────────│ perception      │                       │
│  │    _node        │              │ _encoder_node   │                       │
│  │ (flyby_percept) │              │ (flyby_percept) │                       │
│  └─────────────────┘              └────────┬────────┘                       │
│                                            │                                 │
│                                            │ /perception/encoded            │
│                                            ▼                                 │
│  ┌─────────────────┐              ┌─────────────────┐                       │
│  │ state_estimator │              │ ontology        │                       │
│  │    _node        │──────────────│ _controller     │──────────────────────►│
│  │ (flyby_autonomy)│  /uav/state  │    _node        │  /autonomy/           │
│  └─────────────────┘              │ (flyby_autonomy)│  behavior_command     │
│                                   └────────┬────────┘                       │
│                                            │                                 │
│                                            │ /cmd_vel                       │
│                                            ▼                                 │
│                                   ┌─────────────────┐                       │
│                                   │ PX4 / MAVROS    │                       │
│                                   └─────────────────┘                       │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## ROS 2 Packages

### flyby_msgs
Custom message definitions for the autonomy stack.

**Messages:**
- `Detection.msg` - Single object detection with tracking info
- `DetectionArray.msg` - Array of detections from a frame
- `UAVState.msg` - Complete UAV state for decision-making
- `FlightPhase.msg` - Current flight phase in state machine
- `BehaviorCommand.msg` - Ontology-driven behavior command
- `PerceptionState.msg` - Encoded perception observation (516-dim)

### flyby_perception
YOLO-based object detection with ByteTrack tracking.

**Nodes:**
- `yolo_detector_node` - Runs YOLOv11 inference on camera images
- `perception_encoder_node` - Encodes detections into RL observation vector

**Topics Subscribed:**
- `/camera/image_raw` (sensor_msgs/Image)
- `/uav/state` (flyby_msgs/UAVState)

**Topics Published:**
- `/perception/detections` (flyby_msgs/DetectionArray)
- `/perception/encoded` (flyby_msgs/PerceptionState)
- `/perception/debug_image` (sensor_msgs/Image)

### flyby_autonomy
Ontology-driven behavior control and state estimation.

**Nodes:**
- `ontology_controller_node` - Monitors state, triggers behaviors from axioms
- `state_estimator_node` - Aggregates sensor data into unified UAV state

**Topics Subscribed:**
- `/uav/state` (flyby_msgs/UAVState)
- `/perception/detections` (flyby_msgs/DetectionArray)

**Topics Published:**
- `/autonomy/behavior_command` (flyby_msgs/BehaviorCommand)
- `/autonomy/flight_phase` (flyby_msgs/FlightPhase)
- `/cmd_vel` (geometry_msgs/Twist)
- `/uav/state` (flyby_msgs/UAVState) - from state_estimator

**Services:**
- `/autonomy/arm` (std_srvs/Trigger)
- `/autonomy/reset` (std_srvs/Trigger)

## Topic Summary

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/camera/image_raw` | sensor_msgs/Image | Isaac Sim / RealSense | yolo_detector_node |
| `/perception/detections` | flyby_msgs/DetectionArray | yolo_detector_node | ontology_controller_node |
| `/perception/encoded` | flyby_msgs/PerceptionState | perception_encoder_node | RL Agent |
| `/uav/state` | flyby_msgs/UAVState | state_estimator_node | ontology_controller_node |
| `/autonomy/behavior_command` | flyby_msgs/BehaviorCommand | ontology_controller_node | Action Bridge |
| `/cmd_vel` | geometry_msgs/Twist | ontology_controller_node | PX4 / MAVROS |

## Launch Files

### Isaac Sim Simulation
```bash
# Start all ROS 2 nodes for simulation
ros2 launch flyby_f11_bringup isaac_sim_bridge.launch.py

# Then run simulation script
/isaac-sim/python.sh scripts/phase9_ros2_test.py
```

### Real Hardware Deployment
```bash
# Start perception pipeline
ros2 launch flyby_perception perception.launch.py use_sim_time:=false

# Start autonomy nodes
ros2 launch flyby_autonomy autonomy.launch.py use_sim_time:=false use_simulation:=false

# Start MAVROS for real PX4
ros2 launch mavros px4.launch.py
```

## Simulation-to-Deployment Parity

The ROS 2 architecture ensures identical software runs in simulation and on real hardware:

### What Changes
| Component | Isaac Sim | Real Hardware |
|-----------|-----------|---------------|
| Camera Source | Isaac Sim Camera API | RealSense / USB Camera |
| Flight Controller | PX4 SITL | PX4 on Pixhawk |
| State Estimation | Simulation Ground Truth | MAVROS + VIO |

### What Stays the Same
- All ROS 2 nodes (perception, autonomy)
- Message types and topic names
- Ontology axiom logic
- YOLO model and tracking
- Behavior execution logic

## Data Flow

1. **Camera Image** (30 Hz)
   - Isaac Sim: `Camera.get_rgba()` → ROS2CameraBridge → `/camera/image_raw`
   - Real: RealSense node → `/camera/image_raw`

2. **YOLO Detection** (10 Hz)
   - `yolo_detector_node` subscribes to `/camera/image_raw`
   - Runs YOLOv11 + ByteTrack
   - Publishes to `/perception/detections`

3. **State Estimation** (60 Hz)
   - `state_estimator_node` aggregates sensor data
   - Publishes unified state to `/uav/state`

4. **Ontology Control** (60 Hz)
   - `ontology_controller_node` checks axioms
   - Publishes behavior commands to `/autonomy/behavior_command`
   - Publishes velocity to `/cmd_vel`

## Building

### Inside Container (Isaac Sim)
The ROS 2 packages are built automatically in the container:
```bash
# Packages are pre-built in /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

### On Jetson (Real Hardware)
```bash
cd ~/flyby-f11/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Configuration

### Perception Parameters
```yaml
yolo_detector_node:
  ros__parameters:
    model_path: '/workspace/models/yolo11x.pt'
    confidence_threshold: 0.25
    inference_rate: 10.0  # Hz
    enable_tracking: true
```

### Autonomy Parameters
```yaml
ontology_controller_node:
  ros__parameters:
    takeoff_altitude: 25.0
    battery_per_km: 2.5
    person_critical_distance: 10.0
```

## Testing

### Unit Tests
```bash
colcon test --packages-select flyby_msgs flyby_perception flyby_autonomy
colcon test-result --verbose
```

### Integration Test
```bash
# Start ROS 2 nodes
ros2 launch flyby_f11_bringup isaac_sim_bridge.launch.py &

# Run phase 9 test
/isaac-sim/python.sh scripts/phase9_ros2_test.py
```
