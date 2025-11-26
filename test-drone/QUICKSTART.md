# Home Test Drone - Quick Start Guide

Fast track from setup to autonomous flight.

## Prerequisites

- Home test drone assembled with Pixhawk 6C, Jetson Orin Nano, T265, D455
- Docker installed on Jetson
- Cameras connected via USB 3.0
- Pixhawk connected to Jetson (USB or serial)

## 1. Build the Container (One Time)

```bash
cd /path/to/LLMDrone/home-test-drone/docker
sudo docker-compose build
```

Build time: ~30-45 minutes (compiles librealsense, MAVSDK from source)

## 2. First Launch - Interactive Setup

```bash
# Start container interactively
sudo docker-compose run --rm drone bash

# Inside container: verify cameras detected
rs-enumerate-devices

# Expected output:
# Device 0: Intel RealSense T265
# Device 1: Intel RealSense D455
```

## 3. Build ROS 2 Workspace

```bash
# Inside container
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Note**: `--symlink-install` means you can edit code on the host, and changes are immediately available in the container without rebuild.

## 4. Load PX4 Parameters

### Option A: Via QGroundControl (Recommended First Time)
1. Connect laptop to Pixhawk via USB
2. Open QGroundControl
3. Go to: **Vehicle Setup** → **Parameters** → **Tools** → **Load from file**
4. Select: `/path/to/home-test-drone/config/px4/params.params`
5. Reboot Pixhawk

### Option B: Verify Current Parameters
```bash
# Check critical vision parameters are set
# EKF2_AID_MASK should be 24 (vision enabled)
# EKF2_HGT_MODE should be 3 (vision height)
```

## 5. Mission Progression

### Phase 1: GPS Waypoint Mission (Current)

**Goal**: Validate MAVSDK offboard control outdoors

```bash
# Launch GPS waypoint mission
ros2 launch hometestdrone_bringup gps_mission.launch.py \
  mission_file:=/workspace/missions/gps_waypoints/simple_square.plan
```

**Test Checklist**:
- [ ] Drone arms via offboard command
- [ ] Takeoff to 10m altitude
- [ ] Navigate square pattern
- [ ] Return to launch
- [ ] Land autonomously

### Phase 2: Indoor VIO Flight

**Goal**: Replace GPS with T265 visual-inertial odometry

```bash
# Launch VIO flight test
ros2 launch hometestdrone_bringup indoor_vio.launch.py
```

**Test Checklist**:
- [ ] T265 publishes odometry at >30Hz
- [ ] PX4 EKF2 accepts vision position
- [ ] Position hold works indoors (no GPS)
- [ ] Manual waypoint navigation via offboard
- [ ] Drift < 0.5m over 60 seconds

### Phase 3: Autonomous SLAM Flight

**Goal**: Autonomous exploration with D455 SLAM mapping

```bash
# Launch full autonomy stack
ros2 launch hometestdrone_bringup indoor_slam.launch.py \
  mission_file:=/workspace/missions/indoor_slam/single_room_explore.yaml
```

**Test Checklist**:
- [ ] RTAB-Map builds 3D map
- [ ] Behavior tree executes exploration
- [ ] Obstacle avoidance works
- [ ] Loop closure improves map
- [ ] Return-to-home completes successfully

## 6. Development Workflow

### Edit Code on Host
```bash
# On Jetson host
cd /path/to/LLMDrone/home-test-drone/ros2_ws/src/hometestdrone_behaviors/src
nano my_behavior_node.cpp
```

### Changes Immediately Available in Container
```bash
# In running container
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select hometestdrone_behaviors
source install/setup.bash
ros2 run hometestdrone_behaviors my_behavior_node
```

### No need to rebuild container or copy files!

## 7. Monitoring and Debugging

### Check Sensor Streams
```bash
# Inside container
ros2 topic list | grep -E "(t265|d455)"
ros2 topic hz /t265/odom/sample
ros2 topic hz /d455/color/image_raw
```

### Visualize in RViz2
```bash
# Inside container (requires X11 forwarding)
rviz2 -d /workspace/config/rviz/slam_view.rviz
```

### Check MAVLink Connection
```bash
ros2 topic echo /fmu/out/vehicle_status --once
```

## 8. Safety Reminders

### Before Every Flight
- [ ] Propellers installed correctly (check rotation direction)
- [ ] Battery fully charged and secured
- [ ] Emergency stop controller paired
- [ ] Flight area clear of people/obstacles
- [ ] Geofence configured appropriately

### First Flights
- Start with **tethered/handheld** tests
- Have a spotter ready to catch drone
- Test emergency stop immediately
- Gradually increase autonomy level

### Indoor Flight
- Minimum 3m x 3m x 2m clear space
- Soft barriers around perimeter
- Good lighting for cameras
- No reflective surfaces (mirrors, windows)

## Troubleshooting

### Cameras Not Detected
```bash
# On host, check USB devices
lsusb | grep Intel

# Verify udev rules (may need to add user to plugdev group)
sudo usermod -aG plugdev $USER
# Log out and back in
```

### PX4 Won't Arm
- Check battery voltage (must be > 14.8V typically)
- Verify pre-arm checks: `ros2 topic echo /fmu/out/vehicle_status`
- Ensure offboard mode enabled in parameters
- Check RC transmitter connected (if COM_RC_IN_MODE requires it)

### Vision Position Not Fusing
```bash
# Check T265 publishing
ros2 topic hz /t265/odom/sample  # Should be >30Hz

# Check PX4 receiving vision data
# Look for VISION_POSITION_ESTIMATE in MAVLink inspector

# Verify EKF2 parameters
# EKF2_AID_MASK = 24
# EKF2_HGT_MODE = 3
```

### SLAM Not Building Map
- Ensure D455 depth aligned to color
- Check lighting (avoid dark rooms or direct sunlight)
- Verify RTAB-Map config: `ros2 param dump /rtabmap`
- Slow down movement for better feature tracking

## Next Steps

1. ✓ Container built and sensors verified
2. ✓ GPS waypoint mission validated outdoors
3. → Indoor VIO position hold test
4. → Single room SLAM exploration
5. → Multi-room autonomous navigation
6. → Advanced behaviors (object tracking, etc.)

## Resources

- **Full Documentation**: [home-test-drone/README.md](README.md)
- **PX4 Config Details**: [config/px4/README.md](config/px4/README.md)
- **Behavior Trees**: [ros2_ws/src/hometestdrone_behaviors/behavior_trees/](ros2_ws/src/hometestdrone_behaviors/behavior_trees/)
- **ROS 2 Packages**: [ros2_ws/src/](ros2_ws/src/)

**Questions?** Check main project CLAUDE.md or reach out to MCTSSA Digital Solutions team.
