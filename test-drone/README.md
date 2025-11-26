# Home Test Drone Platform

Indoor autonomous flight testing platform for developing and validating autonomy behaviors before deployment to mission-specific drones.

## Hardware Configuration

### Flight Controller
- **Model**: Pixhawk 6C
- **Firmware**: PX4 (latest stable)
- **Connection**: USB/UART to companion computer

### Companion Computer
- **Model**: NVIDIA Jetson Orin Nano Super
- **RAM**: 8GB
- **Storage**: NVMe SSD recommended
- **GPU**: Integrated NVIDIA GPU for vision processing

### Sensors

#### T265 Tracking Camera
- **Purpose**: Visual-inertial odometry (VIO) for GPS-denied navigation
- **Features**:
  - Dual fisheye cameras
  - Built-in IMU
  - Onboard SLAM processing
  - 6-DOF pose estimation at 200Hz
- **Status**: Deprecated/EOL hardware (requires librealsense v2.50.0)

#### D455 Depth Camera
- **Purpose**: Obstacle detection and 3D mapping
- **Features**:
  - Stereo depth camera
  - RGB camera
  - IMU
  - Up to 90 FPS depth stream
- **Range**: 0.4m - 6m (optimal indoor range)

## Mission Objectives

This platform follows a progressive autonomy development path:

### Phase 1: GPS Waypoint Navigation ✓ (Current)
- Offboard control via MAVSDK
- Outdoor GPS waypoint following
- Basic mission execution
- Telemetry logging and analysis

### Phase 2: Indoor VIO Flight (In Progress)
- T265-based position estimation
- Replace GPS with visual-inertial odometry
- Controlled indoor hovering and position hold
- Manual waypoint navigation in GPS-denied environment

### Phase 3: Autonomous Indoor SLAM Flight (Next)
- Real-time 3D mapping with D455
- RTAB-Map SLAM integration
- Behavior tree mission execution
- Autonomous exploration and navigation
- Obstacle detection and avoidance

### Phase 4: Advanced Behaviors
- Dynamic replanning
- Object detection and tracking
- Multi-room autonomous exploration
- Return-to-home in GPS-denied environments

## Software Stack

### Containerized Environment
- **Base**: Ubuntu 20.04 (ROS 2 Humble compatible)
- **ROS 2**: Humble Hawksbill
- **Flight Control**: MAVSDK (C++ library for MAVLink communication)
- **SLAM**: RTAB-Map (RGB-D and visual SLAM)
- **Behavior Trees**: BehaviorTree.CPP
- **Sensors**: librealsense2 (v2.50.0 for T265, current for D455)

### ROS 2 Package Organization

Platform-specific packages in `home-test-drone/ros2_ws/src/`:

- **`hometestdrone_control/`**: Offboard flight controllers, trajectory planning
- **`hometestdrone_behaviors/`**: Behavior tree definitions and executors
- **`hometestdrone_slam/`**: RTAB-Map configuration and launch files
- **`hometestdrone_perception/`**: Vision processing, obstacle detection
- **`hometestdrone_bringup/`**: System launch files and orchestration

Shared packages from main repo `ros2_ws/`:
- `agents_interface/`: Common message/service definitions
- `agents_mavsdk_bridge/`: MAVSDK integration

## Directory Structure

```
home-test-drone/
├── README.md                    # This file
├── docker/
│   ├── Dockerfile              # Full autonomy stack container
│   └── docker-compose.yml      # Container orchestration
├── config/
│   ├── px4/
│   │   └── params.params       # Pixhawk 6C parameter file
│   └── sensors/
│       ├── t265_calibration.yaml
│       └── d455_calibration.yaml
├── missions/
│   ├── gps_waypoints/
│   │   ├── simple_square.plan
│   │   └── hover_test.plan
│   └── indoor_slam/
│       ├── single_room_explore.yaml
│       └── multi_room_map.yaml
└── ros2_ws/
    └── src/
        ├── hometestdrone_control/
        ├── hometestdrone_behaviors/
        ├── hometestdrone_slam/
        ├── hometestdrone_perception/
        └── hometestdrone_bringup/
```

## Quick Start

### Build Container
```bash
cd home-test-drone/docker
sudo docker-compose build
```

### Run Autonomy Stack
```bash
# Launch full stack (sensors + ROS 2 + MAVSDK)
sudo docker-compose up

# Or run interactively
sudo docker-compose run --rm drone bash
```

### Build ROS 2 Packages
```bash
# Inside container
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch System
```bash
# GPS waypoint mission
ros2 launch hometestdrone_bringup gps_mission.launch.py

# Indoor VIO flight
ros2 launch hometestdrone_bringup indoor_vio.launch.py

# Indoor SLAM mission
ros2 launch hometestdrone_bringup indoor_slam.launch.py
```

## Development Workflow

1. **Develop**: Edit code on host machine in `home-test-drone/ros2_ws/src/`
2. **Mount**: Code mounted into container via docker-compose volumes
3. **Build**: `colcon build --symlink-install` for fast iteration
4. **Test**: Launch nodes and test behaviors indoors (tethered initially)
5. **Fly**: Untethered autonomous flight after behavior validation

## Safety Considerations

### Indoor Flight Precautions
- Clear minimum 3m x 3m x 2m flight space
- Soft barriers around perimeter
- Emergency stop controller paired and tested
- Propeller guards installed
- Start with tethered/handheld tests

### Behavior Tree Safety Nodes
- Battery voltage monitoring
- Position estimate quality checks
- Obstacle proximity thresholds
- Communication loss handling
- Geofence enforcement (virtual boundaries)

## PX4 Configuration

### Key Parameters for Indoor Flight

See `config/px4/params.params` for full configuration. Critical parameters:

```
# Use vision-based position estimation
EKF2_AID_MASK = 24  # Enable vision position + yaw
EKF2_HGT_MODE = 3   # Vision height mode

# Offboard control settings
COM_OBL_ACT = 1     # Return mode on offboard loss
COM_OBL_RC_ACT = 0  # Don't require RC for offboard
```

### Tuning Notes
- Lower rate limits for indoor flight (smooth, predictable motion)
- Conservative position controller gains
- Shorter failsafe timeouts (faster emergency response)

## RTAB-Map SLAM Configuration

RTAB-Map chosen for:
- Excellent RealSense camera integration (T265 + D455)
- Visual-inertial SLAM using T265 odometry
- 3D mapping with D455 depth data
- Loop closure detection for drift correction
- Proven performance on Jetson platforms

Configuration in `ros2_ws/src/hometestdrone_slam/config/rtabmap.yaml`

## Troubleshooting

### Cameras Not Detected
```bash
# Inside container, check USB devices
lsusb | grep Intel

# Check RealSense devices
rs-enumerate-devices

# Verify permissions (host machine)
sudo usermod -aG plugdev $USER
```

### ROS 2 Nodes Not Discovering
```bash
# Check ROS domain
echo $ROS_DOMAIN_ID

# Verify network mode in docker-compose.yml (should be 'host')
```

### MAVSDK Connection Issues
```bash
# Test connection to Pixhawk
ros2 topic echo /fmu/out/vehicle_status

# Check serial connection (adjust device)
ls -l /dev/ttyACM* /dev/ttyUSB*
```

### Jetson Performance Issues
```bash
# Enable max performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor resources
sudo tegrastats
```

## Next Steps

- [ ] Validate GPS waypoint missions outdoors
- [ ] Integrate T265 VIO with PX4 EKF2
- [ ] Test indoor position hold with VIO
- [ ] Configure RTAB-Map for D455 + T265
- [ ] Develop behavior tree for room exploration
- [ ] Implement obstacle avoidance behaviors
- [ ] Create mission validation test suite

## References

- [PX4 Vision Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html)
- [RTAB-Map ROS 2 Integration](http://wiki.ros.org/rtabmap_ros)
- [MAVSDK Documentation](https://mavsdk.mavlink.io/)
- [RealSense ROS 2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)
