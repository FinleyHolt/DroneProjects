# ArduPilot Flight Control Documentation - Download Instructions

This document provides instructions for manually downloading critical ArduPilot and flight control documentation for the flyby-f11 project.

## Directory Structure

Create the following directory structure:
```
/home/finley/Github/DroneProjects/flyby-f11/docs/flight_control/
├── ardupilot_copter/
├── mavsdk/
├── ardupilot_sitl/
├── mavlink/
├── ros2_integration/
├── repos/
└── manifest.txt
```

## 1. ArduPilot Copter Documentation

### Flight Modes
Download from: https://ardupilot.org/copter/docs/flight-modes.html
Save to: `ardupilot_copter/flight_modes.html`

Key flight modes for autonomous operation:
- **GUIDED**: Primary mode for companion computer control via MAVLink
- **LOITER**: GPS hold position
- **RTL**: Return to Launch
- **AUTO**: Execute mission from waypoints
- **STABILIZE**: Manual control with attitude stabilization

### GUIDED Mode (Critical)
Download from: https://ardupilot.org/copter/docs/ac2_guidedmode.html
Save to: `ardupilot_copter/guided_mode.html`

This mode is essential for MAVSDK offboard control. GUIDED mode allows:
- Position control via MAVLink SET_POSITION_TARGET_LOCAL_NED
- Velocity control via MAVLink SET_POSITION_TARGET_LOCAL_NED
- Attitude/thrust control
- Waypoint navigation

### Parameter Reference
Download from: https://ardupilot.org/copter/docs/parameters.html
Save to: `ardupilot_copter/parameters.html`

Full parameter list: https://ardupilot.org/copter/docs/parameters.html
Backup: Download PDF from https://ardupilot.org/copter/docs/parameters-ArduCopter.pdf

Critical parameters for autonomous flight:
- SYSID_THISMAV: MAVLink system ID
- SERIAL2_PROTOCOL: Companion computer serial protocol (MAVLink 2)
- EKF parameters for sensor fusion
- WPNAV parameters for waypoint navigation
- RTL parameters for return-to-launch behavior

### First Time Setup
Download from: https://ardupilot.org/copter/docs/initial-setup.html
Save to: `ardupilot_copter/initial_setup.html`

### Tuning Guide
Download from: https://ardupilot.org/copter/docs/tuning.html
Save to: `ardupilot_copter/tuning.html`

## 2. MAVSDK Documentation and Repository

### MAVSDK Repository
Clone: https://github.com/mavlink/MAVSDK
Command: `git clone --depth 1 https://github.com/mavlink/MAVSDK.git repos/MAVSDK`

MAVSDK provides C++ and Python APIs for drone control via MAVLink.

### Key MAVSDK Guides

#### Offboard Control Guide
Download from: https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html
Save to: `mavsdk/offboard_control_guide.html`

Works with ArduPilot GUIDED mode for:
- Position setpoints
- Velocity setpoints
- Attitude/thrust setpoints

#### C++ API Reference
Download from: https://mavsdk.mavlink.io/main/en/cpp/api_reference/
Save to: `mavsdk/cpp_api_reference.html`

Key classes:
- `mavsdk::Mavsdk`: Main MAVSDK instance
- `mavsdk::System`: Represents connected drone
- `mavsdk::Offboard`: Offboard control plugin
- `mavsdk::Telemetry`: Telemetry data (position, attitude, battery)
- `mavsdk::Action`: Arming, takeoff, land, RTL

#### Python API Reference
Download from: https://mavsdk.mavlink.io/main/en/python/
Save to: `mavsdk/python_api_reference.html`

## 3. ArduPilot SITL (Software In The Loop)

### SITL Overview
Download from: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
Save to: `ardupilot_sitl/sitl_overview.html`

SITL allows testing ArduPilot firmware without hardware.

### Setting Up SITL
Download from: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
Save to: `ardupilot_sitl/setup_linux.html`

Installation:
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### Running SITL
Download from: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
Save to: `ardupilot_sitl/using_sitl.html`

Basic command:
```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map
```

### Gazebo with ArduPilot
Download from: https://ardupilot.org/dev/docs/sitl-with-gazebo.html
Save to: `ardupilot_sitl/gazebo_integration.html`

ArduPilot SITL can connect to Gazebo using the ardupilot_gazebo plugin.

Repository: https://github.com/khancyr/ardupilot_gazebo
Clone: `git clone --depth 1 https://github.com/khancyr/ardupilot_gazebo.git repos/ardupilot_gazebo`

### MAVProxy
Download from: https://ardupilot.org/mavproxy/
Save to: `ardupilot_sitl/mavproxy.html`

MAVProxy is a ground control station for ArduPilot. Useful for:
- Monitoring SITL
- Sending MAVLink commands
- Parameter management
- Mission planning

## 4. MAVLink Protocol

### MAVLink Overview
Download from: https://mavlink.io/en/
Save to: `mavlink/overview.html`

### Message Definitions
Download from: https://mavlink.io/en/messages/common.html
Save to: `mavlink/common_messages.html`

Critical messages for offboard control:
- SET_POSITION_TARGET_LOCAL_NED: Position/velocity setpoints
- COMMAND_LONG: Generic commands (arm, disarm, mode change)
- MISSION_ITEM: Waypoint definition
- HEARTBEAT: System status

### ArduPilot MAVLink Usage
Download from: https://ardupilot.org/dev/docs/mavlink-basics.html
Save to: `mavlink/ardupilot_mavlink.html`

ArduPilot-specific MAVLink implementation details.

## 5. ROS 2 Integration

### drone_mavsdk (ROS 2 MAVSDK Wrapper)
Clone: https://github.com/slaghuis/drone_mavsdk
Command: `git clone --depth 1 https://github.com/slaghuis/drone_mavsdk.git repos/drone_mavsdk`

This provides ROS 2 nodes wrapping MAVSDK functionality:
- Offboard control via ROS 2 topics/services
- Telemetry publishing
- Action servers for arming, takeoff, land

### ArduPilot ROS 2 Integration
Search for: "ArduPilot ROS 2" documentation
Alternative: https://github.com/ArduPilot/ardupilot/wiki/ROS

### micro-ROS with ArduPilot
Download from: https://ardupilot.org/dev/docs/ros2.html
Save to: `ros2_integration/ardupilot_ros2.html`

ArduPilot now has experimental micro-ROS support for direct ROS 2 communication.

## Manual Download Commands

Run these commands from `/home/finley/Github/DroneProjects/flyby-f11/docs/flight_control/`:

```bash
# Create directory structure
mkdir -p ardupilot_copter mavsdk ardupilot_sitl mavlink ros2_integration repos

# ArduPilot Copter
cd ardupilot_copter
wget -O flight_modes.html "https://ardupilot.org/copter/docs/flight-modes.html"
wget -O guided_mode.html "https://ardupilot.org/copter/docs/ac2_guidedmode.html"
wget -O parameters.html "https://ardupilot.org/copter/docs/parameters.html"
wget -O initial_setup.html "https://ardupilot.org/copter/docs/initial-setup.html"
wget -O tuning.html "https://ardupilot.org/copter/docs/tuning.html"
wget -O index.html "https://ardupilot.org/copter/index.html"
cd ..

# MAVSDK
cd mavsdk
wget -O offboard_control_guide.html "https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html"
wget -O cpp_api_reference.html "https://mavsdk.mavlink.io/main/en/cpp/api_reference/"
wget -O python_api_reference.html "https://mavsdk.mavlink.io/main/en/python/"
wget -O getting_started.html "https://mavsdk.mavlink.io/main/en/cpp/guide/getting_started.html"
cd ..

# ArduPilot SITL
cd ardupilot_sitl
wget -O sitl_overview.html "https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html"
wget -O setup_linux.html "https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html"
wget -O using_sitl.html "https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html"
wget -O gazebo_integration.html "https://ardupilot.org/dev/docs/sitl-with-gazebo.html"
wget -O mavproxy.html "https://ardupilot.org/mavproxy/"
cd ..

# MAVLink
cd mavlink
wget -O overview.html "https://mavlink.io/en/"
wget -O common_messages.html "https://mavlink.io/en/messages/common.html"
wget -O ardupilot_mavlink.html "https://ardupilot.org/dev/docs/mavlink-basics.html"
cd ..

# ROS 2 Integration
cd ros2_integration
wget -O ardupilot_ros2.html "https://ardupilot.org/dev/docs/ros2.html"
cd ..

# Clone repositories
cd repos
git clone --depth 1 https://github.com/mavlink/MAVSDK.git
git clone --depth 1 https://github.com/slaghuis/drone_mavsdk.git
git clone --depth 1 https://github.com/khancyr/ardupilot_gazebo.git
cd ..
```

## Critical Concepts for flyby-f11

### 1. GUIDED Mode vs Offboard
- **ArduPilot GUIDED**: Equivalent to PX4 OFFBOARD mode
- Companion computer sends position/velocity setpoints via MAVLink
- MAVSDK's Offboard plugin works with GUIDED mode
- Must send setpoints at >2Hz to maintain control

### 2. Coordinate Frames
- **NED**: North-East-Down (ArduPilot native)
- **ENU**: East-North-Up (ROS 2 convention)
- Convert between frames when using ROS 2 with ArduPilot

### 3. Arming Checks
ArduPilot has extensive pre-arm checks:
- GPS lock (can disable for indoor with EKF3_SRCn_POSXY=0)
- Compass calibration
- Accelerometer calibration
- RC calibration (can bypass with ARMING_CHECK parameter)

### 4. EKF (Extended Kalman Filter)
ArduPilot's EKF3 fuses sensor data:
- GPS, barometer, compass, IMU
- Can use visual odometry (T265) via VISION_POSITION_ESTIMATE
- Configure EKF3_SRCn parameters for sensor fusion

## Recommended Reading Order

1. Start with SITL setup and testing
2. Understand GUIDED mode deeply
3. Review MAVSDK offboard control guide
4. Study MAVLink messages for control
5. Explore ROS 2 integration options
6. Review parameter reference for configuration

## Additional Resources

### ArduPilot Forums
https://discuss.ardupilot.org/

### ArduPilot Discord
https://ardupilot.org/discord

### MAVSDK Examples
https://github.com/mavlink/MAVSDK/tree/main/examples

### ArduPilot Developer Chat
https://gitter.im/ArduPilot/ardupilot

## Notes

- ArduPilot is very different from PX4 in configuration and behavior
- GUIDED mode is critical - it's the ArduPilot equivalent of PX4 OFFBOARD
- Parameter naming is different (ArduPilot uses PARAM_NAME, PX4 uses PARAM_NAME)
- SITL is easier to set up than PX4 SITL (single sim_vehicle.py command)
- MAVProxy is the standard GCS for ArduPilot (QGroundControl also works)
