# ArduPilot Quick Reference for flyby-f11

This quick reference covers essential ArduPilot concepts for autonomous flight development on the flyby-f11 platform.

## ArduPilot vs PX4: Key Differences

| Aspect | ArduPilot | PX4 |
|--------|-----------|-----|
| Offboard Mode | GUIDED mode | OFFBOARD mode |
| Configuration | Parameters via MAVLink/files | Parameters + mixers + configs |
| SITL Command | `sim_vehicle.py` | `make px4_sitl gazebo` |
| GCS Tool | MAVProxy (default) | QGroundControl (default) |
| Parameter Naming | PARAM_NAME | PARAM_NAME (similar but different params) |
| Community | Forums + Discord | Slack + Discuss |

## Critical Flight Modes

### GUIDED Mode (Essential for Autonomy)
- **Purpose**: Companion computer control via MAVLink
- **Equivalent**: PX4 OFFBOARD mode
- **Control Methods**:
  - Position setpoints (NED frame)
  - Velocity setpoints (NED frame)
  - Attitude + thrust setpoints
- **Requirements**:
  - Send setpoints at >2Hz to maintain control
  - Vehicle must be armed
  - GPS lock (or EKF configured for indoor)

### AUTO Mode
- **Purpose**: Execute pre-programmed mission waypoints
- **Use Case**: Structured missions with defined waypoints
- **Upload**: Via MAVLink MISSION_ITEM messages

### LOITER Mode
- **Purpose**: GPS position hold
- **Use Case**: Pause autonomous mission, maintain position

### RTL (Return to Launch)
- **Purpose**: Autonomous return to home position
- **Behavior**: Climb to RTL_ALT, fly home, land or loiter
- **Trigger**: RC failsafe, low battery, or commanded

### STABILIZE Mode
- **Purpose**: Manual flight with attitude stabilization
- **Use Case**: Manual testing, emergency takeover

## MAVSDK with ArduPilot

### Setting Up MAVSDK for ArduPilot

```cpp
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Connect to ArduPilot SITL
mavsdk::Mavsdk mavsdk;
mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14550");

// Get system (drone)
auto system = mavsdk.first_autopilot(3.0);
if (!system) {
    std::cerr << "No autopilot found" << std::endl;
    return 1;
}

// Initialize plugins
auto action = mavsdk::Action{system.value()};
auto offboard = mavsdk::Offboard{system.value()};
auto telemetry = mavsdk::Telemetry{system.value()};
```

### Offboard Control Pattern

```cpp
// 1. Set initial setpoint BEFORE starting offboard
mavsdk::Offboard::PositionNedYaw initial_setpoint{};
initial_setpoint.north_m = 0.0f;
initial_setpoint.east_m = 0.0f;
initial_setpoint.down_m = -5.0f;  // 5m altitude (NED down is negative)
initial_setpoint.yaw_deg = 0.0f;
offboard.set_position_ned(initial_setpoint);

// 2. Start offboard mode (switches to GUIDED in ArduPilot)
mavsdk::Offboard::Result offboard_result = offboard.start();
if (offboard_result != mavsdk::Offboard::Result::Success) {
    std::cerr << "Offboard start failed" << std::endl;
    return 1;
}

// 3. Send setpoints in loop at >2Hz
while (mission_active) {
    mavsdk::Offboard::PositionNedYaw setpoint{};
    setpoint.north_m = target_north;
    setpoint.east_m = target_east;
    setpoint.down_m = target_down;
    setpoint.yaw_deg = target_yaw;
    offboard.set_position_ned(setpoint);

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
}

// 4. Stop offboard when done
offboard.stop();
```

### Velocity Control

```cpp
mavsdk::Offboard::VelocityNedYaw velocity_setpoint{};
velocity_setpoint.north_m_s = 1.0f;  // 1 m/s north
velocity_setpoint.east_m_s = 0.0f;
velocity_setpoint.down_m_s = 0.0f;
velocity_setpoint.yaw_deg = 0.0f;
offboard.set_velocity_ned(velocity_setpoint);
```

### Arming and Takeoff

```cpp
// Arm
auto arm_result = action.arm();
if (arm_result != mavsdk::Action::Result::Success) {
    std::cerr << "Arming failed: " << arm_result << std::endl;
}

// Takeoff (ArduPilot will use GUIDED_TAKEOFF)
auto takeoff_result = action.takeoff();
if (takeoff_result != mavsdk::Action::Result::Success) {
    std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
}

// Wait for altitude
while (telemetry.position().relative_altitude_m < 4.9f) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
```

## Critical Parameters for Autonomous Flight

### Companion Computer Communication

```
SERIAL2_PROTOCOL = 2        # MAVLink 2 on SERIAL2 (Telem2)
SERIAL2_BAUD = 921600       # High baud rate for companion computer
SYSID_THISMAV = 1           # MAVLink system ID
```

### EKF Configuration (GPS-based)

```
EKF3_ENABLE = 1             # Enable EKF3
AHRS_EKF_TYPE = 3           # Use EKF3
EKF3_SRC1_POSXY = 3         # GPS for position XY
EKF3_SRC1_POSZ = 1          # Barometer for altitude
EKF3_SRC1_VELXY = 3         # GPS for velocity XY
EKF3_SRC1_VELZ = 3          # GPS for velocity Z
```

### EKF Configuration (Visual Odometry - T265)

```
EKF3_ENABLE = 1             # Enable EKF3
AHRS_EKF_TYPE = 3           # Use EKF3
EKF3_SRC1_POSXY = 6         # External navigation (T265)
EKF3_SRC1_POSZ = 1          # Barometer for altitude
EKF3_SRC1_VELXY = 6         # External navigation velocity
EKF3_SRC1_VELZ = 0          # No velocity Z
EKF3_SRC1_YAW = 6           # External navigation yaw
VISO_TYPE = 1               # Enable visual odometry
```

Send T265 data via VISION_POSITION_ESTIMATE MAVLink message.

### Arming Checks

```
ARMING_CHECK = 1            # All checks (recommended for outdoor)
ARMING_CHECK = 16384        # Skip all except battery (indoor testing)
```

### RTL Behavior

```
RTL_ALT = 1500              # RTL altitude in cm (15m)
RTL_ALT_FINAL = 0           # Final altitude (0 = land)
RTL_LOIT_TIME = 5000        # Loiter time at home (ms)
```

### Failsafes

```
FS_THR_ENABLE = 1           # RC failsafe enabled
FS_THR_VALUE = 975          # PWM value indicating RC loss
FS_GCS_ENABLE = 1           # GCS failsafe enabled
FS_BATT_ENABLE = 1          # Battery failsafe enabled
FS_BATT_VOLTAGE = 14.4      # Battery failsafe voltage (4S LiPo)
```

## Coordinate Frames

### NED (North-East-Down) - ArduPilot Native
- **North**: Positive north
- **East**: Positive east
- **Down**: Positive down (altitude is negative!)

Example: 5m altitude = -5.0 down_m

### Conversion from ROS 2 ENU to ArduPilot NED

ROS 2 uses ENU (East-North-Up):
```cpp
// ENU to NED conversion
float ned_north = enu_north;
float ned_east = enu_east;
float ned_down = -enu_up;  // Note the sign flip
```

## MAVLink Messages for Control

### SET_POSITION_TARGET_LOCAL_NED
Control position and/or velocity in local NED frame.

```c
// Position control
mavlink_msg_set_position_target_local_ned_pack(
    system_id, component_id, &msg,
    time_boot_ms,
    target_system, target_component,
    MAV_FRAME_LOCAL_NED,
    0b0000111111111000,  // Position only
    north_m, east_m, down_m,
    0, 0, 0,  // velocity (ignored)
    0, 0, 0,  // acceleration (ignored)
    yaw_rad, yaw_rate_rad_s
);
```

### COMMAND_LONG
Send commands like arm, disarm, mode change.

```c
// Arm command
mavlink_msg_command_long_pack(
    system_id, component_id, &msg,
    target_system, target_component,
    MAV_CMD_COMPONENT_ARM_DISARM,  // Command
    0,  // Confirmation
    1,  // Param1: 1=arm, 0=disarm
    0, 0, 0, 0, 0, 0  // Other params unused
);
```

### VISION_POSITION_ESTIMATE
Send visual odometry data (e.g., from T265).

```c
mavlink_msg_vision_position_estimate_pack(
    system_id, component_id, &msg,
    usec,  // Timestamp
    x, y, z,  // Position in NED
    roll, pitch, yaw,  // Attitude
    covariance,  // 21-element covariance (can be zeros)
    reset_counter  // Increments on odometry reset
);
```

## SITL Setup

### Installation

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Running SITL

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map
```

Options:
- `-v ArduCopter`: Vehicle type
- `--console`: Open MAVProxy console
- `--map`: Open map window
- `-L <location>`: Set home location
- `--out=udp:127.0.0.1:14550`: Additional MAVLink output (for MAVSDK)

### Connect MAVSDK to SITL

SITL outputs MAVLink on UDP port 14550 by default.

```cpp
mavsdk.add_any_connection("udp://:14550");
```

Or specify explicit output:
```bash
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550
```

## Gazebo Integration

### ardupilot_gazebo Plugin

Repository: https://github.com/khancyr/ardupilot_gazebo

Installation:
```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

Add to `.bashrc`:
```bash
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH
```

### Run with Gazebo

Terminal 1 (Gazebo):
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```

Terminal 2 (SITL):
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

## Pre-Flight Checklist

### Simulation (SITL)
- [ ] SITL started successfully
- [ ] MAVSDK connected (check heartbeat)
- [ ] GPS lock obtained (or EKF configured for indoor)
- [ ] Sensors initialized (check telemetry)
- [ ] Ready to arm

### Real Hardware
- [ ] Battery charged and connected
- [ ] Propellers installed (correct direction!)
- [ ] RC transmitter on and bound
- [ ] Companion computer booted and connected
- [ ] GPS lock (outdoor) or visual odometry (indoor)
- [ ] Compass calibrated
- [ ] Accelerometer calibrated
- [ ] Pre-arm checks passed
- [ ] Safety pilot ready with RC override

## Common Issues

### "Waiting for home position"
- Ensure GPS has lock (outdoor)
- Or configure EKF for indoor with visual odometry
- Or use `param set EKF3_SRC1_POSXY 0` to disable GPS requirement

### "PreArm: RC not calibrated"
- Calibrate RC in Mission Planner / QGroundControl
- Or bypass with `param set ARMING_CHECK 16384` (testing only!)

### "Offboard mode not available"
- ArduPilot uses GUIDED, not OFFBOARD
- MAVSDK Offboard plugin automatically uses GUIDED on ArduPilot
- Ensure setpoint sent before starting offboard

### Altitude going wrong direction
- Remember NED frame: down is positive!
- 5m altitude = -5.0 down_m, not +5.0

### Setpoints not working
- Must send at >2Hz to maintain control
- Check MAVLink connection
- Verify GUIDED mode is active
- Check for parameter restrictions

## Resources

### Documentation
- ArduPilot Copter: https://ardupilot.org/copter/
- MAVSDK: https://mavsdk.mavlink.io/
- MAVLink: https://mavlink.io/

### Community
- ArduPilot Forums: https://discuss.ardupilot.org/
- ArduPilot Discord: https://ardupilot.org/discord
- MAVSDK GitHub: https://github.com/mavlink/MAVSDK

### Tools
- MAVProxy: Command-line GCS
- QGroundControl: Graphical GCS
- Mission Planner: Windows GCS with advanced features

## Next Steps

1. Set up SITL and verify basic flight
2. Test MAVSDK connection and telemetry
3. Implement simple offboard position control
4. Add velocity control for dynamic maneuvers
5. Integrate with ROS 2 for autonomy stack
6. Test with visual odometry (T265) for indoor flight
