# Flight Control Documentation for flyby-f11

This directory contains essential ArduPilot and flight control documentation for the flyby-f11 autonomous drone project.

## Quick Start

1. **New to ArduPilot?** Start with `ARDUPILOT_QUICK_REFERENCE.md`
2. **Setting up ROS 2?** Read `MAVSDK_ROS2_INTEGRATION.md`
3. **Need to download docs?** Follow `DOWNLOAD_INSTRUCTIONS.md`
4. **Want an overview?** Check `manifest.txt`

## Critical Information

**IMPORTANT**: This project uses **ArduPilot**, NOT PX4.

Key difference: ArduPilot uses **GUIDED mode** for offboard control (equivalent to PX4's OFFBOARD mode).

## Documentation Files

### ARDUPILOT_QUICK_REFERENCE.md
Comprehensive quick reference covering:
- ArduPilot vs PX4 comparison
- Critical flight modes (GUIDED, AUTO, LOITER, RTL)
- MAVSDK integration with code examples
- Parameter configuration
- Coordinate frames (NED ↔ ENU)
- SITL setup and usage
- Gazebo integration
- Troubleshooting guide

### MAVSDK_ROS2_INTEGRATION.md
Complete ROS 2 integration guide:
- Three integration architectures
- drone_mavsdk wrapper documentation
- Custom MAVSDK bridge examples
- Launch file configurations
- Waypoint navigator example
- Frame transformations
- Testing workflows
- Jetson Orin NX optimization

### DOWNLOAD_INSTRUCTIONS.md
Manual download guide with:
- Complete URL list for ArduPilot documentation
- wget commands for HTML documentation
- git clone commands for repositories
- Organized directory structure
- Priority downloads highlighted

### manifest.txt
Detailed manifest including:
- Overview of all documentation
- File descriptions
- Key topics covered
- Recommended download list
- ArduPilot vs PX4 distinctions
- Next steps for flyby-f11 project
- Resources and community links

## Essential Concepts

### GUIDED Mode (Critical)
ArduPilot's companion computer control mode:
- Accepts position/velocity setpoints via MAVLink
- Must send setpoints at >2Hz to maintain control
- MAVSDK Offboard plugin uses GUIDED mode automatically
- Equivalent to PX4's OFFBOARD mode

### Coordinate Frames
- **ArduPilot**: Uses NED (North-East-Down)
- **ROS 2**: Typically uses ENU (East-North-Up)
- **Important**: Altitude in NED is negative (5m up = -5m down)
- Frame conversion required when using ROS 2

### MAVSDK Integration
MAVSDK provides C++/Python API for drone control:
- Works seamlessly with ArduPilot GUIDED mode
- Abstracts MAVLink protocol complexity
- Provides telemetry, action, and offboard plugins
- Can be wrapped in ROS 2 nodes for autonomy stack

## Recommended Learning Path

1. **Understand ArduPilot fundamentals**
   - Read ARDUPILOT_QUICK_REFERENCE.md
   - Focus on GUIDED mode section
   - Review critical parameters

2. **Set up SITL simulation**
   - Install ArduPilot
   - Run sim_vehicle.py
   - Test basic flight with MAVProxy

3. **Test MAVSDK connection**
   - Install MAVSDK library
   - Connect to SITL
   - Try offboard position control

4. **Integrate with ROS 2**
   - Read MAVSDK_ROS2_INTEGRATION.md
   - Clone drone_mavsdk wrapper
   - Test telemetry and control topics

5. **Develop autonomy behaviors**
   - Create mission planning nodes
   - Integrate with behavior trees
   - Add perception pipeline

6. **Deploy to hardware**
   - Configure Jetson Orin NX
   - Set up serial connection
   - Flight testing

## Key Resources

### Official Documentation
- ArduPilot Copter: https://ardupilot.org/copter/
- MAVSDK: https://mavsdk.mavlink.io/
- MAVLink: https://mavlink.io/

### Repositories
- ArduPilot: https://github.com/ArduPilot/ardupilot
- MAVSDK: https://github.com/mavlink/MAVSDK
- drone_mavsdk (ROS 2): https://github.com/slaghuis/drone_mavsdk
- ardupilot_gazebo: https://github.com/khancyr/ardupilot_gazebo

### Community
- ArduPilot Forums: https://discuss.ardupilot.org/
- ArduPilot Discord: https://ardupilot.org/discord
- MAVSDK GitHub: https://github.com/mavlink/MAVSDK/issues

## flyby-f11 Platform

**Hardware**: Flyby Robotics F-11 developer quadcopter
**Compute**: NVIDIA Jetson Orin NX 16GB (50 TOPS)
**Autopilot**: ArduPilot Copter
**Middleware**: ROS 2 Humble
**Control Interface**: MAVSDK

The flyby-f11 uses ArduPilot firmware with MAVSDK for offboard control. The autonomy stack runs on ROS 2, using the drone_mavsdk wrapper or custom MAVSDK bridge nodes.

## Manual Downloads

Due to permission restrictions during automated documentation download, you'll need to manually download some resources. See `DOWNLOAD_INSTRUCTIONS.md` for complete instructions.

### Priority Downloads

1. **ArduPilot SITL** - Essential for simulation
   ```bash
   git clone https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   git submodule update --init --recursive
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   ```

2. **MAVSDK Library** - Flight control interface
   ```bash
   # Option 1: Install from apt
   sudo apt install libmavsdk-dev

   # Option 2: Build from source
   git clone https://github.com/mavlink/MAVSDK.git
   cd MAVSDK
   cmake -Bbuild -H.
   cmake --build build -j4
   sudo cmake --build build --target install
   ```

3. **drone_mavsdk** - ROS 2 wrapper
   ```bash
   cd /home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src
   git clone https://github.com/slaghuis/drone_mavsdk.git
   ```

4. **ardupilot_gazebo** - Simulation plugin
   ```bash
   git clone https://github.com/khancyr/ardupilot_gazebo.git
   cd ardupilot_gazebo
   mkdir build && cd build
   cmake ..
   make -j4
   sudo make install
   ```

## Common Pitfalls

### Altitude Sign Error
Remember NED frame: **down is positive**!
- 5m altitude = -5.0 down_m
- Common mistake: using +5.0 (would command 5m underground!)

### Setpoint Rate Too Low
GUIDED mode requires >2Hz setpoints:
- Send at 10Hz or faster
- Lower rates will cause timeout and mode switch

### Wrong Mode Name
- ArduPilot: **GUIDED** mode (not OFFBOARD!)
- MAVSDK Offboard plugin handles this automatically

### Frame Confusion
- ArduPilot uses NED (North-East-Down)
- ROS 2 uses ENU (East-North-Up)
- Must transform between frames

### Missing Pre-Arm Checks
ArduPilot has extensive pre-arm checks:
- GPS lock required (can disable for indoor)
- Compass calibration
- Accelerometer calibration
- RC calibration
- Use `ARMING_CHECK` parameter to configure

## Safety Notes

- Always have RC transmitter ready for manual override
- Test thoroughly in SITL before hardware
- Follow pre-flight checklists (see ARDUPILOT_QUICK_REFERENCE.md)
- Monitor battery voltage
- Have emergency landing procedures ready
- Start with conservative parameters and tune gradually

## Next Steps

1. Read `ARDUPILOT_QUICK_REFERENCE.md` for foundational knowledge
2. Set up ArduPilot SITL simulation environment
3. Test basic MAVSDK connection and offboard control
4. Read `MAVSDK_ROS2_INTEGRATION.md` for ROS 2 integration
5. Clone and test drone_mavsdk wrapper
6. Integrate with flyby_f11_mission autonomy stack
7. Deploy to flyby-f11 hardware

## Contributing

When adding new documentation:
- Follow existing structure and format
- Use Markdown for all documentation
- Include code examples where applicable
- Update manifest.txt with new files
- Cross-reference related documentation

## Questions?

For project-specific questions:
- Check existing documentation files
- Review ArduPilot forums and Discord
- Consult MAVSDK documentation and examples

For ArduPilot support:
- Forums: https://discuss.ardupilot.org/
- Discord: https://ardupilot.org/discord

For MAVSDK support:
- Documentation: https://mavsdk.mavlink.io/
- GitHub Issues: https://github.com/mavlink/MAVSDK/issues
