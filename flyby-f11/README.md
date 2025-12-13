# Flyby F-11 Mission

Production autonomous mission platform using Flyby Robotics F-11 developer quadcopter with obstacle avoidance and target recognition.

## Status

**IN DEVELOPMENT - NOT YET ACCESSIBLE**

This project is structured and ready but hardware access is not available for several months. Active development is happening in `../project-drone/` where shared autonomy components are being built and tested.

## Hardware Platform

- **Platform**: Flyby Robotics F-11 Developer Quadcopter
- **Compute**: NVIDIA Jetson Orin NX 16GB (50 TOPS)
- **Flight Controller**: ArduPilot firmware (not PX4)
- **Communication Protocol**: MQTT (primary control interface)
- **Sensors**: TBD (different suite than project-drone)

## Shared Components

The following packages will be shared from project-drone development:

- `autonomy_core/` - Core mission planning and waypoint navigation
- `behavior_trees/` - BehaviorTree.CPP-based mission logic
- `perception_pipeline/` - Vision models (potentially heavier models due to 16GB GPU)

## Platform-Specific Components

The F-11 uses ArduPilot and MQTT instead of PX4/MAVSDK:

- `ardupilot_interface/` - ArduPilot MAVLink bridge
- `mqtt_interface/` - MQTT communication layer for F-11 control
- `flyby_f11_bringup/` - F-11-specific launch configurations
- `flyby_f11_sensors/` - F-11 sensor suite drivers

## Directory Structure

```
flyby-f11/
├── ros2_ws/                   # ROS 2 workspace
│   └── src/
│       ├── flyby_f11_bringup/    # F-11 launch configurations
│       ├── flyby_f11_sensors/    # F-11-specific sensor drivers
│       ├── flyby_f11_mission/    # F-11-specific mission logic
│       ├── ardupilot_interface/  # ArduPilot MAVLink bridge
│       ├── mqtt_interface/       # MQTT communication layer
│       ├── autonomy_core/        # Shared: core autonomy (symlink from project-drone)
│       ├── behavior_trees/       # Shared: BT definitions (symlink from project-drone)
│       └── perception_pipeline/  # Shared: vision models (symlink from project-drone)
├── docker/
│   ├── Dockerfile             # Deployment container
│   └── docker-compose.yml
├── simulation/
│   ├── worlds/                # Gazebo worlds for F-11 scenarios
│   └── models/                # F-11 drone model
├── launch/
│   ├── simulation.launch.py   # SITL simulation
│   └── real_hardware.launch.py  # Real drone deployment
├── config/
│   ├── ardupilot_params.parm  # ArduPilot parameter overrides
│   ├── mqtt_config.yaml       # MQTT broker/topic configuration
│   └── mission_waypoints.yaml
└── deployment/
    └── preflight-checks.sh    # Pre-flight validation
```

## Development Workflow

### Phase 1 (Current): Development on project-drone
All shared autonomy components are being developed and tested in `../project-drone/`.

### Phase 2 (Future): Integration
When F-11 hardware becomes available:

1. Link shared packages:
   ```bash
   cd flyby-f11/ros2_ws/src/
   ln -s ../../../project-drone/ros2_ws/src/autonomy_core ./
   ln -s ../../../project-drone/ros2_ws/src/behavior_trees ./
   ln -s ../../../project-drone/ros2_ws/src/perception_pipeline ./
   ```

2. Develop ArduPilot interface package
3. Develop MQTT communication layer
4. Develop F-11-specific sensor integration
5. Configure mission parameters
6. Test in simulation (ArduPilot SITL)
7. Deploy to real hardware

## Build Instructions

**Not yet active** - See project-drone for current development.

When ready:
```bash
cd ros2_ws
colcon build --symlink-install
```

## Running

### Simulation
```bash
docker compose up
# OR
ros2 launch flyby_f11_bringup simulation.launch.py
```

### Real Hardware
```bash
# Run preflight checks
./deployment/preflight-checks.sh

# Launch mission
ros2 launch flyby_f11_bringup real_hardware.launch.py
```

## GPU Memory Considerations

With 16GB GPU memory (Jetson Orin NX), the F-11 can run:
- Larger perception models than project-drone (8GB Orin Nano Super)
- Multiple vision models simultaneously
- Higher resolution processing
- More complex LLM reasoning and vision-language models (VLMs)
- Concurrent AI workloads without memory constraints

## Compute Comparison: F-11 vs project-drone

| Feature | Flyby F-11 | project-drone |
|---------|------------|---------------|
| **Compute** | Jetson Orin NX | Jetson Orin Nano Super |
| **TOPS** | 50 | 67 |
| **Memory** | 16GB | 8GB |
| **CUDA Cores** | 1,024 | 1,024 |
| **Tensor Cores** | 32 | 32 |
| **Advantage** | 2x memory for larger models | 34% more compute throughput |

## ArduPilot & MQTT Integration

The F-11 uses **ArduPilot** instead of PX4:
- MAVLink protocol compatibility (similar to PX4)
- Different parameter structure (.parm files vs .params)
- ArduPilot SITL for simulation testing

**MQTT** is used for primary control interface:
- Publish/subscribe architecture for commands
- Topic-based communication (e.g., `flyby/command/goto`, `flyby/status/position`)
- Integration with F-11 onboard systems
- Broker configuration in [config/mqtt_config.yaml](config/mqtt_config.yaml)

## Notes

- F-11 uses **ArduPilot** (not PX4 like project-drone)
- Communication via **MQTT** and MAVLink
- Both platforms are quadcopters
- Sensor suite differs from project-drone (T265/D455)
- Hardware access expected in several months via MCTSSA
- 3kg payload capacity allows additional sensors/compute
- NDAA-compliant for government/defense applications
