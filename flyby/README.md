# Flyby Mission

Production autonomous flyby mission with obstacle avoidance and target recognition.

## Status

**IN DEVELOPMENT - NOT YET ACCESSIBLE**

This project is structured and ready but hardware access is not available for several months. Active development is happening in `../test-drone/` where shared autonomy components are being built and tested.

## Hardware Platform

- **Platform**: Quadcopter with PX4 autopilot
- **Compute**: NVIDIA Jetson (16GB GPU memory)
- **Sensors**: TBD (different suite than test-drone)
- **Flight Controller**: PX4 firmware

## Shared Components

The following packages will be shared from test-drone development:

- `autonomy_core/` - Core mission planning and waypoint navigation
- `behavior_trees/` - BehaviorTree.CPP-based mission logic
- `px4_interface/` - MAVSDK/MAVLink communication layer
- `perception_pipeline/` - Vision models (potentially heavier models due to 16GB GPU)

## Directory Structure

```
flyby/
├── ros2_ws/                   # ROS 2 workspace
│   └── src/
│       ├── flyby_bringup/     # Launch configurations
│       ├── flyby_sensors/     # Hardware-specific sensor drivers
│       ├── flyby_mission/     # Flyby-specific mission logic
│       ├── autonomy_core/     # Shared: core autonomy (symlink from test-drone)
│       ├── behavior_trees/    # Shared: BT definitions (symlink from test-drone)
│       ├── perception_pipeline/  # Shared: vision models (symlink from test-drone)
│       └── px4_interface/     # Shared: MAVLink bridge (symlink from test-drone)
├── docker/
│   ├── Dockerfile             # Deployment container
│   └── docker-compose.yml
├── simulation/
│   ├── worlds/                # Gazebo worlds for flyby scenarios
│   └── models/                # Flyby drone model
├── launch/
│   ├── simulation.launch.py   # SITL simulation
│   └── real_hardware.launch.py  # Real drone deployment
├── config/
│   ├── px4_params.params      # PX4 parameter overrides
│   └── mission_waypoints.yaml
└── deployment/
    └── preflight-checks.sh    # Pre-flight validation
```

## Development Workflow

### Phase 1 (Current): Development on test-drone
All shared autonomy components are being developed and tested in `../test-drone/`.

### Phase 2 (Future): Integration
When flyby hardware becomes available:

1. Link shared packages:
   ```bash
   cd flyby/ros2_ws/src/
   ln -s ../../../test-drone/ros2_ws/src/autonomy_core ./
   ln -s ../../../test-drone/ros2_ws/src/behavior_trees ./
   ln -s ../../../test-drone/ros2_ws/src/px4_interface ./
   ln -s ../../../test-drone/ros2_ws/src/perception_pipeline ./
   ```

2. Develop flyby-specific sensor integration
3. Configure mission parameters
4. Test in simulation
5. Deploy to real hardware

## Build Instructions

**Not yet active** - See test-drone for current development.

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
ros2 launch flyby_bringup simulation.launch.py
```

### Real Hardware
```bash
# Run preflight checks
./deployment/preflight-checks.sh

# Launch mission
ros2 launch flyby_bringup real_hardware.launch.py
```

## GPU Memory Considerations

With 16GB GPU memory, flyby can run:
- Larger perception models than test-drone (8GB)
- Multiple vision models simultaneously
- Higher resolution processing
- More complex LLM reasoning (when integrated)

## Notes

- Flyby uses same PX4 firmware as test-drone
- Both platforms are quadcopters
- Sensor suite differs from test-drone (T265/D455)
- Hardware access expected in several months
