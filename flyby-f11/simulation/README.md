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
├── scripts/             # Simulation utilities
│   ├── reset_episode.py # Episode reset for RL training
│   └── spawn_vehicle.sh # Vehicle spawning utility
├── launch/              # ROS 2 launch files
│   └── training.launch.py
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

## Performance Notes

- **Headless Mode**: Use `HEADLESS=true` for training (no GUI)
- **Parallel Instances**: Support for 8 parallel SITL instances
- **Real-time Factor**: Target 2-4x real-time for training throughput
