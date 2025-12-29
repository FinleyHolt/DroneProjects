# Isaac Sim Evaluation Phase

**Purpose**: Evaluate NVIDIA Isaac Sim + Pegasus Simulator as an alternative to Gazebo for computer vision-focused drone simulation.

**Status**: In Progress - Core integration working

## Motivation

Gazebo's rendering (basic OpenGL) is insufficient for training/testing perception models that need photorealistic visuals. Isaac Sim provides:
- Path-traced rendering via Omniverse
- Native synthetic data generation (SDG)
- Domain randomization for robust CV training
- Headless container support for CI/CD pipelines

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Development Machine                       │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              Podman Container                        │    │
│  │  ┌─────────────┐    ┌──────────────────────────┐    │    │
│  │  │ ArduPilot   │◄──►│ Pegasus Simulator        │    │    │
│  │  │ SITL        │    │ (Isaac Sim Extension)    │    │    │
│  │  │             │    │                          │    │    │
│  │  │ MAVLink     │    │ - Physics (PhysX 5)      │    │    │
│  │  │ UDP:5760    │    │ - Rendering (RTX)        │    │    │
│  │  └─────────────┘    │ - Sensors (Camera, IMU)  │    │    │
│  │                     │ - ROS 2 Bridge           │    │    │
│  │  ┌─────────────┐    └──────────────────────────┘    │    │
│  │  │ ROS 2       │◄──► Camera topics, TF, etc.        │    │
│  │  │ Humble      │                                    │    │
│  │  └─────────────┘                                    │    │
│  └─────────────────────────────────────────────────────┘    │
│                           │                                  │
│                     GPU Passthrough                          │
│                    (RTX 5090, 24GB)                         │
└─────────────────────────────────────────────────────────────┘
```

## System Requirements

### Development Machine (Verified)
| Component | Available | Required (Min) | Required (Recommended) |
|-----------|-----------|----------------|------------------------|
| GPU | RTX 5090 24GB | RTX 3070 8GB | RTX 4080 16GB |
| RAM | TBD | 32GB | 64GB |
| Storage | TBD | 50GB SSD | 500GB SSD |
| Driver | 590.48.01 | 535.129.03 | 580.65.06+ |

### Target Hardware (Jetson Orin NX)
Isaac Sim does NOT run on Jetson - it's for development/training only. Trained models deploy to Jetson.

## Dependencies

### Isaac Sim Container
- **Image**: `nvcr.io/nvidia/isaac-sim:4.2.0` (matches Pegasus ArduPilot support)
- **Size**: ~15GB compressed, ~30GB extracted

### Pegasus Simulator
- **Repository**: https://github.com/PegasusSimulator/PegasusSimulator
- **Version**: v4.2.0 (last tested with ArduPilot)
- **ArduPilot Support**: Experimental (via PyArduPilotPlugin)

### ArduPilot SITL
- **Version**: ArduCopter 4.4.0
- **Build Target**: MatekH743 SITL

## MVE Success Criteria

The Minimum Viable Example must demonstrate:

1. **Container runs headless** - Isaac Sim starts without X11/display
2. **Outdoor world loads** - Basic terrain with sky, lighting
3. **F-11 drone model** - Quadcopter with camera sensor
4. **ArduPilot SITL connects** - MAVLink communication established
5. **Camera feed works** - RGB image published to ROS 2 topic
6. **Basic flight** - Arm, takeoff, hover, land via MAVProxy

## Evaluation Phases

### Phase 1: Container Setup ✅
- [x] Pull Isaac Sim container from NGC (4.2.0, ~30GB)
- [x] Verify GPU passthrough works (CDI via nvidia-container-toolkit)
- [x] Test GUI and headless startup
- [x] Document container configuration (podman-compose.yml)

### Phase 2: Pegasus Integration ✅
- [x] Install Pegasus Simulator extension (v4.2.0)
- [x] Configure ArduPilot SITL backend
- [x] Verify MAVLink backend available
- [x] Test basic drone spawn with Iris model

### Phase 3: Outdoor World
- [ ] Create/import outdoor environment
- [ ] Add realistic lighting (sun, sky)
- [ ] Configure ground terrain
- [ ] Add environmental objects

### Phase 4: F-11 Integration
- [ ] Create F-11 drone model (URDF/USD)
- [ ] Configure camera sensor
- [ ] Set vehicle parameters
- [ ] Verify sensor data flow

### Phase 5: End-to-End Test
- [ ] ArduPilot SITL + Isaac Sim + ROS 2
- [ ] Complete flight mission
- [ ] Record camera feed
- [ ] Document performance metrics

## File Structure

```
evaluation/isaac-sim/
├── README.md                    # This file
├── Containerfile.isaac          # Isaac Sim + Pegasus container
├── podman-compose.yml           # Development orchestration
├── config/
│   ├── pegasus_config.yaml      # Pegasus settings
│   └── ardupilot_params.param   # SITL parameters
├── worlds/
│   └── outdoor_training.usd     # Outdoor environment
├── models/
│   └── f11_isr/                 # F-11 drone model
│       ├── f11_isr.usd
│       └── f11_isr.urdf
├── scripts/
│   ├── start_headless.sh        # Launch Isaac Sim headless
│   ├── spawn_drone.py           # Spawn F-11 via Pegasus API
│   └── test_flight.py           # Basic flight test
└── docs/
    └── setup_notes.md           # Troubleshooting, lessons learned
```

## References

- [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/)
- [Pegasus ArduPilot Integration](https://pegasussimulator.github.io/PegasusSimulator/source/features/ardupilot.html)
- [Isaac Sim Container Installation](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/install_container.html)
- [Isaac Sim NGC Container](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)
- [Pegasus Simulator Paper (ICUAS 2024)](https://arxiv.org/html/2307.05263v2)

## Notes

- ArduPilot support in Pegasus is **experimental** - expect some manual debugging
- Isaac Sim 4.2.0 chosen for ArduPilot compatibility (newer versions untested)
- Container-first approach aligns with project's Podman workflow
- This evaluation is independent of main Gazebo simulation work
