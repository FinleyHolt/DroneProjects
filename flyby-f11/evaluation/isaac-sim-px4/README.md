# Flyby F-11 Autonomous ISR Evaluation Environment

Photorealistic drone simulation for autonomous ISR mission training and evaluation using NVIDIA Isaac Sim 5.1.0, PX4 Autopilot v1.14.3, and Pegasus Simulator v5.1.0.

## Overview

This environment provides a complete pipeline for training and validating autonomous ISR agents:

- **Isaac Sim 5.1.0**: Photorealistic simulation with PhysX 5, ray-traced rendering
- **Pegasus 5.1.0**: Drone-specific extensions for Isaac Sim
- **PX4 v1.14.3**: Production-grade autopilot (Pegasus recommended version)
- **Dual-Mode Perception**: Ground truth (fast training) + YOLO (E2E validation)
- **Ontology Safety Shielding**: Formal axiom-based action filtering
- **Reinforcement Learning**: SAC/PPO/TD3 via Stable Baselines3

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    AUTONOMOUS ISR TRAINING PIPELINE                       │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌─────────────┐     ┌──────────────────┐     ┌───────────────┐         │
│  │  Mission    │────►│ Ontology Behavior│────►│ Preempts RL   │         │
│  │  Tasking    │     │ Controller       │     │ when safety   │         │
│  │  (NAIs)     │     │ (Axiom-driven)   │     │ violated      │         │
│  └─────────────┘     └────────┬─────────┘     └───────┬───────┘         │
│                               │                        │                 │
│                               ▼                        │                 │
│  ┌─────────────┐     ┌────────────────┐               │                 │
│  │ Procedural  │◄───►│   RL Agent     │◄──────────────┘                 │
│  │ World Gen   │     │   (SAC Policy) │                                  │
│  │ (Forests,   │     └────────┬───────┘                                  │
│  │  Targets)   │              │                                          │
│  └──────┬──────┘              │                                          │
│         │                     ▼                                          │
│         │      ┌──────────────────────────────────┐                     │
│         │      │     DUAL-MODE PERCEPTION          │                     │
│         │      ├─────────────┬────────────────────┤                     │
│         │      │  GT Mode    │    Full Mode       │                     │
│         │      │ (Training)  │ (E2E Validation)   │                     │
│         │      │ Frustum     │ YOLO + ByteTrack   │                     │
│         │      │ 1000+ Hz    │ ~20 Hz             │                     │
│         │      └──────┬──────┴─────────┬──────────┘                     │
│         │             │                │                                 │
│         │             └────────┬───────┘                                 │
│         │                      ▼                                         │
│         │             ┌────────────────┐                                 │
│         │             │ Safety Filter  │                                 │
│         │             │ (Vampire ATP)  │                                 │
│         │             └────────┬───────┘                                 │
│         │                      │                                         │
│         │                      ▼                                         │
│         │             ┌────────────────┐                                 │
│         │             │ Action Bridge  │                                 │
│         │             │ (Z-up → NED)   │                                 │
│         │             └────────┬───────┘                                 │
│         │                      │                                         │
│         │                      ▼                                         │
│         │             ┌────────────────┐                                 │
│         └────────────►│ PX4 Offboard   │                                 │
│                       │ Control        │                                 │
│                       └────────────────┘                                 │
│                                                                          │
│         ISAAC SIM 5.1.0 + PEGASUS 5.1.0 + PX4 v1.14.3                   │
└──────────────────────────────────────────────────────────────────────────┘
```

## Development Phases Summary

| Phase | Feature | Status | Description |
|-------|---------|--------|-------------|
| 2 | Procedural World Gen | ✅ Complete | Terrain, forest, vehicles in proc-gen environments |
| 3 | Ground Truth Perception | ✅ Complete | Frustum-based detection without rendering |
| 4 | Gymnasium Wrapper | ✅ Complete | OpenAI Gym API for RL training |
| 5 | Reward System | ✅ Complete | Coverage, POI detection, safety penalties |
| 6 | SAC Training | ✅ Complete | Stable Baselines3 SAC agent training |
| 7 | Safety Filter | ✅ Complete | Ontology-based action validation |
| 8 | YOLO Integration | ✅ Complete | YOLO11 + ByteTrack perception pipeline |
| 9 | Comms-Denied E2E | 🚧 In Progress | Full autonomous mission validation |

## Canonical ISR Problems

### 1. Comms-Denied Area Surveillance
- **Mission**: Surveillance of 500m×500m area for 15 minutes without comms
- **Environment**: `environments/comms_denied_env.py`
- **Success Criteria**: Coverage ≥85%, POI ≥10, Safe RTL, Battery ≥15%

### 2. Dynamic NFZ Avoidance
- **Mission**: 3km point-to-point transit with NFZ activation at T+3min
- **Environment**: `environments/dynamic_nfz_env.py`
- **Status**: Defined, not tested

### 3. Multi-Objective ISR
- **Mission**: 8 targets with varying priorities, threat zone avoidance
- **Environment**: `environments/multi_objective_env.py`
- **Status**: Defined, not tested

## Prerequisites

- NVIDIA GPU (tested with RTX 5090)
- NVIDIA Driver 525+ (tested with 590.48.01)
- NVIDIA Container Toolkit with CDI configured
- Podman
- ~50GB disk space for images and caches

### GPU Passthrough Setup (one-time)

```bash
# Install nvidia-container-toolkit
sudo pacman -S nvidia-container-toolkit  # Arch
# or: sudo apt install nvidia-container-toolkit  # Ubuntu

# Generate CDI specification
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Verify GPU access
podman run --rm --device nvidia.com/gpu=all nvidia/cuda:12.0-base nvidia-smi
```

## Quick Start

### 1. Pull Isaac Sim Base Image

```bash
# Login to NVIDIA NGC (required for Isaac Sim images)
podman login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key from https://ngc.nvidia.com/setup/api-key>

# Pull Isaac Sim 5.1.0 (this takes a while, ~25GB)
podman pull nvcr.io/nvidia/isaac-sim:5.1.0
```

### 2. Build the Container

```bash
cd flyby-f11/evaluation/isaac-sim-px4

# Build with canonical image tag
podman build -t localhost/isaac-sim-px4:5.1.0-px4-1.14.3 .
```

Build takes ~30 minutes (PX4 compilation + shader warmup).

### 3. Run Phase Tests

```bash
# Allow X11 access (for GUI mode)
xhost +local:

# Run Phase 2: Procedural world generation test
./scripts/run_phase_test.sh 2

# Run Phase 8: YOLO perception test
./scripts/run_phase_test.sh 8

# Run headless
./scripts/run_phase_test.sh 6 --headless
```

### 4. Train SAC Agent

```bash
./run_training.sh
```

## Container Configuration

**Canonical Image Tag**: `localhost/isaac-sim-px4:5.1.0-px4-1.14.3`

All scripts in this project use this single image tag. Do not use other tags.

### Container Contents

| Component | Version | Purpose |
|-----------|---------|---------|
| Isaac Sim | 5.1.0 | Photorealistic simulation |
| Pegasus Simulator | v5.1.0 | Drone extensions |
| PX4 Autopilot | v1.14.3 | Flight controller |
| YOLO11 | ultralytics | Object detection |
| Stable Baselines3 | latest | RL algorithms |
| Gymnasium | latest | RL environment API |
| GCC | 12 | PX4 compilation |

### Environment Variables

| Variable | Value | Purpose |
|----------|-------|---------|
| `ISAACSIM_PATH` | `/isaac-sim` | Isaac Sim installation |
| `PEGASUS_PATH` | `/pegasus` | Pegasus Simulator |
| `PX4_HOME` | `/px4` | PX4 Autopilot |
| `YOLO_MODEL_PATH` | `/workspace/models/yolo11x.pt` | YOLO weights |

### Exposed Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 14540 | UDP/MAVLink | PX4 SITL default |
| 14550 | UDP/MAVLink | QGroundControl |
| 18570 | TCP | Lockstep synchronization |

## Directory Structure

```
isaac-sim-px4/
├── Containerfile              # Container build definition
├── README.md                  # This file
├── run_training.sh            # SAC training launcher
├── yolo11x.pt                 # YOLO model weights (114MB)
├── config/
│   └── canonical_missions.yaml # Mission configurations
├── environments/
│   ├── base_isr_env.py        # Base environment class
│   ├── comms_denied_env.py    # Comms-denied scenario
│   ├── dynamic_nfz_env.py     # Dynamic NFZ scenario
│   ├── multi_objective_env.py # Multi-objective scenario
│   ├── gymnasium_wrapper.py   # Gym API wrapper
│   ├── action_bridge.py       # Z-up → NED conversion
│   ├── safety_filter.py       # Vampire ATP integration
│   ├── ontology_behavior_controller.py # Axiom-driven behaviors
│   └── mission_tasking.py     # Operator tasking system
├── perception/
│   ├── detector.py            # YOLO detector wrapper
│   ├── ground_truth_detector.py # Frustum-based detector
│   ├── dual_mode_perception.py # GT/Full mode switching
│   └── frustum.py             # Camera frustum math
├── scripts/
│   ├── phase2_procgen_test.py # Procedural world test
│   ├── phase3_camera_gt_test.py # Ground truth perception
│   ├── phase4_gymnasium_test.py # Gym wrapper test
│   ├── phase5_reward_test.py  # Reward function test
│   ├── phase6_sac_test.py     # SAC training test
│   ├── phase7_safety_test.py  # Safety filter test
│   ├── phase8_yolo_test.py    # YOLO perception test
│   ├── run_phase_test.sh      # Unified test runner
│   └── training/
│       └── train_canonical.py # Full training script
├── extensions/
│   └── forest_generator/      # Procedural forest extension
├── tests/
│   └── test_integration.py    # Integration tests
├── logs/                      # Training logs (gitignored)
├── checkpoints/               # Model checkpoints (gitignored)
└── output/                    # Test outputs (gitignored)
```

## Running Scripts

### Using run_phase_test.sh (Recommended)

```bash
# Run any phase test with consistent container settings
./scripts/run_phase_test.sh <phase_number> [options]

# Examples:
./scripts/run_phase_test.sh 2              # Phase 2 with GUI
./scripts/run_phase_test.sh 6 --headless   # Phase 6 headless
./scripts/run_phase_test.sh 8 --save-images # Phase 8 with image output
```

### Manual Container Management

```bash
# Start container (detached)
xhost +local:
podman run -d --name flyby-f11 \
  --device nvidia.com/gpu=all \
  -e DISPLAY=$DISPLAY \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e OMNI_KIT_ACCEPT_EULA=YES \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --network host \
  --ipc host \
  --security-opt label=disable \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3

# Run a script
podman exec flyby-f11 bash -c \
  'export PYTHONPATH=/pegasus/extensions/pegasus.simulator:$PYTHONPATH && \
   /isaac-sim/python.sh /workspace/scripts/phase2_procgen_test.py'

# Stop container
podman rm -f flyby-f11
```

### Development with Mounted Volumes

```bash
# Mount local directories for live editing
podman run -d --name flyby-f11-dev \
  --device nvidia.com/gpu=all \
  -e DISPLAY=$DISPLAY \
  -e ACCEPT_EULA=Y \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/scripts:/workspace/scripts:z \
  -v $(pwd)/environments:/workspace/environments:z \
  -v $(pwd)/perception:/workspace/perception:z \
  --network host \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3
```

## Key Technical Details

### PX4 Simulation Loop

**Critical**: The simulation must step continuously without blocking calls.

```python
# WRONG - causes poll timeouts, PX4 starved of sensor data
time.sleep(5)

# CORRECT - use step counting (60 steps ≈ 1 second at 60Hz)
STEPS_TO_WAIT = 300  # 5 seconds
while step_count < STEPS_TO_WAIT:
    world.step(render=True)
    step_count += 1
```

### Coordinate Frame Conversion

| Frame | Convention | +X | +Y | +Z |
|-------|-----------|----|----|-----|
| Isaac Sim | Z-up (ENU-like) | East | North | Up |
| PX4 MAVLink | NED | North | East | Down |

The `ActionBridge` handles conversion automatically.

### Ontology Safety Behaviors

| Axiom | Priority | Trigger | Behavior |
|-------|----------|---------|----------|
| `batteryReserveReturn` | HIGH | battery < reserve | RTL |
| `criticalBattery` | CRITICAL | battery < 15% | EMERGENCY_LAND |
| `geofenceViolation` | HIGH | outside bounds | GEOFENCE_RECOVERY |
| `noFlyZoneViolation` | CRITICAL | inside NFZ | HOVER |
| `minPersonDistance` | CRITICAL | person < 10m | HOVER |

### Dual-Mode Perception

| Mode | Detector | Input | Performance | Use Case |
|------|----------|-------|-------------|----------|
| GT | GroundTruthDetector | Frustum math | 1000+ Hz | Fast RL training |
| Full | YOLODetector | Camera images | ~20 Hz | E2E validation |

Both modes produce identical 516-dimensional observations.

## Troubleshooting

### Poll Timeout Errors
```
ERROR [simulator_mavlink] poll timeout 0, 25
```
**Cause**: Simulation loop blocked
**Fix**: Remove `time.sleep()`, use step counting

### Arming Denied
```
WARN [commander] Arming denied: Resolve system health failures first
```
**Cause**: PX4 preflight checks incomplete
**Fix**: Wait longer before arming (300+ steps)

### Display Issues (GUI mode)
```bash
# Allow X11 access
xhost +local:

# Check DISPLAY
echo $DISPLAY
```

### YOLO Model Not Found
```
FileNotFoundError: yolo11x.pt not found
```
**Cause**: Model weights not in container
**Fix**: Ensure `yolo11x.pt` exists before build, or mount at runtime

## Remaining Work

### Critical
- [ ] E2E validation with YOLO perception (GT→Full parity)
- [ ] Test Dynamic NFZ and Multi-Objective environments
- [ ] Demonstrate trained policy success

### High Priority
- [ ] Log safety filter interventions during training
- [ ] Add unit tests for safety-critical components
- [ ] Document E2E workflow

### Medium Priority
- [ ] Consolidate duplicate code across phase scripts
- [ ] Create reusable test infrastructure module
- [ ] Performance benchmarks

## References

- [Isaac Sim 5.1.0 Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/)
- [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/)
- [PX4 Autopilot v1.14](https://docs.px4.io/v1.14/)
- [Stable Baselines3](https://stable-baselines3.readthedocs.io/)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
