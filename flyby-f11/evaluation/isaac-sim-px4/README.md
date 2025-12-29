# Isaac Sim 5.1.0 + PX4 + Pegasus Simulator

Photorealistic drone simulation using NVIDIA Isaac Sim 5.1.0, PX4 Autopilot v1.14.3, and Pegasus Simulator v5.1.0.

## Overview

This setup provides:
- **Isaac Sim 5.1.0**: Photorealistic simulation with PhysX 5, ray-traced rendering
- **Pegasus 5.1.0**: Drone-specific extensions for Isaac Sim
- **PX4 v1.14.3**: Production-grade autopilot (Pegasus recommended version)
- **Podman**: Rootless containers with GPU passthrough

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
podman build -t isaac-sim-px4:5.1.0 .
```

Build takes ~30 minutes (PX4 compilation).

### 3. Run the Simulation

```bash
# Allow X11 access (required for GUI)
xhost +local:

# Start container with GPU passthrough
podman run -d --name px4-sim \
  --device nvidia.com/gpu=all \
  -e DISPLAY=$DISPLAY \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e OMNI_KIT_ACCEPT_EULA=YES \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --network host \
  --ipc host \
  --security-opt label=disable \
  isaac-sim-px4:5.1.0

# Run the flight mission script
podman exec px4-sim bash -c \
  'export PYTHONPATH=/pegasus/extensions/pegasus.simulator:$PYTHONPATH && \
   /isaac-sim/python.sh /workspace/scripts/fly_mission.py'
```

### 4. Expected Output

```
============================================================
Isaac Sim + PX4 Autonomous Flight Demo
============================================================

[1/5] Initializing Isaac Sim...
  Isaac Sim ready

[2/5] Loading Pegasus extension...
  Pegasus loaded

[3/5] Setting up world...
  Environment loaded

[4/5] Spawning Iris with PX4...
  PX4 path: /isaac-sim/PX4-Autopilot
  PX4 airframe: gazebo-classic_iris
  Iris spawned

[5/5] Starting simulation loop...
============================================================
Simulation running! Watch the drone:
  1. Wait for PX4 ready
  2. Arm
  3. Takeoff to 2.5m
  4. Hover for 10 seconds
  5. Land
Press Ctrl+C to stop.
============================================================

step=  120 | State: INIT       | Pos: [ 0.00, 0.00, 0.05]
step=  240 | State: INIT       | Pos: [ 0.00, 0.00, 0.05]

>>> Waiting for PX4 ready...
>>> Sending arm command...
>>> Sending takeoff command...
step=  720 | State: TAKEOFF    | Pos: [ 0.00, 0.00, 0.05]
step=  960 | State: HOVER      | Pos: [ 0.11, 0.09, 0.95]
step= 1200 | State: HOVER      | Pos: [-0.10,-0.01, 2.34]
step= 1440 | State: HOVER      | Pos: [-0.03, 0.04, 2.52]
>>> Sending land command...
step= 1680 | State: LAND       | Pos: [ 0.00, 0.02, 1.51]
step= 1920 | State: LAND       | Pos: [-0.02, 0.02, 0.05]
>>> Flight complete!
>>> Simulation will continue. Press Ctrl+C to exit.
```

## Complete Workflow Commands

### Container Management

```bash
# Build container
podman build -t isaac-sim-px4:5.1.0 .

# Start container (detached)
xhost +local:
podman run -d --name px4-sim \
  --device nvidia.com/gpu=all \
  -e DISPLAY=$DISPLAY \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e OMNI_KIT_ACCEPT_EULA=YES \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --network host \
  --ipc host \
  --security-opt label=disable \
  isaac-sim-px4:5.1.0

# Check container status
podman ps

# View container logs
podman logs px4-sim

# Stop and remove container
podman rm -f px4-sim

# List images
podman images | grep isaac
```

### Running Scripts

```bash
# Run flight mission (with GUI)
podman exec px4-sim bash -c \
  'export PYTHONPATH=/pegasus/extensions/pegasus.simulator:$PYTHONPATH && \
   /isaac-sim/python.sh /workspace/scripts/fly_mission.py'

# Run headless (no GUI)
podman exec px4-sim bash -c \
  'export PYTHONPATH=/pegasus/extensions/pegasus.simulator:$PYTHONPATH && \
   /isaac-sim/python.sh /workspace/scripts/fly_mission.py --headless'

# Run official Pegasus example
podman exec px4-sim bash -c \
  'export PYTHONPATH=/pegasus/extensions/pegasus.simulator:$PYTHONPATH && \
   /isaac-sim/python.sh /pegasus/examples/1_px4_single_vehicle.py'

# Interactive shell inside container
podman exec -it px4-sim bash
```

### Manual PX4 Control (while simulation is running)

```bash
# Check PX4 status
podman exec px4-sim bash -c 'cd /px4 && build/px4_sitl_default/bin/px4-commander status'

# Arm vehicle
podman exec px4-sim bash -c 'cd /px4 && build/px4_sitl_default/bin/px4-commander arm'

# Takeoff
podman exec px4-sim bash -c 'cd /px4 && build/px4_sitl_default/bin/px4-commander takeoff'

# Land
podman exec px4-sim bash -c 'cd /px4 && build/px4_sitl_default/bin/px4-commander land'

# Check vehicle position
podman exec px4-sim bash -c 'cd /px4 && build/px4_sitl_default/bin/px4-listener vehicle_local_position'
```

### Development with Mounted Scripts

```bash
# Mount local scripts directory for live editing
podman run -d --name px4-dev \
  --device nvidia.com/gpu=all \
  -e DISPLAY=$DISPLAY \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e OMNI_KIT_ACCEPT_EULA=YES \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/scripts:/workspace/scripts:z \
  --network host \
  --ipc host \
  --security-opt label=disable \
  isaac-sim-px4:5.1.0

# Edit scripts locally, then run:
podman exec px4-dev bash -c \
  'export PYTHONPATH=/pegasus/extensions/pegasus.simulator:$PYTHONPATH && \
   /isaac-sim/python.sh /workspace/scripts/fly_mission.py'
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Isaac Sim Container                       │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐ │
│  │ Isaac Sim   │    │  Pegasus    │    │   PX4 SITL      │ │
│  │ 5.1.0       │◄──►│  Simulator  │◄──►│   v1.14.3       │ │
│  │ (Rendering) │    │  v5.1.0     │    │   (Autopilot)   │ │
│  └─────────────┘    └─────────────┘    └─────────────────┘ │
│         │                  │                    │           │
│         │          TCP 4560 (lockstep)         │           │
│         │          HIL_SENSOR ──────►          │           │
│         │          ◄────── HIL_ACTUATOR        │           │
│         │                                       │           │
│         │                           UDP 14550   │           │
│         │                           (MAVLink)   │           │
└─────────────────────────────────────────────────────────────┘
```

### Communication Channels

| Channel | Port | Protocol | Purpose |
|---------|------|----------|---------|
| Lockstep | TCP 4560 | HIL MAVLink | Sensor data & actuator commands |
| GCS | UDP 14550 | MAVLink | QGroundControl connection |
| SITL | UDP 14540 | MAVLink | PX4 default SITL port |
| Offboard | UDP 18570 | MAVLink | Additional MAVLink |

## Key Technical Findings

### 1. Ubuntu Version Compatibility

**Isaac Sim 5.1.0 supports both Ubuntu 22.04 and 24.04** according to [NVIDIA documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html). The official container image is based on Ubuntu 24.04.

**Pegasus Simulator was tested on Ubuntu 22.04** per [Pegasus docs](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html), but works correctly on Ubuntu 24.04 with proper configuration.

### 2. PX4 Version and Compiler Requirements

**PX4 v1.14.3 is required** - This is the version recommended by Pegasus Simulator. Newer versions (v1.15+, v1.16) have protocol changes that break compatibility.

**GCC 12 is required on Ubuntu 24.04** - PX4 v1.14.3 fails to compile with GCC 13/14 (Ubuntu 24.04 defaults) due to:
- Missing `#include <cstdint>` in some files
- Stricter array bounds warnings treated as errors

The Containerfile sets:
```dockerfile
ENV CC=/usr/bin/gcc-12
ENV CXX=/usr/bin/g++-12
```

### 3. Simulation Loop Requirements

**Critical: The simulation must step continuously without blocking calls.**

```python
# WRONG - causes poll timeouts, PX4 starved of sensor data
time.sleep(5)

# CORRECT - use step counting (60 steps ≈ 1 second at 60Hz)
STEPS_TO_WAIT = 300  # 5 seconds
while step_count < STEPS_TO_WAIT:
    world.step(render=True)
    step_count += 1
```

Blocking calls cause:
```
ERROR [simulator_mavlink] poll timeout 0, 25
```

### 4. Command Interfaces

There are **two different ways** to control PX4:

| Interface | Method | Use Case | Works in Script? |
|-----------|--------|----------|------------------|
| PX4 Commander CLI | Internal IPC | Arm/takeoff/land | Yes (threaded) |
| MAVLink UDP | External GCS | QGroundControl | Limited from script |
| Lockstep TCP | HIL protocol | Sensor/actuator | Handled by Pegasus |

**PX4 Commander CLI is the reliable method for script control:**
```python
import subprocess
import threading

def px4_cmd_async(cmd_args):
    """Execute PX4 commander in background thread."""
    def run():
        subprocess.run(
            ["/px4/build/px4_sitl_default/bin/px4-commander"] + cmd_args.split(),
            cwd="/px4",
            timeout=10
        )
    threading.Thread(target=run, daemon=True).start()

# Usage
px4_cmd_async("arm")
px4_cmd_async("takeoff")
px4_cmd_async("land")
```

**Important:** Subprocess calls must run in background threads to avoid blocking the simulation loop and causing glibc pthread assertion errors.

### 5. Pegasus Configuration

Pegasus reads configuration from `/pegasus/extensions/pegasus.simulator/config/configs.yaml`:
```yaml
px4_dir: ~/PX4-Autopilot
px4_default_airframe: gazebo-classic_iris
```

The `~` expands to `/isaac-sim` (isaac-sim user's home). The Containerfile creates a symlink:
```dockerfile
RUN ln -sfn ${PX4_HOME} /isaac-sim/PX4-Autopilot
```

Use `PegasusInterface()` to access these paths:
```python
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
pg = PegasusInterface()
print(pg.px4_path)            # /isaac-sim/PX4-Autopilot
print(pg.px4_default_airframe)  # gazebo-classic_iris
```

### 6. PYTHONPATH for Pegasus

Pegasus is not in Isaac Sim's Python path by default:
```bash
export PYTHONPATH=/pegasus/extensions/pegasus.simulator:$PYTHONPATH
/isaac-sim/python.sh your_script.py
```

Or in Python:
```python
import sys
sys.path.insert(0, "/pegasus/extensions/pegasus.simulator")
```

## File Structure

```
isaac-sim-px4/
├── Containerfile           # Container build definition
├── README.md              # This file
└── scripts/
    └── fly_mission.py     # Working flight demo script
```

## Container Details

| Component | Version | Notes |
|-----------|---------|-------|
| Base Image | nvcr.io/nvidia/isaac-sim:5.1.0 | Ubuntu 24.04 |
| PX4 Autopilot | v1.14.3 | Pegasus recommended |
| Pegasus Simulator | v5.1.0 | Matches Isaac Sim version |
| GCC | 12 | Required for PX4 build on Ubuntu 24.04 |
| Python | 3.11 | Isaac Sim's embedded Python |

## Troubleshooting

### Poll Timeout Errors
```
ERROR [simulator_mavlink] poll timeout 0, 25
```
**Cause:** Simulation loop is blocked, PX4 not receiving sensor data.
**Fix:** Remove all `time.sleep()` calls, use step counting instead.

### Arming Denied
```
WARN [commander] Arming denied: Resolve system health failures first
```
**Cause:** PX4 preflight checks not complete.
**Fix:** Wait longer before arming (increase STEPS_INIT to 300+ steps).

### glibc pthread Assertion Error
```
Fatal glibc error: tpp.c:86 (__pthread_tpp_change_priority): assertion failed
```
**Cause:** Blocking subprocess call in simulation thread.
**Fix:** Run subprocess commands in background threads with `threading.Thread(daemon=True)`.

### Display Issues (GUI mode)
```bash
# Allow X11 access (run on host)
xhost +local:

# Check DISPLAY variable
echo $DISPLAY

# For Wayland, ensure XWayland is running
```

### Isaac Sim Crashes on Start
```bash
# Clear shader cache and retry
rm -rf ~/.cache/nvidia/ComputeCache
```

### PX4 Build Fails with GCC 13/14
```
error: 'uint8_t' was not declared in this scope
error: array subscript 1 is above array bounds
```
**Cause:** PX4 v1.14.3 incompatible with newer GCC.
**Fix:** Already handled in Containerfile (uses GCC 12).

## Flight Sequence States

The `fly_mission.py` script implements this state machine:

| State | Duration | Description |
|-------|----------|-------------|
| INIT | 5s (300 steps) | Wait for Isaac Sim + PX4 initialization |
| WAIT_READY | 3s (180 steps) | Wait for PX4 preflight checks |
| ARM | 2s (120 steps) | Send arm command, wait for confirmation |
| TAKEOFF | 6s (360 steps) | Send takeoff, climb to 2.5m |
| HOVER | 10s (600 steps) | Hold position at altitude |
| LAND | 10s (600 steps) | Send land command, descend |
| DONE | Indefinite | Simulation continues running |

## Next Steps

Potential enhancements for future development:

1. **Offboard Position Control** - Implement MAVLink SET_POSITION_TARGET for waypoint navigation
2. **Camera Integration** - Add RGB/depth camera for perception testing
3. **Custom F-11 USD Model** - Replace Iris with Flyby F-11 geometry
4. **Domain Randomization** - Vary lighting, textures, obstacles for training
5. **ROS 2 Bridge** - Connect to external ROS 2 nodes for autonomy stack testing
6. **Multi-Vehicle** - Spawn multiple drones using Pegasus multi-vehicle examples

## References

- [Isaac Sim 5.1.0 Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/)
- [Isaac Sim Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
- [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/)
- [Pegasus Installation](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)
- [PX4 Autopilot v1.14](https://docs.px4.io/v1.14/)
- [PX4 SITL Simulation](https://docs.px4.io/v1.14/en/simulation/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
