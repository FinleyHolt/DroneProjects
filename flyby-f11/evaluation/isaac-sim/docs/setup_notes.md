# Isaac Sim Evaluation - Setup Notes

This document tracks setup issues, solutions, and lessons learned during the Isaac Sim evaluation.

## Quick Start

```bash
# 1. Pull Isaac Sim container (first time only, ~15GB compressed, ~30GB extracted)
# This can take 15-30 minutes depending on connection speed
podman pull nvcr.io/nvidia/isaac-sim:4.2.0

# 2. Enable X11 access for GUI mode
xhost +local:

# 3. Start Isaac Sim
cd flyby-f11/evaluation/isaac-sim

# Option A: GUI mode
./scripts/run-isaac-sim.sh

# Option B: Headless mode
./scripts/run-isaac-sim.sh --headless

# Option C: Interactive shell only
./scripts/run-isaac-sim.sh --shell

# 4. In another terminal, run tests
podman exec -it isaac-sim-dev python3 /workspace/scripts/test_flight.py
```

## GPU Passthrough Setup (Verified Working)

Podman uses CDI (Container Device Interface) for GPU access:

```bash
# 1. Install NVIDIA Container Toolkit (if not already)
sudo apt install nvidia-container-toolkit

# 2. Generate CDI config
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# 3. Verify GPU access
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

**Current System** (verified):
- GPU: NVIDIA GeForce RTX 5090 Laptop GPU (24GB)
- Driver: 590.48.01
- CDI Config: /etc/cdi/nvidia.yaml (exists)

## X11/Display Setup

For GUI mode on Wayland systems with XWayland:

```bash
# Allow local X11 connections
xhost +local:

# Verify X11 socket exists
ls -la /tmp/.X11-unix/

# Check display variable
echo $DISPLAY  # Should be :0 or :1
```

The compose file mounts:
- `/tmp/.X11-unix` - X11 socket
- `~/.Xauthority` - X11 authentication

## Known Issues

### Issue: NGC Authentication
**Symptom**: `unauthorized: authentication required` when pulling container

**Solution**:
```bash
# Create NGC API key at: https://ngc.nvidia.com/setup/api-key
podman login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>
```

### Issue: GPU Not Detected
**Symptom**: Isaac Sim fails with "No GPU found" or similar

**Solution**:
```bash
# Verify NVIDIA Container Toolkit CDI config exists
ls /etc/cdi/nvidia.yaml

# If missing, regenerate:
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Verify GPU access:
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

### Issue: Shader Compilation Takes Forever
**Symptom**: First startup takes 20+ minutes

**Solution**: This is expected on first run. The shader cache is persisted in the `isaac-cache` volume, so subsequent runs are faster.

### Issue: ArduPilot SITL Doesn't Connect
**Symptom**: MAVProxy shows "Waiting for heartbeat"

**Checklist**:
1. Verify SITL is running: `ps aux | grep arducopter`
2. Check ports: `ss -ulnp | grep 5760`
3. Verify JSON backend is configured in Pegasus
4. Check for lockstep sync issues in Isaac Sim logs

### Issue: Camera Topic Not Publishing
**Symptom**: `ros2 topic list` doesn't show `/f11/camera/image_raw`

**Checklist**:
1. Verify ROS 2 bridge extension is loaded
2. Check camera is attached to drone model
3. Verify ROS_DOMAIN_ID matches between Isaac Sim and listener

## Performance Tuning

### For MVE (Functionality Testing)
- Disable path tracing: `enable_path_tracing: false`
- Lower resolution: 640x480
- Reduce physics substeps if stability allows

### For CV Training (Production)
- Enable path tracing for photorealism
- Use 1080p or higher resolution
- Enable domain randomization

## Useful Commands

```bash
# Check Isaac Sim logs
podman logs -f isaac-sim-dev

# Interactive shell in container
podman exec -it isaac-sim-dev bash

# List ROS 2 topics (from inside container)
ros2 topic list

# View camera feed (requires GUI or X11 forwarding)
ros2 run rqt_image_view rqt_image_view /f11/camera/image_raw

# MAVProxy console
mavproxy.py --master=udp:127.0.0.1:14550

# ArduPilot SITL parameters
# Set in config/ardupilot_params.param
```

## Resources

- [Pegasus Simulator Docs](https://pegasussimulator.github.io/PegasusSimulator/)
- [Isaac Sim Container Guide](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/install_container.html)
- [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [ROS 2 Isaac Sim Bridge](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/ros_ros2_bridge.html)

## Changelog

- **2024-XX-XX**: Initial setup created
