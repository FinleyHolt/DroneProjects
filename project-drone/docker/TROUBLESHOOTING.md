# Docker Simulation Troubleshooting

This document covers common issues and solutions for running the test-drone simulation in Docker.

## Quick Start

```bash
cd test-drone

# First time: Build the image
./sim build

# Start simulation
./sim
```

## Common Issues

### 1. First Run Takes 5-10 Minutes

**Symptom**: When running `./sim` for the first time after `./sim build`, there's a long delay before QGroundControl appears.

**Cause**: PX4 SITL needs to be compiled on first run (not included in Docker image to save space and allow for faster rebuilds).

**Expected Behavior**:
```bash
./sim  # First run
# You'll see: "FIRST RUN: Building PX4 SITL"
# Wait 5-10 minutes for PX4 to compile
# Then QGroundControl and Gazebo will launch automatically
```

**Subsequent runs are instant**:
```bash
./sim  # Second+ run
# You'll see: "PX4 SITL already built, skipping build step"
# QGroundControl and Gazebo launch immediately
```

**Why this design?**
- Keeps Docker image size smaller
- Allows PX4 source code changes without rebuilding entire image
- ccache speeds up incremental rebuilds

### 2. QGroundControl Not Appearing

**Symptom**: QGroundControl window doesn't open even after PX4 build completes.

**Solution**: Check X11 permissions on host:
```bash
xhost +local:docker
echo $DISPLAY  # Should show :0 or similar
```

If still not working, check container logs:
```bash
docker logs test-drone-simulation | grep -i qground
```

### 3. Firmware Version Showing 0.0.0

**Symptom**: QGroundControl shows:
- "Vehicle is not running latest stable firmware! Running 0.0.0"
- "Unsupported PX4 version"

**Cause**: PX4 SITL was being built with `DONT_RUN=1` flag, which skips proper configuration.

**Solution**: Fixed in `scripts/build_px4_sitl.sh`. The build script now:
1. Attempts normal build (may briefly launch/exit PX4)
2. Falls back to `DONT_RUN=1` if needed
3. Ensures proper firmware version detection

To rebuild PX4 inside the container:
```bash
./sim shell
/opt/build_px4_sitl.sh
```

### 4. Serial Permissions Warning

**Symptom**: QGroundControl shows:
- "The current user does not have the correct permissions to access serial devices"
- "You should also remove modemmanager"

**Cause**: QGroundControl checks for serial devices even in simulation mode.

**Solution**:
- Modemmanager is now removed during Docker image build
- Serial device warnings are expected in SITL (no physical hardware)
- These warnings don't affect simulation functionality

**Note**: For SITL simulation, you can safely ignore serial permission warnings since no physical serial devices are needed.

### 5. Gazebo Not Starting

**Symptom**: PX4 starts but Gazebo GUI doesn't appear.

**Possible Causes**:
1. X11 display not forwarded to container
2. GPU drivers not available in container
3. Insufficient GPU memory

**Solutions**:

Check X11 is enabled:
```bash
echo $DISPLAY  # Should show :0 or similar
xhost +local:docker
```

For NVIDIA GPU support, uncomment in `docker/docker-compose.yml`:
```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [gpu, graphics, compute, utility]
```

Verify GPU access:
```bash
./sim shell
nvidia-smi  # Should show GPU info
```

### 6. Build Failures

**Symptom**: Docker image build fails during PX4 compilation.

**Solutions**:

Disable BuildKit if having cache issues:
```bash
DOCKER_BUILDKIT=0 docker compose -f docker/docker-compose.yml build simulation
```

Clean build with no cache:
```bash
docker compose -f docker/docker-compose.yml build --no-cache simulation
```

Increase Docker memory allocation (Docker Desktop settings):
- Recommended: 8GB+ RAM
- Recommended: 4GB+ swap

### 7. Container Won't Start

**Symptom**: `./sim` fails with container errors.

**Solutions**:

Check if container is already running:
```bash
docker ps -a | grep test-drone
```

Remove stale containers:
```bash
docker compose -f docker/docker-compose.yml down
docker rm test-drone-simulation
```

Check Docker disk space:
```bash
docker system df
docker system prune  # If needed
```

## Verification Steps

After starting the simulation, you should see:

1. **Terminal Output**:
   ```
   === Test Drone Simulation Launcher ===
   Starting QGroundControl...
   QGroundControl started (PID: ...)
   Starting PX4 SITL + Gazebo Harmonic...
   ```

2. **QGroundControl Window**: Should open and connect to PX4
3. **Gazebo Window**: Should show drone model in world
4. **No Critical Errors**: Serial warnings are OK for SITL

## Rebuilding After Fixes

If you pulled updates that fix these issues:

```bash
cd test-drone

# Stop any running containers
./sim down

# Rebuild the image
./sim build

# Start fresh
./sim
```

## Advanced Debugging

### View Container Logs
```bash
docker logs test-drone-simulation
```

### Interactive Shell
```bash
./sim shell
# Inside container:
ps aux  # Check running processes
/opt/build_px4_sitl.sh  # Rebuild PX4
```

### Manual PX4 Launch
```bash
./sim shell
cd /opt/PX4-Autopilot
make px4_sitl gz_x500
```

### Check QGroundControl Config
```bash
./sim shell
ls -la /home/qgcuser/.config/QGroundControl.org/
```

## Getting Help

If issues persist:

1. Check Docker/system requirements in `docker/BUILD_OPTIMIZATIONS.md`
2. Verify X11 setup in `docker/GUI_SETUP.md`
3. Review container logs: `docker logs test-drone-simulation`
4. Report issues with full error output and system info
