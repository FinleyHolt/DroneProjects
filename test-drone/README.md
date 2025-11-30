# Test Drone

Development platform for LLMDrone autonomy stack with simulated T265 + D455 sensors.

## Quick Start

```bash
cd test-drone

# Build Docker image (first time only)
./sim build

# Launch simulation (PX4 SITL + Gazebo + QGroundControl)
./sim

# Stop with Ctrl+C
```

**That's it!** First run takes 5-10 minutes to build PX4, then subsequent runs are instant.

**Troubleshooting?** See [docker/TROUBLESHOOTING.md](docker/TROUBLESHOOTING.md)

---

## What You Get

When you run `./sim`, three windows open automatically:

1. **QGroundControl** - Ground control station for mission planning and telemetry
2. **Gazebo Harmonic** - 3D physics simulator with x500 quadcopter
3. **PX4 Console** - Autopilot command line (in your terminal)

---

## Commands

| Command | Description |
|---------|-------------|
| `./sim` | Start simulation (default) |
| `./sim build` | Build Docker image |
| `./sim down` | Stop simulation |
| `./sim shell` | Open shell in running container |

---

## Development Workflow

### 1. Start Simulation

```bash
./sim
```

### 2. Develop ROS 2 Packages

In another terminal:

```bash
./sim shell

# Inside container:
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select my_package
source install/setup.bash
ros2 launch my_package my_launch.py
```

### 3. Test in Simulation

Your ROS 2 nodes run alongside PX4 and can:
- Subscribe to sensor data (simulated T265, D455)
- Send MAVLink commands via MAVSDK
- Plan missions with BehaviorTree.CPP
- Test autonomy algorithms

### 4. Iterate

- Edit code on your host machine (`test-drone/ros2_ws/`)
- Changes appear immediately in container (volume mount)
- Rebuild and test without restarting simulation

---

## Real Hardware (T265 + D455 + PX4)

Coming soon - hardware not yet accessible. When available:

```bash
# Build flight test image
docker compose -f docker/docker-compose.yml build flight-test

# Launch with hardware access
docker compose -f docker/docker-compose.yml up flight-test

# Inside container:
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch test_drone_bringup real_hardware.launch.py
```

---

## Directory Structure

```
test-drone/
├── sim                      # Simulation launcher (./sim)
├── docker/
│   ├── Dockerfile.simulation      # Simulation container
│   ├── Dockerfile.flight_test     # Real hardware container
│   ├── docker-compose.yml         # Container orchestration
│   ├── README.md                  # Detailed Docker guide
│   └── TROUBLESHOOTING.md         # Problem-solving
├── ros2_ws/                 # ROS 2 workspace (mounted to container)
│   └── src/                 # Your ROS 2 packages go here
├── simulation/              # Gazebo models/worlds (mounted)
├── config/                  # PX4 params, calibration (mounted)
└── scripts/                 # Helper scripts (mounted)
    ├── build_px4_sitl.sh          # Build PX4 inside container
    └── start_simulation.sh        # Launch sim components
```

**Mounted directories** sync between host and container - edit on host, run in container.

---

## Common Tasks

### Rebuild PX4 (if needed)

```bash
./sim shell
cd /opt/PX4-Autopilot
make clean
/opt/build_px4_sitl.sh

# Or limit CPU usage:
PX4_BUILD_JOBS=4 /opt/build_px4_sitl.sh
```

### Test Basic Flight

In the PX4 console (terminal running `./sim`):

```bash
pxh> commander takeoff    # Arm and takeoff
pxh> commander land       # Land
pxh> mavlink status       # Check connections
```

### Plan a Mission

Use QGroundControl (GUI window):
1. Click "Plan" tab
2. Add waypoints on map
3. Upload to vehicle
4. Switch to "Fly" tab and start mission

---

## Requirements

- **OS**: Ubuntu 22.04 LTS (or compatible)
- **Docker**: Docker + Docker Compose v2
- **RAM**: 8GB minimum, 16GB recommended
- **Disk**: ~15GB for Docker image + builds
- **CPU**: 4+ cores recommended
- **Display**: X11 server for GUI windows

**Optional:**
- NVIDIA GPU with drivers (better Gazebo performance)

---

## Network & Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 14550 | UDP | MAVLink to QGroundControl |
| 14540 | UDP | MAVLink onboard (MAVSDK) |
| 8888 | UDP | UXRCE-DDS (ROS 2 ↔ PX4 bridge) |

**ROS Domain ID**: 42 (isolated from other ROS systems)

**Network Mode**: Host networking (for ROS 2 DDS discovery)

---

## Troubleshooting

### GUI Windows Don't Appear

```bash
# Enable X11 access
xhost +local:docker

# Verify display
echo $DISPLAY  # Should show :0 or :1

# Restart simulation
./sim down
./sim
```

### First Run is Slow

**This is normal!** First `./sim` run builds PX4 SITL (5-10 minutes). You'll see:

```
==========================================
 FIRST RUN: Building PX4 SITL
==========================================
This will take 5-10 minutes on first run.
Subsequent runs will be instant.
```

Subsequent runs skip the build and launch immediately.

### Common Warnings (Safe to Ignore)

- `libdebuginfod.so.1: cannot open shared object file` - Missing debug symbols
- `Serial permissions error` - No real serial hardware in simulation
- `No connection to the GCS` - Wait 5-10 seconds for QGC to connect

### More Help

See [docker/TROUBLESHOOTING.md](docker/TROUBLESHOOTING.md) for comprehensive debugging.

---

## Documentation

- **[docker/README.md](docker/README.md)** - Comprehensive Docker guide
- **[docker/TROUBLESHOOTING.md](docker/TROUBLESHOOTING.md)** - Problem-solving reference
- **[../CLAUDE.md](../CLAUDE.md)** - Project overview and architecture
- **[TODO.md](TODO.md)** - Development tracking

---

## Platform Details

- **OS**: Ubuntu 22.04
- **ROS**: ROS 2 Humble
- **Simulator**: Gazebo Harmonic
- **Autopilot**: PX4 main branch
- **Autonomy**: MAVSDK, BehaviorTree.CPP, RTAB-Map

---

## Next Steps

1. **Test flight**: Run `commander takeoff` in PX4 console
2. **Plan mission**: Create waypoints in QGroundControl
3. **Develop autonomy**: Create ROS 2 packages in `ros2_ws/src/`
4. **Custom models**: Add Gazebo models in `simulation/models/`
5. **Read architecture**: See [../CLAUDE.md](../CLAUDE.md) for project context

**Questions?** Check [docker/README.md](docker/README.md) for detailed information or [docker/TROUBLESHOOTING.md](docker/TROUBLESHOOTING.md) for common issues.
