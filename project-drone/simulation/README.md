# Test Drone Simulation Assets

Gazebo Harmonic simulation resources for the test-drone platform.

## Directory Structure

```
simulation/
├── models/
│   └── test_drone_x500/      # Custom drone with T265 + D455
│       ├── model.sdf          # SDF model definition
│       ├── model.config       # Model metadata
│       └── README.md          # Model documentation
├── worlds/
│   └── test_drone.sdf         # Custom world with obstacles
└── README.md                  # This file
```

## Quick Start

### Inside Docker Container

The simulation now automatically launches with the **outdoor forest environment** by default.

```bash
# Launch with forest world (default via start_simulation.sh)
# This is what runs when you do: docker compose up
/opt/start_simulation.sh

# Or manually launch forest world with standard x500 model
cd /opt/PX4-Autopilot
PX4_GZ_WORLD=test_drone make px4_sitl gz_x500

# Launch with custom test_drone model (T265 + D455 sensors) in forest
PX4_GZ_WORLD=test_drone PX4_GZ_MODEL=test_drone_x500 make px4_sitl gz_x500
```

### Switching Between Worlds

```bash
# Forest world (default, with trees and obstacles)
PX4_GZ_WORLD=test_drone make px4_sitl gz_x500

# Empty PX4 default world (minimal environment)
PX4_GZ_WORLD=default make px4_sitl gz_x500

# Or modify /opt/start_simulation.sh to change the default world
```

### Manual Launch with Custom Model and World

```bash
# Set model path (usually already set in container)
export GZ_SIM_RESOURCE_PATH="/workspace/simulation/models:/workspace/simulation/worlds:$PX4_ROOT/Tools/simulation/gz/models:$PX4_ROOT/Tools/simulation/gz/worlds"

# Launch PX4 with custom model and custom forest world
cd $PX4_ROOT
PX4_GZ_MODEL=test_drone_x500 PX4_GZ_WORLD=test_drone make px4_sitl gz_x500
```

## Models

### test_drone_x500

Complete quadcopter model matching real test-drone hardware:

**Base Platform**: PX4 x500 quadcopter (inherited via `<include merge="true">`)

**Sensors**:
- **Intel RealSense T265**: Visual odometry (200Hz pose + dual fisheye cameras)
- **Intel RealSense D455**: RGB-D camera (640x480 @ 30Hz, 0.3-10m range)

**ROS 2 Topics** (via ros_gz bridge):
- `/t265/odom` - Odometry (nav_msgs/Odometry)
- `/t265/fisheye1/image_raw` - Left fisheye camera
- `/t265/fisheye2/image_raw` - Right fisheye camera
- `/d455/color/image_raw` - RGB image
- `/d455/depth` - Depth image
- `/d455/infra1/image_raw` - Left IR camera
- `/d455/infra2/image_raw` - Right IR camera

See [models/test_drone_x500/README.md](models/test_drone_x500/README.md) for details.

## Worlds

### test_drone.sdf

Outdoor forest environment for test-drone development and testing:
- Large grass ground plane (200x200m)
- Directional lighting simulating sunlight
- **8 trees** arranged in natural clusters (North, East, South, West)
  - Varied heights (5-7m) and canopy sizes (2-3m radius)
  - Brown trunk cylinders with green spherical canopies
  - Full collision detection for obstacle avoidance testing
- **2 boulders** for terrain variation
- **2 bushes** as smaller vegetation obstacles
- Physics configured for real-time simulation
- Ideal for testing:
  - Depth camera obstacle detection (trees, boulders, bushes)
  - Visual odometry in outdoor environments
  - Navigation through natural obstacles
  - Perception pipeline with organic shapes

## Using Custom Models in PX4

### Why We Build `gz_x500` but Use `test_drone_x500`

**PX4 Build Process**:
```bash
make px4_sitl gz_x500
```
This builds the PX4 firmware configured for x500-class quadcopters (motor mixing, default params, etc.).

**Model Selection at Runtime**:
```bash
PX4_GZ_MODEL=test_drone_x500 make px4_sitl gz_x500
```
This **spawns** our custom model which:
1. Inherits all x500 dynamics (motors, physics, PX4 plugins)
2. Adds our custom sensors (T265, D455)

**Key Point**: The build target (`gz_x500`) sets the **vehicle dynamics**. The `PX4_GZ_MODEL` environment variable selects which **model to spawn** at runtime.

### Environment Variables

- `PX4_GZ_MODEL`: Model name to spawn (default: `x500`)
- `PX4_GZ_WORLD`: World file to load (default: `default`)
- `GZ_SIM_RESOURCE_PATH`: Paths to search for models/worlds

## Verifying Sensor Data

### Check Gazebo Topics

```bash
# List all Gazebo topics
gz topic -l

# Echo depth camera data
gz topic -e -t /d455/depth

# Echo odometry
gz topic -e -t /t265/odom
```

### Check ROS 2 Topics (with ros_gz bridge)

```bash
# List ROS 2 topics
ros2 topic list

# View RGB camera feed
ros2 run rqt_image_view rqt_image_view /d455/color/image_raw

# View depth camera feed
ros2 run rqt_image_view rqt_image_view /d455/depth

# Echo odometry
ros2 topic echo /t265/odom
```

## Adding New Models

1. Create model directory in `models/`
2. Add `model.sdf` and `model.config`
3. Optionally inherit from existing PX4 models:
   ```xml
   <include merge="true">
     <uri>model://x500</uri>
   </include>
   ```
4. Launch with `PX4_GZ_MODEL=your_model_name`

## Adding New Worlds

1. Create world SDF in `worlds/`
2. Launch with `PX4_GZ_WORLD=your_world_name`

Example:
```bash
PX4_GZ_WORLD=test_drone make px4_sitl gz_x500
```

## Troubleshooting

### World Not Found / Gazebo Hangs on "Waiting for Gazebo world..."

```
INFO  [init] Starting gazebo with world: /opt/PX4-Autopilot/Tools/simulation/gz/worlds/test_drone.sdf
Unable to find or download file
INFO  [init] Waiting for Gazebo world...
```

**Cause**: PX4 looks for world files in its own `Tools/simulation/gz/worlds/` directory, not in `GZ_SIM_RESOURCE_PATH`.

**Fix**: The custom world must be symlinked into PX4's worlds directory. This happens automatically during build:

```bash
# Inside container - rebuild PX4 to create symlinks
/opt/build_px4_sitl.sh

# Or manually create the symlink
ln -s /workspace/simulation/worlds/test_drone.sdf /opt/PX4-Autopilot/Tools/simulation/gz/worlds/test_drone.sdf

# Verify symlink
ls -la /opt/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### Model Not Found

```
Error: Unable to find or download model[test_drone_x500]
```

**Fix**: Ensure `GZ_SIM_RESOURCE_PATH` includes `/workspace/simulation/models`:
```bash
export GZ_SIM_RESOURCE_PATH="/workspace/simulation/models:$PX4_ROOT/Tools/simulation/gz/models"
```

### Sensor Data Not Publishing

Check that required Gazebo plugins are loaded:
- `gz-sim-sensors-system` (for cameras, depth sensors)
- `gz-sim-imu-system` (for IMU)
- `gz-sim-odometry-publisher-system` (for odometry)

### ros_gz Bridge Not Working

1. Verify ros_gz is sourced:
   ```bash
   source /opt/ros_gz_ws/install/setup.bash
   ```

2. Check bridge is running:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge /d455/depth@sensor_msgs/msg/Image[gz.msgs.Image
   ```

## Performance Tips

- **Headless mode**: Set `HEADLESS=1` to run without GUI
- **Reduce sensor rates**: Lower update rates in model.sdf if simulation is slow
- **Disable sensors**: Comment out unused sensors in model.sdf
- **Use GPU**: Uncomment GPU support in docker-compose.yml for hardware acceleration

## References

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [PX4 Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [SDF Format Specification](http://sdformat.org/)
- [ros_gz Bridge](https://github.com/gazebosim/ros_gz)
