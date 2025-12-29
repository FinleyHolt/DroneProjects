# Flyby F-11 Simulation Environment

This directory contains the Gazebo Harmonic simulation environment for training and testing the F-11 autonomous drone.

## Quick Start

```bash
# Build the container
cd simulation
podman build -t flyby-f11-sim:latest -f Containerfile.sitl .

# Run with GUI (for visualization)
podman run --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    --network host \
    localhost/flyby-f11-sim:latest

# Run headless (for training)
podman run --rm \
    -e HEADLESS=true \
    --network host \
    localhost/flyby-f11-sim:latest
```

## Directory Structure

```
simulation/
├── Containerfile.sitl          # Container definition with Gazebo + ArduPilot
├── README.md                   # This file
├── configs/
│   ├── arducopter.parm         # ArduPilot parameter file
│   └── locations.txt           # SITL location definitions
├── models/
│   └── f11_isr_camera/         # F-11 drone model with ISR camera + gimbal
│       └── model.sdf
├── worlds/                     # SDF world files
│   ├── urban_training.sdf      # Dense city (300x300m)
│   ├── rural_training.sdf      # Forest/terrain (400x400m)
│   ├── industrial_training.sdf # Port/warehouse (350x350m)
│   ├── coastal_training.sdf    # Maritime (400x400m, 50% water)
│   ├── randomized_training.sdf # Mixed template (350x350m)
│   └── world_configs/          # YAML configs for procedural generation
│       ├── urban_config.yaml
│       ├── rural_config.yaml
│       ├── industrial_config.yaml
│       ├── coastal_config.yaml
│       └── randomized_config.yaml
├── scripts/
│   ├── entrypoint.sh           # Container entrypoint
│   ├── randomize_world.py      # Domain randomization generator
│   └── test_worlds.py          # World testing script
└── test_recordings/            # Test output directory
```

## Training Worlds

All worlds include:
- F-11 drone with ISR camera and 3-axis gimbal
- Geofence boundaries (green visual markers)
- 2+ No-Fly Zones (red transparent volumes)
- 4 landing zones (white/yellow/orange circles)
- GPS, IMU, and barometer sensors
- **Configurable lighting presets** (day, dawn, dusk, overcast, morning)
- **Atmospheric fog effects** for visual variety
- **Properly-sized collision primitives** for realistic physics

### 1. Urban Training (`urban_training.sdf`)
- **Theme**: Dense city environment (Phoenix, AZ area)
- **Size**: 300m x 300m
- **Features**: 12+ buildings (10-50m), vehicles with collision boxes, hospital helipad, street canyons
- **Infrastructure**: Traffic lights, stop signs, crosswalks, dumpsters
- **NFZ**: Hospital helipad (cylinder), Government building (box)
- **Use Case**: Navigation in complex urban airspace, obstacle avoidance

### 2. Rural Training (`rural_training.sdf`)
- **Theme**: Forest and natural terrain (Northern California)
- **Size**: 400m x 400m
- **Features**: Trees with trunk+canopy collisions, hills, varied rock types (Stone 1/5/8), cabin
- **NFZ**: Wildlife nesting area (cylinder), Private property (box)
- **Use Case**: Natural obstacle avoidance, terrain-following

### 3. Industrial Training (`industrial_training.sdf`)
- **Theme**: Port/warehouse environment (Long Beach area)
- **Size**: 350m x 350m
- **Features**: Warehouses, stacked containers, gantry crane (30m), forklifts with collisions
- **Weather**: Industrial haze fog option for reduced visibility training
- **NFZ**: Chemical storage (cylinder), Active crane zone (box)
- **Use Case**: Structured environment navigation, height-varying obstacles

### 4. Coastal Training (`coastal_training.sdf`)
- **Theme**: Maritime/harbor environment (San Diego area)
- **Size**: 400m x 400m (50% water, 50% land)
- **Features**: Piers, boats (Fishing Boat, Speed Boat, Jetski), lighthouse, floating landing platform
- **Weather**: Sea fog option for maritime visibility conditions
- **NFZ**: Shipping lane (large box), Restricted harbor (cylinder)
- **Use Case**: Over-water flight, maritime infrastructure

### 5. Randomized Training (`randomized_training.sdf`)
- **Theme**: Mixed obstacle template for domain randomization
- **Size**: 350m x 350m
- **Features**: Buildings, towers, tanks, poles, trees (all marked randomizable)
- **Lighting**: Configurable via `randomize_world.py` script
- **NFZ**: 2 zones with ground facilities (positions randomizable)
- **Use Case**: Base template for domain randomization - generates world variants

## F-11 Drone Model

The `f11_isr_camera` model includes:

### Sensors
| Sensor | Topic | Rate | Resolution |
|--------|-------|------|------------|
| ISR Camera | `/world/{world}/model/f11_isr/.../isr_camera/image` | 30 Hz | 1920x1080 |
| IMU | `/world/{world}/model/f11_isr/.../imu_sensor/imu` | 200 Hz | - |
| GPS | `/world/{world}/model/f11_isr/.../gps_sensor/navsat` | 10 Hz | - |
| Barometer | `/world/{world}/model/f11_isr/.../air_pressure` | 50 Hz | - |

### Gimbal Control

| Joint | Topic | Range |
|-------|-------|-------|
| Yaw | `/f11_isr/gimbal/yaw` | Continuous |
| Pitch | `/f11_isr/gimbal/pitch` | ±120° |
| Roll | `/f11_isr/gimbal/roll` | ±45° |

Control via `gz.msgs.Double`:
```bash
gz topic -t /f11_isr/gimbal/pitch -m gz.msgs.Double -p 'data: -0.5'
```

## Domain Randomization

The `randomize_world.py` script generates world variants for RL training to prevent overfitting:

```bash
# Generate 10 variants of all worlds
python scripts/randomize_world.py \
    --config-dir simulation/worlds/world_configs \
    --output-dir simulation/worlds/generated \
    --num-variants 10 \
    --seed 42 \
    --generate-sdf

# Generate variants for a single world
python scripts/randomize_world.py \
    --base-config simulation/worlds/world_configs/urban_config.yaml \
    --output-dir simulation/worlds/generated \
    --num-variants 20 \
    --seed 123
```

### Randomization Parameters

Each world config YAML includes a `randomization` section:

```yaml
randomization:
  enabled: true

  obstacle_placement:
    enabled: true
    position_noise: 15    # meters - random offset applied to obstacle positions
    rotation_noise: 0.3   # radians - random yaw rotation applied to obstacles

  nfz_placement:
    enabled: true
    position_noise: 20    # meters - random offset to NFZ centers
    size_noise: 0.2       # fraction - random scaling of NFZ radius/size

  lighting:
    enabled: true
    options: ["day", "dawn", "dusk", "overcast", "morning"]  # randomly selected

  spawn_variation:
    enabled: true
    position_noise: 5     # meters - variation in drone spawn position
```

### Lighting Presets

The randomization system supports 5 lighting presets for domain variation:

| Preset | Description | Sun Direction | Shadows |
|--------|-------------|---------------|---------|
| `day` | Bright midday sun | Overhead (-0.4, 0.2, -0.9) | Yes |
| `dawn` | Orange sunrise | East (0.8, 0.2, -0.3) | Yes |
| `dusk` | Red/orange sunset | West (-0.8, 0.2, -0.3) | Yes |
| `overcast` | Cloudy, diffuse light | Straight down | No |
| `morning` | Softer morning light | East-overhead (0.6, 0.3, -0.7) | Yes |

Each preset adjusts:
- **Diffuse**: Primary light color and intensity
- **Specular**: Highlight reflections
- **Direction**: Sun angle (affects shadow direction)
- **Ambient**: Background lighting level
- **Background**: Sky color

### Fog/Weather Effects

All worlds include configurable fog settings for visual variety during training:

```xml
<scene>
  <fog>
    <color>0.85 0.87 0.9 1</color>
    <type>linear</type>
    <start>50</start>   <!-- Fog begins at 50m -->
    <end>300</end>      <!-- Full fog at 300m -->
    <density>0.02</density>
  </fog>
</scene>
```

| World | Fog Type | Description |
|-------|----------|-------------|
| Urban | Light haze | Mild urban visibility reduction |
| Rural | None | Clear conditions by default |
| Industrial | Industrial haze | Moderate visibility (pollution effect) |
| Coastal | Sea fog | Maritime fog conditions |

### Output Format

Generated variants include:
- `{variant_name}.yaml` - Full configuration with randomized parameters
- `{variant_name}.json` - JSON format for Python loading
- `{variant_name}.sdf` - Modified SDF file (with `--generate-sdf`)
- `manifest.json` - Index of all generated variants with metadata

## World Configuration Schema

Each world config YAML follows this structure:

```yaml
world:
  name: "flyby_urban"
  sdf_file: "urban_training.sdf"
  description: "Dense urban environment..."
  dimensions:
    x_min: -150
    x_max: 150
    y_min: -150
    y_max: 150
    z_min: 0
    z_max: 100

coordinates:
  latitude_deg: 33.4484
  longitude_deg: -112.0740
  elevation: 331
  frame: ENU

spawn_points:
  - name: "default"
    pose: [0, 0, 0.5, 0, 0, 0]  # [x, y, z, roll, pitch, yaw]
    description: "Ground level at origin"
  - name: "rooftop_A1"
    pose: [-80, -80, 51, 0, 0, 0]
    description: "On building rooftop"

landing_zones:
  - name: "ground_primary"
    position: [0, 0, 0]
    radius: 3.0
    type: "primary"
    surface: "concrete"
  - name: "rooftop_A1"
    position: [-80, -80, 50]
    radius: 2.5
    type: "rooftop"

no_fly_zones:
  - name: "hospital_helipad"
    type: "cylinder"
    center: [0, 80, 0]
    radius: 25
    height: 60
    reason: "Active hospital helipad"
    severity: "critical"
    randomizable: true
  - name: "government_building"
    type: "box"
    min: [60, -15, 0]
    max: [100, 15, 45]
    reason: "Restricted airspace"
    severity: "critical"

obstacles:
  buildings:
    - name: "building_A1"
      type: "box"
      pose: [-80, -80, 25]
      size: [30, 30, 50]
      randomizable: true
  vehicles:
    count: 5
    height_range: [1.0, 3.0]
    positions: "street_level"

training_parameters:
  episode_timeout: 300  # seconds
  max_altitude: 80
  min_altitude: 1
  min_obstacle_distance: 3.0
  reward_weights:
    goal_reached: 100.0
    nfz_violation: -200.0
    collision: -500.0
    geofence_violation: -300.0

environment:
  type: "urban"
  ground_texture: "asphalt"
  lighting: "day"
  complexity: "high"

randomization:
  enabled: true
  # ... (see above)
```

## Gazebo Fuel Models

The simulation worlds use **textured models from [Gazebo Fuel](https://app.gazebosim.org/fuel/models)** for improved visual fidelity while maintaining simple collision primitives for RL training performance.

### Urban Training World - Model Mapping

The `urban_training.sdf` world uses the following Fuel models from OpenRobotics:

| World Element | Fuel Model | Position | Notes |
|---------------|------------|----------|-------|
| **Buildings** | | | |
| building_A1 (Office, 50m) | `Office Building` | (-80, -80, 0) | Tall office building |
| building_A2 (Apartments, 30m) | `Apartment` | (-80, 0, 0) | Medium apartment complex |
| building_A3 (House, 15m) | `House 1` | (-80, 80, 0) | Residential |
| building_B1_tower1/2 (Twin, 40m) | `Apartment` x2 | (0, -90/70, 0) | Rotated ±0.5 rad |
| building_B2 (Commercial, 25m) | `Gas Station` | (0, 0, 0) | Central commercial |
| hospital_building (35m) | `House 2` | (0, 80, 0) | NFZ location |
| building_C1 (High-rise, 45m) | `Office Building` | (80, -80, 0) | Rotated 90° |
| government_building (30m) | `Fire Station` | (80, 0, 0) | NFZ location |
| building_C3 (Residential, 20m) | `House 3` | (80, 80, 0) | Residential |
| building_small_1 | `Post Office` | (-40, -40, 0) | Small building |
| building_small_2 | `Fast Food` | (40, -40, 0) | Restaurant |
| building_small_3 | `Grocery Store` | (-40, 40, 0) | Store |
| building_small_4 | `Thrift Shop` | (40, 40, 0) | Shop |
| **Vehicles** | | | |
| vehicle_1 | `Hatchback blue` | (-50, -50, 0) | Street parked |
| vehicle_2 | `Hatchback red` | (30, -55, 0) | Street parked |
| vehicle_3 | `SUV` | (-55, 30, 0) | Street parked |
| vehicle_4 | `Pickup` | (55, 55, 0) | Street parked |
| vehicle_5 | `Prius Hybrid` | (-30, -70, 0) | Street parked |
| vehicle_6 | `Hatchback` | (70, -30, 0) | Street parked |
| truck_1 | `Bus` | (-30, 60, 0) | Delivery truck |
| **Infrastructure** | | | |
| lamp_post_1-4 | `Lamp Post` | ±60, ±60 | Street lighting |
| fire_hydrant_1-2 | `Fire Hydrant` | Various | Street details |
| telephone_pole_1-2 | `Telephone Pole` | ±100, ±50 | Utility poles |

### Collision Handling

Each Fuel model has an associated collision primitive for RL training performance. Collision sizes are calibrated to match actual Fuel model dimensions:

**Buildings & Structures** (box primitives):
```xml
<model name="building_A1_collision">
  <static>true</static>
  <pose>-80 -80 10 0 0 0</pose>  <!-- z = height/2 -->
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>15 12 20</size></box>  <!-- Matches Office Building footprint -->
      </geometry>
    </collision>
  </link>
</model>
```

**Trees** (trunk + canopy approach):
```xml
<model name="tree_collision">
  <static>true</static>
  <link name="link">
    <collision name="trunk">
      <pose>0 0 4 0 0 0</pose>
      <geometry>
        <cylinder><radius>0.4</radius><length>8</length></cylinder>
      </geometry>
    </collision>
    <collision name="canopy">
      <pose>0 0 10 0 0 0</pose>
      <geometry>
        <sphere><radius>3</radius></sphere>
      </geometry>
    </collision>
  </link>
</model>
```

**Vehicles** (box primitives matching actual dimensions):
```xml
<model name="vehicle_collision">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>4.5 1.8 1.5</size></box>  <!-- Standard car dimensions -->
      </geometry>
    </collision>
  </link>
</model>
```

This approach:
- **Visual**: Textured Fuel models for realistic camera images
- **Collision**: Simple primitives calibrated to actual model dimensions
- **Training**: Maintains RL training throughput while improving perception model domain transfer

### First Run - Model Download

On first run, Gazebo will download Fuel models from the internet. This requires:
1. Network access in the container (`--network host`)
2. Patience during initial download (~5-10 minutes depending on connection)
3. Models are cached in `~/.gz/fuel/` for subsequent runs

To pre-download models:
```bash
# Inside container
gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Office%20Building
gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Apartment
gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback%20blue
# ... etc
```

### Adding Custom Fuel Models

To add more Fuel models to a world:

#### Method 1: Include from Fuel URL (Recommended)
```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse</uri>
  <name>warehouse_1</name>
  <pose>-80 -60 0 0 0 0</pose>
  <static>true</static>
</include>
```

#### Method 2: Download and Include Locally
```bash
# Download model
gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse

# Reference in SDF
<include>
  <uri>model://Warehouse</uri>
  <pose>-80 -60 0 0 0 0</pose>
</include>
```

#### Method 3: Custom Meshes with PBR Materials
```xml
<visual name="building">
  <geometry>
    <mesh>
      <uri>file://models/my_building/meshes/building.dae</uri>
    </mesh>
  </geometry>
  <material>
    <pbr>
      <metal>
        <albedo_map>file://models/my_building/materials/brick.png</albedo_map>
        <normal_map>file://models/my_building/materials/brick_normal.png</normal_map>
        <roughness_map>file://models/my_building/materials/brick_roughness.png</roughness_map>
      </metal>
    </pbr>
  </material>
</visual>
```

### Recommended Fuel Models by World Type

**Urban:**
- `OpenRobotics/Office Building`, `OpenRobotics/Apartment`, `OpenRobotics/House 1/2/3`
- `OpenRobotics/Gas Station`, `OpenRobotics/Fire Station`, `OpenRobotics/Post Office`
- `OpenRobotics/Fast Food`, `OpenRobotics/Grocery Store`, `OpenRobotics/Thrift Shop`
- `OpenRobotics/Hatchback`, `OpenRobotics/Hatchback blue/red`, `OpenRobotics/SUV`, `OpenRobotics/Pickup`
- `OpenRobotics/Prius Hybrid`, `OpenRobotics/Bus`
- `OpenRobotics/Lamp Post`, `OpenRobotics/Fire Hydrant`, `OpenRobotics/Telephone Pole`

**Industrial:**
- `OpenRobotics/Warehouse`, `OpenRobotics/Shipping Container`, `OpenRobotics/Forklift`
- `OpenRobotics/Storage Tank`, `OpenRobotics/Jersey Barrier`, `OpenRobotics/Pallet`

**Coastal:**
- `OpenRobotics/Fishing Boat`, `OpenRobotics/Dock`, `OpenRobotics/Buoy`
- `OpenRobotics/Lighthouse`

**Rural:**
- `OpenRobotics/Oak Tree`, `OpenRobotics/Pine Tree`, `OpenRobotics/Rock`
- `OpenRobotics/Cabin`, `OpenRobotics/Fence`

### Upgrading Other Worlds

The same approach can be applied to other training worlds:
1. Identify appropriate Fuel models for the theme
2. Replace primitive `<model>` definitions with `<include>` from Fuel
3. Keep collision primitives for RL training performance
4. Test with `gz sdf -p world.sdf` to verify syntax

## Testing

```bash
# Test all worlds
python scripts/test_worlds.py --all --duration 30

# Test specific world
python scripts/test_worlds.py --world urban_training --duration 60
```

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `HEADLESS` | `false` | Run without GUI (uses Xvfb) |
| `SITL_SPEEDUP` | `1` | Simulation speed multiplier |
| `SITL_INSTANCE` | `0` | Instance number for parallel sims |
| `GAZEBO_ONLY` | `false` | Start only Gazebo (no SITL) |
| `SITL_ONLY` | `false` | Start only SITL (no Gazebo) |
| `LAUNCH_MAVROS` | `false` | Also start MAVROS bridge |

## Gazebo Commands Reference

```bash
# List topics
gz topic -l

# List models
gz model --list

# Move drone
gz service -s /world/flyby_urban/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --req 'name: "f11_isr", position: {x: 0, y: 0, z: 30}'

# Pause/unpause
gz service -s /world/flyby_urban/control \
    --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean \
    --req 'pause: false'
```

## ROS GZ Bridge

The ROS GZ Bridge connects Gazebo simulation topics to ROS 2:

```bash
ros2 launch flyby_f11_simulation ros_gz_bridge.launch.py
```

### Bridged Topics

| ROS 2 Topic | Direction | Message Type |
|-------------|-----------|--------------|
| `/f11/camera/image_raw` | GZ -> ROS | `sensor_msgs/msg/Image` |
| `/f11/camera/camera_info` | GZ -> ROS | `sensor_msgs/msg/CameraInfo` |
| `/f11/imu` | GZ -> ROS | `sensor_msgs/msg/Imu` |
| `/f11/gps` | GZ -> ROS | `sensor_msgs/msg/NavSatFix` |
| `/f11/gimbal/pitch_cmd` | ROS -> GZ | `std_msgs/msg/Float64` |
| `/f11/gimbal/yaw_cmd` | ROS -> GZ | `std_msgs/msg/Float64` |

## Performance Notes

- **Headless Mode**: Use `HEADLESS=true` for training (no GUI overhead)
- **Parallel Instances**: Support for multiple SITL instances via `SITL_INSTANCE`
- **Real-time Factor**: Target 2-4x real-time for training throughput
