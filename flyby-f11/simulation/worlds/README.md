# Flyby F-11 Gazebo Training Worlds

This directory contains Gazebo Harmonic (SDF 1.9+) world files for UAV reinforcement learning training.

## Directory Structure

```
worlds/
├── training_world.sdf           # Original v1 training world (baseline)
├── training_world_config.json   # Configuration for v1 world
├── rl_training_v2.sdf          # Enhanced v2 training world
├── generated/                   # Procedurally generated variants
│   ├── variant_000_*.sdf
│   ├── variant_000_*.json
│   └── ...
└── README.md                    # This file
```

## Available Worlds

### training_world.sdf (v1)

Basic training environment for initial algorithm development.

**Features:**
- 250m x 250m operational area
- 2 no-fly zones (cylindrical and rectangular)
- 7 static obstacles
- 3 landing zones
- Camp Pendleton GPS coordinates

**Use Case:** Initial testing, baseline comparisons

### rl_training_v2.sdf (v2)

Enhanced training environment with diverse obstacle types and zones.

**Features:**
- 300m x 300m operational area
- 3 no-fly zones with varied types
- 5 distinct obstacle zones:
  - Zone A (NE): Dense urban buildings with overhangs
  - Zone B (NW): Pole forest for precision navigation
  - Zone C (SW): Forest with tree canopies
  - Zone D (SE): Industrial area with tanks and pipe bridges
  - Zone E (Center): Mixed obstacles near origin
- 4 landing zones (primary, secondary, emergency, forward)
- Wind effects system enabled
- Waypoint markers for visual reference
- Optional fog particle emitter (commented out)

**Use Case:** Main training environment, curriculum learning

### Generated Variants

Procedurally generated worlds for domain randomization.

**Features:**
- Reproducible generation with seeds
- Configurable difficulty (easy, medium, hard, extreme)
- Lighting variations (day, dusk, overcast, night)
- Wind variations
- Randomized obstacle placement

**Use Case:** Domain randomization, generalization testing

## Usage

### Loading a World in Gazebo

```bash
# From container
gz sim worlds/rl_training_v2.sdf

# Or with ROS 2 launch
ros2 launch flyby_f11_simulation training.launch.py world:=rl_training_v2.sdf
```

### Generating World Variants

```bash
# Generate 10 medium difficulty variants
python scripts/generate_world_variants.py --num-worlds 10

# Generate hard difficulty with specific seed
python scripts/generate_world_variants.py --difficulty hard --seed 12345

# Generate to custom directory
python scripts/generate_world_variants.py --output-dir ./custom_worlds --num-worlds 5

# Full options
python scripts/generate_world_variants.py \
    --num-worlds 20 \
    --seed 42 \
    --difficulty hard \
    --geofence-size 400 \
    --no-lighting-variation
```

### Generator Options

| Option | Default | Description |
|--------|---------|-------------|
| `--num-worlds` | 10 | Number of variants to generate |
| `--seed` | 42 | Base random seed |
| `--difficulty` | medium | easy, medium, hard, extreme |
| `--output-dir` | worlds/generated | Output directory |
| `--geofence-size` | 300 | Geofence size in meters |
| `--no-lighting-variation` | false | Use only 'day' lighting |
| `--no-wind-variation` | false | Enable wind in all worlds |

### Difficulty Presets

| Preset | Obstacles | NFZs | Max Height | Narrow Passages |
|--------|-----------|------|------------|-----------------|
| easy | 5-10 | 1-2 | 15m | No |
| medium | 15-25 | 2-3 | 25m | Yes |
| hard | 30-45 | 3-5 | 35m | Yes |
| extreme | 50-70 | 4-6 | 45m | Yes |

## World Coordinate System

All worlds use ENU (East-North-Up) coordinate frame:
- **Origin (0, 0, 0)**: Center of training area, primary landing zone
- **X-axis**: East (positive)
- **Y-axis**: North (positive)
- **Z-axis**: Up (positive)

### Geofence Boundaries

| World | X Range | Y Range | Z Range |
|-------|---------|---------|---------|
| v1 | -125 to 125 | -125 to 125 | 0 to 100 |
| v2 | -150 to 150 | -150 to 150 | 0 to 100 |

## Obstacle Types

### Cylindrical (Poles, Towers)
- Thin vertical obstacles requiring precision avoidance
- Varying heights (15-45m) and radii (0.3-2m)
- Used in pole forests and as standalone obstacles

### Box (Buildings, Walls)
- Rectangular obstacles with rotation
- Variable dimensions for path planning challenges
- Include L-shaped and overhang variants

### Trees
- Trunk + canopy structure
- Collision volume simplified to cylinder
- Varied sizes for natural environment simulation

### Industrial
- Storage tanks (cylindrical)
- Pipe bridges (elevated)
- Loading docks with overhangs

## No-Fly Zones

NFZs are rendered as translucent red volumes:
- **Cylindrical**: Tower/communication exclusion zones
- **Rectangular**: Facility/restricted area zones
- **Priority levels**: 1 (low) to 3 (high) - affects visual intensity

NFZ enforcement is handled by the ontology layer, not Gazebo physics.

## Landing Zones

| Type | Color | Marker | Purpose |
|------|-------|--------|---------|
| Primary | White | Blue H | Main launch/landing |
| Secondary | Light gray | Green circle | Alternate landing |
| Emergency | Orange | White cross | Emergency landing |
| Forward | Gray | None | Forward operating point |

## Physics Configuration

Optimized for RL training throughput:

```xml
<physics name="training_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

**Performance Tips:**
- Disable shadows for faster training (`<cast_shadows>false</cast_shadows>`)
- Set `real_time_update_rate` to 0 for maximum speed
- Use headless mode: `gz sim --headless-rendering`

## Weather Effects

### Wind

Wind is configured globally and affects UAV dynamics:

```xml
<wind>
  <linear_velocity>3 1 0</linear_velocity>
</wind>
```

The wind effects system plugin applies forces to the UAV model.

### Fog (Optional)

Fog can be enabled by uncommenting the fog_emitter model in v2 world:

```xml
<particle_emitter name="fog" type="box">
  <emitting>true</emitting>
  <size>200 200 20</size>
  <particle_scatter_ratio>0.3</particle_scatter_ratio>
</particle_emitter>
```

Fog affects camera and LiDAR sensors with configurable scatter ratio.

## Lighting Presets

| Preset | Sun Direction | Ambient | Use Case |
|--------|--------------|---------|----------|
| day | -0.5, 0.1, -0.9 | Bright | Standard training |
| dusk | -0.1, 0.3, -0.3 | Warm/dim | Low-light testing |
| overcast | -0.3, 0.0, -0.95 | Diffuse | Shadow-free |
| night | 0, 0, -1 | Very dim | Night ops |

## Integration with Ontology

The world configuration exports to the ontology layer:

```
World Elements → training_world_config.json → StateGroundingNode → Vampire Prover
```

**Exported Properties:**
- Geofence boundaries
- NFZ positions and dimensions
- Landing zone locations
- Obstacle collision volumes

## Customization

### Adding New Obstacles

1. Edit SDF file or use generator API
2. Ensure collision geometry matches visual
3. Add to ontology configuration if needed

### Creating Custom Presets

Edit `DIFFICULTY_PRESETS` in `generate_world_variants.py`:

```python
DIFFICULTY_PRESETS["custom"] = {
    "num_obstacles": (20, 30),
    "num_nfz": (2, 4),
    "obstacle_density": 0.6,
    "max_obstacle_height": 30,
    "narrow_passages": True,
}
```

### GPS Coordinates

Default location is Camp Pendleton (MCTSSA operations):
- Latitude: 33.3853
- Longitude: -117.5653
- Elevation: 100m

Modify `<spherical_coordinates>` for different locations.

## Troubleshooting

### World fails to load

```bash
# Check SDF syntax
gz sdf -k worlds/rl_training_v2.sdf
```

### Model not found

Ensure model path is set:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models
```

### Performance issues

- Reduce obstacle count
- Disable shadows
- Use simpler collision geometry
- Run headless: `gz sim --headless-rendering`

## References

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted/)
- [SDF Specification](http://sdformat.org/spec)
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [Domain Randomization Best Practices](../docs/research/world_environments.md)
