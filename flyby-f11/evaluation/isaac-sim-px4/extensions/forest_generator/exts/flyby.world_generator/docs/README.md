# Flyby World Generator

Procedural world generation extension for ISR (Intelligence, Surveillance, Reconnaissance) training environments in NVIDIA Isaac Sim.

## Features

- **Terrain Generation**: Perlin noise-based terrain with configurable size and roughness
- **PBR Materials**: Realistic ground textures (forest floor, grass) with normal maps
- **HDRI Skybox**: High dynamic range environment lighting for photorealism
- **Vegetation**: Procedural forests with birch, spruce, and pine trees
- **Vehicles**: Spawn civilian and military vehicles
- **People**: Spawn groups, patrols, and crowds of pedestrians

## Installation

1. Copy the extension to your Isaac Sim extensions directory
2. Enable the extension from Window > Extensions
3. The "Flyby World Generator" window will appear

## Usage

### UI Mode

Use the extension window to:
1. Configure terrain size and roughness
2. Set HDRI path and lighting parameters
3. Adjust tree density and proportions
4. Click "Generate Full Environment" for one-click setup

### Programmatic API (for RL training)

```python
from flyby.world_generator import WorldGenerator, WorldConfig

# Configure world
config = WorldConfig(
    terrain_size=(500.0, 500.0),
    terrain_roughness=5.0,
    tree_density=3.0,
    randomize_lighting=True,
)

# Create generator
world_gen = WorldGenerator(
    models_path="/workspace/extensions/forest_generator/models",
    config=config,
)

# Generate full environment
world_gen.generate_full_environment()

# Or generate components separately
world_gen.generate_terrain()
world_gen.setup_lighting()
world_gen.generate_forest()

# Randomize for new episode
world_gen.randomize_environment(seed=42)

# Get stats
stats = world_gen.get_stats()
print(f"Trees: {stats['trees']}, Vehicles: {stats['vehicles']}")
```

### Adding Custom Models

#### Vehicles
```python
from flyby.world_generator.spawners import VehicleSpawner, VehicleConfig

# Register custom vehicle
world_gen.vehicles.register_custom_model(
    name="humvee",
    usd_path="/path/to/humvee.usd",
    category="military",
    scale=1.0,
)

# Spawn it
world_gen.vehicles.spawn_vehicle("humvee", position=(10, 20))
```

#### People
```python
# Register custom person model
world_gen.people.register_custom_model(
    name="soldier_standing",
    usd_path="/path/to/soldier.usd",
    category="military",
    pose="standing",
)

# Spawn patrol
world_gen.people.spawn_patrol(
    "soldier_standing",
    waypoints=[(0, 0), (50, 0), (50, 50)],
    count=4,
)
```

## Key Learnings from Development

### Tree Model Orientation
Tree models from the original forest generator are Y-up oriented. To display correctly in Isaac Sim (Z-up):
- Apply 90° rotation on X axis
- Apply 180° rotation on Y axis
- Scale factor of ~50x (base models are very small)

### HDRI Lighting
Use `UsdLux.DomeLight` with texture file attribute for HDRI:
```python
dome_light = UsdLux.DomeLight.Define(stage, path)
dome_light.CreateTextureFileAttr().Set(hdri_path)
```

### Ground Materials
Apply materials using `UsdShade`:
```python
material = UsdShade.Material.Define(stage, mat_path)
UsdShade.MaterialBindingAPI(terrain_prim).Bind(material)
```

## Available Assets

### Trees
- `Birch_obj/Birch.usd`
- `Spruce_obj/Spruce.usd`
- `Pine_obj/Pine.usd`

### Vegetation
- `Bush_obj/Bush.usd`
- `Blueberry_obj/Blueberry.usd`
- `Rock_obj/Rock.usd`

### Textures
- `forest_floor_diff_2k.jpg` - Ground diffuse
- `forest_floor_nor_gl_2k.exr` - Ground normal
- `autumn_park_2k.hdr` - HDRI skybox

## Credits

- Original forest generator by Joel Ventola (University of Oulu)
- Extended for ISR training by Finley Holt
