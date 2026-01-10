# World Generator Architecture

This document describes the architecture of the `flyby.world_generator` extension, which provides procedural environment generation for ISR training.

## Design Principles

1. **Single Responsibility**: Each class has one clear purpose
2. **Composition over Inheritance**: The orchestrator composes specialized components
3. **Delegation**: The orchestrator delegates to components rather than implementing everything
4. **Maximum Class Size**: ~300 lines per class to maintain readability

## Component Structure

```
world_generator/
├── __init__.py              # Package exports
├── world_generator.py       # WorldGenerator orchestrator (~300 lines)
├── terrain_generator.py     # TerrainGenerator (~180 lines)
├── material_manager.py      # MaterialManager (~100 lines)
├── lighting_manager.py      # LightingManager (~220 lines)
├── atmosphere_controller.py # AtmosphereController (~160 lines)
├── extension.py             # Omniverse extension UI
├── ARCHITECTURE.md          # This file
├── spawners/
│   ├── base_spawner.py      # Base class for all spawners
│   ├── vegetation_spawner.py
│   ├── vehicle_spawner.py
│   ├── people_spawner.py
│   └── drone_spawner.py
└── drones/
    ├── f11_base.py          # F-11 drone base class
    ├── physics.py           # Drone physics simulation
    └── px4_bridge.py        # PX4 SITL integration
```

## Class Responsibilities

### WorldGenerator (Orchestrator)
**File**: `world_generator.py`
**Lines**: ~300
**Purpose**: High-level API for world generation

Responsibilities:
- Initialize and coordinate all components
- Provide unified API for environment generation
- Manage domain randomization across components
- Handle spawner coordination
- Aggregate statistics

Does NOT:
- Implement terrain mesh generation
- Configure PBR materials
- Setup lighting/shadows
- Apply fog effects

### TerrainGenerator
**File**: `terrain_generator.py`
**Lines**: ~180
**Purpose**: Generate terrain geometry

Responsibilities:
- Perlin noise heightfield generation
- Mesh vertex/triangle creation
- UV coordinate generation
- Collision geometry setup
- Physics material for terrain

### MaterialManager
**File**: `material_manager.py`
**Lines**: ~100
**Purpose**: Create and apply PBR materials

Responsibilities:
- PBR shader creation
- Material application to prims
- UV reader setup
- Material registry management

### LightingManager
**File**: `lighting_manager.py`
**Lines**: ~220
**Purpose**: Environment lighting setup

Responsibilities:
- HDRI dome light creation
- Sun/distant light setup
- Time-of-day presets
- Color temperature conversion
- Lighting randomization

### AtmosphereController
**File**: `atmosphere_controller.py`
**Lines**: ~160
**Purpose**: Weather and atmospheric effects

Responsibilities:
- Weather preset management
- RTX fog configuration
- Visibility control
- Light adjustment callbacks

## Class Composition

```
WorldGenerator
├── TerrainGenerator
├── MaterialManager
├── LightingManager ←──────┐
├── AtmosphereController ──┘ (callback for weather→lighting)
├── VegetationSpawner
├── VehicleSpawner
├── PeopleSpawner
└── DroneSpawner
```

## Adding New Features

### Adding a New Weather Preset
1. Add to `WEATHER_PRESETS` dict in `atmosphere_controller.py`
2. No changes needed elsewhere

### Adding a New Time-of-Day Preset
1. Add to `TIME_OF_DAY_PRESETS` dict in `lighting_manager.py`
2. No changes needed elsewhere

### Adding a New Material
1. Add to `DEFAULT_MATERIALS` dict in `material_manager.py`
2. Or call `material_manager.register_material()` at runtime

### Adding a New Terrain Type
1. Add method to `TerrainGenerator` (e.g., `generate_canyon()`)
2. Add delegation method to `WorldGenerator` if needed

### Adding a New Spawner
1. Create new spawner class extending `BaseSpawner`
2. Add to `spawners/` directory
3. Initialize in `WorldGenerator.__init__()`
4. Add delegation methods to `WorldGenerator`

## Guidelines

### Class Size Limits
- **Target**: ~200 lines per class
- **Maximum**: ~300 lines per class
- **If exceeding 300 lines**: Consider splitting responsibilities

### When to Split a Class
Split when a class:
- Exceeds 300 lines
- Has more than 10 public methods
- Has "and" in its description (does X and Y and Z)
- Mixes concerns (e.g., I/O + business logic + rendering)

### Naming Conventions
- `*Generator`: Creates USD geometry/prims
- `*Manager`: Manages reusable resources (materials, etc.)
- `*Controller`: Handles dynamic state/effects
- `*Spawner`: Places multiple instances of objects

## API Stability

The `WorldGenerator` class maintains backwards compatibility:
- All original public methods preserved
- Property accessors delegate to components
- Original attribute names work via properties

Internal components (`TerrainGenerator`, etc.) can be used directly for advanced customization but are considered lower-level API.
