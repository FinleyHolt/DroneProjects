# Phase 6c: Diverse Training Worlds

## Overview

Create multiple diverse Gazebo Harmonic worlds to support domain randomization and prevent RL overfitting. Each world presents unique environmental characteristics, obstacle layouts, and operational challenges to ensure trained policies generalize across deployment scenarios.

## Human Description

A single training world leads to policies that memorize specific obstacle positions rather than learning general navigation skills. This phase creates 5 distinct worlds:

1. **Urban Environment** - Dense buildings, streets, vehicles, rooftop landing zones
2. **Rural/Forest Environment** - Trees, terrain elevation, natural obstacles, clearings
3. **Industrial Environment** - Warehouses, equipment, restricted zones, storage areas
4. **Coastal/Maritime Environment** - Water bodies, boats, piers, beach terrain
5. **Mixed/Randomized Environment** - Procedurally varied elements for domain randomization

Each world includes:
- Defined NFZ (No-Fly Zone) regions matching ontology format
- Multiple landing zones (primary, secondary, emergency)
- Static obstacles for avoidance training
- Consistent coordinate systems and spawn points
- World configuration files for dynamic loading

## AI Agent Instructions

### Prerequisites
- Phase 6a completed (F-11 drone model exists)
- Gazebo Harmonic (gz-sim8) installed in simulation container
- Understanding of SDF 1.9 world format
- Familiarity with SUMO/KIF ontology spatial predicates

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. Create World Directory Structure

```
simulation/worlds/
├── training_world.sdf          # Existing basic world
├── urban_training.sdf          # Phase 6c
├── rural_training.sdf          # Phase 6c
├── industrial_training.sdf     # Phase 6c
├── coastal_training.sdf        # Phase 6c
├── randomized_training.sdf     # Phase 6c
└── world_configs/
    ├── urban_config.yaml
    ├── rural_config.yaml
    ├── industrial_config.yaml
    ├── coastal_config.yaml
    └── randomized_config.yaml
```

#### 2. Urban Training World (urban_training.sdf)

**Features:**
- 300m x 300m operational area
- Grid of buildings (varying heights 10-50m)
- Street canyons for navigation challenges
- Parked vehicles at street level
- Rooftop landing zones
- 2-3 NFZ regions (helipads, private property)
- GPS-denied pockets (urban canyons)

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="flyby_urban">
    <!-- Physics optimized for training -->
    <physics name="training_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Standard plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <!-- Urban location coordinates -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>33.4484</latitude_deg>
      <longitude_deg>-112.0740</longitude_deg>
      <elevation>331</elevation>
    </spherical_coordinates>

    <!-- Ground: Asphalt/concrete texture -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Building grid (example - 3x3 block) -->
    <model name="building_block_A1">
      <static>true</static>
      <pose>-80 -80 15 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>30 30 30</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>30 30 30</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Additional buildings, vehicles, NFZ markers... -->
    <!-- Full implementation includes 15-20 buildings of varying sizes -->

    <!-- NFZ: Hospital helipad -->
    <model name="nfz_hospital">
      <static>true</static>
      <pose>50 -50 25 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>25</radius><length>50</length></cylinder></geometry>
          <material><ambient>1 0 0 0.3</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Landing zones on rooftops -->
    <model name="landing_zone_rooftop_1">
      <static>true</static>
      <pose>-80 -80 30.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>2</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>1 1 1 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- F-11 spawn point -->
    <include>
      <uri>/simulation/models/f11_isr_camera</uri>
      <name>f11_isr</name>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

  </world>
</sdf>
```

#### 3. Rural/Forest Training World (rural_training.sdf)

**Features:**
- 400m x 400m operational area with terrain variation
- Clustered tree groups (varying heights 5-20m)
- Terrain elevation changes (hills, valleys)
- Clearings suitable for landing
- Natural obstacles (rock formations, streams)
- Wildlife NFZ areas (nesting sites)
- Variable canopy density

**Key elements:**
- Heightmap for terrain variation
- Tree models with collision geometry
- Open meadow landing zones
- Protected wildlife area NFZ

#### 4. Industrial Training World (industrial_training.sdf)

**Features:**
- 350m x 350m operational area
- Large warehouse buildings (flat roofs)
- Stacked shipping containers
- Crane and equipment models
- Restricted chemical storage NFZ
- Loading dock activity areas
- Multiple emergency landing options

**Key elements:**
- Regular warehouse grid layout
- Container stacks as complex obstacles
- Restricted zone markers (red boundaries)
- Ground-level and elevated landing zones

#### 5. Coastal/Maritime Training World (coastal_training.sdf)

**Features:**
- 400m x 400m area (50% water, 50% land)
- Pier and dock structures
- Boat models (static and moving)
- Beach terrain transition
- Harbor NFZ (shipping lane)
- Water landing zones (platform)
- Wind exposure simulation setup

**Key elements:**
- Water plane with visual differentiation
- Pier collision geometry
- Boat obstacles at various positions
- Floating platform landing zones

#### 6. Randomized Training World (randomized_training.sdf)

**Features:**
- Procedural obstacle placement via SDF parameters
- Variable obstacle density configuration
- Randomized NFZ positions and sizes
- Configurable terrain features
- Seeds for reproducibility

**Implementation approach:**
- Use SDF `<include>` with pose variations
- World config YAML specifies randomization bounds
- Python script generates SDF variants at training time

#### 7. Create World Configuration Files

Each world needs a YAML config for the RL training system:

```yaml
# world_configs/urban_config.yaml
world:
  name: "flyby_urban"
  sdf_file: "urban_training.sdf"
  dimensions:
    x_min: -150
    x_max: 150
    y_min: -150
    y_max: 150
    z_min: 0
    z_max: 100

spawn_points:
  - name: "default"
    pose: [0, 0, 0.5, 0, 0, 0]
  - name: "rooftop_start"
    pose: [-80, -80, 31, 0, 0, 0]
  - name: "street_level"
    pose: [20, 0, 1, 0, 0, 0]

landing_zones:
  - name: "ground_primary"
    position: [0, 0, 0]
    radius: 3.0
  - name: "rooftop_A1"
    position: [-80, -80, 30]
    radius: 2.0
  - name: "emergency_park"
    position: [100, 100, 0]
    radius: 5.0

no_fly_zones:
  - name: "hospital_helipad"
    type: "cylinder"
    center: [50, -50, 0]
    radius: 25
    height: 50
  - name: "government_building"
    type: "box"
    min: [80, 40, 0]
    max: [120, 80, 60]

obstacles:
  buildings:
    count: 20
    height_range: [10, 50]
  vehicles:
    count: 15
    positions: "street_level"

training_parameters:
  episode_timeout: 300  # seconds
  max_altitude: 80  # meters
  min_obstacle_distance: 3.0
  reward_weights:
    goal_reached: 100.0
    nfz_violation: -200.0
    collision: -500.0
    time_penalty: -0.1
```

#### 8. Create World Loader Utility

```python
# ros2_ws/src/ontology_rl/ontology_rl/utils/world_loader.py
"""
Utility to load and configure training worlds dynamically.
"""
import yaml
import os
from typing import Dict, List, Optional
from dataclasses import dataclass

@dataclass
class WorldConfig:
    """Configuration for a training world."""
    name: str
    sdf_file: str
    dimensions: Dict[str, float]
    spawn_points: List[Dict]
    landing_zones: List[Dict]
    no_fly_zones: List[Dict]
    obstacles: Dict
    training_parameters: Dict

class WorldLoader:
    """Loads and manages training world configurations."""

    def __init__(self, config_dir: str = None):
        self.config_dir = config_dir or os.path.join(
            os.path.dirname(__file__),
            '../../../../simulation/worlds/world_configs'
        )
        self.worlds: Dict[str, WorldConfig] = {}
        self._load_all_configs()

    def _load_all_configs(self):
        """Load all world configuration files."""
        for filename in os.listdir(self.config_dir):
            if filename.endswith('_config.yaml'):
                world_name = filename.replace('_config.yaml', '')
                config_path = os.path.join(self.config_dir, filename)
                with open(config_path, 'r') as f:
                    config_data = yaml.safe_load(f)
                self.worlds[world_name] = self._parse_config(config_data)

    def _parse_config(self, data: dict) -> WorldConfig:
        """Parse YAML config into WorldConfig dataclass."""
        return WorldConfig(
            name=data['world']['name'],
            sdf_file=data['world']['sdf_file'],
            dimensions=data['world']['dimensions'],
            spawn_points=data.get('spawn_points', []),
            landing_zones=data.get('landing_zones', []),
            no_fly_zones=data.get('no_fly_zones', []),
            obstacles=data.get('obstacles', {}),
            training_parameters=data.get('training_parameters', {})
        )

    def get_world(self, name: str) -> Optional[WorldConfig]:
        """Get configuration for a specific world."""
        return self.worlds.get(name)

    def list_worlds(self) -> List[str]:
        """List all available training worlds."""
        return list(self.worlds.keys())

    def get_random_spawn(self, world_name: str) -> List[float]:
        """Get a random spawn point for the given world."""
        import random
        world = self.worlds.get(world_name)
        if world and world.spawn_points:
            spawn = random.choice(world.spawn_points)
            return spawn['pose']
        return [0, 0, 0.5, 0, 0, 0]  # Default

    def nfz_to_tptp(self, world_name: str) -> List[str]:
        """Convert NFZ definitions to TPTP facts for Vampire."""
        world = self.worlds.get(world_name)
        if not world:
            return []

        facts = []
        for nfz in world.no_fly_zones:
            name = nfz['name'].replace(' ', '_').lower()
            if nfz['type'] == 'cylinder':
                # fof(nfz_def, axiom, noFlyZone(nfz_name, cylinder, cx, cy, r, h))
                cx, cy, _ = nfz['center']
                r = nfz['radius']
                h = nfz['height']
                facts.append(
                    f"fof(nfz_{name}, axiom, "
                    f"noFlyZone({name}, cylinder, {cx}, {cy}, {r}, {h}))."
                )
            elif nfz['type'] == 'box':
                # fof(nfz_def, axiom, noFlyZone(nfz_name, box, xmin, ymin, xmax, ymax, zmax))
                xmin, ymin, zmin = nfz['min']
                xmax, ymax, zmax = nfz['max']
                facts.append(
                    f"fof(nfz_{name}, axiom, "
                    f"noFlyZone({name}, box, {xmin}, {ymin}, {xmax}, {ymax}, {zmax}))."
                )
        return facts
```

#### 9. Update ontology_rl Package for Multi-World Support

Modify the base environment to accept world configuration:

```python
# ontology_rl/envs/base_env.py (additions)

from ontology_rl.utils.world_loader import WorldLoader

class FlybyBaseEnv(gym.Env):
    def __init__(self, config: dict = None):
        # ... existing init ...

        # World configuration
        self.world_loader = WorldLoader()
        self.world_name = self.config.get('world', 'training')
        self.world_config = self.world_loader.get_world(self.world_name)

        # Load NFZ facts for Vampire
        if self.world_config:
            self._nfz_facts = self.world_loader.nfz_to_tptp(self.world_name)
```

#### 10. Create Domain Randomization Script

```python
# scripts/randomize_world.py
"""
Generate randomized world variants for domain randomization training.
"""
import argparse
import random
import yaml
from pathlib import Path

def randomize_obstacles(base_config: dict, seed: int) -> dict:
    """Randomize obstacle positions within bounds."""
    random.seed(seed)
    config = base_config.copy()

    dims = config['world']['dimensions']
    x_range = (dims['x_min'] * 0.8, dims['x_max'] * 0.8)
    y_range = (dims['y_min'] * 0.8, dims['y_max'] * 0.8)

    # Randomize spawn points
    for spawn in config.get('spawn_points', []):
        if spawn['name'] != 'default':
            spawn['pose'][0] = random.uniform(*x_range)
            spawn['pose'][1] = random.uniform(*y_range)

    # Randomize NFZ positions (within bounds)
    for nfz in config.get('no_fly_zones', []):
        if nfz['type'] == 'cylinder':
            nfz['center'][0] = random.uniform(*x_range)
            nfz['center'][1] = random.uniform(*y_range)
            nfz['radius'] = random.uniform(15, 35)

    return config

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--base-config', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--num-variants', type=int, default=10)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    with open(args.base_config) as f:
        base_config = yaml.safe_load(f)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    for i in range(args.num_variants):
        variant = randomize_obstacles(base_config, args.seed + i)
        variant_path = output_dir / f"randomized_v{i:03d}_config.yaml"
        with open(variant_path, 'w') as f:
            yaml.dump(variant, f)
        print(f"Generated: {variant_path}")

if __name__ == '__main__':
    main()
```

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] 5 distinct world SDF files created and valid
- [ ] Each world loads in Gazebo Harmonic without errors
- [ ] All worlds have defined geofence boundaries
- [ ] Each world has 2+ NFZ regions with visual markers
- [ ] Each world has 3+ landing zones defined
- [ ] World configuration YAML files created for all worlds
- [ ] WorldLoader utility successfully parses all configs
- [ ] NFZ-to-TPTP conversion produces valid facts
- [ ] Domain randomization script generates variants
- [ ] F-11 drone spawns correctly in each world

### Verification

Run automated verification:
```bash
bash .phases/phase-06c-diverse-training-worlds/verification.sh
```

### Common Pitfalls

- **SDF version**: Use SDF 1.9 for Gazebo Harmonic compatibility
- **Coordinate consistency**: All worlds use ENU frame, origin at center
- **Collision geometry**: Every visual obstacle needs collision for physics
- **NFZ alignment**: World NFZ must match YAML config exactly
- **Spawn altitude**: Ensure spawn Z > 0 to avoid ground collision
- **Model paths**: Use relative paths from simulation/ directory

### References

- [Gazebo Harmonic SDF Format](https://gazebosim.org/docs/harmonic/sdf_worlds)
- [Domain Randomization for Sim-to-Real](https://arxiv.org/abs/1703.06907)
- [Training World (existing)](../../simulation/worlds/training_world.sdf)
- [UAV Domain Ontology](../../ontology/planning_mode/uav_domain.kif)

### Dependencies
See `dependencies.json` - requires Phase 6a completion.

### Next Phase
After completion, proceed to Phase 6d: Perception Pipeline
