"""
World Loader Utility for Flyby F-11 Training Environments

Provides functionality to:
- Load and parse world configuration YAML files
- Convert No-Fly Zone (NFZ) definitions to TPTP facts for Vampire theorem prover
- Manage multiple training worlds for domain randomization
- Select random spawn points and landing zones

Author: Finley Holt
"""

import os
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

import yaml


@dataclass
class NFZDefinition:
    """Definition of a No-Fly Zone."""
    name: str
    nfz_type: str  # 'cylinder' or 'box'
    center: Optional[List[float]] = None  # For cylinder: [x, y, z]
    radius: Optional[float] = None  # For cylinder
    height: Optional[float] = None  # For cylinder
    min_bounds: Optional[List[float]] = None  # For box: [x_min, y_min, z_min]
    max_bounds: Optional[List[float]] = None  # For box: [x_max, y_max, z_max]
    reason: str = ""
    severity: str = "high"


@dataclass
class LandingZone:
    """Definition of a landing zone."""
    name: str
    position: List[float]
    radius: float
    zone_type: str = "primary"
    surface: str = "concrete"


@dataclass
class SpawnPoint:
    """Definition of a spawn point."""
    name: str
    pose: List[float]  # [x, y, z, roll, pitch, yaw]
    description: str = ""


@dataclass
class WorldConfig:
    """Configuration for a training world."""
    name: str
    sdf_file: str
    description: str
    dimensions: Dict[str, float]
    spawn_points: List[SpawnPoint]
    landing_zones: List[LandingZone]
    no_fly_zones: List[NFZDefinition]
    obstacles: Dict[str, Any]
    training_parameters: Dict[str, Any]
    coordinates: Optional[Dict[str, float]] = None
    environment: Optional[Dict[str, Any]] = None


class WorldLoader:
    """
    Loads and manages training world configurations.

    Provides methods to:
    - Load all world configs from a directory
    - Get specific world configuration
    - Convert NFZ definitions to TPTP format for Vampire
    - Select random spawn points
    """

    def __init__(self, config_dir: Optional[str] = None):
        """
        Initialize the WorldLoader.

        Args:
            config_dir: Path to directory containing world config YAML files.
                       Defaults to simulation/worlds/world_configs relative to project.
        """
        if config_dir is None:
            # Default path relative to this file
            this_file = Path(__file__)
            project_root = this_file.parent.parent.parent.parent.parent.parent
            config_dir = project_root / "simulation" / "worlds" / "world_configs"

        self.config_dir = Path(config_dir)
        self.worlds: Dict[str, WorldConfig] = {}
        self._load_all_configs()

    def _load_all_configs(self) -> None:
        """Load all world configuration files from config directory."""
        if not self.config_dir.exists():
            return

        for config_file in self.config_dir.glob("*_config.yaml"):
            world_name = config_file.stem.replace("_config", "")
            try:
                with open(config_file, 'r') as f:
                    config_data = yaml.safe_load(f)
                self.worlds[world_name] = self._parse_config(config_data)
            except Exception as e:
                print(f"Warning: Failed to load {config_file}: {e}")

    def _parse_nfz(self, nfz_data: Dict) -> NFZDefinition:
        """Parse NFZ data from YAML into NFZDefinition."""
        nfz_type = nfz_data.get('type', 'cylinder')

        if nfz_type == 'cylinder':
            return NFZDefinition(
                name=nfz_data['name'],
                nfz_type='cylinder',
                center=nfz_data.get('center', [0, 0, 0]),
                radius=nfz_data.get('radius', 10),
                height=nfz_data.get('height', 30),
                reason=nfz_data.get('reason', ''),
                severity=nfz_data.get('severity', 'high')
            )
        elif nfz_type == 'box':
            return NFZDefinition(
                name=nfz_data['name'],
                nfz_type='box',
                min_bounds=nfz_data.get('min', [0, 0, 0]),
                max_bounds=nfz_data.get('max', [10, 10, 30]),
                reason=nfz_data.get('reason', ''),
                severity=nfz_data.get('severity', 'high')
            )
        else:
            raise ValueError(f"Unknown NFZ type: {nfz_type}")

    def _parse_landing_zone(self, lz_data: Dict) -> LandingZone:
        """Parse landing zone data from YAML."""
        return LandingZone(
            name=lz_data['name'],
            position=lz_data['position'],
            radius=lz_data.get('radius', 3.0),
            zone_type=lz_data.get('type', 'primary'),
            surface=lz_data.get('surface', 'concrete')
        )

    def _parse_spawn_point(self, sp_data: Dict) -> SpawnPoint:
        """Parse spawn point data from YAML."""
        return SpawnPoint(
            name=sp_data['name'],
            pose=sp_data['pose'],
            description=sp_data.get('description', '')
        )

    def _parse_config(self, data: Dict) -> WorldConfig:
        """Parse YAML config data into WorldConfig dataclass."""
        world_data = data.get('world', {})

        # Parse spawn points
        spawn_points = [
            self._parse_spawn_point(sp)
            for sp in data.get('spawn_points', [])
        ]

        # Parse landing zones
        landing_zones = [
            self._parse_landing_zone(lz)
            for lz in data.get('landing_zones', [])
        ]

        # Parse no-fly zones
        no_fly_zones = [
            self._parse_nfz(nfz)
            for nfz in data.get('no_fly_zones', [])
        ]

        return WorldConfig(
            name=world_data.get('name', 'unknown'),
            sdf_file=world_data.get('sdf_file', ''),
            description=world_data.get('description', ''),
            dimensions=world_data.get('dimensions', {}),
            spawn_points=spawn_points,
            landing_zones=landing_zones,
            no_fly_zones=no_fly_zones,
            obstacles=data.get('obstacles', {}),
            training_parameters=data.get('training_parameters', {}),
            coordinates=data.get('coordinates'),
            environment=data.get('environment')
        )

    def get_world(self, name: str) -> Optional[WorldConfig]:
        """
        Get configuration for a specific world.

        Args:
            name: World name (e.g., 'urban', 'rural', 'industrial')

        Returns:
            WorldConfig or None if not found
        """
        return self.worlds.get(name)

    def list_worlds(self) -> List[str]:
        """
        List all available training worlds.

        Returns:
            List of world names
        """
        return list(self.worlds.keys())

    def get_random_spawn(self, world_name: str, seed: Optional[int] = None) -> List[float]:
        """
        Get a random spawn point for the given world.

        Args:
            world_name: Name of the world
            seed: Optional random seed for reproducibility

        Returns:
            Spawn pose [x, y, z, roll, pitch, yaw]
        """
        if seed is not None:
            random.seed(seed)

        world = self.worlds.get(world_name)
        if world and world.spawn_points:
            spawn = random.choice(world.spawn_points)
            return spawn.pose
        return [0, 0, 0.5, 0, 0, 0]  # Default spawn

    def get_random_landing_zone(
        self,
        world_name: str,
        exclude_emergency: bool = True,
        seed: Optional[int] = None
    ) -> Optional[LandingZone]:
        """
        Get a random landing zone for the given world.

        Args:
            world_name: Name of the world
            exclude_emergency: If True, exclude emergency landing zones
            seed: Optional random seed

        Returns:
            LandingZone or None
        """
        if seed is not None:
            random.seed(seed)

        world = self.worlds.get(world_name)
        if not world or not world.landing_zones:
            return None

        zones = world.landing_zones
        if exclude_emergency:
            zones = [lz for lz in zones if lz.zone_type != 'emergency']

        if not zones:
            return None

        return random.choice(zones)

    def nfz_to_tptp(self, world_name: str) -> List[str]:
        """
        Convert NFZ definitions to TPTP facts for Vampire theorem prover.

        The generated TPTP facts define No-Fly Zones that can be used
        by Vampire to verify safety constraints during mission planning.

        Args:
            world_name: Name of the world

        Returns:
            List of TPTP fact strings
        """
        world = self.worlds.get(world_name)
        if not world:
            return []

        facts = []

        for nfz in world.no_fly_zones:
            # Sanitize name for TPTP (lowercase, underscores)
            safe_name = nfz.name.replace(' ', '_').replace('-', '_').lower()

            if nfz.nfz_type == 'cylinder':
                # Cylindrical NFZ: noFlyZoneCylinder(name, cx, cy, radius, height)
                cx, cy, cz = nfz.center[0], nfz.center[1], nfz.center[2]
                fact = (
                    f"fof(nfz_{safe_name}, axiom, "
                    f"noFlyZoneCylinder({safe_name}, {cx}, {cy}, {nfz.radius}, {nfz.height}))."
                )
                facts.append(fact)

                # Add base position fact
                base_fact = (
                    f"fof(nfz_{safe_name}_base, axiom, "
                    f"nfzBasePosition({safe_name}, {cx}, {cy}, {cz}))."
                )
                facts.append(base_fact)

            elif nfz.nfz_type == 'box':
                # Box NFZ: noFlyZoneBox(name, xmin, ymin, zmin, xmax, ymax, zmax)
                xmin, ymin, zmin = nfz.min_bounds
                xmax, ymax, zmax = nfz.max_bounds
                fact = (
                    f"fof(nfz_{safe_name}, axiom, "
                    f"noFlyZoneBox({safe_name}, {xmin}, {ymin}, {zmin}, "
                    f"{xmax}, {ymax}, {zmax}))."
                )
                facts.append(fact)

            # Add severity classification
            severity_fact = (
                f"fof(nfz_{safe_name}_severity, axiom, "
                f"nfzSeverity({safe_name}, {nfz.severity}))."
            )
            facts.append(severity_fact)

        return facts

    def geofence_to_tptp(self, world_name: str) -> List[str]:
        """
        Convert geofence dimensions to TPTP facts.

        Args:
            world_name: Name of the world

        Returns:
            List of TPTP fact strings defining the geofence
        """
        world = self.worlds.get(world_name)
        if not world or not world.dimensions:
            return []

        dims = world.dimensions
        facts = [
            f"fof(geofence_{world_name}, axiom, "
            f"geofenceBounds({world_name}_geofence, "
            f"{dims.get('x_min', -100)}, {dims.get('y_min', -100)}, {dims.get('z_min', 0)}, "
            f"{dims.get('x_max', 100)}, {dims.get('y_max', 100)}, {dims.get('z_max', 100)}))."
        ]

        return facts

    def landing_zones_to_tptp(self, world_name: str) -> List[str]:
        """
        Convert landing zone definitions to TPTP facts.

        Args:
            world_name: Name of the world

        Returns:
            List of TPTP fact strings
        """
        world = self.worlds.get(world_name)
        if not world:
            return []

        facts = []
        for lz in world.landing_zones:
            safe_name = lz.name.replace(' ', '_').replace('-', '_').lower()
            x, y, z = lz.position

            fact = (
                f"fof(lz_{safe_name}, axiom, "
                f"landingZone({safe_name}, {x}, {y}, {z}, {lz.radius}))."
            )
            facts.append(fact)

            # Add type classification
            type_fact = (
                f"fof(lz_{safe_name}_type, axiom, "
                f"landingZoneType({safe_name}, {lz.zone_type}))."
            )
            facts.append(type_fact)

        return facts

    def get_world_tptp_facts(self, world_name: str) -> str:
        """
        Get all TPTP facts for a world (NFZ, geofence, landing zones).

        Args:
            world_name: Name of the world

        Returns:
            Combined TPTP facts as a single string
        """
        nfz_facts = self.nfz_to_tptp(world_name)
        geofence_facts = self.geofence_to_tptp(world_name)
        lz_facts = self.landing_zones_to_tptp(world_name)

        all_facts = (
            [f"% No-Fly Zones for {world_name}"] +
            nfz_facts +
            ["", f"% Geofence for {world_name}"] +
            geofence_facts +
            ["", f"% Landing Zones for {world_name}"] +
            lz_facts
        )

        return "\n".join(all_facts)

    def get_training_params(self, world_name: str) -> Dict[str, Any]:
        """
        Get training parameters for a world.

        Args:
            world_name: Name of the world

        Returns:
            Dictionary of training parameters
        """
        world = self.worlds.get(world_name)
        if world:
            return world.training_parameters
        return {}

    def get_sdf_path(self, world_name: str) -> Optional[Path]:
        """
        Get the full path to the SDF file for a world.

        Args:
            world_name: Name of the world

        Returns:
            Path to SDF file or None
        """
        world = self.worlds.get(world_name)
        if not world:
            return None

        # SDF files are in the parent directory of config
        sdf_dir = self.config_dir.parent
        sdf_path = sdf_dir / world.sdf_file

        if sdf_path.exists():
            return sdf_path
        return None


def main():
    """Test the WorldLoader functionality."""
    loader = WorldLoader()

    print("Available worlds:", loader.list_worlds())

    for world_name in loader.list_worlds():
        print(f"\n=== {world_name.upper()} ===")
        world = loader.get_world(world_name)
        if world:
            print(f"  SDF: {world.sdf_file}")
            print(f"  Description: {world.description}")
            print(f"  Dimensions: {world.dimensions}")
            print(f"  Spawn points: {len(world.spawn_points)}")
            print(f"  Landing zones: {len(world.landing_zones)}")
            print(f"  No-fly zones: {len(world.no_fly_zones)}")

            print("\n  TPTP Facts:")
            tptp = loader.get_world_tptp_facts(world_name)
            for line in tptp.split('\n')[:10]:
                print(f"    {line}")
            if len(tptp.split('\n')) > 10:
                print("    ...")


if __name__ == '__main__':
    main()
