"""
World Generator - Main API for procedural environment generation.

Provides a unified interface for:
- Terrain generation with materials
- HDRI skybox/lighting
- Vegetation, vehicles, and people spawning
- Domain randomization for RL training

This is an orchestrator class that composes specialized components:
- TerrainGenerator: Heightfield and mesh generation
- MaterialManager: PBR material creation and application
- LightingManager: HDRI skybox and sun lighting
- AtmosphereController: Weather and fog effects
- Spawners: Vegetation, vehicles, people, drones
"""

# Standard library
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

# Third-party
import numpy as np

# Isaac Sim / Omniverse
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.core.utils.stage import get_current_stage

# Local - extracted components
from .terrain_generator import TerrainGenerator
from .material_manager import MaterialManager
from .lighting_manager import LightingManager
from .atmosphere_controller import AtmosphereController, WeatherConfig, WEATHER_PRESETS

# Local - spawners
from .spawners.base_spawner import BaseSpawner, SpawnConfig
from .spawners.drone_spawner import DroneSpawner
from .spawners.people_spawner import PeopleSpawner
from .spawners.vehicle_spawner import VehicleSpawner
from .spawners.vegetation_spawner import VegetationSpawner


# Re-export for backwards compatibility
__all__ = [
    "WorldConfig",
    "WorldGenerator",
    "WeatherConfig",
    "WEATHER_PRESETS",
]


@dataclass
class WorldConfig:
    """Configuration for world generation."""
    # Terrain
    terrain_size: Tuple[float, float] = (200.0, 200.0)
    terrain_roughness: float = 3.0
    terrain_material: str = "forest_floor"

    # Lighting/Sky
    hdri_path: str = None
    sun_intensity: float = 3000.0
    sun_angle: Tuple[float, float] = (-45.0, 30.0)
    time_of_day: Optional[str] = None  # "dawn", "morning", "noon", "afternoon", "dusk", "overcast"

    # Weather/Atmosphere
    weather: Optional[str] = None  # "clear", "light_haze", "moderate_haze", "heavy_haze"

    # Vegetation (lower defaults for performance)
    tree_density: float = 1.5  # Trees per 100m² (reduced from 3.0)
    tree_proportions: Dict[str, float] = field(default_factory=lambda: {
        "Birch": 30, "Spruce": 40, "Pine": 30
    })
    undergrowth_density: float = 0.0  # Disabled by default for performance

    # Domain randomization
    randomize_lighting: bool = True
    randomize_weather: bool = False
    randomize_terrain: bool = True

    # Spawning
    seed: Optional[int] = None

    # Drone spawn exclusion zone - prevents objects from spawning at drone's spawn point
    # Format: (x, y, radius) or None to disable
    # This is registered BEFORE any spawning occurs to ensure nothing spawns under the drone
    drone_spawn_exclusion: Optional[Tuple[float, float, float]] = (0.0, 0.0, 10.0)


class WorldGenerator:
    """
    Main orchestrator for procedural world generation.

    Composes specialized components:
    - TerrainGenerator: Terrain mesh and collision
    - MaterialManager: PBR materials
    - LightingManager: Sky and sun lighting
    - AtmosphereController: Weather effects

    Coordinates spawners:
    - VegetationSpawner: Trees and bushes
    - VehicleSpawner: Military/civilian vehicles
    - PeopleSpawner: Human characters
    - DroneSpawner: F-11 drone variants
    """

    def __init__(
        self,
        models_path: str,
        config: WorldConfig = None,
    ):
        """
        Initialize WorldGenerator.

        Args:
            models_path: Path to models directory
            config: World configuration
        """
        self.models_path = models_path
        self.textures_path = f"{models_path}/textures"
        self.config = config or WorldConfig()

        if self.config.seed is not None:
            random.seed(self.config.seed)
            np.random.seed(self.config.seed)

        # Get stage
        self.stage = get_current_stage()

        # Create world structure
        self._init_world_structure()

        # Initialize component managers
        self.terrain = TerrainGenerator(
            self.stage,
            randomize=self.config.randomize_terrain,
            seed=self.config.seed,
        )
        self.materials = MaterialManager(self.stage, self.textures_path)
        self.lighting = LightingManager(
            self.stage,
            self.textures_path,
            randomize=self.config.randomize_lighting,
            time_of_day=self.config.time_of_day,
        )
        self.atmosphere = AtmosphereController(
            self.stage,
            randomize=self.config.randomize_weather,
            weather=self.config.weather,
        )

        # Connect atmosphere to lighting for weather adjustments
        self.atmosphere.set_lighting_callback(self.lighting.adjust_for_weather)

        # Initialize spawners with appropriate spacing for each type
        veg_spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
            min_spacing=1.0,
        )
        vehicle_spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
            min_spacing=8.0,
        )
        people_spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
            min_spacing=1.5,
        )
        drone_spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
            min_spacing=5.0,
        )

        self.vegetation = VegetationSpawner(self.stage, models_path, veg_spawn_config)
        self.vehicles = VehicleSpawner(self.stage, models_path, vehicle_spawn_config)
        self.people = PeopleSpawner(self.stage, models_path, people_spawn_config)
        self.drones = DroneSpawner(self.stage, models_path, drone_spawn_config)

        # Register drone spawn exclusion zone BEFORE any spawning occurs
        # This ensures nothing spawns at the drone's spawn point
        self._register_drone_exclusion_zone()

    def _init_world_structure(self) -> None:
        """Create base prim structure."""
        paths = [
            "/World",
            "/World/Terrain",
            "/World/Vegetation",
            "/World/Vegetation/Trees",
            "/World/Vegetation/Bushes",
            "/World/Vehicles",
            "/World/People",
            "/World/Lighting",
        ]
        for path in paths:
            if not self.stage.GetPrimAtPath(path):
                prim_utils.create_prim(path, "Xform")

    def _register_drone_exclusion_zone(self) -> None:
        """
        Register drone spawn exclusion zone in the global spatial hash.

        This is called during initialization BEFORE any spawning occurs,
        ensuring that vehicles, people, and trees cannot spawn at the
        drone's intended spawn point.
        """
        if self.config.drone_spawn_exclusion is not None:
            x, y, radius = self.config.drone_spawn_exclusion
            # Use any spawner to register (they all share the global spatial hash)
            self.drones.register_position(x, y, radius)

    def register_exclusion_zone(self, x: float, y: float, radius: float) -> None:
        """
        Register an exclusion zone in the global spatial hash.

        Objects (vehicles, people, trees) will not spawn within this zone.
        Call this BEFORE spawning objects if you need custom exclusion zones.

        Args:
            x: X coordinate of exclusion zone center
            y: Y coordinate of exclusion zone center
            radius: Radius of exclusion zone in meters
        """
        self.drones.register_position(x, y, radius)

    # -------------------------------------------------------------------------
    # Terrain API (delegates to TerrainGenerator)
    # -------------------------------------------------------------------------

    def generate_terrain(
        self,
        size: Tuple[float, float] = None,
        roughness: float = None,
        apply_material: bool = True,
    ) -> str:
        """Generate procedural terrain mesh."""
        size = size or self.config.terrain_size
        roughness = roughness if roughness is not None else self.config.terrain_roughness

        terrain_path = self.terrain.generate(size, roughness)

        if apply_material:
            self.materials.apply_ground_material(terrain_path, self.config.terrain_material)

        return terrain_path

    @property
    def terrain_prim(self) -> Optional[str]:
        """Path to terrain prim."""
        return self.terrain.terrain_prim

    @terrain_prim.setter
    def terrain_prim(self, value: Optional[str]) -> None:
        """Set terrain prim path (for backwards compatibility)."""
        self.terrain.terrain_prim = value

    # -------------------------------------------------------------------------
    # Lighting API (delegates to LightingManager)
    # -------------------------------------------------------------------------

    def setup_lighting(self, hdri_path: str = None, sun_intensity: float = None) -> None:
        """Setup environment lighting with HDRI skybox and sun."""
        self.lighting.setup(
            hdri_path=hdri_path,
            sun_intensity=sun_intensity,
            sun_angle=self.config.sun_angle,
        )
        # Apply atmosphere effects after lighting setup
        self.atmosphere.setup()

    @property
    def skybox_light(self) -> Optional[str]:
        """Path to skybox light."""
        return self.lighting.skybox_light

    @skybox_light.setter
    def skybox_light(self, value: Optional[str]) -> None:
        """Set skybox light path (for backwards compatibility)."""
        self.lighting.skybox_light = value

    @property
    def time_of_day(self) -> Optional[str]:
        """Current time of day preset name."""
        return self.lighting.current_time_of_day

    # -------------------------------------------------------------------------
    # Atmosphere API (delegates to AtmosphereController)
    # -------------------------------------------------------------------------

    def setup_atmosphere(self, weather: str = None) -> None:
        """Setup atmosphere/weather effects."""
        self.atmosphere.setup(weather)

    @property
    def visibility_range(self) -> float:
        """Current visibility range in meters."""
        return self.atmosphere.visibility_range

    @property
    def current_weather(self) -> str:
        """Current weather preset name."""
        return self.atmosphere.current_weather

    @property
    def weather_cfg(self) -> WeatherConfig:
        """Current weather configuration object."""
        return self.atmosphere.weather_config

    # -------------------------------------------------------------------------
    # Vegetation API (delegates to VegetationSpawner)
    # -------------------------------------------------------------------------

    def generate_forest(
        self,
        density: float = None,
        proportions: Dict[str, float] = None,
        include_undergrowth: bool = True,
    ) -> Dict[str, List[str]]:
        """Generate complete forest with trees and undergrowth."""
        density = density or self.config.tree_density
        proportions = proportions or self.config.tree_proportions

        result = {"trees": [], "bushes": []}
        result["trees"] = self.vegetation.spawn_forest(density, proportions)

        if include_undergrowth:
            result["bushes"] = self.vegetation.spawn_undergrowth(
                self.config.undergrowth_density
            )

        return result

    # -------------------------------------------------------------------------
    # Spawning API (delegates to spawners)
    # -------------------------------------------------------------------------

    def spawn_drone(
        self,
        variant: str = "isr_camera",
        position: Tuple[float, float, float] = None,
        heading: float = 0.0,
    ) -> Optional[str]:
        """
        Spawn an F-11 drone with exclusion zone protection.

        IMPORTANT: For the exclusion zone to work correctly, the drone should be
        spawned FIRST before other objects (vehicles, people, trees). The drone
        registers a 15m exclusion radius in the global spatial hash, preventing
        other spawners from placing objects within that zone.

        Recommended spawn order:
        1. spawn_drone() - spawns drone and registers exclusion zone
        2. spawn_poi_clusters() - vehicles/people avoid drone's exclusion zone
        3. generate_forest() - trees avoid drone's exclusion zone

        Args:
            variant: Drone variant ("isr_camera", "base", "lidar", "multispectral")
            position: (x, y, z) spawn position, or None for random with default altitude
            heading: Heading angle in radians

        Returns:
            Prim path of spawned drone, or None if no valid position found
        """
        return self.drones.spawn(variant, position or (0.0, 0.0, 10.0), heading)

    def spawn_drone_formation(
        self,
        variant: str = "isr_camera",
        center: Tuple[float, float, float] = (0, 0, 15),
        count: int = 1,
        spacing: float = 5.0,
        formation: str = "line",
    ) -> List[str]:
        """Spawn multiple drones in formation."""
        return self.drones.spawn_formation(variant, center, count, spacing, formation)

    def spawn_poi_clusters(
        self,
        num_clusters: int = 3,
        targets_per_cluster: Tuple[int, int] = (3, 8),
        cluster_radius: float = 30.0,
        include_vehicles: bool = True,
        include_people: bool = True,
    ) -> List[Dict[str, Any]]:
        """Spawn realistic POI clusters with vehicles and people for ISR training."""
        clusters = []
        half_size = min(self.config.terrain_size) / 2 * 0.8

        for i in range(num_clusters):
            center_x, center_y = self._find_cluster_center(clusters, half_size, cluster_radius)
            center = (center_x, center_y)
            num_targets = random.randint(*targets_per_cluster)
            cluster_targets = []

            # Decide composition
            if include_vehicles and include_people:
                num_vehicles = random.randint(1, max(1, num_targets // 2))
                num_people = num_targets - num_vehicles
            elif include_vehicles:
                num_vehicles, num_people = num_targets, 0
            else:
                num_vehicles, num_people = 0, num_targets

            # Spawn vehicles
            if num_vehicles > 0:
                vehicle_types = ["sedan", "sedan2", "suv", "tank", "tank2"]
                available_types = [vt for vt in vehicle_types
                                  if vt in self.vehicles.vehicle_configs]
                if available_types:
                    vehicle_paths = self.vehicles.spawn_vehicle_group(
                        vehicle_types=available_types,
                        count=num_vehicles,
                        clustering=0.8,
                        center=center,
                    )
                    for vp in vehicle_paths:
                        cluster_targets.append({'type': 'vehicle', 'path': vp})

            # Spawn people
            if num_people > 0 and self.people.person_configs:
                person_types = list(self.people.person_configs.keys())
                people_paths = self.people.spawn_crowd(
                    person_types=person_types,
                    count=num_people,
                    center=center,
                    radius=cluster_radius * 0.5,
                )
                for pp in people_paths:
                    cluster_targets.append({'type': 'person', 'path': pp})

            clusters.append({
                'id': f'cluster_{i}',
                'center': center,
                'radius': cluster_radius,
                'targets': cluster_targets,
                'num_targets': len(cluster_targets),
            })

        return clusters

    def _find_cluster_center(
        self,
        existing_clusters: List[Dict],
        half_size: float,
        cluster_radius: float,
    ) -> Tuple[float, float]:
        """Find non-overlapping center for a new cluster."""
        for _ in range(50):
            candidate_x = random.uniform(-half_size, half_size)
            candidate_y = random.uniform(-half_size, half_size)

            if existing_clusters:
                min_dist = min(
                    np.sqrt((candidate_x - c['center'][0])**2 +
                           (candidate_y - c['center'][1])**2)
                    for c in existing_clusters
                )
                if min_dist > cluster_radius * 2.5:
                    return candidate_x, candidate_y
            else:
                return candidate_x, candidate_y

        # Fallback
        return random.uniform(-half_size, half_size), random.uniform(-half_size, half_size)

    # -------------------------------------------------------------------------
    # Full Environment API
    # -------------------------------------------------------------------------

    def generate_full_environment(self) -> Dict[str, Any]:
        """Generate complete environment with terrain, lighting, and vegetation."""
        result = {
            "terrain": self.generate_terrain(),
            "lighting": "/World/Lighting",
            "forest": None,
        }
        self.setup_lighting()
        result["forest"] = self.generate_forest()
        return result

    def generate_isr_environment(
        self,
        drone_variant: str = "isr_camera",
        drone_position: Tuple[float, float, float] = None,
        drone_heading: float = 0.0,
        num_poi_clusters: int = 3,
        targets_per_cluster: Tuple[int, int] = (3, 8),
        include_forest: bool = True,
    ) -> Dict[str, Any]:
        """
        Generate complete ISR training environment with correct spawn ordering.

        This method ensures the drone is spawned FIRST before any other objects,
        so its exclusion zone (15m radius) is respected by subsequent spawns.
        No vehicles, people, or trees will spawn within the drone's exclusion zone.

        Spawn order:
        1. Terrain and lighting
        2. Drone (registers exclusion zone)
        3. POI clusters (vehicles/people)
        4. Forest (trees)

        Args:
            drone_variant: Drone variant ("isr_camera", "base", "lidar", "multispectral")
            drone_position: (x, y, z) spawn position, or None for default (0, 0, 10)
            drone_heading: Heading angle in radians
            num_poi_clusters: Number of POI clusters to spawn
            targets_per_cluster: (min, max) targets per cluster
            include_forest: Whether to generate forest vegetation

        Returns:
            Dictionary with environment components:
            - "terrain": terrain prim path
            - "lighting": lighting prim path
            - "drone": drone prim path (or None if spawn failed)
            - "clusters": list of POI cluster info
            - "forest": forest generation result (if include_forest=True)
        """
        result = {
            "terrain": self.generate_terrain(),
            "lighting": "/World/Lighting",
            "drone": None,
            "clusters": [],
            "forest": None,
        }
        self.setup_lighting()

        # CRITICAL: Spawn drone FIRST to register exclusion zone
        # This ensures no other objects spawn within the drone's 15m radius
        result["drone"] = self.spawn_drone(
            variant=drone_variant,
            position=drone_position,
            heading=drone_heading,
        )

        # Spawn POI clusters - these will avoid the drone's exclusion zone
        if num_poi_clusters > 0:
            result["clusters"] = self.spawn_poi_clusters(
                num_clusters=num_poi_clusters,
                targets_per_cluster=targets_per_cluster,
            )

        # Generate forest last - trees will avoid drone's exclusion zone
        if include_forest:
            result["forest"] = self.generate_forest()

        return result

    def randomize_environment(self, seed: int = None) -> None:
        """Randomize environment for new episode based on config randomization flags."""
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
            self.config.seed = seed

        self.clear_spawned_objects()

        if self.config.randomize_terrain:
            self.generate_terrain()

        if self.config.randomize_lighting:
            self.setup_lighting()
        elif self.config.randomize_weather:
            self.setup_atmosphere()

    # -------------------------------------------------------------------------
    # Cleanup API
    # -------------------------------------------------------------------------

    def clear_spawned_objects(self) -> None:
        """Clear all spawned objects (vegetation, vehicles, people, drones)."""
        self.vegetation.clear_all()
        self.vehicles.clear_all()
        self.people.clear_all()
        self.drones.clear_all()
        BaseSpawner.clear_position_registry()
        # Re-register drone exclusion zone after clearing
        self._register_drone_exclusion_zone()

    def clear_all(self) -> None:
        """Clear entire generated world."""
        self.clear_spawned_objects()
        self.terrain.clear()
        self.lighting.clear()
        self.atmosphere.clear()

    def get_stats(self) -> Dict[str, Any]:
        """Get statistics about generated world."""
        return {
            "trees": self.vegetation.tree_count,
            "bushes": self.vegetation.bush_count,
            "vehicles": self.vehicles.vehicle_count,
            "people": self.people.person_count,
            "drones": self.drones.drone_count,
            "time_of_day": self.lighting.current_time_of_day,
            "weather": self.atmosphere.current_weather,
        }
