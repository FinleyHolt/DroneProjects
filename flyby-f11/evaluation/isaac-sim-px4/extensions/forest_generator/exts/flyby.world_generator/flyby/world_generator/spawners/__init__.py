"""
Spawner modules for procedural object placement.

Each spawner handles a specific category of objects and provides:
- Random placement within defined areas
- Terrain-following (raycast to ground)
- Scale and rotation variation
- Collision avoidance (optional)
"""

from .base_spawner import BaseSpawner, SpawnConfig
from .vegetation_spawner import VegetationSpawner, TreeConfig, BushConfig
from .vehicle_spawner import VehicleSpawner, VehicleConfig
from .people_spawner import PeopleSpawner, PersonConfig

__all__ = [
    "BaseSpawner",
    "SpawnConfig",
    "VegetationSpawner",
    "TreeConfig",
    "BushConfig",
    "VehicleSpawner",
    "VehicleConfig",
    "PeopleSpawner",
    "PersonConfig",
]
