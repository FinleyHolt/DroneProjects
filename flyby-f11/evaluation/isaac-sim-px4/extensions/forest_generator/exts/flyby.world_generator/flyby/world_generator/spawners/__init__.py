"""
Spawner modules for procedural object placement.

Each spawner handles a specific category of objects and provides:
- Random placement within defined areas
- Terrain-following (raycast to ground)
- Scale and rotation variation
- Collision avoidance (optional)

Architecture:
    DroneSpawner composes:
    - DroneGeometryBuilder: Visual meshes and materials
    - DronePhysicsBuilder: Rigid bodies, mass, joints
    - DronePayloadBuilder: Gimbal, camera, LiDAR attachments
"""

from .base_spawner import BaseSpawner, SpawnConfig
from .vegetation_spawner import VegetationSpawner, TreeConfig, BushConfig
from .vehicle_spawner import VehicleSpawner, VehicleConfig
from .people_spawner import PeopleSpawner, PersonConfig
from .drone_spawner import DroneSpawner, DroneConfig, F11_VARIANTS

# Drone component builders (for advanced customization)
from .drone_geometry import DroneGeometryBuilder
from .drone_physics import DronePhysicsBuilder
from .drone_payloads import DronePayloadBuilder

__all__ = [
    # Base
    "BaseSpawner",
    "SpawnConfig",
    # Vegetation
    "VegetationSpawner",
    "TreeConfig",
    "BushConfig",
    # Vehicles
    "VehicleSpawner",
    "VehicleConfig",
    # People
    "PeopleSpawner",
    "PersonConfig",
    # Drones
    "DroneSpawner",
    "DroneConfig",
    "F11_VARIANTS",
    # Drone component builders
    "DroneGeometryBuilder",
    "DronePhysicsBuilder",
    "DronePayloadBuilder",
]
