"""
Base spawner class providing common functionality for all spawners.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import random
import numpy as np

from pxr import Gf, UsdGeom, Sdf
import carb


@dataclass
class SpawnConfig:
    """Configuration for spawn area and behavior."""
    area_size: Tuple[float, float] = (200.0, 200.0)
    edge_margin: float = 1.0
    min_spacing: float = 2.0  # Minimum distance between objects
    seed: Optional[int] = None


# Global position registry shared across all spawners
_global_positions: List[Tuple[float, float, float]] = []  # (x, y, radius)


class BaseSpawner:
    """
    Base class for all object spawners.

    Provides:
    - Random position generation within bounds
    - Euler to quaternion conversion
    - Raycast to ground for terrain-following
    - Prim creation and property setting
    """

    def __init__(self, stage, config: SpawnConfig = None):
        self.stage = stage
        self.config = config or SpawnConfig()
        self.spawned_objects: List[str] = []

        if self.config.seed is not None:
            random.seed(self.config.seed)
            np.random.seed(self.config.seed)

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert Euler angles (degrees) to quaternion (w, x, y, z).

        Args:
            roll: Rotation around X axis in degrees
            pitch: Rotation around Y axis in degrees
            yaw: Rotation around Z axis in degrees

        Returns:
            Tuple of (qw, qx, qy, qz)
        """
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

        cr, sr = np.cos(roll / 2), np.sin(roll / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qw, qx, qy, qz

    def get_random_position(self, object_radius: float = 1.0, max_attempts: int = 50) -> Tuple[float, float]:
        """
        Get random (x, y) position within spawn area that doesn't overlap with existing objects.

        Args:
            object_radius: Radius of the object being spawned (for collision checking)
            max_attempts: Maximum attempts to find non-overlapping position

        Returns:
            (x, y) position
        """
        half_x = (self.config.area_size[0] / 2) - self.config.edge_margin
        half_y = (self.config.area_size[1] / 2) - self.config.edge_margin

        for _ in range(max_attempts):
            x = random.uniform(-half_x, half_x)
            y = random.uniform(-half_y, half_y)

            if self._check_position_valid(x, y, object_radius):
                return x, y

        # If no valid position found, return random position anyway (sparse environments)
        return random.uniform(-half_x, half_x), random.uniform(-half_y, half_y)

    def _check_position_valid(self, x: float, y: float, radius: float) -> bool:
        """Check if position doesn't overlap with existing objects."""
        global _global_positions
        min_dist = self.config.min_spacing

        for ox, oy, oradius in _global_positions:
            dist = ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5
            required_dist = radius + oradius + min_dist
            if dist < required_dist:
                return False
        return True

    def find_valid_position(self, x: float, y: float, radius: float, max_attempts: int = 20) -> Tuple[float, float]:
        """
        Find a valid position near the requested position, adjusting if needed to avoid overlap.

        Args:
            x, y: Requested position
            radius: Object radius for collision checking
            max_attempts: Number of attempts to find nearby valid position

        Returns:
            Valid (x, y) position (may be adjusted from original)
        """
        # First check if requested position is valid
        if self._check_position_valid(x, y, radius):
            return x, y

        # Try to find a nearby valid position
        half_x = (self.config.area_size[0] / 2) - self.config.edge_margin
        half_y = (self.config.area_size[1] / 2) - self.config.edge_margin

        for attempt in range(max_attempts):
            # Spiral outward from requested position
            offset = (attempt + 1) * (radius + self.config.min_spacing)
            angle = attempt * 2.39996  # Golden angle for even distribution
            new_x = x + offset * np.cos(angle)
            new_y = y + offset * np.sin(angle)

            # Clamp to bounds
            new_x = max(-half_x, min(half_x, new_x))
            new_y = max(-half_y, min(half_y, new_y))

            if self._check_position_valid(new_x, new_y, radius):
                return new_x, new_y

        # Fallback to random position
        return self.get_random_position(radius)

    def register_position(self, x: float, y: float, radius: float = 1.0) -> None:
        """Register a spawned object's position for overlap prevention."""
        global _global_positions
        _global_positions.append((x, y, radius))

    @staticmethod
    def clear_position_registry() -> None:
        """Clear all registered positions (call when resetting environment)."""
        global _global_positions
        _global_positions.clear()

    def raycast_to_ground(self, x: float, y: float, start_height: float = 100.0) -> float:
        """
        Cast ray down from position to find ground height.

        Args:
            x: X position
            y: Y position
            start_height: Height to start raycast from

        Returns:
            Ground height at (x, y) position
        """
        try:
            from omni.physx import get_physx_scene_query_interface

            origin = carb.Float3(x, y, start_height)
            direction = carb.Float3(0.0, 0.0, -1.0)
            max_distance = start_height * 2

            hit = get_physx_scene_query_interface().raycast_closest(
                origin, direction, max_distance
            )

            if hit["hit"]:
                return start_height - hit["distance"]
            else:
                return 0.0

        except Exception as e:
            carb.log_warn(f"Raycast failed: {e}, defaulting to z=0")
            return 0.0

    def clear_all(self) -> None:
        """Remove all spawned objects."""
        for prim_path in self.spawned_objects:
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                self.stage.RemovePrim(prim_path)

        self.spawned_objects.clear()

    def get_spawn_count(self) -> int:
        """Get number of spawned objects."""
        return len(self.spawned_objects)
