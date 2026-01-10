"""
Base spawner class providing common functionality for all spawners.

Includes semantic labeling for Isaac Sim's native annotators (bounding_box_2d_tight, etc.)
"""

# Standard library
import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Third-party
import numpy as np
from pxr import Gf, Sdf, UsdGeom

# Isaac Sim / Omniverse
import carb

# Constants for position finding algorithm
GOLDEN_ANGLE = (math.sqrt(5) - 1) * math.pi  # ~2.39996 radians, for even spiral distribution
DEFAULT_MAX_SPAWN_ATTEMPTS = 50
DEFAULT_SPIRAL_SEARCH_ATTEMPTS = 20

# Discrete yaw angles (24 total, 15-degree increments)
# Figures can ONLY spawn at these orientations to ensure upright poses
DISCRETE_YAW_ANGLES = tuple(i * 15.0 for i in range(24))  # 0, 15, 30, ... 345


@dataclass
class SpawnConfig:
    """Configuration for spawn area and behavior."""
    area_size: Tuple[float, float] = (200.0, 200.0)
    edge_margin: float = 1.0
    min_spacing: float = 2.0  # Minimum distance between objects
    seed: Optional[int] = None


class SpatialHash:
    """Grid-based spatial index for O(1) collision lookups."""

    def __init__(self, cell_size: float = 10.0):
        self.cell_size = cell_size
        self.cells: Dict[Tuple[int, int], List[Tuple[float, float, float]]] = {}
        self.all_positions: List[Tuple[float, float, float]] = []  # For iteration

    def _get_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Get the cell coordinates for a position."""
        return (int(x // self.cell_size), int(y // self.cell_size))

    def _get_neighboring_cells(self, x: float, y: float, radius: float) -> List[Tuple[int, int]]:
        """Get all cells that could contain objects within radius."""
        cx, cy = self._get_cell(x, y)
        # Calculate how many cells the radius spans
        cell_radius = int(radius // self.cell_size) + 1
        cells = []
        for dx in range(-cell_radius, cell_radius + 1):
            for dy in range(-cell_radius, cell_radius + 1):
                cells.append((cx + dx, cy + dy))
        return cells

    def insert(self, x: float, y: float, radius: float) -> None:
        """Insert a position into the spatial hash."""
        cell = self._get_cell(x, y)
        if cell not in self.cells:
            self.cells[cell] = []
        entry = (x, y, radius)
        self.cells[cell].append(entry)
        self.all_positions.append(entry)

    def query_nearby(self, x: float, y: float, radius: float) -> List[Tuple[float, float, float]]:
        """Get all positions that could potentially overlap with given position+radius."""
        nearby = []
        for cell in self._get_neighboring_cells(x, y, radius):
            if cell in self.cells:
                nearby.extend(self.cells[cell])
        return nearby

    def clear(self) -> None:
        """Clear all positions."""
        self.cells.clear()
        self.all_positions.clear()

    def __iter__(self):
        """Iterate over all positions."""
        return iter(self.all_positions)

    def __len__(self):
        return len(self.all_positions)


# Global spatial hash shared across all spawners
_global_spatial_hash: SpatialHash = SpatialHash(cell_size=10.0)


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
        self._spatial_hash = _global_spatial_hash

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

    def get_random_position(self, object_radius: float = 1.0, max_attempts: int = 50) -> Optional[Tuple[float, float]]:
        """
        Get random (x, y) position within spawn area that doesn't overlap with existing objects.

        Args:
            object_radius: Radius of the object being spawned (for collision checking)
            max_attempts: Maximum attempts to find non-overlapping position

        Returns:
            (x, y) position, or None if no valid position could be found after max_attempts.
            When None is returned, the caller should skip spawning rather than
            force the object into an invalid position.
        """
        half_x = (self.config.area_size[0] / 2) - self.config.edge_margin
        half_y = (self.config.area_size[1] / 2) - self.config.edge_margin

        for _ in range(max_attempts):
            x = random.uniform(-half_x, half_x)
            y = random.uniform(-half_y, half_y)

            if self._check_position_valid(x, y, object_radius):
                return x, y

        # No valid position found after max_attempts - return None to let caller handle
        carb.log_warn(f"Could not find valid position after {max_attempts} attempts")
        return None

    def _check_position_valid(self, x: float, y: float, radius: float) -> bool:
        """Check if position doesn't overlap with existing objects using spatial hash."""
        min_dist = self.config.min_spacing

        # Only check positions in nearby cells (O(1) average case)
        for ox, oy, oradius in self._spatial_hash.query_nearby(x, y, radius + min_dist):
            dist_sq = (x - ox) ** 2 + (y - oy) ** 2
            required_dist = radius + oradius + min_dist
            if dist_sq < required_dist ** 2:  # Compare squared distances (avoid sqrt)
                return False
        return True

    def find_valid_position(self, x: float, y: float, radius: float, max_attempts: int = 20) -> Optional[Tuple[float, float]]:
        """
        Find a valid position near the requested position, adjusting if needed to avoid overlap.

        Args:
            x, y: Requested position
            radius: Object radius for collision checking
            max_attempts: Number of attempts to find nearby valid position

        Returns:
            Valid (x, y) position, or None if no valid position can be found.
            When None is returned, the caller should skip spawning rather than
            force the object into an invalid position.
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
            angle = attempt * GOLDEN_ANGLE  # Golden angle for even distribution
            new_x = x + offset * np.cos(angle)
            new_y = y + offset * np.sin(angle)

            # Clamp to bounds
            new_x = max(-half_x, min(half_x, new_x))
            new_y = max(-half_y, min(half_y, new_y))

            if self._check_position_valid(new_x, new_y, radius):
                return new_x, new_y

        # No valid position found - return None so caller can skip spawning
        # This prevents objects from being forced into overlapping positions
        return None

    def register_position(self, x: float, y: float, radius: float = 1.0) -> None:
        """Register a position in the spatial hash."""
        self._spatial_hash.insert(x, y, radius)

    def get_spawn_position(
        self,
        position: Optional[Tuple[float, float]],
        radius: float,
    ) -> Optional[Tuple[float, float]]:
        """
        Get a valid spawn position, either random or near the requested position.

        This consolidates the common spawn position logic used across all spawners.

        Args:
            position: Requested (x, y) position, or None for random
            radius: Object radius for collision checking

        Returns:
            Valid (x, y) position and registers it, or None if no valid position found
        """
        if position is None:
            result = self.get_random_position(object_radius=radius)
        else:
            result = self.find_valid_position(position[0], position[1], radius)

        if result is not None:
            self.register_position(result[0], result[1], radius)

        return result

    @staticmethod
    def clear_position_registry() -> None:
        """Clear all registered positions (call when resetting environment)."""
        global _global_spatial_hash
        _global_spatial_hash.clear()

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

    def add_semantic_label(
        self,
        prim_path: str,
        class_name: str,
        instance_id: Optional[int] = None,
    ) -> bool:
        """
        Add semantic label to a prim for Isaac Sim's native annotators.

        This enables bounding_box_2d_tight and other replicator annotators
        to detect and label this object.

        Args:
            prim_path: USD path to the prim
            class_name: Semantic class name (e.g., "vehicle", "person", "tree")
            instance_id: Optional unique instance ID

        Returns:
            True if label was added successfully
        """
        try:
            prim = self.stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                carb.log_warn(f"Cannot add semantic label: prim not found at {prim_path}")
                return False

            # Add Semantics schema using Isaac Sim's API
            from pxr import Semantics

            if not prim.HasAPI(Semantics.SemanticsAPI):
                Semantics.SemanticsAPI.Apply(prim, "Semantics")

            sem_api = Semantics.SemanticsAPI.Get(prim, "Semantics")
            sem_api.CreateSemanticTypeAttr().Set("class")
            sem_api.CreateSemanticDataAttr().Set(class_name)

            # Add instance ID if provided
            if instance_id is not None:
                # Create instance semantics
                if not prim.HasAPI(Semantics.SemanticsAPI, "instance"):
                    Semantics.SemanticsAPI.Apply(prim, "instance")
                inst_api = Semantics.SemanticsAPI.Get(prim, "instance")
                inst_api.CreateSemanticTypeAttr().Set("instance")
                inst_api.CreateSemanticDataAttr().Set(str(instance_id))

            return True

        except ImportError:
            # Fall back to manual attribute creation if Semantics schema not available
            try:
                prim = self.stage.GetPrimAtPath(prim_path)
                if not prim.IsValid():
                    return False

                # Create semantic attributes manually
                prim.CreateAttribute("semantic:Semantics:params:semanticType", Sdf.ValueTypeNames.String).Set("class")
                prim.CreateAttribute("semantic:Semantics:params:semanticData", Sdf.ValueTypeNames.String).Set(class_name)

                if instance_id is not None:
                    prim.CreateAttribute("semantic:instance:params:semanticType", Sdf.ValueTypeNames.String).Set("instance")
                    prim.CreateAttribute("semantic:instance:params:semanticData", Sdf.ValueTypeNames.String).Set(str(instance_id))

                return True
            except Exception as e:
                carb.log_warn(f"Failed to add semantic label manually: {e}")
                return False

        except Exception as e:
            carb.log_warn(f"Failed to add semantic label to {prim_path}: {e}")
            return False

    def add_semantic_labels_recursive(
        self,
        root_prim_path: str,
        class_name: str,
        instance_id: Optional[int] = None,
    ) -> int:
        """
        Add semantic label to the root prim only.

        Note: Previously labeled all descendants, but this caused duplicate
        detections from the bounding_box_2d_tight annotator. Now only labels
        the root prim which provides a single bounding box per object.

        Args:
            root_prim_path: USD path to the root prim
            class_name: Semantic class name
            instance_id: Optional unique instance ID

        Returns:
            Number of prims labeled (0 or 1)
        """
        # Only label the root prim to avoid duplicate bounding boxes
        if self.add_semantic_label(root_prim_path, class_name, instance_id):
            return 1
        return 0
