"""
People/pedestrian spawner for ISR training.

Spawns human figures in various poses and groupings for aerial detection training.
Uses NVIDIA Reallusion character models from the Characters asset pack.
"""

from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import random
import os

from pxr import Gf, UsdGeom
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.core.utils.stage import add_reference_to_stage

from .base_spawner import BaseSpawner, SpawnConfig


# Discrete yaw angles (24 total, 15-degree increments)
# Figures can ONLY spawn at these orientations to ensure upright poses
DISCRETE_YAW_ANGLES = tuple(i * 15.0 for i in range(24))  # 0, 15, 30, ... 345


@dataclass
class PersonConfig:
    """Configuration for person/pedestrian spawning."""
    usd_path: str = None
    category: str = "civilian"
    base_scale: float = 1.0
    scale_variation: tuple = (0.9, 1.1)
    pose: str = "standing"
    # Rotation needed to orient character correctly (degrees)
    rotation_offset: Tuple[float, float, float] = (0, 0, 0)


class PeopleSpawner(BaseSpawner):
    """
    Spawner for people/pedestrians in ISR training environments.

    Uses NVIDIA Reallusion character models:
    - Worker: Construction/industrial worker
    - Debra: Female civilian

    Supports:
    - Individual pedestrians
    - Groups (crowds, patrols)
    - Various poses (standing, walking, sitting)
    - Military and civilian categories

    For ISR training, people should be:
    - Visible from 50-150m altitude
    - Varied in pose/stance
    - Placed in realistic groupings
    """

    # Quaternius character models (relative to models_path)
    # Men: Male_Casual, Male_LongSleeve, Male_Shirt, Male_Suit + Smooth variants
    # Women: Female_Alternative, Female_Casual, Female_Dress, Female_TankTop + Smooth variants
    CHARACTER_PATHS = {
        # Men
        "male_casual": "people/glb/Male_Casual.glb",
        "male_shirt": "people/glb/Male_Shirt.glb",
        "male_suit": "people/glb/Male_Suit.glb",
        "male_longsleeve": "people/glb/Male_LongSleeve.glb",
        # Women
        "female_casual": "people/glb/Female_Casual.glb",
        "female_dress": "people/glb/Female_Dress.glb",
        "female_tanktop": "people/glb/Female_TankTop.glb",
        "female_alt": "people/glb/Female_Alternative.glb",
    }

    def __init__(
        self,
        stage,
        models_path: str = None,
        config: SpawnConfig = None,
        person_configs: Dict[str, PersonConfig] = None,
    ):
        super().__init__(stage, config)
        self.models_path = models_path
        self.person_configs = person_configs or {}
        self.person_count = 0

        # Ensure parent prim exists
        if not self.stage.GetPrimAtPath("/World/People"):
            prim_utils.create_prim("/World/People", "Xform")

        # Auto-register available characters if models_path provided
        if models_path:
            self._register_available_characters()

    def _register_available_characters(self) -> None:
        """Auto-register characters found in models_path."""
        for name, rel_path in self.CHARACTER_PATHS.items():
            full_path = f"{self.models_path}/{rel_path}"
            # Only register if the file exists
            if os.path.exists(full_path):
                self.person_configs[name] = PersonConfig(
                    usd_path=full_path,
                    category="civilian",
                    base_scale=0.7,  # Scale down 30% for proper proportions
                    pose="standing",
                    rotation_offset=(0, 0, 0),  # No rotation needed - models are Z-up
                )
            else:
                print(f"Warning: Character model not found: {full_path}")

    def register_person(self, name: str, config: PersonConfig) -> None:
        """Register a new person type."""
        self.person_configs[name] = config

    def register_custom_model(
        self,
        name: str,
        usd_path: str,
        category: str = "civilian",
        pose: str = "standing",
        scale: float = 1.0,
    ) -> None:
        """
        Register a custom USD person model.

        Args:
            name: Identifier for this person type
            usd_path: Path to USD file
            category: "civilian" or "military"
            pose: Pose description
            scale: Base scale factor
        """
        self.person_configs[name] = PersonConfig(
            usd_path=usd_path,
            category=category,
            pose=pose,
            base_scale=scale,
        )

    def spawn_person(
        self,
        person_type: str,
        position: Tuple[float, float] = None,
        facing: float = None,
    ) -> Optional[str]:
        """
        Spawn a single person.

        Args:
            person_type: Type of person from registered configs
            position: Optional (x, y) position
            facing: Optional facing direction in degrees

        Returns:
            Prim path of spawned person, or None if no valid position available
        """
        if person_type not in self.person_configs:
            raise ValueError(f"Unknown person type: {person_type}. "
                           f"Available: {list(self.person_configs.keys())}")

        cfg = self.person_configs[person_type]

        # Person approximate radius for overlap prevention (~0.5m)
        person_radius = 0.5 * cfg.base_scale

        if position is None:
            result = self.get_random_position(object_radius=person_radius)
            if result is None:
                return None  # No valid position, skip spawning
            x, y = result
        else:
            # Even with explicit position, find valid nearby spot to avoid overlap
            result = self.find_valid_position(position[0], position[1], person_radius)
            if result is None:
                # No valid position found - skip spawning this person
                return None
            x, y = result

        # Register position to prevent future overlaps
        self.register_position(x, y, person_radius)

        if facing is None:
            # Use discrete yaw angles only (15-degree increments)
            # This ensures people spawn upright with only Z-axis rotation
            facing = random.choice(DISCRETE_YAW_ANGLES)
        else:
            # Snap provided facing to nearest discrete angle
            facing = min(DISCRETE_YAW_ANGLES, key=lambda x: abs((x - facing + 180) % 360 - 180))

        scale_var = random.uniform(*cfg.scale_variation)
        scale = cfg.base_scale * scale_var

        person_path = f'/World/People/{person_type}_{self.person_count:04d}'
        prim_utils.create_prim(person_path, "Xform")

        # Add USD reference
        add_reference_to_stage(usd_path=cfg.usd_path, prim_path=person_path)

        # Get ground height
        z = self.raycast_to_ground(x, y)

        # Apply transforms using Xformable API for proper ordering
        person_prim = self.stage.GetPrimAtPath(person_path)
        person_xform = UsdGeom.Xformable(person_prim)

        # Clear any existing xform ops
        person_xform.ClearXformOpOrder()

        # Apply in order: translate, rotate (model orientation + facing), scale
        # Use explicit double precision to avoid type conflicts
        person_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, z))

        # Apply model rotation offset first (e.g., Y-up to Z-up)
        rx, ry, rz = cfg.rotation_offset
        if rx != 0:
            person_xform.AddRotateXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(rx)
        if ry != 0:
            person_xform.AddRotateYOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(ry)
        # Then apply facing direction
        person_xform.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(facing + rz)

        person_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(scale, scale, scale))

        self.spawned_objects.append(person_path)
        self.person_count += 1

        return person_path

    def spawn_group(
        self,
        person_types: List[str],
        center: Tuple[float, float] = None,
        radius: float = 5.0,
        facing_center: bool = True,
    ) -> List[str]:
        """
        Spawn a group of people clustered together.

        Args:
            person_types: List of person types to spawn
            center: Center of the group (random if None)
            radius: Spread radius of the group
            facing_center: If True, people face toward group center

        Returns:
            List of spawned person prim paths
        """
        if center is None:
            center = self.get_random_position()

        spawned = []

        for person_type in person_types:
            # Random offset from center
            angle = random.uniform(0, 360)
            dist = random.uniform(0, radius)

            offset_x = dist * (angle * 3.14159 / 180)
            offset_y = dist * ((angle + 90) * 3.14159 / 180)

            pos = (center[0] + offset_x, center[1] + offset_y)

            if facing_center:
                # Calculate facing toward center
                dx = center[0] - pos[0]
                dy = center[1] - pos[1]
                facing = ((dx) / 3.14159 * 180 if dx != 0 else 0) % 360
            else:
                facing = None

            path = self.spawn_person(person_type, position=pos, facing=facing)
            if path is not None:
                spawned.append(path)

        return spawned

    def spawn_patrol(
        self,
        person_type: str,
        waypoints: List[Tuple[float, float]],
        count: int = 4,
        spacing: float = 3.0,
    ) -> List[str]:
        """
        Spawn people in a patrol formation along waypoints.

        Args:
            person_type: Type of person to spawn
            waypoints: List of (x, y) waypoints
            count: Number of people in patrol
            spacing: Distance between people

        Returns:
            List of spawned person prim paths
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints for patrol")

        spawned = []

        # Calculate total path length
        total_length = 0
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            total_length += (dx**2 + dy**2)**0.5

        # Distribute people along path
        for i in range(count):
            target_dist = (i * spacing) % total_length

            # Find position along path
            current_dist = 0
            for j in range(1, len(waypoints)):
                dx = waypoints[j][0] - waypoints[j-1][0]
                dy = waypoints[j][1] - waypoints[j-1][1]
                segment_len = (dx**2 + dy**2)**0.5

                if current_dist + segment_len >= target_dist:
                    # Position is on this segment
                    t = (target_dist - current_dist) / segment_len
                    pos_x = waypoints[j-1][0] + t * dx
                    pos_y = waypoints[j-1][1] + t * dy

                    # Facing along path
                    facing = (dy / dx * 180 / 3.14159 if dx != 0 else 90) % 360

                    path = self.spawn_person(person_type, position=(pos_x, pos_y), facing=facing)
                    if path is not None:
                        spawned.append(path)
                    break

                current_dist += segment_len

        return spawned

    def spawn_crowd(
        self,
        person_types: List[str],
        count: int,
        center: Tuple[float, float] = None,
        radius: float = 15.0,
    ) -> List[str]:
        """
        Spawn a crowd of randomly distributed people.

        Args:
            person_types: Types of people to randomly select from
            count: Number of people
            center: Center of crowd (random if None)
            radius: Maximum radius from center

        Returns:
            List of spawned person prim paths
        """
        if center is None:
            center = self.get_random_position()

        selected = [random.choice(person_types) for _ in range(count)]
        return self.spawn_group(selected, center=center, radius=radius, facing_center=False)
