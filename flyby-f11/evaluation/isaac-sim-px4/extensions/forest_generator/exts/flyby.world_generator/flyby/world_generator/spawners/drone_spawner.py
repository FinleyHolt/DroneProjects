"""
Drone spawner for F-11 quadcopter variants.

Spawns fully articulated F-11 drones with physics simulation and PX4 integration.

Architecture:
    DroneSpawner (orchestrator) composes:
    - DroneGeometryBuilder: Visual meshes and materials
    - DronePhysicsBuilder: Rigid bodies, mass, joints
    - DronePayloadBuilder: Gimbal, camera, LiDAR attachments
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math

from pxr import Gf, UsdGeom
import omni.isaac.core.utils.prims as prim_utils

from .base_spawner import BaseSpawner, SpawnConfig
from .drone_geometry import DroneGeometryBuilder
from .drone_physics import DronePhysicsBuilder
from .drone_payloads import DronePayloadBuilder


@dataclass
class DroneConfig:
    """Configuration for drone spawning."""
    # Variant identifier
    variant: str = "base"

    # Mass properties
    mass: float = 4.5  # kg
    inertia: Tuple[float, float, float] = (0.085, 0.085, 0.145)  # Ixx, Iyy, Izz
    center_of_gravity: Tuple[float, float, float] = (0, 0, 0)  # m, relative to base_link

    # Frame dimensions
    wheelbase: float = 0.55  # m (motor-to-motor diagonal)
    arm_length: float = 0.195  # m (from center)
    body_size: Tuple[float, float, float] = (0.30, 0.30, 0.12)  # m
    prop_diameter: float = 0.38  # m (15" props)

    # Spawn parameters
    spawn_altitude: float = 10.0  # m above ground
    min_altitude: float = 2.0
    max_altitude: float = 50.0

    # Exclusion zone - prevents other objects from spawning within this radius
    # Set large (15m) to ensure clear space around drone for safe operation
    exclusion_radius: float = 15.0  # meters

    # Visual properties
    frame_color: Tuple[float, float, float] = (0.15, 0.15, 0.15)  # Dark gray
    prop_color: Tuple[float, float, float] = (0.1, 0.1, 0.1)  # Near black

    # Physics parameters
    rotor_mass: float = 0.025  # kg per rotor
    rotor_inertia: Tuple[float, float, float] = (1e-5, 5e-4, 5e-4)  # kg*m^2
    joint_damping: float = 0.004  # N*m*s/rad

    # Payload (variant-specific)
    payload_mass: float = 0.0
    payload_offset: Tuple[float, float, float] = (0, 0, 0)
    has_gimbal: bool = False
    has_lidar: bool = False
    has_camera: bool = False


# F-11 Variant Configurations
F11_VARIANTS = {
    "base": DroneConfig(
        variant="base",
        mass=4.5,
        inertia=(0.085, 0.085, 0.145),
        center_of_gravity=(0, 0, 0),
    ),
    "isr_camera": DroneConfig(
        variant="isr_camera",
        mass=6.3,
        inertia=(0.115, 0.115, 0.175),
        center_of_gravity=(0, 0, -0.05),
        payload_mass=1.8,
        payload_offset=(0, 0, -0.10),
        has_gimbal=True,
        has_camera=True,
    ),
    "lidar": DroneConfig(
        variant="lidar",
        mass=5.25,
        inertia=(0.098, 0.098, 0.158),
        center_of_gravity=(0, 0, -0.02),
        payload_mass=0.75,
        payload_offset=(0, 0, -0.08),
        has_lidar=True,
    ),
    "multispectral": DroneConfig(
        variant="multispectral",
        mass=4.95,
        inertia=(0.090, 0.090, 0.152),
        center_of_gravity=(0, 0, -0.01),
        payload_mass=0.45,
        payload_offset=(0, 0, -0.06),
        has_camera=True,
    ),
}


class DroneSpawner(BaseSpawner):
    """
    Spawner for F-11 quadcopter drones.

    Creates fully articulated USD drones with:
    - Accurate physics (mass, inertia, CoG)
    - Revolute joints for rotor actuation
    - Optional payload attachments (gimbal, LiDAR, camera)
    - PX4 integration support

    Composes:
    - DroneGeometryBuilder: Visual geometry
    - DronePhysicsBuilder: Physics setup
    - DronePayloadBuilder: Payload assemblies
    """

    # Rotor positions relative to base_link (X-configuration)
    # Order: FR, BL, FL, BR (matching PX4 motor numbering)
    ROTOR_POSITIONS = [
        (+0.195, -0.195, +0.04),  # Rotor 0: Front-Right (CW)
        (-0.195, +0.195, +0.04),  # Rotor 1: Back-Left (CW)
        (+0.195, +0.195, +0.04),  # Rotor 2: Front-Left (CCW)
        (-0.195, -0.195, +0.04),  # Rotor 3: Back-Right (CCW)
    ]

    # Rotor spin directions (1 = CW, -1 = CCW)
    ROTOR_DIRECTIONS = [1, 1, -1, -1]

    def __init__(
        self,
        stage,
        models_path: str = None,
        config: SpawnConfig = None,
        drone_configs: Dict[str, DroneConfig] = None,
    ):
        super().__init__(stage, config)
        self.models_path = models_path
        self.drone_configs = drone_configs or F11_VARIANTS.copy()
        self.drone_count = 0
        self.spawned_drones: Dict[str, dict] = {}

        # Ensure parent prim exists
        if not self.stage.GetPrimAtPath("/World/Drones"):
            prim_utils.create_prim("/World/Drones", "Xform")

        # Initialize component builders
        self.geometry = DroneGeometryBuilder(stage)
        self.physics = DronePhysicsBuilder(stage)
        self.payloads = DronePayloadBuilder(stage, self.geometry, self.physics)

    def register_variant(self, name: str, config: DroneConfig) -> None:
        """Register a new drone variant."""
        self.drone_configs[name] = config

    def spawn(
        self,
        variant: str = "isr_camera",
        position: Tuple[float, float, float] = None,
        heading: float = 0.0,
        drone_id: int = None,
    ) -> Optional[str]:
        """
        Spawn an F-11 drone at the specified position with given heading.

        The drone registers an exclusion zone in the global spatial hash to prevent
        other objects (vehicles, people, trees) from spawning within its radius.
        For this to work correctly, the drone should be spawned FIRST before other
        objects in the environment.

        Args:
            variant: Drone variant name (e.g., "isr_camera", "base", "lidar")
            position: (x, y, z) position or None for random xy with default altitude
            heading: Heading angle in radians
            drone_id: Optional drone ID, auto-assigned if None

        Returns:
            Prim path of spawned drone, or None if no valid position found
        """
        if variant not in self.drone_configs:
            raise ValueError(f"Unknown drone variant: {variant}. "
                           f"Available: {list(self.drone_configs.keys())}")

        cfg = self.drone_configs[variant]

        # Auto-assign drone ID if not specified
        if drone_id is None:
            drone_id = self.drone_count

        # Determine spawn position using exclusion zone logic
        # This registers the position in the global spatial hash so other spawners
        # will avoid spawning within the drone's exclusion radius
        if position is None:
            # Get random xy position with exclusion zone
            result = self.get_spawn_position(None, cfg.exclusion_radius)
            if result is None:
                return None
            x, y = result
            z = cfg.spawn_altitude
        else:
            x, y, z = position
            # Register the provided position with exclusion zone
            # Use get_spawn_position to check validity and register
            result = self.get_spawn_position((x, y), cfg.exclusion_radius)
            if result is None:
                return None
            x, y = result

        # Create unique drone path
        drone_path = f"/World/Drones/F11_{variant}_{drone_id:04d}"

        # Build the drone USD structure
        self._build_drone(drone_path, cfg)

        # Position the drone
        drone_prim = self.stage.GetPrimAtPath(drone_path)
        drone_xform = UsdGeom.Xformable(drone_prim)
        drone_xform.ClearXformOpOrder()
        drone_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(x, y, z)
        )
        drone_xform.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            math.degrees(heading)
        )

        # Track spawned drone
        self.spawned_objects.append(drone_path)
        self.spawned_drones[drone_path] = {
            "id": drone_id,
            "variant": variant,
            "config": cfg,
            "position": (x, y, z),
            "heading": heading,
        }
        self.drone_count += 1

        return drone_path

    def _build_drone(self, drone_path: str, cfg: DroneConfig) -> None:
        """Build complete drone USD structure using component builders."""
        # Create root Xform
        prim_utils.create_prim(drone_path, "Xform")

        # Create base_link
        base_link_path = f"{drone_path}/base_link"
        prim_utils.create_prim(base_link_path, "Xform")

        # Setup physics on base_link
        self.physics.setup_articulation_root(
            base_link_path,
            cfg.mass,
            cfg.inertia,
            cfg.center_of_gravity,
        )

        # Create body visual geometry
        self.geometry.create_body_visual(base_link_path, cfg.body_size)

        # Create arm visuals
        for i, pos in enumerate(self.ROTOR_POSITIONS):
            self.geometry.create_arm_visual(base_link_path, i, pos, cfg.arm_length)

        # Create battery visual
        self.geometry.create_battery_visual(base_link_path)

        # Create collision geometry
        self.geometry.create_collision_box(
            base_link_path,
            cfg.wheelbase,
            cfg.body_size[2],
        )

        # Create rotors with joints
        for i in range(4):
            rotor_path = f"{drone_path}/rotor_{i}"
            joint_path = f"{drone_path}/Joints/rotor_{i}_joint"

            # Create rotor prim
            prim_utils.create_prim(rotor_path, "Xform")

            # Setup rotor physics
            self.physics.setup_rotor_rigid_body(
                rotor_path,
                cfg.rotor_mass,
                cfg.rotor_inertia,
                self.ROTOR_POSITIONS[i],
            )

            # Create rotor joint
            self.physics.create_rotor_joint(
                joint_path,
                base_link_path,
                rotor_path,
                self.ROTOR_POSITIONS[i],
                self.ROTOR_DIRECTIONS[i],
                cfg.joint_damping,
            )

            # Create rotor visuals
            self.geometry.create_rotor_visuals(rotor_path, cfg.prop_diameter)

        # Create sensor links
        self.payloads.create_sensor_links(drone_path)

        # Create payload if applicable
        if cfg.has_gimbal:
            self.payloads.create_gimbal(drone_path, cfg.payload_offset)
        elif cfg.has_lidar:
            self.payloads.create_lidar_mount(drone_path, cfg.payload_offset)
        elif cfg.has_camera and not cfg.has_gimbal:
            self.payloads.create_fixed_camera(drone_path, cfg.payload_offset)

    def spawn_formation(
        self,
        variant: str,
        center: Tuple[float, float, float],
        count: int,
        spacing: float = 5.0,
        formation: str = "line",
    ) -> List[str]:
        """Spawn multiple drones in formation (line, wedge, box, or circle)."""
        spawned = []
        for i in range(count):
            offset_x, offset_y = self._compute_formation_offset(
                i, count, spacing, formation
            )

            position = (
                center[0] + offset_x,
                center[1] + offset_y,
                center[2],
            )

            path = self.spawn(variant, position=position)
            spawned.append(path)

        return spawned

    def _compute_formation_offset(
        self,
        index: int,
        count: int,
        spacing: float,
        formation: str,
    ) -> Tuple[float, float]:
        """Compute x,y offset for a drone in formation."""
        if formation == "line":
            offset_x = (index - (count - 1) / 2) * spacing
            offset_y = 0
        elif formation == "wedge":
            row = int((-1 + (1 + 8 * index) ** 0.5) / 2)
            col = index - row * (row + 1) // 2
            offset_x = row * spacing
            offset_y = (col - row / 2) * spacing
        elif formation == "box":
            cols = int(count ** 0.5) or 1
            offset_x = (index % cols - (cols - 1) / 2) * spacing
            offset_y = (index // cols - (count // cols - 1) / 2) * spacing
        elif formation == "circle":
            angle = 2 * math.pi * index / count
            radius = spacing * count / (2 * math.pi) if count > 1 else 0
            offset_x = radius * math.cos(angle)
            offset_y = radius * math.sin(angle)
        else:
            offset_x = offset_y = 0

        return offset_x, offset_y

    def get_drone_info(self, drone_path: str) -> Optional[dict]:
        """Get information about a spawned drone."""
        return self.spawned_drones.get(drone_path)

    def clear_all(self) -> None:
        """Remove all spawned drones."""
        super().clear_all()
        self.spawned_drones.clear()
        self.drone_count = 0
