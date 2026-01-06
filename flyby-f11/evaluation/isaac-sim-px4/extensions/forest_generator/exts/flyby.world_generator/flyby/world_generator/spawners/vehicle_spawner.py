"""
Vehicle spawner for cars, trucks, and military vehicles.

Supports both custom USD models and procedurally generated vehicles.
"""

from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import random

from pxr import Gf, UsdGeom, UsdShade, Sdf, UsdPhysics
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.core.utils.stage import add_reference_to_stage

from .base_spawner import BaseSpawner, SpawnConfig


# Discrete yaw angles (24 total, 15-degree increments)
# Figures can ONLY spawn at these orientations to ensure upright poses
DISCRETE_YAW_ANGLES = tuple(i * 15.0 for i in range(24))  # 0, 15, 30, ... 345


@dataclass
class VehicleConfig:
    """Configuration for vehicle spawning."""
    usd_path: str = None  # Optional - if None, use procedural
    category: str = "civilian"
    base_scale: float = 1.0
    scale_variation: tuple = (0.95, 1.05)
    spawn_on_ground: bool = True
    ground_offset: float = 0.0
    # Procedural vehicle dimensions (meters)
    length: float = 4.5
    width: float = 1.8
    height: float = 1.5
    color: Tuple[float, float, float] = (0.3, 0.3, 0.35)
    # Rotation offset to fix model orientation (degrees)
    rotation_offset: Tuple[float, float, float] = (0, 0, 0)  # (rx, ry, rz)


# Quaternius GLB models (relative to models_path)
# Vehicles: Cop, NormalCar1, NormalCar2, SportsCar, SportsCar2, SUV, Taxi
# Tanks: Tank, Tank2, Tank3, Tank4

# Default vehicle types for ISR training
# Unit conversion handled by Isaac Sim GLB importer
# Fine-tuned scales based on visual inspection:
#   - Cars: 2.25x scale (50% bigger than previous 1.5)
#   - Tanks: 15% smaller for realistic proportions
DEFAULT_VEHICLES = {
    # Civilian vehicles - scale 2.25x
    # Note: Quaternius GLB models have varied coordinate systems
    # Isaac Sim's GLB importer handles Y-up to Z-up conversion, but individual
    # models may need additional rotation. Verify each model visually.
    # rotation_offset is applied BEFORE random heading rotation.
    # NOTE: NormalCar1.glb (sedan) removed - model has broken orientation that
    # cannot be fixed with rotation offsets. Use sedan2 (NormalCar2) instead.
    # Cars use 7m exclusion zone (reduced 30% from 10m since kinematic physics prevents tipping)
    "sedan2": VehicleConfig(
        usd_path="vehicles/glb/NormalCar2.glb",
        category="civilian",
        base_scale=2.25,
        length=7.0,
        width=7.0,
    ),
    "suv": VehicleConfig(
        usd_path="vehicles/glb/SUV.glb",
        category="civilian",
        base_scale=2.3,
        length=7.0,
        width=7.0,
    ),
    "sports_car": VehicleConfig(
        usd_path="vehicles/glb/SportsCar.glb",
        category="civilian",
        base_scale=2.2,
        length=7.0,
        width=7.0,
    ),
    "sports_car2": VehicleConfig(
        usd_path="vehicles/glb/SportsCar2.glb",
        category="civilian",
        base_scale=2.2,
        length=7.0,
        width=7.0,
    ),
    "taxi": VehicleConfig(
        usd_path="vehicles/glb/Taxi.glb",
        category="civilian",
        base_scale=2.25,
        length=7.0,
        width=7.0,
    ),
    "police": VehicleConfig(
        usd_path="vehicles/glb/Cop.glb",
        category="civilian",
        base_scale=2.25,
        length=7.0,
        width=7.0,
    ),
    # Military vehicles - scale ~1.0 (0.85 * 1.15 ≈ 0.98)
    # All tanks face +X instead of +Y, need +90 Z rotation to face forward
    # Use large exclusion zone (20m diameter) to keep trees/objects well clear
    "tank": VehicleConfig(
        usd_path="tanks/glb/Tank.glb",
        category="military",
        base_scale=1.0,
        rotation_offset=(0, 0, 90),  # Faces +X, rotate +90 Z to face +Y (forward)
        length=20.0,  # Large exclusion zone for tanks
        width=20.0,
    ),
    "tank2": VehicleConfig(
        usd_path="tanks/glb/Tank2.glb",
        category="military",
        base_scale=1.0,
        rotation_offset=(0, 0, 90),  # Faces +X, rotate +90 Z to face +Y (forward)
        length=20.0,
        width=20.0,
    ),
    "tank3": VehicleConfig(
        usd_path="tanks/glb/Tank3.glb",
        category="military",
        base_scale=1.0,
        rotation_offset=(0, 0, 90),  # Faces +X, rotate +90 Z to face +Y (forward)
        length=20.0,
        width=20.0,
    ),
    "tank4": VehicleConfig(
        usd_path="tanks/glb/Tank4.glb",
        category="military",
        base_scale=1.0,
        rotation_offset=(0, 0, 90),  # Faces +X, rotate +90 Z to face +Y (forward)
        length=20.0,
        width=20.0,
    ),
}


class VehicleSpawner(BaseSpawner):
    """
    Spawner for vehicles in ISR training environments.

    Supports:
    - Procedural vehicles (simple geometry, good for RL training)
    - Civilian vehicles (cars, trucks, SUVs)
    - Military vehicles (Humvees, trucks, APCs)
    - Custom USD models

    For ISR training, vehicles should be:
    - Visible from aerial perspective
    - Varied in color/type for detection challenges
    - Simple enough for fast rendering
    """

    def __init__(
        self,
        stage,
        models_path: str = None,
        config: SpawnConfig = None,
        vehicle_configs: Dict[str, VehicleConfig] = None,
    ):
        super().__init__(stage, config)
        self.models_path = models_path
        self.vehicle_configs = vehicle_configs or DEFAULT_VEHICLES.copy()
        self.vehicle_count = 0

        # Ensure parent prim exists
        if not self.stage.GetPrimAtPath("/World/Vehicles"):
            prim_utils.create_prim("/World/Vehicles", "Xform")

        # Create materials for vehicles
        self._create_vehicle_materials()

    def _create_vehicle_materials(self) -> None:
        """Create reusable materials for procedural vehicles."""
        if not self.stage.GetPrimAtPath("/World/Looks"):
            prim_utils.create_prim("/World/Looks", "Xform")

        self.vehicle_materials = {}

        for name, cfg in self.vehicle_configs.items():
            mat_path = f"/World/Looks/vehicle_{name}_mat"
            if self.stage.GetPrimAtPath(mat_path):
                continue

            material = UsdShade.Material.Define(self.stage, mat_path)
            shader = UsdShade.Shader.Define(self.stage, f"{mat_path}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")

            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*cfg.color)
            )
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.1)

            material.CreateSurfaceOutput().ConnectToSource(
                shader.ConnectableAPI(), "surface"
            )

            self.vehicle_materials[name] = mat_path

    def _create_procedural_vehicle(
        self,
        vehicle_path: str,
        cfg: VehicleConfig,
        vehicle_type: str,
    ) -> None:
        """Create a simple procedural vehicle using USD primitives."""
        # Main body (box)
        body_path = f"{vehicle_path}/body"
        body = UsdGeom.Cube.Define(self.stage, body_path)

        # Scale cube to vehicle dimensions
        # Cube is 2x2x2 centered at origin, so we scale to half dimensions
        body_xform = UsdGeom.Xformable(body.GetPrim())
        body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(
            cfg.length / 2,
            cfg.width / 2,
            cfg.height / 2
        ))
        # Lift body so bottom is at z=0
        body_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, cfg.height / 2))

        # Apply material
        if vehicle_type in self.vehicle_materials:
            mat = self.stage.GetPrimAtPath(self.vehicle_materials[vehicle_type])
            if mat.IsValid():
                material = UsdShade.Material(mat)
                UsdShade.MaterialBindingAPI(body.GetPrim()).Bind(material)

        # Add wheels (4 cylinders)
        wheel_radius = 0.35
        wheel_width = 0.25
        wheel_positions = [
            (cfg.length * 0.35, cfg.width / 2, wheel_radius),   # Front right
            (cfg.length * 0.35, -cfg.width / 2, wheel_radius),  # Front left
            (-cfg.length * 0.35, cfg.width / 2, wheel_radius),  # Rear right
            (-cfg.length * 0.35, -cfg.width / 2, wheel_radius), # Rear left
        ]

        for i, (wx, wy, wz) in enumerate(wheel_positions):
            wheel_path = f"{vehicle_path}/wheel_{i}"
            wheel = UsdGeom.Cylinder.Define(self.stage, wheel_path)
            wheel.CreateRadiusAttr(wheel_radius)
            wheel.CreateHeightAttr(wheel_width)
            wheel.CreateAxisAttr("Y")

            wheel_xform = UsdGeom.Xformable(wheel.GetPrim())
            wheel_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(wx, wy, wz))

            # Dark wheel color
            wheel_mat_path = "/World/Looks/wheel_mat"
            if not self.stage.GetPrimAtPath(wheel_mat_path):
                wheel_mat = UsdShade.Material.Define(self.stage, wheel_mat_path)
                wheel_shader = UsdShade.Shader.Define(self.stage, f"{wheel_mat_path}/Shader")
                wheel_shader.CreateIdAttr("UsdPreviewSurface")
                wheel_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(0.1, 0.1, 0.1)
                )
                wheel_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
                wheel_mat.CreateSurfaceOutput().ConnectToSource(
                    wheel_shader.ConnectableAPI(), "surface"
                )

            wheel_material = UsdShade.Material(self.stage.GetPrimAtPath(wheel_mat_path))
            UsdShade.MaterialBindingAPI(wheel.GetPrim()).Bind(wheel_material)

    def register_vehicle(self, name: str, config: VehicleConfig) -> None:
        """Register a new vehicle type."""
        self.vehicle_configs[name] = config

    def register_custom_model(
        self,
        name: str,
        usd_path: str,
        category: str = "civilian",
        scale: float = 1.0,
    ) -> None:
        """
        Register a custom USD vehicle model.

        Args:
            name: Identifier for this vehicle type
            usd_path: Path to USD file
            category: "civilian" or "military"
            scale: Base scale factor
        """
        self.vehicle_configs[name] = VehicleConfig(
            usd_path=usd_path,
            category=category,
            base_scale=scale,
        )

    def spawn_vehicle(
        self,
        vehicle_type: str,
        position: Tuple[float, float] = None,
        heading: float = None,
    ) -> Optional[str]:
        """
        Spawn a single vehicle.

        Args:
            vehicle_type: Type of vehicle from registered configs
            position: Optional (x, y) position
            heading: Optional heading in degrees (0 = +X axis)

        Returns:
            Prim path of spawned vehicle, or None if no valid position available
        """
        if vehicle_type not in self.vehicle_configs:
            raise ValueError(f"Unknown vehicle type: {vehicle_type}. "
                           f"Available: {list(self.vehicle_configs.keys())}")

        cfg = self.vehicle_configs[vehicle_type]

        # Vehicle approximate radius for overlap prevention
        vehicle_radius = max(cfg.length, cfg.width) / 2 * cfg.base_scale

        if position is None:
            result = self.get_random_position(object_radius=vehicle_radius)
            if result is None:
                return None  # No valid position, skip spawning
            x, y = result
        else:
            # Even with explicit position, find valid nearby spot to avoid overlap
            result = self.find_valid_position(position[0], position[1], vehicle_radius)
            if result is None:
                # No valid position found - skip spawning this vehicle
                return None
            x, y = result

        # Register position to prevent future overlaps
        self.register_position(x, y, vehicle_radius)

        if heading is None:
            # Use discrete yaw angles only (15-degree increments)
            # This ensures vehicles spawn upright with only Z-axis rotation
            heading = random.choice(DISCRETE_YAW_ANGLES)
        else:
            # Snap provided heading to nearest discrete angle
            heading = min(DISCRETE_YAW_ANGLES, key=lambda x: abs((x - heading + 180) % 360 - 180))

        scale_var = random.uniform(*cfg.scale_variation)
        scale = cfg.base_scale * scale_var

        vehicle_path = f'/World/Vehicles/{vehicle_type}_{self.vehicle_count:04d}'
        prim_utils.create_prim(vehicle_path, "Xform")

        # Use USD/GLB model if provided, otherwise create procedural vehicle
        if cfg.usd_path:
            import os
            # Resolve relative paths against models_path
            if not os.path.isabs(cfg.usd_path) and self.models_path:
                full_path = os.path.join(self.models_path, cfg.usd_path)
            else:
                full_path = cfg.usd_path

            if os.path.exists(full_path):
                add_reference_to_stage(usd_path=full_path, prim_path=vehicle_path)
            else:
                print(f"  Warning: Vehicle model not found: {full_path}, using procedural")
                self._create_procedural_vehicle(vehicle_path, cfg, vehicle_type)
        else:
            self._create_procedural_vehicle(vehicle_path, cfg, vehicle_type)

        # Get ground height
        if cfg.spawn_on_ground:
            z = self.raycast_to_ground(x, y) + cfg.ground_offset
        else:
            z = cfg.ground_offset

        # Apply transforms to the parent Xform
        vehicle_prim = self.stage.GetPrimAtPath(vehicle_path)
        vehicle_xform = UsdGeom.Xformable(vehicle_prim)

        # Clear existing ops and add new ones with explicit double precision
        vehicle_xform.ClearXformOpOrder()

        vehicle_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, z))

        # Apply rotation offset to fix model orientation, with yaw-only heading
        # IMPORTANT: We use a quaternion approach to ensure vehicles stay upright.
        # The rotation_offset fixes the model's default orientation (e.g., +90 X to bring wheels down).
        # The heading (yaw) rotates the vehicle around the WORLD Z-axis only.
        # Order: First apply model orientation fix, then yaw around world Z.
        rx, ry, rz = cfg.rotation_offset

        # Compute quaternion for model orientation fix (rotation_offset)
        qw_offset, qx_offset, qy_offset, qz_offset = self.euler_to_quaternion(rx, ry, rz)
        q_offset = Gf.Quatd(qw_offset, qx_offset, qy_offset, qz_offset)

        # Compute quaternion for yaw-only heading (rotation around world Z-axis)
        qw_yaw, qx_yaw, qy_yaw, qz_yaw = self.euler_to_quaternion(0, 0, heading)
        q_yaw = Gf.Quatd(qw_yaw, qx_yaw, qy_yaw, qz_yaw)

        # Compose: first model offset, then yaw. In quaternion math, q_final = q_yaw * q_offset
        # This applies offset first (in local frame), then yaw (in world frame)
        q_final = q_yaw * q_offset

        vehicle_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(q_final)

        vehicle_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(scale, scale, scale))

        # Make vehicle kinematic to prevent physics from tipping it over
        # Kinematic bodies maintain their position/orientation and don't respond to forces
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(vehicle_prim)
        rigid_body_api.CreateKinematicEnabledAttr(True)

        self.spawned_objects.append(vehicle_path)
        self.vehicle_count += 1

        return vehicle_path

    def spawn_vehicle_group(
        self,
        vehicle_types: List[str],
        count: int,
        clustering: float = 0.0,
        center: Tuple[float, float] = None,
    ) -> List[str]:
        """
        Spawn a group of vehicles.

        Args:
            vehicle_types: List of vehicle types to randomly select from
            count: Number of vehicles to spawn
            clustering: 0.0 = random placement, 1.0 = tight clustering
            center: Optional explicit (x, y) center for cluster. If None,
                   first vehicle position becomes the center.

        Returns:
            List of spawned vehicle prim paths
        """
        spawned = []
        center_x, center_y = center if center else (None, None)

        for i in range(count):
            vehicle_type = random.choice(vehicle_types)

            if center_x is not None:
                if i == 0 and center is not None:
                    # First vehicle at explicit center
                    position = (center_x, center_y)
                else:
                    # Subsequent vehicles clustered around center
                    # Max spread is 30m when clustering=0, minimum 5m spread always
                    spread = (1.0 - clustering) * 30.0
                    offset_x = random.gauss(0, max(5.0, spread))
                    offset_y = random.gauss(0, max(5.0, spread))
                    position = (center_x + offset_x, center_y + offset_y)
            else:
                position = None

            path = self.spawn_vehicle(vehicle_type, position=position)
            if path is None:
                # No valid position found, skip this vehicle
                continue
            spawned.append(path)

            if i == 0 and center is None:
                # Record center from first vehicle for subsequent clustering
                prim = self.stage.GetPrimAtPath(path)
                if prim.IsValid():
                    translate = prim_utils.get_prim_property(path, "xformOp:translate")
                    if translate:
                        center_x, center_y = translate[0], translate[1]

        return spawned

    def spawn_convoy(
        self,
        vehicle_types: List[str],
        start_pos: Tuple[float, float],
        direction: float,
        spacing: float = 10.0,
    ) -> List[str]:
        """
        Spawn vehicles in a convoy formation.

        Args:
            vehicle_types: List of vehicle types (spawned in order)
            start_pos: Starting (x, y) position
            direction: Heading in degrees
            spacing: Distance between vehicles

        Returns:
            List of spawned vehicle prim paths
        """
        spawned = []
        direction_rad = direction * (3.14159 / 180)

        dx = spacing * (-1) * (direction_rad + 3.14159 / 2)
        dy = spacing * (direction_rad + 3.14159 / 2)

        for i, vehicle_type in enumerate(vehicle_types):
            pos_x = start_pos[0] + i * dx
            pos_y = start_pos[1] + i * dy
            path = self.spawn_vehicle(vehicle_type, position=(pos_x, pos_y), heading=direction)
            if path is not None:
                spawned.append(path)

        return spawned
