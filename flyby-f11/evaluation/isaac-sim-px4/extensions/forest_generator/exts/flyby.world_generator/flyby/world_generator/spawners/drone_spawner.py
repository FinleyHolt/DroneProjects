"""
Drone spawner for F-11 quadcopter variants.

Spawns fully articulated F-11 drones with physics simulation and PX4 integration.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import random
import math

from pxr import Gf, UsdGeom, UsdShade, Sdf, UsdPhysics, PhysxSchema
import omni.isaac.core.utils.prims as prim_utils

from .base_spawner import BaseSpawner, SpawnConfig


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
        self.spawned_drones: Dict[str, dict] = {}  # path -> drone info

        # Ensure parent prim exists
        if not self.stage.GetPrimAtPath("/World/Drones"):
            prim_utils.create_prim("/World/Drones", "Xform")

        # Create shared materials
        self._create_drone_materials()

    def _create_drone_materials(self) -> None:
        """Create reusable materials for drone components."""
        if not self.stage.GetPrimAtPath("/World/Looks"):
            prim_utils.create_prim("/World/Looks", "Xform")

        # Frame material (dark carbon fiber look)
        self._create_pbr_material(
            "/World/Looks/drone_frame_mat",
            color=(0.15, 0.15, 0.15),
            roughness=0.4,
            metallic=0.2,
        )

        # Propeller material
        self._create_pbr_material(
            "/World/Looks/drone_prop_mat",
            color=(0.08, 0.08, 0.08),
            roughness=0.6,
            metallic=0.1,
        )

        # Motor material (silver)
        self._create_pbr_material(
            "/World/Looks/drone_motor_mat",
            color=(0.7, 0.7, 0.72),
            roughness=0.3,
            metallic=0.8,
        )

        # Battery material (blue accent)
        self._create_pbr_material(
            "/World/Looks/drone_battery_mat",
            color=(0.1, 0.2, 0.5),
            roughness=0.5,
            metallic=0.1,
        )

    def _create_pbr_material(
        self,
        mat_path: str,
        color: Tuple[float, float, float],
        roughness: float = 0.5,
        metallic: float = 0.0,
    ) -> None:
        """Create a PBR material."""
        if self.stage.GetPrimAtPath(mat_path):
            return

        material = UsdShade.Material.Define(self.stage, mat_path)
        shader = UsdShade.Shader.Define(self.stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")

        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)

        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )

    def _bind_material(self, prim_path: str, mat_path: str) -> None:
        """Bind material to a prim."""
        prim = self.stage.GetPrimAtPath(prim_path)
        mat_prim = self.stage.GetPrimAtPath(mat_path)
        if prim.IsValid() and mat_prim.IsValid():
            material = UsdShade.Material(mat_prim)
            UsdShade.MaterialBindingAPI(prim).Bind(material)

    def register_variant(self, name: str, config: DroneConfig) -> None:
        """Register a new drone variant."""
        self.drone_configs[name] = config

    def spawn(
        self,
        variant: str = "isr_camera",
        position: Tuple[float, float, float] = None,
        heading: float = 0.0,
        drone_id: int = None,
    ) -> str:
        """
        Spawn an F-11 drone.

        Args:
            variant: Drone variant ("base", "isr_camera", "lidar", "multispectral")
            position: (x, y, z) spawn position in meters
            heading: Yaw angle in radians
            drone_id: Optional specific ID for multi-vehicle (auto-assigned if None)

        Returns:
            Prim path of spawned drone
        """
        if variant not in self.drone_configs:
            raise ValueError(f"Unknown drone variant: {variant}. "
                           f"Available: {list(self.drone_configs.keys())}")

        cfg = self.drone_configs[variant]

        # Auto-assign drone ID if not specified
        if drone_id is None:
            drone_id = self.drone_count

        # Determine spawn position
        if position is None:
            x, y = self.get_random_position(object_radius=cfg.wheelbase / 2)
            z = cfg.spawn_altitude
        else:
            x, y, z = position
            # For drones, we don't need ground-level overlap prevention
            # but we can still register for airspace awareness

        # Create unique drone path
        drone_path = f"/World/Drones/F11_{variant}_{drone_id:04d}"

        # Build the drone USD structure
        self._build_drone(drone_path, cfg, heading)

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

    def _build_drone(
        self,
        drone_path: str,
        cfg: DroneConfig,
        heading: float = 0.0,
    ) -> None:
        """Build complete drone USD structure."""
        # Create root Xform
        prim_utils.create_prim(drone_path, "Xform")

        # Create base_link as articulation root
        base_link_path = f"{drone_path}/base_link"
        self._create_base_link(base_link_path, cfg)

        # Create rotors with joints
        for i in range(4):
            rotor_path = f"{drone_path}/rotor_{i}"
            joint_path = f"{drone_path}/Joints/rotor_{i}_joint"
            self._create_rotor(
                rotor_path,
                joint_path,
                base_link_path,
                i,
                cfg,
            )

        # Create sensor links
        self._create_sensor_links(drone_path, cfg)

        # Create payload if applicable
        if cfg.has_gimbal:
            self._create_gimbal(drone_path, cfg)
        elif cfg.has_lidar:
            self._create_lidar_mount(drone_path, cfg)
        elif cfg.has_camera and not cfg.has_gimbal:
            self._create_fixed_camera(drone_path, cfg)

    def _create_base_link(self, prim_path: str, cfg: DroneConfig) -> None:
        """Create the main drone body as articulation root."""
        # Create Xform for base_link
        prim_utils.create_prim(prim_path, "Xform")
        prim = self.stage.GetPrimAtPath(prim_path)

        # Apply rigid body API
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)

        # Apply mass properties
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(cfg.mass)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*cfg.inertia))
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*cfg.center_of_gravity))

        # Apply articulation root
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        physx_articulation.CreateEnabledSelfCollisionsAttr(False)

        # Create visual geometry (central body box)
        body_visual_path = f"{prim_path}/body_visual"
        body_visual = UsdGeom.Cube.Define(self.stage, body_visual_path)
        body_xform = UsdGeom.Xformable(body_visual.GetPrim())
        body_xform.ClearXformOpOrder()
        body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(
                cfg.body_size[0] / 2,
                cfg.body_size[1] / 2,
                cfg.body_size[2] / 2,
            )
        )
        self._bind_material(body_visual_path, "/World/Looks/drone_frame_mat")

        # Create arms (4 cylindrical arms)
        for i, (rx, ry, rz) in enumerate(self.ROTOR_POSITIONS):
            arm_path = f"{prim_path}/arm_{i}_visual"
            arm = UsdGeom.Cylinder.Define(self.stage, arm_path)
            arm.CreateRadiusAttr(0.015)  # 15mm diameter arms
            arm.CreateHeightAttr(cfg.arm_length)
            arm.CreateAxisAttr("X")

            # Position arm from center toward rotor
            arm_xform = UsdGeom.Xformable(arm.GetPrim())
            arm_xform.ClearXformOpOrder()
            arm_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(rx / 2, ry / 2, rz)
            )
            # Rotate to point toward rotor
            angle = math.degrees(math.atan2(ry, rx))
            arm_xform.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(angle)

            self._bind_material(arm_path, "/World/Looks/drone_frame_mat")

        # Create battery visual
        battery_path = f"{prim_path}/battery_visual"
        battery = UsdGeom.Cube.Define(self.stage, battery_path)
        battery_xform = UsdGeom.Xformable(battery.GetPrim())
        battery_xform.ClearXformOpOrder()
        battery_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.08, 0.04, 0.025)  # 160x80x50mm battery
        )
        battery_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0, 0, -0.04)  # Under the frame
        )
        self._bind_material(battery_path, "/World/Looks/drone_battery_mat")

        # Add collision geometry (simplified box)
        collision_path = f"{prim_path}/collision"
        collision = UsdGeom.Cube.Define(self.stage, collision_path)
        collision_xform = UsdGeom.Xformable(collision.GetPrim())
        collision_xform.ClearXformOpOrder()
        collision_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(cfg.wheelbase / 2, cfg.wheelbase / 2, cfg.body_size[2])
        )

        # Make collision invisible but enable collision API
        collision.GetPrim().GetAttribute("visibility").Set("invisible")
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())

    def _create_rotor(
        self,
        rotor_path: str,
        joint_path: str,
        parent_path: str,
        rotor_idx: int,
        cfg: DroneConfig,
    ) -> None:
        """Create a rotor with revolute joint."""
        pos = self.ROTOR_POSITIONS[rotor_idx]
        direction = self.ROTOR_DIRECTIONS[rotor_idx]

        # Create rotor rigid body
        prim_utils.create_prim(rotor_path, "Xform")
        rotor_prim = self.stage.GetPrimAtPath(rotor_path)

        # Apply rigid body
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(rotor_prim)

        # Apply mass (prop + motor)
        mass_api = UsdPhysics.MassAPI.Apply(rotor_prim)
        mass_api.CreateMassAttr(cfg.rotor_mass)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*cfg.rotor_inertia))

        # Position rotor - clear existing ops first then set position
        rotor_xform = UsdGeom.Xformable(rotor_prim)
        rotor_xform.ClearXformOpOrder()
        rotor_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*pos)
        )

        # Create propeller visual (disc/cylinder)
        prop_path = f"{rotor_path}/prop_visual"
        prop = UsdGeom.Cylinder.Define(self.stage, prop_path)
        prop.CreateRadiusAttr(cfg.prop_diameter / 2)
        prop.CreateHeightAttr(0.005)  # 5mm thick
        prop.CreateAxisAttr("Z")
        self._bind_material(prop_path, "/World/Looks/drone_prop_mat")

        # Create motor visual (small cylinder)
        motor_path = f"{rotor_path}/motor_visual"
        motor = UsdGeom.Cylinder.Define(self.stage, motor_path)
        motor.CreateRadiusAttr(0.02)  # 20mm radius motor
        motor.CreateHeightAttr(0.03)  # 30mm tall
        motor.CreateAxisAttr("Z")
        motor_xform = UsdGeom.Xformable(motor.GetPrim())
        motor_xform.ClearXformOpOrder()
        motor_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0, 0, -0.02)  # Below prop
        )
        self._bind_material(motor_path, "/World/Looks/drone_motor_mat")

        # Create Joints parent if not exists
        joints_parent = f"{'/'.join(rotor_path.split('/')[:-1])}/Joints"
        if not self.stage.GetPrimAtPath(joints_parent):
            prim_utils.create_prim(joints_parent, "Xform")

        # Create revolute joint
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, joint_path)
        joint.CreateAxisAttr("Z")
        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([rotor_path])

        # Set local transforms for joint
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*pos))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        # No limits for continuous rotation
        joint.CreateLowerLimitAttr(-1e10)
        joint.CreateUpperLimitAttr(1e10)

        # Add angular drive for velocity control
        drive_api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive_api.CreateTypeAttr("force")
        drive_api.CreateDampingAttr(cfg.joint_damping)
        drive_api.CreateStiffnessAttr(0.0)  # Pure velocity control
        drive_api.CreateMaxForceAttr(1000.0)  # High max force

        # Store rotor direction as custom attribute
        joint.GetPrim().CreateAttribute(
            "flyby:rotorDirection",
            Sdf.ValueTypeNames.Int
        ).Set(direction)

    def _create_sensor_links(self, drone_path: str, cfg: DroneConfig) -> None:
        """Create sensor frame links (IMU, GPS)."""
        base_link_path = f"{drone_path}/base_link"

        # IMU link (at base_link origin)
        imu_path = f"{drone_path}/imu_link"
        prim_utils.create_prim(imu_path, "Xform")

        # GPS link (antenna on top)
        gps_path = f"{drone_path}/gps_link"
        prim_utils.create_prim(gps_path, "Xform")
        gps_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(gps_path))
        gps_xform.ClearXformOpOrder()
        gps_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0, 0, 0.10)  # 10cm above base_link
        )

        # GPS antenna visual (small cylinder)
        gps_visual_path = f"{gps_path}/antenna_visual"
        gps_visual = UsdGeom.Cylinder.Define(self.stage, gps_visual_path)
        gps_visual.CreateRadiusAttr(0.03)
        gps_visual.CreateHeightAttr(0.015)
        gps_visual.CreateAxisAttr("Z")

    def _create_gimbal(self, drone_path: str, cfg: DroneConfig) -> None:
        """Create 3-axis gimbal with camera for ISR variant."""
        base_link_path = f"{drone_path}/base_link"
        joints_path = f"{drone_path}/Joints"

        # Gimbal mount link
        mount_path = f"{drone_path}/gimbal_mount_link"
        prim_utils.create_prim(mount_path, "Xform")
        mount_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(mount_path))
        mount_xform.ClearXformOpOrder()
        mount_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*cfg.payload_offset)
        )

        # Yaw link + joint
        yaw_path = f"{drone_path}/gimbal_yaw_link"
        prim_utils.create_prim(yaw_path, "Xform")

        yaw_joint = UsdPhysics.RevoluteJoint.Define(
            self.stage, f"{joints_path}/gimbal_yaw_joint"
        )
        yaw_joint.CreateAxisAttr("Z")
        yaw_joint.CreateBody0Rel().SetTargets([mount_path])
        yaw_joint.CreateBody1Rel().SetTargets([yaw_path])
        yaw_joint.CreateLowerLimitAttr(-345.0)  # degrees
        yaw_joint.CreateUpperLimitAttr(345.0)

        # Pitch link + joint
        pitch_path = f"{drone_path}/gimbal_pitch_link"
        prim_utils.create_prim(pitch_path, "Xform")

        pitch_joint = UsdPhysics.RevoluteJoint.Define(
            self.stage, f"{joints_path}/gimbal_pitch_joint"
        )
        pitch_joint.CreateAxisAttr("Y")
        pitch_joint.CreateBody0Rel().SetTargets([yaw_path])
        pitch_joint.CreateBody1Rel().SetTargets([pitch_path])
        pitch_joint.CreateLowerLimitAttr(-120.0)
        pitch_joint.CreateUpperLimitAttr(120.0)

        # Camera link + roll joint
        camera_link_path = f"{drone_path}/camera_link"
        prim_utils.create_prim(camera_link_path, "Xform")

        roll_joint = UsdPhysics.RevoluteJoint.Define(
            self.stage, f"{joints_path}/camera_roll_joint"
        )
        roll_joint.CreateAxisAttr("X")
        roll_joint.CreateBody0Rel().SetTargets([pitch_path])
        roll_joint.CreateBody1Rel().SetTargets([camera_link_path])
        roll_joint.CreateLowerLimitAttr(-45.0)
        roll_joint.CreateUpperLimitAttr(45.0)

        # Camera visual (box representing camera body)
        camera_visual_path = f"{camera_link_path}/camera_visual"
        camera_visual = UsdGeom.Cube.Define(self.stage, camera_visual_path)
        camera_xform = UsdGeom.Xformable(camera_visual.GetPrim())
        camera_xform.ClearXformOpOrder()
        camera_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.064, 0.047, 0.024)  # ~127x94x48mm
        )
        self._bind_material(camera_visual_path, "/World/Looks/drone_frame_mat")

    def _create_lidar_mount(self, drone_path: str, cfg: DroneConfig) -> None:
        """Create LiDAR sensor mount."""
        # LiDAR link
        lidar_path = f"{drone_path}/lidar_link"
        prim_utils.create_prim(lidar_path, "Xform")
        lidar_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(lidar_path))
        lidar_xform.ClearXformOpOrder()
        lidar_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*cfg.payload_offset)
        )

        # LiDAR visual (cylinder representing VLP-16)
        lidar_visual_path = f"{lidar_path}/lidar_visual"
        lidar_visual = UsdGeom.Cylinder.Define(self.stage, lidar_visual_path)
        lidar_visual.CreateRadiusAttr(0.052)  # VLP-16 diameter
        lidar_visual.CreateHeightAttr(0.072)  # VLP-16 height
        lidar_visual.CreateAxisAttr("Z")
        self._bind_material(lidar_visual_path, "/World/Looks/drone_frame_mat")

    def _create_fixed_camera(self, drone_path: str, cfg: DroneConfig) -> None:
        """Create fixed downward camera (multispectral variant)."""
        camera_path = f"{drone_path}/camera_link"
        prim_utils.create_prim(camera_path, "Xform")
        camera_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(camera_path))
        camera_xform.ClearXformOpOrder()
        camera_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*cfg.payload_offset)
        )
        # Point downward
        camera_xform.AddRotateXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(90.0)

        # Camera visual
        camera_visual_path = f"{camera_path}/camera_visual"
        camera_visual = UsdGeom.Cube.Define(self.stage, camera_visual_path)
        camera_vis_xform = UsdGeom.Xformable(camera_visual.GetPrim())
        camera_vis_xform.ClearXformOpOrder()
        camera_vis_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.04, 0.03, 0.02)  # Compact multispectral camera
        )
        self._bind_material(camera_visual_path, "/World/Looks/drone_frame_mat")

    def spawn_formation(
        self,
        variant: str,
        center: Tuple[float, float, float],
        count: int,
        spacing: float = 5.0,
        formation: str = "line",
    ) -> List[str]:
        """
        Spawn multiple drones in formation.

        Args:
            variant: Drone variant to spawn
            center: Center position (x, y, z)
            count: Number of drones
            spacing: Distance between drones in meters
            formation: "line", "wedge", "box", or "circle"

        Returns:
            List of spawned drone prim paths
        """
        spawned = []

        for i in range(count):
            if formation == "line":
                offset_x = (i - (count - 1) / 2) * spacing
                offset_y = 0
            elif formation == "wedge":
                row = int((-1 + (1 + 8 * i) ** 0.5) / 2)
                col = i - row * (row + 1) // 2
                offset_x = row * spacing
                offset_y = (col - row / 2) * spacing
            elif formation == "box":
                cols = int(count ** 0.5) or 1
                offset_x = (i % cols - (cols - 1) / 2) * spacing
                offset_y = (i // cols - (count // cols - 1) / 2) * spacing
            elif formation == "circle":
                angle = 2 * math.pi * i / count
                radius = spacing * count / (2 * math.pi) if count > 1 else 0
                offset_x = radius * math.cos(angle)
                offset_y = radius * math.sin(angle)
            else:
                offset_x = offset_y = 0

            position = (
                center[0] + offset_x,
                center[1] + offset_y,
                center[2],
            )

            path = self.spawn(variant, position=position)
            spawned.append(path)

        return spawned

    def get_drone_info(self, drone_path: str) -> Optional[dict]:
        """Get information about a spawned drone."""
        return self.spawned_drones.get(drone_path)

    def clear_all(self) -> None:
        """Remove all spawned drones."""
        super().clear_all()
        self.spawned_drones.clear()
        self.drone_count = 0
