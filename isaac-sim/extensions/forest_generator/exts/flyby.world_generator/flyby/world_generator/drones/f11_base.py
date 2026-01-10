"""
F-11 Base Drone USD Builder.

Creates complete USD articulation for F-11 quadcopter with accurate physics.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math

from pxr import Gf, UsdGeom, UsdShade, Sdf, UsdPhysics, PhysxSchema


@dataclass
class F11Specs:
    """Physical specifications for F-11 drone variants."""
    # Variant name
    name: str = "base"

    # Mass properties
    total_mass: float = 4.5  # kg
    inertia_xx: float = 0.085  # kg*m^2
    inertia_yy: float = 0.085
    inertia_zz: float = 0.145
    cog_x: float = 0.0  # m
    cog_y: float = 0.0
    cog_z: float = 0.0

    # Frame geometry
    wheelbase: float = 0.55  # m diagonal
    arm_length: float = 0.195  # m from center
    body_length: float = 0.30  # m
    body_width: float = 0.30
    body_height: float = 0.12

    # Propulsion
    prop_diameter: float = 0.38  # m (15")
    motor_kv: float = 340  # KV rating
    max_rpm: float = 8000
    max_thrust_per_motor: float = 30.9  # N (for ~2x T/W at 6.3kg)

    # Rotor properties
    rotor_mass: float = 0.025  # kg (prop + motor bell)
    rotor_ixx: float = 1e-5  # kg*m^2
    rotor_iyy: float = 5e-4
    rotor_izz: float = 5e-4

    # Payload
    payload_mass: float = 0.0  # kg
    payload_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Sensors enabled
    has_gimbal: bool = False
    has_lidar: bool = False
    has_camera: bool = False

    # Gimbal specs (if applicable)
    gimbal_yaw_range: float = 345.0  # degrees
    gimbal_pitch_range: float = 120.0
    gimbal_roll_range: float = 45.0


# Pre-defined F-11 variants
F11_SPECS = {
    "base": F11Specs(
        name="base",
        total_mass=4.5,
        inertia_xx=0.085,
        inertia_yy=0.085,
        inertia_zz=0.145,
    ),
    "isr_camera": F11Specs(
        name="isr_camera",
        total_mass=6.3,
        inertia_xx=0.115,
        inertia_yy=0.115,
        inertia_zz=0.175,
        cog_z=-0.05,
        payload_mass=1.8,
        payload_offset=(0.0, 0.0, -0.10),
        has_gimbal=True,
        has_camera=True,
    ),
    "lidar": F11Specs(
        name="lidar",
        total_mass=5.25,
        inertia_xx=0.098,
        inertia_yy=0.098,
        inertia_zz=0.158,
        cog_z=-0.02,
        payload_mass=0.75,
        payload_offset=(0.0, 0.0, -0.08),
        has_lidar=True,
    ),
    "multispectral": F11Specs(
        name="multispectral",
        total_mass=4.95,
        inertia_xx=0.090,
        inertia_yy=0.090,
        inertia_zz=0.152,
        cog_z=-0.01,
        payload_mass=0.45,
        payload_offset=(0.0, 0.0, -0.06),
        has_camera=True,
    ),
}


class F11DroneBuilder:
    """
    Builder class for creating F-11 drone USD prims.

    Creates complete articulated drone with:
    - Main body with accurate mass/inertia
    - 4 rotor assemblies with revolute joints
    - Optional gimbal/camera/LiDAR payloads
    - Sensor links (IMU, GPS, barometer)
    """

    # Rotor positions in X-configuration (m from base_link origin)
    # Order matches PX4 motor mapping: FR(CW), BL(CW), FL(CCW), BR(CCW)
    ROTOR_POSITIONS = [
        (+0.195, -0.195, +0.04),  # Motor 0: Front-Right
        (-0.195, +0.195, +0.04),  # Motor 1: Back-Left
        (+0.195, +0.195, +0.04),  # Motor 2: Front-Left
        (-0.195, -0.195, +0.04),  # Motor 3: Back-Right
    ]

    # Spin directions: 1 = CW (thrust up with CW rotation), -1 = CCW
    ROTOR_DIRECTIONS = [1, 1, -1, -1]

    def __init__(self, stage, specs: F11Specs = None):
        """
        Initialize drone builder.

        Args:
            stage: USD stage to build on
            specs: F11Specs dataclass with physical parameters
        """
        self.stage = stage
        self.specs = specs or F11_SPECS["base"]

    def build(self, root_path: str) -> str:
        """
        Build complete drone USD structure.

        Args:
            root_path: USD path for drone root prim (e.g., "/World/Drones/F11_0")

        Returns:
            Path to base_link prim
        """
        # Create root Xform
        self._ensure_prim(root_path, "Xform")

        # Build main body
        base_link_path = f"{root_path}/base_link"
        self._build_base_link(base_link_path)

        # Build rotors with joints
        joints_path = f"{root_path}/Joints"
        self._ensure_prim(joints_path, "Xform")

        for i in range(4):
            rotor_path = f"{root_path}/rotor_{i}"
            joint_path = f"{joints_path}/rotor_{i}_joint"
            self._build_rotor(rotor_path, joint_path, base_link_path, i)

        # Build sensor links
        self._build_sensor_links(root_path)

        # Build payload (if configured)
        if self.specs.has_gimbal:
            self._build_gimbal_assembly(root_path, joints_path)
        elif self.specs.has_lidar:
            self._build_lidar_mount(root_path)
        elif self.specs.has_camera:
            self._build_fixed_camera(root_path)

        return base_link_path

    def _ensure_prim(self, path: str, prim_type: str) -> None:
        """Create prim if it doesn't exist."""
        if not self.stage.GetPrimAtPath(path):
            self.stage.DefinePrim(path, prim_type)

    def _build_base_link(self, path: str) -> None:
        """Build main body rigid body with articulation root."""
        self._ensure_prim(path, "Xform")
        prim = self.stage.GetPrimAtPath(path)

        # Rigid body physics
        UsdPhysics.RigidBodyAPI.Apply(prim)

        # Mass properties
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(self.specs.total_mass)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(
            self.specs.inertia_xx,
            self.specs.inertia_yy,
            self.specs.inertia_zz,
        ))
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(
            self.specs.cog_x,
            self.specs.cog_y,
            self.specs.cog_z,
        ))

        # Articulation root (for joint-based control)
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        physx_art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        physx_art.CreateEnabledSelfCollisionsAttr(False)
        physx_art.CreateSolverPositionIterationCountAttr(16)
        physx_art.CreateSolverVelocityIterationCountAttr(4)

        # Visual geometry - central body
        self._build_body_visual(path)

        # Arms
        for i, pos in enumerate(self.ROTOR_POSITIONS):
            self._build_arm_visual(path, i, pos)

        # Battery housing
        self._build_battery_visual(path)

        # Collision geometry
        self._build_body_collision(path)

    def _build_body_visual(self, parent_path: str) -> None:
        """Build central body visual."""
        path = f"{parent_path}/body_visual"
        body = UsdGeom.Cube.Define(self.stage, path)

        xform = UsdGeom.Xformable(body.GetPrim())
        xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(
                self.specs.body_length / 2,
                self.specs.body_width / 2,
                self.specs.body_height / 2,
            )
        )

        # Dark carbon fiber appearance
        body.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.15, 0.15, 0.15)])

    def _build_arm_visual(
        self,
        parent_path: str,
        arm_idx: int,
        rotor_pos: Tuple[float, float, float],
    ) -> None:
        """Build arm connecting body to motor."""
        path = f"{parent_path}/arm_{arm_idx}_visual"
        arm = UsdGeom.Cylinder.Define(self.stage, path)
        arm.CreateRadiusAttr(0.012)  # 24mm diameter carbon tube
        arm.CreateHeightAttr(self.specs.arm_length * 0.9)  # Slightly shorter than full length
        arm.CreateAxisAttr("X")  # Will be rotated

        xform = UsdGeom.Xformable(arm.GetPrim())

        # Position at midpoint between center and rotor
        mid_x = rotor_pos[0] / 2
        mid_y = rotor_pos[1] / 2
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(mid_x, mid_y, rotor_pos[2])
        )

        # Rotate to point toward rotor position
        angle = math.degrees(math.atan2(rotor_pos[1], rotor_pos[0]))
        xform.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(angle)

        arm.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.12, 0.12, 0.12)])

    def _build_battery_visual(self, parent_path: str) -> None:
        """Build battery pack visual."""
        path = f"{parent_path}/battery_visual"
        battery = UsdGeom.Cube.Define(self.stage, path)

        xform = UsdGeom.Xformable(battery.GetPrim())
        xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.075, 0.035, 0.025)  # 150x70x50mm 6S LiPo
        )
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0, 0, -0.035)  # Mounted under frame
        )

        # Blue accent for battery
        battery.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.1, 0.15, 0.4)])

    def _build_body_collision(self, parent_path: str) -> None:
        """Build simplified collision geometry."""
        path = f"{parent_path}/collision"
        col = UsdGeom.Cube.Define(self.stage, path)

        # Cover full drone footprint
        xform = UsdGeom.Xformable(col.GetPrim())
        xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(
                self.specs.wheelbase / 2,
                self.specs.wheelbase / 2,
                self.specs.body_height,
            )
        )

        # Invisible but has collision
        col.GetVisibilityAttr().Set("invisible")
        UsdPhysics.CollisionAPI.Apply(col.GetPrim())

    def _build_rotor(
        self,
        rotor_path: str,
        joint_path: str,
        parent_path: str,
        rotor_idx: int,
    ) -> None:
        """Build rotor assembly with revolute joint."""
        pos = self.ROTOR_POSITIONS[rotor_idx]
        direction = self.ROTOR_DIRECTIONS[rotor_idx]

        # Rotor body
        self._ensure_prim(rotor_path, "Xform")
        rotor_prim = self.stage.GetPrimAtPath(rotor_path)

        # Rigid body for rotor
        UsdPhysics.RigidBodyAPI.Apply(rotor_prim)

        # Mass properties (prop + motor bell)
        mass_api = UsdPhysics.MassAPI.Apply(rotor_prim)
        mass_api.CreateMassAttr(self.specs.rotor_mass)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(
            self.specs.rotor_ixx,
            self.specs.rotor_iyy,
            self.specs.rotor_izz,
        ))

        # Position at motor mount point
        rotor_xform = UsdGeom.Xformable(rotor_prim)
        rotor_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*pos)
        )

        # Propeller disc visual
        prop_path = f"{rotor_path}/prop_visual"
        prop = UsdGeom.Cylinder.Define(self.stage, prop_path)
        prop.CreateRadiusAttr(self.specs.prop_diameter / 2)
        prop.CreateHeightAttr(0.003)  # Thin disc
        prop.CreateAxisAttr("Z")
        prop.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.05, 0.05, 0.05)])

        # Motor housing visual
        motor_path = f"{rotor_path}/motor_visual"
        motor = UsdGeom.Cylinder.Define(self.stage, motor_path)
        motor.CreateRadiusAttr(0.022)  # ~44mm motor
        motor.CreateHeightAttr(0.025)
        motor.CreateAxisAttr("Z")
        motor_xform = UsdGeom.Xformable(motor.GetPrim())
        motor_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0, 0, -0.015)
        )
        motor.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.6, 0.6, 0.65)])  # Silver

        # Revolute joint
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, joint_path)
        joint.CreateAxisAttr("Z")
        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([rotor_path])

        # Joint frame transforms
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*pos))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        # Continuous rotation (no limits)
        joint.CreateLowerLimitAttr(-1e10)
        joint.CreateUpperLimitAttr(1e10)

        # Angular velocity drive
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)  # No position control
        drive.CreateDampingAttr(0.004)  # Joint damping
        drive.CreateMaxForceAttr(100.0)  # Motor torque limit

        # Custom attribute for spin direction
        joint.GetPrim().CreateAttribute(
            "flyby:rotorDirection",
            Sdf.ValueTypeNames.Int
        ).Set(direction)

        joint.GetPrim().CreateAttribute(
            "flyby:rotorIndex",
            Sdf.ValueTypeNames.Int
        ).Set(rotor_idx)

    def _build_sensor_links(self, root_path: str) -> None:
        """Build IMU, GPS, and barometer sensor frames."""
        # IMU link (at body center)
        imu_path = f"{root_path}/imu_link"
        self._ensure_prim(imu_path, "Xform")

        # GPS link (antenna on top)
        gps_path = f"{root_path}/gps_link"
        self._ensure_prim(gps_path, "Xform")
        gps_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(gps_path))
        gps_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0, 0, 0.08)
        )

        # GPS antenna visual (small puck)
        gps_vis_path = f"{gps_path}/antenna_visual"
        gps_vis = UsdGeom.Cylinder.Define(self.stage, gps_vis_path)
        gps_vis.CreateRadiusAttr(0.025)
        gps_vis.CreateHeightAttr(0.012)
        gps_vis.CreateAxisAttr("Z")
        gps_vis.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.2, 0.2, 0.2)])

        # Barometer link (internal, at body center)
        baro_path = f"{root_path}/baro_link"
        self._ensure_prim(baro_path, "Xform")

    def _build_gimbal_assembly(self, root_path: str, joints_path: str) -> None:
        """Build 3-axis gimbal with camera."""
        base_link = f"{root_path}/base_link"
        offset = self.specs.payload_offset

        # Gimbal mount (fixed to body)
        mount_path = f"{root_path}/gimbal_mount"
        self._ensure_prim(mount_path, "Xform")
        mount_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(mount_path))
        mount_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*offset)
        )

        # Create fixed joint from base_link to mount
        mount_joint = UsdPhysics.FixedJoint.Define(
            self.stage, f"{joints_path}/gimbal_mount_joint"
        )
        mount_joint.CreateBody0Rel().SetTargets([base_link])
        mount_joint.CreateBody1Rel().SetTargets([mount_path])
        mount_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*offset))

        # Yaw stage
        yaw_path = f"{root_path}/gimbal_yaw"
        self._ensure_prim(yaw_path, "Xform")
        UsdPhysics.RigidBodyAPI.Apply(self.stage.GetPrimAtPath(yaw_path))

        yaw_joint = UsdPhysics.RevoluteJoint.Define(
            self.stage, f"{joints_path}/gimbal_yaw_joint"
        )
        yaw_joint.CreateAxisAttr("Z")
        yaw_joint.CreateBody0Rel().SetTargets([mount_path])
        yaw_joint.CreateBody1Rel().SetTargets([yaw_path])
        yaw_joint.CreateLowerLimitAttr(-self.specs.gimbal_yaw_range)
        yaw_joint.CreateUpperLimitAttr(self.specs.gimbal_yaw_range)

        # Pitch stage
        pitch_path = f"{root_path}/gimbal_pitch"
        self._ensure_prim(pitch_path, "Xform")
        UsdPhysics.RigidBodyAPI.Apply(self.stage.GetPrimAtPath(pitch_path))

        pitch_joint = UsdPhysics.RevoluteJoint.Define(
            self.stage, f"{joints_path}/gimbal_pitch_joint"
        )
        pitch_joint.CreateAxisAttr("Y")
        pitch_joint.CreateBody0Rel().SetTargets([yaw_path])
        pitch_joint.CreateBody1Rel().SetTargets([pitch_path])
        pitch_joint.CreateLowerLimitAttr(-self.specs.gimbal_pitch_range)
        pitch_joint.CreateUpperLimitAttr(self.specs.gimbal_pitch_range)

        # Roll stage with camera
        camera_path = f"{root_path}/camera_link"
        self._ensure_prim(camera_path, "Xform")
        UsdPhysics.RigidBodyAPI.Apply(self.stage.GetPrimAtPath(camera_path))

        roll_joint = UsdPhysics.RevoluteJoint.Define(
            self.stage, f"{joints_path}/gimbal_roll_joint"
        )
        roll_joint.CreateAxisAttr("X")
        roll_joint.CreateBody0Rel().SetTargets([pitch_path])
        roll_joint.CreateBody1Rel().SetTargets([camera_path])
        roll_joint.CreateLowerLimitAttr(-self.specs.gimbal_roll_range)
        roll_joint.CreateUpperLimitAttr(self.specs.gimbal_roll_range)

        # Camera body visual
        cam_vis_path = f"{camera_path}/camera_visual"
        cam_vis = UsdGeom.Cube.Define(self.stage, cam_vis_path)
        cam_xform = UsdGeom.Xformable(cam_vis.GetPrim())
        cam_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.0635, 0.047, 0.024)  # Sony A7R body
        )
        cam_vis.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.1, 0.1, 0.1)])

        # Lens visual
        lens_path = f"{camera_path}/lens_visual"
        lens = UsdGeom.Cylinder.Define(self.stage, lens_path)
        lens.CreateRadiusAttr(0.035)
        lens.CreateHeightAttr(0.06)
        lens.CreateAxisAttr("Z")
        lens_xform = UsdGeom.Xformable(lens.GetPrim())
        lens_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0, 0, -0.05)
        )
        lens_xform.AddRotateXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(90)

    def _build_lidar_mount(self, root_path: str) -> None:
        """Build LiDAR sensor mount."""
        offset = self.specs.payload_offset

        lidar_path = f"{root_path}/lidar_link"
        self._ensure_prim(lidar_path, "Xform")
        lidar_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(lidar_path))
        lidar_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*offset)
        )

        # VLP-16 housing visual
        housing_path = f"{lidar_path}/lidar_visual"
        housing = UsdGeom.Cylinder.Define(self.stage, housing_path)
        housing.CreateRadiusAttr(0.052)  # 103mm diameter
        housing.CreateHeightAttr(0.072)  # 72mm height
        housing.CreateAxisAttr("Z")
        housing.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.2, 0.2, 0.22)])

    def _build_fixed_camera(self, root_path: str) -> None:
        """Build fixed downward-facing camera (multispectral)."""
        offset = self.specs.payload_offset

        camera_path = f"{root_path}/camera_link"
        self._ensure_prim(camera_path, "Xform")
        cam_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(camera_path))
        cam_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*offset)
        )
        # Point camera downward
        cam_xform.AddRotateXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(180)

        # Multispectral camera body
        body_path = f"{camera_path}/camera_visual"
        body = UsdGeom.Cube.Define(self.stage, body_path)
        body_xform = UsdGeom.Xformable(body.GetPrim())
        body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.045, 0.035, 0.025)  # Compact multispectral
        )
        body.GetPrim().CreateAttribute(
            "primvars:displayColor",
            Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.15, 0.15, 0.18)])
