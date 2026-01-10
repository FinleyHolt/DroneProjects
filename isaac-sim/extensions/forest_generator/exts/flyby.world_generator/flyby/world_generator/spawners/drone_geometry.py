"""
Drone Geometry Builder - Visual and collision geometry for F-11 drones.

Single responsibility: Create visual meshes, materials, and collision shapes.
"""

from typing import Tuple

from pxr import Gf, UsdGeom, UsdShade, Sdf, UsdPhysics
import omni.isaac.core.utils.prims as prim_utils


class DroneGeometryBuilder:
    """
    Builds visual and collision geometry for F-11 drones.

    Responsibilities:
    - Create body/frame visual geometry
    - Create arm geometry
    - Create battery visual
    - Create collision shapes
    - Apply materials to geometry
    """

    # Material paths
    FRAME_MAT = "/World/Looks/drone_frame_mat"
    PROP_MAT = "/World/Looks/drone_prop_mat"
    MOTOR_MAT = "/World/Looks/drone_motor_mat"
    BATTERY_MAT = "/World/Looks/drone_battery_mat"

    def __init__(self, stage):
        """
        Initialize DroneGeometryBuilder.

        Args:
            stage: USD stage
        """
        self.stage = stage
        self._ensure_materials()

    def _ensure_materials(self) -> None:
        """Ensure drone materials exist."""
        if not self.stage.GetPrimAtPath("/World/Looks"):
            prim_utils.create_prim("/World/Looks", "Xform")

        # Frame material (dark carbon fiber look)
        self._create_pbr_material(
            self.FRAME_MAT,
            color=(0.15, 0.15, 0.15),
            roughness=0.4,
            metallic=0.2,
        )

        # Propeller material
        self._create_pbr_material(
            self.PROP_MAT,
            color=(0.08, 0.08, 0.08),
            roughness=0.6,
            metallic=0.1,
        )

        # Motor material (silver)
        self._create_pbr_material(
            self.MOTOR_MAT,
            color=(0.7, 0.7, 0.72),
            roughness=0.3,
            metallic=0.8,
        )

        # Battery material (blue accent)
        self._create_pbr_material(
            self.BATTERY_MAT,
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
        """Create a PBR material if it doesn't exist."""
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

    def create_body_visual(
        self,
        prim_path: str,
        body_size: Tuple[float, float, float],
    ) -> None:
        """Create central body cube visual."""
        body_visual_path = f"{prim_path}/body_visual"
        body_visual = UsdGeom.Cube.Define(self.stage, body_visual_path)
        body_xform = UsdGeom.Xformable(body_visual.GetPrim())
        body_xform.ClearXformOpOrder()
        body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(
                body_size[0] / 2,
                body_size[1] / 2,
                body_size[2] / 2,
            )
        )
        self._bind_material(body_visual_path, self.FRAME_MAT)

    def create_arm_visual(
        self,
        prim_path: str,
        arm_index: int,
        rotor_position: Tuple[float, float, float],
        arm_length: float,
    ) -> None:
        """Create a single arm visual cylinder."""
        import math

        rx, ry, rz = rotor_position
        arm_path = f"{prim_path}/arm_{arm_index}_visual"
        arm = UsdGeom.Cylinder.Define(self.stage, arm_path)
        arm.CreateRadiusAttr(0.015)  # 15mm diameter arms
        arm.CreateHeightAttr(arm_length)
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

        self._bind_material(arm_path, self.FRAME_MAT)

    def create_battery_visual(self, prim_path: str) -> None:
        """Create battery visual cube."""
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
        self._bind_material(battery_path, self.BATTERY_MAT)

    def create_collision_box(
        self,
        prim_path: str,
        wheelbase: float,
        body_height: float,
    ) -> None:
        """Create simplified collision box."""
        collision_path = f"{prim_path}/collision"
        collision = UsdGeom.Cube.Define(self.stage, collision_path)
        collision_xform = UsdGeom.Xformable(collision.GetPrim())
        collision_xform.ClearXformOpOrder()
        collision_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(wheelbase / 2, wheelbase / 2, body_height)
        )

        # Make collision invisible but enable collision API
        collision.GetPrim().GetAttribute("visibility").Set("invisible")
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())

    def create_rotor_visuals(
        self,
        rotor_path: str,
        prop_diameter: float,
    ) -> None:
        """Create propeller and motor visuals for a rotor."""
        # Propeller visual (disc/cylinder)
        prop_path = f"{rotor_path}/prop_visual"
        prop = UsdGeom.Cylinder.Define(self.stage, prop_path)
        prop.CreateRadiusAttr(prop_diameter / 2)
        prop.CreateHeightAttr(0.005)  # 5mm thick
        prop.CreateAxisAttr("Z")
        self._bind_material(prop_path, self.PROP_MAT)

        # Motor visual (small cylinder)
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
        self._bind_material(motor_path, self.MOTOR_MAT)

    def create_sensor_antenna(self, gps_path: str) -> None:
        """Create GPS antenna visual."""
        gps_visual_path = f"{gps_path}/antenna_visual"
        gps_visual = UsdGeom.Cylinder.Define(self.stage, gps_visual_path)
        gps_visual.CreateRadiusAttr(0.03)
        gps_visual.CreateHeightAttr(0.015)
        gps_visual.CreateAxisAttr("Z")

    def create_camera_visual(
        self,
        camera_link_path: str,
        size: Tuple[float, float, float] = (0.064, 0.047, 0.024),
    ) -> None:
        """Create camera body visual."""
        camera_visual_path = f"{camera_link_path}/camera_visual"
        camera_visual = UsdGeom.Cube.Define(self.stage, camera_visual_path)
        camera_xform = UsdGeom.Xformable(camera_visual.GetPrim())
        camera_xform.ClearXformOpOrder()
        camera_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*size)
        )
        self._bind_material(camera_visual_path, self.FRAME_MAT)

    def create_lidar_visual(self, lidar_path: str) -> None:
        """Create LiDAR visual (VLP-16 style)."""
        lidar_visual_path = f"{lidar_path}/lidar_visual"
        lidar_visual = UsdGeom.Cylinder.Define(self.stage, lidar_visual_path)
        lidar_visual.CreateRadiusAttr(0.052)  # VLP-16 diameter
        lidar_visual.CreateHeightAttr(0.072)  # VLP-16 height
        lidar_visual.CreateAxisAttr("Z")
        self._bind_material(lidar_visual_path, self.FRAME_MAT)
