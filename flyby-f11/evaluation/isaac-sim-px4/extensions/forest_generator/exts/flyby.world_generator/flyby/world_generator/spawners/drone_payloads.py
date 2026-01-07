"""
Drone Payload Builder - Payload attachments for F-11 drones.

Single responsibility: Create gimbal, camera, and LiDAR payload assemblies.
"""

from typing import Tuple

from pxr import Gf, UsdGeom
import omni.isaac.core.utils.prims as prim_utils

from .drone_geometry import DroneGeometryBuilder
from .drone_physics import DronePhysicsBuilder


class DronePayloadBuilder:
    """
    Builds payload assemblies for F-11 drones.

    Responsibilities:
    - 3-axis gimbal creation with joints
    - LiDAR mount creation
    - Fixed camera mount creation
    - Sensor link frame creation
    """

    def __init__(self, stage, geometry: DroneGeometryBuilder, physics: DronePhysicsBuilder):
        """
        Initialize DronePayloadBuilder.

        Args:
            stage: USD stage
            geometry: Geometry builder for visuals
            physics: Physics builder for joints
        """
        self.stage = stage
        self.geometry = geometry
        self.physics = physics

    def create_sensor_links(self, drone_path: str) -> None:
        """
        Create sensor frame links (IMU, GPS).

        Args:
            drone_path: Root path for drone
        """
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

        # GPS antenna visual
        self.geometry.create_sensor_antenna(gps_path)

    def create_gimbal(
        self,
        drone_path: str,
        payload_offset: Tuple[float, float, float],
    ) -> None:
        """
        Create 3-axis gimbal with camera for ISR variant.

        Args:
            drone_path: Root path for drone
            payload_offset: Offset from base_link
        """
        joints_path = f"{drone_path}/Joints"

        # Gimbal mount link
        mount_path = f"{drone_path}/gimbal_mount_link"
        prim_utils.create_prim(mount_path, "Xform")
        mount_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(mount_path))
        mount_xform.ClearXformOpOrder()
        mount_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*payload_offset)
        )

        # Yaw link + joint
        yaw_path = f"{drone_path}/gimbal_yaw_link"
        prim_utils.create_prim(yaw_path, "Xform")

        self.physics.create_gimbal_joint(
            joint_path=f"{joints_path}/gimbal_yaw_joint",
            parent_path=mount_path,
            child_path=yaw_path,
            axis="Z",
            lower_limit=-345.0,
            upper_limit=345.0,
        )

        # Pitch link + joint
        pitch_path = f"{drone_path}/gimbal_pitch_link"
        prim_utils.create_prim(pitch_path, "Xform")

        self.physics.create_gimbal_joint(
            joint_path=f"{joints_path}/gimbal_pitch_joint",
            parent_path=yaw_path,
            child_path=pitch_path,
            axis="Y",
            lower_limit=-120.0,
            upper_limit=120.0,
        )

        # Camera link + roll joint
        camera_link_path = f"{drone_path}/camera_link"
        prim_utils.create_prim(camera_link_path, "Xform")

        self.physics.create_gimbal_joint(
            joint_path=f"{joints_path}/camera_roll_joint",
            parent_path=pitch_path,
            child_path=camera_link_path,
            axis="X",
            lower_limit=-45.0,
            upper_limit=45.0,
        )

        # Camera visual
        self.geometry.create_camera_visual(
            camera_link_path,
            size=(0.064, 0.047, 0.024),  # ~127x94x48mm
        )

    def create_lidar_mount(
        self,
        drone_path: str,
        payload_offset: Tuple[float, float, float],
    ) -> None:
        """
        Create LiDAR sensor mount.

        Args:
            drone_path: Root path for drone
            payload_offset: Offset from base_link
        """
        lidar_path = f"{drone_path}/lidar_link"
        prim_utils.create_prim(lidar_path, "Xform")
        lidar_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(lidar_path))
        lidar_xform.ClearXformOpOrder()
        lidar_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*payload_offset)
        )

        # LiDAR visual
        self.geometry.create_lidar_visual(lidar_path)

    def create_fixed_camera(
        self,
        drone_path: str,
        payload_offset: Tuple[float, float, float],
    ) -> None:
        """
        Create fixed downward camera (multispectral variant).

        Args:
            drone_path: Root path for drone
            payload_offset: Offset from base_link
        """
        camera_path = f"{drone_path}/camera_link"
        prim_utils.create_prim(camera_path, "Xform")
        camera_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(camera_path))
        camera_xform.ClearXformOpOrder()
        camera_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*payload_offset)
        )
        # Point downward
        camera_xform.AddRotateXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(90.0)

        # Camera visual (compact multispectral)
        self.geometry.create_camera_visual(
            camera_path,
            size=(0.04, 0.03, 0.02),
        )
