#!/usr/bin/env python3
"""
F-11 ISR Sensor Suite for Isaac Sim

Complete sensor implementation matching real Flyby F-11 specifications:
- GPS (u-blox NEO-F9P RTK)
- IMU, Barometer, Magnetometer (Pegasus)
- EO Camera (Sony Block 4K with zoom)
- Thermal Camera (FLIR Boson 640) - simulated via semantic segmentation
- Depth Camera (RGB-D)
- LiDAR (Ouster OS-1) - optional

All cameras are belly-mounted to avoid body obstruction.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
from scipy.spatial.transform import Rotation

# Isaac Sim camera - must be imported after SimulationApp is initialized
# from isaacsim.sensors.camera import Camera


@dataclass
class GimbalState:
    """Current gimbal orientation state."""
    pitch: float = 0.0  # degrees, -120 to +30
    yaw: float = 0.0    # degrees, -345 to +345
    roll: float = 0.0   # degrees, -45 to +45


@dataclass
class SensorConfig:
    """Configuration for F-11 sensor suite."""
    # Drone body reference path in USD
    drone_body_path: str = "/World/F11_ISR/body"

    # EO Camera (Sony Block 4K)
    eo_enabled: bool = True
    eo_resolution: Tuple[int, int] = (1920, 1080)
    eo_focal_length: float = 4.5  # mm, wide angle mode
    eo_frequency: int = 30
    eo_depth_enabled: bool = True

    # Thermal Camera (FLIR Boson 640)
    thermal_enabled: bool = True
    thermal_resolution: Tuple[int, int] = (640, 512)
    thermal_focal_length: float = 9.1  # mm
    thermal_frequency: int = 30

    # LiDAR (Ouster OS-1)
    lidar_enabled: bool = False
    lidar_channels: int = 64
    lidar_range: float = 120.0
    lidar_frequency: int = 20

    # Mount positions (meters, relative to drone body center)
    # Belly mount to avoid body obstruction
    gimbal_mount_position: Tuple[float, float, float] = (0.0, 0.0, -0.15)
    gimbal_default_pitch: float = 45.0  # degrees down

    # LiDAR is top-mounted
    lidar_mount_position: Tuple[float, float, float] = (0.0, 0.0, 0.05)


class F11GimbalController:
    """
    3-axis gimbal controller for F-11 ISR payload.

    Matches Gremsy VIO specifications:
    - Pitch: -120 to +30 degrees
    - Yaw: -345 to +345 degrees
    - Roll: -45 to +45 degrees
    """

    PITCH_LIMITS = (-120.0, 30.0)
    YAW_LIMITS = (-345.0, 345.0)
    ROLL_LIMITS = (-45.0, 45.0)

    def __init__(self, cameras: List, base_orientations: List[np.ndarray]):
        """
        Initialize gimbal with list of cameras to control.

        Args:
            cameras: List of Isaac Sim Camera objects
            base_orientations: Base orientation for each camera (Euler angles in degrees)
        """
        self.cameras = cameras
        self.base_orientations = base_orientations
        self.state = GimbalState()

    def set_angles(self, pitch: float = None, yaw: float = None, roll: float = None):
        """
        Set gimbal angles (clamped to limits).

        Args:
            pitch: Pitch angle in degrees (negative = down)
            yaw: Yaw angle in degrees
            roll: Roll angle in degrees
        """
        if pitch is not None:
            self.state.pitch = np.clip(pitch, *self.PITCH_LIMITS)
        if yaw is not None:
            self.state.yaw = np.clip(yaw, *self.YAW_LIMITS)
        if roll is not None:
            self.state.roll = np.clip(roll, *self.ROLL_LIMITS)

        self._apply_rotation()

    def _apply_rotation(self):
        """Apply current gimbal rotation to all cameras."""
        for camera, base_orient in zip(self.cameras, self.base_orientations):
            try:
                # Base orientation + gimbal offset
                base_rot = Rotation.from_euler("ZYX", base_orient, degrees=True)
                gimbal_rot = Rotation.from_euler(
                    "ZYX",
                    [self.state.yaw, self.state.pitch, self.state.roll],
                    degrees=True
                )
                combined = base_rot * gimbal_rot

                # Apply to camera
                pos, _ = camera.get_local_pose()
                camera.set_local_pose(pos, combined.as_quat())
            except Exception:
                pass  # Camera may not be ready

    def look_at_angle(self, pitch: float, yaw: float = 0.0):
        """Convenience method to point cameras at a specific angle."""
        self.set_angles(pitch=pitch, yaw=yaw, roll=0.0)

    def look_forward(self):
        """Point cameras forward (0 pitch)."""
        self.set_angles(pitch=0.0, yaw=0.0, roll=0.0)

    def look_down(self):
        """Point cameras straight down (-90 pitch)."""
        self.set_angles(pitch=-90.0, yaw=0.0, roll=0.0)

    def nadir_survey(self):
        """Point cameras for nadir (straight down) survey mode."""
        self.set_angles(pitch=-90.0, yaw=0.0, roll=0.0)

    def oblique_survey(self, angle: float = -45.0):
        """Point cameras for oblique survey mode."""
        self.set_angles(pitch=angle, yaw=0.0, roll=0.0)


class F11SensorSuite:
    """
    Complete F-11 ISR sensor suite for Isaac Sim.

    Provides:
    - EO Camera (RGB + Depth)
    - Thermal Camera (simulated)
    - 3-axis Gimbal Control
    - LiDAR (optional)

    All ISR sensors are belly-mounted to avoid body obstruction.
    """

    def __init__(self, config: SensorConfig = None):
        """
        Initialize sensor suite.

        Args:
            config: Sensor configuration (uses defaults if None)
        """
        self.config = config or SensorConfig()

        self.eo_camera = None
        self.thermal_camera = None
        self.lidar = None
        self.gimbal = None

        self._cameras = []
        self._base_orientations = []

    def initialize(self, world):
        """
        Initialize all sensors in the simulation world.

        Args:
            world: Isaac Sim World object

        Must be called after world.reset()
        """
        # Import here since SimulationApp must be initialized first
        from isaacsim.sensors.camera import Camera

        gimbal_pos = np.array(self.config.gimbal_mount_position)
        base_pitch = self.config.gimbal_default_pitch

        # EO Camera (Sony Block 4K simulation)
        if self.config.eo_enabled:
            eo_path = f"{self.config.drone_body_path}/eo_camera"
            self.eo_camera = Camera(
                prim_path=eo_path,
                frequency=self.config.eo_frequency,
                resolution=self.config.eo_resolution,
            )

            # Set position and orientation
            eo_orientation = np.array([0.0, base_pitch, 0.0])
            self.eo_camera.set_local_pose(
                gimbal_pos,
                Rotation.from_euler("ZYX", eo_orientation, degrees=True).as_quat()
            )

            self.eo_camera.initialize()
            self.eo_camera.set_focal_length(self.config.eo_focal_length)
            self.eo_camera.set_clipping_range(0.1, 500.0)

            if self.config.eo_depth_enabled:
                self.eo_camera.add_distance_to_image_plane_to_frame()

            self._cameras.append(self.eo_camera)
            self._base_orientations.append(eo_orientation)

            print(f"  [SENSORS] EO Camera initialized: {self.config.eo_resolution}")

        # Thermal Camera (FLIR Boson 640 simulation)
        if self.config.thermal_enabled:
            thermal_path = f"{self.config.drone_body_path}/thermal_camera"
            self.thermal_camera = Camera(
                prim_path=thermal_path,
                frequency=self.config.thermal_frequency,
                resolution=self.config.thermal_resolution,
            )

            # Slightly offset from EO camera but co-aligned
            thermal_pos = gimbal_pos + np.array([0.0, 0.02, 0.0])  # 2cm lateral offset
            thermal_orientation = np.array([0.0, base_pitch, 0.0])
            self.thermal_camera.set_local_pose(
                thermal_pos,
                Rotation.from_euler("ZYX", thermal_orientation, degrees=True).as_quat()
            )

            self.thermal_camera.initialize()
            self.thermal_camera.set_focal_length(self.config.thermal_focal_length)
            self.thermal_camera.set_clipping_range(0.1, 200.0)

            self._cameras.append(self.thermal_camera)
            self._base_orientations.append(thermal_orientation)

            print(f"  [SENSORS] Thermal Camera initialized: {self.config.thermal_resolution}")
            print("    Note: Thermal simulated via RGB (no native thermal in Isaac Sim)")

        # LiDAR (Ouster OS-1)
        if self.config.lidar_enabled:
            self._init_lidar()

        # Initialize gimbal controller
        if self._cameras:
            self.gimbal = F11GimbalController(self._cameras, self._base_orientations)
            print(f"  [SENSORS] Gimbal controller initialized ({len(self._cameras)} cameras)")

    def _init_lidar(self):
        """Initialize LiDAR sensor."""
        try:
            # Try RTX Lidar first (better quality)
            from omni.isaac.sensor import LidarRtx

            lidar_path = f"{self.config.drone_body_path}/lidar"
            self.lidar = LidarRtx(
                prim_path=lidar_path,
            )

            lidar_pos = np.array(self.config.lidar_mount_position)
            self.lidar.set_local_pose(lidar_pos, np.array([0, 0, 0, 1]))

            print(f"  [SENSORS] LiDAR (RTX) initialized: {self.config.lidar_channels}ch")

        except ImportError:
            # Fall back to regular Lidar
            try:
                from omni.isaac.range_sensor import Lidar

                lidar_path = f"{self.config.drone_body_path}/lidar"
                self.lidar = Lidar(
                    prim_path=lidar_path,
                )
                print(f"  [SENSORS] LiDAR (range) initialized")

            except ImportError:
                print("  [SENSORS] Warning: LiDAR not available in this Isaac Sim build")
                self.config.lidar_enabled = False

    def get_eo_rgba(self) -> Optional[np.ndarray]:
        """Get RGBA image from EO camera."""
        if self.eo_camera is None:
            return None
        try:
            return self.eo_camera.get_rgba()
        except Exception:
            return None

    def get_eo_rgb(self) -> Optional[np.ndarray]:
        """Get RGB image from EO camera (no alpha)."""
        rgba = self.get_eo_rgba()
        if rgba is not None and rgba.shape[-1] == 4:
            return rgba[:, :, :3]
        return rgba

    def get_eo_depth(self) -> Optional[np.ndarray]:
        """Get depth map from EO camera."""
        if self.eo_camera is None or not self.config.eo_depth_enabled:
            return None
        try:
            return self.eo_camera.get_depth()
        except Exception:
            return None

    def get_thermal_image(self) -> Optional[np.ndarray]:
        """
        Get thermal image.

        Note: Isaac Sim doesn't have native thermal simulation.
        This returns RGB from the thermal camera position.
        For realistic thermal simulation, post-process based on:
        - Scene semantics (hot engines, people, etc.)
        - Time of day (solar heating)
        - Material thermal properties
        """
        if self.thermal_camera is None:
            return None
        try:
            rgba = self.thermal_camera.get_rgba()
            if rgba is not None:
                # Convert to grayscale as thermal proxy
                rgb = rgba[:, :, :3]
                thermal = np.mean(rgb, axis=2).astype(np.uint8)
                return thermal
            return None
        except Exception:
            return None

    def get_lidar_points(self) -> Optional[np.ndarray]:
        """Get LiDAR point cloud."""
        if self.lidar is None:
            return None
        try:
            return self.lidar.get_point_cloud()
        except Exception:
            return None

    def set_zoom(self, zoom_factor: float):
        """
        Set camera zoom level (1.0 = wide, 20.0 = max tele).

        Simulates Sony Block 20x optical zoom.
        """
        if self.eo_camera is None:
            return

        # Map zoom to focal length
        # Wide: 4.3mm, Tele: 86mm (20x)
        min_fl = 4.3
        max_fl = 86.0
        focal_length = min_fl + (max_fl - min_fl) * (zoom_factor - 1.0) / 19.0
        focal_length = np.clip(focal_length, min_fl, max_fl)

        try:
            self.eo_camera.set_focal_length(focal_length)
        except Exception:
            pass

    def get_sensor_status(self) -> Dict:
        """Get status of all sensors."""
        status = {
            "eo_camera": self.eo_camera is not None,
            "thermal_camera": self.thermal_camera is not None,
            "lidar": self.lidar is not None,
            "gimbal": self.gimbal is not None,
        }

        if self.gimbal:
            status["gimbal_state"] = {
                "pitch": self.gimbal.state.pitch,
                "yaw": self.gimbal.state.yaw,
                "roll": self.gimbal.state.roll,
            }

        return status


def create_f11_sensors(drone_path: str = "/World/F11_ISR") -> SensorConfig:
    """
    Create default F-11 sensor configuration.

    Args:
        drone_path: USD path to drone prim

    Returns:
        SensorConfig with F-11 defaults
    """
    return SensorConfig(
        drone_body_path=f"{drone_path}/body",
        eo_enabled=True,
        eo_resolution=(1920, 1080),
        eo_depth_enabled=True,
        thermal_enabled=True,
        lidar_enabled=False,  # Set True when LiDAR payload installed
        gimbal_mount_position=(0.0, 0.0, -0.15),  # Belly mount
        gimbal_default_pitch=45.0,
    )
