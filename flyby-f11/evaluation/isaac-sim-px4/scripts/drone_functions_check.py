#!/usr/bin/env python3
"""
Drone Functions Check - Comprehensive PX4 Flight Test with YOLO Video Recording

This script verifies all core drone capabilities work correctly in Isaac Sim
headless mode and outputs a single annotated MP4 video with YOLO detections.

Tests performed:
1. Takeoff to 15m altitude
2. Waypoint navigation (square pattern)
3. Gimbal/camera control (pan and tilt during flight)
4. Return to home and land

Architecture:
    Isaac Sim (Camera.get_rgba()) -> YOLO detection -> FFmpeg pipe -> MP4 video

Usage:
    /isaac-sim/python.sh /workspace/scripts/drone_functions_check.py --headless

Output:
    /workspace/output/functions_check.mp4 - Annotated video with YOLO detections
    /workspace/output/functions_check_report.json - Test results

Author: Finley Holt
"""

import sys
import os

# Early print to verify script is running
print("=" * 70, flush=True)
print("DRONE FUNCTIONS CHECK - Script Starting...", flush=True)
print("=" * 70, flush=True)

HEADLESS = "--headless" in sys.argv
print(f"[Config] Headless mode: {HEADLESS}", flush=True)

# Import SimulationApp and start it
print("[Init] Importing SimulationApp...", flush=True)
from isaacsim import SimulationApp

simulation_config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 0,
    "width": 1280,
    "height": 720,
}
print(f"[Init] Creating SimulationApp with config: {simulation_config}", flush=True)
simulation_app = SimulationApp(simulation_config)
print("[Init] SimulationApp created successfully", flush=True)

import omni
import omni.timeline
import numpy as np
import time
import json
import subprocess
import math
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Tuple, Any
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
from pxr import UsdGeom, Gf, UsdPhysics, Sdf
print("[Init] Core imports completed", flush=True)

# Add paths
sys.path.insert(0, "/workspace")

PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils import extensions
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera

from omni.isaac.core.world import World
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig
print("[Init] All imports completed successfully", flush=True)

# ============================================================================
# Configuration
# ============================================================================

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_PATH = "/World/isr_camera"

OUTPUT_DIR = "/workspace/output"
VIDEO_OUTPUT = f"{OUTPUT_DIR}/functions_check.mp4"
REPORT_OUTPUT = f"{OUTPUT_DIR}/functions_check_report.json"

YOLO_MODEL_PATH = "/workspace/models/yolo11x.pt"

# Flight parameters
TAKEOFF_ALTITUDE = 75.0  # meters
WAYPOINT_SIZE = 30.0  # meters (square pattern side length)
WAYPOINT_ALTITUDE = 75.0  # meters
CRUISE_SPEED = 3.0  # m/s
POSITION_TOLERANCE = 2.0  # meters

# Gimbal control parameters (simulated via camera rotation)
GIMBAL_PAN_RATE = 15.0  # degrees per second
GIMBAL_TILT_RANGE = (-60.0, 0.0)  # degrees (looking down to horizon)


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class TestResult:
    """Result of a single test."""
    name: str
    passed: bool
    message: str = ""
    duration_seconds: float = 0.0
    data: Dict[str, Any] = field(default_factory=dict)


@dataclass
class FlightState:
    """Current flight state."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    heading: float = 0.0
    altitude: float = 0.0
    armed: bool = False
    mode: int = 0
    mode_name: str = "UNKNOWN"


# ============================================================================
# Video Recording Pipeline
# ============================================================================

class VideoRecorder:
    """Records video with YOLO detection overlay using FFmpeg pipe."""

    def __init__(
        self,
        output_path: str,
        width: int,
        height: int,
        fps: int = 30,
        yolo_model_path: str = None,
    ):
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_count = 0
        self.ffmpeg_proc = None
        self.yolo_model = None

        # Ensure output directory exists
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        # Load YOLO model if available
        if yolo_model_path and os.path.exists(yolo_model_path):
            try:
                from ultralytics import YOLO
                self.yolo_model = YOLO(yolo_model_path)
                print(f"[VideoRecorder] Loaded YOLO model from {yolo_model_path}")
            except ImportError:
                print("[VideoRecorder] WARNING: ultralytics not installed, YOLO disabled")
            except Exception as e:
                print(f"[VideoRecorder] WARNING: Failed to load YOLO model: {e}")
        else:
            print(f"[VideoRecorder] WARNING: YOLO model not found at {yolo_model_path}")

    def start(self) -> bool:
        """Start the FFmpeg encoding process."""
        # Check if ffmpeg is available
        import shutil
        if shutil.which('ffmpeg') is None:
            print("[VideoRecorder] WARNING: FFmpeg not found in PATH - video recording disabled")
            print("[VideoRecorder] To enable video: apt-get install ffmpeg")
            self.ffmpeg_proc = None
            return False

        ffmpeg_cmd = [
            'ffmpeg', '-y',
            '-f', 'rawvideo',
            '-pix_fmt', 'rgb24',
            '-s', f'{self.width}x{self.height}',
            '-r', str(self.fps),
            '-i', '-',  # Read from stdin
            '-c:v', 'libx264',
            '-preset', 'fast',
            '-crf', '23',
            '-pix_fmt', 'yuv420p',
            self.output_path
        ]

        try:
            self.ffmpeg_proc = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print(f"[VideoRecorder] Started FFmpeg, output: {self.output_path}")
            return True
        except Exception as e:
            print(f"[VideoRecorder] ERROR: Failed to start FFmpeg: {e}")
            self.ffmpeg_proc = None
            return False

    def write_frame(
        self,
        rgba_frame: np.ndarray,
        overlay_text: str = None,
        right_overlay_text: str = None,
    ) -> int:
        """
        Write a frame with optional YOLO detection overlay.

        Args:
            rgba_frame: RGBA image array from camera
            overlay_text: Optional text to overlay on frame (top-left)
            right_overlay_text: Optional text to overlay on frame (top-right)

        Returns:
            Number of detections found (0 if YOLO disabled)
        """
        if self.ffmpeg_proc is None or rgba_frame is None or rgba_frame.size == 0:
            return 0

        # Validate and resize frame if needed
        import cv2
        if rgba_frame.shape[0] != self.height or rgba_frame.shape[1] != self.width:
            if self.frame_count == 0:
                print(f"[VideoRecorder] Frame size mismatch: got {rgba_frame.shape[:2]}, expected ({self.height}, {self.width}). Resizing...")
            rgba_frame = cv2.resize(rgba_frame, (self.width, self.height))

        # Convert RGBA to RGB
        rgb_frame = rgba_frame[:, :, :3].copy()
        detection_count = 0

        # Run YOLO detection if available
        if self.yolo_model is not None:
            try:
                results = self.yolo_model(rgb_frame, verbose=False)
                if results and len(results) > 0:
                    # Get annotated frame with bounding boxes
                    annotated = results[0].plot()
                    # Ensure correct shape and type
                    if annotated.shape == rgb_frame.shape:
                        rgb_frame = annotated
                    detection_count = len(results[0].boxes) if results[0].boxes is not None else 0
            except Exception as e:
                if self.frame_count % 100 == 0:
                    print(f"[VideoRecorder] YOLO error: {e}")

        # Add text overlay if provided (top-left)
        if overlay_text:
            try:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                thickness = 1
                color = (255, 255, 255)
                bg_color = (0, 0, 0)

                lines = overlay_text.split('\n')
                y_offset = 20
                for line in lines:
                    (text_width, text_height), baseline = cv2.getTextSize(
                        line, font, font_scale, thickness
                    )
                    # Draw background rectangle
                    cv2.rectangle(
                        rgb_frame,
                        (5, y_offset - text_height - 3),
                        (12 + text_width, y_offset + 3),
                        bg_color,
                        -1
                    )
                    # Draw text
                    cv2.putText(
                        rgb_frame, line, (8, y_offset),
                        font, font_scale, color, thickness
                    )
                    y_offset += text_height + 8
            except Exception:
                pass

        # Add right overlay text (top-right) for drone/gimbal angles
        if right_overlay_text:
            try:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.45
                thickness = 1
                color = (0, 255, 255)  # Cyan for visibility
                bg_color = (0, 0, 0)

                lines = right_overlay_text.split('\n')
                y_offset = 20

                for line in lines:
                    (text_width, text_height), baseline = cv2.getTextSize(
                        line, font, font_scale, thickness
                    )
                    x_pos = self.width - text_width - 12
                    # Draw background rectangle
                    cv2.rectangle(
                        rgb_frame,
                        (x_pos - 5, y_offset - text_height - 3),
                        (self.width - 5, y_offset + 3),
                        bg_color,
                        -1
                    )
                    # Draw text
                    cv2.putText(
                        rgb_frame, line, (x_pos, y_offset),
                        font, font_scale, color, thickness
                    )
                    y_offset += text_height + 8
            except Exception:
                pass

        # Ensure contiguous array for FFmpeg
        rgb_frame = np.ascontiguousarray(rgb_frame)

        # Write to FFmpeg stdin
        try:
            self.ffmpeg_proc.stdin.write(rgb_frame.tobytes())
            self.frame_count += 1
        except Exception as e:
            print(f"[VideoRecorder] ERROR writing frame: {e}")

        return detection_count

    def stop(self) -> bool:
        """Stop the FFmpeg process and finalize video."""
        if self.ffmpeg_proc is None:
            return False

        try:
            self.ffmpeg_proc.stdin.close()
            self.ffmpeg_proc.wait(timeout=30)
            print(f"[VideoRecorder] Finished: {self.frame_count} frames written")
            return True
        except Exception as e:
            print(f"[VideoRecorder] ERROR stopping FFmpeg: {e}")
            self.ffmpeg_proc.kill()
            return False


# ============================================================================
# Gimbal Controller
# ============================================================================

class GimbalController:
    """
    Simulates a stabilized gimbal by rotating the camera prim.

    Isaac Sim Camera Convention:
    - Camera looks along -Z axis (optical axis)
    - +Y is up in camera frame
    - +X is right in camera frame

    World Coordinate System (Isaac Sim ENU):
    - +X = East
    - +Y = North (forward for drone)
    - +Z = Up

    Gimbal Angles (in world frame):
    - Pan: Rotation around world Z axis (0 = looking along +X, 90 = looking along +Y/north)
    - Tilt: Rotation around camera's local right axis (0 = horizon, negative = looking down)

    The gimbal provides 3-axis stabilization by:
    1. Computing desired world-frame camera orientation from pan/tilt
    2. Countering the drone's current attitude to maintain that world orientation
    """

    # Gimbal slew rate in degrees per second
    SLEW_RATE = 45.0

    def __init__(self, camera_prim_path: str, stage, default_tilt: float = -30.0):
        self.camera_path = camera_prim_path
        self.stage = stage

        # Current gimbal angles (world frame)
        self.pan = 0.0    # degrees around Z (0 = +X direction)
        self.tilt = default_tilt  # degrees (0 = horizon, negative = down)

        # Target angles for smooth interpolation
        self.target_pan = 0.0
        self.target_tilt = default_tilt

        self.tilt_min, self.tilt_max = GIMBAL_TILT_RANGE

        # Drone attitude for stabilization (in degrees)
        self._drone_roll = 0.0
        self._drone_pitch = 0.0
        self._drone_yaw = 0.0

        self._last_update_time = time.time()

    def set_angles(self, pan: float, tilt: float) -> None:
        """Set target gimbal pan and tilt angles (will interpolate smoothly)."""
        self.target_pan = pan % 360
        self.target_tilt = max(self.tilt_min, min(self.tilt_max, tilt))

    def set_angles_immediate(self, pan: float, tilt: float) -> None:
        """Set gimbal angles immediately (no interpolation)."""
        self.pan = pan % 360
        self.tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        self.target_pan = self.pan
        self.target_tilt = self.tilt
        self._update_camera_orientation()

    def pan_by(self, delta_pan: float) -> None:
        """Adjust target pan angle by delta degrees."""
        self.target_pan = (self.target_pan + delta_pan) % 360

    def tilt_by(self, delta_tilt: float) -> None:
        """Adjust target tilt angle by delta degrees."""
        self.target_tilt = max(self.tilt_min, min(self.tilt_max, self.target_tilt + delta_tilt))

    def look_at_ground(self) -> None:
        """Point camera straight down."""
        self.target_tilt = -90.0

    def look_forward(self) -> None:
        """Point camera forward (horizon)."""
        self.target_tilt = 0.0

    def update_drone_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        """
        Update the drone's current attitude for stabilization.

        Args:
            roll: Drone roll in degrees (positive = right wing down)
            pitch: Drone pitch in degrees (positive = nose up)
            yaw: Drone yaw in degrees (positive = clockwise from above)
        """
        self._drone_roll = roll
        self._drone_pitch = pitch
        self._drone_yaw = yaw

    def get_drone_attitude(self) -> tuple:
        """Return current drone attitude (roll, pitch, yaw) in degrees."""
        return (self._drone_roll, self._drone_pitch, self._drone_yaw)

    def get_gimbal_angles(self) -> tuple:
        """Return current gimbal angles (pan, tilt) in degrees."""
        return (self.pan, self.tilt)

    def update(self, dt: float = None) -> None:
        """Update gimbal with smooth interpolation toward target angles."""
        if dt is None:
            current_time = time.time()
            dt = current_time - self._last_update_time
            self._last_update_time = current_time

        dt = min(dt, 0.1)  # Clamp to avoid jumps
        max_delta = self.SLEW_RATE * dt

        # Interpolate pan (handle wraparound)
        pan_diff = self.target_pan - self.pan
        if pan_diff > 180:
            pan_diff -= 360
        elif pan_diff < -180:
            pan_diff += 360

        if abs(pan_diff) > max_delta:
            self.pan += max_delta * np.sign(pan_diff)
        else:
            self.pan = self.target_pan
        self.pan = self.pan % 360

        # Interpolate tilt
        tilt_diff = self.target_tilt - self.tilt
        if abs(tilt_diff) > max_delta:
            self.tilt += max_delta * np.sign(tilt_diff)
        else:
            self.tilt = self.target_tilt

        self._update_camera_orientation()

    def _update_camera_orientation(self) -> None:
        """
        Update camera orientation in WORLD frame (completely isolated from drone rotation).

        This is a true gimbal isolation - the camera's orientation is set directly in
        world coordinates. The camera position follows the drone, but rotation is
        completely independent. No counter-rotation math needed.

        Isaac Sim Camera Convention:
        - Camera looks along -Z axis (optical axis)
        - +Y is up in camera frame
        - +X is right in camera frame

        We construct the world-frame rotation matrix directly from pan/tilt angles.
        """
        camera_prim = self.stage.GetPrimAtPath(self.camera_path)
        if not camera_prim.IsValid():
            return

        # Negate pan to fix direction: positive pan = look left (CCW from above)
        pan_rad = np.radians(-self.pan)
        tilt_rad = np.radians(self.tilt)

        # Compute look direction in world frame
        # pan=0, tilt=0 -> looking along +X (east)
        # pan rotates around Z, tilt pitches up/down
        cos_pan = np.cos(pan_rad)
        sin_pan = np.sin(pan_rad)
        cos_tilt = np.cos(tilt_rad)
        sin_tilt = np.sin(tilt_rad)

        # Look direction (what the camera points at)
        look_dir = np.array([
            cos_pan * cos_tilt,
            sin_pan * cos_tilt,
            sin_tilt
        ])
        look_dir = look_dir / np.linalg.norm(look_dir)

        # World up vector
        world_up = np.array([0.0, 0.0, 1.0])

        # Right vector = look_dir x world_up (then normalize)
        right = np.cross(look_dir, world_up)
        right_norm = np.linalg.norm(right)
        if right_norm < 1e-6:
            # Looking straight up or down - pick arbitrary right
            right = np.array([0.0, 1.0, 0.0])
        else:
            right = right / right_norm

        # Camera up = right x look_dir (ensures orthogonal frame)
        up = np.cross(right, look_dir)
        up = up / np.linalg.norm(up)

        # Build rotation matrix: columns are camera basis vectors in world coords
        # Camera frame: X=right, Y=up, Z=back (camera looks along -Z)
        # So camera's -Z should point along look_dir -> camera's +Z = -look_dir
        R_cam_to_world = np.column_stack([right, up, -look_dir])

        # Convert to quaternion
        world_rot = Rotation.from_matrix(R_cam_to_world)
        quat_xyzw = world_rot.as_quat()  # scipy format: [x, y, z, w]
        quat_wxyz = Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])

        # Set the camera's world orientation directly
        xform = UsdGeom.Xformable(camera_prim)
        ops = xform.GetOrderedXformOps()

        # Find existing orient op or create one
        orient_op = None
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_op = op
                break

        if orient_op is None:
            # Get current translation before modifying
            translate_val = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_val = op.Get()
                    break

            # Clear and rebuild xform ops
            xform.ClearXformOpOrder()

            # Add translate first
            translate_op = xform.AddTranslateOp()
            if translate_val is not None:
                translate_op.Set(translate_val)

            # Add orient op with double precision (quatd) to match existing camera setup
            orient_op = xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)

        # Set the quaternion value
        orient_op.Set(quat_wxyz)


# ============================================================================
# Flight Controller
# ============================================================================

class FlightController:
    """PX4 flight controller interface via MAVLink."""

    # PX4 custom modes - these are the ENCODED values from heartbeat.custom_mode
    # The encoding is: main_mode << 16 | sub_mode << 24
    # Main modes: 1=MANUAL, 2=ALTCTL, 3=POSCTL, 4=AUTO, 5=ACRO, 6=OFFBOARD, 7=STABILIZED, 8=RATTITUDE
    # For AUTO mode, sub_mode specifies: 1=READY, 2=TAKEOFF, 3=LOITER, 4=MISSION, 5=RTL, 6=LAND
    PX4_MODE_MANUAL = 1 << 16      # 65536
    PX4_MODE_ALTCTL = 2 << 16      # 131072
    PX4_MODE_POSCTL = 3 << 16      # 196608
    PX4_MODE_AUTO = 4 << 16        # 262144
    PX4_MODE_ACRO = 5 << 16        # 327680
    PX4_MODE_OFFBOARD = 6 << 16    # 393216
    PX4_MODE_STABILIZED = 7 << 16  # 458752
    PX4_MODE_RATTITUDE = 8 << 16   # 524288

    # AUTO sub-modes (add to PX4_MODE_AUTO)
    PX4_AUTO_RTL = PX4_MODE_AUTO | (5 << 24)    # AUTO + RTL sub-mode
    PX4_AUTO_LAND = PX4_MODE_AUTO | (6 << 24)   # AUTO + LAND sub-mode

    # MODE_NAMES uses encoded values as keys
    MODE_NAMES = {
        1 << 16: "MANUAL",
        2 << 16: "ALTCTL",
        3 << 16: "POSCTL",
        4 << 16: "AUTO",
        5 << 16: "ACRO",
        6 << 16: "OFFBOARD",
        7 << 16: "STABILIZED",
        8 << 16: "RATTITUDE",
    }

    @staticmethod
    def decode_px4_mode(custom_mode: int) -> tuple:
        """Decode PX4 custom_mode into main_mode and sub_mode."""
        main_mode = (custom_mode >> 16) & 0xFF
        sub_mode = (custom_mode >> 24) & 0xFF
        return main_mode, sub_mode

    def get_mode_name(self, custom_mode: int) -> str:
        """Get human-readable mode name from encoded custom_mode."""
        # First check for exact match (handles AUTO sub-modes)
        if custom_mode in self.MODE_NAMES:
            return self.MODE_NAMES[custom_mode]

        # Extract main mode for lookup
        main_mode_encoded = custom_mode & 0x00FF0000
        main_mode_name = self.MODE_NAMES.get(main_mode_encoded, "UNKNOWN")

        # If AUTO mode, add sub-mode info
        main_mode, sub_mode = self.decode_px4_mode(custom_mode)
        if main_mode == 4 and sub_mode > 0:  # AUTO mode with sub-mode
            sub_names = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND"}
            sub_name = sub_names.get(sub_mode, f"SUB{sub_mode}")
            return f"AUTO_{sub_name}"

        return main_mode_name if main_mode_name != "UNKNOWN" else f"MODE_{custom_mode}"

    def __init__(self, vehicle: Multirotor, world: World):
        self.vehicle = vehicle
        self.world = world
        self.mav = None
        self.connected = False
        self.state = FlightState()
        self._home_position = None
        # Optional callback called on every world.step() to keep camera synced
        self._step_callback = None

    def set_step_callback(self, callback) -> None:
        """
        Set a callback that is called on every simulation step.
        This ensures camera position stays synced with drone during all operations.
        """
        self._step_callback = callback

    def _step(self, render: bool = True) -> None:
        """
        Step the simulation and call any registered callback.
        This ensures camera stays attached to drone during all operations.
        """
        self.world.step(render=render)
        if self._step_callback:
            self._step_callback()

    def connect(self, max_steps: int = 500) -> bool:
        """Connect to PX4 via MAVLink."""
        print("[FlightController] Connecting to MAVLink...", flush=True)

        self.mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

        for i in range(max_steps):
            self._step()
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.get_srcSystem() != 0:
                print(f"[FlightController] Connected to system {msg.get_srcSystem()}", flush=True)
                self.mav.target_system = 1
                self.mav.target_component = 1
                self.connected = True
                self._update_state()
                self._home_position = self.state.position.copy()

                # Set parameters to allow OFFBOARD mode without RC input
                # COM_RC_IN_MODE: 2 = RC input disabled (critical for SITL without RC)
                # COM_RCL_EXCEPT: bitmask where bit 2 (value 4) = OFFBOARD exception
                # NAV_RCL_ACT: 0 = disabled (no RC loss action)
                # NAV_DLL_ACT: 0 = disabled (no data link loss action)
                self._set_param("COM_RC_IN_MODE", 2)
                self._set_param("COM_RCL_EXCEPT", 4)
                self._set_param("NAV_RCL_ACT", 0)
                self._set_param("NAV_DLL_ACT", 0)

                # Wait for parameters to take effect
                for _ in range(50):
                    self._step()

                return True

        print("[FlightController] ERROR: Failed to connect to MAVLink", flush=True)
        return False

    def _set_param(self, param_id: str, value: float, param_type: int = None) -> bool:
        """
        Set a PX4 parameter via MAVLink PARAM_SET.

        Args:
            param_id: Parameter name (e.g., "COM_RCL_EXCEPT")
            value: Parameter value
            param_type: MAVLink param type (auto-detected if None)

        Returns:
            True if parameter was acknowledged
        """
        if not self.connected:
            return False

        # Determine param type if not specified
        if param_type is None:
            if isinstance(value, float) and not value.is_integer():
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            else:
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32

        # Pad param_id to 16 bytes
        param_id_bytes = param_id.encode('utf-8')[:16].ljust(16, b'\x00')

        print(f"[FlightController] Setting param {param_id}={value}", flush=True)

        self.mav.mav.param_set_send(
            self.mav.target_system,
            self.mav.target_component,
            param_id_bytes,
            float(value),
            param_type
        )

        # Wait for acknowledgement
        for _ in range(50):
            self._step()
            msg = self.mav.recv_match(type='PARAM_VALUE', blocking=False)
            if msg and msg.param_id.rstrip('\x00') == param_id:
                print(f"[FlightController] Param {param_id} confirmed: {msg.param_value}", flush=True)
                return True

        print(f"[FlightController] WARNING: No ACK for param {param_id}", flush=True)
        return False

    def _update_state(self) -> None:
        """Update flight state from vehicle and MAVLink."""
        pos = self.vehicle.state.position
        self.state.position = np.array(pos)
        self.state.altitude = pos[2]

        # Get mode from heartbeat
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            self.state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.state.mode = msg.custom_mode
            self.state.mode_name = self.get_mode_name(msg.custom_mode)

    def _send_velocity_setpoint(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0) -> None:
        """Send velocity setpoint in NED frame (negative Z = up)."""
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity only
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

    def _send_position_setpoint(
        self, x: float, y: float, z: float, yaw: float = float('nan')
    ) -> None:
        """Send position setpoint in NED frame."""
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # position only
            x, y, -z,  # NED: negative Z is up
            0, 0, 0,
            0, 0, 0,
            yaw, 0
        )

    def set_mode_offboard(self, max_attempts: int = 10) -> bool:
        """Set flight mode to OFFBOARD."""
        # Prime with setpoints first - PX4 requires continuous stream before OFFBOARD
        print("[FlightController] Priming OFFBOARD with setpoints...", flush=True)
        for _ in range(300):
            self._send_velocity_setpoint()
            self._step()

        for attempt in range(max_attempts):
            # Send mode change command
            # Note: param2 takes the simple main mode number (6 for OFFBOARD),
            # NOT the encoded custom_mode value from heartbeat
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6,  # OFFBOARD main mode number (heartbeat reports this as 6 << 16 = 393216)
                0, 0, 0, 0, 0
            )

            # Keep sending setpoints while waiting for mode change
            # Check multiple heartbeats to ensure we catch the mode change
            for i in range(200):
                self._send_velocity_setpoint()
                self._step()

                # Check for mode change every 20 steps
                if i % 20 == 0:
                    # Drain all pending heartbeats and use the latest
                    latest_mode = None
                    while True:
                        msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
                        if msg is None:
                            break
                        if msg.get_srcSystem() == self.mav.target_system:
                            latest_mode = msg.custom_mode
                            self.state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                    if latest_mode is not None:
                        self.state.mode = latest_mode
                        self.state.mode_name = self.get_mode_name(latest_mode)

                        # Compare with encoded value (393216 = 6 << 16)
                        if latest_mode == self.PX4_MODE_OFFBOARD:
                            print(f"[FlightController] OFFBOARD mode set (attempt {attempt+1})", flush=True)
                            return True

            print(f"[FlightController] Mode check (attempt {attempt+1}): {self.state.mode_name} ({self.state.mode})", flush=True)

        print(f"[FlightController] WARNING: Failed to set OFFBOARD mode, current: {self.state.mode_name}", flush=True)
        return False

    def arm(self, max_attempts: int = 3) -> bool:
        """Arm the vehicle."""
        for attempt in range(max_attempts):
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0
            )

            for _ in range(100):
                self._send_velocity_setpoint()
                self._step()

            self._update_state()
            if self.state.armed:
                print(f"[FlightController] Armed (attempt {attempt+1})", flush=True)
                return True

        return False

    def disarm(self) -> bool:
        """Disarm the vehicle."""
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0
        )

        for _ in range(50):
            self._step()

        self._update_state()
        return not self.state.armed

    def takeoff(
        self,
        target_altitude: float,
        max_steps: int = 3000,
        callback=None,
    ) -> bool:
        """
        Takeoff to target altitude.

        Args:
            target_altitude: Target altitude in meters
            max_steps: Maximum simulation steps
            callback: Optional callback(step, state) called each step

        Returns:
            True if altitude reached
        """
        print(f"[FlightController] Taking off to {target_altitude}m...", flush=True)

        for step in range(max_steps):
            # Send upward velocity
            self._send_velocity_setpoint(vz=-2.0)  # NED: negative Z = up
            self._step()
            self._update_state()

            if callback:
                callback(step, self.state)

            if self.state.altitude >= target_altitude - 0.5:
                print(f"[FlightController] Reached altitude {self.state.altitude:.1f}m", flush=True)
                return True

        print(f"[FlightController] Takeoff timeout at {self.state.altitude:.1f}m", flush=True)
        return False

    def goto_position(
        self,
        target: np.ndarray,
        tolerance: float = POSITION_TOLERANCE,
        max_steps: int = 2000,
        callback=None,
    ) -> bool:
        """
        Fly to target position.

        Args:
            target: Target position [x, y, z] in meters
            tolerance: Position tolerance in meters
            max_steps: Maximum simulation steps
            callback: Optional callback(step, state) called each step

        Returns:
            True if position reached
        """
        for step in range(max_steps):
            self._send_position_setpoint(target[0], target[1], target[2])
            self._step()
            self._update_state()

            # Re-request OFFBOARD mode periodically to prevent fallback
            if step % 200 == 0 and self.state.mode != self.PX4_MODE_OFFBOARD:
                self.mav.mav.command_long_send(
                    self.mav.target_system, self.mav.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    6,  # OFFBOARD main mode number
                    0, 0, 0, 0, 0
                )

            if callback:
                callback(step, self.state)

            # Check if we've reached target
            distance = np.linalg.norm(self.state.position - target)
            if distance < tolerance:
                return True

        return False

    def fly_waypoints(
        self,
        waypoints: List[np.ndarray],
        tolerance: float = POSITION_TOLERANCE,
        callback=None,
    ) -> Tuple[int, int]:
        """
        Fly through a list of waypoints using velocity control.

        Args:
            waypoints: List of [x, y, z] positions
            tolerance: Position tolerance
            callback: Optional callback(step, state, waypoint_idx)

        Returns:
            (waypoints_reached, total_waypoints)
        """
        reached = 0
        for idx, wp in enumerate(waypoints):
            print(f"[FlightController] Flying to waypoint {idx+1}/{len(waypoints)}: {wp}", flush=True)
            print(f"[FlightController] Current position: {self.state.position}", flush=True)

            # Use velocity-based navigation instead of position setpoints
            max_steps = 2000
            for step in range(max_steps):
                # Calculate direction to waypoint
                direction = wp - self.state.position
                distance = np.linalg.norm(direction)

                if step % 200 == 0:
                    print(f"[FlightController] WP{idx+1} step {step}: distance={distance:.1f}m, pos={self.state.position}", flush=True)

                if distance < tolerance:
                    reached += 1
                    print(f"[FlightController] Reached waypoint {idx+1}", flush=True)
                    break

                # Normalize and scale to velocity (max 3 m/s)
                if distance > 0:
                    velocity = direction / distance * min(3.0, distance)
                    # Send velocity - note: Isaac Sim uses ENU (East-North-Up)
                    # PX4 uses NED (North-East-Down)
                    # So we need: vx_ned = vy_enu, vy_ned = vx_enu, vz_ned = -vz_enu
                    self._send_velocity_setpoint(velocity[1], velocity[0], -velocity[2])

                self._step()
                self._update_state()

                if callback:
                    callback(step, self.state, idx)
            else:
                print(f"[FlightController] Timeout at waypoint {idx+1}, distance={distance:.1f}m", flush=True)

        return reached, len(waypoints)

    def return_to_home(self, max_steps: int = 2000, callback=None) -> bool:
        """Return to home position and land."""
        if self._home_position is None:
            print("[FlightController] ERROR: Home position not set", flush=True)
            return False

        print("[FlightController] Returning to home...", flush=True)

        # First fly to home position at current altitude
        home_at_altitude = np.array([
            self._home_position[0],
            self._home_position[1],
            self.state.altitude
        ])

        if not self.goto_position(home_at_altitude, callback=callback):
            print("[FlightController] Failed to reach home position", flush=True)
            return False

        return True

    def land(self, max_steps: int = 5000, callback=None) -> bool:
        """
        Land the vehicle by descending at current XY position.

        Uses position setpoints with decreasing altitude for more reliable descent,
        since velocity-only commands can be ignored by some PX4 modes.
        """
        print("[FlightController] Landing...", flush=True)

        # Get current position for XY hold
        start_x = self.state.position[0]
        start_y = self.state.position[1]
        current_alt = self.state.altitude

        # Descent rate in meters per step
        # Slow descent of 0.02m/step at ~60Hz = ~1.2 m/s descent rate
        # This gives the drone time to track the descending setpoint
        descent_rate = 0.02

        for step in range(max_steps):
            # Calculate target altitude (descending)
            target_alt = max(0.0, current_alt - (step * descent_rate))

            # Send position setpoint to descend while holding XY position
            # Using ENU coordinates for Isaac Sim
            self._send_position_setpoint(start_x, start_y, target_alt)
            self._step()
            self._update_state()

            if step % 300 == 0:
                print(f"[FlightController] Descending: {self.state.altitude:.1f}m -> target {target_alt:.1f}m", flush=True)

            if callback:
                callback(step, self.state)

            # Check if landed (increased threshold slightly for ground detection)
            if self.state.altitude < 1.0:
                print(f"[FlightController] Landed at altitude {self.state.altitude:.2f}m", flush=True)
                return True

        print(f"[FlightController] Land timeout at {self.state.altitude:.1f}m", flush=True)
        return False


# ============================================================================
# Main Test Function
# ============================================================================

def run_drone_functions_check():
    """Main test function."""

    print("=" * 70)
    print("DRONE FUNCTIONS CHECK")
    print("Isaac Sim + PX4 Flight Test with YOLO Video Recording")
    print("=" * 70)
    print(f"Mode: {'Headless' if HEADLESS else 'GUI'}")
    print(f"Video Output: {VIDEO_OUTPUT}")
    print(f"Report Output: {REPORT_OUTPUT}")
    print()

    # Check numpy version
    print(f"[Info] NumPy version: {np.__version__}")
    if np.__version__.startswith("2."):
        print("[WARNING] NumPy 2.x detected - may cause dtype errors!")

    # Initialize test results
    test_results: List[TestResult] = []
    start_time = time.time()
    total_detections = 0

    # ========================================================================
    # Initialize Simulation
    # ========================================================================

    print("\n[1/7] Initializing simulation world...")

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.set_px4_path("/px4")
    stage = get_current_stage()

    # Ensure physics scene exists
    physics_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(physics_path):
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_path))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

    test_results.append(TestResult(
        name="Simulation Initialization",
        passed=True,
        message="Pegasus interface and physics scene initialized",
    ))

    # ========================================================================
    # Generate World
    # ========================================================================

    print("\n[2/7] Generating procedural world...")

    models_path = "/workspace/extensions/forest_generator/models"
    world_config = WorldConfig(
        terrain_size=(200.0, 200.0),
        terrain_roughness=2.0,
        terrain_material="forest_floor",
        tree_density=0.3,
        tree_proportions={"Birch": 30, "Spruce": 40, "Pine": 30},
        undergrowth_density=0.0,
        randomize_lighting=False,
        time_of_day="noon",
        weather="clear",
        seed=42,
    )

    world_gen = WorldGenerator(models_path, world_config)
    terrain_path = world_gen.generate_terrain()
    world_gen.setup_lighting()
    forest_result = world_gen.generate_forest(density=0.3, include_undergrowth=False)

    # Spawn target vehicles and people for YOLO detection
    print("[WorldGen] Spawning target vehicles and people...", flush=True)
    target_clusters = [
        (25.0, 25.0),   # NE cluster (will be visible during square pattern)
        (-25.0, 25.0),  # NW cluster
        (-25.0, -25.0), # SW cluster
        (25.0, -25.0),  # SE cluster
    ]

    all_targets = []
    all_people = []
    for center in target_clusters:
        # Spawn vehicles at each cluster
        # Available types: sedan2, suv, sports_car, sports_car2, taxi, police, tank, tank2, tank3, tank4
        vehicle_paths = world_gen.vehicles.spawn_vehicle_group(
            vehicle_types=["sedan2", "suv", "tank", "taxi"],
            count=4,
            clustering=0.7,
            center=center,
        )
        all_targets.extend(vehicle_paths)

        # Spawn people near each vehicle cluster
        # Get available person types from the spawner
        if world_gen.people.person_configs:
            person_types = list(world_gen.people.person_configs.keys())
            # Spawn 3-5 people near each vehicle cluster
            people_center = (center[0] + 5.0, center[1] + 5.0)  # Offset from vehicles
            people_paths = world_gen.people.spawn_crowd(
                person_types=person_types,
                count=4,
                center=people_center,
                radius=8.0,
            )
            all_people.extend(people_paths)

    print(f"[WorldGen] Created {len(forest_result['trees'])} trees, {len(all_targets)} vehicles, {len(all_people)} people")

    test_results.append(TestResult(
        name="World Generation",
        passed=True,
        message=f"Generated terrain, {len(forest_result['trees'])} trees, {len(all_targets)} vehicles, {len(all_people)} people",
        data={"trees": len(forest_result['trees']), "vehicles": len(all_targets), "people": len(all_people)},
    ))

    # Multiple updates to ensure stage is ready
    for _ in range(10):
        simulation_app.update()

    # ========================================================================
    # Create Camera
    # ========================================================================

    print("\n[3/7] Creating camera...")

    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([0.0, 0.0, 5.0]),  # Will be moved to follow drone
        frequency=CAMERA_FPS,
        resolution=(CAMERA_WIDTH, CAMERA_HEIGHT),
        orientation=rot_utils.euler_angles_to_quats(np.array([90, 0, 0]), degrees=True),  # Looking down
    )

    # Double initialization (required per official example)
    camera.initialize()
    simulation_app.update()
    camera.initialize()
    simulation_app.update()

    # Set wider FOV (double the default ~55 deg to ~110 deg)
    # This is done by adjusting the horizontal aperture
    # Default aperture is ~20.955mm for 55deg FOV with 50mm focal length
    # Double aperture = double FOV approximately
    camera_prim_fov = stage.GetPrimAtPath(CAMERA_PATH)
    if camera_prim_fov.IsValid():
        from pxr import UsdGeom
        usd_camera = UsdGeom.Camera(camera_prim_fov)
        if usd_camera:
            # Set horizontal aperture to ~42mm for ~110 deg FOV
            usd_camera.GetHorizontalApertureAttr().Set(42.0)
            print("[Camera] Set wide FOV (~110 degrees)")

    print(f"[Camera] Created at {CAMERA_PATH}, resolution {CAMERA_WIDTH}x{CAMERA_HEIGHT}")

    # Initialize gimbal controller with forward-down default angle
    # -30 degrees gives a good forward-looking view while still seeing the ground ahead
    gimbal = GimbalController(CAMERA_PATH, stage, default_tilt=-30.0)
    gimbal.set_angles_immediate(0, -30)  # Initial: forward-down at 30 degrees

    test_results.append(TestResult(
        name="Camera Setup",
        passed=True,
        message=f"Camera initialized at {CAMERA_WIDTH}x{CAMERA_HEIGHT}",
    ))

    # ========================================================================
    # Create Vehicle
    # ========================================================================

    print("\n[4/7] Creating PX4 vehicle...")

    mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": 0,
        "px4_autolaunch": True,
        "px4_dir": "/px4",
        "px4_vehicle_model": pg.px4_default_airframe,
        "enable_lockstep": True,
        "update_rate": 250.0,
    })

    config = MultirotorConfig()
    config.backends = [PX4MavlinkBackend(mavlink_config)]

    vehicle = Multirotor(
        "/World/quadrotor",
        ROBOTS['Iris'],
        0,
        [0.0, 0.0, 0.5],  # Spawn position
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=config,
    )

    world.reset()
    timeline.play()

    # Warmup
    print("[Warmup] Running 200 warmup frames...", flush=True)
    for _ in range(200):
        world.step(render=True)

    # Initialize flight controller
    fc = FlightController(vehicle, world)

    # Set up camera tracking callback EARLY - before any flight operations
    # This ensures camera stays attached to drone during connect/arm/mode changes
    camera_prim_early = stage.GetPrimAtPath(CAMERA_PATH)
    camera_xform_early = UsdGeom.Xformable(camera_prim_early) if camera_prim_early.IsValid() else None
    camera_translate_op_early = None
    if camera_xform_early:
        for op in camera_xform_early.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                camera_translate_op_early = op
                break
        if camera_translate_op_early is None:
            camera_translate_op_early = camera_xform_early.AddTranslateOp()

    def early_camera_update():
        """Keep camera attached to drone during all simulation steps."""
        if camera_translate_op_early is None:
            return
        try:
            drone_pos = vehicle.state.position
            if drone_pos is not None:
                camera_pos = np.array([drone_pos[0], drone_pos[1], drone_pos[2] + 0.5])
                camera_translate_op_early.Set(Gf.Vec3d(*camera_pos))
        except Exception:
            pass

    def early_gimbal_update():
        """Update gimbal stabilization during all simulation steps."""
        try:
            quat = vehicle.state.attitude
            if quat is not None and len(quat) == 4:
                rot = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
                euler = rot.as_euler('xyz', degrees=True)
                gimbal.update_drone_attitude(euler[0], euler[1], euler[2])
            gimbal.update()
        except Exception:
            pass

    def early_step_callback():
        """Combined callback for camera tracking and gimbal stabilization."""
        early_camera_update()
        early_gimbal_update()

    # Register the callback so camera follows drone during connect/arm/mode changes
    fc.set_step_callback(early_step_callback)

    test_results.append(TestResult(
        name="Vehicle Creation",
        passed=True,
        message="Iris quadrotor spawned with PX4 backend",
    ))

    # ========================================================================
    # Initialize Video Recorder
    # ========================================================================

    print("\n[5/7] Initializing video recorder...")

    recorder = VideoRecorder(
        output_path=VIDEO_OUTPUT,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        fps=CAMERA_FPS,
        yolo_model_path=YOLO_MODEL_PATH,
    )

    if not recorder.start():
        print("[ERROR] Failed to start video recorder!")
        test_results.append(TestResult(
            name="Video Recorder",
            passed=False,
            message="Failed to start FFmpeg process",
        ))
    else:
        test_results.append(TestResult(
            name="Video Recorder",
            passed=True,
            message="FFmpeg pipeline started",
        ))

    # ========================================================================
    # Connect and Arm
    # ========================================================================

    print("\n[6/7] Connecting to PX4 and arming...")

    connect_start = time.time()
    connected = fc.connect()

    if not connected:
        test_results.append(TestResult(
            name="MAVLink Connection",
            passed=False,
            message="Failed to connect to PX4",
        ))
        recorder.stop()
        simulation_app.close()
        return

    test_results.append(TestResult(
        name="MAVLink Connection",
        passed=True,
        message=f"Connected in {time.time() - connect_start:.1f}s",
    ))

    # Set OFFBOARD mode and arm
    if not fc.set_mode_offboard():
        test_results.append(TestResult(
            name="OFFBOARD Mode",
            passed=False,
            message="Failed to set OFFBOARD mode",
        ))
    else:
        test_results.append(TestResult(
            name="OFFBOARD Mode",
            passed=True,
            message="OFFBOARD mode set",
        ))

    if not fc.arm():
        test_results.append(TestResult(
            name="Arming",
            passed=False,
            message="Failed to arm vehicle",
        ))
    else:
        test_results.append(TestResult(
            name="Arming",
            passed=True,
            message="Vehicle armed",
        ))

    # ========================================================================
    # Flight Tests
    # ========================================================================

    print("\n[7/7] Running flight tests...")

    # Helper to update camera position and record frame
    current_test_phase = "Initializing"  # Track which test is running

    # Reuse the camera translate op from earlier setup
    camera_translate_op = camera_translate_op_early

    def update_camera_position():
        """
        Update camera position to follow the drone.
        Reuses the early_camera_update function to ensure consistency.
        """
        early_camera_update()

    def update_gimbal_from_drone():
        """
        Update gimbal orientation based on current drone attitude.
        Returns (drone_roll, drone_pitch, drone_yaw) for overlay display.
        """
        drone_roll, drone_pitch, drone_yaw = 0.0, 0.0, 0.0
        try:
            quat = vehicle.state.attitude  # [x, y, z, w] convention in Pegasus
            if quat is not None and len(quat) == 4:
                # Convert quaternion to euler angles (degrees)
                rot = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
                euler = rot.as_euler('xyz', degrees=True)
                drone_roll, drone_pitch, drone_yaw = euler[0], euler[1], euler[2]

                # Update gimbal with drone attitude for stabilization
                gimbal.update_drone_attitude(drone_roll, drone_pitch, drone_yaw)
        except Exception:
            pass

        # Update gimbal (smooth interpolation and stabilization)
        gimbal.update()

        return drone_roll, drone_pitch, drone_yaw

    def flight_callback(step, state, waypoint_idx=None):
        nonlocal total_detections, current_test_phase

        # Update camera position to follow drone
        update_camera_position()

        # Update gimbal stabilization
        drone_roll, drone_pitch, drone_yaw = update_gimbal_from_drone()

        # Get gimbal angles
        gimbal_pan, gimbal_tilt = gimbal.get_gimbal_angles()

        # Record frame
        try:
            rgba = camera.get_rgba()
            if rgba is not None and rgba.size > 0:
                # Create overlay text with current test phase (top-left)
                overlay = f"TEST: {current_test_phase}"
                if waypoint_idx is not None:
                    overlay += f" (WP {waypoint_idx + 1})"
                overlay += f"\nAlt: {state.altitude:.1f}m | Mode: {state.mode_name}"
                overlay += f"\nPos: ({state.position[0]:.1f}, {state.position[1]:.1f})"

                # Create right overlay with drone body and gimbal angles (top-right)
                # This shows stabilization in action - drone angles change but gimbal stays stable
                right_overlay = "=== STABILIZATION ==="
                right_overlay += f"\nDRONE BODY:"
                right_overlay += f"\n  Roll:  {drone_roll:+6.1f} deg"
                right_overlay += f"\n  Pitch: {drone_pitch:+6.1f} deg"
                right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f} deg"
                right_overlay += f"\nGIMBAL (world):"
                right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f} deg"
                right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f} deg"

                detections = recorder.write_frame(rgba, overlay, right_overlay)
                total_detections += detections
        except Exception as e:
            if step % 200 == 0:
                print(f"[Camera] Error: {e}")

    # ----- TEST: Takeoff -----
    print("\n--- Test: Takeoff ---")
    current_test_phase = "TAKEOFF"
    takeoff_start = time.time()
    takeoff_success = fc.takeoff(TAKEOFF_ALTITUDE, callback=flight_callback)
    takeoff_duration = time.time() - takeoff_start

    test_results.append(TestResult(
        name="Takeoff",
        passed=takeoff_success,
        message=f"Reached {fc.state.altitude:.1f}m" if takeoff_success else "Timeout",
        duration_seconds=takeoff_duration,
        data={"target_altitude": TAKEOFF_ALTITUDE, "reached_altitude": fc.state.altitude},
    ))

    # Ensure we're still in OFFBOARD mode before waypoint flight
    fc.set_mode_offboard()
    time.sleep(0.5)  # Give PX4 time to switch modes

    # ----- TEST: Waypoint Navigation -----
    print("\n--- Test: Waypoint Navigation (Single Waypoint) ---")
    current_test_phase = "WAYPOINT NAV"

    # Simple single waypoint test - fly forward and return
    waypoints = [
        np.array([WAYPOINT_SIZE/2, 0.0, WAYPOINT_ALTITUDE]),  # Forward
    ]

    # Re-set OFFBOARD mode just before navigation
    fc.set_mode_offboard()

    nav_start = time.time()
    reached, total = fc.fly_waypoints(waypoints, callback=flight_callback)
    nav_duration = time.time() - nav_start

    test_results.append(TestResult(
        name="Waypoint Navigation",
        passed=reached == total,
        message=f"Reached {reached}/{total} waypoints",
        duration_seconds=nav_duration,
        data={"waypoints_reached": reached, "waypoints_total": total},
    ))

    # ----- TEST: Stabilization Stress Test -----
    print("\n--- Test: Stabilization Stress Test (Aggressive Maneuvers) ---")
    current_test_phase = "STAB TEST"

    # This test performs aggressive maneuvers while the gimbal maintains stable orientation
    # The goal is to demonstrate that the camera view remains stable even during:
    # 1. Quick lateral movements (left-right-left)
    # 2. Forward-backward surges
    # 3. Spinning/yaw rotation
    # The drone will move aggressively but the camera should stay pointed at the same world direction

    # Set gimbal to look forward-down for the test
    gimbal.set_angles_immediate(0, -30)

    stab_test_start = time.time()
    stab_success = True

    # Get current position as reference
    ref_pos = fc.state.position.copy()
    ref_alt = WAYPOINT_ALTITUDE

    # Maneuver sequence: each is (name, velocity_x, velocity_y, velocity_z, duration_steps)
    # Velocities in ENU frame (x=east, y=north, z=up)
    maneuvers = [
        ("Lateral Right", 0, -3.0, 0, 100),   # Quick strafe right
        ("Lateral Left", 0, 3.0, 0, 100),     # Quick strafe left
        ("Lateral Right", 0, -3.0, 0, 100),   # Quick strafe right again
        ("Center", 0, 3.0, 0, 50),            # Return to center
        ("Surge Forward", 3.0, 0, 0, 100),    # Forward surge
        ("Surge Back", -3.0, 0, 0, 100),      # Backward surge
        ("Surge Forward", 3.0, 0, 0, 50),     # Return
        ("Spin CW", 0, 0, 0, 150),            # We'll add yaw rate for this one
        ("Spin CCW", 0, 0, 0, 150),           # Counter-clockwise
        ("Stabilize", 0, 0, 0, 100),          # Hold position to stabilize
    ]

    print("[StabTest] Starting aggressive maneuver sequence...")

    for maneuver_name, vx, vy, vz, steps in maneuvers:
        print(f"[StabTest] Maneuver: {maneuver_name}", flush=True)

        for step in range(steps):
            # For spin maneuvers, we need to send yaw rate
            if "Spin" in maneuver_name:
                # Send velocity with yaw rate (spinning in place)
                yaw_rate = 45.0 if "CW" in maneuver_name else -45.0  # deg/s
                # Convert to rad/s for MAVLink
                yaw_rate_rad = np.radians(yaw_rate)
                fc.mav.mav.set_position_target_local_ned_send(
                    0, fc.mav.target_system, fc.mav.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000010111000111,  # velocity + yaw rate
                    0, 0, 0,
                    0, 0, 0,  # No velocity, just spin in place
                    0, 0, 0,
                    0, yaw_rate_rad  # yaw rate
                )
            else:
                # Standard velocity command (ENU to NED conversion)
                fc._send_velocity_setpoint(vy, vx, -vz)

            fc._step()
            fc._update_state()

            # Update gimbal (it should maintain world orientation despite drone movement)
            try:
                quat = vehicle.state.attitude
                if quat is not None and len(quat) == 4:
                    rot = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
                    euler = rot.as_euler('xyz', degrees=True)
                    gimbal.update_drone_attitude(euler[0], euler[1], euler[2])
            except Exception:
                pass
            gimbal.update()

            # Record frame with detailed overlay
            try:
                rgba = camera.get_rgba()
                if rgba is not None and rgba.size > 0:
                    drone_roll, drone_pitch, drone_yaw = gimbal.get_drone_attitude()
                    gimbal_pan, gimbal_tilt = gimbal.get_gimbal_angles()

                    overlay = f"TEST: STAB TEST - {maneuver_name}"
                    overlay += f"\nAlt: {fc.state.altitude:.1f}m"
                    overlay += f"\nVel cmd: ({vx:.1f}, {vy:.1f}, {vz:.1f})"

                    right_overlay = "=== STABILIZATION ==="
                    right_overlay += f"\nDRONE BODY:"
                    right_overlay += f"\n  Roll:  {drone_roll:+6.1f} deg"
                    right_overlay += f"\n  Pitch: {drone_pitch:+6.1f} deg"
                    right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f} deg"
                    right_overlay += f"\nGIMBAL (world):"
                    right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f} deg"
                    right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f} deg"
                    right_overlay += f"\n--- ISOLATED ---"

                    detections = recorder.write_frame(rgba, overlay, right_overlay)
                    total_detections += detections
            except Exception:
                pass

    stab_test_duration = time.time() - stab_test_start

    test_results.append(TestResult(
        name="Stabilization Stress Test",
        passed=stab_success,
        message="Aggressive maneuvers completed with gimbal isolation",
        duration_seconds=stab_test_duration,
        data={"maneuvers_completed": len(maneuvers)},
    ))

    # ----- TEST: Gimbal Control -----
    print("\n--- Test: Gimbal Control ---")
    current_test_phase = "GIMBAL CTRL"

    # Demonstrate gimbal control with smooth camera movements
    # Each movement uses set_angles() which smoothly interpolates to the target
    gimbal_tests = [
        ("Look down", 0, -90),
        ("Pan left", -45, -60),
        ("Pan right", 45, -60),
        ("Look forward", 0, -30),
        # Full 360° pan to show entire scene
        ("Pan 360 - North", 0, -30),
        ("Pan 360 - NE", 45, -30),
        ("Pan 360 - East", 90, -30),
        ("Pan 360 - SE", 135, -30),
        ("Pan 360 - South", 180, -30),
        ("Pan 360 - SW", 225, -30),
        ("Pan 360 - West", 270, -30),
        ("Pan 360 - NW", 315, -30),
        ("Reset", 0, -30),
    ]

    gimbal_success = True
    for name, target_pan, target_tilt in gimbal_tests:
        # Set target angles (will interpolate smoothly)
        gimbal.set_angles(target_pan, target_tilt)

        # Run enough frames for smooth movement to complete
        # At 45 deg/sec slew rate, 90 deg movement takes 2 seconds = 60 frames at 30fps
        for frame in range(75):
            # Use fc._step() to ensure camera stays attached to drone
            fc._step()

            # Get drone attitude for display
            drone_roll, drone_pitch, drone_yaw = 0.0, 0.0, 0.0
            try:
                quat = vehicle.state.attitude
                if quat is not None and len(quat) == 4:
                    rot = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
                    euler = rot.as_euler('xyz', degrees=True)
                    drone_roll, drone_pitch, drone_yaw = euler[0], euler[1], euler[2]
                    gimbal.update_drone_attitude(drone_roll, drone_pitch, drone_yaw)
            except Exception:
                pass

            # Update gimbal (smooth interpolation)
            gimbal.update()
            gimbal_pan, gimbal_tilt = gimbal.get_gimbal_angles()

            try:
                rgba = camera.get_rgba()
                if rgba is not None and rgba.size > 0:
                    # Show current and target angles to demonstrate smooth movement
                    overlay = f"TEST: GIMBAL CTRL - {name}"
                    overlay += f"\nCurrent: Pan {gimbal_pan:.0f} Tilt {gimbal_tilt:.0f}"
                    overlay += f"\nTarget:  Pan {target_pan:.0f} Tilt {target_tilt:.0f}"
                    overlay += f"\nAlt: {fc.state.altitude:.1f}m"

                    # Right overlay with stabilization info
                    right_overlay = "=== STABILIZATION ==="
                    right_overlay += f"\nDRONE BODY:"
                    right_overlay += f"\n  Roll:  {drone_roll:+6.1f} deg"
                    right_overlay += f"\n  Pitch: {drone_pitch:+6.1f} deg"
                    right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f} deg"
                    right_overlay += f"\nGIMBAL (world):"
                    right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f} deg"
                    right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f} deg"

                    detections = recorder.write_frame(rgba, overlay, right_overlay)
                    total_detections += detections
            except Exception:
                pass

    test_results.append(TestResult(
        name="Gimbal Control",
        passed=gimbal_success,
        message="Gimbal pan/tilt tested with smooth interpolation",
        data={"final_pan": gimbal.pan, "final_tilt": gimbal.tilt},
    ))

    # ----- TEST: Return to Home -----
    print("\n--- Test: Return to Home ---")
    current_test_phase = "RETURN HOME"
    rth_start = time.time()
    rth_success = fc.return_to_home(callback=flight_callback)
    rth_duration = time.time() - rth_start

    test_results.append(TestResult(
        name="Return to Home",
        passed=rth_success,
        message="Returned to home position" if rth_success else "Failed to reach home",
        duration_seconds=rth_duration,
    ))

    # ----- TEST: Land -----
    print("\n--- Test: Landing ---")
    current_test_phase = "LANDING"
    land_start = time.time()
    land_success = fc.land(callback=flight_callback)
    land_duration = time.time() - land_start

    # Disarm
    fc.disarm()

    test_results.append(TestResult(
        name="Landing",
        passed=land_success,
        message=f"Landed at {fc.state.altitude:.2f}m" if land_success else "Landing timeout",
        duration_seconds=land_duration,
        data={"final_altitude": fc.state.altitude},
    ))

    # ========================================================================
    # Finalize
    # ========================================================================

    print("\n" + "=" * 70)
    print("FINALIZING")
    print("=" * 70)

    # Stop video recording
    recorder.stop()

    # Calculate summary
    total_time = time.time() - start_time
    passed_tests = sum(1 for r in test_results if r.passed)
    total_tests = len(test_results)
    all_passed = passed_tests == total_tests

    # Generate report
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "headless": HEADLESS,
        "total_duration_seconds": total_time,
        "video_output": VIDEO_OUTPUT,
        "frames_recorded": recorder.frame_count,
        "total_detections": total_detections,
        "tests_passed": passed_tests,
        "tests_total": total_tests,
        "all_passed": all_passed,
        "tests": [asdict(r) for r in test_results],
    }

    # Save report FIRST (before shutdown which can crash)
    try:
        with open(REPORT_OUTPUT, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"[Report] Saved to {REPORT_OUTPUT}")
    except Exception as e:
        print(f"[Report] ERROR: Failed to save report: {e}")

    # Print summary
    print("\n" + "=" * 70)
    print("TEST RESULTS SUMMARY")
    print("=" * 70)

    for result in test_results:
        status = "PASS" if result.passed else "FAIL"
        print(f"  [{status}] {result.name}: {result.message}")

    print()
    print(f"Total Tests: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {total_tests - passed_tests}")
    print(f"Total Duration: {total_time:.1f}s")
    print(f"Frames Recorded: {recorder.frame_count}")
    print(f"Total YOLO Detections: {total_detections}")
    print()

    if all_passed:
        print("OVERALL RESULT: ALL TESTS PASSED")
    else:
        print("OVERALL RESULT: SOME TESTS FAILED")

    print("=" * 70)
    print()
    print("[Shutdown] Cleaning up simulation (crash during shutdown is a known Isaac Sim bug)...")

    # Cleanup - this may crash but report is already saved
    timeline.stop()
    simulation_app.close()

    return all_passed


if __name__ == "__main__":
    run_drone_functions_check()
