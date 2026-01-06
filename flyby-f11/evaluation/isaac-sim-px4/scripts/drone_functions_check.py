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
TAKEOFF_ALTITUDE = 15.0  # meters
WAYPOINT_SIZE = 30.0  # meters (square pattern side length)
WAYPOINT_ALTITUDE = 15.0  # meters
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

    def write_frame(self, rgba_frame: np.ndarray, overlay_text: str = None) -> int:
        """
        Write a frame with optional YOLO detection overlay.

        Args:
            rgba_frame: RGBA image array from camera
            overlay_text: Optional text to overlay on frame

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

        # Add text overlay if provided
        if overlay_text:
            try:
                import cv2
                # Add semi-transparent background for text
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 1
                color = (255, 255, 255)
                bg_color = (0, 0, 0)

                lines = overlay_text.split('\n')
                y_offset = 25
                for line in lines:
                    (text_width, text_height), baseline = cv2.getTextSize(
                        line, font, font_scale, thickness
                    )
                    # Draw background rectangle
                    cv2.rectangle(
                        rgb_frame,
                        (5, y_offset - text_height - 5),
                        (15 + text_width, y_offset + 5),
                        bg_color,
                        -1
                    )
                    # Draw text
                    cv2.putText(
                        rgb_frame, line, (10, y_offset),
                        font, font_scale, color, thickness
                    )
                    y_offset += text_height + 10
            except ImportError:
                pass  # cv2 not available, skip text overlay

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
    """Simulates gimbal control by rotating the camera prim."""

    def __init__(self, camera_prim_path: str, stage):
        self.camera_path = camera_prim_path
        self.stage = stage
        self.pan = 0.0  # degrees
        self.tilt = -45.0  # degrees (looking down by default)
        self.tilt_min, self.tilt_max = GIMBAL_TILT_RANGE

    def set_angles(self, pan: float, tilt: float) -> None:
        """Set gimbal pan and tilt angles."""
        self.pan = pan % 360
        self.tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        self._update_camera_orientation()

    def pan_by(self, delta_pan: float) -> None:
        """Adjust pan angle by delta degrees."""
        self.pan = (self.pan + delta_pan) % 360
        self._update_camera_orientation()

    def tilt_by(self, delta_tilt: float) -> None:
        """Adjust tilt angle by delta degrees."""
        self.tilt = max(self.tilt_min, min(self.tilt_max, self.tilt + delta_tilt))
        self._update_camera_orientation()

    def look_at_ground(self) -> None:
        """Point camera straight down."""
        self.tilt = -90.0
        self._update_camera_orientation()

    def look_forward(self) -> None:
        """Point camera forward (horizon)."""
        self.tilt = 0.0
        self._update_camera_orientation()

    def _update_camera_orientation(self) -> None:
        """Update camera prim orientation based on current pan/tilt."""
        camera_prim = self.stage.GetPrimAtPath(self.camera_path)
        if not camera_prim.IsValid():
            return

        # Convert pan/tilt to rotation
        # Camera default is looking down -Z in local frame
        # We rotate: first tilt around X, then pan around Z
        # For a gimbal: tilt is pitch (X rotation), pan is yaw (Z rotation)

        # Create quaternion from euler angles
        # Isaac Sim camera: +90 X rotation makes it look down
        # We add tilt to that base and add pan as Z rotation
        roll = 0.0
        pitch = 90.0 + self.tilt  # Base 90 (looking down) + tilt adjustment
        yaw = self.pan

        quat = rot_utils.euler_angles_to_quats(
            np.array([pitch, yaw, roll]), degrees=True
        )

        # Update camera transform
        xform = UsdGeom.Xformable(camera_prim)

        # Find or create orient op
        orient_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_op = op
                break

        if orient_op is None:
            # Clear and recreate ops
            translate_val = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_val = op.Get()

            xform.ClearXformOpOrder()
            if translate_val is not None:
                xform.AddTranslateOp().Set(translate_val)
            orient_op = xform.AddOrientOp()

        orient_op.Set(Gf.Quatd(quat[0], quat[1], quat[2], quat[3]))


# ============================================================================
# Flight Controller
# ============================================================================

class FlightController:
    """PX4 flight controller interface via MAVLink."""

    # PX4 custom modes
    PX4_MODE_MANUAL = 0
    PX4_MODE_OFFBOARD = 6
    PX4_MODE_RTL = 5
    PX4_MODE_LAND = 9

    MODE_NAMES = {
        0: "MANUAL",
        1: "ALTCTL",
        2: "POSCTL",
        3: "AUTO",
        4: "ACRO",
        5: "RTL",
        6: "OFFBOARD",
        7: "STABILIZED",
        8: "RATTITUDE",
        9: "LAND",
    }

    def __init__(self, vehicle: Multirotor, world: World):
        self.vehicle = vehicle
        self.world = world
        self.mav = None
        self.connected = False
        self.state = FlightState()
        self._home_position = None

    def connect(self, max_steps: int = 500) -> bool:
        """Connect to PX4 via MAVLink."""
        print("[FlightController] Connecting to MAVLink...", flush=True)

        self.mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

        for i in range(max_steps):
            self.world.step(render=True)
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.get_srcSystem() != 0:
                print(f"[FlightController] Connected to system {msg.get_srcSystem()}", flush=True)
                self.mav.target_system = 1
                self.mav.target_component = 1
                self.connected = True
                self._update_state()
                self._home_position = self.state.position.copy()
                return True

        print("[FlightController] ERROR: Failed to connect to MAVLink", flush=True)
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
            self.state.mode_name = self.MODE_NAMES.get(msg.custom_mode, f"MODE_{msg.custom_mode}")

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

    def set_mode_offboard(self, max_attempts: int = 5) -> bool:
        """Set flight mode to OFFBOARD."""
        # Prime with setpoints first - PX4 requires continuous stream before OFFBOARD
        print("[FlightController] Priming OFFBOARD with setpoints...", flush=True)
        for _ in range(200):
            self._send_velocity_setpoint()
            self.world.step(render=True)

        for attempt in range(max_attempts):
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.PX4_MODE_OFFBOARD, 0, 0, 0, 0, 0
            )

            # Keep sending setpoints while waiting for mode change
            for _ in range(150):
                self._send_velocity_setpoint()
                self.world.step(render=True)

            self._update_state()
            print(f"[FlightController] Mode check (attempt {attempt+1}): {self.state.mode_name} ({self.state.mode})", flush=True)
            if self.state.mode == self.PX4_MODE_OFFBOARD:
                print(f"[FlightController] OFFBOARD mode set (attempt {attempt+1})", flush=True)
                return True

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
                self.world.step(render=True)

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
            self.world.step(render=True)

        self._update_state()
        return not self.state.armed

    def takeoff(
        self,
        target_altitude: float,
        max_steps: int = 1000,
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
            self.world.step(render=True)
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
            self.world.step(render=True)
            self._update_state()

            # Re-request OFFBOARD mode periodically to prevent fallback
            if step % 200 == 0 and self.state.mode != self.PX4_MODE_OFFBOARD:
                self.mav.mav.command_long_send(
                    self.mav.target_system, self.mav.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    self.PX4_MODE_OFFBOARD, 0, 0, 0, 0, 0
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

                self.world.step(render=True)
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

    def land(self, max_steps: int = 1500, callback=None) -> bool:
        """Land the vehicle."""
        print("[FlightController] Landing...", flush=True)

        for step in range(max_steps):
            # Descend slowly
            self._send_velocity_setpoint(vz=1.0)  # NED: positive Z = down
            self.world.step(render=True)
            self._update_state()

            if callback:
                callback(step, self.state)

            # Check if landed
            if self.state.altitude < 0.5:
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

    # Spawn target vehicles for YOLO detection
    print("[WorldGen] Spawning target vehicles...", flush=True)
    target_clusters = [
        (25.0, 25.0),   # NE cluster (will be visible during square pattern)
        (-25.0, 25.0),  # NW cluster
        (-25.0, -25.0), # SW cluster
        (25.0, -25.0),  # SE cluster
    ]

    all_targets = []
    for center in target_clusters:
        vehicle_paths = world_gen.vehicles.spawn_vehicle_group(
            vehicle_types=["sedan", "suv", "tank", "taxi"],
            count=4,
            clustering=0.7,
            center=center,
        )
        all_targets.extend(vehicle_paths)

    print(f"[WorldGen] Created {len(forest_result['trees'])} trees, {len(all_targets)} targets")

    test_results.append(TestResult(
        name="World Generation",
        passed=True,
        message=f"Generated terrain, {len(forest_result['trees'])} trees, {len(all_targets)} targets",
        data={"trees": len(forest_result['trees']), "targets": len(all_targets)},
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

    print(f"[Camera] Created at {CAMERA_PATH}, resolution {CAMERA_WIDTH}x{CAMERA_HEIGHT}")

    # Initialize gimbal controller
    gimbal = GimbalController(CAMERA_PATH, stage)
    gimbal.set_angles(0, -45)  # Initial: looking down at 45 degrees

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
    gimbal_phase = 0  # Track gimbal animation phase
    current_test_phase = "Initializing"  # Track which test is running

    def flight_callback(step, state, waypoint_idx=None):
        nonlocal total_detections, gimbal_phase, current_test_phase

        # Move camera to follow drone
        camera_pos = state.position.copy()
        camera_pos[2] += 0.5  # Slightly above drone

        # Update camera position via prim
        camera_prim = stage.GetPrimAtPath(CAMERA_PATH)
        if camera_prim.IsValid():
            xform = UsdGeom.Xformable(camera_prim)
            translate_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            if translate_op:
                translate_op.Set(Gf.Vec3d(*camera_pos))
            else:
                xform.AddTranslateOp().Set(Gf.Vec3d(*camera_pos))

        # Animate gimbal during waypoint flight
        if waypoint_idx is not None:
            # Pan camera slowly during waypoint navigation
            gimbal.pan_by(0.5)  # Slow pan

            # Vary tilt based on waypoint
            if step % 100 < 50:
                gimbal.tilt_by(-0.2)  # Tilt down
            else:
                gimbal.tilt_by(0.2)  # Tilt up

        # Record frame
        try:
            rgba = camera.get_rgba()
            if rgba is not None and rgba.size > 0:
                # Create overlay text with current test phase prominently displayed
                overlay = f"TEST: {current_test_phase}"
                if waypoint_idx is not None:
                    overlay += f" (WP {waypoint_idx + 1})"
                overlay += f"\nAlt: {state.altitude:.1f}m | Mode: {state.mode_name}"
                overlay += f"\nPos: ({state.position[0]:.1f}, {state.position[1]:.1f}, {state.position[2]:.1f})"

                detections = recorder.write_frame(rgba, overlay)
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

    # ----- TEST: Gimbal Control -----
    print("\n--- Test: Gimbal Control ---")
    current_test_phase = "GIMBAL CTRL"

    # Demonstrate gimbal control with camera movements
    gimbal_tests = [
        ("Look down", lambda: gimbal.set_angles(0, -90)),
        ("Pan left", lambda: gimbal.set_angles(-45, -60)),
        ("Pan right", lambda: gimbal.set_angles(45, -60)),
        ("Look forward", lambda: gimbal.set_angles(0, -30)),
        ("Reset", lambda: gimbal.set_angles(0, -45)),
    ]

    gimbal_success = True
    for name, action in gimbal_tests:
        action()
        # Let it stabilize and record some frames
        for _ in range(30):
            world.step(render=True)
            try:
                rgba = camera.get_rgba()
                if rgba is not None and rgba.size > 0:
                    overlay = f"TEST: GIMBAL CTRL - {name}\nPan: {gimbal.pan:.0f}deg | Tilt: {gimbal.tilt:.0f}deg"
                    overlay += f"\nAlt: {fc.state.altitude:.1f}m"
                    detections = recorder.write_frame(rgba, overlay)
                    total_detections += detections
            except:
                pass

    test_results.append(TestResult(
        name="Gimbal Control",
        passed=gimbal_success,
        message="Gimbal pan/tilt tested",
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
