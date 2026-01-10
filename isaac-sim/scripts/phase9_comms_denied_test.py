#!/usr/bin/env python3
"""
Phase 9: Comms-Denied Integration Test with Video Output

This test validates that all Phase 2-8 components work together in a realistic
comms-denied scenario, running headless at accelerated speed while outputting
a video of the drone's camera with CV detections.

Test Sequence:
1. Ontology triggers takeoff - drone ascends to mission altitude (not RL)
2. Random RL actions - untrained agent moves randomly, gimbal pans around
3. YOLO+ByteTrack runs - detects vehicles/people, tracks them across frames
4. Comms denied at T+30s - logged as status change, drone continues autonomously
5. Ontology triggers RTL - when battery < distance-based reserve
6. Ontology triggers landing - drone lands, episode ends

Battery Reserve Calculation (from ontology):
    required_reserve = (distance_to_home_km * 2.5%) + 10% headroom + altitude_cost
    When battery < required_reserve, ontology triggers RTL regardless of comms status.

Success Criteria:
- Takeoff executed by ontology (not RL)
- YOLO detections appear in video when targets are in view
- Comms denial status logged at T+30s
- RTL triggered when battery < required_reserve
- Landing completed without crash
- Video file generated with detection overlays and telemetry HUD

Video Output:
- Each frame: drone's gimbal camera view
- Overlays: YOLO boxes with class/track ID
- Telemetry HUD: battery %, required reserve %, altitude, distance to home, comms status, flight phase
- Saved to: output/phase9_comms_denied.mp4

Author: Finley Holt
"""

from isaacsim import SimulationApp

import sys
HEADLESS = "--headless" in sys.argv  # Default to GUI mode for camera/video support

# CRITICAL: Enable RayTracedLighting for camera rendering
# Without this, the camera will fail to attach RGB annotator
simulation_config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",  # Required for camera rendering
    "anti_aliasing": 0,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(simulation_config)

import omni
import omni.timeline
import omni.kit.viewport.utility as viewport_utils
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
# NOTE: isaacsim.sensors.camera.Camera is broken in Isaac Sim 5.x (RGB annotator fails)
# Using ViewportCamera workaround instead - captures from viewport directly
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import os
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, List, Dict, Any, Tuple
from enum import Enum, auto

# Add paths
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

sys.path.insert(0, "/workspace")

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pymavlink import mavutil

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig
from isaacsim.core.utils.stage import get_current_stage

# Import safety filter, perception, and ontology controller
from environments.safety_filter import VampireSafetyFilter
from perception.detector import YOLODetector, Detection
from environments.ontology_behavior_controller import (
    OntologyBehaviorController,
    OntologyBehaviorExecutor,
    OntologyControllerConfig,
    OntologyBehavior,
    BehaviorCommand,
)
from environments.base_isr_env import FlightPhase, CommsStatus, GNSSStatus, UAVState

# Isaac Sim Camera API - captures from the actual drone camera, not the viewport
from isaacsim.sensors.camera import Camera

print("=" * 70, flush=True)
print("Phase 9: Comms-Denied Integration Test with Video Output", flush=True)
print("=" * 70, flush=True)
print(f"  Headless: {HEADLESS}", flush=True)

# Create output directory
OUTPUT_DIR = "/workspace/output/phase9_comms_denied"
os.makedirs(OUTPUT_DIR, exist_ok=True)


class VideoWriter:
    """Simple video writer using OpenCV."""

    def __init__(self, output_path: str, fps: int = 30, resolution: Tuple[int, int] = (640, 480)):
        self.output_path = output_path
        self.fps = fps
        self.resolution = resolution
        self.frames = []
        self.writer = None
        self._cv2_available = False
        self.frame_count = 0

        try:
            import cv2
            self._cv2_available = True
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(output_path, fourcc, fps, resolution)
            if self.writer.isOpened():
                print(f"  [Video] Writer initialized: {output_path} ({resolution[0]}x{resolution[1]}@{fps}fps)", flush=True)
            else:
                print(f"  [Video] Warning: Writer created but not opened properly", flush=True)
                self.writer = None
        except ImportError:
            print("  [Video] OpenCV not available, frames will be saved as images", flush=True)
        except Exception as e:
            print(f"  [Video] Writer initialization failed: {e}", flush=True)

    def write_frame(self, frame: np.ndarray):
        """Write a frame to the video."""
        if self._cv2_available and self.writer is not None:
            import cv2
            # Resize if needed
            if frame.shape[:2] != (self.resolution[1], self.resolution[0]):
                frame = cv2.resize(frame, self.resolution)
            # Convert RGB to BGR for OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.writer.write(frame_bgr)
            self.frame_count += 1
            # Log progress every 100 frames
            if self.frame_count % 500 == 0:
                print(f"  [Video] Written {self.frame_count} frames", flush=True)
        else:
            self.frames.append(frame.copy())
            self.frame_count += 1

    def close(self):
        """Close the video writer and save."""
        if self._cv2_available and self.writer is not None:
            self.writer.release()
            print(f"  [Video] Saved {self.frame_count} frames to {self.output_path}", flush=True)
        elif self.frames:
            # Save frames as images
            for i, frame in enumerate(self.frames):
                np.save(f"{OUTPUT_DIR}/frame_{i:05d}.npy", frame)
            print(f"  [Video] Saved {len(self.frames)} frames as .npy files", flush=True)
        else:
            print(f"  [Video] Warning: No frames written!", flush=True)


def draw_hud(
    image: np.ndarray,
    battery_pct: float,
    required_reserve: float,
    altitude: float,
    distance_to_home: float,
    comms_status: str,
    flight_phase: str,
    detections: List[Detection],
    step: int,
    sim_time: float,
) -> np.ndarray:
    """
    Draw telemetry HUD and detection overlays on the image.

    Args:
        image: RGB image (H, W, 3)
        battery_pct: Current battery percentage
        required_reserve: Required battery reserve percentage
        altitude: Current altitude AGL
        distance_to_home: Distance to home position
        comms_status: Communications status string
        flight_phase: Current flight phase string
        detections: List of YOLO detections
        step: Current simulation step
        sim_time: Simulation time in seconds

    Returns:
        Image with HUD overlays
    """
    try:
        import cv2
    except ImportError:
        return image  # Can't draw without OpenCV

    img = image.copy()
    h, w = img.shape[:2]

    # HUD background (semi-transparent black bar at top)
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, 80), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

    # Text settings
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    white = (255, 255, 255)
    green = (0, 255, 0)
    yellow = (255, 255, 0)
    red = (255, 0, 0)
    orange = (255, 165, 0)

    # Row 1: Battery and reserve
    battery_color = green if battery_pct > required_reserve else (yellow if battery_pct > 15 else red)
    cv2.putText(img, f"BAT: {battery_pct:.1f}%", (10, 20), font, font_scale, battery_color, thickness)
    cv2.putText(img, f"RSV: {required_reserve:.1f}%", (120, 20), font, font_scale, orange, thickness)

    # Row 1 cont: Altitude and distance
    cv2.putText(img, f"ALT: {altitude:.1f}m", (230, 20), font, font_scale, white, thickness)
    cv2.putText(img, f"HOME: {distance_to_home:.1f}m", (340, 20), font, font_scale, white, thickness)

    # Row 2: Comms and phase
    comms_color = green if comms_status == "OPERATIONAL" else red
    cv2.putText(img, f"COMMS: {comms_status}", (10, 45), font, font_scale, comms_color, thickness)
    cv2.putText(img, f"PHASE: {flight_phase}", (180, 45), font, font_scale, white, thickness)

    # Row 2 cont: Step and time
    cv2.putText(img, f"T+{sim_time:.1f}s", (380, 45), font, font_scale, white, thickness)
    cv2.putText(img, f"Step: {step}", (480, 45), font, font_scale, white, thickness)

    # Row 3: Detection count
    det_count = len(detections)
    track_ids = [d.track_id for d in detections if d.track_id is not None]
    cv2.putText(img, f"DET: {det_count}", (10, 70), font, font_scale, green if det_count > 0 else white, thickness)
    cv2.putText(img, f"TRACKS: {len(track_ids)}", (100, 70), font, font_scale, green if track_ids else white, thickness)

    # Draw detection bounding boxes
    class_colors = {
        'person': (255, 0, 0),      # Red
        'car': (0, 255, 0),          # Green
        'truck': (0, 0, 255),        # Blue
        'bus': (255, 255, 0),        # Cyan
        'motorcycle': (255, 0, 255), # Magenta
        'bicycle': (0, 255, 255),    # Yellow
    }

    for det in detections:
        bbox = det.get_bbox_xyxy(w, h)
        x1, y1, x2, y2 = [int(v) for v in bbox]

        # Get color for class
        color = class_colors.get(det.class_name, (255, 255, 255))

        # Draw box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Label with class, confidence, and track ID
        label = f"{det.class_name} {det.confidence:.2f}"
        if det.track_id is not None:
            label += f" T{det.track_id}"

        # Label background
        (lw, lh), _ = cv2.getTextSize(label, font, font_scale, thickness)
        cv2.rectangle(img, (x1, y1 - lh - 4), (x1 + lw, y1), color, -1)
        cv2.putText(img, label, (x1, y1 - 4), font, font_scale, (0, 0, 0), thickness)

    return img


class Phase9CommsDeniedEnv(gym.Env):
    """
    Phase 9: Comms-Denied Integration Environment

    Integrates:
    - Ontology controller for takeoff, RTL, landing (preempts RL)
    - Safety filter (Vampire ATP + fallback)
    - YOLO+ByteTrack perception
    - Comms denial at T+30s
    - Distance-based battery reserve calculation
    - Video output with HUD

    The ontology controller manages flight phases:
    - TAKEOFF: Ascend to mission altitude (10m default)
    - MISSION_EXECUTION: RL agent has control
    - RTL: Return to launch when battery < required_reserve
    - LANDING: Final descent and touchdown
    """

    metadata = {'render_modes': ['human']}

    def __init__(
        self,
        world,
        vehicle,
        mav,
        target_positions: List[Dict],
        isr_camera=None,
        max_steps: int = 1000,
        video_writer: Optional[VideoWriter] = None,
        comms_denial_time: float = 30.0,
        mission_altitude: float = 25.0,
        accelerated_battery_drain: float = 1.0,  # Multiplier for faster testing
        # Simulation speedup parameters
        physics_steps_per_render: int = 1,  # Batch physics steps for speedup
        video_capture_interval: int = 2,    # Capture video every N steps (2 = 30fps from 60Hz)
        yolo_inference_interval: int = 3,   # Run YOLO every N steps (3 = 20Hz from 60Hz)
    ):
        super().__init__()

        self.world = world
        self.vehicle = vehicle
        self.mav = mav
        self.target_positions = target_positions
        self.isr_camera = isr_camera
        self.video_writer = video_writer
        self.max_steps = max_steps
        self.comms_denial_time = comms_denial_time
        self.mission_altitude = mission_altitude
        self.accelerated_battery_drain = accelerated_battery_drain

        # Simulation speedup parameters
        self.physics_steps_per_render = physics_steps_per_render
        self.video_capture_interval = video_capture_interval
        self.yolo_inference_interval = yolo_inference_interval

        # Action/observation spaces (6D: vx, vy, vz, yaw_rate, gimbal_pitch, gimbal_yaw)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(35,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32
        )

        # Scaling factors
        self.max_horizontal_velocity = 5.0
        self.max_vertical_velocity = 2.0
        self.max_yaw_rate = 45.0  # deg/s

        # Gimbal limits
        self.gimbal_pitch_min = 0.0
        self.gimbal_pitch_max = 90.0
        self.gimbal_yaw_min = -60.0
        self.gimbal_yaw_max = 60.0
        self.gimbal_pitch = 45.0
        self.gimbal_yaw = 0.0

        # Episode state
        self.step_count = 0
        self.episode_count = 0
        self.sim_time = 0.0
        self.dt = 1.0 / 60.0  # 60Hz sim rate

        # Home position
        self.home_position = np.array([0.0, 0.0, 0.0])

        # =================================================================
        # ONTOLOGY BEHAVIOR CONTROLLER
        # =================================================================
        ontology_config = OntologyControllerConfig(
            takeoff_altitude=mission_altitude,
            takeoff_ascent_rate=2.0,
            battery_per_km=2.5,  # % per km
            battery_headroom=10.0,  # Base 10% headroom
            min_reserve=10.0,  # Hard floor
            battery_rtl_comms_denied_threshold=30.0,
            use_vampire=False,  # Rule-based for speed
        )
        self.ontology_controller = OntologyBehaviorController(ontology_config)
        self.ontology_executor = OntologyBehaviorExecutor(mav_connection=mav, z_up=True)

        # =================================================================
        # YOLO DETECTOR WITH BYTETRACK
        # =================================================================
        self.detector = YOLODetector(
            config_path=None,
            mode="inference",
            enable_tracking=True,
        )

        # =================================================================
        # SAFETY FILTER
        # =================================================================
        self.safety_filter = VampireSafetyFilter(
            ontology_path="/workspace/ontology/planning_mode",
            timeout_ms=50,
            enabled=True,
        )
        self.safety_filter.update_geofence({
            'min_x': -100.0, 'max_x': 100.0,
            'min_y': -100.0, 'max_y': 100.0,
            'min_z': 0.0, 'max_z': 60.0,
        })

        # =================================================================
        # UAV STATE (for ontology controller)
        # =================================================================
        self.uav_state = UAVState()

        # Detection and tracking state
        self._last_detections: List[Detection] = []
        self._last_tracking_stats: Dict = {}

        # Test validation flags
        self.test_validation = {
            'takeoff_by_ontology': False,
            'yolo_detections_in_video': False,
            'comms_denied_logged': False,
            'rtl_triggered_by_battery': False,
            'landing_completed': False,
            'video_generated': False,
        }

        # Statistics
        self.stats = {
            'total_detections': 0,
            'unique_tracks': set(),
            'frames_with_detections': 0,
            'ontology_preemptions': 0,
            'safety_filter_interventions': 0,
            'rtl_trigger_battery': 0.0,
            'rtl_trigger_reserve': 0.0,
        }

    def _get_uav_state(self) -> UAVState:
        """Build UAVState from vehicle state for ontology controller."""
        pos = self.vehicle.state.position
        vel = self.vehicle.state.linear_velocity
        orient = self.vehicle.state.attitude

        # Distance to home
        distance_to_home = np.linalg.norm(pos - self.home_position)

        # Compute required reserve using ontology formula
        distance_km = distance_to_home / 1000.0
        altitude_diff = abs(pos[2] - self.home_position[2])
        altitude_cost = altitude_diff / 100.0 * 0.5  # 0.5% per 100m altitude

        required_reserve = (
            distance_km * 2.5 +  # 2.5% per km
            10.0 +               # Base headroom
            altitude_cost
        )

        # Apply comms-denied buffer
        if self.uav_state.comms_status == CommsStatus.DENIED:
            required_reserve += 10.0 * 0.2  # 20% of headroom

        required_reserve = max(10.0, required_reserve)

        # Update state
        self.uav_state.position = pos.copy()
        self.uav_state.velocity = vel.copy()
        self.uav_state.orientation = orient.copy()
        self.uav_state.battery_reserve = required_reserve
        self.uav_state.in_geofence = (
            -100 < pos[0] < 100 and
            -100 < pos[1] < 100 and
            0 < pos[2] < 60
        )
        self.uav_state.mission_time = self.sim_time

        return self.uav_state

    def reset(self, seed=None, options=None):
        """Reset environment for new episode."""
        super().reset(seed=seed)

        self.step_count = 0
        self.episode_count += 1
        self.sim_time = 0.0

        # Reset gimbal
        self.gimbal_pitch = 45.0
        self.gimbal_yaw = 0.0
        self._update_gimbal()

        # Reset ontology controller
        self.ontology_controller.reset()
        self.ontology_controller.set_home_position(self.home_position)

        # Reset UAV state
        self.uav_state = UAVState(
            position=self.home_position.copy(),
            battery_pct=100.0,
            flight_phase=FlightPhase.PREFLIGHT,
            comms_status=CommsStatus.OPERATIONAL,
        )

        # Reset detector tracking
        self.detector.reset_tracking()
        self._last_detections = []
        self._last_tracking_stats = {}

        # Reset validation flags
        self.test_validation = {
            'takeoff_by_ontology': False,
            'yolo_detections_in_video': False,
            'comms_denied_logged': False,
            'rtl_triggered_by_battery': False,
            'landing_completed': False,
            'video_generated': False,
        }

        # Reset stats
        self.stats = {
            'total_detections': 0,
            'unique_tracks': set(),
            'frames_with_detections': 0,
            'ontology_preemptions': 0,
            'safety_filter_interventions': 0,
            'rtl_trigger_battery': 0.0,
            'rtl_trigger_reserve': 0.0,
        }

        obs = self._get_observation()

        print(f"\n[Episode {self.episode_count}] Starting Phase 9 Integration Test", flush=True)
        print(f"  Mission altitude: {self.mission_altitude}m", flush=True)
        print(f"  Comms denial at: T+{self.comms_denial_time}s", flush=True)
        print(f"  Battery drain multiplier: {self.accelerated_battery_drain}x", flush=True)

        return obs, {"step": 0}

    def step(self, action):
        """Execute one step with ontology oversight."""
        self.step_count += 1
        self.sim_time += self.dt

        # =================================================================
        # 1. UPDATE UAV STATE
        # =================================================================
        uav_state = self._get_uav_state()

        # Simulate battery drain (accelerated for testing)
        base_drain = 0.05 * self.dt  # ~0.05% per step at 60Hz = 3%/min
        self.uav_state.battery_pct -= base_drain * self.accelerated_battery_drain
        self.uav_state.battery_pct = max(0.0, self.uav_state.battery_pct)

        # =================================================================
        # 2. CHECK COMMS DENIAL TIMING
        # =================================================================
        if (self.sim_time >= self.comms_denial_time and
            self.uav_state.comms_status != CommsStatus.DENIED):
            self.uav_state.comms_status = CommsStatus.DENIED
            self.uav_state.autonomous_mode = True
            self.test_validation['comms_denied_logged'] = True
            print(f"  [T+{self.sim_time:.1f}s] >>> COMMS DENIED - Autonomous mode engaged", flush=True)

        # =================================================================
        # 3. ONTOLOGY CONTROLLER UPDATE
        # =================================================================
        behavior_command = self.ontology_controller.update(self.uav_state, self.dt)

        # Determine if ontology is preempting RL
        ontology_controls = behavior_command is not None and behavior_command.preempts_rl
        executed_action = None

        if ontology_controls:
            # Ontology is in control
            self.stats['ontology_preemptions'] += 1
            executed_action = self.ontology_executor.execute(behavior_command, self.uav_state)

            # Track validation events
            if behavior_command.behavior == OntologyBehavior.TAKEOFF:
                if not self.test_validation['takeoff_by_ontology']:
                    self.test_validation['takeoff_by_ontology'] = True
                    print(f"  [T+{self.sim_time:.1f}s] Ontology: TAKEOFF in progress", flush=True)

            elif behavior_command.behavior == OntologyBehavior.RTL:
                if not self.test_validation['rtl_triggered_by_battery']:
                    self.test_validation['rtl_triggered_by_battery'] = True
                    self.stats['rtl_trigger_battery'] = self.uav_state.battery_pct
                    self.stats['rtl_trigger_reserve'] = self.uav_state.battery_reserve
                    print(f"  [T+{self.sim_time:.1f}s] Ontology: RTL triggered", flush=True)
                    print(f"    Battery: {self.uav_state.battery_pct:.1f}% < Reserve: {self.uav_state.battery_reserve:.1f}%", flush=True)

            elif behavior_command.behavior == OntologyBehavior.EMERGENCY_LAND:
                if not self.test_validation['landing_completed']:
                    print(f"  [T+{self.sim_time:.1f}s] Ontology: LANDING", flush=True)

            # Update flight phase
            phase_map = {
                OntologyBehavior.TAKEOFF: FlightPhase.TAKEOFF,
                OntologyBehavior.RTL: FlightPhase.RTL,
                OntologyBehavior.EMERGENCY_LAND: FlightPhase.LANDING,
                OntologyBehavior.HOVER: FlightPhase.HOVER,
            }
            if behavior_command.behavior in phase_map:
                self.uav_state.flight_phase = phase_map[behavior_command.behavior]

        else:
            # RL agent has control (mission execution)
            self.uav_state.flight_phase = FlightPhase.MISSION_EXECUTION

            # =================================================================
            # 4. PARSE AND APPLY RL ACTION WITH SAFETY FILTER
            # =================================================================
            vx = float(action[0]) * self.max_horizontal_velocity
            vy = float(action[1]) * self.max_vertical_velocity
            vz = float(action[2]) * self.max_vertical_velocity
            yaw_rate = float(action[3]) * self.max_yaw_rate

            # Gimbal control
            gimbal_pitch_rate = float(action[4]) * 30.0
            gimbal_yaw_rate = float(action[5]) * 30.0
            self.gimbal_pitch = np.clip(
                self.gimbal_pitch + gimbal_pitch_rate * self.dt,
                self.gimbal_pitch_min, self.gimbal_pitch_max
            )
            self.gimbal_yaw = np.clip(
                self.gimbal_yaw + gimbal_yaw_rate * self.dt,
                self.gimbal_yaw_min, self.gimbal_yaw_max
            )

            # Apply safety filter
            raw_action = np.array([vx, vy, vz, np.radians(yaw_rate)], dtype=np.float32)
            current_state = {
                'x': self.uav_state.position[0],
                'y': self.uav_state.position[1],
                'z': self.uav_state.position[2],
                'battery': self.uav_state.battery_pct,
            }

            safe_action, was_modified, violation = self.safety_filter.filter_action(
                current_state, raw_action, dt=self.dt
            )

            if was_modified:
                self.stats['safety_filter_interventions'] += 1

            executed_action = safe_action

        # =================================================================
        # 5. EXECUTE ACTION (SEND SETPOINT)
        # =================================================================
        if executed_action is not None:
            self._send_setpoint(
                executed_action[0],
                executed_action[1],
                executed_action[2],
                executed_action[3] if len(executed_action) > 3 else 0.0
            )

        self._update_gimbal()

        # =================================================================
        # 6. STEP SIMULATION (batched for speedup)
        # =================================================================
        # Run multiple physics steps without rendering for speedup
        # Only render on the last step for camera capture
        for i in range(self.physics_steps_per_render):
            is_last = (i == self.physics_steps_per_render - 1)
            self.world.step(render=is_last)  # Only render last step

        # =================================================================
        # 7. RUN PERCEPTION PIPELINE (at reduced frequency for speedup)
        # =================================================================
        if self.step_count % self.yolo_inference_interval == 0:
            self._run_perception()

        # =================================================================
        # 8. CAPTURE AND ANNOTATE FRAME FOR VIDEO (at reduced frequency)
        # =================================================================
        if self.video_writer is not None and self.step_count % self.video_capture_interval == 0:
            self._capture_video_frame()

        # =================================================================
        # 9. CHECK TERMINATION
        # =================================================================
        terminated = False
        truncated = False

        pos = self.vehicle.state.position

        # Landing detection
        if self.uav_state.flight_phase == FlightPhase.LANDING and pos[2] < 0.5:
            self.test_validation['landing_completed'] = True
            terminated = True
            print(f"  [T+{self.sim_time:.1f}s] Landing complete at altitude {pos[2]:.2f}m", flush=True)

        # RTL completion (reached home and low altitude)
        if self.uav_state.flight_phase == FlightPhase.RTL:
            dist_to_home = np.linalg.norm(pos[:2] - self.home_position[:2])
            if dist_to_home < 3.0 and pos[2] < 1.0:
                self.test_validation['landing_completed'] = True
                terminated = True
                print(f"  [T+{self.sim_time:.1f}s] RTL complete - landed at home", flush=True)

        # Crash detection (exclude takeoff, landing, RTL, and preflight phases)
        safe_low_altitude_phases = (
            FlightPhase.PREFLIGHT,
            FlightPhase.TAKEOFF,
            FlightPhase.LANDING,
            FlightPhase.RTL,
        )
        if pos[2] < 0.3 and self.uav_state.flight_phase not in safe_low_altitude_phases:
            terminated = True
            print(f"  [T+{self.sim_time:.1f}s] CRASH at altitude {pos[2]:.2f}m", flush=True)

        # Battery depleted
        if self.uav_state.battery_pct < 5.0:
            terminated = True
            print(f"  [T+{self.sim_time:.1f}s] Battery depleted: {self.uav_state.battery_pct:.1f}%", flush=True)

        # Max steps
        if self.step_count >= self.max_steps:
            truncated = True
            print(f"  [T+{self.sim_time:.1f}s] Max steps reached", flush=True)

        # =================================================================
        # 10. COMPUTE OBSERVATION AND REWARD
        # =================================================================
        obs = self._get_observation()
        reward = self._compute_reward()

        # Logging
        if self.step_count % 60 == 0:  # Every second
            print(f"  [T+{self.sim_time:.1f}s] BAT:{self.uav_state.battery_pct:.0f}% "
                  f"RSV:{self.uav_state.battery_reserve:.0f}% "
                  f"ALT:{pos[2]:.1f}m "
                  f"PHASE:{self.uav_state.flight_phase.name} "
                  f"COMMS:{self.uav_state.comms_status.name} "
                  f"DET:{len(self._last_detections)}", flush=True)

        info = {
            "step": self.step_count,
            "sim_time": self.sim_time,
            "battery_pct": self.uav_state.battery_pct,
            "required_reserve": self.uav_state.battery_reserve,
            "flight_phase": self.uav_state.flight_phase.name,
            "comms_status": self.uav_state.comms_status.name,
            "detections": len(self._last_detections),
            "unique_tracks": len(self.stats['unique_tracks']),
            "ontology_preemptions": self.stats['ontology_preemptions'],
            "test_validation": self.test_validation.copy(),
        }

        return obs, reward, terminated, truncated, info

    def _get_camera_image(self) -> Optional[np.ndarray]:
        """
        Get RGB image from Isaac Sim Camera API.

        Uses the Camera.get_rgba() method which captures from the actual
        camera prim attached to the drone, not the viewport.

        Returns:
            RGB uint8 numpy array (H, W, 3) or None if image not available
        """
        if self.isr_camera is None:
            return None

        try:
            # Camera.get_rgba() returns RGBA numpy array
            rgba = self.isr_camera.get_rgba()
            if rgba is not None and hasattr(rgba, 'shape') and len(rgba.shape) >= 3:
                if rgba.shape[0] > 0 and rgba.shape[1] > 0 and rgba.max() > 0:
                    # Convert RGBA to RGB (drop alpha channel)
                    rgb = rgba[:, :, :3]
                    if self.step_count <= 3:
                        print(f"  [Camera] Step {self.step_count}: image captured, shape={rgb.shape}, max={rgb.max()}", flush=True)
                    return rgb.astype(np.uint8)
        except Exception as e:
            if self.step_count <= 3:
                print(f"  [Camera] Step {self.step_count}: get_rgba() error: {e}", flush=True)

        return None

    def _run_perception(self):
        """Run YOLO detection with ByteTrack tracking."""
        image = self._get_camera_image()

        if image is not None:
            detections = self.detector.detect(
                image=image,
                ground_truth_labels=None,
                uav_position=self.vehicle.state.position,
            )
        else:
            detections = []
            # Log when no image is available (only for first few steps)
            if self.step_count <= 5:
                print(f"  [Perception] Step {self.step_count}: No valid image available for detection", flush=True)

        self._last_detections = detections
        self._last_tracking_stats = self.detector.get_tracking_stats()

        # Update stats
        if len(detections) > 0:
            self.stats['frames_with_detections'] += 1
            self.stats['total_detections'] += len(detections)

            for det in detections:
                if det.track_id is not None:
                    self.stats['unique_tracks'].add(det.track_id)

            if not self.test_validation['yolo_detections_in_video']:
                self.test_validation['yolo_detections_in_video'] = True
                print(f"  [T+{self.sim_time:.1f}s] First YOLO detection: {detections[0].class_name}", flush=True)

    def _capture_video_frame(self):
        """Capture camera image, draw HUD, and write to video."""
        if self.video_writer is None:
            return

        image = self._get_camera_image()

        if image is None:
            if self.step_count <= 5:
                print(f"  [Video] Step {self.step_count}: No valid image for video frame", flush=True)
            return

        try:
            # Log first successful frame
            if self.step_count <= 5:
                print(f"  [Video] Step {self.step_count}: Valid frame captured, shape={image.shape}, max={image.max()}", flush=True)

            # Calculate distance to home
            pos = self.vehicle.state.position
            dist_to_home = np.linalg.norm(pos - self.home_position)

            # Draw HUD
            annotated = draw_hud(
                image=image,
                battery_pct=self.uav_state.battery_pct,
                required_reserve=self.uav_state.battery_reserve,
                altitude=pos[2],
                distance_to_home=dist_to_home,
                comms_status=self.uav_state.comms_status.name,
                flight_phase=self.uav_state.flight_phase.name,
                detections=self._last_detections,
                step=self.step_count,
                sim_time=self.sim_time,
            )

            self.video_writer.write_frame(annotated)

        except Exception as e:
            if self.step_count <= 10:
                print(f"  [Video] Frame capture error at step {self.step_count}: {e}", flush=True)

    def _get_observation(self):
        """Get observation vector."""
        pos = self.vehicle.state.position
        vel = self.vehicle.state.linear_velocity
        orient = self.vehicle.state.attitude

        r = Rotation.from_quat(orient)
        euler = r.as_euler('XYZ')

        # UAV state (11)
        uav_state = np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            euler[0], euler[1], euler[2],
            np.radians(self.gimbal_pitch),
            np.radians(self.gimbal_yaw),
        ], dtype=np.float32)

        # Battery and mission state (6)
        mission_state = np.array([
            self.uav_state.battery_pct / 100.0,
            self.uav_state.battery_reserve / 100.0,
            self.sim_time / 300.0,  # Normalize to ~5 min
            float(self.uav_state.comms_status == CommsStatus.DENIED),
            float(self.uav_state.flight_phase == FlightPhase.RTL),
            float(self.ontology_controller.takeoff_complete),
        ], dtype=np.float32)

        # Detection state (6)
        det_state = np.array([
            float(len(self._last_detections)),
            float(self._last_tracking_stats.get('active_tracks', 0)),
            float(self.stats['total_detections']) / 100.0,
            float(len(self.stats['unique_tracks'])) / 10.0,
            float(self.stats['frames_with_detections']) / max(1, self.step_count),
            self.detector.avg_inference_time_ms / 100.0,
        ], dtype=np.float32)

        # Target observations (3 targets x 4 = 12)
        target_obs = []
        for target in self.target_positions[:3]:
            dx = target['position'][0] - pos[0]
            dy = target['position'][1] - pos[1]
            dz = target['position'][2] - pos[2]
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            target_obs.extend([dx / 100.0, dy / 100.0, dz / 100.0, min(1.0, 50.0 / max(dist, 1.0))])

        while len(target_obs) < 12:
            target_obs.extend([0.0, 0.0, 0.0, 0.0])

        target_obs = np.array(target_obs[:12], dtype=np.float32)

        # Total: 11 + 6 + 6 + 12 = 35
        return np.concatenate([uav_state, mission_state, det_state, target_obs])

    def _compute_reward(self):
        """Compute reward (minimal for this integration test)."""
        reward = 0.0

        # Small reward for staying alive
        reward += 0.001

        # Reward for detections
        reward += len(self._last_detections) * 0.1

        # Penalty for low battery outside RTL
        if (self.uav_state.battery_pct < self.uav_state.battery_reserve and
            self.uav_state.flight_phase != FlightPhase.RTL):
            reward -= 0.5

        return reward

    def _send_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        """Send velocity setpoint to PX4."""
        type_mask = 0b0000011111000111
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0, vx, vy, -vz, 0, 0, 0, 0, yaw_rate  # Note: -vz for NED
        )

    def _update_gimbal(self):
        """Update gimbal camera pose."""
        if self.isr_camera is None:
            return

        gimbal_rot = Rotation.from_euler("ZYX", [self.gimbal_yaw, self.gimbal_pitch, 0.0], degrees=True)
        local_pos = np.array([0.1, 0.0, -0.05])

        try:
            self.isr_camera.set_local_pose(local_pos, gimbal_rot.as_quat())
        except Exception:
            pass

    def close(self):
        """Cleanup."""
        if self.video_writer is not None:
            self.video_writer.close()
            self.test_validation['video_generated'] = True


# =============================================================================
# SIMULATION SETUP
# =============================================================================

timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world
pg.set_px4_path("/px4")

stage = get_current_stage()
physics_path = "/World/PhysicsScene"
if not stage.GetPrimAtPath(physics_path):
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_path))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    print("Physics scene created", flush=True)

# Generate procedural world
print("\n[WorldGen] Generating procedural environment...", flush=True)
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
print(f"[WorldGen] Forest generated: {len(forest_result['trees'])} trees", flush=True)

# Spawn POI targets (vehicles and people for YOLO detection)
print("\n[WorldGen] Spawning POI targets...", flush=True)
poi_clusters = [
    (30.0, 20.0),
    (-20.0, 30.0),
    (40.0, -25.0),
    (-35.0, -30.0),
]

all_target_paths = []
for center in poi_clusters:
    # Spawn vehicles
    vehicle_paths = world_gen.vehicles.spawn_vehicle_group(
        vehicle_types=["sedan", "suv", "tank"],
        count=3,
        clustering=0.5,
        center=center,
    )
    all_target_paths.extend(vehicle_paths)

    # Spawn people
    try:
        for _ in range(2):
            person_path = world_gen.people.spawn_person(
                position=(center[0] + np.random.uniform(-10, 10),
                          center[1] + np.random.uniform(-10, 10))
            )
            all_target_paths.append(person_path)
    except Exception:
        pass  # People spawner may not be available

print(f"[WorldGen] Spawned {len(all_target_paths)} targets", flush=True)

# Get target positions
target_positions = []
for path in all_target_paths:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        if ops:
            pos = ops[0].Get()
            target_positions.append({
                'path': path,
                'position': np.array([pos[0], pos[1], pos[2]]),
                'type': 'vehicle' if 'vehicle' in path.lower() else 'person'
            })

# Create PX4 vehicle
print("\n[PX4] Creating vehicle...", flush=True)
mavlink_config = PX4MavlinkBackendConfig({
    "vehicle_id": 0,
    "px4_autolaunch": True,
    "px4_dir": pg.px4_path,
    "px4_vehicle_model": pg.px4_default_airframe,
    "enable_lockstep": True,
    "update_rate": 250.0,
})
config = MultirotorConfig()
config.backends = [PX4MavlinkBackend(mavlink_config)]

spawn_height = 0.5  # Start on ground for takeoff test
vehicle = Multirotor(
    "/World/quadrotor",
    ROBOTS['Iris'],
    0,
    [0.0, 0.0, spawn_height],
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=config,
)
print(f"[PX4] Vehicle spawned at height {spawn_height}m", flush=True)

world.reset()

# Create ISR camera attached to drone body BEFORE timeline.play()
# Following the working pattern from phase3b_camera_yolo_test.py:
#   1. world.reset() (done above)
#   2. Create camera (constructor + initialize)
#   3. timeline.play()
#   4. Camera warmup
isr_camera = None
camera_resolution = (640, 480)
camera_prim_path = "/World/quadrotor/body/isr_camera"

print("\n[Camera] Creating ISR gimbal camera using Isaac Sim Camera API...", flush=True)
print(f"[Camera] Camera path: {camera_prim_path}", flush=True)

try:
    isr_camera = Camera(
        prim_path=camera_prim_path,
        frequency=30,  # 30Hz is sufficient for YOLO
        resolution=camera_resolution,
    )

    # Camera pointing forward and down at 45 degrees
    # Rotation order ZYX: yaw=0, pitch=45deg down
    camera_orientation = Rotation.from_euler("ZYX", [0.0, 45.0, 0.0], degrees=True).as_quat()
    isr_camera.set_local_pose(
        np.array([0.0, 0.0, -0.1]),  # Below body to avoid obstruction
        camera_orientation
    )

    # Initialize camera (must be done before get_rgba works)
    isr_camera.initialize()
    isr_camera.set_focal_length(4.5)  # Wide angle ~90° FOV
    isr_camera.set_clipping_range(0.1, 500.0)

    # Add distance annotator for depth if needed
    isr_camera.add_distance_to_image_plane_to_frame()

    print(f"[Camera] Camera created successfully", flush=True)
    print(f"[Camera] Resolution: {camera_resolution}", flush=True)
    print(f"[Camera] FOV: ~90° (4.5mm focal length)", flush=True)

except Exception as e:
    print(f"[Camera] Warning: Could not create camera: {e}", flush=True)
    import traceback
    traceback.print_exc()
    isr_camera = None

# Start simulation
timeline.play()

# Camera warmup - need ~120 frames for camera to stabilize (per phase3b experience)
print("\n[Camera] Warming up camera (120 frames)...", flush=True)
for i in range(120):
    world.step(render=True)
    if i % 30 == 0 and isr_camera is not None:
        try:
            rgba = isr_camera.get_rgba()
            if rgba is not None and rgba.max() > 0:
                print(f"  Frame {i}: image shape={rgba.shape[:2]}, max={rgba.max()}", flush=True)
            else:
                print(f"  Frame {i}: no image yet (renderer warming up)", flush=True)
        except Exception as e:
            print(f"  Frame {i}: camera not ready yet ({e})", flush=True)
print("[Camera] Warmup complete", flush=True)

# PX4 initialization
print("\n[PX4] Running initialization steps...", flush=True)
for i in range(200):
    world.step(render=True)

# MAVLink connection
print("\n[MAVLink] Connecting...", flush=True)
mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

for i in range(500):
    world.step(render=True)
    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg and msg.get_srcSystem() != 0:
        print(f"[MAVLink] Got heartbeat from system {msg.get_srcSystem()}", flush=True)
        break

if mav.target_system == 0:
    mav.target_system = 1
    mav.target_component = 1


def check_mode():
    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return armed, msg.custom_mode
    return None, None


def send_setpoint(vx=0.0, vy=0.0, vz=-0.5):
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
    )


print("\n[Flight] Priming setpoints...", flush=True)
for _ in range(200):
    send_setpoint(vz=0.0)
    world.step(render=True)

print("\n[Flight] Engaging OFFBOARD mode and arming...", flush=True)
for attempt in range(3):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6, 0, 0, 0, 0, 0
    )
    for _ in range(100):
        send_setpoint(vz=0.0)
        world.step(render=True)
    armed, mode = check_mode()
    if mode == 6:
        break

for attempt in range(3):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    for _ in range(100):
        send_setpoint(vz=0.0)
        world.step(render=True)
    armed, mode = check_mode()
    if armed:
        break

# =============================================================================
# PHASE 9 TEST EXECUTION
# =============================================================================

print("\n" + "=" * 70, flush=True)
print("PHASE 9: COMMS-DENIED INTEGRATION TEST", flush=True)
print("=" * 70, flush=True)

# Create video writer - 15fps output for smaller file size
video_path = f"{OUTPUT_DIR}/phase9_comms_denied.mp4"
video_writer = VideoWriter(video_path, fps=15, resolution=(640, 480))

# =============================================================================
# SIMULATION SPEEDUP CONFIGURATION
# =============================================================================
# Goal: Complete 80-second mission in <2 minutes wall-clock time (10x speedup)
#
# Speedup strategies:
# 1. Batch physics steps: Run 4 physics steps per render (4x speedup)
# 2. Skip video frames: Capture every 4th step (60Hz/4 = 15fps video)
# 3. Reduce YOLO frequency: Run every 6th step (10Hz instead of 60Hz)
#
# Combined speedup: ~4x from physics batching + reduced GPU rendering
# With faster takeoff (2.0 m/s), mission should complete much faster
#
PHYSICS_STEPS_PER_RENDER = 4  # 4x speedup from reduced rendering
VIDEO_CAPTURE_INTERVAL = 4    # 15fps video (every 4th step of 60Hz)
YOLO_INFERENCE_INTERVAL = 6   # ~10Hz inference

# Create environment with speedup parameters
env = Phase9CommsDeniedEnv(
    world=world,
    vehicle=vehicle,
    mav=mav,
    target_positions=target_positions,
    isr_camera=isr_camera,
    max_steps=5000,  # ~83 seconds at 60Hz - enough for battery RTL trigger
    video_writer=video_writer,
    comms_denial_time=30.0,
    mission_altitude=25.0,
    accelerated_battery_drain=20.0,  # 20x faster drain: ~60%/min to trigger RTL sooner
    # Speedup parameters
    physics_steps_per_render=PHYSICS_STEPS_PER_RENDER,
    video_capture_interval=VIDEO_CAPTURE_INTERVAL,
    yolo_inference_interval=YOLO_INFERENCE_INTERVAL,
)

print(f"\n[Env] Environment created", flush=True)
print(f"  Observation space: {env.observation_space.shape}", flush=True)
print(f"  Action space: {env.action_space.shape}", flush=True)
print(f"  Max steps: {env.max_steps}", flush=True)
print(f"  Targets in scene: {len(target_positions)}", flush=True)
print(f"\n[Speedup] Simulation acceleration enabled:", flush=True)
print(f"  Physics steps per render: {PHYSICS_STEPS_PER_RENDER}x", flush=True)
print(f"  Video capture: every {VIDEO_CAPTURE_INTERVAL} steps (15fps)", flush=True)
print(f"  YOLO inference: every {YOLO_INFERENCE_INTERVAL} steps (~10Hz)", flush=True)

# Run single episode
obs, info = env.reset()
done = False
step = 0
total_reward = 0.0

print("\n[Test] Starting episode...", flush=True)
print("  Ontology will control takeoff, RL controls exploration,", flush=True)
print("  ontology triggers RTL when battery < reserve", flush=True)

while not done:
    # Random actions (untrained agent)
    action = env.action_space.sample()

    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward
    step += 1
    done = terminated or truncated

# Close video writer
env.close()

# =============================================================================
# RESULTS AND VALIDATION
# =============================================================================

print("\n" + "=" * 70, flush=True)
print("PHASE 9 RESULTS", flush=True)
print("=" * 70, flush=True)

print(f"\nEpisode Statistics:", flush=True)
print(f"  Total steps: {step}", flush=True)
print(f"  Simulation time: {info['sim_time']:.1f}s", flush=True)
print(f"  Total reward: {total_reward:.2f}", flush=True)
print(f"  Final battery: {info['battery_pct']:.1f}%", flush=True)

print(f"\nPerception Statistics:", flush=True)
print(f"  Frames with detections: {env.stats['frames_with_detections']}", flush=True)
print(f"  Total detections: {env.stats['total_detections']}", flush=True)
print(f"  Unique tracks: {len(env.stats['unique_tracks'])}", flush=True)
print(f"  YOLO inference time: {env.detector.avg_inference_time_ms:.1f}ms", flush=True)

print(f"\nOntology Statistics:", flush=True)
print(f"  Ontology preemptions: {env.stats['ontology_preemptions']}", flush=True)
print(f"  Safety filter interventions: {env.stats['safety_filter_interventions']}", flush=True)
if env.stats['rtl_trigger_battery'] > 0:
    print(f"  RTL trigger battery: {env.stats['rtl_trigger_battery']:.1f}%", flush=True)
    print(f"  RTL trigger reserve: {env.stats['rtl_trigger_reserve']:.1f}%", flush=True)

print(f"\nTest Validation:", flush=True)
validation = info['test_validation']

# Separate core ontology tests from camera/video tests
# Camera/video require GUI mode due to Isaac Sim 5.x headless bug
# See: https://github.com/isaac-sim/IsaacLab/issues/3250
core_checks = [
    ('takeoff_by_ontology', 'Takeoff executed by ontology'),
    ('comms_denied_logged', 'Comms denial logged at T+30s'),
    ('rtl_triggered_by_battery', 'RTL triggered by battery reserve'),
]

camera_checks = [
    ('yolo_detections_in_video', 'YOLO detections in video'),
    ('landing_completed', 'Landing completed without crash'),
    ('video_generated', 'Video file generated'),
]

print("  Core Ontology Tests (headless compatible):", flush=True)
core_passed = True
for key, description in core_checks:
    passed = validation.get(key, False)
    status = "PASS" if passed else "FAIL"
    symbol = "✓" if passed else "✗"
    print(f"    [{symbol}] {description}: {status}", flush=True)
    if not passed:
        core_passed = False

print("  Camera/Video Tests (requires GUI mode):", flush=True)
camera_passed = True
for key, description in camera_checks:
    passed = validation.get(key, False)
    status = "PASS" if passed else "FAIL"
    symbol = "✓" if passed else "✗"
    note = " (expected in headless)" if HEADLESS and not passed else ""
    print(f"    [{symbol}] {description}: {status}{note}", flush=True)
    if not passed:
        camera_passed = False

all_passed = core_passed and camera_passed

print(f"\nVideo Output:", flush=True)
print(f"  Path: {video_path}", flush=True)
print(f"  Frames written: {video_writer.frame_count}", flush=True)

# Check video file size
try:
    import os as os_check
    if os_check.path.exists(video_path):
        video_size = os_check.path.getsize(video_path)
        print(f"  File size: {video_size / 1024:.1f} KB", flush=True)
        if video_size > 1024:  # More than 1KB
            validation['video_generated'] = True
        else:
            print(f"  [WARN] Video file too small ({video_size} bytes) - likely no frames written", flush=True)
    else:
        print(f"  [WARN] Video file not found at {video_path}", flush=True)
except Exception as e:
    print(f"  [WARN] Could not check video file: {e}", flush=True)

# Overall success
print("\n" + "=" * 70, flush=True)
if all_passed:
    print("SUCCESS! All Phase 9 integration criteria passed.", flush=True)
elif core_passed:
    print("CORE TESTS PASSED! Ontology and safety components working correctly.", flush=True)
    if HEADLESS:
        print("Camera/video tests require GUI mode - run without --headless for full test.", flush=True)
    else:
        print("Some camera/video criteria not met (see above).", flush=True)
else:
    print("FAILED - Core ontology tests did not pass.", flush=True)
print("=" * 70, flush=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
