#!/usr/bin/env python3
"""
Drone Functions Check - Modular PX4 Flight Test with YOLO Video Recording

This script verifies drone capabilities in Isaac Sim using modular flight profiles.
Each profile targets specific test scenarios for faster iteration.

Available Profiles:
    --profile full      Complete ~12 min test (default)
    --profile detection 360° pan for YOLO/GT testing (~2-3 min)
    --profile tracking  Linear flight for ByteTrack (~2-3 min)
    --profile gimbal    Hover + pan/tilt sequences (~1-2 min)
    --profile nav       Waypoint navigation patterns (~3-4 min)
    --profile stress    Aggressive maneuvers (~2 min)

Usage:
    /isaac-sim/python.sh /workspace/scripts/drone_functions_check.py --headless
    /isaac-sim/python.sh /workspace/scripts/drone_functions_check.py --headless --profile detection

Output:
    /workspace/output/functions_check[_profile].mp4 - Annotated video
    /workspace/output/functions_check[_profile]_report.json - Test results

Author: Finley Holt
"""

import sys
import os
import argparse
import time
import json

# Parse arguments early (before heavy imports)
parser = argparse.ArgumentParser(description="Drone Functions Check with Flight Profiles")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument("--profile", type=str, default="full",
                    help="Flight profile: full, detection, tracking, gimbal, nav, stress")
parser.add_argument("--list-profiles", action="store_true", help="List available profiles")
args = parser.parse_args()

# Early print to verify script is running
print("=" * 70, flush=True)
print("DRONE FUNCTIONS CHECK - Modular Flight Profiles", flush=True)
print("=" * 70, flush=True)

# Handle --list-profiles before heavy imports
if args.list_profiles:
    print("\nAvailable flight profiles:\n")
    print("  full       Complete ~12 min test (world gen, takeoff, nav, gimbal, stress, RTH, land)")
    print("  detection  360° pan for YOLO/GT overlay testing (~2-3 min)")
    print("  tracking   Linear flight over vehicle clusters for ByteTrack (~2-3 min)")
    print("  gimbal     Hover + pan/tilt sequences (~1-2 min)")
    print("  nav        Waypoint navigation patterns (~3-4 min)")
    print("  stress     Aggressive maneuvers for attitude control (~2 min)")
    print()
    sys.exit(0)

HEADLESS = args.headless
PROFILE_NAME = args.profile

print(f"[Config] Headless mode: {HEADLESS}", flush=True)
print(f"[Config] Profile: {PROFILE_NAME}", flush=True)

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
from dataclasses import asdict
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
from pxr import UsdGeom, Gf, UsdPhysics, Sdf
print("[Init] Core imports completed", flush=True)

# Add paths
sys.path.insert(0, "/workspace")
sys.path.insert(0, "/workspace/scripts")

PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

from isaacsim.core.utils.stage import get_current_stage
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera

from omni.isaac.core.world import World
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig

# Import flight profiles - use importlib to avoid namespace conflicts with Isaac Sim
import importlib.util
_fp_init_path = "/workspace/scripts/flight_profiles/__init__.py"
_fp_spec = importlib.util.spec_from_file_location("flight_profiles", _fp_init_path)
_flight_profiles = importlib.util.module_from_spec(_fp_spec)
sys.modules["flight_profiles"] = _flight_profiles
_fp_spec.loader.exec_module(_flight_profiles)
get_profile = _flight_profiles.get_profile
list_profiles = _flight_profiles.list_profiles

_fp_base_path = "/workspace/scripts/flight_profiles/base.py"
_fpb_spec = importlib.util.spec_from_file_location("flight_profiles.base", _fp_base_path)
_flight_profiles_base = importlib.util.module_from_spec(_fpb_spec)
sys.modules["flight_profiles.base"] = _flight_profiles_base
_fpb_spec.loader.exec_module(_flight_profiles_base)
SimulationContext = _flight_profiles_base.SimulationContext
TestResult = _flight_profiles_base.TestResult

# Import perception components
ENVIRONMENTS_PATH = "/workspace/environments"
if ENVIRONMENTS_PATH not in sys.path:
    sys.path.insert(0, ENVIRONMENTS_PATH)

try:
    from byte_tracker import ByteTracker, ByteTrackerConfig
    from debug_overlay import DebugOverlay, OverlayConfig
    from perception_manager import Detection, PerceptionManager, PerceptionConfig, GroundTruthConfig
    BYTETRACK_AVAILABLE = True
    print("[Init] ByteTrack temporal coherence tracking loaded", flush=True)
except ImportError as e:
    BYTETRACK_AVAILABLE = False
    print(f"[Init] ByteTrack not available: {e}", flush=True)

print("[Init] All imports completed successfully", flush=True)

# ============================================================================
# Configuration
# ============================================================================

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_PATH = "/World/isr_camera"

OUTPUT_DIR = "/workspace/output"
YOLO_MODEL_PATH = "/workspace/models/yolo11x.pt"

# Flight parameters (defaults, profiles may override)
TAKEOFF_ALTITUDE = 75.0
WAYPOINT_SIZE = 30.0
WAYPOINT_ALTITUDE = 75.0
CRUISE_SPEED = 3.0
POSITION_TOLERANCE = 2.0

# Gimbal control parameters
GIMBAL_PAN_RATE = 15.0
GIMBAL_TILT_RANGE = (-90.0, 0.0)

ENABLE_TEMPORAL_COHERENCE = True


# ============================================================================
# Import existing components from original script
# ============================================================================

# VideoRecorder, GimbalController, FlightController classes
# These are kept inline for now to avoid breaking changes

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Any
import subprocess


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


class VideoRecorder:
    """Records video with YOLO detection overlay and ByteTrack temporal coherence."""

    def __init__(
        self,
        output_path: str,
        width: int,
        height: int,
        fps: int = 30,
        yolo_model_path: str = None,
        enable_tracking: bool = True,
    ):
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_count = 0
        self.ffmpeg_proc = None
        self.yolo_model = None
        self.tracker = None
        self.debug_overlay = None

        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        if yolo_model_path and os.path.exists(yolo_model_path):
            try:
                from ultralytics import YOLO
                self.yolo_model = YOLO(yolo_model_path)
                print(f"[VideoRecorder] Loaded YOLO model from {yolo_model_path}")
            except ImportError:
                print("[VideoRecorder] WARNING: ultralytics not installed")
            except Exception as e:
                print(f"[VideoRecorder] WARNING: Failed to load YOLO: {e}")

        if enable_tracking and BYTETRACK_AVAILABLE:
            try:
                tracker_config = ByteTrackerConfig(
                    high_thresh=0.3, low_thresh=0.1, match_thresh=0.8,
                    track_buffer=30, min_hits=1,
                )
                self.tracker = ByteTracker(tracker_config)
                overlay_config = OverlayConfig(
                    show_track_ids=True, show_velocity_arrows=True,
                    show_track_trails=True, trail_length=15,
                    use_track_colors=True, save_frames=False,
                )
                self.debug_overlay = DebugOverlay(overlay_config)
                print("[VideoRecorder] ByteTrack enabled")
            except Exception as e:
                print(f"[VideoRecorder] WARNING: ByteTrack init failed: {e}")

    def start(self) -> bool:
        import shutil
        if shutil.which('ffmpeg') is None:
            print("[VideoRecorder] WARNING: FFmpeg not found")
            return False

        ffmpeg_cmd = [
            'ffmpeg', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', f'{self.width}x{self.height}', '-r', str(self.fps),
            '-i', '-', '-c:v', 'libx264', '-preset', 'fast',
            '-crf', '23', '-pix_fmt', 'yuv420p', self.output_path
        ]

        try:
            self.ffmpeg_proc = subprocess.Popen(
                ffmpeg_cmd, stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            print(f"[VideoRecorder] Started FFmpeg, output: {self.output_path}")
            return True
        except Exception as e:
            print(f"[VideoRecorder] ERROR: {e}")
            return False

    def write_frame(self, rgba_frame, overlay_text=None, right_overlay_text=None, ground_truth=None) -> int:
        if self.ffmpeg_proc is None or rgba_frame is None or rgba_frame.size == 0:
            return 0

        import cv2
        if rgba_frame.shape[0] != self.height or rgba_frame.shape[1] != self.width:
            rgba_frame = cv2.resize(rgba_frame, (self.width, self.height))

        rgb_frame = rgba_frame[:, :, :3].copy()
        detection_count = 0
        tracks = []

        if self.yolo_model is not None:
            try:
                results = self.yolo_model(rgb_frame, verbose=False)
                if results and len(results) > 0:
                    detection_count = len(results[0].boxes) if results[0].boxes is not None else 0

                    if self.tracker is not None and results[0].boxes is not None:
                        detections = []
                        boxes = results[0].boxes
                        for i in range(len(boxes)):
                            box = boxes[i]
                            xyxy = box.xyxy[0].cpu().numpy()
                            conf = float(box.conf[0].cpu().numpy())
                            cls_id = int(box.cls[0].cpu().numpy())
                            cls_name = self.yolo_model.names[cls_id]
                            det = Detection(class_id=cls_id, class_name=cls_name,
                                          confidence=conf, bbox=tuple(xyxy))
                            detections.append(det)
                        tracks = self.tracker.update(detections)
                        rgb_frame = self._draw_ground_truth_overlay(rgb_frame, ground_truth, tracks)
                    else:
                        annotated = results[0].plot()
                        if annotated.shape == rgb_frame.shape:
                            rgb_frame = annotated
            except Exception as e:
                if self.frame_count % 100 == 0:
                    print(f"[VideoRecorder] Error: {e}")

        # Draw overlays
        if overlay_text:
            self._draw_text_overlay(rgb_frame, overlay_text, "left")
        if right_overlay_text:
            self._draw_text_overlay(rgb_frame, right_overlay_text, "right")

        rgb_frame = np.ascontiguousarray(rgb_frame)
        try:
            self.ffmpeg_proc.stdin.write(rgb_frame.tobytes())
            self.frame_count += 1
        except Exception as e:
            print(f"[VideoRecorder] Write error: {e}")

        return detection_count

    def _draw_text_overlay(self, frame, text, position):
        import cv2
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5 if position == "left" else 0.45
        color = (255, 255, 255) if position == "left" else (0, 255, 255)

        lines = text.split('\n')
        y_offset = 20

        for line in lines:
            (tw, th), _ = cv2.getTextSize(line, font, font_scale, 1)
            x_pos = 8 if position == "left" else self.width - tw - 12
            cv2.rectangle(frame, (x_pos - 3, y_offset - th - 3),
                         (x_pos + tw + 3, y_offset + 3), (0, 0, 0), -1)
            cv2.putText(frame, line, (x_pos, y_offset), font, font_scale, color, 1)
            y_offset += th + 8

    def _draw_ground_truth_overlay(self, rgb_frame, ground_truth, tracks):
        """
        Draw GT overlay with color-coded boxes:
        - RED: GT object not detected (missed detection)
        - YELLOW: GT object detected but wrong class
        - GREEN: GT object correctly detected and classified
        - CYAN: YOLO detection with no matching GT (only shown if confident detection)

        Uses IoU threshold of 0.15 for matching (accounts for bbox size differences).
        """
        import cv2

        # Color definitions (BGR format for OpenCV)
        COLOR_RED = (0, 0, 255)       # GT missed
        COLOR_YELLOW = (0, 255, 255)  # GT detected, wrong class
        COLOR_GREEN = (0, 255, 0)     # GT correctly detected
        COLOR_CYAN = (255, 255, 0)    # Unmatched track (potential FP or missing GT)

        # IoU threshold - lower because GT projection may not be perfect
        IOU_MATCH_THRESHOLD = 0.15

        h, w = rgb_frame.shape[:2]

        # Debug logging - log every 100 frames
        if not hasattr(self, '_overlay_debug_counter'):
            self._overlay_debug_counter = 0
        self._overlay_debug_counter += 1
        do_debug_log = (self._overlay_debug_counter % 100 == 1)

        if do_debug_log:
            print(f"\n[OVERLAY DEBUG] Frame {self._overlay_debug_counter}, Image size: {w}x{h}")
            print(f"  GT detections: {len(ground_truth) if ground_truth else 0}")
            print(f"  YOLO tracks: {len(tracks) if tracks else 0}")
            if ground_truth:
                for i, gt in enumerate(ground_truth[:8]):  # First 8 GT
                    gt_cx, gt_cy, gt_w, gt_h = gt.bbox
                    px_x1 = int((gt_cx - gt_w/2) * w)
                    px_y1 = int((gt_cy - gt_h/2) * h)
                    px_x2 = int((gt_cx + gt_w/2) * w)
                    px_y2 = int((gt_cy + gt_h/2) * h)
                    world_pos = gt.world_position if hasattr(gt, 'world_position') else 'N/A'
                    dist = gt.distance if hasattr(gt, 'distance') else 'N/A'
                    print(f"  GT[{i}] {gt.class_name}: bbox_norm=({gt_cx:.3f},{gt_cy:.3f},{gt_w:.3f},{gt_h:.3f}) px=({px_x1},{px_y1})-({px_x2},{px_y2}) world={world_pos} dist={dist}")
            if tracks:
                for i, t in enumerate(tracks[:8]):  # First 8 tracks
                    tx1, ty1, tx2, ty2 = map(int, t.tlbr)
                    # Compute normalized coords for comparison
                    t_cx_norm = (tx1 + tx2) / 2 / w
                    t_cy_norm = (ty1 + ty2) / 2 / h
                    t_w_norm = (tx2 - tx1) / w
                    t_h_norm = (ty2 - ty1) / h
                    score = t.score if hasattr(t, 'score') else 'N/A'
                    print(f"  TRACK[{i}] #{t.track_id} {t.class_name}: bbox_norm=({t_cx_norm:.3f},{t_cy_norm:.3f},{t_w_norm:.3f},{t_h_norm:.3f}) px=({tx1},{ty1})-({tx2},{ty2}) score={score}")

        # Handle case with no GT - just draw tracks normally without FP labels
        if ground_truth is None or len(ground_truth) == 0:
            if tracks:
                for track in tracks:
                    bbox = track.tlbr
                    x1, y1, x2, y2 = map(int, bbox)
                    cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), COLOR_CYAN, 2)
                    label = f"#{track.track_id} {track.class_name}"
                    cv2.putText(rgb_frame, label, (x1, y1 - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_CYAN, 1)
            return rgb_frame

        # Track matching state
        gt_matched = [False] * len(ground_truth)
        gt_matched_track = [None] * len(ground_truth)
        gt_class_match = [False] * len(ground_truth)
        track_matched = [False] * len(tracks)

        # Convert GT bboxes to pixel coordinates
        gt_pixel_boxes = []
        for gt in ground_truth:
            gt_cx, gt_cy, gt_w, gt_h = gt.bbox
            gt_x1 = int((gt_cx - gt_w / 2) * w)
            gt_y1 = int((gt_cy - gt_h / 2) * h)
            gt_x2 = int((gt_cx + gt_w / 2) * w)
            gt_y2 = int((gt_cy + gt_h / 2) * h)
            gt_pixel_boxes.append((gt_x1, gt_y1, gt_x2, gt_y2))

        # Match GT to tracks using IoU
        for gt_idx, gt in enumerate(ground_truth):
            gt_x1, gt_y1, gt_x2, gt_y2 = gt_pixel_boxes[gt_idx]
            gt_area = (gt_x2 - gt_x1) * (gt_y2 - gt_y1)

            # Skip degenerate GT boxes
            if gt_area <= 0:
                continue

            best_iou = 0.0
            best_idx = -1

            for t_idx, track in enumerate(tracks):
                if track_matched[t_idx]:
                    continue

                t_bbox = track.tlbr
                t_x1, t_y1, t_x2, t_y2 = map(int, t_bbox)
                t_area = (t_x2 - t_x1) * (t_y2 - t_y1)

                # Skip degenerate track boxes
                if t_area <= 0:
                    continue

                # Calculate IoU
                inter_x1, inter_y1 = max(gt_x1, t_x1), max(gt_y1, t_y1)
                inter_x2, inter_y2 = min(gt_x2, t_x2), min(gt_y2, t_y2)
                inter_w = max(0, inter_x2 - inter_x1)
                inter_h = max(0, inter_y2 - inter_y1)
                inter_area = inter_w * inter_h
                union = gt_area + t_area - inter_area

                if union > 0:
                    iou = inter_area / union
                    if iou > best_iou and iou >= IOU_MATCH_THRESHOLD:
                        best_iou = iou
                        best_idx = t_idx

            if best_idx >= 0:
                gt_matched[gt_idx] = True
                gt_matched_track[gt_idx] = tracks[best_idx]
                track_matched[best_idx] = True

                # Check class match - be generous since YOLO uses COCO names
                gt_class = gt.class_name.lower()
                track_class = tracks[best_idx].class_name.lower()

                # YOLO COCO vehicle classes (exact matches)
                yolo_vehicle_set = {'car', 'truck', 'bus', 'motorcycle', 'bicycle'}
                # GT vehicle classes (exact matches for full class names)
                gt_vehicle_set = {'civilian_vehicle', 'military_vehicle', 'tank', 'apc', 'armored_vehicle', 'vehicle'}

                # YOLO COCO person class (exact match)
                yolo_person_set = {'person'}
                # GT person classes (exact matches)
                gt_person_set = {'person', 'military_person', 'soldier', 'civilian_person'}

                # Check if track is a vehicle (exact match to YOLO class)
                track_is_vehicle = track_class in yolo_vehicle_set
                # Check if GT is a vehicle (exact match to GT class)
                gt_is_vehicle = gt_class in gt_vehicle_set

                # Check if track is a person (exact match)
                track_is_person = track_class in yolo_person_set
                # Check if GT is a person (exact match)
                gt_is_person = gt_class in gt_person_set

                # Match if categories align
                if (track_is_vehicle and gt_is_vehicle) or (track_is_person and gt_is_person):
                    gt_class_match[gt_idx] = True

        # Draw GT boxes
        for gt_idx, gt in enumerate(ground_truth):
            gt_x1, gt_y1, gt_x2, gt_y2 = gt_pixel_boxes[gt_idx]

            # Clamp to frame bounds
            gt_x1 = max(0, min(w-1, gt_x1))
            gt_y1 = max(0, min(h-1, gt_y1))
            gt_x2 = max(0, min(w-1, gt_x2))
            gt_y2 = max(0, min(h-1, gt_y2))

            # Skip tiny or invalid boxes
            if gt_x2 - gt_x1 < 5 or gt_y2 - gt_y1 < 5:
                continue

            # Determine color based on match status
            if not gt_matched[gt_idx]:
                color = COLOR_RED  # Missed detection
                status = "MISS"
            elif not gt_class_match[gt_idx]:
                color = COLOR_YELLOW  # Wrong class
                status = "WRONG"
            else:
                color = COLOR_GREEN  # Correct detection
                status = "OK"

            cv2.rectangle(rgb_frame, (gt_x1, gt_y1), (gt_x2, gt_y2), color, 2)

            # Label below box
            label = f"GT:{gt.class_name}"
            cv2.putText(rgb_frame, label, (gt_x1, gt_y2 + 12),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

            # If matched, show track info above box
            if gt_matched[gt_idx] and gt_matched_track[gt_idx]:
                track = gt_matched_track[gt_idx]
                cv2.putText(rgb_frame, f"#{track.track_id} {track.class_name} [{status}]",
                           (gt_x1, gt_y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        # Draw unmatched tracks (cyan) - only high confidence ones
        for t_idx, track in enumerate(tracks):
            if not track_matched[t_idx]:
                # Only show unmatched tracks with reasonable confidence
                if hasattr(track, 'score') and track.score < 0.25:
                    continue

                t_bbox = track.tlbr
                t_x1, t_y1, t_x2, t_y2 = map(int, t_bbox)

                # Skip tracks at very edge of frame (likely partial detections)
                margin = 10
                if t_x1 < margin or t_y1 < margin or t_x2 > w - margin or t_y2 > h - margin:
                    # Check if mostly outside frame
                    visible_w = min(t_x2, w - margin) - max(t_x1, margin)
                    visible_h = min(t_y2, h - margin) - max(t_y1, margin)
                    total_w = t_x2 - t_x1
                    total_h = t_y2 - t_y1
                    if total_w > 0 and total_h > 0:
                        visible_ratio = (visible_w * visible_h) / (total_w * total_h)
                        if visible_ratio < 0.5:
                            continue

                cv2.rectangle(rgb_frame, (t_x1, t_y1), (t_x2, t_y2), COLOR_CYAN, 1)
                cv2.putText(rgb_frame, f"#{track.track_id} {track.class_name}",
                           (t_x1, t_y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, COLOR_CYAN, 1)

        return rgb_frame

    def stop(self) -> bool:
        if self.ffmpeg_proc is None:
            return False
        try:
            self.ffmpeg_proc.stdin.close()
            self.ffmpeg_proc.wait(timeout=30)
            print(f"[VideoRecorder] Finished: {self.frame_count} frames")
            return True
        except Exception as e:
            print(f"[VideoRecorder] Stop error: {e}")
            self.ffmpeg_proc.kill()
            return False

    def get_tracking_stats(self) -> dict:
        stats = {"tracking_enabled": self.tracker is not None,
                "total_tracks_created": 0, "active_tracks": 0, "unique_tracks": 0}
        if self.tracker:
            ts = self.tracker.get_stats()
            stats["total_tracks_created"] = ts["total_tracks_created"]
            stats["active_tracks"] = ts["active_tracks"]
        if self.debug_overlay:
            stats["unique_tracks"] = self.debug_overlay.get_session_stats()["unique_tracks"]
        return stats


class GimbalController:
    """Simulates a stabilized gimbal by rotating the camera prim."""

    SLEW_RATE = 45.0

    def __init__(self, camera_prim_path: str, stage, default_tilt: float = -30.0):
        self.camera_path = camera_prim_path
        self.stage = stage
        self.pan = 0.0
        self.tilt = default_tilt
        self.target_pan = 0.0
        self.target_tilt = default_tilt
        self.tilt_min, self.tilt_max = GIMBAL_TILT_RANGE
        self._drone_roll = 0.0
        self._drone_pitch = 0.0
        self._drone_yaw = 0.0
        self._last_update_time = time.time()

    def set_angles(self, pan: float, tilt: float):
        self.target_pan = pan % 360
        self.target_tilt = max(self.tilt_min, min(self.tilt_max, tilt))

    def set_angles_immediate(self, pan: float, tilt: float):
        self.pan = pan % 360
        self.tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        self.target_pan = self.pan
        self.target_tilt = self.tilt
        self._update_camera_orientation()

    def update_drone_attitude(self, roll: float, pitch: float, yaw: float):
        self._drone_roll = roll
        self._drone_pitch = pitch
        self._drone_yaw = yaw

    def get_drone_attitude(self) -> tuple:
        return (self._drone_roll, self._drone_pitch, self._drone_yaw)

    def get_gimbal_angles(self) -> tuple:
        return (self.pan, self.tilt)

    def update(self, dt: float = None):
        if dt is None:
            current_time = time.time()
            dt = current_time - self._last_update_time
            self._last_update_time = current_time

        dt = min(dt, 0.1)
        max_delta = self.SLEW_RATE * dt

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

        tilt_diff = self.target_tilt - self.tilt
        if abs(tilt_diff) > max_delta:
            self.tilt += max_delta * np.sign(tilt_diff)
        else:
            self.tilt = self.target_tilt

        self._update_camera_orientation()

    def _update_camera_orientation(self):
        camera_prim = self.stage.GetPrimAtPath(self.camera_path)
        if not camera_prim.IsValid():
            return

        pan_rad = np.radians(-self.pan)
        tilt_rad = np.radians(self.tilt)

        cos_pan, sin_pan = np.cos(pan_rad), np.sin(pan_rad)
        cos_tilt, sin_tilt = np.cos(tilt_rad), np.sin(tilt_rad)

        look_dir = np.array([cos_pan * cos_tilt, sin_pan * cos_tilt, sin_tilt])
        look_dir = look_dir / np.linalg.norm(look_dir)

        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(look_dir, world_up)
        right_norm = np.linalg.norm(right)
        if right_norm < 1e-6:
            right = np.array([0.0, 1.0, 0.0])
        else:
            right = right / right_norm

        up = np.cross(right, look_dir)
        up = up / np.linalg.norm(up)

        R_cam_to_world = np.column_stack([right, up, -look_dir])
        world_rot = Rotation.from_matrix(R_cam_to_world)
        quat_xyzw = world_rot.as_quat()
        quat_wxyz = Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])

        xform = UsdGeom.Xformable(camera_prim)
        ops = xform.GetOrderedXformOps()

        orient_op = None
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_op = op
                break

        if orient_op is None:
            translate_val = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_val = op.Get()
                    break
            xform.ClearXformOpOrder()
            translate_op = xform.AddTranslateOp()
            if translate_val is not None:
                translate_op.Set(translate_val)
            orient_op = xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)

        orient_op.Set(quat_wxyz)


class FlightController:
    """PX4 flight controller interface via MAVLink."""

    PX4_MODE_OFFBOARD = 6 << 16

    MODE_NAMES = {
        1 << 16: "MANUAL", 2 << 16: "ALTCTL", 3 << 16: "POSCTL",
        4 << 16: "AUTO", 5 << 16: "ACRO", 6 << 16: "OFFBOARD",
        7 << 16: "STABILIZED", 8 << 16: "RATTITUDE",
    }

    def __init__(self, vehicle: Multirotor, world: World):
        self.vehicle = vehicle
        self.world = world
        self.mav = None
        self.connected = False
        self.state = FlightState()
        self._home_position = None
        self._step_callback = None

    def set_step_callback(self, callback):
        self._step_callback = callback

    def _step(self, render: bool = True):
        self.world.step(render=render)
        if self._step_callback:
            self._step_callback()

    def get_mode_name(self, custom_mode: int) -> str:
        if custom_mode in self.MODE_NAMES:
            return self.MODE_NAMES[custom_mode]
        main_mode_encoded = custom_mode & 0x00FF0000
        main_mode_name = self.MODE_NAMES.get(main_mode_encoded, "UNKNOWN")
        main_mode = (custom_mode >> 16) & 0xFF
        sub_mode = (custom_mode >> 24) & 0xFF
        if main_mode == 4 and sub_mode > 0:
            sub_names = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND"}
            return f"AUTO_{sub_names.get(sub_mode, f'SUB{sub_mode}')}"
        return main_mode_name

    def connect(self, max_steps: int = 500) -> bool:
        print("[FlightController] Connecting to MAVLink...", flush=True)
        self.mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

        for i in range(max_steps):
            self._step()
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.get_srcSystem() != 0:
                print(f"[FlightController] Connected to system {msg.get_srcSystem()}")
                self.mav.target_system = 1
                self.mav.target_component = 1
                self.connected = True
                self._update_state()
                self._home_position = self.state.position.copy()

                for param, val in [("COM_RC_IN_MODE", 2), ("COM_RCL_EXCEPT", 4),
                                  ("NAV_RCL_ACT", 0), ("NAV_DLL_ACT", 0)]:
                    self._set_param(param, val)

                for _ in range(50):
                    self._step()
                return True

        print("[FlightController] ERROR: Failed to connect")
        return False

    def _set_param(self, param_id: str, value: float) -> bool:
        if not self.connected:
            return False
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        param_id_bytes = param_id.encode('utf-8')[:16].ljust(16, b'\x00')
        self.mav.mav.param_set_send(self.mav.target_system, self.mav.target_component,
                                    param_id_bytes, float(value), param_type)
        for _ in range(50):
            self._step()
            msg = self.mav.recv_match(type='PARAM_VALUE', blocking=False)
            if msg and msg.param_id.rstrip('\x00') == param_id:
                return True
        return False

    def _update_state(self):
        pos = self.vehicle.state.position
        self.state.position = np.array(pos)
        self.state.altitude = pos[2]
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            self.state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.state.mode = msg.custom_mode
            self.state.mode_name = self.get_mode_name(msg.custom_mode)

    def _send_velocity_setpoint(self, vx=0.0, vy=0.0, vz=0.0):
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

    def _send_position_setpoint(self, x, y, z, yaw=float('nan')):
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
            x, y, -z, 0, 0, 0, 0, 0, 0, yaw, 0)

    def set_mode_offboard(self, max_attempts: int = 10) -> bool:
        for _ in range(300):
            self._send_velocity_setpoint()
            self._step()

        for attempt in range(max_attempts):
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0)

            for i in range(200):
                self._send_velocity_setpoint()
                self._step()
                if i % 20 == 0:
                    latest_mode = None
                    while True:
                        msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
                        if msg is None:
                            break
                        if msg.get_srcSystem() == self.mav.target_system:
                            latest_mode = msg.custom_mode
                    if latest_mode == self.PX4_MODE_OFFBOARD:
                        print(f"[FlightController] OFFBOARD mode set")
                        return True
        return False

    def arm(self, max_attempts: int = 3) -> bool:
        for attempt in range(max_attempts):
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            for _ in range(100):
                self._send_velocity_setpoint()
                self._step()
            self._update_state()
            if self.state.armed:
                print(f"[FlightController] Armed")
                return True
        return False

    def disarm(self) -> bool:
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        for _ in range(50):
            self._step()
        self._update_state()
        return not self.state.armed

    def takeoff(self, target_altitude: float, max_steps: int = 3000, callback=None) -> bool:
        print(f"[FlightController] Taking off to {target_altitude}m...")
        for step in range(max_steps):
            self._send_velocity_setpoint(vz=-2.0)
            self._step()
            self._update_state()
            if callback:
                callback(step, self.state)
            if self.state.altitude >= target_altitude - 0.5:
                print(f"[FlightController] Reached {self.state.altitude:.1f}m")
                return True
        return False

    def goto_position(self, target, tolerance=2.0, max_steps=2000, callback=None) -> bool:
        for step in range(max_steps):
            self._send_position_setpoint(target[0], target[1], target[2])
            self._step()
            self._update_state()
            if step % 200 == 0 and self.state.mode != self.PX4_MODE_OFFBOARD:
                self.mav.mav.command_long_send(
                    self.mav.target_system, self.mav.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0)
            if callback:
                callback(step, self.state, None)
            if np.linalg.norm(self.state.position - target) < tolerance:
                return True
        return False

    def fly_waypoints(self, waypoints, tolerance=2.0, callback=None):
        reached = 0
        for idx, wp in enumerate(waypoints):
            print(f"[FlightController] Waypoint {idx+1}/{len(waypoints)}")
            for step in range(2000):
                direction = wp - self.state.position
                distance = np.linalg.norm(direction)
                if distance < tolerance:
                    reached += 1
                    break
                if distance > 0:
                    velocity = direction / distance * min(3.0, distance)
                    self._send_velocity_setpoint(velocity[1], velocity[0], -velocity[2])
                self._step()
                self._update_state()
                if callback:
                    callback(step, self.state, idx)
        return reached, len(waypoints)

    def return_to_home(self, max_steps=2000, callback=None) -> bool:
        if self._home_position is None:
            return False
        print("[FlightController] Returning to home...")
        home_at_alt = np.array([self._home_position[0], self._home_position[1], self.state.altitude])
        return self.goto_position(home_at_alt, callback=callback)

    def land(self, max_steps=5000, callback=None) -> bool:
        print("[FlightController] Landing...")
        start_x, start_y = self.state.position[0], self.state.position[1]
        current_alt = self.state.altitude
        descent_rate = 0.02

        for step in range(max_steps):
            target_alt = max(0.0, current_alt - (step * descent_rate))
            self._send_position_setpoint(start_x, start_y, target_alt)
            self._step()
            self._update_state()
            if callback:
                callback(step, self.state)
            if self.state.altitude < 1.0:
                print(f"[FlightController] Landed at {self.state.altitude:.2f}m")
                return True
        return False


# ============================================================================
# Main Function
# ============================================================================

def run_drone_functions_check():
    """Main test function with profile support."""

    # Get profile
    try:
        profile = get_profile(PROFILE_NAME)
    except ValueError as e:
        print(f"ERROR: {e}")
        print("\nAvailable profiles:")
        for name, desc in list_profiles().items():
            print(f"  {name}: {desc}")
        return False

    print(f"\n{'=' * 70}")
    print(f"DRONE FUNCTIONS CHECK - {profile.name.upper()} PROFILE")
    print(f"{'=' * 70}")
    print(f"Description: {profile.description}")
    print(f"Estimated duration: {profile.estimated_duration}")

    # Determine output paths
    suffix = profile.get_output_suffix()
    VIDEO_OUTPUT = f"{OUTPUT_DIR}/functions_check{suffix}.mp4"
    REPORT_OUTPUT = f"{OUTPUT_DIR}/functions_check{suffix}_report.json"

    print(f"Video Output: {VIDEO_OUTPUT}")
    print(f"Report Output: {REPORT_OUTPUT}")
    print()

    start_time = time.time()
    test_results = []

    # ========================================================================
    # Initialize Simulation
    # ========================================================================

    print("\n[1/6] Initializing simulation world...")

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

    test_results.append(TestResult(
        name="Simulation Initialization", passed=True,
        message="Pegasus interface and physics scene initialized",
    ))

    # ========================================================================
    # Generate World (using profile config)
    # ========================================================================

    print("\n[2/6] Generating procedural world...")

    models_path = "/workspace/extensions/forest_generator/models"
    world_config_params = profile.get_world_config()
    world_config = WorldConfig(**world_config_params)

    world_gen = WorldGenerator(models_path, world_config)
    terrain_path = world_gen.generate_terrain()
    world_gen.setup_lighting()
    forest_result = world_gen.generate_forest(density=world_config_params["tree_density"],
                                               include_undergrowth=False)

    # Spawn targets using profile's cluster configuration
    all_targets = []
    all_people = []
    clusters = profile.get_vehicle_clusters()

    for center in clusters:
        vehicle_paths = world_gen.vehicles.spawn_vehicle_group(
            vehicle_types=["sedan2", "suv", "tank", "taxi", "police", "sports_car"],
            count=profile.config.vehicles_per_cluster,
            clustering=0.3,
            center=center,
        )
        all_targets.extend(vehicle_paths)

        if world_gen.people.person_configs:
            person_types = list(world_gen.people.person_configs.keys())
            people_paths = world_gen.people.spawn_crowd(
                person_types=person_types,
                count=profile.config.people_per_cluster,
                center=(center[0] + 10.0, center[1] + 10.0),
                radius=15.0,
            )
            all_people.extend(people_paths)

    print(f"[WorldGen] Created {len(forest_result['trees'])} trees, "
          f"{len(all_targets)} vehicles, {len(all_people)} people")

    test_results.append(TestResult(
        name="World Generation", passed=True,
        message=f"Generated {len(all_targets)} vehicles, {len(all_people)} people",
        data={"trees": len(forest_result['trees']), "vehicles": len(all_targets), "people": len(all_people)},
    ))

    for _ in range(10):
        simulation_app.update()

    # ========================================================================
    # Create Camera
    # ========================================================================

    print("\n[3/6] Creating camera...")

    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([0.0, 0.0, 5.0]),
        frequency=CAMERA_FPS,
        resolution=(CAMERA_WIDTH, CAMERA_HEIGHT),
        orientation=rot_utils.euler_angles_to_quats(np.array([90, 0, 0]), degrees=True),
    )

    camera.initialize()
    simulation_app.update()
    camera.initialize()
    simulation_app.update()

    camera_prim_fov = stage.GetPrimAtPath(CAMERA_PATH)
    if camera_prim_fov.IsValid():
        usd_camera = UsdGeom.Camera(camera_prim_fov)
        if usd_camera:
            usd_camera.GetHorizontalApertureAttr().Set(42.0)
            # Query actual camera parameters for debugging
            h_aperture = usd_camera.GetHorizontalApertureAttr().Get()
            v_aperture = usd_camera.GetVerticalApertureAttr().Get()
            focal_length = usd_camera.GetFocalLengthAttr().Get()
            if h_aperture and focal_length:
                import math
                actual_hfov = 2 * math.degrees(math.atan(h_aperture / (2 * focal_length)))
                actual_vfov = 2 * math.degrees(math.atan(v_aperture / (2 * focal_length))) if v_aperture else 0
                print(f"[CAMERA] Aperture: {h_aperture}x{v_aperture}mm, Focal: {focal_length}mm")
                print(f"[CAMERA] Actual FOV: {actual_hfov:.1f}° x {actual_vfov:.1f}°")

    gimbal = GimbalController(CAMERA_PATH, stage, default_tilt=-30.0)
    gimbal.set_angles_immediate(0, -30)

    test_results.append(TestResult(
        name="Camera Setup", passed=True,
        message=f"Camera initialized at {CAMERA_WIDTH}x{CAMERA_HEIGHT}",
    ))

    # ========================================================================
    # Create Vehicle
    # ========================================================================

    print("\n[4/6] Creating PX4 vehicle...")

    mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": 0, "px4_autolaunch": True, "px4_dir": "/px4",
        "px4_vehicle_model": pg.px4_default_airframe,
        "enable_lockstep": True, "update_rate": 250.0,
    })

    config = MultirotorConfig()
    config.backends = [PX4MavlinkBackend(mavlink_config)]

    vehicle = Multirotor(
        "/World/quadrotor", ROBOTS['Iris'], 0,
        [0.0, 0.0, 0.5],
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=config,
    )

    world.reset()
    timeline.play()

    print("[Warmup] Running 200 warmup frames...")
    for _ in range(200):
        world.step(render=True)

    fc = FlightController(vehicle, world)

    # Camera tracking callback
    camera_prim = stage.GetPrimAtPath(CAMERA_PATH)
    camera_xform = UsdGeom.Xformable(camera_prim)
    camera_translate_op = None
    for op in camera_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            camera_translate_op = op
            break
    if camera_translate_op is None:
        camera_translate_op = camera_xform.AddTranslateOp()

    def step_callback():
        try:
            drone_pos = vehicle.state.position
            if drone_pos is not None:
                camera_pos = np.array([drone_pos[0], drone_pos[1], drone_pos[2] + 0.5])
                camera_translate_op.Set(Gf.Vec3d(*camera_pos))
        except Exception:
            pass
        try:
            quat = vehicle.state.attitude
            if quat is not None and len(quat) == 4:
                rot = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
                euler = rot.as_euler('xyz', degrees=True)
                gimbal.update_drone_attitude(euler[0], euler[1], euler[2])
            gimbal.update()
        except Exception:
            pass

    fc.set_step_callback(step_callback)

    test_results.append(TestResult(
        name="Vehicle Creation", passed=True,
        message="Iris quadrotor spawned with PX4 backend",
    ))

    # ========================================================================
    # Initialize Video Recorder
    # ========================================================================

    print("\n[5/6] Initializing video recorder...")

    recorder = VideoRecorder(
        output_path=VIDEO_OUTPUT,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        fps=CAMERA_FPS,
        yolo_model_path=YOLO_MODEL_PATH,
        enable_tracking=ENABLE_TEMPORAL_COHERENCE,
    )

    if not recorder.start():
        test_results.append(TestResult(
            name="Video Recorder", passed=False,
            message="Failed to start FFmpeg",
        ))
    else:
        test_results.append(TestResult(
            name="Video Recorder", passed=True,
            message="FFmpeg pipeline started",
        ))

    # Initialize perception manager with native annotator
    perception_manager = None
    gt_config = None
    if BYTETRACK_AVAILABLE:
        try:
            perception_config = PerceptionConfig(
                enabled=True,
                mode="gt",
                camera_resolution=(CAMERA_WIDTH, CAMERA_HEIGHT),
                camera_prim_path=CAMERA_PATH,
            )
            perception_manager = PerceptionManager(perception_config)

            # Initialize native annotator (must be done after camera exists)
            if perception_manager.initialize(camera_prim_path=CAMERA_PATH):
                print("[Init] PerceptionManager initialized with native annotator")
            else:
                print("[Init] PerceptionManager: native annotator not available, GT disabled")
                perception_manager = None

            # GT config for noise injection (optional)
            gt_config = GroundTruthConfig(
                bbox_noise_std=0.0,
                confidence_noise_std=0.0,
                position_noise_std=0.0,
                detection_probability=1.0,
                max_detection_range=500.0,
            )
        except Exception as e:
            print(f"[Init] PerceptionManager error: {e}")

    # ========================================================================
    # Connect and Arm
    # ========================================================================

    print("\n[6/6] Connecting to PX4 and arming...")

    if not fc.connect():
        test_results.append(TestResult(name="MAVLink Connection", passed=False, message="Failed"))
        recorder.stop()
        simulation_app.close()
        return False

    test_results.append(TestResult(name="MAVLink Connection", passed=True, message="Connected"))

    if not fc.set_mode_offboard():
        test_results.append(TestResult(name="OFFBOARD Mode", passed=False, message="Failed"))
    else:
        test_results.append(TestResult(name="OFFBOARD Mode", passed=True, message="Set"))

    if not fc.arm():
        test_results.append(TestResult(name="Arming", passed=False, message="Failed"))
    else:
        test_results.append(TestResult(name="Arming", passed=True, message="Armed"))

    # ========================================================================
    # Build Simulation Context and Run Profile
    # ========================================================================

    print(f"\n{'=' * 70}")
    print(f"RUNNING {profile.name.upper()} PROFILE")
    print(f"{'=' * 70}")

    ctx = SimulationContext()
    ctx.simulation_app = simulation_app
    ctx.world = world
    ctx.stage = stage
    ctx.timeline = timeline
    ctx.vehicle = vehicle
    ctx.camera = camera
    ctx.gimbal = gimbal
    ctx.flight_controller = fc
    ctx.video_recorder = recorder
    ctx.world_generator = world_gen
    ctx.perception_manager = perception_manager
    ctx.gt_config = gt_config
    ctx.headless = HEADLESS
    ctx.camera_width = CAMERA_WIDTH
    ctx.camera_height = CAMERA_HEIGHT
    ctx.camera_fps = CAMERA_FPS

    # Run the profile's flight sequence
    flight_results = profile.run_flight(ctx)
    test_results.extend(flight_results)

    # ========================================================================
    # Finalize
    # ========================================================================

    print(f"\n{'=' * 70}")
    print("FINALIZING")
    print(f"{'=' * 70}")

    recorder.stop()

    total_time = time.time() - start_time
    passed = sum(1 for r in test_results if r.passed)
    total = len(test_results)
    all_passed = passed == total

    tracking_stats = recorder.get_tracking_stats()

    report = {
        "profile": profile.name,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "headless": HEADLESS,
        "total_duration_seconds": total_time,
        "video_output": VIDEO_OUTPUT,
        "frames_recorded": recorder.frame_count,
        "tests_passed": passed,
        "tests_total": total,
        "all_passed": all_passed,
        "tracking_stats": tracking_stats,
        "tests": [asdict(r) for r in test_results],
    }

    try:
        with open(REPORT_OUTPUT, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"[Report] Saved to {REPORT_OUTPUT}")
    except Exception as e:
        print(f"[Report] ERROR: {e}")

    print(f"\n{'=' * 70}")
    print("TEST RESULTS SUMMARY")
    print(f"{'=' * 70}")

    for result in test_results:
        status = "PASS" if result.passed else "FAIL"
        print(f"  [{status}] {result.name}: {result.message}")

    print()
    print(f"Profile: {profile.name}")
    print(f"Total Tests: {total}")
    print(f"Passed: {passed}")
    print(f"Failed: {total - passed}")
    print(f"Duration: {total_time:.1f}s")
    print(f"Frames: {recorder.frame_count}")
    print()

    if all_passed:
        print("OVERALL RESULT: ALL TESTS PASSED")
    else:
        print("OVERALL RESULT: SOME TESTS FAILED")

    print(f"{'=' * 70}")
    print()
    print("[Shutdown] Cleaning up...")

    timeline.stop()
    simulation_app.close()

    return all_passed


if __name__ == "__main__":
    run_drone_functions_check()
