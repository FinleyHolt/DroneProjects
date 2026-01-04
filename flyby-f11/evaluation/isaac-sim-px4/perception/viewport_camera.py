"""
Viewport-based Camera for Isaac Sim Perception Pipeline

This module provides a viewport-based alternative to the buggy Isaac Sim Camera API.
Instead of using the Camera class + replicator annotators (which crash in Isaac Sim 5.1),
this uses direct viewport capture to get rendered images for YOLO inference.

Architecture:
1. Creates a USD Camera prim that follows the drone
2. Sets the viewport's active camera to this prim
3. Captures viewport to buffer/file for YOLO processing
4. Logs detections with timestamps for post-processing replay

Usage in environments:
    from perception.viewport_camera import ViewportCamera

    camera = ViewportCamera(resolution=(640, 480))
    camera.initialize(stage, drone_prim_path="/World/quadrotor/body")

    # In step loop:
    camera.update_pose(drone_position, drone_orientation)
    image = camera.capture()  # Returns RGB numpy array

    # For detection logging:
    camera.log_detections(detections, frame_id, timestamp)
"""

import os
import time
import json
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple
from pathlib import Path
from scipy.spatial.transform import Rotation


@dataclass
class ViewportCameraConfig:
    """Configuration for viewport-based camera capture."""
    # Image resolution
    resolution: Tuple[int, int] = (640, 480)

    # Camera mounting
    mount_offset: Tuple[float, float, float] = (0.0, 0.0, -0.1)  # Below drone body
    pitch_down_degrees: float = 45.0  # Angle to look down

    # Camera optics
    focal_length: float = 4.5  # Wide angle
    clipping_range: Tuple[float, float] = (0.1, 500.0)

    # Capture settings
    capture_to_file: bool = False  # Save each frame to disk
    output_dir: str = "/tmp/perception_frames"

    # Detection logging
    log_detections: bool = True
    detection_log_path: str = "/tmp/perception_detections.jsonl"


class ViewportCamera:
    """
    Viewport-based camera that avoids the buggy Camera API.

    Uses the viewport render API to capture images during simulation,
    providing a stable alternative for YOLO inference.
    """

    def __init__(self, config: Optional[ViewportCameraConfig] = None):
        self.config = config or ViewportCameraConfig()

        self._stage = None
        self._camera_prim_path = None
        self._viewport = None
        self._initialized = False

        # Frame counting
        self._frame_count = 0
        self._capture_count = 0

        # Performance tracking
        self._capture_times: List[float] = []

        # Detection log file handle
        self._detection_log_file = None

        # Last captured image cache
        self._last_image: Optional[np.ndarray] = None
        self._last_capture_time: float = 0.0

    def initialize(
        self,
        stage,
        drone_prim_path: str = "/World/quadrotor",
        camera_name: str = "perception_camera"
    ) -> bool:
        """
        Initialize the viewport camera.

        Args:
            stage: USD stage object
            drone_prim_path: Path to drone prim (camera attached here)
            camera_name: Name for the camera prim

        Returns:
            True if initialization successful
        """
        try:
            from pxr import UsdGeom, Gf
            import omni.kit.viewport.utility as vp_utils

            self._stage = stage

            # Create camera prim under drone body
            body_path = f"{drone_prim_path}/body"
            if not stage.GetPrimAtPath(body_path):
                body_path = drone_prim_path  # Fallback to drone root

            self._camera_prim_path = f"{body_path}/{camera_name}"

            # Create USD camera
            camera_prim = UsdGeom.Camera.Define(stage, self._camera_prim_path)
            camera_prim.CreateFocalLengthAttr(self.config.focal_length)
            camera_prim.CreateClippingRangeAttr(Gf.Vec2f(*self.config.clipping_range))

            print(f"[ViewportCamera] Created camera at {self._camera_prim_path}")

            # Get viewport and set active camera
            self._viewport = vp_utils.get_active_viewport()
            if self._viewport:
                self._viewport.set_active_camera(self._camera_prim_path)
                print("[ViewportCamera] Set as active viewport camera")
            else:
                print("[ViewportCamera] Warning: Could not get active viewport")

            # Create output directory if needed
            if self.config.capture_to_file:
                os.makedirs(self.config.output_dir, exist_ok=True)
                print(f"[ViewportCamera] Frame output: {self.config.output_dir}")

            # Open detection log file
            if self.config.log_detections:
                log_dir = os.path.dirname(self.config.detection_log_path)
                if log_dir:
                    os.makedirs(log_dir, exist_ok=True)
                self._detection_log_file = open(self.config.detection_log_path, 'a')
                print(f"[ViewportCamera] Detection log: {self.config.detection_log_path}")

            self._initialized = True
            return True

        except Exception as e:
            print(f"[ViewportCamera] Initialization error: {e}")
            return False

    def update_pose(
        self,
        drone_position: np.ndarray,
        drone_orientation: np.ndarray  # quaternion [x, y, z, w]
    ) -> None:
        """
        Update camera pose to follow drone.

        Args:
            drone_position: Drone world position [x, y, z]
            drone_orientation: Drone orientation as quaternion [x, y, z, w]
        """
        if not self._initialized or self._stage is None:
            return

        try:
            from pxr import UsdGeom, Gf

            camera_xform = UsdGeom.Xformable(self._stage.GetPrimAtPath(self._camera_prim_path))
            camera_xform.ClearXformOpOrder()

            # Position: drone position + mount offset
            offset = np.array(self.config.mount_offset)
            camera_pos = drone_position + offset
            camera_xform.AddTranslateOp().Set(Gf.Vec3d(*camera_pos.tolist()))

            # Orientation: drone yaw + pitch down
            # Get drone yaw from quaternion
            drone_rot = Rotation.from_quat(drone_orientation)
            drone_euler = drone_rot.as_euler('ZYX', degrees=True)
            drone_yaw = drone_euler[0]

            # Build camera rotation:
            # Isaac Sim cameras look down -Z by default
            # Rx(90 - pitch) makes it look forward-and-down
            pitch_angle = 90.0 - self.config.pitch_down_degrees
            camera_rot = Rotation.from_euler("ZX", [drone_yaw, pitch_angle], degrees=True)
            quat = camera_rot.as_quat()  # [x, y, z, w]
            camera_xform.AddOrientOp().Set(Gf.Quatf(quat[3], quat[0], quat[1], quat[2]))

        except Exception as e:
            print(f"[ViewportCamera] Pose update error: {e}")

    def capture(self) -> Optional[np.ndarray]:
        """
        Capture current viewport to RGB numpy array.

        Returns:
            RGB image as numpy array (H, W, 3) or None if capture failed
        """
        if not self._initialized or self._viewport is None:
            return None

        start_time = time.perf_counter()
        self._frame_count += 1

        try:
            import omni.kit.viewport.utility as vp_utils

            if self.config.capture_to_file:
                # Capture to file and read back
                filename = f"{self.config.output_dir}/frame_{self._capture_count:06d}.png"
                vp_utils.capture_viewport_to_file(self._viewport, filename)

                # Read the file back as numpy array
                from PIL import Image
                if os.path.exists(filename):
                    img = Image.open(filename)
                    image = np.array(img)[:, :, :3]  # RGB only
                    self._capture_count += 1
                    self._last_image = image
                    self._last_capture_time = time.time()

                    # Track timing
                    elapsed = (time.perf_counter() - start_time) * 1000
                    self._capture_times.append(elapsed)
                    if len(self._capture_times) > 100:
                        self._capture_times.pop(0)

                    return image
            else:
                # Use async capture to buffer (more efficient)
                # Note: capture_viewport_to_buffer requires callback in Isaac Sim 5.1
                # Fallback to file-based for now
                filename = f"/tmp/viewport_capture_{self._frame_count % 10}.png"
                vp_utils.capture_viewport_to_file(self._viewport, filename)

                from PIL import Image
                if os.path.exists(filename):
                    img = Image.open(filename)
                    image = np.array(img)[:, :, :3]
                    self._last_image = image
                    self._last_capture_time = time.time()

                    elapsed = (time.perf_counter() - start_time) * 1000
                    self._capture_times.append(elapsed)
                    if len(self._capture_times) > 100:
                        self._capture_times.pop(0)

                    return image

        except Exception as e:
            print(f"[ViewportCamera] Capture error: {e}")

        return None

    def log_detections(
        self,
        detections: List[Dict[str, Any]],
        frame_id: int,
        timestamp: float,
        uav_position: Optional[np.ndarray] = None
    ) -> None:
        """
        Log detections to JSONL file for post-processing.

        Each line is a JSON object with:
        - frame_id: Sequential frame number
        - timestamp: Simulation time
        - uav_position: [x, y, z] world position
        - detections: List of detection dicts

        Args:
            detections: List of detection dictionaries
            frame_id: Frame number for reference
            timestamp: Simulation timestamp
            uav_position: UAV position at time of detection
        """
        if not self.config.log_detections or self._detection_log_file is None:
            return

        try:
            log_entry = {
                'frame_id': frame_id,
                'timestamp': timestamp,
                'uav_position': uav_position.tolist() if uav_position is not None else None,
                'detections': [
                    {
                        'class_id': d.get('class_id', d.get('class', 0)),
                        'class_name': d.get('class_name', 'unknown'),
                        'confidence': float(d.get('confidence', 0.0)),
                        'bbox': d.get('bbox', [0, 0, 0, 0]),
                        'position_3d': d.get('position_3d', None),
                        'distance': float(d.get('distance', 0.0)),
                    }
                    for d in detections
                ],
                'detection_count': len(detections),
            }

            self._detection_log_file.write(json.dumps(log_entry) + '\n')
            self._detection_log_file.flush()

        except Exception as e:
            print(f"[ViewportCamera] Detection log error: {e}")

    def get_stats(self) -> Dict[str, Any]:
        """Get camera statistics."""
        avg_capture_time = (
            sum(self._capture_times) / len(self._capture_times)
            if self._capture_times else 0.0
        )

        return {
            'initialized': self._initialized,
            'camera_prim': self._camera_prim_path,
            'frame_count': self._frame_count,
            'capture_count': self._capture_count,
            'avg_capture_time_ms': avg_capture_time,
            'resolution': self.config.resolution,
            'capture_to_file': self.config.capture_to_file,
            'log_detections': self.config.log_detections,
        }

    def close(self) -> None:
        """Close camera and cleanup resources."""
        if self._detection_log_file is not None:
            self._detection_log_file.close()
            self._detection_log_file = None

        self._initialized = False


class DetectionLogger:
    """
    Standalone detection logger for recording YOLO detections during training.

    Logs detections in JSONL format with:
    - Timestamp and frame ID
    - UAV state (position, orientation, velocity)
    - All detections with class, confidence, bbox, 3D position
    - Episode metadata

    Can be used independently of ViewportCamera for ground truth logging.
    """

    def __init__(
        self,
        log_dir: str = "/tmp/detection_logs",
        episode_id: Optional[str] = None
    ):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Generate episode ID if not provided
        if episode_id is None:
            episode_id = time.strftime("%Y%m%d_%H%M%S")
        self.episode_id = episode_id

        # Log file path
        self.log_path = self.log_dir / f"detections_{episode_id}.jsonl"
        self._log_file = open(self.log_path, 'w')

        # Statistics
        self._total_detections = 0
        self._frame_count = 0
        self._detection_counts = {'person': 0, 'vehicle': 0, 'building': 0, 'other': 0}

        # Write episode header
        header = {
            'type': 'episode_start',
            'episode_id': episode_id,
            'start_time': time.time(),
            'log_path': str(self.log_path),
        }
        self._log_file.write(json.dumps(header) + '\n')

        print(f"[DetectionLogger] Logging to {self.log_path}")

    def log_frame(
        self,
        frame_id: int,
        timestamp: float,
        uav_position: np.ndarray,
        uav_orientation: np.ndarray,
        uav_velocity: np.ndarray,
        detections: List[Any],  # List of Detection objects or dicts
        perception_mode: str = "unknown"
    ) -> None:
        """
        Log detections for a single frame.

        Args:
            frame_id: Sequential frame number
            timestamp: Simulation time in seconds
            uav_position: [x, y, z] world position
            uav_orientation: Quaternion [x, y, z, w]
            uav_velocity: [vx, vy, vz] velocity
            detections: List of Detection objects or dictionaries
            perception_mode: "gt" or "full" (for analysis)
        """
        # Convert detections to serializable format
        det_list = []
        for det in detections:
            if hasattr(det, 'class_id'):
                # Detection dataclass
                det_dict = {
                    'class_id': det.class_id,
                    'class_name': det.class_name,
                    'ontology_class': getattr(det, 'ontology_class', 'unknown'),
                    'confidence': float(det.confidence),
                    'bbox': list(det.bbox) if det.bbox else None,
                    'distance': float(det.distance),
                    'bearing': float(det.bearing),
                    'priority': det.priority,
                    'track_id': det.track_id,
                }
                if det.position_3d is not None:
                    det_dict['position_3d'] = det.position_3d.tolist()
            else:
                # Already a dict
                det_dict = dict(det)

            det_list.append(det_dict)

            # Update counts
            class_name = det_dict.get('class_name', 'other')
            if class_name in self._detection_counts:
                self._detection_counts[class_name] += 1
            else:
                self._detection_counts['other'] += 1

        # Build frame entry
        entry = {
            'type': 'frame',
            'frame_id': frame_id,
            'timestamp': timestamp,
            'perception_mode': perception_mode,
            'uav_state': {
                'position': uav_position.tolist(),
                'orientation': uav_orientation.tolist(),
                'velocity': uav_velocity.tolist(),
            },
            'detections': det_list,
            'detection_count': len(det_list),
        }

        self._log_file.write(json.dumps(entry) + '\n')
        self._frame_count += 1
        self._total_detections += len(det_list)

        # Flush periodically
        if self._frame_count % 100 == 0:
            self._log_file.flush()

    def log_episode_end(
        self,
        total_reward: float,
        episode_length: int,
        success: bool,
        info: Optional[Dict] = None
    ) -> None:
        """Log episode completion with summary statistics."""
        summary = {
            'type': 'episode_end',
            'episode_id': self.episode_id,
            'end_time': time.time(),
            'total_frames': self._frame_count,
            'total_detections': self._total_detections,
            'detection_counts': self._detection_counts.copy(),
            'total_reward': total_reward,
            'episode_length': episode_length,
            'success': success,
            'info': info or {},
        }
        self._log_file.write(json.dumps(summary) + '\n')
        self._log_file.flush()

    def get_stats(self) -> Dict[str, Any]:
        """Get logging statistics."""
        return {
            'episode_id': self.episode_id,
            'log_path': str(self.log_path),
            'frame_count': self._frame_count,
            'total_detections': self._total_detections,
            'detection_counts': self._detection_counts.copy(),
        }

    def close(self) -> None:
        """Close log file."""
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
