"""
Dual-Mode Perception Pipeline for Training and E2E Validation

This module provides the top-level perception interface that switches between:

1. TRAINING MODE ("gt" / ground_truth):
   - Uses GroundTruthDetector with frustum math
   - No rendering required
   - Runs at 1000+ env steps/second
   - Configurable noise matches real detector behavior

2. E2E VALIDATION MODE ("full" / inference):
   - Uses YOLODetector with real inference
   - Requires rendered camera images
   - Runs at ~20 Hz (real-time)
   - Validates full pipeline before deployment

Both modes produce IDENTICAL 516-dim observation vectors.
Policy trained in GT mode works in E2E mode with <10% performance delta.

Usage:
    # In environment __init__
    perception = DualModePerception(
        mode="gt",  # or "full" for E2E
        camera_params=CameraParams(fov_horizontal=90.0),
    )

    # In environment step()
    if mode == "gt":
        obs = perception.get_observation_gt(
            world_objects=sim_objects,
            uav_position=pos,
            uav_orientation=quat,
        )
    else:
        obs = perception.get_observation_full(
            image=camera_image,
            uav_position=pos,
        )
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Tuple, Literal
import time

try:
    from .perception_encoder import PerceptionEncoder, PerceptionConfig, OUTPUT_DIM
    from .detector import YOLODetector, Detection
    from .ground_truth_detector import (
        GroundTruthDetector, GroundTruthConfig, WorldObject,
        create_world_objects_from_sim
    )
    from .frustum import CameraParams, camera_pose_from_position_orientation
    from .tptp_generator import TPTPGenerator
    from .viewport_camera import ViewportCamera, ViewportCameraConfig, DetectionLogger
except ImportError:
    from perception_encoder import PerceptionEncoder, PerceptionConfig, OUTPUT_DIM
    from detector import YOLODetector, Detection
    from ground_truth_detector import (
        GroundTruthDetector, GroundTruthConfig, WorldObject,
        create_world_objects_from_sim
    )
    from frustum import CameraParams, camera_pose_from_position_orientation
    from tptp_generator import TPTPGenerator
    try:
        from viewport_camera import ViewportCamera, ViewportCameraConfig, DetectionLogger
    except ImportError:
        ViewportCamera = None
        ViewportCameraConfig = None
        DetectionLogger = None


PerceptionMode = Literal["gt", "full", "ground_truth", "inference"]


@dataclass
class DualModeConfig:
    """Configuration for dual-mode perception."""
    # Mode selection
    mode: PerceptionMode = "gt"

    # Camera parameters (shared between modes)
    camera_params: CameraParams = field(default_factory=CameraParams)

    # Ground truth detector config (training mode)
    gt_bbox_noise_std: float = 0.02
    gt_confidence_noise_std: float = 0.05
    gt_position_noise_std: float = 0.5
    gt_detection_prob: float = 0.98

    # YOLO detector config (E2E mode)
    yolo_model: str = "yolo11n.pt"
    yolo_confidence: float = 0.25
    yolo_config_path: Optional[str] = None

    # Encoder config (shared)
    max_priority_detections: int = 10
    detection_feature_dim: int = 10
    grid_size: Tuple[int, int] = (8, 8)
    grid_channels: int = 6

    # Performance
    skip_frames: int = 0
    max_encode_time_ms: float = 20.0

    # Isaac Sim viewport camera config (E2E mode)
    use_viewport_camera: bool = False
    viewport_resolution: Tuple[int, int] = (640, 480)
    viewport_pitch_down: float = 45.0
    capture_frames_to_disk: bool = False
    frame_output_dir: str = "/tmp/perception_frames"

    # Detection logging
    log_detections: bool = True
    detection_log_dir: str = "/tmp/detection_logs"


class DualModePerception:
    """
    Top-level perception interface supporting both training and E2E modes.

    This class orchestrates the perception pipeline, switching between:
    - Ground truth detection (fast, for training)
    - Real YOLO inference (accurate, for E2E validation)

    Both paths feed into the SAME encoder, producing identical observation formats.
    """

    # Formally specified output dimension
    OUTPUT_DIM = 516

    def __init__(self, config: DualModeConfig = None):
        self.config = config or DualModeConfig()

        # Normalize mode string
        self.mode = self._normalize_mode(self.config.mode)

        # Initialize shared encoder
        self.encoder = PerceptionEncoder(PerceptionConfig(
            max_priority_detections=self.config.max_priority_detections,
            detection_feature_dim=self.config.detection_feature_dim,
            grid_size=self.config.grid_size,
            grid_channels=self.config.grid_channels,
        ))

        # Initialize mode-specific detectors
        self._gt_detector: Optional[GroundTruthDetector] = None
        self._yolo_detector: Optional[YOLODetector] = None
        self._tptp_generator: Optional[TPTPGenerator] = None

        # Viewport camera for Isaac Sim E2E mode
        self._viewport_camera: Optional[ViewportCamera] = None
        self._detection_logger: Optional[DetectionLogger] = None

        # Initialize detectors based on mode
        self._init_detectors()

        # State
        self._frame_count = 0
        self._skip_counter = 0
        self._last_observation: Optional[np.ndarray] = None
        self._last_detections: List[Detection] = []

        # Statistics
        self._total_detect_time = 0.0
        self._total_encode_time = 0.0
        self._total_capture_time = 0.0
        self._detection_counts = {"person": 0, "vehicle": 0, "building": 0, "other": 0}

    def _normalize_mode(self, mode: str) -> str:
        """Normalize mode string to canonical form."""
        mode = mode.lower()
        if mode in ["gt", "ground_truth", "training"]:
            return "gt"
        elif mode in ["full", "inference", "e2e", "yolo"]:
            return "full"
        else:
            raise ValueError(f"Unknown perception mode: {mode}. Use 'gt' or 'full'.")

    def _init_detectors(self):
        """Initialize detectors based on mode."""
        if self.mode == "gt":
            gt_config = GroundTruthConfig(
                camera_params=self.config.camera_params,
                bbox_noise_std=self.config.gt_bbox_noise_std,
                confidence_noise_std=self.config.gt_confidence_noise_std,
                position_noise_std=self.config.gt_position_noise_std,
                base_detection_prob=self.config.gt_detection_prob,
            )
            self._gt_detector = GroundTruthDetector(gt_config)
        else:
            self._yolo_detector = YOLODetector(
                config_path=self.config.yolo_config_path,
                mode="inference",
            )

        # TPTP generator for Vampire integration (both modes)
        try:
            self._tptp_generator = TPTPGenerator()
        except Exception:
            self._tptp_generator = None

        # Initialize detection logger for E2E mode
        if self.config.log_detections and DetectionLogger is not None:
            try:
                self._detection_logger = DetectionLogger(
                    log_dir=self.config.detection_log_dir
                )
            except Exception as e:
                print(f"[DualModePerception] Warning: Could not init detection logger: {e}")
                self._detection_logger = None

    def init_viewport_camera(self, stage, drone_prim_path: str = "/World/quadrotor") -> bool:
        """
        Initialize viewport camera for Isaac Sim E2E mode.

        Call this method after world initialization to enable viewport-based
        image capture for YOLO inference.

        Args:
            stage: USD stage object
            drone_prim_path: Path to drone prim

        Returns:
            True if initialization successful
        """
        if ViewportCamera is None:
            print("[DualModePerception] ViewportCamera not available")
            return False

        if not self.config.use_viewport_camera and self.mode != "full":
            return False

        try:
            vp_config = ViewportCameraConfig(
                resolution=self.config.viewport_resolution,
                pitch_down_degrees=self.config.viewport_pitch_down,
                capture_to_file=self.config.capture_frames_to_disk,
                output_dir=self.config.frame_output_dir,
                log_detections=self.config.log_detections,
                detection_log_path=f"{self.config.detection_log_dir}/viewport_detections.jsonl",
            )

            self._viewport_camera = ViewportCamera(vp_config)
            success = self._viewport_camera.initialize(stage, drone_prim_path)

            if success:
                print(f"[DualModePerception] Viewport camera initialized")
            return success

        except Exception as e:
            print(f"[DualModePerception] Viewport camera init error: {e}")
            return False

    @property
    def output_dim(self) -> int:
        """Output observation dimension."""
        return self.OUTPUT_DIM

    def set_seed(self, seed: int):
        """Set random seed for reproducible ground truth noise."""
        if self._gt_detector is not None:
            self._gt_detector.set_seed(seed)

    def get_observation(
        self,
        uav_position: np.ndarray,
        uav_orientation: np.ndarray,
        world_objects: Optional[List[WorldObject]] = None,
        image: Optional[np.ndarray] = None,
        dt: float = 0.05,
    ) -> np.ndarray:
        """
        Get perception observation (auto-selects based on mode).

        Args:
            uav_position: UAV position (x, y, z) in world frame
            uav_orientation: UAV orientation as quaternion (w, x, y, z)
            world_objects: List of WorldObject (required for GT mode)
            image: RGB image (required for full mode)
            dt: Time since last frame

        Returns:
            516-dimensional perception observation vector
        """
        if self.mode == "gt":
            if world_objects is None:
                raise ValueError("world_objects required for GT mode")
            return self.get_observation_gt(
                world_objects, uav_position, uav_orientation, dt
            )
        else:
            if image is None:
                raise ValueError("image required for full/E2E mode")
            return self.get_observation_full(
                image, uav_position, dt
            )

    def get_observation_gt(
        self,
        world_objects: List[WorldObject],
        uav_position: np.ndarray,
        uav_orientation: np.ndarray,
        dt: float = 0.05,
    ) -> np.ndarray:
        """
        Get perception observation using ground truth detection.

        This is the FAST PATH for training - no rendering required.

        Args:
            world_objects: List of WorldObject with current positions
            uav_position: UAV position (x, y, z)
            uav_orientation: UAV orientation quaternion (w, x, y, z)
            dt: Time since last frame

        Returns:
            516-dimensional perception observation
        """
        # Handle frame skipping
        if self._should_skip_frame():
            return self._get_cached_observation()

        start_time = time.perf_counter()

        # Update world objects in detector
        self._gt_detector.update_world_objects(world_objects)

        # Construct camera pose from UAV state
        camera_pose = camera_pose_from_position_orientation(
            uav_position, uav_orientation
        )

        # Run ground truth detection
        detect_start = time.perf_counter()
        detections = self._gt_detector.detect(camera_pose, uav_position)
        self._total_detect_time += time.perf_counter() - detect_start

        # Encode detections
        encode_start = time.perf_counter()
        observation = self.encoder.encode_from_detections(detections, uav_position, dt)
        self._total_encode_time += time.perf_counter() - encode_start

        # Update state
        self._update_stats(detections, time.perf_counter() - start_time)
        self._last_observation = observation
        self._last_detections = detections

        return observation

    def get_observation_full(
        self,
        image: np.ndarray,
        uav_position: np.ndarray,
        dt: float = 0.05,
    ) -> np.ndarray:
        """
        Get perception observation using full YOLO inference.

        This is the E2E PATH for validation - runs real inference.

        Args:
            image: RGB image (H, W, 3) uint8
            uav_position: UAV position (x, y, z)
            dt: Time since last frame

        Returns:
            516-dimensional perception observation
        """
        # Handle frame skipping
        if self._should_skip_frame():
            return self._get_cached_observation()

        start_time = time.perf_counter()

        # Run YOLO detection
        detect_start = time.perf_counter()
        detections = self._yolo_detector.detect(image, uav_position=uav_position)
        self._total_detect_time += time.perf_counter() - detect_start

        # Encode detections
        encode_start = time.perf_counter()
        observation = self.encoder.encode_from_detections(detections, uav_position, dt)
        self._total_encode_time += time.perf_counter() - encode_start

        # Update state
        self._update_stats(detections, time.perf_counter() - start_time)
        self._last_observation = observation
        self._last_detections = detections

        return observation

    def get_observation_viewport(
        self,
        uav_position: np.ndarray,
        uav_orientation: np.ndarray,
        uav_velocity: np.ndarray,
        dt: float = 0.05,
    ) -> np.ndarray:
        """
        Get perception observation using viewport camera capture + YOLO.

        This is the Isaac Sim E2E PATH - captures from viewport and runs inference.
        Requires init_viewport_camera() to be called first.

        Args:
            uav_position: UAV position (x, y, z) in world frame
            uav_orientation: UAV orientation as quaternion (x, y, z, w)
            uav_velocity: UAV velocity (vx, vy, vz)
            dt: Time since last frame

        Returns:
            516-dimensional perception observation
        """
        if self._viewport_camera is None:
            # Fallback to zero observation if viewport not initialized
            return np.zeros(self.OUTPUT_DIM, dtype=np.float32)

        # Handle frame skipping
        if self._should_skip_frame():
            return self._get_cached_observation()

        start_time = time.perf_counter()

        # Update camera pose to follow drone
        capture_start = time.perf_counter()
        self._viewport_camera.update_pose(uav_position, uav_orientation)

        # Capture image from viewport
        image = self._viewport_camera.capture()
        self._total_capture_time += time.perf_counter() - capture_start

        if image is None:
            return self._get_cached_observation()

        # Run YOLO detection
        detect_start = time.perf_counter()
        detections = self._yolo_detector.detect(image, uav_position=uav_position)
        self._total_detect_time += time.perf_counter() - detect_start

        # Log detections
        if self._detection_logger is not None:
            self._detection_logger.log_frame(
                frame_id=self._frame_count,
                timestamp=time.time(),
                uav_position=uav_position,
                uav_orientation=uav_orientation,
                uav_velocity=uav_velocity,
                detections=detections,
                perception_mode="full"
            )

        # Encode detections
        encode_start = time.perf_counter()
        observation = self.encoder.encode_from_detections(detections, uav_position, dt)
        self._total_encode_time += time.perf_counter() - encode_start

        # Update state
        self._update_stats(detections, time.perf_counter() - start_time)
        self._last_observation = observation
        self._last_detections = detections

        return observation

    def get_detections(self) -> List[Detection]:
        """Get the last set of detections (for TPTP generation, visualization)."""
        return self._last_detections

    def generate_tptp_facts(self) -> List[str]:
        """Generate TPTP facts from last detections for Vampire safety checking."""
        if self._tptp_generator is None or not self._last_detections:
            return []
        return self._tptp_generator.generate_facts(self._last_detections)

    def _should_skip_frame(self) -> bool:
        """Check if we should skip this frame."""
        if self.config.skip_frames <= 0:
            return False

        self._skip_counter += 1
        if self._skip_counter <= self.config.skip_frames:
            return True

        self._skip_counter = 0
        return False

    def _get_cached_observation(self) -> np.ndarray:
        """Return cached observation for skipped frames."""
        if self._last_observation is not None:
            return self._last_observation
        return np.zeros(self.OUTPUT_DIM, dtype=np.float32)

    def _update_stats(self, detections: List[Detection], total_time: float):
        """Update internal statistics."""
        self._frame_count += 1

        for det in detections:
            if det.class_id == 1:
                self._detection_counts["person"] += 1
            elif det.class_id == 2:
                self._detection_counts["vehicle"] += 1
            elif det.class_id == 3:
                self._detection_counts["building"] += 1
            else:
                self._detection_counts["other"] += 1

    def get_stats(self) -> Dict[str, Any]:
        """Get perception statistics for logging."""
        frames = max(1, self._frame_count)
        total_time = self._total_detect_time + self._total_encode_time + self._total_capture_time

        stats = {
            "mode": self.mode,
            "frames_processed": self._frame_count,
            "avg_detect_time_ms": (self._total_detect_time / frames) * 1000,
            "avg_encode_time_ms": (self._total_encode_time / frames) * 1000,
            "avg_capture_time_ms": (self._total_capture_time / frames) * 1000,
            "avg_total_time_ms": (total_time / frames) * 1000,
            "detection_counts": self._detection_counts.copy(),
            "within_budget": (total_time / frames) * 1000 < self.config.max_encode_time_ms,
        }

        # Add viewport camera stats if available
        if self._viewport_camera is not None:
            stats["viewport_camera"] = self._viewport_camera.get_stats()

        # Add detection logger stats if available
        if self._detection_logger is not None:
            stats["detection_logger"] = self._detection_logger.get_stats()

        return stats

    def reset(self):
        """Reset perception state for new episode."""
        self._frame_count = 0
        self._skip_counter = 0
        self._last_observation = None
        self._last_detections = []
        self._total_detect_time = 0.0
        self._total_encode_time = 0.0
        self._total_capture_time = 0.0
        self._detection_counts = {"person": 0, "vehicle": 0, "building": 0, "other": 0}

        # Reset encoder tracker
        self.encoder.tracker.reset() if hasattr(self.encoder.tracker, 'reset') else None

        # Reset GT detector if present
        if self._gt_detector is not None:
            self._gt_detector.clear_world_objects()

    def close(self):
        """Cleanup resources."""
        if self._viewport_camera is not None:
            self._viewport_camera.close()
            self._viewport_camera = None

        if self._detection_logger is not None:
            self._detection_logger.close()
            self._detection_logger = None


def create_dual_mode_perception(
    mode: str = "gt",
    camera_fov: float = 90.0,
    **kwargs
) -> DualModePerception:
    """
    Factory function for creating DualModePerception with common settings.

    Args:
        mode: "gt" for training, "full" for E2E validation
        camera_fov: Horizontal field of view in degrees
        **kwargs: Additional DualModeConfig parameters

    Returns:
        Configured DualModePerception instance
    """
    config = DualModeConfig(
        mode=mode,
        camera_params=CameraParams(fov_horizontal=camera_fov),
        **kwargs
    )
    return DualModePerception(config)
