"""
Main perception encoder combining all components.

Produces a fixed-size observation vector for RL training:
- Priority detections (top-10): 100 dims
- Spatial grid (8x8x6): 384 dims
- Scene statistics: 32 dims
- Total: 516 dims

This ensures NO detection information is lost while maintaining
a fixed tensor size for neural network input.

DUAL-MODE ARCHITECTURE:
The encoder accepts Detection objects from ANY source:
- Training Mode: GroundTruthDetector (frustum math, no rendering)
- E2E Mode: YOLODetector (real inference on rendered images)

Both produce identical Detection dataclass, so the encoder logic is shared.
The mode switch happens at the environment level, not here.
"""
import numpy as np
from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass
import time

try:
    from .detector import YOLODetector, Detection
    from .spatial_encoder import SpatialGridEncoder, SpatialGridConfig
    from .scene_statistics import SceneStatistics, SceneStatisticsConfig
except ImportError:
    from detector import YOLODetector, Detection
    from spatial_encoder import SpatialGridEncoder, SpatialGridConfig
    from scene_statistics import SceneStatistics, SceneStatisticsConfig

# Note: TemporalTracker has been deprecated in favor of YOLODetector's
# built-in ByteTrack tracking (via model.track(persist=True))


# Formally specified output dimension
OUTPUT_DIM = 516


@dataclass
class PerceptionConfig:
    """Configuration for perception encoder."""
    # Detector (only used by legacy encode() method)
    detector_mode: str = "ground_truth"  # "ground_truth", "inference", "hybrid"
    detector_config_path: Optional[str] = None

    # Priority detections
    max_priority_detections: int = 10
    detection_feature_dim: int = 10  # Features per detection

    # Spatial grid
    grid_size: Tuple[int, int] = (8, 8)
    grid_channels: int = 6

    # Scene statistics
    person_danger_distance: float = 15.0

    # Tracker
    tracker_iou_threshold: float = 0.3
    tracker_max_age: int = 10


class PerceptionEncoder:
    """
    Main perception encoder for RL observation space.

    Combines:
    1. Spatial grid encoder
    2. Scene statistics
    3. Temporal tracker

    Output dimensions:
    - Priority detections: 10 x 10 = 100
    - Spatial grid: 8 x 8 x 6 = 384
    - Scene statistics: 32
    - TOTAL: 516 dims

    USAGE (Dual-Mode):
        # Training Mode (ground truth, no rendering)
        gt_detector = GroundTruthDetector(config)
        detections = gt_detector.detect(camera_pose, uav_position)
        obs = encoder.encode_from_detections(detections, uav_position)

        # E2E Mode (real inference)
        yolo_detector = YOLODetector(mode="inference")
        detections = yolo_detector.detect(image, uav_position=uav_position)
        obs = encoder.encode_from_detections(detections, uav_position)

    Both modes produce identical 516-dim observation vectors.
    """

    # Class constant for observation dimension
    OUTPUT_DIM = 516

    def __init__(self, config: PerceptionConfig = None):
        self.config = config or PerceptionConfig()

        # Detector is optional - only needed for legacy encode() method
        self._detector = None

        self.spatial_encoder = SpatialGridEncoder(
            SpatialGridConfig(
                grid_size=self.config.grid_size,
                channels=self.config.grid_channels,
            )
        )

        self.scene_stats = SceneStatistics(
            SceneStatisticsConfig(
                person_danger_distance=self.config.person_danger_distance,
            )
        )

        # Note: External tracker removed - YOLODetector now handles tracking
        # via built-in ByteTrack (model.track(persist=True))

        # Calculate output dimensions
        self.priority_dim = self.config.max_priority_detections * self.config.detection_feature_dim
        self.grid_dim = self.spatial_encoder.output_dim
        self.stats_dim = self.scene_stats.output_dim
        self.output_dim = self.priority_dim + self.grid_dim + self.stats_dim

        assert self.output_dim == OUTPUT_DIM, f"Output dim mismatch: {self.output_dim} != {OUTPUT_DIM}"

        # Performance tracking
        self._encode_times = []

    @property
    def detector(self) -> YOLODetector:
        """Lazy initialization of detector for legacy encode() method."""
        if self._detector is None:
            self._detector = YOLODetector(
                config_path=self.config.detector_config_path,
                mode=self.config.detector_mode,
            )
        return self._detector

    def encode_from_detections(
        self,
        detections: List[Detection],
        uav_position: np.ndarray,
        dt: float = 0.05,
    ) -> np.ndarray:
        """
        Encode detections into 516-dim observation vector.

        This is the PRIMARY encoding method for dual-mode operation.
        It accepts Detection objects from ANY source (GT or inference).

        Args:
            detections: List of Detection objects from any detector
            uav_position: UAV world position (x, y, z) for relative coords
            dt: Time since last frame (for tracking)

        Returns:
            516-dimensional perception observation vector (np.float32)
        """
        start_time = time.perf_counter()

        # Note: Tracking is now handled by YOLODetector's built-in ByteTrack
        # Detections already have track_id set if tracking was enabled

        # 1. Encode priority detections (top-10)
        priority_obs = self._encode_priority_detections(detections, uav_position)

        # 2. Encode spatial grid
        grid_obs = self.spatial_encoder.encode(detections)

        # 3. Compute scene statistics
        stats_obs = self.scene_stats.compute(detections)

        # Tracking stats are now computed from detections directly
        tracked_count = sum(1 for d in detections if d.track_id is not None)
        avg_conf = np.mean([d.confidence for d in detections]) if detections else 0.0
        stats_obs[25] = 0.0  # lost_tracks - not available without external tracker
        stats_obs[29] = avg_conf

        # Track encoding time
        elapsed = (time.perf_counter() - start_time) * 1000
        self._encode_times.append(elapsed)
        if len(self._encode_times) > 100:
            self._encode_times.pop(0)
        stats_obs[31] = elapsed / 50.0  # Normalize to 50ms budget

        # Concatenate all observations
        observation = np.concatenate([priority_obs, grid_obs, stats_obs])

        return observation.astype(np.float32)

    def encode(
        self,
        image: np.ndarray,
        uav_position: np.ndarray,
        uav_orientation: np.ndarray,
        ground_truth_labels: Optional[List[Dict]] = None,
        dt: float = 0.05,
    ) -> np.ndarray:
        """
        Encode camera image into perception observation.

        LEGACY METHOD: This runs detection internally.
        For dual-mode operation, use encode_from_detections() instead.

        Args:
            image: RGB image (H, W, 3) uint8
            uav_position: UAV world position (x, y, z)
            uav_orientation: UAV orientation quaternion (unused, kept for API compat)
            ground_truth_labels: Isaac Sim labels (for training)
            dt: Time since last frame

        Returns:
            Perception observation vector (516 dims)
        """
        # Run detection using internal detector
        detections = self.detector.detect(
            image,
            ground_truth_labels=ground_truth_labels,
            uav_position=uav_position,
        )

        return self.encode_from_detections(detections, uav_position, dt=dt)

    def _encode_priority_detections(
        self,
        detections: List[Detection],
        uav_position: np.ndarray
    ) -> np.ndarray:
        """
        Encode top-10 priority detections.

        Each detection: 10 features
        - class_id (normalized)
        - confidence
        - rel_x, rel_y, rel_z (relative position)
        - distance (normalized)
        - bearing (normalized)
        - velocity_x, velocity_y (if tracked)
        - bbox_size
        """
        obs = np.zeros(self.priority_dim, dtype=np.float32)

        # Sort by priority (already done in detector, but ensure)
        sorted_dets = sorted(
            detections,
            key=lambda d: (d.priority, -d.confidence)
        )[:self.config.max_priority_detections]

        for i, det in enumerate(sorted_dets):
            idx = i * self.config.detection_feature_dim

            # Class (normalized to 0-1)
            obs[idx + 0] = det.class_id / 10.0

            # Confidence
            obs[idx + 1] = det.confidence

            # Relative position
            if det.position_3d is not None:
                rel_pos = det.position_3d - uav_position
                obs[idx + 2] = np.clip(rel_pos[0] / 100.0, -1, 1)
                obs[idx + 3] = np.clip(rel_pos[1] / 100.0, -1, 1)
                obs[idx + 4] = np.clip(rel_pos[2] / 50.0, -1, 1)
            else:
                # Estimate from bbox
                obs[idx + 2] = (det.bbox[0] - 0.5) * 2
                obs[idx + 3] = (det.bbox[1] - 0.5) * 2
                obs[idx + 4] = -0.5  # Assume below

            # Distance (normalized to 200m max)
            obs[idx + 5] = np.clip(det.distance / 200.0, 0, 1)

            # Bearing (normalized to -1, 1)
            obs[idx + 6] = det.bearing / np.pi

            # Velocity (if tracked)
            if det.velocity is not None:
                obs[idx + 7] = np.clip(det.velocity[0] / 10.0, -1, 1)
                obs[idx + 8] = np.clip(det.velocity[1] / 10.0, -1, 1)

            # Bbox size
            obs[idx + 9] = det.bbox[2] * det.bbox[3]

        return obs

    def get_detections_for_tptp(self, image: np.ndarray, uav_position: np.ndarray) -> List[Detection]:
        """
        Get raw detections for TPTP fact generation.

        Returns ALL detections (not just top-10) for Vampire safety checking.
        """
        return self.detector.detect(image, uav_position=uav_position)

    @property
    def avg_encode_time_ms(self) -> float:
        """Average encoding time in milliseconds."""
        if not self._encode_times:
            return 0.0
        return sum(self._encode_times) / len(self._encode_times)

    def get_observation_space_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get observation space bounds for Gymnasium."""
        low = np.zeros(self.output_dim, dtype=np.float32)
        high = np.ones(self.output_dim, dtype=np.float32)

        # Priority detections can have negative values (relative positions, bearing)
        for i in range(self.config.max_priority_detections):
            idx = i * self.config.detection_feature_dim
            low[idx + 2:idx + 5] = -1.0  # rel_x, rel_y, rel_z
            low[idx + 6] = -1.0  # bearing
            low[idx + 7:idx + 9] = -1.0  # velocity

        return low, high
