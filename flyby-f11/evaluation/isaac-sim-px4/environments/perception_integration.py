"""
Perception Integration for RL Training Environments

This module integrates the perception pipeline (YOLO detection, spatial encoding,
TPTP generation) directly into the RL training loop. It captures camera images
and runs real-time inference to provide realistic perception observations.

The perception observation is a 516-dimensional vector:
- 100 dims: Priority detections (top 20 objects × 5 features)
- 384 dims: Spatial grid (8×8×6 channels)
- 32 dims: Scene statistics

This is designed for sim-to-real transfer: the agent learns from actual camera
inference, not ground truth labels, so it experiences the same perception
characteristics (noise, failures, latency) as deployment.

Modes:
- "inference": Real YOLO inference on camera images (realistic, slower)
- "ground_truth": Uses known object positions (fast, for debugging)
- "hybrid": Both inference and ground truth (for supervised pretraining)
"""

import sys
import time
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple

# Add perception to path
sys.path.insert(0, '/workspace/perception')

# Try to import perception modules
try:
    from perception_encoder import PerceptionEncoder, PerceptionConfig
    from detector import YOLODetector, Detection
    from tptp_generator import TPTPGenerator
    PERCEPTION_AVAILABLE = True
except ImportError as e:
    print(f"[PerceptionIntegration] Warning: Perception modules not available: {e}")
    PERCEPTION_AVAILABLE = False


@dataclass
class PerceptionIntegrationConfig:
    """Configuration for perception integration in RL environments."""

    # Detection mode
    mode: str = "hybrid"  # "inference", "ground_truth", or "hybrid"

    # Camera settings
    camera_resolution: Tuple[int, int] = (640, 480)
    camera_fps: int = 30

    # Performance settings
    max_encode_time_ms: float = 20.0  # Max time budget for encoding
    skip_frames: int = 0  # Skip N frames between perception updates (0 = every frame)

    # YOLO settings
    yolo_model: str = "yolov8n.pt"  # YOLOv8 nano for speed
    yolo_confidence: float = 0.25
    yolo_classes: List[int] = field(default_factory=lambda: [0, 2, 7])  # person, car, truck

    # Ground truth settings
    detection_range: float = 150.0  # Max detection range in meters
    fov_degrees: float = 90.0  # Field of view

    # Observation settings
    include_in_observation: bool = True  # Add perception obs to RL observation
    observation_dim: int = 516  # 100 + 384 + 32


class PerceptionIntegration:
    """
    Integrates perception pipeline with Isaac Sim RL environments.

    This class provides real-time perception observations from the drone's
    camera during RL training. It can operate in three modes:

    1. "inference": Runs YOLO on each camera frame (realistic but slower)
    2. "ground_truth": Uses known object positions (fast, for debugging)
    3. "hybrid": Combines both (useful for supervised pretraining)

    Usage in BaseISREnvironment:
        self.perception = PerceptionIntegration(config)
        self.perception.initialize(camera)

        # In step():
        image = self.perception.capture_image()
        perception_obs = self.perception.encode(image, uav_state)

        # Add to observation
        full_obs = np.concatenate([state_obs, perception_obs])
    """

    def __init__(self, config: PerceptionIntegrationConfig):
        self.config = config
        self._camera = None
        self._encoder = None
        self._tptp_gen = None
        self._detector = None
        self._initialized = False

        # Known objects for ground truth mode
        self._known_people: List[np.ndarray] = []
        self._known_vehicles: List[np.ndarray] = []
        self._known_buildings: List[np.ndarray] = []

        # Statistics
        self._frame_count = 0
        self._total_encode_time = 0.0
        self._detection_counts = {"person": 0, "vehicle": 0, "building": 0}
        self._danger_alerts = 0
        self._skip_counter = 0
        self._last_observation = None  # Cache for skipped frames

        # Warning tracking
        self._warmup_complete = False
        self._warmup_frames = 0

    def initialize(self, camera=None) -> bool:
        """
        Initialize the perception pipeline.

        Args:
            camera: Isaac Sim Camera object (optional if using ground truth only)

        Returns:
            True if initialization successful
        """
        if not PERCEPTION_AVAILABLE:
            print("[Perception] WARNING: Perception modules not available")
            print("[Perception] Returning zero observation vectors")
            self._initialized = True
            return True

        self._camera = camera

        # Initialize perception encoder
        try:
            perception_config = PerceptionConfig(
                detector_mode=self.config.mode,
                yolo_model=self.config.yolo_model,
                confidence_threshold=self.config.yolo_confidence,
            )
            self._encoder = PerceptionEncoder(perception_config)
            self._tptp_gen = TPTPGenerator()

            print(f"[Perception] Initialized in '{self.config.mode}' mode")
            print(f"[Perception] Output dimension: {self._encoder.output_dim}")
            print(f"[Perception] Camera: {'Attached' if camera else 'None (ground truth only)'}")

            self._initialized = True
            return True

        except Exception as e:
            print(f"[Perception] ERROR during initialization: {e}")
            self._initialized = True  # Continue without perception
            return False

    def set_known_objects(
        self,
        people: List[np.ndarray] = None,
        vehicles: List[np.ndarray] = None,
        buildings: List[np.ndarray] = None
    ):
        """
        Set known object positions for ground truth mode.

        Args:
            people: List of [x, y, z] positions for people
            vehicles: List of [x, y, z] positions for vehicles
            buildings: List of [x, y, z] positions for buildings
        """
        self._known_people = people or []
        self._known_vehicles = vehicles or []
        self._known_buildings = buildings or []

    def capture_image(self) -> Optional[np.ndarray]:
        """
        Capture RGB image from camera.

        Returns:
            RGB image as numpy array (H, W, 3) or None if not available
        """
        if self._camera is None:
            return None

        # Handle warmup
        if not self._warmup_complete:
            self._warmup_frames += 1
            if self._warmup_frames >= 60:  # 1 second at 60fps
                self._warmup_complete = True
            return None

        try:
            rgba = self._camera.get_rgba()
            if rgba is None or rgba.max() < 5:
                return None
            return rgba[:, :, :3]  # RGB only
        except Exception as e:
            return None

    def _generate_ground_truth_labels(
        self,
        uav_position: np.ndarray
    ) -> List[Dict[str, Any]]:
        """
        Generate ground truth detection labels based on known object positions.

        Args:
            uav_position: Current UAV position [x, y, z]

        Returns:
            List of label dictionaries
        """
        labels = []

        # Helper to project 3D position to image coordinates
        def project_to_image(pos: np.ndarray) -> Optional[Tuple[float, float, float]]:
            rel = pos - uav_position
            dist = np.linalg.norm(rel)

            # Check range
            if dist > self.config.detection_range:
                return None

            # Simple projection (assumes camera pointing down)
            # Real implementation would use camera intrinsics/extrinsics
            img_x = 0.5 + rel[0] / (dist + 10) * 0.3
            img_y = 0.5 + rel[1] / (dist + 10) * 0.3

            # Check FOV
            if not (0.05 < img_x < 0.95 and 0.05 < img_y < 0.95):
                return None

            return img_x, img_y, dist

        # Process people
        for pos in self._known_people:
            result = project_to_image(pos)
            if result:
                img_x, img_y, dist = result
                labels.append({
                    'class_id': 1,
                    'bbox': (img_x, img_y, 0.05, 0.1),
                    'position_3d': pos.astype(np.float32),
                })

        # Process vehicles
        for pos in self._known_vehicles:
            result = project_to_image(pos)
            if result:
                img_x, img_y, dist = result
                labels.append({
                    'class_id': 2,
                    'bbox': (img_x, img_y, 0.1, 0.08),
                    'position_3d': pos.astype(np.float32),
                })

        # Process buildings
        for pos in self._known_buildings:
            result = project_to_image(pos)
            if result:
                img_x, img_y, dist = result
                labels.append({
                    'class_id': 3,
                    'bbox': (img_x, img_y, 0.2, 0.3),
                    'position_3d': pos.astype(np.float32),
                })

        return labels

    def encode(
        self,
        image: Optional[np.ndarray],
        uav_position: np.ndarray,
        uav_orientation: np.ndarray = None
    ) -> np.ndarray:
        """
        Encode perception observation from camera image.

        Args:
            image: RGB image from camera (H, W, 3) or None
            uav_position: UAV position [x, y, z]
            uav_orientation: UAV orientation as quaternion [w, x, y, z]

        Returns:
            516-dimensional perception observation vector
        """
        # Handle frame skipping
        if self.config.skip_frames > 0:
            self._skip_counter += 1
            if self._skip_counter <= self.config.skip_frames:
                if self._last_observation is not None:
                    return self._last_observation
                else:
                    return np.zeros(self.config.observation_dim, dtype=np.float32)
            self._skip_counter = 0

        # Return zeros if not initialized or perception unavailable
        if not self._initialized or self._encoder is None:
            return np.zeros(self.config.observation_dim, dtype=np.float32)

        # Use dummy image if none provided
        if image is None:
            image = np.zeros((480, 640, 3), dtype=np.uint8)

        if uav_orientation is None:
            uav_orientation = np.array([1.0, 0.0, 0.0, 0.0])

        # Generate ground truth labels for hybrid/ground_truth mode
        gt_labels = None
        if self.config.mode in ["ground_truth", "hybrid"]:
            gt_labels = self._generate_ground_truth_labels(uav_position)

        # Encode perception
        start = time.perf_counter()
        try:
            obs = self._encoder.encode(
                image=image,
                uav_position=uav_position,
                uav_orientation=uav_orientation,
                ground_truth_labels=gt_labels,
            )
        except Exception as e:
            print(f"[Perception] Encoding error: {e}")
            obs = np.zeros(self.config.observation_dim, dtype=np.float32)

        encode_time = (time.perf_counter() - start) * 1000

        # Update statistics
        self._frame_count += 1
        self._total_encode_time += encode_time

        # Get raw detections for stats
        if self._encoder is not None and hasattr(self._encoder, 'detector'):
            try:
                detections = self._encoder.detector.detect(
                    image, ground_truth_labels=gt_labels, uav_position=uav_position
                )
                for det in detections:
                    if det.class_id == 1:
                        self._detection_counts["person"] += 1
                        if det.distance < 15:
                            self._danger_alerts += 1
                    elif det.class_id == 2:
                        self._detection_counts["vehicle"] += 1
                    elif det.class_id == 3:
                        self._detection_counts["building"] += 1
            except:
                pass

        # Cache observation for skipped frames
        self._last_observation = obs

        return obs

    def get_tptp_facts(
        self,
        image: Optional[np.ndarray],
        uav_position: np.ndarray
    ) -> List[str]:
        """
        Generate TPTP facts for Vampire ATP safety checking.

        Args:
            image: RGB image from camera
            uav_position: UAV position [x, y, z]

        Returns:
            List of TPTP fact strings
        """
        if self._tptp_gen is None or self._encoder is None:
            return []

        try:
            gt_labels = None
            if self.config.mode in ["ground_truth", "hybrid"]:
                gt_labels = self._generate_ground_truth_labels(uav_position)

            if image is None:
                image = np.zeros((480, 640, 3), dtype=np.uint8)

            detections = self._encoder.detector.detect(
                image, ground_truth_labels=gt_labels, uav_position=uav_position
            )
            return self._tptp_gen.generate_facts(detections)
        except Exception as e:
            print(f"[Perception] TPTP generation error: {e}")
            return []

    def get_danger_zone_count(
        self,
        image: Optional[np.ndarray],
        uav_position: np.ndarray
    ) -> int:
        """
        Get count of people within danger zone (15m).

        Args:
            image: RGB image from camera
            uav_position: UAV position [x, y, z]

        Returns:
            Number of people within 15m
        """
        if self._encoder is None:
            return 0

        try:
            gt_labels = None
            if self.config.mode in ["ground_truth", "hybrid"]:
                gt_labels = self._generate_ground_truth_labels(uav_position)

            if image is None:
                image = np.zeros((480, 640, 3), dtype=np.uint8)

            detections = self._encoder.detector.detect(
                image, ground_truth_labels=gt_labels, uav_position=uav_position
            )
            return sum(1 for d in detections if d.class_id == 1 and d.distance < 15)
        except:
            return 0

    def get_stats(self) -> Dict[str, Any]:
        """Get perception statistics."""
        avg_encode = self._total_encode_time / max(1, self._frame_count)
        return {
            "perception_mode": self.config.mode,
            "frames_processed": self._frame_count,
            "avg_encode_time_ms": avg_encode,
            "encode_within_budget": avg_encode < self.config.max_encode_time_ms,
            "detection_counts": self._detection_counts.copy(),
            "danger_zone_alerts": self._danger_alerts,
            "camera_attached": self._camera is not None,
            "warmup_complete": self._warmup_complete,
        }

    def reset(self):
        """Reset perception state for new episode."""
        self._frame_count = 0
        self._total_encode_time = 0.0
        self._detection_counts = {"person": 0, "vehicle": 0, "building": 0}
        self._danger_alerts = 0
        self._skip_counter = 0
        self._last_observation = None
        self._warmup_complete = False
        self._warmup_frames = 0


def create_perception_observation_space(
    base_obs_dim: int,
    perception_config: PerceptionIntegrationConfig
) -> int:
    """
    Calculate total observation dimension with perception.

    Args:
        base_obs_dim: Dimension of base UAV state observation
        perception_config: Perception configuration

    Returns:
        Total observation dimension
    """
    if perception_config.include_in_observation:
        return base_obs_dim + perception_config.observation_dim
    return base_obs_dim


def combine_observations(
    state_obs: np.ndarray,
    perception_obs: np.ndarray,
    include_perception: bool = True
) -> np.ndarray:
    """
    Combine UAV state observation with perception observation.

    Args:
        state_obs: UAV state observation vector
        perception_obs: Perception observation vector (516 dims)
        include_perception: Whether to include perception

    Returns:
        Combined observation vector
    """
    if include_perception:
        return np.concatenate([state_obs, perception_obs]).astype(np.float32)
    return state_obs.astype(np.float32)
