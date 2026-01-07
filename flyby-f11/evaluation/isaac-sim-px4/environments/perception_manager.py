"""
Perception Manager - Ground truth and detection integration using Isaac Sim native annotators.

Uses omni.replicator's bounding_box_2d_tight annotator for pixel-perfect ground truth
bounding boxes computed directly by the renderer.

Supports two perception modes:
1. Ground Truth (GT) Mode: Uses native Isaac Sim annotators for perfect annotations.
   - Pixel-perfect bounding boxes from renderer
   - Semantic labels from USD prims
   - No projection math required

2. Full Inference Mode: Runs YOLO detection on camera images.
   - Realistic perception with noise, occlusions, missed detections
   - Required for sim-to-real validation

Both modes produce identical output formats for seamless switching.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import numpy as np


# Military-oriented class mapping for ISR detection
MILITARY_CLASS_MAP = {
    "person": 0,
    "civilian_person": 0,
    "military_person": 1,
    "civilian_vehicle": 2,
    "military_vehicle": 3,
    "tank": 4,
    "apc": 5,
    "ifv": 5,
    "technical": 6,
    "truck": 7,
    "emplacement": 8,
    "building": 9,
}

# Classes to filter out from GT detections (not targets of interest)
FILTERED_CLASSES = {"tree", "vegetation", "bush", "grass", "terrain"}

CLASS_ID_TO_NAME = {
    0: "person",
    1: "military_person",
    2: "civilian_vehicle",
    3: "military_vehicle",
    4: "tank",
    5: "armored_vehicle",
    6: "technical",
    7: "truck",
    8: "emplacement",
    9: "building",
}


@dataclass
class Detection:
    """Single detection result (works for both GT and inference)."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # x_center, y_center, width, height (normalized 0-1)
    world_position: Optional[np.ndarray] = None
    prim_path: Optional[str] = None
    track_id: Optional[int] = None
    distance: Optional[float] = None
    occlusion_ratio: float = 0.0


@dataclass
class GroundTruthConfig:
    """Configuration for ground truth detection mode."""
    # Noise injection for sim-to-real transfer
    bbox_noise_std: float = 0.0
    confidence_noise_std: float = 0.0
    position_noise_std: float = 0.0

    # Detection simulation
    detection_probability: float = 1.0
    max_detection_range: float = 500.0
    min_bbox_size: float = 0.005  # Minimum bbox dimension (normalized)
    max_occlusion: float = 0.9  # Skip objects more than 90% occluded

    # Semantic filtering
    semantic_types: List[str] = field(default_factory=lambda: ["class", "prim"])


@dataclass
class PerceptionConfig:
    """Configuration for perception system."""
    enabled: bool = True
    mode: str = "gt"  # "gt" or "full"

    # Camera settings
    camera_resolution: Tuple[int, int] = (640, 480)
    camera_prim_path: str = "/World/isr_camera"

    # Ground truth settings
    gt_bbox_noise_std: float = 0.0
    gt_confidence_noise_std: float = 0.0
    gt_position_noise_std: float = 0.0

    # Performance
    max_encode_time_ms: float = 20.0
    skip_frames: int = 0
    observation_dim: int = 516


class PerceptionManager:
    """
    Manages perception using Isaac Sim's native annotators.

    Uses bounding_box_2d_tight annotator for pixel-perfect ground truth.
    """

    OUTPUT_DIM = 516

    def __init__(self, config: PerceptionConfig):
        self.config = config
        self._mode = config.mode
        self._annotator = None
        self._render_product = None
        self._initialized = False
        self._frame_count = 0

        # Cache for world positions (from USD stage queries)
        self._prim_positions: Dict[str, np.ndarray] = {}
        self._camera_position: np.ndarray = np.zeros(3)

    def initialize(
        self,
        camera_prim_path: str = None,
        render_product=None,
    ) -> bool:
        """
        Initialize the native annotator.

        Args:
            camera_prim_path: Path to camera prim (uses config default if None)
            render_product: Existing render product (created if None)

        Returns:
            True if initialization successful
        """
        if not self.config.enabled:
            return False

        try:
            import omni.replicator.core as rep

            camera_path = camera_prim_path or self.config.camera_prim_path

            # Create or use existing render product
            if render_product is not None:
                self._render_product = render_product
            else:
                self._render_product = rep.create.render_product(
                    camera_path,
                    self.config.camera_resolution
                )

            # Create bounding box annotator
            self._annotator = rep.AnnotatorRegistry.get_annotator(
                "bounding_box_2d_tight",
                init_params={"semanticTypes": ["class", "prim"]}
            )
            self._annotator.attach(self._render_product)

            self._initialized = True
            print(f"[PerceptionManager] Initialized with native annotator on {camera_path}")
            return True

        except ImportError as e:
            print(f"[PerceptionManager] omni.replicator not available: {e}")
            return False
        except Exception as e:
            print(f"[PerceptionManager] Initialization failed: {e}")
            return False

    def get_ground_truth_detections(
        self,
        uav_position: np.ndarray = None,
        gt_config: GroundTruthConfig = None,
        stage=None,
    ) -> List[Detection]:
        """
        Get ground truth detections using native Isaac Sim annotator.

        Args:
            uav_position: UAV position for distance calculation
            gt_config: Ground truth configuration
            stage: USD stage for world position queries

        Returns:
            List of Detection objects with pixel-perfect bboxes
        """
        if not self._initialized or self._annotator is None:
            return []

        if gt_config is None:
            gt_config = GroundTruthConfig(
                bbox_noise_std=self.config.gt_bbox_noise_std,
                confidence_noise_std=self.config.gt_confidence_noise_std,
                position_noise_std=self.config.gt_position_noise_std,
            )

        if uav_position is not None:
            self._camera_position = uav_position

        try:
            # Get bbox data from annotator
            data = self._annotator.get_data()

            if data is None:
                return []

            bbox_data = data.get("data")
            if bbox_data is None or len(bbox_data) == 0:
                return []

            info = data.get("info", {})
            prim_paths = info.get("primPaths", [])
            id_to_labels = info.get("idToLabels", {})

            img_w, img_h = self.config.camera_resolution
            detections = []

            for i, bbox in enumerate(bbox_data):
                detection = self._process_bbox(
                    bbox, i, prim_paths, id_to_labels,
                    img_w, img_h, gt_config, stage
                )
                if detection is not None:
                    detections.append(detection)

            self._frame_count += 1

            # Debug logging every 100 frames
            if self._frame_count % 100 == 1:
                print(f"[PerceptionManager] Frame {self._frame_count}: {len(detections)} GT detections")

            return detections

        except Exception as e:
            if self._frame_count % 100 == 0:
                print(f"[PerceptionManager] Error getting GT: {e}")
            return []

    def _process_bbox(
        self,
        bbox,
        idx: int,
        prim_paths: List[str],
        id_to_labels: Dict,
        img_w: int,
        img_h: int,
        gt_config: GroundTruthConfig,
        stage,
    ) -> Optional[Detection]:
        """Process a single bbox from the annotator."""
        try:
            # Handle both dict and numpy structured array (numpy.void) formats
            if hasattr(bbox, 'get'):
                # Dict format
                x_min = int(bbox["x_min"])
                y_min = int(bbox["y_min"])
                x_max = int(bbox["x_max"])
                y_max = int(bbox["y_max"])
                occlusion = float(bbox.get("occlusionRatio", 0.0))
                sem_id = bbox.get("semanticId", 0)
            else:
                # Numpy structured array format (numpy.void record)
                x_min = int(bbox["x_min"])
                y_min = int(bbox["y_min"])
                x_max = int(bbox["x_max"])
                y_max = int(bbox["y_max"])
                # Use try/except for optional fields in structured array
                try:
                    occlusion = float(bbox["occlusionRatio"])
                except (KeyError, ValueError):
                    occlusion = 0.0
                try:
                    sem_id = int(bbox["semanticId"])
                except (KeyError, ValueError):
                    sem_id = 0
        except Exception as e:
            print(f"[PerceptionManager] Error parsing bbox: {e}, type={type(bbox)}")
            return None

        # Skip heavily occluded objects
        if occlusion > gt_config.max_occlusion:
            return None

        # Convert to normalized center format
        cx = (x_min + x_max) / 2 / img_w
        cy = (y_min + y_max) / 2 / img_h
        w = (x_max - x_min) / img_w
        h = (y_max - y_min) / img_h

        # Skip tiny bboxes
        if w < gt_config.min_bbox_size or h < gt_config.min_bbox_size:
            return None

        # Get semantic info (sem_id already extracted above)
        labels = id_to_labels.get(str(sem_id), {})
        prim_path = prim_paths[idx] if idx < len(prim_paths) else None

        # Determine class from labels or prim path
        class_name, class_id = self._classify_from_labels(labels, prim_path)

        # Skip filtered classes (trees, vegetation, etc.)
        if class_name is None:
            return None

        # Get world position if stage available
        world_pos = None
        distance = None
        if stage is not None and prim_path:
            world_pos = self._get_prim_world_position(stage, prim_path)
            if world_pos is not None:
                distance = float(np.linalg.norm(world_pos - self._camera_position))

                # Skip if beyond max range
                if distance > gt_config.max_detection_range:
                    return None

        # Simulate detection probability
        if gt_config.detection_probability < 1.0:
            if np.random.random() > gt_config.detection_probability:
                return None

        # Compute confidence (inverse of occlusion)
        confidence = 1.0 - occlusion

        # Apply noise if configured
        bbox_tuple = (cx, cy, w, h)
        if gt_config.bbox_noise_std > 0:
            bbox_tuple = self._add_bbox_noise(bbox_tuple, gt_config.bbox_noise_std)
        if gt_config.confidence_noise_std > 0:
            confidence = np.clip(
                confidence + np.random.normal(0, gt_config.confidence_noise_std),
                0.1, 1.0
            )
        if gt_config.position_noise_std > 0 and world_pos is not None:
            world_pos = world_pos + np.random.normal(0, gt_config.position_noise_std, 3)

        return Detection(
            class_id=class_id,
            class_name=class_name,
            confidence=confidence,
            bbox=bbox_tuple,
            world_position=world_pos.astype(np.float32) if world_pos is not None else None,
            prim_path=prim_path,
            distance=distance,
            occlusion_ratio=occlusion,
        )

    def _classify_from_labels(
        self,
        labels: Dict,
        prim_path: Optional[str],
    ) -> Tuple[Optional[str], int]:
        """
        Determine class from semantic labels or prim path.

        Returns:
            Tuple of (class_name, class_id). Returns (None, -1) for filtered classes.
        """
        # Try semantic labels first
        class_label = labels.get("class", "").lower()
        if class_label:
            # Filter out non-target classes
            if class_label in FILTERED_CLASSES:
                return None, -1
            if class_label in MILITARY_CLASS_MAP:
                return class_label, MILITARY_CLASS_MAP[class_label]

        # Fall back to prim path analysis
        if prim_path:
            path_lower = prim_path.lower()
            name = prim_path.split("/")[-1].lower()

            # Check for vehicles
            if "/vehicles/" in path_lower or "vehicle" in name:
                if "tank" in name:
                    return "tank", MILITARY_CLASS_MAP["tank"]
                elif any(t in name for t in ["apc", "ifv"]):
                    return "armored_vehicle", MILITARY_CLASS_MAP["apc"]
                elif any(t in name for t in ["military", "humvee", "mrap"]):
                    return "military_vehicle", MILITARY_CLASS_MAP["military_vehicle"]
                else:
                    return "civilian_vehicle", MILITARY_CLASS_MAP["civilian_vehicle"]

            # Check for people
            if "/people/" in path_lower or "person" in name:
                if any(t in name for t in ["military", "soldier"]):
                    return "military_person", MILITARY_CLASS_MAP["military_person"]
                else:
                    return "person", MILITARY_CLASS_MAP["person"]

        return "unknown", 0

    def _get_prim_world_position(
        self,
        stage,
        prim_path: str,
    ) -> Optional[np.ndarray]:
        """Get world position of a prim with caching."""

        # Check cache (refreshed each frame via _frame_count)
        cache_key = f"{self._frame_count}:{prim_path}"
        if cache_key in self._prim_positions:
            return self._prim_positions[cache_key]

        try:
            from pxr import UsdGeom

            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return None

            xformable = UsdGeom.Xformable(prim)
            transform = xformable.ComputeLocalToWorldTransform(0)
            translation = transform.ExtractTranslation()

            pos = np.array([translation[0], translation[1], translation[2]])
            self._prim_positions[cache_key] = pos

            # Clear old cache entries
            if len(self._prim_positions) > 1000:
                self._prim_positions.clear()

            return pos

        except Exception:
            return None

    def _add_bbox_noise(
        self,
        bbox: Tuple[float, float, float, float],
        noise_std: float,
    ) -> Tuple[float, float, float, float]:
        """Add Gaussian noise to bounding box."""
        noisy = np.array(bbox) + np.random.normal(0, noise_std, 4)
        noisy[0] = np.clip(noisy[0], 0, 1)  # cx
        noisy[1] = np.clip(noisy[1], 0, 1)  # cy
        noisy[2] = np.clip(noisy[2], 0.01, 1)  # w
        noisy[3] = np.clip(noisy[3], 0.01, 1)  # h
        return tuple(noisy)

    def get_detections(
        self,
        image: Optional[np.ndarray] = None,
        uav_position: np.ndarray = None,
        stage=None,
    ) -> List[Detection]:
        """
        Get detections using current mode.

        Args:
            image: Camera image (required for full inference mode)
            uav_position: UAV position
            stage: USD stage

        Returns:
            List of Detection objects
        """
        if self._mode == "gt":
            return self.get_ground_truth_detections(uav_position, stage=stage)
        else:
            return self._get_inference_detections(image, uav_position)

    def _get_inference_detections(
        self,
        image: Optional[np.ndarray],  # noqa: ARG002
        uav_position: np.ndarray,  # noqa: ARG002
    ) -> List[Detection]:
        """Get detections using YOLO inference (not yet implemented)."""
        return []

    def get_observation(
        self,
        uav_position: np.ndarray,  # noqa: ARG002
        uav_orientation: np.ndarray,  # noqa: ARG002
        world_objects: List = None,  # noqa: ARG002
    ) -> np.ndarray:
        """Get perception observation vector (placeholder for RL integration)."""
        return np.zeros(self.OUTPUT_DIM, dtype=np.float32)

    def get_stats(self) -> Dict[str, Any]:
        """Get perception statistics."""
        return {
            "mode": self._mode,
            "initialized": self._initialized,
            "frame_count": self._frame_count,
        }

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def enabled(self) -> bool:
        return self.config.enabled

    @property
    def initialized(self) -> bool:
        return self._initialized
