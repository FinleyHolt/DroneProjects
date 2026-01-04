"""
YOLO detector wrapper for Isaac Sim and real cameras.

Supports:
- Ground truth labels from Isaac Sim (training)
- YOLOv11 inference with built-in ByteTrack tracking (deployment)
- TensorRT acceleration (Jetson)

Uses Ultralytics model.track() for temporal tracking instead of
custom tracker - this provides Kalman filtering, motion prediction,
and better occlusion handling.
"""
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
from pathlib import Path
import time

# Conditional import for yaml
try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# Conditional import for Ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


@dataclass
class Detection:
    """Single detection from YOLO or ground truth.

    Bbox formats:
    - For YOLO inference: xyxy pixel coordinates (x1, y1, x2, y2)
    - For ground truth: normalized center (cx, cy, w, h) - convert before use

    The bbox_xyxy property provides pixel coordinates for visualization.
    The bbox_normalized property provides normalized coords for encoding.
    """
    class_id: int
    class_name: str
    ontology_class: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # xyxy pixel coords (YOLO) or normalized (GT)
    position_3d: Optional[np.ndarray] = None  # World frame position
    distance: float = 0.0
    bearing: float = 0.0
    velocity: Optional[np.ndarray] = None
    priority: int = 10
    safety_distance: float = 0.0
    track_id: Optional[int] = None
    _image_size: Optional[Tuple[int, int]] = field(default=None, repr=False)  # (width, height)

    def get_bbox_xyxy(self, image_width: int = None, image_height: int = None) -> Tuple[float, float, float, float]:
        """Get bbox in xyxy pixel coordinates for visualization.

        Args:
            image_width: Image width (uses stored size if None)
            image_height: Image height (uses stored size if None)

        Returns:
            (x1, y1, x2, y2) pixel coordinates, clamped to image bounds
        """
        if self.bbox is None:
            return (0, 0, 0, 0)

        w = image_width or (self._image_size[0] if self._image_size else 640)
        h = image_height or (self._image_size[1] if self._image_size else 480)

        x1, y1, x2, y2 = self.bbox

        # Check if already in pixel coords (values > 1 suggest pixels)
        if max(x1, y1, x2, y2) > 1.5:
            # Already pixel coords, just clamp
            x1 = max(0, min(x1, w))
            y1 = max(0, min(y1, h))
            x2 = max(0, min(x2, w))
            y2 = max(0, min(y2, h))
        else:
            # Normalized coords - convert to pixels
            # Assume it's in center format (cx, cy, w, h)
            cx, cy, bw, bh = self.bbox
            x1 = max(0, (cx - bw / 2) * w)
            y1 = max(0, (cy - bh / 2) * h)
            x2 = min(w, (cx + bw / 2) * w)
            y2 = min(h, (cy + bh / 2) * h)

        # Ensure x1 < x2 and y1 < y2
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1

        return (x1, y1, x2, y2)

    def get_bbox_normalized(self, image_width: int = None, image_height: int = None) -> Tuple[float, float, float, float]:
        """Get bbox in normalized center format (cx, cy, w, h) for encoding.

        Returns:
            (cx, cy, w, h) normalized to 0-1 range
        """
        if self.bbox is None:
            return (0.5, 0.5, 0.0, 0.0)

        w = image_width or (self._image_size[0] if self._image_size else 640)
        h = image_height or (self._image_size[1] if self._image_size else 480)

        # Check if in pixel coords
        if max(self.bbox) > 1.5:
            # xyxy pixel coords - convert to normalized center
            x1, y1, x2, y2 = self.bbox
            cx = ((x1 + x2) / 2) / w
            cy = ((y1 + y2) / 2) / h
            bw = abs(x2 - x1) / w
            bh = abs(y2 - y1) / h
            return (
                max(0, min(1, cx)),
                max(0, min(1, cy)),
                max(0, min(1, bw)),
                max(0, min(1, bh)),
            )
        else:
            # Already normalized (assume center format)
            return self.bbox


class YOLODetector:
    """
    YOLO detector for UAV perception with built-in ByteTrack tracking.

    Modes:
    - ground_truth: Use Isaac Sim semantic labels (perfect detections)
    - inference: Run YOLOv11 with tracking on RGB images
    - hybrid: Use GT for training, inference for eval

    Tracking is built-in via model.track() which uses ByteTrack by default.
    This provides:
    - Kalman filtering for motion prediction
    - Better association across occlusions
    - Persistent track IDs across frames
    """

    def __init__(
        self,
        config_path: Optional[str] = None,
        mode: str = "ground_truth",
        enable_tracking: bool = True,
    ):
        self.mode = mode
        self.enable_tracking = enable_tracking
        self.config = self._load_config(config_path)
        self.class_mapping = self.config.get('detection_classes', {})
        self.coco_mapping = self.config.get('coco_to_ontology', {})

        # Initialize YOLO model if needed
        self.model = None
        if mode in ["inference", "hybrid"] and YOLO_AVAILABLE:
            model_config = self.config.get('model', {})
            model_path = model_config.get('weights', 'yolo11x.pt')
            self.model = YOLO(model_path)
            self.confidence_threshold = model_config.get('confidence_threshold', 0.4)
            self.nms_threshold = model_config.get('nms_iou_threshold', 0.45)
            tracking_str = "with ByteTrack" if enable_tracking else "detection only"
            print(f"[YOLODetector] Loaded {model_path} in {mode} mode ({tracking_str})")
        elif mode == "inference":
            print("[YOLODetector] Warning: ultralytics not available, using mock detections")

        # Performance tracking
        self._inference_times = []

        # Track statistics (populated by tracking)
        self.tracking_stats = {
            'active_tracks': 0,
            'new_tracks': 0,
            'lost_tracks': 0,
        }

    def _load_config(self, config_path: Optional[str]) -> Dict:
        """Load detection configuration."""
        if config_path and Path(config_path).exists() and YAML_AVAILABLE:
            with open(config_path) as f:
                return yaml.safe_load(f)
        return self._default_config()

    def _default_config(self) -> Dict:
        """Default configuration if file not found.

        COCO class IDs for ISR targets:
        - 0: person
        - 2: car
        - 5: bus
        - 7: truck
        """
        return {
            'detection_classes': {
                1: {'name': 'person', 'ontology_class': 'Person', 'priority': 1, 'safety_distance': 15.0},
                2: {'name': 'car', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
                3: {'name': 'building', 'ontology_class': 'StaticObstacle', 'priority': 3, 'safety_distance': 5.0},
                4: {'name': 'poi_target', 'ontology_class': 'TargetOfInterest', 'priority': 4, 'safety_distance': 0.0},
                5: {'name': 'bus', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
                6: {'name': 'landing_zone', 'ontology_class': 'LandingZone', 'priority': 5, 'safety_distance': 0.0},
                7: {'name': 'truck', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
            },
            # COCO class ID -> internal class ID
            # person (0) -> 1, car (2) -> 2, bus (5) -> 5, truck (7) -> 7
            'coco_to_ontology': {0: 1, 2: 2, 5: 5, 7: 7},
            'model': {'confidence_threshold': 0.25, 'weights': 'yolo11x.pt'}
        }

    def detect(
        self,
        image: np.ndarray,
        ground_truth_labels: Optional[List[Dict]] = None,
        uav_position: Optional[np.ndarray] = None,
    ) -> List[Detection]:
        """
        Run detection on image.

        Args:
            image: RGB image (H, W, 3) uint8
            ground_truth_labels: Isaac Sim semantic labels (for training)
            uav_position: UAV world position for distance calculation

        Returns:
            List of Detection objects
        """
        start_time = time.perf_counter()

        if self.mode == "ground_truth" and ground_truth_labels is not None:
            detections = self._process_ground_truth(ground_truth_labels)
        elif self.model is not None:
            detections = self._run_yolo_inference(image)
        else:
            detections = self._mock_detections()

        # Calculate distances if UAV position provided
        if uav_position is not None:
            for det in detections:
                if det.position_3d is not None:
                    det.distance = float(np.linalg.norm(det.position_3d - uav_position))
                    # Calculate bearing (angle from UAV heading)
                    rel_pos = det.position_3d - uav_position
                    det.bearing = float(np.arctan2(rel_pos[1], rel_pos[0]))

        # Sort by priority (safety-critical first)
        detections.sort(key=lambda d: (d.priority, -d.confidence))

        # Track inference time
        elapsed = (time.perf_counter() - start_time) * 1000
        self._inference_times.append(elapsed)
        if len(self._inference_times) > 100:
            self._inference_times.pop(0)

        return detections

    def _run_yolo_inference(self, image: np.ndarray) -> List[Detection]:
        """Run YOLO inference with optional ByteTrack tracking.

        When tracking is enabled, uses model.track() which provides:
        - Persistent track IDs across frames
        - Kalman filtering for motion prediction
        - ByteTrack association algorithm

        Returns detections with bbox in xyxy pixel coordinates.
        Use Detection.get_bbox_normalized() for encoding.
        """
        img_h, img_w = image.shape[:2]

        # Use track() for temporal consistency, or predict() for single-frame
        if self.enable_tracking:
            results = self.model.track(
                image,
                verbose=False,
                conf=self.confidence_threshold,
                iou=self.nms_threshold,
                persist=True,  # Keep track IDs across frames
                tracker="bytetrack.yaml",  # Use ByteTrack (default)
            )
        else:
            results = self.model(
                image,
                verbose=False,
                conf=self.confidence_threshold,
                iou=self.nms_threshold
            )

        detections = []
        track_ids_seen = set()

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for i, box in enumerate(boxes):
                coco_class = int(box.cls[0])
                ontology_class_id = self.coco_mapping.get(coco_class)

                if ontology_class_id is None:
                    continue  # Skip unmapped classes

                class_config = self.class_mapping.get(ontology_class_id, {})

                # Get xyxy pixel coordinates (NOT normalized)
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = xyxy

                # Clamp to image bounds
                x1 = max(0, min(float(x1), img_w))
                y1 = max(0, min(float(y1), img_h))
                x2 = max(0, min(float(x2), img_w))
                y2 = max(0, min(float(y2), img_h))

                if x1 >= x2 or y1 >= y2:
                    continue

                bbox = (x1, y1, x2, y2)

                # Get track ID if tracking is enabled
                track_id = None
                if self.enable_tracking and box.id is not None:
                    track_id = int(box.id[0])
                    track_ids_seen.add(track_id)

                detections.append(Detection(
                    class_id=ontology_class_id,
                    class_name=class_config.get('name', 'unknown'),
                    ontology_class=class_config.get('ontology_class', 'Object'),
                    confidence=float(box.conf[0]),
                    bbox=bbox,
                    priority=class_config.get('priority', 10),
                    safety_distance=class_config.get('safety_distance', 0.0),
                    track_id=track_id,
                    _image_size=(img_w, img_h),
                ))

        # Update tracking stats
        self.tracking_stats['active_tracks'] = len(track_ids_seen)

        return detections

    def _process_ground_truth(self, labels: List[Dict]) -> List[Detection]:
        """Process Isaac Sim ground truth labels."""
        detections = []
        for label in labels:
            class_id = label.get('semantic_id', label.get('class_id', 0))
            class_config = self.class_mapping.get(class_id, {})

            detections.append(Detection(
                class_id=class_id,
                class_name=class_config.get('name', 'unknown'),
                ontology_class=class_config.get('ontology_class', 'Object'),
                confidence=1.0,  # Ground truth = perfect confidence
                bbox=tuple(label.get('bbox', (0.5, 0.5, 0.1, 0.1))),
                position_3d=np.array(label['position_3d']) if 'position_3d' in label else None,
                priority=class_config.get('priority', 10),
                safety_distance=class_config.get('safety_distance', 0.0),
            ))
        return detections

    def _mock_detections(self) -> List[Detection]:
        """Generate mock detections for testing."""
        import random
        detections = []

        # Random chance of detections
        if random.random() < 0.3:
            detections.append(Detection(
                class_id=1,
                class_name='person',
                ontology_class='Person',
                confidence=random.uniform(0.5, 0.95),
                bbox=(random.uniform(0.2, 0.8), random.uniform(0.2, 0.8), 0.1, 0.2),
                priority=1,
                safety_distance=15.0,
            ))
        if random.random() < 0.5:
            detections.append(Detection(
                class_id=3,
                class_name='building',
                ontology_class='StaticObstacle',
                confidence=random.uniform(0.6, 0.98),
                bbox=(random.uniform(0.1, 0.9), random.uniform(0.1, 0.9), 0.15, 0.25),
                priority=3,
                safety_distance=5.0,
            ))
        return detections

    def reset_tracking(self):
        """Reset tracking state between episodes.

        Call this at the start of each new episode to clear track history.
        """
        if self.model is not None and self.enable_tracking:
            # Reset the tracker by calling track with a blank image
            # Ultralytics resets on new persist=True sequence
            self.model.predictor = None  # Force re-initialization
        self.tracking_stats = {
            'active_tracks': 0,
            'new_tracks': 0,
            'lost_tracks': 0,
        }

    def get_tracking_stats(self) -> Dict:
        """Get current tracking statistics.

        Returns dict with:
        - active_tracks: Number of currently tracked objects
        - new_tracks: New tracks this frame (if available)
        - lost_tracks: Lost tracks this frame (if available)
        """
        return self.tracking_stats.copy()

    @property
    def avg_inference_time_ms(self) -> float:
        """Average inference time in milliseconds."""
        if not self._inference_times:
            return 0.0
        return sum(self._inference_times) / len(self._inference_times)
