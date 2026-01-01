"""
YOLO detector wrapper for Isaac Sim and real cameras.

Supports:
- Ground truth labels from Isaac Sim (training)
- YOLOv11n inference (deployment)
- TensorRT acceleration (Jetson)
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
    """Single detection from YOLO or ground truth."""
    class_id: int
    class_name: str
    ontology_class: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # x_center, y_center, width, height (normalized)
    position_3d: Optional[np.ndarray] = None  # World frame position
    distance: float = 0.0
    bearing: float = 0.0
    velocity: Optional[np.ndarray] = None
    priority: int = 10
    safety_distance: float = 0.0
    track_id: Optional[int] = None


class YOLODetector:
    """
    YOLO detector for UAV perception.

    Modes:
    - ground_truth: Use Isaac Sim semantic labels (perfect detections)
    - inference: Run YOLOv11n on RGB images
    - hybrid: Use GT for training, inference for eval
    """

    def __init__(
        self,
        config_path: Optional[str] = None,
        mode: str = "ground_truth",
    ):
        self.mode = mode
        self.config = self._load_config(config_path)
        self.class_mapping = self.config.get('detection_classes', {})
        self.coco_mapping = self.config.get('coco_to_ontology', {})

        # Initialize YOLO model if needed
        self.model = None
        if mode in ["inference", "hybrid"] and YOLO_AVAILABLE:
            model_config = self.config.get('model', {})
            model_path = model_config.get('weights', 'yolo11n.pt')
            self.model = YOLO(model_path)
            self.confidence_threshold = model_config.get('confidence_threshold', 0.4)
            self.nms_threshold = model_config.get('nms_iou_threshold', 0.45)
            print(f"[YOLODetector] Loaded {model_path} in {mode} mode")
        elif mode == "inference":
            print("[YOLODetector] Warning: ultralytics not available, using mock detections")

        # Performance tracking
        self._inference_times = []

    def _load_config(self, config_path: Optional[str]) -> Dict:
        """Load detection configuration."""
        if config_path and Path(config_path).exists() and YAML_AVAILABLE:
            with open(config_path) as f:
                return yaml.safe_load(f)
        return self._default_config()

    def _default_config(self) -> Dict:
        """Default configuration if file not found."""
        return {
            'detection_classes': {
                1: {'name': 'person', 'ontology_class': 'Person', 'priority': 1, 'safety_distance': 15.0},
                2: {'name': 'vehicle', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
                3: {'name': 'building', 'ontology_class': 'StaticObstacle', 'priority': 3, 'safety_distance': 5.0},
                4: {'name': 'poi_target', 'ontology_class': 'TargetOfInterest', 'priority': 4, 'safety_distance': 0.0},
                5: {'name': 'landing_zone', 'ontology_class': 'LandingZone', 'priority': 5, 'safety_distance': 0.0},
            },
            'coco_to_ontology': {0: 1, 2: 2, 7: 2},
            'model': {'confidence_threshold': 0.4}
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
        """Run YOLOv11n inference."""
        results = self.model(
            image,
            verbose=False,
            conf=self.confidence_threshold,
            iou=self.nms_threshold
        )

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for box in boxes:
                coco_class = int(box.cls[0])
                ontology_class_id = self.coco_mapping.get(coco_class)

                if ontology_class_id is None:
                    continue  # Skip unmapped classes

                class_config = self.class_mapping.get(ontology_class_id, {})

                # Get normalized bbox (center x, center y, width, height)
                xyxy = box.xyxyn[0].cpu().numpy()
                bbox = (
                    (xyxy[0] + xyxy[2]) / 2,  # center x
                    (xyxy[1] + xyxy[3]) / 2,  # center y
                    xyxy[2] - xyxy[0],         # width
                    xyxy[3] - xyxy[1],         # height
                )

                detections.append(Detection(
                    class_id=ontology_class_id,
                    class_name=class_config.get('name', 'unknown'),
                    ontology_class=class_config.get('ontology_class', 'Object'),
                    confidence=float(box.conf[0]),
                    bbox=bbox,
                    priority=class_config.get('priority', 10),
                    safety_distance=class_config.get('safety_distance', 0.0),
                ))

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

    @property
    def avg_inference_time_ms(self) -> float:
        """Average inference time in milliseconds."""
        if not self._inference_times:
            return 0.0
        return sum(self._inference_times) / len(self._inference_times)
