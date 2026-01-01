# Phase 6f: Production Perception Pipeline

## Overview

Implement a production-ready perception pipeline that converts camera images into structured observations for RL training and deployment. This pipeline must handle real-world complexity (crowds, convoys, complex scenes) while running within the 50ms control loop budget alongside Vampire ATP.

## Human Description

The current RL environments use geometric footprint models for coverage and POI detection. For real-world deployment, we need actual visual perception:

1. **Object Detection** - YOLOv11n for people, vehicles, buildings, POIs, landing zones
2. **Spatial Encoding** - 8×8 grid capturing detection density across the FOV
3. **Scene Statistics** - Aggregate metrics for safety and mission awareness
4. **Temporal Tracking** - Multi-frame consistency for reliable detections
5. **TPTP Integration** - Convert ALL detections to Vampire facts for safety checking

### Why This Design?

| Requirement | Solution |
|-------------|----------|
| No detection limit | Spatial grid captures ALL objects |
| Fixed RL input size | Grid + stats = 400 dims (constant) |
| Real-time (20Hz) | YOLOv11n @ 12ms + grid @ 2ms = 14ms |
| Vampire headroom | 14ms perception + 25ms Vampire = 39ms < 50ms |
| Sim-to-real | Same pipeline runs in Isaac Sim and on Jetson |

### Compute Budget (Jetson Orin NX 16GB)

| Component | VRAM | Latency | Notes |
|-----------|------|---------|-------|
| YOLOv11n (TensorRT FP16) | 500 MB | 10-12ms | From MODEL_CONFIGURATIONS.md |
| Spatial Grid Encoder | ~50 MB | 2ms | NumPy operations |
| Scene Statistics | ~10 MB | 1ms | NumPy aggregations |
| Temporal Tracker | ~50 MB | 1ms | Deque + matching |
| Vampire ATP | ~50 MB | 25ms | Safety queries |
| RL Agent (SAC) | ~50 MB | 5ms | MLP inference |
| **TOTAL** | **~710 MB** | **~46ms** | Fits 20Hz loop |

## AI Agent Instructions

### Prerequisites
- Phase 6e completed (RL training bridge working)
- Isaac Sim container with camera topics publishing
- Understanding of YOLOv11/Ultralytics API
- Familiarity with Gymnasium observation spaces

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. Create Perception Package Structure

**Location:** `evaluation/isaac-sim-px4/perception/`

```
evaluation/isaac-sim-px4/perception/
├── __init__.py
├── detector.py              # YOLO wrapper
├── spatial_encoder.py       # 8×8 grid encoding
├── scene_statistics.py      # Aggregate metrics
├── temporal_tracker.py      # Multi-frame tracking
├── perception_encoder.py    # Main encoder (combines all)
├── tptp_generator.py        # Detection → TPTP facts
├── config/
│   ├── detection_classes.yaml
│   └── perception_config.yaml
└── tests/
    ├── test_detector.py
    ├── test_spatial_encoder.py
    ├── test_perception_encoder.py
    └── test_tptp_generator.py
```

#### 2. Define Detection Classes

**File:** `perception/config/detection_classes.yaml`

```yaml
# Detection classes for ISR missions
# Matches ontology concepts in uav_domain.kif and isr_extensions.kif

detection_classes:
  # Class ID → Ontology mapping
  # Priority determines sorting order (lower = higher priority)

  1:  # SAFETY CRITICAL - Always highest priority
    name: "person"
    ontology_class: "Person"
    priority: 1
    safety_distance: 15.0  # meters (from ontology minPersonDistance)
    color: [255, 0, 0]     # red

  2:
    name: "vehicle"
    ontology_class: "DynamicObstacle"
    priority: 2
    safety_distance: 10.0
    color: [255, 165, 0]   # orange

  3:
    name: "building"
    ontology_class: "StaticObstacle"
    priority: 3
    safety_distance: 5.0
    color: [128, 128, 128] # gray

  4:
    name: "poi_target"
    ontology_class: "TargetOfInterest"
    priority: 4
    safety_distance: 0.0   # We want to approach these
    color: [0, 255, 0]     # green

  5:
    name: "landing_zone"
    ontology_class: "LandingZone"
    priority: 5
    safety_distance: 0.0
    color: [0, 255, 255]   # cyan

  6:
    name: "tree"
    ontology_class: "StaticObstacle"
    priority: 6
    safety_distance: 3.0
    color: [0, 100, 0]     # dark green

  7:
    name: "power_line"
    ontology_class: "StaticObstacle"
    priority: 2            # High priority - hard to see
    safety_distance: 10.0
    color: [255, 255, 0]   # yellow

  8:
    name: "water"
    ontology_class: "WaterSurface"
    priority: 7
    safety_distance: 0.0   # Terrain, not obstacle
    color: [0, 0, 255]     # blue

# COCO class mapping (for pretrained YOLO)
coco_to_ontology:
  0: 1    # person → person
  1: 2    # bicycle → vehicle
  2: 2    # car → vehicle
  3: 2    # motorcycle → vehicle
  5: 2    # bus → vehicle
  7: 2    # truck → vehicle
  # Note: buildings, POIs, landing zones need custom training
  # For Isaac Sim, use ground truth labels instead

# Model configuration
model:
  name: "yolo11n"
  weights: "yolo11n.pt"      # Or .engine for TensorRT
  input_size: [640, 480]
  confidence_threshold: 0.4
  nms_iou_threshold: 0.45
  device: "cuda:0"
  half_precision: true       # FP16 for Jetson
```

#### 3. Implement YOLO Detector Wrapper

**File:** `perception/detector.py`

```python
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
import yaml
import time

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
        if config_path and Path(config_path).exists():
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
```

#### 4. Implement Spatial Grid Encoder

**File:** `perception/spatial_encoder.py`

```python
"""
Spatial grid encoder for perception observations.

Encodes ALL detections into an 8×8 spatial grid, ensuring no information loss
regardless of detection count. Essential for real-world scenarios with crowds,
convoys, or complex urban environments.
"""
import numpy as np
from typing import List, Tuple
from dataclasses import dataclass

from .detector import Detection


@dataclass
class SpatialGridConfig:
    """Configuration for spatial grid encoding."""
    grid_size: Tuple[int, int] = (8, 8)
    channels: int = 6  # person_count, obstacle_count, nearest_dist, motion_x, motion_y, max_priority
    max_count_per_cell: int = 10  # For normalization
    max_distance: float = 100.0   # meters


class SpatialGridEncoder:
    """
    Encodes detections into fixed-size spatial grid.

    Grid layout (8×8 cells, 6 channels each = 384 dims):
    - Channel 0: Person count (normalized)
    - Channel 1: Obstacle count (vehicles + buildings + trees)
    - Channel 2: Nearest detection distance (inverted: 0=far, 1=close)
    - Channel 3: Average motion vector X
    - Channel 4: Average motion vector Y
    - Channel 5: Highest priority in cell (1=person, lower=safer)

    This ensures ALL detections contribute to the observation, even with
    100+ objects in frame.
    """

    def __init__(self, config: SpatialGridConfig = None):
        self.config = config or SpatialGridConfig()
        self.grid_h, self.grid_w = self.config.grid_size
        self.channels = self.config.channels

        # Output dimensions: 8 × 8 × 6 = 384
        self.output_dim = self.grid_h * self.grid_w * self.channels

    def encode(self, detections: List[Detection]) -> np.ndarray:
        """
        Encode detections into spatial grid.

        Args:
            detections: List of Detection objects with bbox (normalized 0-1)

        Returns:
            Flat numpy array of shape (384,) with values in [0, 1]
        """
        # Initialize grid: (H, W, C)
        grid = np.zeros((self.grid_h, self.grid_w, self.channels), dtype=np.float32)

        # Track per-cell statistics
        cell_distances = [[[] for _ in range(self.grid_w)] for _ in range(self.grid_h)]
        cell_motions = [[[] for _ in range(self.grid_w)] for _ in range(self.grid_h)]
        cell_priorities = [[[] for _ in range(self.grid_w)] for _ in range(self.grid_h)]

        for det in detections:
            # Map bbox center to grid cell
            cx, cy = det.bbox[0], det.bbox[1]
            cell_x = min(int(cx * self.grid_w), self.grid_w - 1)
            cell_y = min(int(cy * self.grid_h), self.grid_h - 1)

            # Channel 0: Person count
            if det.class_id == 1:  # person
                grid[cell_y, cell_x, 0] += 1

            # Channel 1: Obstacle count (vehicles, buildings, trees, power lines)
            if det.class_id in [2, 3, 6, 7]:
                grid[cell_y, cell_x, 1] += 1

            # Track distance for this cell
            cell_distances[cell_y][cell_x].append(det.distance)

            # Track motion if available
            if det.velocity is not None:
                cell_motions[cell_y][cell_x].append(det.velocity[:2])

            # Track priority
            cell_priorities[cell_y][cell_x].append(det.priority)

        # Normalize counts
        grid[:, :, 0] = np.clip(grid[:, :, 0] / self.config.max_count_per_cell, 0, 1)
        grid[:, :, 1] = np.clip(grid[:, :, 1] / self.config.max_count_per_cell, 0, 1)

        # Compute per-cell aggregates
        for y in range(self.grid_h):
            for x in range(self.grid_w):
                # Channel 2: Nearest distance (inverted: 1 = close/dangerous)
                if cell_distances[y][x]:
                    min_dist = min(cell_distances[y][x])
                    grid[y, x, 2] = 1.0 - np.clip(min_dist / self.config.max_distance, 0, 1)

                # Channels 3-4: Average motion vector
                if cell_motions[y][x]:
                    avg_motion = np.mean(cell_motions[y][x], axis=0)
                    grid[y, x, 3] = np.clip(avg_motion[0] / 10.0, -1, 1)  # Normalize to ±10 m/s
                    grid[y, x, 4] = np.clip(avg_motion[1] / 10.0, -1, 1)

                # Channel 5: Highest priority (lowest number = most important)
                if cell_priorities[y][x]:
                    min_priority = min(cell_priorities[y][x])
                    # Invert so 1.0 = highest priority (person), 0.0 = lowest
                    grid[y, x, 5] = 1.0 - (min_priority - 1) / 10.0

        # Flatten to 1D vector
        return grid.flatten()

    def decode(self, flat_grid: np.ndarray) -> np.ndarray:
        """
        Decode flat vector back to spatial grid (for visualization).

        Args:
            flat_grid: Array of shape (384,)

        Returns:
            Grid of shape (8, 8, 6)
        """
        return flat_grid.reshape(self.grid_h, self.grid_w, self.channels)

    def get_danger_cells(self, flat_grid: np.ndarray, threshold: float = 0.5) -> List[Tuple[int, int]]:
        """
        Get grid cells with high danger (persons nearby).

        Returns list of (row, col) tuples.
        """
        grid = self.decode(flat_grid)
        danger_cells = []

        for y in range(self.grid_h):
            for x in range(self.grid_w):
                # High person count or close proximity
                if grid[y, x, 0] > 0.1 or grid[y, x, 2] > threshold:
                    danger_cells.append((y, x))

        return danger_cells
```

#### 5. Implement Scene Statistics

**File:** `perception/scene_statistics.py`

```python
"""
Scene-level statistics from detections.

Provides aggregate metrics that don't depend on individual detection positions:
- Counts by class
- Nearest distances for safety
- Collision risk scoring
- Mission-relevant metrics
"""
import numpy as np
from typing import List, Dict
from dataclasses import dataclass

from .detector import Detection


@dataclass
class SceneStatisticsConfig:
    """Configuration for scene statistics."""
    person_danger_distance: float = 15.0    # meters (ontology: minPersonDistance)
    person_warning_distance: float = 30.0   # meters
    obstacle_danger_distance: float = 5.0   # meters
    max_detection_count: int = 50           # For normalization


class SceneStatistics:
    """
    Computes scene-level aggregate statistics.

    Output: 32-dimensional vector covering:
    - Class counts (8 dims)
    - Safety metrics (8 dims)
    - Mission metrics (8 dims)
    - Temporal metrics (8 dims)
    """

    def __init__(self, config: SceneStatisticsConfig = None):
        self.config = config or SceneStatisticsConfig()
        self.output_dim = 32

        # History for temporal metrics
        self._prev_detection_count = 0
        self._frames_since_person = 0
        self._frames_since_obstacle = 0
        self._frames_since_poi = 0

    def compute(self, detections: List[Detection]) -> np.ndarray:
        """
        Compute scene statistics from detections.

        Args:
            detections: List of Detection objects

        Returns:
            32-dim numpy array with values in [0, 1]
        """
        stats = np.zeros(32, dtype=np.float32)

        # Categorize detections
        persons = [d for d in detections if d.class_id == 1]
        vehicles = [d for d in detections if d.class_id == 2]
        buildings = [d for d in detections if d.class_id == 3]
        pois = [d for d in detections if d.class_id == 4]
        landing_zones = [d for d in detections if d.class_id == 5]
        obstacles = [d for d in detections if d.class_id in [2, 3, 6, 7]]

        # === Class Counts (8 dims) ===
        stats[0] = min(len(persons), self.config.max_detection_count) / self.config.max_detection_count
        stats[1] = min(len(vehicles), self.config.max_detection_count) / self.config.max_detection_count
        stats[2] = min(len(buildings), self.config.max_detection_count) / self.config.max_detection_count
        stats[3] = min(len(pois), 10) / 10.0
        stats[4] = min(len(landing_zones), 5) / 5.0
        stats[5] = min(len(obstacles), self.config.max_detection_count) / self.config.max_detection_count
        stats[6] = min(len(detections), self.config.max_detection_count * 2) / (self.config.max_detection_count * 2)
        stats[7] = self._compute_detection_density(detections)

        # === Safety Metrics (8 dims) ===
        stats[8] = self._nearest_distance_normalized(persons, self.config.person_danger_distance * 2)
        stats[9] = self._nearest_distance_normalized(obstacles, self.config.obstacle_danger_distance * 4)
        stats[10] = self._count_in_zone(persons, self.config.person_danger_distance) / 10.0
        stats[11] = self._count_in_zone(persons, self.config.person_warning_distance) / 20.0
        stats[12] = float(self._any_approaching(persons))
        stats[13] = self._fastest_approach_speed(detections) / 20.0  # Normalize to 20 m/s
        stats[14] = self._compute_collision_risk(detections)
        stats[15] = self._max_crowd_density(persons)

        # === Mission Metrics (8 dims) ===
        stats[16] = min(len(pois), 10) / 10.0  # POIs visible
        stats[17] = 0.0  # POIs captured (set externally)
        stats[18] = len([d for d in pois if d.distance < 50]) / 10.0  # Nearby targets
        stats[19] = min(len(landing_zones), 3) / 3.0
        stats[20] = float(self._path_ahead_clear(detections))
        stats[21] = self._scene_complexity(detections)
        stats[22] = self._avg_confidence(detections)
        stats[23] = 1.0  # Perception health (placeholder)

        # === Temporal Metrics (8 dims) ===
        stats[24] = min(abs(len(detections) - self._prev_detection_count), 20) / 20.0
        stats[25] = 0.0  # Lost tracks (set by tracker)
        stats[26] = self._scene_change_magnitude(detections)
        stats[27] = min(self._frames_since_person, 30) / 30.0
        stats[28] = min(self._frames_since_obstacle, 30) / 30.0
        stats[29] = 0.5  # Tracking confidence (set by tracker)
        stats[30] = 0.0  # Prediction error (set by tracker)
        stats[31] = 0.0  # Perception latency (set externally)

        # Update temporal state
        self._update_temporal_state(persons, obstacles, pois)
        self._prev_detection_count = len(detections)

        return stats

    def _nearest_distance_normalized(self, detections: List[Detection], max_dist: float) -> float:
        """Get normalized distance to nearest detection (0=close, 1=far)."""
        if not detections:
            return 1.0  # No detections = safe
        distances = [d.distance for d in detections if d.distance > 0]
        if not distances:
            return 0.5  # Unknown distance
        return np.clip(min(distances) / max_dist, 0, 1)

    def _count_in_zone(self, detections: List[Detection], radius: float) -> int:
        """Count detections within radius."""
        return sum(1 for d in detections if 0 < d.distance <= radius)

    def _any_approaching(self, detections: List[Detection]) -> bool:
        """Check if any detection is approaching (negative radial velocity)."""
        for d in detections:
            if d.velocity is not None:
                # Simple check: velocity toward UAV
                if d.position_3d is not None and d.distance > 0:
                    radial_vel = np.dot(d.velocity, -d.position_3d) / d.distance
                    if radial_vel > 1.0:  # Approaching at > 1 m/s
                        return True
        return False

    def _fastest_approach_speed(self, detections: List[Detection]) -> float:
        """Get fastest approach speed of any detection."""
        max_speed = 0.0
        for d in detections:
            if d.velocity is not None:
                speed = np.linalg.norm(d.velocity)
                max_speed = max(max_speed, speed)
        return max_speed

    def _compute_collision_risk(self, detections: List[Detection]) -> float:
        """Compute aggregate collision risk score."""
        if not detections:
            return 0.0

        risk = 0.0
        for d in detections:
            if d.distance > 0:
                # Risk inversely proportional to distance, weighted by priority
                dist_risk = 1.0 / (1.0 + d.distance / 10.0)
                priority_weight = 1.0 / d.priority
                risk += dist_risk * priority_weight

        return np.clip(risk / 10.0, 0, 1)  # Normalize

    def _max_crowd_density(self, persons: List[Detection]) -> float:
        """Estimate maximum crowd density in any area."""
        if len(persons) < 2:
            return len(persons) / 10.0

        # Simple clustering: count persons within 5m of each other
        max_cluster = 1
        for p1 in persons:
            cluster_size = 1
            for p2 in persons:
                if p1 is not p2 and p1.position_3d is not None and p2.position_3d is not None:
                    if np.linalg.norm(p1.position_3d - p2.position_3d) < 5.0:
                        cluster_size += 1
            max_cluster = max(max_cluster, cluster_size)

        return min(max_cluster, 20) / 20.0

    def _path_ahead_clear(self, detections: List[Detection]) -> bool:
        """Check if forward path is clear of obstacles."""
        for d in detections:
            # Check if detection is roughly ahead (bearing near 0)
            if abs(d.bearing) < 0.5 and d.distance < 20:
                return False
        return True

    def _scene_complexity(self, detections: List[Detection]) -> float:
        """Compute scene complexity (entropy of detection distribution)."""
        if not detections:
            return 0.0

        # Count unique classes
        class_counts = {}
        for d in detections:
            class_counts[d.class_id] = class_counts.get(d.class_id, 0) + 1

        # Compute entropy
        total = len(detections)
        entropy = 0.0
        for count in class_counts.values():
            p = count / total
            if p > 0:
                entropy -= p * np.log2(p)

        # Normalize to 0-1 (max entropy with 8 classes = 3 bits)
        return entropy / 3.0

    def _avg_confidence(self, detections: List[Detection]) -> float:
        """Average detection confidence."""
        if not detections:
            return 0.0
        return np.mean([d.confidence for d in detections])

    def _compute_detection_density(self, detections: List[Detection]) -> float:
        """Overall detection density."""
        return min(len(detections), 100) / 100.0

    def _scene_change_magnitude(self, detections: List[Detection]) -> float:
        """How much the scene changed from last frame."""
        change = abs(len(detections) - self._prev_detection_count)
        return min(change, 20) / 20.0

    def _update_temporal_state(
        self,
        persons: List[Detection],
        obstacles: List[Detection],
        pois: List[Detection]
    ):
        """Update frames-since counters."""
        if persons:
            self._frames_since_person = 0
        else:
            self._frames_since_person += 1

        if obstacles:
            self._frames_since_obstacle = 0
        else:
            self._frames_since_obstacle += 1

        if pois:
            self._frames_since_poi = 0
        else:
            self._frames_since_poi += 1
```

#### 6. Implement Temporal Tracker

**File:** `perception/temporal_tracker.py`

```python
"""
Multi-frame temporal tracking for detection consistency.

Provides:
- Track persistence across frames
- Velocity estimation from position history
- Track confidence scoring
- Lost track detection
"""
import numpy as np
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field
from collections import deque
import time

from .detector import Detection


@dataclass
class Track:
    """Single object track across frames."""
    track_id: int
    class_id: int
    class_name: str
    ontology_class: str

    # Current state
    position: Optional[np.ndarray] = None
    velocity: Optional[np.ndarray] = None
    bbox: Tuple[float, float, float, float] = (0.5, 0.5, 0.1, 0.1)

    # History
    position_history: deque = field(default_factory=lambda: deque(maxlen=10))
    bbox_history: deque = field(default_factory=lambda: deque(maxlen=10))

    # Tracking state
    confidence: float = 1.0
    age: int = 0  # Frames since track created
    hits: int = 1  # Frames with matching detection
    misses: int = 0  # Consecutive frames without match

    # Timestamps
    created_at: float = 0.0
    last_seen: float = 0.0


class TemporalTracker:
    """
    Simple IoU-based tracker for detection consistency.

    Matches detections across frames using bounding box IoU,
    estimates velocity from position history, and maintains
    track confidence scores.
    """

    def __init__(
        self,
        iou_threshold: float = 0.3,
        max_age: int = 10,  # Frames before track deleted
        min_hits: int = 2,  # Hits before track confirmed
    ):
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.min_hits = min_hits

        self.tracks: Dict[int, Track] = {}
        self._next_id = 0
        self._frame_time = time.time()

        # Metrics
        self.new_tracks_this_frame = 0
        self.lost_tracks_this_frame = 0

    def update(self, detections: List[Detection], dt: float = 0.05) -> List[Detection]:
        """
        Update tracks with new detections.

        Args:
            detections: New detections this frame
            dt: Time since last frame (seconds)

        Returns:
            Detections augmented with track IDs and velocities
        """
        current_time = time.time()
        self.new_tracks_this_frame = 0
        self.lost_tracks_this_frame = 0

        # Match detections to existing tracks
        matched_pairs, unmatched_dets, unmatched_tracks = self._match_detections(detections)

        # Update matched tracks
        for det_idx, track_id in matched_pairs:
            det = detections[det_idx]
            track = self.tracks[track_id]

            # Update track state
            track.bbox = det.bbox
            track.bbox_history.append(det.bbox)

            if det.position_3d is not None:
                # Estimate velocity from position change
                if track.position is not None:
                    track.velocity = (det.position_3d - track.position) / dt
                track.position = det.position_3d.copy()
                track.position_history.append(det.position_3d.copy())

            track.hits += 1
            track.misses = 0
            track.age += 1
            track.last_seen = current_time
            track.confidence = min(1.0, track.hits / (track.hits + track.misses + 1))

            # Augment detection with track info
            det.track_id = track_id
            det.velocity = track.velocity

        # Create new tracks for unmatched detections
        for det_idx in unmatched_dets:
            det = detections[det_idx]
            track_id = self._create_track(det, current_time)
            det.track_id = track_id
            self.new_tracks_this_frame += 1

        # Age unmatched tracks
        for track_id in unmatched_tracks:
            track = self.tracks[track_id]
            track.misses += 1
            track.age += 1
            track.confidence *= 0.9  # Decay confidence

            # Delete old tracks
            if track.misses > self.max_age:
                del self.tracks[track_id]
                self.lost_tracks_this_frame += 1

        return detections

    def _match_detections(
        self,
        detections: List[Detection]
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """Match detections to tracks using IoU."""
        if not detections or not self.tracks:
            return [], list(range(len(detections))), list(self.tracks.keys())

        # Compute IoU matrix
        n_dets = len(detections)
        n_tracks = len(self.tracks)
        track_ids = list(self.tracks.keys())

        iou_matrix = np.zeros((n_dets, n_tracks))
        for i, det in enumerate(detections):
            for j, track_id in enumerate(track_ids):
                track = self.tracks[track_id]
                # Only match same class
                if det.class_id == track.class_id:
                    iou_matrix[i, j] = self._compute_iou(det.bbox, track.bbox)

        # Greedy matching
        matched_pairs = []
        matched_dets = set()
        matched_tracks = set()

        while True:
            if iou_matrix.size == 0:
                break
            max_iou = iou_matrix.max()
            if max_iou < self.iou_threshold:
                break

            det_idx, track_idx = np.unravel_index(iou_matrix.argmax(), iou_matrix.shape)
            matched_pairs.append((det_idx, track_ids[track_idx]))
            matched_dets.add(det_idx)
            matched_tracks.add(track_ids[track_idx])

            # Remove matched from consideration
            iou_matrix[det_idx, :] = 0
            iou_matrix[:, track_idx] = 0

        unmatched_dets = [i for i in range(n_dets) if i not in matched_dets]
        unmatched_tracks = [tid for tid in track_ids if tid not in matched_tracks]

        return matched_pairs, unmatched_dets, unmatched_tracks

    def _compute_iou(
        self,
        bbox1: Tuple[float, float, float, float],
        bbox2: Tuple[float, float, float, float]
    ) -> float:
        """Compute IoU between two bboxes (center x, center y, width, height)."""
        # Convert to corners
        x1_min = bbox1[0] - bbox1[2] / 2
        x1_max = bbox1[0] + bbox1[2] / 2
        y1_min = bbox1[1] - bbox1[3] / 2
        y1_max = bbox1[1] + bbox1[3] / 2

        x2_min = bbox2[0] - bbox2[2] / 2
        x2_max = bbox2[0] + bbox2[2] / 2
        y2_min = bbox2[1] - bbox2[3] / 2
        y2_max = bbox2[1] + bbox2[3] / 2

        # Intersection
        inter_x = max(0, min(x1_max, x2_max) - max(x1_min, x2_min))
        inter_y = max(0, min(y1_max, y2_max) - max(y1_min, y2_min))
        inter_area = inter_x * inter_y

        # Union
        area1 = bbox1[2] * bbox1[3]
        area2 = bbox2[2] * bbox2[3]
        union_area = area1 + area2 - inter_area

        if union_area <= 0:
            return 0.0
        return inter_area / union_area

    def _create_track(self, det: Detection, timestamp: float) -> int:
        """Create new track from detection."""
        track_id = self._next_id
        self._next_id += 1

        self.tracks[track_id] = Track(
            track_id=track_id,
            class_id=det.class_id,
            class_name=det.class_name,
            ontology_class=det.ontology_class,
            position=det.position_3d.copy() if det.position_3d is not None else None,
            bbox=det.bbox,
            created_at=timestamp,
            last_seen=timestamp,
        )

        if det.position_3d is not None:
            self.tracks[track_id].position_history.append(det.position_3d.copy())
        self.tracks[track_id].bbox_history.append(det.bbox)

        return track_id

    def get_tracking_stats(self) -> Dict:
        """Get tracking statistics for observation."""
        active_tracks = len(self.tracks)
        confirmed_tracks = sum(1 for t in self.tracks.values() if t.hits >= self.min_hits)
        avg_confidence = np.mean([t.confidence for t in self.tracks.values()]) if self.tracks else 0.0

        return {
            'active_tracks': active_tracks,
            'confirmed_tracks': confirmed_tracks,
            'new_tracks': self.new_tracks_this_frame,
            'lost_tracks': self.lost_tracks_this_frame,
            'avg_confidence': avg_confidence,
        }
```

#### 7. Implement Main Perception Encoder

**File:** `perception/perception_encoder.py`

```python
"""
Main perception encoder combining all components.

Produces a fixed-size observation vector for RL training:
- Priority detections (top-10): 100 dims
- Spatial grid (8×8×6): 384 dims
- Scene statistics: 32 dims
- Total: 516 dims

This ensures NO detection information is lost while maintaining
a fixed tensor size for neural network input.
"""
import numpy as np
from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass
import time

from .detector import YOLODetector, Detection
from .spatial_encoder import SpatialGridEncoder, SpatialGridConfig
from .scene_statistics import SceneStatistics, SceneStatisticsConfig
from .temporal_tracker import TemporalTracker


@dataclass
class PerceptionConfig:
    """Configuration for perception encoder."""
    # Detector
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
    1. YOLO detector (or ground truth labels)
    2. Spatial grid encoder
    3. Scene statistics
    4. Temporal tracker

    Output dimensions:
    - Priority detections: 10 × 10 = 100
    - Spatial grid: 8 × 8 × 6 = 384
    - Scene statistics: 32
    - TOTAL: 516 dims
    """

    def __init__(self, config: PerceptionConfig = None):
        self.config = config or PerceptionConfig()

        # Initialize components
        self.detector = YOLODetector(
            config_path=self.config.detector_config_path,
            mode=self.config.detector_mode,
        )

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

        self.tracker = TemporalTracker(
            iou_threshold=self.config.tracker_iou_threshold,
            max_age=self.config.tracker_max_age,
        )

        # Calculate output dimensions
        self.priority_dim = self.config.max_priority_detections * self.config.detection_feature_dim
        self.grid_dim = self.spatial_encoder.output_dim
        self.stats_dim = self.scene_stats.output_dim
        self.output_dim = self.priority_dim + self.grid_dim + self.stats_dim

        # Performance tracking
        self._encode_times = []

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

        Args:
            image: RGB image (H, W, 3) uint8
            uav_position: UAV world position (x, y, z)
            uav_orientation: UAV orientation quaternion
            ground_truth_labels: Isaac Sim labels (for training)
            dt: Time since last frame

        Returns:
            Perception observation vector (516 dims)
        """
        start_time = time.perf_counter()

        # 1. Run detection
        detections = self.detector.detect(
            image,
            ground_truth_labels=ground_truth_labels,
            uav_position=uav_position,
        )

        # 2. Track detections
        detections = self.tracker.update(detections, dt=dt)

        # 3. Encode priority detections (top-10)
        priority_obs = self._encode_priority_detections(detections, uav_position)

        # 4. Encode spatial grid
        grid_obs = self.spatial_encoder.encode(detections)

        # 5. Compute scene statistics
        stats_obs = self.scene_stats.compute(detections)

        # Update tracking stats in scene statistics
        tracking_stats = self.tracker.get_tracking_stats()
        stats_obs[25] = tracking_stats['lost_tracks'] / 10.0
        stats_obs[29] = tracking_stats['avg_confidence']

        # Track encoding time
        elapsed = (time.perf_counter() - start_time) * 1000
        self._encode_times.append(elapsed)
        if len(self._encode_times) > 100:
            self._encode_times.pop(0)
        stats_obs[31] = elapsed / 50.0  # Normalize to 50ms budget

        # Concatenate all observations
        observation = np.concatenate([priority_obs, grid_obs, stats_obs])

        return observation.astype(np.float32)

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
```

#### 8. Implement TPTP Fact Generator

**File:** `perception/tptp_generator.py`

```python
"""
TPTP fact generator for Vampire theorem prover integration.

Converts ALL detections (not just top-10) into TPTP facts for safety checking.
This ensures Vampire can reason about every detected object in the scene.
"""
import numpy as np
from typing import List, Dict
from dataclasses import dataclass
import hashlib
from collections import deque
import time

from .detector import Detection


@dataclass
class TPTPConfig:
    """Configuration for TPTP fact generation."""
    max_facts_per_query: int = 100
    dedup_window_ms: float = 500.0
    include_velocity: bool = True
    include_tracking: bool = True


class TPTPGenerator:
    """
    Generates TPTP facts from detections for Vampire safety queries.

    Fact types generated:
    1. observedObject - Object detected with class and position
    2. distanceTo - Distance from UAV to object
    3. personDetected - Special fact for safety-critical person detections
    4. approachingObject - Object moving toward UAV
    5. crowdDensity - High density of persons in area
    """

    def __init__(self, config: TPTPConfig = None):
        self.config = config or TPTPConfig()
        self._fact_counter = 0
        self._recent_facts = deque(maxlen=1000)

    def generate_facts(
        self,
        detections: List[Detection],
        uav_id: str = "f11_uav",
        timestamp: float = None
    ) -> List[str]:
        """
        Generate TPTP facts from detections.

        Args:
            detections: All detections from perception
            uav_id: UAV identifier for facts
            timestamp: Current timestamp (for temporal facts)

        Returns:
            List of TPTP fact strings
        """
        if timestamp is None:
            timestamp = time.time()

        facts = []
        persons = []

        for det in detections:
            self._fact_counter += 1
            fact_id = self._fact_counter
            object_id = f"{det.class_name}_{fact_id}"

            # Skip if duplicate (same class + similar position recently)
            if self._is_duplicate(det):
                continue

            # 1. Basic observation fact
            if det.position_3d is not None:
                x, y, z = det.position_3d
                facts.append(
                    f"fof(obs_{fact_id}, axiom, "
                    f"observedObject({object_id}, {det.ontology_class}, "
                    f"position({x:.2f}, {y:.2f}, {z:.2f}), "
                    f"confidence({det.confidence:.3f}), "
                    f"time({timestamp:.3f})))."
                )

                # 2. Distance fact
                if det.distance > 0:
                    facts.append(
                        f"fof(dist_{fact_id}, axiom, "
                        f"distanceTo({uav_id}, {object_id}, {det.distance:.2f}))."
                    )

            # 3. Person-specific facts (safety critical)
            if det.class_id == 1:  # Person
                persons.append(det)
                facts.append(
                    f"fof(person_{fact_id}, axiom, "
                    f"(and "
                    f"(instance({object_id}, Person)) "
                    f"(distanceTo({uav_id}, {object_id}, {det.distance:.2f}))))."
                )

                # Check if person is in danger zone
                if det.distance < 15.0:  # minPersonDistance
                    facts.append(
                        f"fof(person_danger_{fact_id}, axiom, "
                        f"personInDangerZone({uav_id}, {object_id}))."
                    )

            # 4. Approaching object facts
            if self.config.include_velocity and det.velocity is not None:
                vel = det.velocity
                speed = np.linalg.norm(vel)

                if speed > 0.5:  # Moving > 0.5 m/s
                    facts.append(
                        f"fof(vel_{fact_id}, axiom, "
                        f"objectVelocity({object_id}, "
                        f"velocity({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f if len(vel) > 2 else 0:.2f})))."
                    )

                    # Check if approaching UAV
                    if det.position_3d is not None and det.distance > 0:
                        radial_vel = -np.dot(vel[:2], det.position_3d[:2]) / det.distance
                        if radial_vel > 1.0:  # Approaching at > 1 m/s
                            facts.append(
                                f"fof(approach_{fact_id}, axiom, "
                                f"approachingObject({object_id}, {uav_id}, {radial_vel:.2f}))."
                            )

            # 5. Track confidence facts
            if self.config.include_tracking and det.track_id is not None:
                facts.append(
                    f"fof(track_{fact_id}, axiom, "
                    f"trackedObject({object_id}, track_{det.track_id}))."
                )

        # 6. Crowd density fact
        if len(persons) >= 3:
            facts.append(
                f"fof(crowd_{self._fact_counter}, axiom, "
                f"crowdPresent({uav_id}, {len(persons)}))."
            )

        # Limit facts
        if len(facts) > self.config.max_facts_per_query:
            # Prioritize person-related facts
            person_facts = [f for f in facts if 'person' in f.lower() or 'Person' in f]
            other_facts = [f for f in facts if f not in person_facts]
            facts = person_facts + other_facts[:self.config.max_facts_per_query - len(person_facts)]

        return facts

    def generate_safety_query(
        self,
        detections: List[Detection],
        current_state: Dict,
        proposed_action: np.ndarray,
        uav_id: str = "f11_uav"
    ) -> str:
        """
        Generate complete TPTP safety query for Vampire.

        Args:
            detections: All detections
            current_state: Current UAV state dict
            proposed_action: Proposed action vector
            uav_id: UAV identifier

        Returns:
            Complete TPTP query string
        """
        # Generate perception facts
        perception_facts = self.generate_facts(detections, uav_id)

        # Add state facts
        state_facts = [
            f"fof(state_pos, axiom, "
            f"position({uav_id}, {current_state['x']:.2f}, {current_state['y']:.2f}, {current_state['z']:.2f})).",

            f"fof(state_battery, axiom, "
            f"batteryLevel({uav_id}, {current_state['battery']:.1f})).",

            f"fof(state_geofence, axiom, "
            f"{'inGeofence' if current_state.get('in_geofence', True) else 'outsideGeofence'}({uav_id})).",
        ]

        # Add predicted next state
        vx, vy, vz = proposed_action[0], proposed_action[1], proposed_action[2]
        dt = 0.05  # 20Hz control loop
        next_x = current_state['x'] + vx * dt
        next_y = current_state['y'] + vy * dt
        next_z = current_state['z'] + vz * dt

        state_facts.append(
            f"fof(next_pos, axiom, "
            f"nextPosition({uav_id}, {next_x:.2f}, {next_y:.2f}, {next_z:.2f}))."
        )

        # Safety query conjecture
        conjecture = """
% Query: Is the proposed action safe?
fof(safety_query, conjecture,
    ~(geofenceViolation(f11_uav) |
      noFlyZoneViolation(f11_uav) |
      personInDangerZone(f11_uav, _) |
      highThreatExposure(f11_uav))
).
"""

        # Combine all parts
        query = "% Perception facts\n"
        query += "\n".join(perception_facts)
        query += "\n\n% State facts\n"
        query += "\n".join(state_facts)
        query += "\n\n" + conjecture

        return query

    def _is_duplicate(self, det: Detection) -> bool:
        """Check if detection is duplicate of recent fact."""
        current_time = time.time() * 1000  # ms

        # Create hash of detection
        det_hash = hashlib.md5(
            f"{det.class_id}_{det.bbox[0]:.2f}_{det.bbox[1]:.2f}".encode()
        ).hexdigest()[:16]

        # Clean old entries
        while self._recent_facts and (current_time - self._recent_facts[0][0]) > self.config.dedup_window_ms:
            self._recent_facts.popleft()

        # Check for duplicate
        for _, h in self._recent_facts:
            if h == det_hash:
                return True

        # Add new fact
        self._recent_facts.append((current_time, det_hash))
        return False
```

#### 9. Update Gymnasium Wrapper

**Edit:** `evaluation/isaac-sim-px4/environments/gymnasium_wrapper.py`

Integrate perception encoder into the observation space:

```python
"""
Updated Gymnasium wrapper with perception integration.
"""
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Any, Dict, Optional, Tuple

from .base_isr_env import BaseISREnvironment, UAVState
from ..perception.perception_encoder import PerceptionEncoder, PerceptionConfig


class IsaacSimGymWrapper(gym.Env):
    """
    Gymnasium wrapper with full perception pipeline.

    Observation space:
    - Base UAV state: 28 dims
    - Perception: 516 dims (priority + grid + stats)
    - Environment-specific: 8 dims
    - TOTAL: 552 dims
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self,
        isaac_env: BaseISREnvironment,
        config: Optional[Dict] = None,
        use_perception: bool = True,
    ):
        super().__init__()
        self.isaac_env = isaac_env
        self.config = config or {}
        self.use_perception = use_perception

        # Initialize perception encoder
        if use_perception:
            perception_config = PerceptionConfig(
                detector_mode=self.config.get('perception_mode', 'ground_truth'),
                detector_config_path=self.config.get('perception_config'),
            )
            self.perception_encoder = PerceptionEncoder(perception_config)
            perception_dim = self.perception_encoder.output_dim
        else:
            self.perception_encoder = None
            perception_dim = 0

        # Calculate observation dimensions
        base_dim = self.isaac_env.observation_dim
        self.obs_dim = base_dim + perception_dim

        # Define observation space
        low = np.full(self.obs_dim, -1.0, dtype=np.float32)
        high = np.full(self.obs_dim, 1.0, dtype=np.float32)

        # Adjust bounds for known ranges
        low[:3] = -1000  # position
        high[:3] = 1000
        low[3:6] = -20   # velocity
        high[3:6] = 20

        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        # Action space (velocity commands)
        self.action_space = spaces.Box(
            low=np.array([-1, -1, -1, -1], dtype=np.float32),
            high=np.array([1, 1, 1, 1], dtype=np.float32),
            dtype=np.float32
        )

        self._max_episode_steps = self.config.get('max_episode_steps', 10000)
        self._step_count = 0
        self._last_image = None
        self._dt = 0.05  # 20Hz

    def _get_observation(self, state: UAVState) -> np.ndarray:
        """Build full observation from state and perception."""
        # Base observation from environment
        base_obs = self.isaac_env.state_to_observation(state)

        if not self.use_perception or self.perception_encoder is None:
            return base_obs

        # Get camera image
        if hasattr(self.isaac_env, 'get_camera_image'):
            image = self.isaac_env.get_camera_image()
        else:
            image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Get ground truth labels from Isaac Sim (for training)
        gt_labels = None
        if hasattr(self.isaac_env, 'get_semantic_labels'):
            gt_labels = self.isaac_env.get_semantic_labels()

        # Encode perception
        perception_obs = self.perception_encoder.encode(
            image=image,
            uav_position=state.position,
            uav_orientation=state.orientation,
            ground_truth_labels=gt_labels,
            dt=self._dt,
        )

        return np.concatenate([base_obs, perception_obs])

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        self._step_count = 0

        state = self.isaac_env.reset(seed=seed)
        obs = self._get_observation(state)

        info = {
            "raw_state": state,
            "step": 0,
            "perception_time_ms": self.perception_encoder.avg_encode_time_ms if self.perception_encoder else 0,
        }
        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        self._step_count += 1

        # Execute action
        state, reward, terminated, info = self.isaac_env.step(action)

        obs = self._get_observation(state)
        truncated = self._step_count >= self._max_episode_steps

        info.update({
            "raw_state": state,
            "step": self._step_count,
            "perception_time_ms": self.perception_encoder.avg_encode_time_ms if self.perception_encoder else 0,
        })

        return obs, reward, terminated, truncated, info

    def render(self):
        if hasattr(self.isaac_env, 'get_camera_image'):
            return self.isaac_env.get_camera_image()
        return None

    def close(self):
        self.isaac_env.close()
```

#### 10. Create Integration Tests

**File:** `perception/tests/test_perception_encoder.py`

```python
"""
Integration tests for perception pipeline.
"""
import pytest
import numpy as np
import sys
sys.path.insert(0, '/workspace/perception')

from perception_encoder import PerceptionEncoder, PerceptionConfig
from detector import Detection


class TestPerceptionEncoder:
    """Test perception encoder end-to-end."""

    @pytest.fixture
    def encoder(self):
        """Create perception encoder."""
        config = PerceptionConfig(detector_mode="ground_truth")
        return PerceptionEncoder(config)

    @pytest.fixture
    def sample_image(self):
        """Create dummy image."""
        return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    @pytest.fixture
    def sample_labels(self):
        """Create sample ground truth labels."""
        return [
            {
                'class_id': 1,
                'bbox': (0.3, 0.4, 0.1, 0.2),
                'position_3d': np.array([10.0, 5.0, 0.0]),
            },
            {
                'class_id': 3,
                'bbox': (0.7, 0.6, 0.15, 0.25),
                'position_3d': np.array([30.0, -10.0, 0.0]),
            },
        ]

    def test_output_shape(self, encoder, sample_image, sample_labels):
        """Test output has correct shape."""
        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=sample_labels,
        )

        assert obs.shape == (encoder.output_dim,)
        assert obs.dtype == np.float32

    def test_output_range(self, encoder, sample_image, sample_labels):
        """Test output values are normalized."""
        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=sample_labels,
        )

        # Most values should be in [-1, 1] or [0, 1]
        assert np.all(obs >= -1.1)
        assert np.all(obs <= 1.1)

    def test_person_detection_priority(self, encoder, sample_image):
        """Test persons are prioritized in observations."""
        labels = [
            {'class_id': 1, 'bbox': (0.5, 0.5, 0.1, 0.2), 'position_3d': np.array([5.0, 0.0, 0.0])},
            {'class_id': 3, 'bbox': (0.3, 0.3, 0.2, 0.3), 'position_3d': np.array([3.0, 0.0, 0.0])},
        ]

        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=labels,
        )

        # First detection should be person (class_id=1, normalized=0.1)
        assert obs[0] == pytest.approx(0.1, abs=0.01)

    def test_encoding_speed(self, encoder, sample_image, sample_labels):
        """Test encoding completes within budget."""
        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        # Warm up
        for _ in range(5):
            encoder.encode(
                image=sample_image,
                uav_position=uav_pos,
                uav_orientation=uav_ori,
                ground_truth_labels=sample_labels,
            )

        # Measure
        import time
        times = []
        for _ in range(20):
            start = time.perf_counter()
            encoder.encode(
                image=sample_image,
                uav_position=uav_pos,
                uav_orientation=uav_ori,
                ground_truth_labels=sample_labels,
            )
            times.append((time.perf_counter() - start) * 1000)

        avg_time = np.mean(times)
        print(f"Average encoding time: {avg_time:.2f} ms")

        # Should complete within 20ms budget (leaving 30ms for Vampire)
        assert avg_time < 20.0, f"Encoding too slow: {avg_time:.2f} ms"

    def test_empty_detections(self, encoder, sample_image):
        """Test handling of empty detection list."""
        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=[],
        )

        assert obs.shape == (encoder.output_dim,)
        # Priority detections should be zeros
        assert np.all(obs[:100] == 0)

    def test_many_detections(self, encoder, sample_image):
        """Test handling of many detections (crowd scenario)."""
        # Create 50 person detections
        labels = [
            {
                'class_id': 1,
                'bbox': (np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), 0.05, 0.1),
                'position_3d': np.array([
                    np.random.uniform(-50, 50),
                    np.random.uniform(-50, 50),
                    0.0
                ]),
            }
            for _ in range(50)
        ]

        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=labels,
        )

        assert obs.shape == (encoder.output_dim,)

        # Grid should capture all detections (person count should be high)
        grid_start = 100  # After priority detections
        grid_obs = obs[grid_start:grid_start + 384]

        # Some cells should have high person counts
        assert np.max(grid_obs) > 0.1


class TestTPTPGenerator:
    """Test TPTP fact generation."""

    @pytest.fixture
    def generator(self):
        from tptp_generator import TPTPGenerator
        return TPTPGenerator()

    def test_person_facts(self, generator):
        """Test person detection generates safety facts."""
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([10.0, 5.0, 0.0]),
                distance=11.2,
            )
        ]

        facts = generator.generate_facts(detections)

        # Should have observation, distance, and person-specific facts
        assert len(facts) >= 3
        assert any("observedObject" in f for f in facts)
        assert any("distanceTo" in f for f in facts)
        assert any("Person" in f for f in facts)

    def test_danger_zone_fact(self, generator):
        """Test person in danger zone generates warning fact."""
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([5.0, 0.0, 0.0]),
                distance=5.0,  # Within 15m danger zone
            )
        ]

        facts = generator.generate_facts(detections)

        assert any("personInDangerZone" in f for f in facts)

    def test_safety_query_format(self, generator):
        """Test complete safety query is valid TPTP."""
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([10.0, 5.0, 0.0]),
                distance=11.2,
            )
        ]

        state = {'x': 0.0, 'y': 0.0, 'z': 50.0, 'battery': 80.0, 'in_geofence': True}
        action = np.array([1.0, 0.0, 0.0, 0.0])

        query = generator.generate_safety_query(detections, state, action)

        # Should have proper TPTP structure
        assert "fof(" in query
        assert "axiom" in query
        assert "conjecture" in query
        assert "safety_query" in query


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
```

#### 11. Create Verification Script

**File:** `.phases/phase-06f-perception-pipeline/verification.sh`

```bash
#!/bin/bash
# Phase 6f Verification Script
# Tests perception pipeline integration

set -e

echo "=============================================="
echo "Phase 6f: Perception Pipeline Verification"
echo "=============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0

check_result() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✓ PASS${NC}: $2"
        ((PASS_COUNT++))
    else
        echo -e "${RED}✗ FAIL${NC}: $2"
        ((FAIL_COUNT++))
    fi
}

# Check if running in container
if [ -d "/workspace/perception" ]; then
    WORKSPACE="/workspace"
else
    WORKSPACE="/home/finley/Github/DroneProjects/flyby-f11/evaluation/isaac-sim-px4"
fi

echo ""
echo "=== 1. File Structure Check ==="

# Check required files exist
FILES=(
    "perception/__init__.py"
    "perception/detector.py"
    "perception/spatial_encoder.py"
    "perception/scene_statistics.py"
    "perception/temporal_tracker.py"
    "perception/perception_encoder.py"
    "perception/tptp_generator.py"
    "perception/config/detection_classes.yaml"
)

for file in "${FILES[@]}"; do
    if [ -f "$WORKSPACE/$file" ]; then
        check_result 0 "File exists: $file"
    else
        check_result 1 "File missing: $file"
    fi
done

echo ""
echo "=== 2. Python Import Check ==="

# Test imports
python3 << 'EOF'
import sys
sys.path.insert(0, '/workspace/perception')

try:
    from detector import YOLODetector, Detection
    print("OK: detector imports")
except Exception as e:
    print(f"FAIL: detector imports - {e}")
    sys.exit(1)

try:
    from spatial_encoder import SpatialGridEncoder
    print("OK: spatial_encoder imports")
except Exception as e:
    print(f"FAIL: spatial_encoder imports - {e}")
    sys.exit(1)

try:
    from scene_statistics import SceneStatistics
    print("OK: scene_statistics imports")
except Exception as e:
    print(f"FAIL: scene_statistics imports - {e}")
    sys.exit(1)

try:
    from temporal_tracker import TemporalTracker
    print("OK: temporal_tracker imports")
except Exception as e:
    print(f"FAIL: temporal_tracker imports - {e}")
    sys.exit(1)

try:
    from perception_encoder import PerceptionEncoder
    print("OK: perception_encoder imports")
except Exception as e:
    print(f"FAIL: perception_encoder imports - {e}")
    sys.exit(1)

try:
    from tptp_generator import TPTPGenerator
    print("OK: tptp_generator imports")
except Exception as e:
    print(f"FAIL: tptp_generator imports - {e}")
    sys.exit(1)

print("All imports successful")
EOF
check_result $? "Python imports"

echo ""
echo "=== 3. Unit Tests ==="

# Run pytest if available
if command -v pytest &> /dev/null; then
    cd "$WORKSPACE"
    pytest perception/tests/ -v --tb=short 2>&1 | head -50
    check_result ${PIPESTATUS[0]} "Unit tests"
else
    echo -e "${YELLOW}SKIP${NC}: pytest not available"
fi

echo ""
echo "=== 4. Encoding Speed Test ==="

python3 << 'EOF'
import sys
sys.path.insert(0, '/workspace/perception')
import numpy as np
import time

from perception_encoder import PerceptionEncoder, PerceptionConfig

# Create encoder
config = PerceptionConfig(detector_mode="ground_truth")
encoder = PerceptionEncoder(config)

# Sample inputs
image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
uav_pos = np.array([0.0, 0.0, 50.0])
uav_ori = np.array([1.0, 0.0, 0.0, 0.0])
labels = [
    {'class_id': 1, 'bbox': (0.3, 0.4, 0.1, 0.2), 'position_3d': np.array([10.0, 5.0, 0.0])},
    {'class_id': 3, 'bbox': (0.7, 0.6, 0.15, 0.25), 'position_3d': np.array([30.0, -10.0, 0.0])},
]

# Warm up
for _ in range(5):
    encoder.encode(image, uav_pos, uav_ori, labels)

# Benchmark
times = []
for _ in range(50):
    start = time.perf_counter()
    obs = encoder.encode(image, uav_pos, uav_ori, labels)
    times.append((time.perf_counter() - start) * 1000)

avg_time = np.mean(times)
max_time = np.max(times)
print(f"Average encoding time: {avg_time:.2f} ms")
print(f"Max encoding time: {max_time:.2f} ms")
print(f"Output shape: {obs.shape}")

if avg_time < 20.0:
    print("PASS: Encoding within 20ms budget")
    sys.exit(0)
else:
    print("FAIL: Encoding exceeds 20ms budget")
    sys.exit(1)
EOF
check_result $? "Encoding speed < 20ms"

echo ""
echo "=== 5. Output Dimensions Check ==="

python3 << 'EOF'
import sys
sys.path.insert(0, '/workspace/perception')
import numpy as np

from perception_encoder import PerceptionEncoder, PerceptionConfig

config = PerceptionConfig()
encoder = PerceptionEncoder(config)

expected_dim = 100 + 384 + 32  # priority + grid + stats = 516
actual_dim = encoder.output_dim

print(f"Expected dimension: {expected_dim}")
print(f"Actual dimension: {actual_dim}")

if actual_dim == expected_dim:
    print("PASS: Output dimensions correct")
    sys.exit(0)
else:
    print("FAIL: Output dimensions mismatch")
    sys.exit(1)
EOF
check_result $? "Output dimensions = 516"

echo ""
echo "=== 6. TPTP Generation Check ==="

python3 << 'EOF'
import sys
sys.path.insert(0, '/workspace/perception')
import numpy as np

from detector import Detection
from tptp_generator import TPTPGenerator

generator = TPTPGenerator()

detections = [
    Detection(
        class_id=1,
        class_name="person",
        ontology_class="Person",
        confidence=0.9,
        bbox=(0.5, 0.5, 0.1, 0.2),
        position_3d=np.array([10.0, 5.0, 0.0]),
        distance=11.2,
    )
]

facts = generator.generate_facts(detections)

print(f"Generated {len(facts)} TPTP facts")
for fact in facts[:3]:
    print(f"  {fact[:80]}...")

if len(facts) >= 2 and all("fof(" in f for f in facts):
    print("PASS: TPTP facts generated correctly")
    sys.exit(0)
else:
    print("FAIL: TPTP facts invalid")
    sys.exit(1)
EOF
check_result $? "TPTP fact generation"

echo ""
echo "=============================================="
echo "Verification Summary"
echo "=============================================="
echo -e "Passed: ${GREEN}$PASS_COUNT${NC}"
echo -e "Failed: ${RED}$FAIL_COUNT${NC}"

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "\n${GREEN}All verifications passed!${NC}"
    exit 0
else
    echo -e "\n${RED}Some verifications failed.${NC}"
    exit 1
fi
```

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `perception/` package structure created with all modules
- [ ] `YOLODetector` runs in ground_truth and inference modes
- [ ] `SpatialGridEncoder` produces 384-dim output
- [ ] `SceneStatistics` produces 32-dim output
- [ ] `TemporalTracker` maintains tracks across frames
- [ ] `PerceptionEncoder` produces 516-dim observation
- [ ] `TPTPGenerator` produces valid TPTP facts
- [ ] Encoding completes in < 20ms (leaving 30ms for Vampire)
- [ ] Unit tests pass
- [ ] Integration with `gymnasium_wrapper.py` works
- [ ] All detections captured (no information loss)

### Verification

Run automated verification:
```bash
bash .phases/phase-06f-perception-pipeline/verification.sh
```

Manual verification:
```bash
# Start container
podman exec -it isaac-6e-test bash

# Run tests
cd /workspace
python -m pytest perception/tests/ -v

# Test encoding speed
python -c "
from perception.perception_encoder import PerceptionEncoder, PerceptionConfig
import numpy as np
import time

encoder = PerceptionEncoder(PerceptionConfig(detector_mode='ground_truth'))
image = np.zeros((480, 640, 3), dtype=np.uint8)
labels = [{'class_id': 1, 'bbox': (0.5, 0.5, 0.1, 0.2), 'position_3d': np.array([10, 5, 0])}]

times = []
for _ in range(100):
    start = time.perf_counter()
    obs = encoder.encode(image, np.zeros(3), np.array([1,0,0,0]), labels)
    times.append((time.perf_counter() - start) * 1000)

print(f'Avg: {np.mean(times):.2f} ms, Max: {np.max(times):.2f} ms')
print(f'Output shape: {obs.shape}')
"
```

### Common Pitfalls

1. **YOLO model loading**: Ensure `yolo11n.pt` is downloaded or use ground_truth mode
2. **NumPy broadcasting**: Spatial grid operations can fail with wrong shapes
3. **Deque memory**: Tracker history can grow large with many detections
4. **TPTP syntax**: Vampire is strict - test facts with simple queries first
5. **Coordinate frames**: Ensure bbox (normalized 0-1) vs position_3d (meters) are consistent

### References

- [YOLOv11 Documentation](https://docs.ultralytics.com/)
- [MODEL_CONFIGURATIONS.md](../../docs/vision/MODEL_CONFIGURATIONS.md)
- [Phase 6e: Isaac Sim RL Bridge](../phase-06e-isaac-sim-rl-bridge/TASK.md)
- [Vampire TPTP Syntax](http://tptp.org/TPTP/SyntaxBNF.html)
- [Gymnasium Spaces](https://gymnasium.farama.org/api/spaces/)

### Dependencies
See `dependencies.json` - requires Phase 6e completion.

### Next Phase
After completion, proceed to Phase 7: Mission Planner RL Agent training with full perception.
