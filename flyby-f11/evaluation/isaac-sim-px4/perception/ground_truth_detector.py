"""
Ground Truth Detector for Fast RL Training

Uses frustum intersection with known world object positions to generate
detections WITHOUT rendering. Enables 1000+ env steps/second for RL training.

The detector produces the same Detection dataclass as the YOLO detector,
ensuring seamless switching between training (GT) and E2E (inference) modes.

Key features:
- No rendering required (frustum math only)
- Configurable detection noise to match real YOLO behavior
- Same output format as YOLODetector for policy interoperability
- Supports occlusion estimation (optional)
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple, Any
import time

try:
    from .frustum import (
        CameraFrustum, CameraParams, AABB,
        camera_pose_from_position_orientation,
        create_aabb_for_object, DEFAULT_OBJECT_SIZES
    )
    from .detector import Detection
except ImportError:
    from frustum import (
        CameraFrustum, CameraParams, AABB,
        camera_pose_from_position_orientation,
        create_aabb_for_object, DEFAULT_OBJECT_SIZES
    )
    from detector import Detection


@dataclass
class WorldObject:
    """Object in the simulation world."""
    id: str
    class_id: int
    class_name: str
    position: np.ndarray  # World frame (x, y, z)
    size: Optional[np.ndarray] = None  # (width, depth, height) override
    velocity: Optional[np.ndarray] = None  # World frame velocity
    ontology_class: str = "Object"
    priority: int = 10
    safety_distance: float = 0.0

    def get_aabb(self) -> AABB:
        """Get axis-aligned bounding box for this object."""
        return create_aabb_for_object(self.position, self.class_id, self.size)


@dataclass
class GroundTruthConfig:
    """Configuration for ground truth detection."""
    # Camera parameters
    camera_params: CameraParams = None

    # Detection noise (to simulate YOLO uncertainty)
    bbox_noise_std: float = 0.02  # Std dev of bbox position noise
    confidence_base: float = 0.95  # Base confidence for detections
    confidence_distance_decay: float = 0.001  # Confidence decay per meter
    confidence_noise_std: float = 0.05  # Random confidence variation

    # Position estimation noise
    position_noise_std: float = 0.5  # Std dev of 3D position noise (meters)
    depth_estimation_error_pct: float = 0.05  # 5% depth error

    # Detection probability (simulate missed detections)
    base_detection_prob: float = 0.98  # Probability of detecting visible object
    small_object_threshold: float = 0.01  # Min bbox area to detect
    detection_prob_distance_decay: float = 0.002  # Miss rate increases with distance

    # Occlusion
    enable_occlusion: bool = False  # Approximate occlusion checking
    occlusion_sample_count: int = 5  # Rays to sample for occlusion

    # Class mapping (matches detector.py)
    class_config: Dict[int, Dict[str, Any]] = None

    def __post_init__(self):
        if self.camera_params is None:
            self.camera_params = CameraParams()

        if self.class_config is None:
            self.class_config = {
                1: {'name': 'person', 'ontology_class': 'Person', 'priority': 1, 'safety_distance': 15.0},
                2: {'name': 'vehicle', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
                3: {'name': 'building', 'ontology_class': 'StaticObstacle', 'priority': 3, 'safety_distance': 5.0},
                4: {'name': 'poi_target', 'ontology_class': 'TargetOfInterest', 'priority': 4, 'safety_distance': 0.0},
                5: {'name': 'landing_zone', 'ontology_class': 'LandingZone', 'priority': 5, 'safety_distance': 0.0},
                6: {'name': 'tree', 'ontology_class': 'StaticObstacle', 'priority': 6, 'safety_distance': 3.0},
                7: {'name': 'power_line', 'ontology_class': 'StaticObstacle', 'priority': 2, 'safety_distance': 5.0},
            }


class GroundTruthDetector:
    """
    Fast ground truth detector using frustum intersection.

    This detector uses the known positions of objects in the simulation
    to generate detections without any rendering. It adds configurable
    noise to simulate real detector behavior for robust sim-to-real transfer.

    Usage:
        config = GroundTruthConfig(
            camera_params=CameraParams(fov_horizontal=90.0),
            confidence_noise_std=0.05
        )
        detector = GroundTruthDetector(config)

        # Register objects from simulation
        detector.update_world_objects(sim_objects)

        # Get detections
        detections = detector.detect(
            camera_pose=np.eye(4),  # Or from UAV state
            uav_position=np.array([0, 0, 10])
        )
    """

    def __init__(self, config: GroundTruthConfig = None):
        self.config = config or GroundTruthConfig()
        self.frustum = CameraFrustum(self.config.camera_params)

        # World objects registry
        self._world_objects: List[WorldObject] = []
        self._objects_by_id: Dict[str, WorldObject] = {}

        # Random state for reproducible noise
        self._rng = np.random.default_rng()

        # Performance tracking
        self._inference_times: List[float] = []

    def set_seed(self, seed: int):
        """Set random seed for reproducible noise."""
        self._rng = np.random.default_rng(seed)

    def update_world_objects(self, objects: List[WorldObject]):
        """
        Update the registry of world objects.

        Call this each step with current object states from simulation.

        Args:
            objects: List of WorldObject with current positions/velocities
        """
        self._world_objects = objects
        self._objects_by_id = {obj.id: obj for obj in objects}

    def add_world_object(self, obj: WorldObject):
        """Add a single object to the world."""
        self._world_objects.append(obj)
        self._objects_by_id[obj.id] = obj

    def remove_world_object(self, obj_id: str):
        """Remove an object from the world."""
        if obj_id in self._objects_by_id:
            obj = self._objects_by_id.pop(obj_id)
            self._world_objects.remove(obj)

    def clear_world_objects(self):
        """Clear all world objects."""
        self._world_objects.clear()
        self._objects_by_id.clear()

    def detect(
        self,
        camera_pose: np.ndarray,
        uav_position: np.ndarray,
    ) -> List[Detection]:
        """
        Generate detections for objects visible in camera frustum.

        Args:
            camera_pose: 4x4 camera-to-world transformation matrix
            uav_position: UAV position in world frame (for distance calculation)

        Returns:
            List of Detection objects (same format as YOLODetector output)
        """
        start_time = time.perf_counter()
        detections = []

        # Get AABBs for all objects
        aabbs = [obj.get_aabb() for obj in self._world_objects]

        # Find visible objects using frustum intersection
        visible_indices = self.frustum.get_visible_objects(aabbs, camera_pose)

        for idx in visible_indices:
            obj = self._world_objects[idx]
            aabb = aabbs[idx]

            # Compute projected bounding box
            bbox = self.frustum.compute_bbox_from_aabb(aabb, camera_pose)
            if bbox is None:
                continue

            cx, cy, w, h = bbox

            # Check minimum size
            if w * h < self.config.small_object_threshold:
                continue

            # Calculate distance
            distance = float(np.linalg.norm(obj.position - uav_position))

            # Simulate detection probability
            detection_prob = self._compute_detection_probability(distance, w * h)
            if self._rng.random() > detection_prob:
                continue  # Missed detection

            # Add detection noise
            noisy_bbox = self._add_bbox_noise(bbox)
            noisy_confidence = self._compute_confidence(distance)
            noisy_position = self._add_position_noise(obj.position, distance)

            # Calculate bearing
            rel_pos = obj.position - uav_position
            bearing = float(np.arctan2(rel_pos[1], rel_pos[0]))

            # Get class config
            class_cfg = self.config.class_config.get(obj.class_id, {})

            detection = Detection(
                class_id=obj.class_id,
                class_name=class_cfg.get('name', obj.class_name),
                ontology_class=class_cfg.get('ontology_class', obj.ontology_class),
                confidence=noisy_confidence,
                bbox=noisy_bbox,
                position_3d=noisy_position,
                distance=distance + self._rng.normal(0, distance * 0.02),  # Small distance noise
                bearing=bearing,
                velocity=obj.velocity,
                priority=class_cfg.get('priority', obj.priority),
                safety_distance=class_cfg.get('safety_distance', obj.safety_distance),
            )
            detections.append(detection)

        # Sort by priority (safety-critical first)
        detections.sort(key=lambda d: (d.priority, -d.confidence))

        # Track inference time
        elapsed = (time.perf_counter() - start_time) * 1000
        self._inference_times.append(elapsed)
        if len(self._inference_times) > 100:
            self._inference_times.pop(0)

        return detections

    def detect_from_uav_state(
        self,
        uav_position: np.ndarray,
        uav_orientation: np.ndarray,
    ) -> List[Detection]:
        """
        Convenience method to detect from UAV state vectors.

        Constructs camera pose from UAV position and orientation.

        Args:
            uav_position: UAV position (x, y, z) in world frame
            uav_orientation: UAV orientation as quaternion (w, x, y, z)

        Returns:
            List of Detection objects
        """
        camera_pose = camera_pose_from_position_orientation(
            uav_position, uav_orientation
        )
        return self.detect(camera_pose, uav_position)

    def _compute_detection_probability(
        self,
        distance: float,
        bbox_area: float
    ) -> float:
        """Compute probability of detecting an object."""
        # Base probability
        prob = self.config.base_detection_prob

        # Distance decay
        prob -= self.config.detection_prob_distance_decay * distance

        # Small objects are harder to detect
        if bbox_area < 0.05:  # Less than 5% of image
            prob *= bbox_area / 0.05

        return np.clip(prob, 0.0, 1.0)

    def _compute_confidence(self, distance: float) -> float:
        """Compute detection confidence with noise."""
        # Base confidence decays with distance
        conf = self.config.confidence_base
        conf -= self.config.confidence_distance_decay * distance

        # Add random noise
        conf += self._rng.normal(0, self.config.confidence_noise_std)

        return float(np.clip(conf, 0.1, 1.0))

    def _add_bbox_noise(
        self,
        bbox: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float, float]:
        """Add noise to bounding box coordinates."""
        cx, cy, w, h = bbox

        noise_scale = self.config.bbox_noise_std

        cx_noisy = cx + self._rng.normal(0, noise_scale)
        cy_noisy = cy + self._rng.normal(0, noise_scale)
        w_noisy = w * (1 + self._rng.normal(0, noise_scale))
        h_noisy = h * (1 + self._rng.normal(0, noise_scale))

        # Keep in valid range
        cx_noisy = np.clip(cx_noisy, 0.01, 0.99)
        cy_noisy = np.clip(cy_noisy, 0.01, 0.99)
        w_noisy = np.clip(w_noisy, 0.01, 0.5)
        h_noisy = np.clip(h_noisy, 0.01, 0.5)

        return (float(cx_noisy), float(cy_noisy), float(w_noisy), float(h_noisy))

    def _add_position_noise(
        self,
        position: np.ndarray,
        distance: float
    ) -> np.ndarray:
        """Add noise to 3D position estimate."""
        # Position noise scales with distance (depth uncertainty)
        base_noise = self.config.position_noise_std
        depth_noise = distance * self.config.depth_estimation_error_pct

        noise = self._rng.normal(0, base_noise, 3)
        noise[2] += self._rng.normal(0, depth_noise)  # Extra depth uncertainty

        return position + noise

    @property
    def avg_inference_time_ms(self) -> float:
        """Average inference time in milliseconds."""
        if not self._inference_times:
            return 0.0
        return sum(self._inference_times) / len(self._inference_times)


def create_world_objects_from_sim(
    sim_objects: List[Dict[str, Any]]
) -> List[WorldObject]:
    """
    Create WorldObject instances from simulation object data.

    Args:
        sim_objects: List of dicts with keys:
            - id: str
            - class_id: int
            - position: [x, y, z]
            - velocity: [vx, vy, vz] (optional)
            - size: [w, d, h] (optional)

    Returns:
        List of WorldObject instances
    """
    world_objects = []

    # Class name mapping
    class_names = {
        1: ('person', 'Person'),
        2: ('vehicle', 'DynamicObstacle'),
        3: ('building', 'StaticObstacle'),
        4: ('poi_target', 'TargetOfInterest'),
        5: ('landing_zone', 'LandingZone'),
        6: ('tree', 'StaticObstacle'),
        7: ('power_line', 'StaticObstacle'),
    }

    for obj in sim_objects:
        class_id = obj.get('class_id', obj.get('semantic_id', 0))
        names = class_names.get(class_id, ('unknown', 'Object'))

        world_obj = WorldObject(
            id=obj.get('id', f"obj_{len(world_objects)}"),
            class_id=class_id,
            class_name=names[0],
            position=np.array(obj['position']),
            size=np.array(obj['size']) if 'size' in obj else None,
            velocity=np.array(obj['velocity']) if 'velocity' in obj else None,
            ontology_class=names[1],
        )
        world_objects.append(world_obj)

    return world_objects
