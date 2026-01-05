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

try:
    from .detector import Detection
except ImportError:
    from detector import Detection


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
        # Internal class IDs:
        #   1: person, 2: car, 3: building, 4: poi_target, 5: bus,
        #   6: landing_zone, 7: truck, 8: bicycle, 9: motorcycle
        persons = [d for d in detections if d.class_id == 1]
        # All vehicle types: car(2), bus(5), truck(7), bicycle(8), motorcycle(9)
        vehicles = [d for d in detections if d.class_id in [2, 5, 7, 8, 9]]
        buildings = [d for d in detections if d.class_id == 3]
        pois = [d for d in detections if d.class_id == 4]
        landing_zones = [d for d in detections if d.class_id == 6]
        # Obstacles include vehicles and buildings (anything that needs avoidance)
        obstacles = [d for d in detections if d.class_id in [2, 3, 5, 7, 8, 9]]

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
