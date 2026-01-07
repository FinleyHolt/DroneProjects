"""
Object Tracker - Temporal coherence tracking for detections.

Single responsibility: Maintain persistent object IDs across frames using
IoU-based association and Hungarian algorithm matching.

Features:
- Persistent track IDs across frames
- Handles occlusion and re-identification
- Tracks object velocity/heading for prediction
- Confidence decay for objects not seen recently
- Configurable association thresholds
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


@dataclass
class TrackerConfig:
    """Configuration for object tracking."""
    # Association thresholds
    iou_threshold: float = 0.3  # Min IoU for association
    distance_threshold: float = 30.0  # Max distance for association (meters)

    # Track lifecycle
    max_age: int = 30  # Frames before deleting unmatched track
    min_hits: int = 3  # Min detections before track is confirmed
    n_init: int = 3  # Frames to wait before outputting track

    # Confidence decay
    confidence_decay: float = 0.95  # Per-frame confidence multiplier when not seen
    min_confidence: float = 0.1  # Delete track below this confidence

    # Velocity estimation
    velocity_smoothing: float = 0.7  # EMA smoothing for velocity (0=no smoothing)

    # Use world position vs bbox for association
    use_world_position: bool = True  # Prefer 3D position matching when available


@dataclass
class Track:
    """Single tracked object with state history."""
    track_id: int
    class_id: int
    class_name: str

    # Current state
    bbox: Tuple[float, float, float, float]  # x_center, y_center, width, height
    world_position: Optional[np.ndarray] = None
    confidence: float = 1.0

    # Tracking state
    hits: int = 1  # Total successful associations
    age: int = 0  # Frames since creation
    frames_since_seen: int = 0  # Frames since last detection
    is_confirmed: bool = False

    # Velocity estimation (world coordinates)
    velocity: Optional[np.ndarray] = None  # [vx, vy, vz] m/frame
    heading: Optional[float] = None  # Heading in degrees

    # State history for smoothing
    position_history: List[np.ndarray] = field(default_factory=list)
    bbox_history: List[Tuple[float, float, float, float]] = field(default_factory=list)

    def predict(self, dt: float = 1.0) -> np.ndarray:
        """Predict next position using velocity."""
        if self.velocity is not None and self.world_position is not None:
            return self.world_position + self.velocity * dt
        return self.world_position if self.world_position is not None else np.zeros(3)

    def update_velocity(self, new_position: np.ndarray, smoothing: float = 0.7):
        """Update velocity estimate from new position."""
        if len(self.position_history) > 0:
            prev_pos = self.position_history[-1]
            new_velocity = new_position - prev_pos

            if self.velocity is None:
                self.velocity = new_velocity
            else:
                # Exponential moving average
                self.velocity = smoothing * self.velocity + (1 - smoothing) * new_velocity

            # Update heading from horizontal velocity
            if abs(self.velocity[0]) > 0.01 or abs(self.velocity[1]) > 0.01:
                self.heading = np.degrees(np.arctan2(self.velocity[1], self.velocity[0]))

        self.position_history.append(new_position.copy())
        # Keep limited history
        if len(self.position_history) > 10:
            self.position_history.pop(0)


class ObjectTracker:
    """
    Multi-object tracker with temporal coherence.

    Uses Hungarian algorithm for optimal detection-to-track association
    based on IoU (2D) or Euclidean distance (3D).

    Usage:
        tracker = ObjectTracker(TrackerConfig())

        # Each frame
        detections = perception_manager.get_detections(...)
        tracks = tracker.update(detections)

        for track in tracks:
            print(f"Track {track.track_id}: {track.class_name} at {track.world_position}")
    """

    def __init__(self, config: TrackerConfig = None):
        self.config = config or TrackerConfig()
        self._tracks: Dict[int, Track] = {}
        self._next_track_id = 1
        self._frame_count = 0

    def update(self, detections: List) -> List[Track]:
        """
        Update tracker with new detections.

        Args:
            detections: List of Detection objects from PerceptionManager

        Returns:
            List of confirmed Track objects with persistent IDs
        """
        self._frame_count += 1

        # Age all tracks
        for track in self._tracks.values():
            track.age += 1
            track.frames_since_seen += 1
            track.confidence *= self.config.confidence_decay

        if len(detections) == 0:
            # No detections - just age tracks and remove old ones
            self._remove_stale_tracks()
            return self._get_confirmed_tracks()

        if len(self._tracks) == 0:
            # No existing tracks - create new ones for all detections
            for det in detections:
                self._create_track(det)
            return self._get_confirmed_tracks()

        # Build cost matrix for Hungarian algorithm
        cost_matrix = self._build_cost_matrix(detections)

        # Perform association
        if SCIPY_AVAILABLE and cost_matrix.size > 0:
            matched, unmatched_dets, unmatched_tracks = self._hungarian_matching(
                cost_matrix, detections
            )
        else:
            # Fallback to greedy matching
            matched, unmatched_dets, unmatched_tracks = self._greedy_matching(
                cost_matrix, detections
            )

        # Update matched tracks
        for det_idx, track_id in matched:
            self._update_track(track_id, detections[det_idx])

        # Create new tracks for unmatched detections
        for det_idx in unmatched_dets:
            self._create_track(detections[det_idx])

        # Remove stale tracks
        self._remove_stale_tracks()

        return self._get_confirmed_tracks()

    def _build_cost_matrix(self, detections: List) -> np.ndarray:
        """Build cost matrix for detection-track association."""
        n_tracks = len(self._tracks)
        n_dets = len(detections)

        if n_tracks == 0 or n_dets == 0:
            return np.empty((0, 0))

        cost_matrix = np.full((n_tracks, n_dets), 1e6)
        track_ids = list(self._tracks.keys())

        for t_idx, track_id in enumerate(track_ids):
            track = self._tracks[track_id]

            for d_idx, det in enumerate(detections):
                # Class must match
                if det.class_id != track.class_id:
                    continue

                # Calculate cost based on position or bbox
                if (self.config.use_world_position and
                    track.world_position is not None and
                    det.world_position is not None):
                    # 3D Euclidean distance
                    predicted_pos = track.predict()
                    distance = np.linalg.norm(det.world_position - predicted_pos)
                    if distance < self.config.distance_threshold:
                        cost_matrix[t_idx, d_idx] = distance
                else:
                    # 2D IoU
                    iou = self._compute_iou(track.bbox, det.bbox)
                    if iou > self.config.iou_threshold:
                        cost_matrix[t_idx, d_idx] = 1.0 - iou

        return cost_matrix

    def _hungarian_matching(
        self, cost_matrix: np.ndarray, detections: List
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """Perform Hungarian algorithm matching."""
        n_tracks = cost_matrix.shape[0]
        n_dets = cost_matrix.shape[1]
        track_ids = list(self._tracks.keys())

        # Run Hungarian algorithm
        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        matched = []
        unmatched_dets = list(range(n_dets))
        unmatched_tracks = list(track_ids)

        for row, col in zip(row_indices, col_indices):
            if cost_matrix[row, col] < 1e5:  # Valid match
                track_id = track_ids[row]
                matched.append((col, track_id))
                if col in unmatched_dets:
                    unmatched_dets.remove(col)
                if track_id in unmatched_tracks:
                    unmatched_tracks.remove(track_id)

        return matched, unmatched_dets, unmatched_tracks

    def _greedy_matching(
        self, cost_matrix: np.ndarray, detections: List
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """Fallback greedy matching when scipy not available."""
        n_tracks = cost_matrix.shape[0]
        n_dets = cost_matrix.shape[1]
        track_ids = list(self._tracks.keys())

        matched = []
        unmatched_dets = list(range(n_dets))
        unmatched_tracks = list(track_ids)

        # Greedy: match lowest cost pairs iteratively
        used_tracks = set()
        used_dets = set()

        while True:
            best_cost = 1e6
            best_pair = None

            for t_idx in range(n_tracks):
                if t_idx in used_tracks:
                    continue
                for d_idx in range(n_dets):
                    if d_idx in used_dets:
                        continue
                    if cost_matrix[t_idx, d_idx] < best_cost:
                        best_cost = cost_matrix[t_idx, d_idx]
                        best_pair = (t_idx, d_idx)

            if best_pair is None or best_cost >= 1e5:
                break

            t_idx, d_idx = best_pair
            track_id = track_ids[t_idx]
            matched.append((d_idx, track_id))
            used_tracks.add(t_idx)
            used_dets.add(d_idx)

            if d_idx in unmatched_dets:
                unmatched_dets.remove(d_idx)
            if track_id in unmatched_tracks:
                unmatched_tracks.remove(track_id)

        return matched, unmatched_dets, unmatched_tracks

    def _compute_iou(
        self,
        bbox1: Tuple[float, float, float, float],
        bbox2: Tuple[float, float, float, float],
    ) -> float:
        """Compute IoU between two bboxes in [cx, cy, w, h] format."""
        # Convert to [x1, y1, x2, y2]
        x1_1 = bbox1[0] - bbox1[2] / 2
        y1_1 = bbox1[1] - bbox1[3] / 2
        x2_1 = bbox1[0] + bbox1[2] / 2
        y2_1 = bbox1[1] + bbox1[3] / 2

        x1_2 = bbox2[0] - bbox2[2] / 2
        y1_2 = bbox2[1] - bbox2[3] / 2
        x2_2 = bbox2[0] + bbox2[2] / 2
        y2_2 = bbox2[1] + bbox2[3] / 2

        # Intersection
        xi1 = max(x1_1, x1_2)
        yi1 = max(y1_1, y1_2)
        xi2 = min(x2_1, x2_2)
        yi2 = min(y2_1, y2_2)

        inter_width = max(0, xi2 - xi1)
        inter_height = max(0, yi2 - yi1)
        intersection = inter_width * inter_height

        # Union
        area1 = bbox1[2] * bbox1[3]
        area2 = bbox2[2] * bbox2[3]
        union = area1 + area2 - intersection

        if union <= 0:
            return 0.0

        return intersection / union

    def _create_track(self, detection) -> int:
        """Create new track from detection."""
        track_id = self._next_track_id
        self._next_track_id += 1

        world_pos = None
        if detection.world_position is not None:
            world_pos = np.array(detection.world_position)

        track = Track(
            track_id=track_id,
            class_id=detection.class_id,
            class_name=detection.class_name,
            bbox=detection.bbox,
            world_position=world_pos,
            confidence=detection.confidence,
            hits=1,
            age=0,
            frames_since_seen=0,
            is_confirmed=False,
        )

        if world_pos is not None:
            track.position_history.append(world_pos.copy())

        self._tracks[track_id] = track
        return track_id

    def _update_track(self, track_id: int, detection) -> None:
        """Update existing track with new detection."""
        track = self._tracks[track_id]

        # Update state
        track.bbox = detection.bbox
        track.confidence = detection.confidence
        track.hits += 1
        track.frames_since_seen = 0

        # Update world position and velocity
        if detection.world_position is not None:
            new_pos = np.array(detection.world_position)
            track.update_velocity(new_pos, self.config.velocity_smoothing)
            track.world_position = new_pos

        # Confirm track if enough hits
        if track.hits >= self.config.min_hits:
            track.is_confirmed = True

    def _remove_stale_tracks(self) -> None:
        """Remove tracks that are too old or low confidence."""
        to_remove = []

        for track_id, track in self._tracks.items():
            if track.frames_since_seen > self.config.max_age:
                to_remove.append(track_id)
            elif track.confidence < self.config.min_confidence:
                to_remove.append(track_id)

        for track_id in to_remove:
            del self._tracks[track_id]

    def _get_confirmed_tracks(self) -> List[Track]:
        """Get list of confirmed tracks."""
        return [
            track for track in self._tracks.values()
            if track.is_confirmed or track.age >= self.config.n_init
        ]

    def get_all_tracks(self) -> List[Track]:
        """Get all tracks including unconfirmed."""
        return list(self._tracks.values())

    def get_track_by_id(self, track_id: int) -> Optional[Track]:
        """Get specific track by ID."""
        return self._tracks.get(track_id)

    def reset(self) -> None:
        """Reset tracker state for new episode."""
        self._tracks.clear()
        self._next_track_id = 1
        self._frame_count = 0

    def get_stats(self) -> Dict:
        """Get tracker statistics."""
        confirmed = len([t for t in self._tracks.values() if t.is_confirmed])
        return {
            "total_tracks": len(self._tracks),
            "confirmed_tracks": confirmed,
            "next_id": self._next_track_id,
            "frame_count": self._frame_count,
        }
