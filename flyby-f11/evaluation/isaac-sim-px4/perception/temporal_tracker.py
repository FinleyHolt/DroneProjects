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

try:
    from .detector import Detection
except ImportError:
    from detector import Detection


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
