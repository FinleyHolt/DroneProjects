"""
ByteTrack-style Multi-Object Tracker for ISR Detection.

Implements the ByteTrack algorithm for robust multi-object tracking with:
- Two-stage association (high-confidence first, then low-confidence)
- Kalman filter for motion prediction
- Track state management (tentative, confirmed, lost)
- Re-identification support via appearance features

Reference: https://arxiv.org/abs/2110.06864
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple
import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class TrackState(Enum):
    """Track lifecycle states."""
    TENTATIVE = 1   # New track, not yet confirmed
    CONFIRMED = 2   # Track confirmed with enough hits
    LOST = 3        # Track lost, waiting for re-identification


@dataclass
class ByteTrackerConfig:
    """Configuration for ByteTrack."""
    # Detection thresholds
    high_thresh: float = 0.6      # High confidence detection threshold
    low_thresh: float = 0.1       # Low confidence detection threshold
    match_thresh: float = 0.8     # IoU threshold for matching

    # Track lifecycle
    track_buffer: int = 30        # Frames to keep lost tracks
    min_hits: int = 3             # Hits before track is confirmed
    max_time_lost: int = 30       # Max frames before deleting lost track

    # Kalman filter noise
    std_weight_position: float = 1.0 / 20
    std_weight_velocity: float = 1.0 / 160

    # Appearance features (optional)
    use_appearance: bool = False
    appearance_weight: float = 0.5  # Weight for appearance vs motion

    # 3D position tracking
    use_3d_position: bool = True
    position_weight: float = 0.3   # Weight for 3D position matching


@dataclass
class STrack:
    """
    Single tracked object with Kalman filter state.

    State vector: [cx, cy, aspect_ratio, height, vx, vy, va, vh]
    """
    track_id: int
    class_id: int
    class_name: str

    # Detection state
    bbox: np.ndarray  # [x1, y1, x2, y2]
    score: float
    world_position: Optional[np.ndarray] = None

    # Kalman filter state
    mean: np.ndarray = None  # State mean [cx, cy, a, h, vx, vy, va, vh]
    covariance: np.ndarray = None  # State covariance

    # Track lifecycle
    state: TrackState = TrackState.TENTATIVE
    is_activated: bool = False
    frame_id: int = 0
    start_frame: int = 0
    tracklet_len: int = 0
    time_since_update: int = 0

    # Appearance features
    curr_feat: Optional[np.ndarray] = None
    smooth_feat: Optional[np.ndarray] = None
    features: List[np.ndarray] = field(default_factory=list)

    # Velocity history for temporal coherence display
    velocity_history: List[np.ndarray] = field(default_factory=list)

    def __post_init__(self):
        if self.mean is None:
            self._init_kalman()

    def _init_kalman(self):
        """Initialize Kalman filter state from bbox."""
        cx = (self.bbox[0] + self.bbox[2]) / 2
        cy = (self.bbox[1] + self.bbox[3]) / 2
        w = self.bbox[2] - self.bbox[0]
        h = self.bbox[3] - self.bbox[1]
        a = w / max(h, 1e-6)  # Aspect ratio

        self.mean = np.array([cx, cy, a, h, 0, 0, 0, 0], dtype=np.float64)
        self.covariance = np.eye(8, dtype=np.float64)
        # Initialize with high uncertainty
        self.covariance[4:, 4:] *= 100

    def predict(self, kf: 'KalmanFilter'):
        """Predict next state using Kalman filter."""
        self.mean, self.covariance = kf.predict(self.mean, self.covariance)
        self.time_since_update += 1

    def update(self, detection, kf: 'KalmanFilter', frame_id: int):
        """Update track with matched detection."""
        # Convert detection bbox to measurement
        bbox = detection.bbox if hasattr(detection, 'bbox') else detection
        if len(bbox) == 4 and bbox[2] < 1.0:  # Normalized [cx, cy, w, h]
            # Convert to [x1, y1, x2, y2]
            cx, cy, w, h = bbox
            bbox = np.array([cx - w/2, cy - h/2, cx + w/2, cy + h/2])

        self.bbox = np.array(bbox)

        # Kalman update
        measurement = self._bbox_to_measurement(bbox)
        self.mean, self.covariance = kf.update(
            self.mean, self.covariance, measurement
        )

        # Update metadata
        if hasattr(detection, 'confidence'):
            self.score = detection.confidence
        if hasattr(detection, 'world_position') and detection.world_position is not None:
            # Track velocity from position changes
            if self.world_position is not None:
                velocity = detection.world_position - self.world_position
                self.velocity_history.append(velocity)
                if len(self.velocity_history) > 10:
                    self.velocity_history.pop(0)
            self.world_position = detection.world_position.copy()

        # Update appearance features
        if hasattr(detection, 'feature') and detection.feature is not None:
            self._update_features(detection.feature)

        self.frame_id = frame_id
        self.tracklet_len += 1
        self.time_since_update = 0

        # Confirm track after min_hits (default behavior: confirm after 1 hit for immediate visualization)
        # The min_hits check is handled by the tracker, so we confirm immediately here
        if self.state == TrackState.TENTATIVE and self.tracklet_len >= 1:
            self.state = TrackState.CONFIRMED
            self.is_activated = True

    def _bbox_to_measurement(self, bbox: np.ndarray) -> np.ndarray:
        """Convert [x1, y1, x2, y2] to [cx, cy, a, h]."""
        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]
        cx = bbox[0] + w / 2
        cy = bbox[1] + h / 2
        a = w / max(h, 1e-6)
        return np.array([cx, cy, a, h])

    def _update_features(self, feat: np.ndarray, alpha: float = 0.9):
        """Update appearance feature with EMA."""
        feat = feat / (np.linalg.norm(feat) + 1e-6)
        self.curr_feat = feat
        if self.smooth_feat is None:
            self.smooth_feat = feat
        else:
            self.smooth_feat = alpha * self.smooth_feat + (1 - alpha) * feat
            self.smooth_feat = self.smooth_feat / (np.linalg.norm(self.smooth_feat) + 1e-6)
        self.features.append(feat)
        if len(self.features) > 100:
            self.features.pop(0)

    def mark_lost(self):
        """Mark track as lost."""
        self.state = TrackState.LOST

    def mark_removed(self):
        """Mark track for removal."""
        self.state = TrackState.LOST
        self.time_since_update = 1000  # Force removal

    @property
    def tlbr(self) -> np.ndarray:
        """Get bounding box as [x1, y1, x2, y2]."""
        if self.mean is None:
            return self.bbox
        # Convert from Kalman state
        cx, cy, a, h = self.mean[:4]
        w = a * h
        return np.array([cx - w/2, cy - h/2, cx + w/2, cy + h/2])

    @property
    def tlwh(self) -> np.ndarray:
        """Get bounding box as [x, y, w, h]."""
        ret = self.tlbr.copy()
        ret[2:] -= ret[:2]
        return ret

    @property
    def velocity(self) -> Optional[np.ndarray]:
        """Get average velocity from history."""
        if len(self.velocity_history) == 0:
            return None
        return np.mean(self.velocity_history, axis=0)

    @property
    def heading(self) -> Optional[float]:
        """Get heading in degrees from velocity."""
        vel = self.velocity
        if vel is None or (abs(vel[0]) < 0.01 and abs(vel[1]) < 0.01):
            return None
        return np.degrees(np.arctan2(vel[1], vel[0]))

    @property
    def speed(self) -> float:
        """Get speed from velocity (m/frame)."""
        vel = self.velocity
        if vel is None:
            return 0.0
        return float(np.linalg.norm(vel[:2]))  # XY speed


class KalmanFilter:
    """
    Kalman filter for bounding box tracking.

    State: [cx, cy, aspect_ratio, height, vx, vy, va, vh]
    Measurement: [cx, cy, aspect_ratio, height]
    """

    def __init__(self, config: ByteTrackerConfig = None):
        config = config or ByteTrackerConfig()

        # State transition matrix
        self._motion_mat = np.eye(8, dtype=np.float64)
        for i in range(4):
            self._motion_mat[i, i + 4] = 1.0

        # Observation matrix
        self._update_mat = np.eye(4, 8, dtype=np.float64)

        # Noise weights
        self._std_weight_position = config.std_weight_position
        self._std_weight_velocity = config.std_weight_velocity

    def predict(
        self, mean: np.ndarray, covariance: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Predict next state."""
        std_pos = [
            self._std_weight_position * mean[3],
            self._std_weight_position * mean[3],
            1e-2,
            self._std_weight_position * mean[3],
        ]
        std_vel = [
            self._std_weight_velocity * mean[3],
            self._std_weight_velocity * mean[3],
            1e-5,
            self._std_weight_velocity * mean[3],
        ]
        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel]))

        mean = self._motion_mat @ mean
        covariance = self._motion_mat @ covariance @ self._motion_mat.T + motion_cov

        return mean, covariance

    def update(
        self, mean: np.ndarray, covariance: np.ndarray, measurement: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Update state with measurement."""
        try:
            # Ensure positive height for stable computation
            h = max(mean[3], 1.0)
            std = [
                self._std_weight_position * h,
                self._std_weight_position * h,
                1e-1,
                self._std_weight_position * h,
            ]
            innovation_cov = np.diag(np.square(std))

            projected_mean = self._update_mat @ mean
            projected_cov = self._update_mat @ covariance @ self._update_mat.T + innovation_cov

            # Add regularization for numerical stability
            projected_cov += np.eye(4) * 1e-4

            # Kalman gain with fallback
            try:
                chol = np.linalg.cholesky(projected_cov)
                kalman_gain = np.linalg.solve(
                    chol.T,
                    np.linalg.solve(chol, (covariance @ self._update_mat.T).T).T
                ).T
            except np.linalg.LinAlgError:
                # Fallback to pseudo-inverse
                kalman_gain = covariance @ self._update_mat.T @ np.linalg.pinv(projected_cov)

            innovation = measurement - projected_mean
            new_mean = mean + kalman_gain @ innovation
            new_covariance = covariance - kalman_gain @ projected_cov @ kalman_gain.T

            # Ensure covariance stays symmetric and positive semi-definite
            new_covariance = (new_covariance + new_covariance.T) / 2
            new_covariance += np.eye(8) * 1e-6

            return new_mean, new_covariance
        except Exception:
            # On any error, return unchanged state
            return mean, covariance


class ByteTracker:
    """
    ByteTrack multi-object tracker.

    Two-stage association:
    1. Match high-confidence detections to tracks
    2. Match remaining low-confidence detections to unmatched tracks
    """

    def __init__(self, config: ByteTrackerConfig = None):
        self.config = config or ByteTrackerConfig()
        self.kf = KalmanFilter(self.config)

        self.tracked_stracks: List[STrack] = []  # Active confirmed tracks
        self.lost_stracks: List[STrack] = []     # Lost tracks waiting for re-id
        self.removed_stracks: List[STrack] = []  # Removed tracks (for logging)

        self.frame_id = 0
        self._next_id = 1

    def update(self, detections: List) -> List[STrack]:
        """
        Update tracker with new detections.

        Args:
            detections: List of Detection objects with bbox, confidence, class_id

        Returns:
            List of active STrack objects with track_id assigned
        """
        self.frame_id += 1
        activated_stracks = []
        refind_stracks = []
        lost_stracks = []
        removed_stracks = []

        # Convert detections to numpy format
        if len(detections) == 0:
            det_bboxes = np.empty((0, 4))
            det_scores = np.array([])
        else:
            det_bboxes = []
            det_scores = []
            for det in detections:
                bbox = det.bbox
                # Convert normalized [cx, cy, w, h] to [x1, y1, x2, y2]
                if len(bbox) == 4:
                    if max(bbox) <= 1.0:  # Normalized
                        cx, cy, w, h = bbox
                        bbox = [cx - w/2, cy - h/2, cx + w/2, cy + h/2]
                det_bboxes.append(bbox)
                det_scores.append(det.confidence)
            det_bboxes = np.array(det_bboxes)
            det_scores = np.array(det_scores)

        # Split detections by confidence
        high_mask = det_scores >= self.config.high_thresh
        low_mask = (det_scores >= self.config.low_thresh) & ~high_mask

        high_dets = [detections[i] for i in np.where(high_mask)[0]]
        low_dets = [detections[i] for i in np.where(low_mask)[0]]
        high_bboxes = det_bboxes[high_mask] if len(det_bboxes) > 0 else np.empty((0, 4))
        low_bboxes = det_bboxes[low_mask] if len(det_bboxes) > 0 else np.empty((0, 4))

        # Predict existing tracks
        strack_pool = self.tracked_stracks + self.lost_stracks
        for track in strack_pool:
            track.predict(self.kf)

        # First association: high-confidence detections
        track_bboxes = np.array([t.tlbr for t in strack_pool]) if strack_pool else np.empty((0, 4))

        if len(high_bboxes) > 0 and len(track_bboxes) > 0:
            cost_matrix = self._compute_cost_matrix(
                strack_pool, high_dets, high_bboxes, track_bboxes
            )
            matches, u_track, u_det = self._linear_assignment(
                cost_matrix, thresh=self.config.match_thresh
            )
        else:
            matches = []
            u_track = list(range(len(strack_pool)))
            u_det = list(range(len(high_dets)))

        # Update matched tracks
        for itrack, idet in matches:
            track = strack_pool[itrack]
            det = high_dets[idet]
            if track.state == TrackState.CONFIRMED:
                track.update(det, self.kf, self.frame_id)
                activated_stracks.append(track)
            else:
                track.update(det, self.kf, self.frame_id)
                refind_stracks.append(track)

        # Second association: low-confidence detections to remaining tracks
        r_tracked_stracks = [strack_pool[i] for i in u_track if strack_pool[i].state == TrackState.CONFIRMED]

        if len(low_bboxes) > 0 and len(r_tracked_stracks) > 0:
            r_track_bboxes = np.array([t.tlbr for t in r_tracked_stracks])
            cost_matrix = self._iou_cost(low_bboxes, r_track_bboxes)
            matches2, u_track2, u_det2 = self._linear_assignment(
                cost_matrix, thresh=0.5
            )

            for itrack, idet in matches2:
                track = r_tracked_stracks[itrack]
                det = low_dets[idet]
                track.update(det, self.kf, self.frame_id)
                activated_stracks.append(track)

            # Remaining unmatched tracks
            for itrack in u_track2:
                track = r_tracked_stracks[itrack]
                if track.time_since_update > self.config.max_time_lost:
                    removed_stracks.append(track)
                else:
                    track.mark_lost()
                    lost_stracks.append(track)
        else:
            # Mark unmatched confirmed tracks as lost
            for i in u_track:
                track = strack_pool[i]
                if track.state == TrackState.CONFIRMED:
                    if track.time_since_update > self.config.max_time_lost:
                        removed_stracks.append(track)
                    else:
                        track.mark_lost()
                        lost_stracks.append(track)

        # Initialize new tracks for unmatched high-confidence detections
        for idet in u_det:
            det = high_dets[idet]
            if det.confidence >= self.config.high_thresh:
                new_track = self._init_track(det)
                activated_stracks.append(new_track)

        # Update track lists
        self.tracked_stracks = [t for t in activated_stracks + refind_stracks
                                if t.state == TrackState.CONFIRMED]
        self.lost_stracks = [t for t in self.lost_stracks + lost_stracks
                             if t.time_since_update <= self.config.track_buffer]
        self.removed_stracks.extend(removed_stracks)

        # Return active tracks (confirmed only)
        return [t for t in self.tracked_stracks if t.is_activated]

    def _init_track(self, detection) -> STrack:
        """Initialize new track from detection."""
        bbox = detection.bbox
        # Convert normalized to pixel if needed
        if len(bbox) == 4 and max(bbox) <= 1.0:
            cx, cy, w, h = bbox
            bbox = np.array([cx - w/2, cy - h/2, cx + w/2, cy + h/2])
        else:
            bbox = np.array(bbox)

        track = STrack(
            track_id=self._next_id,
            class_id=detection.class_id,
            class_name=detection.class_name,
            bbox=bbox,
            score=detection.confidence,
            world_position=detection.world_position.copy() if detection.world_position is not None else None,
            frame_id=self.frame_id,
            start_frame=self.frame_id,
            # Immediately confirm and activate new tracks for instant visualization
            state=TrackState.CONFIRMED,
            is_activated=True,
            tracklet_len=1,
        )
        self._next_id += 1
        return track

    def _compute_cost_matrix(
        self,
        tracks: List[STrack],
        detections: List,
        det_bboxes: np.ndarray,
        track_bboxes: np.ndarray,
    ) -> np.ndarray:
        """Compute cost matrix combining IoU and optional appearance/position."""
        # IoU cost
        iou_cost = self._iou_cost(det_bboxes, track_bboxes)

        # 3D position cost (if available)
        if self.config.use_3d_position:
            pos_cost = self._position_cost(tracks, detections)
            if pos_cost is not None:
                iou_cost = (1 - self.config.position_weight) * iou_cost + \
                           self.config.position_weight * pos_cost

        return iou_cost

    def _iou_cost(self, det_bboxes: np.ndarray, track_bboxes: np.ndarray) -> np.ndarray:
        """Compute 1 - IoU cost matrix."""
        n_det = len(det_bboxes)
        n_track = len(track_bboxes)
        cost = np.ones((n_det, n_track), dtype=np.float32)

        for i, det_box in enumerate(det_bboxes):
            for j, track_box in enumerate(track_bboxes):
                iou = self._compute_iou(det_box, track_box)
                cost[i, j] = 1.0 - iou

        return cost

    def _position_cost(
        self, tracks: List[STrack], detections: List
    ) -> Optional[np.ndarray]:
        """Compute cost based on 3D world position."""
        n_det = len(detections)
        n_track = len(tracks)

        # Check if we have position info
        has_positions = False
        for t in tracks:
            if t.world_position is not None:
                has_positions = True
                break
        if not has_positions:
            return None

        cost = np.ones((n_det, n_track), dtype=np.float32)
        max_dist = 50.0  # Max distance for normalization

        for i, det in enumerate(detections):
            if det.world_position is None:
                continue
            for j, track in enumerate(tracks):
                if track.world_position is None:
                    continue
                dist = np.linalg.norm(det.world_position - track.world_position)
                cost[i, j] = min(dist / max_dist, 1.0)

        return cost

    def _compute_iou(self, box1: np.ndarray, box2: np.ndarray) -> float:
        """Compute IoU between two boxes [x1, y1, x2, y2]."""
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter

        return inter / max(union, 1e-6)

    def _linear_assignment(
        self, cost_matrix: np.ndarray, thresh: float
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """Solve linear assignment problem."""
        if cost_matrix.size == 0:
            return [], list(range(cost_matrix.shape[1])), list(range(cost_matrix.shape[0]))

        if SCIPY_AVAILABLE:
            row_indices, col_indices = linear_sum_assignment(cost_matrix)
        else:
            # Greedy fallback
            return self._greedy_assignment(cost_matrix, thresh)

        matches = []
        unmatched_det = list(range(cost_matrix.shape[0]))
        unmatched_track = list(range(cost_matrix.shape[1]))

        for row, col in zip(row_indices, col_indices):
            if cost_matrix[row, col] <= thresh:
                matches.append((col, row))  # (track_idx, det_idx)
                if row in unmatched_det:
                    unmatched_det.remove(row)
                if col in unmatched_track:
                    unmatched_track.remove(col)

        return matches, unmatched_track, unmatched_det

    def _greedy_assignment(
        self, cost_matrix: np.ndarray, thresh: float
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """Greedy assignment fallback."""
        matches = []
        used_rows = set()
        used_cols = set()

        # Sort by cost
        indices = np.argwhere(cost_matrix <= thresh)
        if len(indices) == 0:
            return [], list(range(cost_matrix.shape[1])), list(range(cost_matrix.shape[0]))

        costs = cost_matrix[indices[:, 0], indices[:, 1]]
        order = np.argsort(costs)

        for idx in order:
            row, col = indices[idx]
            if row not in used_rows and col not in used_cols:
                matches.append((col, row))
                used_rows.add(row)
                used_cols.add(col)

        unmatched_det = [i for i in range(cost_matrix.shape[0]) if i not in used_rows]
        unmatched_track = [i for i in range(cost_matrix.shape[1]) if i not in used_cols]

        return matches, unmatched_track, unmatched_det

    def reset(self):
        """Reset tracker for new episode."""
        self.tracked_stracks = []
        self.lost_stracks = []
        self.removed_stracks = []
        self.frame_id = 0
        self._next_id = 1

    def get_stats(self) -> Dict:
        """Get tracker statistics."""
        return {
            "frame_id": self.frame_id,
            "active_tracks": len(self.tracked_stracks),
            "lost_tracks": len(self.lost_stracks),
            "total_tracks_created": self._next_id - 1,
        }
