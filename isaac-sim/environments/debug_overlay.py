"""
Debug Overlay - Visual debugging for perception validation with temporal coherence.

Renders annotated frames showing ground truth vs model detections with track IDs
and temporal coherence visualization.

Features:
- Ground truth boxes in GREEN with "GT: {class}" labels
- Model detection boxes in per-track colors with track IDs
- Velocity arrows showing object motion
- Track trails showing recent position history
- Match lines between corresponding GT and detection boxes
- Summary statistics overlay with tracking metrics
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np
import time

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


def generate_track_colors(n_colors: int = 100) -> List[Tuple[int, int, int]]:
    """Generate visually distinct colors for track IDs."""
    colors = []
    for i in range(n_colors):
        hue = int(180 * i / n_colors)
        color = np.array([[[hue, 255, 200]]], dtype=np.uint8)
        if CV2_AVAILABLE:
            bgr = cv2.cvtColor(color, cv2.COLOR_HSV2BGR)[0, 0]
            colors.append((int(bgr[0]), int(bgr[1]), int(bgr[2])))
        else:
            colors.append((255, 255, 255))
    return colors


TRACK_COLORS = generate_track_colors(100)


@dataclass
class OverlayConfig:
    """Configuration for debug overlay."""
    gt_color: Tuple[int, int, int] = (0, 255, 0)
    det_color: Tuple[int, int, int] = (255, 100, 0)
    match_color: Tuple[int, int, int] = (0, 255, 255)
    text_color: Tuple[int, int, int] = (255, 255, 255)
    bg_color: Tuple[int, int, int] = (0, 0, 0)
    velocity_color: Tuple[int, int, int] = (0, 165, 255)

    box_thickness: int = 2
    font_scale: float = 0.5
    font_thickness: int = 1
    track_id_font_scale: float = 0.7
    track_id_font_thickness: int = 2

    show_track_ids: bool = True
    show_velocity_arrows: bool = True
    show_track_trails: bool = True
    trail_length: int = 10
    velocity_arrow_scale: float = 50.0
    match_iou_threshold: float = 0.3
    use_track_colors: bool = True

    output_dir: str = "/tmp/perception_debug"
    save_frames: bool = False
    max_saved_frames: int = 100


@dataclass
class MatchResult:
    """Result of matching ground truth to detections."""
    gt_idx: int
    det_idx: int
    iou: float


class DebugOverlay:
    """Renders debug overlays for perception validation with temporal coherence."""

    def __init__(self, config: OverlayConfig = None):
        self.config = config or OverlayConfig()
        self._frame_count = 0
        self._session_start = time.time()

        self._total_gt = 0
        self._total_det = 0
        self._total_matches = 0
        self._total_fp = 0
        self._total_fn = 0

        self._track_history: Dict[int, List[Tuple[float, float]]] = {}
        self._all_track_ids_seen: set = set()

        if not CV2_AVAILABLE:
            print("[DebugOverlay] Warning: OpenCV not available, overlay disabled")

    def render_with_tracks(
        self,
        image: np.ndarray,
        gt_detections: List,
        tracks: List,
        uav_info: Optional[Dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        """
        Render debug overlay with ByteTrack tracks.

        Args:
            image: RGB or BGR image (H, W, 3)
            gt_detections: List of ground truth Detection objects
            tracks: List of STrack objects from ByteTracker
            uav_info: Optional UAV state info

        Returns:
            Tuple of (annotated_image, statistics_dict)
        """
        if not CV2_AVAILABLE:
            return image, self._compute_stats(gt_detections, tracks, [])

        annotated = cv2.cvtColor(image.copy(), cv2.COLOR_RGB2BGR) if image.shape[2] == 3 else image[:, :, :3].copy()
        h, w = annotated.shape[:2]

        self._update_track_history(tracks)

        if self.config.show_track_trails:
            self._draw_track_trails(annotated)

        for det in gt_detections:
            self._draw_detection(annotated, det, w, h, self.config.gt_color, "GT")

        for track in tracks:
            color = self._get_track_color(track.track_id)
            self._draw_track(annotated, track, w, h, color)

        matches = self._match_gt_to_tracks(gt_detections, tracks, w, h)

        for match in matches:
            self._draw_match_line(annotated, gt_detections[match.gt_idx], tracks[match.det_idx], w, h, match.iou)

        stats = self._compute_stats(gt_detections, tracks, matches)
        self._draw_stats_overlay(annotated, stats, tracks, uav_info)

        self._frame_count += 1

        return cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB), stats

    def _get_track_color(self, track_id: int) -> Tuple[int, int, int]:
        """Get consistent color for track ID."""
        if self.config.use_track_colors:
            return TRACK_COLORS[track_id % len(TRACK_COLORS)]
        return self.config.det_color

    def _update_track_history(self, tracks: List) -> None:
        """Update position history for track trails."""
        active_ids = set()

        for track in tracks:
            track_id = track.track_id
            active_ids.add(track_id)
            self._all_track_ids_seen.add(track_id)

            bbox = track.tlbr if hasattr(track, 'tlbr') else track.bbox
            cx = (bbox[0] + bbox[2]) / 2
            cy = (bbox[1] + bbox[3]) / 2

            if track_id not in self._track_history:
                self._track_history[track_id] = []

            self._track_history[track_id].append((cx, cy))

            if len(self._track_history[track_id]) > self.config.trail_length:
                self._track_history[track_id].pop(0)

        # Clean up old tracks
        for old_id in set(self._track_history.keys()) - active_ids:
            if self._track_history[old_id]:
                self._track_history[old_id].pop(0)
            if not self._track_history[old_id]:
                del self._track_history[old_id]

    def _draw_track_trails(self, image: np.ndarray) -> None:
        """Draw trails showing track history."""
        for track_id, positions in self._track_history.items():
            if len(positions) < 2:
                continue

            color = self._get_track_color(track_id)
            trail_color = tuple(int(c * 0.5) for c in color)

            for i in range(1, len(positions)):
                alpha = i / len(positions)
                pt1 = (int(positions[i-1][0]), int(positions[i-1][1]))
                pt2 = (int(positions[i][0]), int(positions[i][1]))
                thickness = max(1, int(2 * alpha))
                cv2.line(image, pt1, pt2, trail_color, thickness, cv2.LINE_AA)

    def _draw_track(
        self,
        image: np.ndarray,
        track,
        img_w: int,
        img_h: int,
        color: Tuple[int, int, int],
    ) -> None:
        """Draw tracked detection with ID and velocity."""
        bbox = track.tlbr if hasattr(track, 'tlbr') else track.bbox
        x1, y1, x2, y2 = [max(0, min(d - 1, int(v))) for v, d in zip(bbox, [img_w, img_h, img_w, img_h])]

        cv2.rectangle(image, (x1, y1), (x2, y2), color, self.config.box_thickness)

        if self.config.show_track_ids:
            label = f"#{track.track_id}"
            id_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                         self.config.track_id_font_scale, self.config.track_id_font_thickness)
            cv2.rectangle(image, (x1, y1), (x1 + id_size[0] + 6, y1 + id_size[1] + 6), color, -1)
            cv2.putText(image, label, (x1 + 3, y1 + id_size[1] + 3),
                       cv2.FONT_HERSHEY_SIMPLEX, self.config.track_id_font_scale, (0, 0, 0),
                       self.config.track_id_font_thickness)

        # Info label
        label_parts = [track.class_name]
        if hasattr(track, 'score'):
            label_parts.append(f"{track.score:.2f}")
        label = " ".join(label_parts)

        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                        self.config.font_scale, self.config.font_thickness)
        label_y = min(y2 + label_size[1] + 5, img_h - 5)
        cv2.rectangle(image, (x1, y2), (x1 + label_size[0] + 5, label_y + 5), self.config.bg_color, -1)
        cv2.putText(image, label, (x1 + 2, label_y),
                   cv2.FONT_HERSHEY_SIMPLEX, self.config.font_scale, color, self.config.font_thickness)

        # Velocity arrow
        if self.config.show_velocity_arrows and hasattr(track, 'velocity'):
            velocity = track.velocity
            if velocity is not None and np.linalg.norm(velocity[:2]) > 0.01:
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                end_x = int(cx + velocity[0] * self.config.velocity_arrow_scale)
                end_y = int(cy + velocity[1] * self.config.velocity_arrow_scale)
                cv2.arrowedLine(image, (cx, cy), (end_x, end_y),
                               self.config.velocity_color, 2, cv2.LINE_AA, tipLength=0.3)

    def _draw_detection(
        self,
        image: np.ndarray,
        detection,
        img_w: int,
        img_h: int,
        color: Tuple[int, int, int],
        prefix: str,
    ) -> None:
        """Draw single detection box with label."""
        cx, cy, bw, bh = detection.bbox
        x1 = max(0, min(img_w - 1, int((cx - bw / 2) * img_w)))
        y1 = max(0, min(img_h - 1, int((cy - bh / 2) * img_h)))
        x2 = max(0, min(img_w - 1, int((cx + bw / 2) * img_w)))
        y2 = max(0, min(img_h - 1, int((cy + bh / 2) * img_h)))

        cv2.rectangle(image, (x1, y1), (x2, y2), color, self.config.box_thickness)

        label = f"{prefix}: {detection.class_name}"
        if detection.distance is not None:
            label += f" ({detection.distance:.0f}m)"

        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                        self.config.font_scale, self.config.font_thickness)
        label_y = max(y1 - 5, label_size[1] + 5)
        cv2.rectangle(image, (x1, label_y - label_size[1] - 5), (x1 + label_size[0] + 5, label_y + 5),
                     self.config.bg_color, -1)
        cv2.putText(image, label, (x1 + 2, label_y),
                   cv2.FONT_HERSHEY_SIMPLEX, self.config.font_scale, color, self.config.font_thickness)

    def _draw_match_line(
        self,
        image: np.ndarray,
        gt_det,
        track,
        img_w: int,
        img_h: int,
        iou: float,
    ) -> None:
        """Draw line connecting matched GT and track."""
        gt_cx = int(gt_det.bbox[0] * img_w)
        gt_cy = int(gt_det.bbox[1] * img_h)

        bbox = track.tlbr if hasattr(track, 'tlbr') else track.bbox
        det_cx = int((bbox[0] + bbox[2]) / 2)
        det_cy = int((bbox[1] + bbox[3]) / 2)

        cv2.line(image, (gt_cx, gt_cy), (det_cx, det_cy), self.config.match_color, 1, cv2.LINE_AA)

        mid_x, mid_y = (gt_cx + det_cx) // 2, (gt_cy + det_cy) // 2
        cv2.putText(image, f"IoU:{iou:.2f}", (mid_x, mid_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.config.match_color, 1)

    def _draw_stats_overlay(
        self,
        image: np.ndarray,
        stats: Dict,
        tracks: List,
        uav_info: Optional[Dict],
    ) -> None:
        """Draw statistics overlay."""
        h, w = image.shape[:2]

        lines = [
            f"Frame: {self._frame_count}",
            f"GT Objects: {stats['gt_count']}",
            f"Active Tracks: {stats['track_count']}",
            f"Matches: {stats['matches']}",
            f"FP: {stats['false_positives']} | FN: {stats['false_negatives']}",
        ]

        if stats['gt_count'] > 0:
            lines.append(f"Recall: {stats['matches'] / stats['gt_count']:.1%}")
        if stats['track_count'] > 0:
            lines.append(f"Precision: {stats['matches'] / stats['track_count']:.1%}")

        if tracks:
            avg_len = np.mean([t.tracklet_len for t in tracks if hasattr(t, 'tracklet_len')])
            lines.append(f"Avg Track Len: {avg_len:.1f}")

        if uav_info and 'position' in uav_info:
            pos = uav_info['position']
            lines.append(f"UAV: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

        padding = 10
        line_height = 18
        box_height = len(lines) * line_height + padding * 2
        box_width = 200
        box_y_start = h - box_height - padding

        overlay = image.copy()
        cv2.rectangle(overlay, (padding, box_y_start), (box_width + padding, h - padding), self.config.bg_color, -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)

        y = box_y_start + line_height
        for line in lines:
            cv2.putText(image, line, (padding + 5, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.config.text_color, 1)
            y += line_height

    def _match_gt_to_tracks(
        self,
        gt_detections: List,
        tracks: List,
        img_w: int,
        img_h: int,
    ) -> List[MatchResult]:
        """Match ground truth to tracks by IoU."""
        matches = []
        used_tracks = set()

        for gt_idx, gt_det in enumerate(gt_detections):
            best_iou = 0.0
            best_track_idx = -1

            # Convert GT to pixel coords
            gt_cx, gt_cy, gt_bw, gt_bh = gt_det.bbox
            gt_x1 = (gt_cx - gt_bw / 2) * img_w
            gt_y1 = (gt_cy - gt_bh / 2) * img_h
            gt_x2 = (gt_cx + gt_bw / 2) * img_w
            gt_y2 = (gt_cy + gt_bh / 2) * img_h

            for track_idx, track in enumerate(tracks):
                if track_idx in used_tracks:
                    continue

                bbox = track.tlbr if hasattr(track, 'tlbr') else track.bbox
                t_x1, t_y1, t_x2, t_y2 = bbox

                iou = self._compute_iou_pixels(gt_x1, gt_y1, gt_x2, gt_y2, t_x1, t_y1, t_x2, t_y2)

                if iou > best_iou and iou >= self.config.match_iou_threshold:
                    best_iou = iou
                    best_track_idx = track_idx

            if best_track_idx >= 0:
                matches.append(MatchResult(gt_idx=gt_idx, det_idx=best_track_idx, iou=best_iou))
                used_tracks.add(best_track_idx)

        return matches

    def _compute_iou_pixels(
        self,
        x1_a: float, y1_a: float, x2_a: float, y2_a: float,
        x1_b: float, y1_b: float, x2_b: float, y2_b: float,
    ) -> float:
        """Compute IoU between two bboxes in pixel coordinates."""
        xi1, yi1 = max(x1_a, x1_b), max(y1_a, y1_b)
        xi2, yi2 = min(x2_a, x2_b), min(y2_a, y2_b)

        inter_w = max(0, xi2 - xi1)
        inter_h = max(0, yi2 - yi1)
        intersection = inter_w * inter_h

        area_a = (x2_a - x1_a) * (y2_a - y1_a)
        area_b = (x2_b - x1_b) * (y2_b - y1_b)
        union = area_a + area_b - intersection

        return intersection / union if union > 0 else 0.0

    def _compute_stats(
        self,
        gt_detections: List,
        tracks: List,
        matches: List[MatchResult],
    ) -> Dict:
        """Compute detection statistics."""
        gt_count = len(gt_detections)
        track_count = len(tracks)
        match_count = len(matches)

        self._total_gt += gt_count
        self._total_det += track_count
        self._total_matches += match_count
        self._total_fp += track_count - match_count
        self._total_fn += gt_count - match_count

        return {
            "gt_count": gt_count,
            "track_count": track_count,
            "det_count": track_count,
            "matches": match_count,
            "false_positives": track_count - match_count,
            "false_negatives": gt_count - match_count,
            "frame": self._frame_count,
        }

    def get_session_stats(self) -> Dict:
        """Get accumulated statistics for entire session."""
        elapsed = time.time() - self._session_start
        fps = self._frame_count / elapsed if elapsed > 0 else 0

        precision = self._total_matches / self._total_det if self._total_det > 0 else 0.0
        recall = self._total_matches / self._total_gt if self._total_gt > 0 else 0.0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

        return {
            "frames_processed": self._frame_count,
            "elapsed_seconds": elapsed,
            "fps": fps,
            "total_gt": self._total_gt,
            "total_detections": self._total_det,
            "total_matches": self._total_matches,
            "total_false_positives": self._total_fp,
            "total_false_negatives": self._total_fn,
            "precision": precision,
            "recall": recall,
            "f1_score": f1,
            "unique_tracks": len(self._all_track_ids_seen),
        }

    def reset(self) -> None:
        """Reset statistics for new session."""
        self._frame_count = 0
        self._session_start = time.time()
        self._total_gt = 0
        self._total_det = 0
        self._total_matches = 0
        self._total_fp = 0
        self._total_fn = 0
        self._track_history.clear()
        self._all_track_ids_seen.clear()
