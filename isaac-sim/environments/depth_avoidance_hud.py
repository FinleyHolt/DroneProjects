"""
Depth and Avoidance HUD Renderer for Navigation Tests.

Provides split-screen visualization with:
- RGB camera view (left 60%)
- Colorized depth map (right 40%)
- Telemetry overlays (scale, rangefinder, safety status)
- VFH+ recommendation visualization
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


@dataclass
class DepthData:
    """Data from DepthMap message for HUD display."""
    scale_factor: float = 1.0
    scale_confidence: float = 0.0
    rangefinder_reading: float = 0.0
    rangefinder_valid: bool = False
    min_depth: float = 0.0
    max_depth: float = 100.0
    mean_depth: float = 50.0
    inference_time_ms: float = 0.0


@dataclass
class AvoidanceData:
    """Data from ObstacleMap message for HUD display."""
    closest_obstacle_distance: float = float('inf')
    path_blocked: bool = False
    recommended_heading: float = 0.0
    recommended_pitch: float = 0.0
    recommendation_confidence: float = 0.0
    blocked_bins: int = 0
    total_bins: int = 36
    candidate_directions: int = 0
    safety_level: int = 0  # 0=CLEAR, 1=CAUTION, 2=WARNING, 3=CRITICAL
    processing_time_ms: float = 0.0


@dataclass
class TelemetryData:
    """UAV telemetry for HUD display."""
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    heading_deg: float = 0.0
    goal_distance: float = 0.0
    step: int = 0
    sim_time: float = 0.0


class DepthAvoidanceHUD:
    """
    Renders split-screen HUD with depth and avoidance visualization.

    Layout:
    +----------------------------------------------+----------------------------------+
    |  [RGB CAMERA VIEW - 60%]                    |  [DEPTH COLORIZED - 40%]        |
    |                                              |  Scale: 1.23 (conf: 0.92)       |
    |                                              |  Range: 45.2m                   |
    +----------------------------------------------+----------------------------------+
    |  POS: (125, -12, 30)  HDG: 045°  GOAL: 175m  |  FPS: Depth 12.3 | VFH 18.7    |
    +--------------------------------------------------------------------------------+
    |  SAFETY: [CAUTION]     |  DEPTH STATS        |  VFH+ RECOMMENDATION           |
    |  Closest: 18.5m        |  Min: 12.3m         |  Heading: +15.2°              |
    |  Path: BLOCKED         |  Max: 89.4m         |  Confidence: 0.87              |
    |  Blocked: 8/36 bins    |  Mean: 45.6m        |  Candidates: 12                |
    +------------------------+---------------------+--------------------------------+
    """

    # Safety level colors (BGR for OpenCV)
    SAFETY_COLORS = {
        0: (0, 255, 0),      # CLEAR - Green
        1: (0, 255, 255),    # CAUTION - Yellow
        2: (0, 165, 255),    # WARNING - Orange
        3: (0, 0, 255),      # CRITICAL - Red
    }

    SAFETY_NAMES = {
        0: "CLEAR",
        1: "CAUTION",
        2: "WARNING",
        3: "CRITICAL",
    }

    def __init__(self, output_resolution: Tuple[int, int] = (1280, 720)):
        """
        Initialize HUD renderer.

        Args:
            output_resolution: Final frame resolution (width, height)
        """
        self.output_width, self.output_height = output_resolution
        self.rgb_width = int(self.output_width * 0.6)
        self.depth_width = self.output_width - self.rgb_width

        # Reserve bottom for telemetry panels
        self.main_height = int(self.output_height * 0.75)
        self.panel_height = self.output_height - self.main_height

    def colorize_depth(
        self,
        depth: np.ndarray,
        min_depth: float = 0.5,
        max_depth: float = 100.0,
    ) -> np.ndarray:
        """
        Convert depth map to colorized visualization.

        Args:
            depth: Depth map (H, W) in meters
            min_depth: Minimum depth for colormap
            max_depth: Maximum depth for colormap

        Returns:
            RGB colorized depth (H, W, 3)
        """
        if not CV2_AVAILABLE:
            # Return grayscale fallback
            normalized = np.clip((depth - min_depth) / (max_depth - min_depth), 0, 1)
            gray = (normalized * 255).astype(np.uint8)
            return np.stack([gray, gray, gray], axis=-1)

        # Normalize to 0-255
        normalized = np.clip((depth - min_depth) / (max_depth - min_depth), 0, 1)
        depth_uint8 = (normalized * 255).astype(np.uint8)

        # Apply turbo colormap (warm=close, cool=far)
        # Keep as BGR since all cv2 drawing functions use BGR
        colorized = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_TURBO)
        return colorized

    def render(
        self,
        bgr_image: np.ndarray,
        depth_image: Optional[np.ndarray],
        depth_data: DepthData,
        avoidance_data: AvoidanceData,
        telemetry: TelemetryData,
    ) -> np.ndarray:
        """
        Render complete HUD frame.

        Args:
            bgr_image: BGR camera/view image (H, W, 3) - cv2 native format
            depth_image: Depth map (H, W) in meters, or None
            depth_data: Depth estimation statistics
            avoidance_data: VFH+ avoidance data
            telemetry: UAV telemetry

        Returns:
            Composited HUD frame in BGR format (output_height, output_width, 3)
        """
        if not CV2_AVAILABLE:
            # Return image resized if no OpenCV
            return bgr_image

        # Create output frame
        frame = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)

        # Resize and place BGR image (left 60%)
        bgr_resized = cv2.resize(bgr_image, (self.rgb_width, self.main_height))
        frame[:self.main_height, :self.rgb_width] = bgr_resized

        # Colorize and place depth image (right 40%)
        if depth_image is not None:
            depth_colored = self.colorize_depth(
                depth_image,
                min_depth=depth_data.min_depth,
                max_depth=depth_data.max_depth,
            )
            depth_resized = cv2.resize(depth_colored, (self.depth_width, self.main_height))
        else:
            # Placeholder if no depth
            depth_resized = np.zeros((self.main_height, self.depth_width, 3), dtype=np.uint8)
            cv2.putText(depth_resized, "NO DEPTH", (50, self.main_height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2)

        frame[:self.main_height, self.rgb_width:] = depth_resized

        # Draw depth panel overlays
        self._draw_depth_overlay(frame, depth_data)

        # Draw bottom telemetry panels
        self._draw_telemetry_bar(frame, telemetry, depth_data, avoidance_data)
        self._draw_info_panels(frame, depth_data, avoidance_data)

        return frame

    def _draw_depth_overlay(self, frame: np.ndarray, depth_data: DepthData):
        """Draw overlays on depth panel including color scale legend."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        white = (255, 255, 255)

        x_offset = self.rgb_width + 10
        y = 25

        # Scale factor
        conf_color = (0, 255, 0) if depth_data.scale_confidence > 0.7 else (0, 255, 255)
        cv2.putText(frame, f"Scale: {depth_data.scale_factor:.2f} (conf: {depth_data.scale_confidence:.2f})",
                    (x_offset, y), font, font_scale, conf_color, thickness)

        # Rangefinder reading
        y += 25
        range_color = white if depth_data.rangefinder_valid else (128, 128, 128)
        valid_str = "" if depth_data.rangefinder_valid else " [INVALID]"
        cv2.putText(frame, f"Range: {depth_data.rangefinder_reading:.1f}m{valid_str}",
                    (x_offset, y), font, font_scale, range_color, thickness)

        # Draw depth color scale legend on right edge
        self._draw_depth_scale_legend(frame, depth_data)

    def _draw_depth_scale_legend(self, frame: np.ndarray, depth_data: DepthData):
        """Draw a vertical color scale bar showing depth range."""
        # Position: right edge of depth panel, leaving some margin
        bar_width = 20
        bar_height = self.main_height - 80  # Leave space for labels
        bar_x = self.output_width - bar_width - 10
        bar_y_start = 60

        # Create color gradient (turbo colormap: 0=red/close, 255=blue/far)
        for i in range(bar_height):
            # Map position to colormap value (top=far/blue, bottom=close/red)
            val = int(255 * (1.0 - i / bar_height))
            color_img = np.array([[val]], dtype=np.uint8)
            color_bgr = cv2.applyColorMap(color_img, cv2.COLORMAP_TURBO)[0, 0]
            color = tuple(int(c) for c in color_bgr)
            cv2.line(frame, (bar_x, bar_y_start + i), (bar_x + bar_width, bar_y_start + i), color, 1)

        # Draw border
        cv2.rectangle(frame, (bar_x, bar_y_start), (bar_x + bar_width, bar_y_start + bar_height),
                      (255, 255, 255), 1)

        # Draw labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        white = (255, 255, 255)

        # Far label (top) - blue
        far_label = f"{depth_data.max_depth:.0f}m"
        cv2.putText(frame, far_label, (bar_x - 35, bar_y_start + 5), font, font_scale, white, 1)

        # Close label (bottom) - red
        close_label = f"{depth_data.min_depth:.0f}m"
        cv2.putText(frame, close_label, (bar_x - 35, bar_y_start + bar_height), font, font_scale, white, 1)

        # Middle label
        mid_depth = (depth_data.min_depth + depth_data.max_depth) / 2
        mid_label = f"{mid_depth:.0f}m"
        cv2.putText(frame, mid_label, (bar_x - 35, bar_y_start + bar_height // 2), font, font_scale, white, 1)

    def _draw_telemetry_bar(
        self,
        frame: np.ndarray,
        telemetry: TelemetryData,
        depth_data: DepthData,
        avoidance_data: AvoidanceData,
    ):
        """Draw top telemetry bar below main view."""
        bar_y = self.main_height
        bar_height = 30

        # Background
        cv2.rectangle(frame, (0, bar_y), (self.output_width, bar_y + bar_height),
                      (40, 40, 40), -1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        white = (255, 255, 255)
        cyan = (255, 255, 0)

        # Position
        pos = telemetry.position
        cv2.putText(frame, f"POS: ({pos[0]:.0f}, {pos[1]:.0f}, {pos[2]:.0f})",
                    (10, bar_y + 20), font, font_scale, white, thickness)

        # Heading
        cv2.putText(frame, f"HDG: {telemetry.heading_deg:03.0f}",
                    (200, bar_y + 20), font, font_scale, white, thickness)

        # Goal distance
        cv2.putText(frame, f"GOAL: {telemetry.goal_distance:.0f}m",
                    (300, bar_y + 20), font, font_scale, cyan, thickness)

        # Time
        cv2.putText(frame, f"T+{telemetry.sim_time:.1f}s",
                    (420, bar_y + 20), font, font_scale, white, thickness)

        # Step
        cv2.putText(frame, f"Step: {telemetry.step}",
                    (520, bar_y + 20), font, font_scale, white, thickness)

        # FPS indicators (right side)
        depth_fps = 1000.0 / max(depth_data.inference_time_ms, 1.0)
        vfh_fps = 1000.0 / max(avoidance_data.processing_time_ms, 1.0)
        cv2.putText(frame, f"FPS: Depth {depth_fps:.1f} | VFH {vfh_fps:.1f}",
                    (self.output_width - 250, bar_y + 20), font, font_scale, white, thickness)

    def _draw_info_panels(
        self,
        frame: np.ndarray,
        depth_data: DepthData,
        avoidance_data: AvoidanceData,
    ):
        """Draw bottom info panels."""
        panel_y = self.main_height + 30
        panel_width = self.output_width // 3

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        thickness = 1
        white = (255, 255, 255)

        # Panel 1: Safety Status
        self._draw_safety_panel(frame, panel_y, 0, panel_width, avoidance_data)

        # Panel 2: Depth Stats
        self._draw_depth_stats_panel(frame, panel_y, panel_width, panel_width, depth_data)

        # Panel 3: VFH+ Recommendation
        self._draw_vfh_panel(frame, panel_y, panel_width * 2, panel_width, avoidance_data)

    def _draw_safety_panel(
        self,
        frame: np.ndarray,
        y: int,
        x: int,
        width: int,
        avoidance_data: AvoidanceData,
    ):
        """Draw safety status panel."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        thickness = 1

        safety_color = self.SAFETY_COLORS.get(avoidance_data.safety_level, (128, 128, 128))
        safety_name = self.SAFETY_NAMES.get(avoidance_data.safety_level, "UNKNOWN")

        # Safety level (BGR for OpenCV)
        cv2.putText(frame, f"SAFETY: [{safety_name}]",
                    (x + 10, y + 20), font, font_scale, safety_color, thickness)

        # Closest obstacle
        white = (255, 255, 255)
        dist = avoidance_data.closest_obstacle_distance
        dist_str = f"{dist:.1f}m" if dist < 1000 else "---"
        cv2.putText(frame, f"Closest: {dist_str}",
                    (x + 10, y + 40), font, font_scale, white, thickness)

        # Path status
        path_color = (0, 0, 255) if avoidance_data.path_blocked else (0, 255, 0)
        path_str = "BLOCKED" if avoidance_data.path_blocked else "CLEAR"
        cv2.putText(frame, f"Path: {path_str}",
                    (x + 10, y + 60), font, font_scale, path_color, thickness)

        # Blocked bins
        cv2.putText(frame, f"Blocked: {avoidance_data.blocked_bins}/{avoidance_data.total_bins} bins",
                    (x + 10, y + 80), font, font_scale, white, thickness)

    def _draw_depth_stats_panel(
        self,
        frame: np.ndarray,
        y: int,
        x: int,
        width: int,
        depth_data: DepthData,
    ):
        """Draw depth statistics panel."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        thickness = 1
        white = (255, 255, 255)
        cyan = (255, 255, 0)

        cv2.putText(frame, "DEPTH STATS",
                    (x + 10, y + 20), font, font_scale, cyan, thickness)

        cv2.putText(frame, f"Min: {depth_data.min_depth:.1f}m",
                    (x + 10, y + 40), font, font_scale, white, thickness)

        cv2.putText(frame, f"Max: {depth_data.max_depth:.1f}m",
                    (x + 10, y + 60), font, font_scale, white, thickness)

        cv2.putText(frame, f"Mean: {depth_data.mean_depth:.1f}m",
                    (x + 10, y + 80), font, font_scale, white, thickness)

    def _draw_vfh_panel(
        self,
        frame: np.ndarray,
        y: int,
        x: int,
        width: int,
        avoidance_data: AvoidanceData,
    ):
        """Draw VFH+ recommendation panel."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        thickness = 1
        white = (255, 255, 255)
        cyan = (255, 255, 0)

        cv2.putText(frame, "VFH+ RECOMMENDATION",
                    (x + 10, y + 20), font, font_scale, cyan, thickness)

        # Heading recommendation with arrow indicator
        heading_deg = np.degrees(avoidance_data.recommended_heading)
        direction = "" if abs(heading_deg) < 0.1 else ("" if heading_deg > 0 else "")
        cv2.putText(frame, f"Heading: {heading_deg:+.1f} {direction}",
                    (x + 10, y + 40), font, font_scale, white, thickness)

        cv2.putText(frame, f"Confidence: {avoidance_data.recommendation_confidence:.2f}",
                    (x + 10, y + 60), font, font_scale, white, thickness)

        cv2.putText(frame, f"Candidates: {avoidance_data.candidate_directions}",
                    (x + 10, y + 80), font, font_scale, white, thickness)
