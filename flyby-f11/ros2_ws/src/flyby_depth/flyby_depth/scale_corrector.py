"""
Scale correction for monocular depth estimation using rangefinder measurements.

The rangefinder provides ground truth metric distance at the image center,
which is used to convert relative depth values to metric depth.

Scale factor: metric_depth = relative_depth * scale_factor
where scale_factor = rangefinder_reading / depth_at_center
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class ScaleEstimate:
    """Result of scale estimation."""
    scale_factor: float
    confidence: float  # 0-1, based on depth consistency at center
    rangefinder_reading: float
    depth_at_center: float
    valid: bool


class ScaleCorrector:
    """
    Estimates metric scale from rangefinder readings.

    Uses exponential moving average to smooth scale estimates
    and detect/reject outliers.
    """

    def __init__(
        self,
        alpha: float = 0.3,
        min_valid_range: float = 0.5,
        max_valid_range: float = 1200.0,
        center_region: float = 0.1,
        outlier_threshold: float = 0.5,
    ):
        """
        Initialize scale corrector.

        Args:
            alpha: EMA filter coefficient (0-1, higher = more responsive)
            min_valid_range: Minimum valid rangefinder reading (meters)
            max_valid_range: Maximum valid rangefinder reading (meters)
            center_region: Fraction of image center to sample for depth
            outlier_threshold: Reject scale changes larger than this fraction
        """
        self.alpha = alpha
        self.min_valid_range = min_valid_range
        self.max_valid_range = max_valid_range
        self.center_region = center_region
        self.outlier_threshold = outlier_threshold

        # State
        self._scale_factor: Optional[float] = None
        self._confidence: float = 0.0
        self._history: list = []
        self._max_history = 10

    def update(
        self,
        relative_depth: np.ndarray,
        rangefinder_reading: float,
        rangefinder_valid: bool = True,
    ) -> ScaleEstimate:
        """
        Update scale estimate with new depth map and rangefinder reading.

        Args:
            relative_depth: Relative depth map from neural network (H, W)
            rangefinder_reading: Distance from rangefinder (meters)
            rangefinder_valid: Whether rangefinder reading is valid

        Returns:
            ScaleEstimate with current scale factor and confidence
        """
        # Extract depth at image center (where rangefinder points)
        depth_at_center = self._sample_center_depth(relative_depth)

        # Check validity
        valid = (
            rangefinder_valid
            and self.min_valid_range <= rangefinder_reading <= self.max_valid_range
            and depth_at_center > 0.01  # Non-zero depth at center
        )

        if not valid:
            # Return current estimate without update
            return ScaleEstimate(
                scale_factor=self._scale_factor or 1.0,
                confidence=self._confidence * 0.95,  # Decay confidence
                rangefinder_reading=rangefinder_reading,
                depth_at_center=depth_at_center,
                valid=False,
            )

        # Compute instantaneous scale
        instant_scale = rangefinder_reading / depth_at_center

        # Check for outliers
        if self._scale_factor is not None:
            relative_change = abs(instant_scale - self._scale_factor) / self._scale_factor
            if relative_change > self.outlier_threshold:
                # Large change - reduce weight
                effective_alpha = self.alpha * 0.1
            else:
                effective_alpha = self.alpha
        else:
            # First estimate
            effective_alpha = 1.0

        # Update scale with EMA
        if self._scale_factor is None:
            self._scale_factor = instant_scale
        else:
            self._scale_factor = (
                effective_alpha * instant_scale
                + (1 - effective_alpha) * self._scale_factor
            )

        # Update confidence based on depth consistency
        confidence = self._compute_confidence(relative_depth, depth_at_center)
        self._confidence = 0.8 * confidence + 0.2 * self._confidence

        # Track history for statistics
        self._history.append(instant_scale)
        if len(self._history) > self._max_history:
            self._history.pop(0)

        return ScaleEstimate(
            scale_factor=self._scale_factor,
            confidence=self._confidence,
            rangefinder_reading=rangefinder_reading,
            depth_at_center=depth_at_center,
            valid=True,
        )

    def apply_scale(
        self,
        relative_depth: np.ndarray,
        min_depth: float = 0.5,
        max_depth: float = 100.0,
    ) -> np.ndarray:
        """
        Apply current scale factor to convert relative depth to metric.

        Args:
            relative_depth: Relative depth map (H, W)
            min_depth: Minimum output depth (meters)
            max_depth: Maximum output depth (meters)

        Returns:
            Metric depth map (H, W) in meters
        """
        scale = self._scale_factor if self._scale_factor is not None else 1.0
        metric_depth = relative_depth * scale
        return np.clip(metric_depth, min_depth, max_depth).astype(np.float32)

    def _sample_center_depth(self, depth: np.ndarray) -> float:
        """
        Sample depth at image center region.

        Uses median of center region for robustness to noise.
        """
        h, w = depth.shape[:2]
        margin_h = int(h * self.center_region / 2)
        margin_w = int(w * self.center_region / 2)

        center_h = h // 2
        center_w = w // 2

        # Extract center region
        region = depth[
            center_h - margin_h : center_h + margin_h + 1,
            center_w - margin_w : center_w + margin_w + 1,
        ]

        # Use median for robustness
        valid_mask = region > 0.01
        if valid_mask.sum() > 0:
            return float(np.median(region[valid_mask]))
        return 0.0

    def _compute_confidence(
        self, depth: np.ndarray, center_depth: float
    ) -> float:
        """
        Compute confidence based on depth consistency at center.

        Low variance at center = high confidence in scale.
        """
        h, w = depth.shape[:2]
        margin_h = int(h * self.center_region / 2)
        margin_w = int(w * self.center_region / 2)

        center_h = h // 2
        center_w = w // 2

        region = depth[
            center_h - margin_h : center_h + margin_h + 1,
            center_w - margin_w : center_w + margin_w + 1,
        ]

        valid_mask = region > 0.01
        if valid_mask.sum() < 10:
            return 0.0

        # Coefficient of variation (lower = more consistent)
        std = np.std(region[valid_mask])
        mean = np.mean(region[valid_mask])
        if mean < 0.01:
            return 0.0

        cv = std / mean
        # Map CV to confidence (CV of 0 = confidence 1, CV of 0.5+ = confidence 0)
        confidence = max(0.0, 1.0 - cv * 2)
        return confidence

    @property
    def scale_factor(self) -> float:
        """Current scale factor."""
        return self._scale_factor if self._scale_factor is not None else 1.0

    @property
    def confidence(self) -> float:
        """Current confidence in scale estimate."""
        return self._confidence

    @property
    def is_initialized(self) -> bool:
        """Whether scale has been initialized."""
        return self._scale_factor is not None

    def reset(self):
        """Reset scale estimator state."""
        self._scale_factor = None
        self._confidence = 0.0
        self._history.clear()
