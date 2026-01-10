"""
MVP scale correction with warmup-then-freeze strategy.

Simplified alternative to affine_depth_corrector.py that avoids the broken
least-squares fallback when depth values don't vary enough.

Strategy:
1. Collect (model_center, rangefinder) pairs during warmup
2. Compute scale via median ratio when sufficient diverse samples
3. Freeze scale for remainder of flight
4. EMA fallback if warmup fails to freeze

This approach:
- Uses scale-only (no shift) - sufficient for similar scene types
- Uses median ratio (robust to outliers, can't fail from rank deficiency)
- Freezes after warmup (no runtime estimation failures)

Author: Finley Holt
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class ScaleEstimate:
    """Result of scale estimation."""
    scale: float
    confidence: float
    rangefinder_reading: float
    model_center: float
    frozen: bool
    valid: bool


class WarmupScaleCorrector:
    """
    Simple scale-only correction with warmup-then-freeze.

    During warmup, collects (model_center, rangefinder) sample pairs.
    Once sufficient diverse samples are collected, computes scale via
    median ratio and freezes it for the remainder of the flight.

    If warmup fails to freeze (insufficient variation), falls back to
    slow EMA updates.
    """

    def __init__(
        self,
        warmup_frames: int = 60,       # 2 seconds at 30Hz
        min_samples: int = 20,
        min_depth_cv: float = 0.02,    # 2% coefficient of variation (relaxed from 5%)
        alpha: float = 0.1,            # EMA for fallback mode
        min_valid_range: float = 1.0,  # Minimum rangefinder reading (m)
        max_valid_range: float = 200.0,  # Maximum rangefinder reading (m)
        center_region: float = 0.1,    # Fraction of image center to sample
    ):
        """
        Initialize MVP scale corrector.

        Args:
            warmup_frames: Maximum frames to collect warmup samples
            min_samples: Minimum samples required before attempting freeze
            min_depth_cv: Minimum coefficient of variation required to freeze
            alpha: EMA coefficient for fallback mode (lower = more stable)
            min_valid_range: Minimum valid rangefinder reading
            max_valid_range: Maximum valid rangefinder reading
            center_region: Fraction of image center to sample for model output
        """
        self.warmup_frames = warmup_frames
        self.min_samples = min_samples
        self.min_depth_cv = min_depth_cv
        self.alpha = alpha
        self.min_valid_range = min_valid_range
        self.max_valid_range = max_valid_range
        self.center_region = center_region

        # State
        self._warmup_samples: List[Tuple[float, float]] = []  # (model_center, range)
        self._frame_count: int = 0
        self._scale: float = 1.0
        self._frozen: bool = False
        self._confidence: float = 0.0
        self._warmup_complete: bool = False

    def update(
        self,
        model_output: np.ndarray,
        rangefinder_reading: float,
        rangefinder_valid: bool = True,
    ) -> ScaleEstimate:
        """
        Update scale estimate with new depth map and rangefinder reading.

        Args:
            model_output: Raw model output (H, W), normalized 0-1
            rangefinder_reading: Distance from rangefinder (meters)
            rangefinder_valid: Whether rangefinder reading is valid

        Returns:
            ScaleEstimate with current scale and status
        """
        self._frame_count += 1

        # Sample model output at center
        d_model = self._sample_center(model_output)

        # Validity checks
        valid = (
            rangefinder_valid
            and self.min_valid_range <= rangefinder_reading <= self.max_valid_range
            and d_model > 0.01
        )

        if not valid:
            # Decay confidence when readings invalid
            self._confidence *= 0.98
            return ScaleEstimate(
                scale=self._scale,
                confidence=self._confidence,
                rangefinder_reading=rangefinder_reading,
                model_center=d_model,
                frozen=self._frozen,
                valid=False,
            )

        if self._frozen:
            # Already frozen - just return current scale
            return ScaleEstimate(
                scale=self._scale,
                confidence=self._confidence,
                rangefinder_reading=rangefinder_reading,
                model_center=d_model,
                frozen=True,
                valid=True,
            )

        # Warmup phase - collect samples
        if self._frame_count <= self.warmup_frames:
            self._warmup_samples.append((d_model, rangefinder_reading))

            # Try to compute and freeze scale once we have enough samples
            if len(self._warmup_samples) >= self.min_samples:
                scale, success = self._compute_warmup_scale()
                if success:
                    self._scale = scale
                    self._frozen = True
                    self._confidence = 0.95
                    self._warmup_complete = True
                    print(f"[MVP Scale] FROZEN: scale={scale:.3f} "
                          f"(frame {self._frame_count}, {len(self._warmup_samples)} samples)")

        else:
            # Warmup period ended without freezing
            if not self._warmup_complete:
                print(f"[MVP Scale] WARNING: Warmup ended without freeze "
                      f"({len(self._warmup_samples)} samples) - using EMA fallback")
                self._warmup_complete = True

            # EMA fallback mode
            instant_scale = rangefinder_reading / d_model
            if 0.1 < instant_scale < 500:
                self._scale = self.alpha * instant_scale + (1 - self.alpha) * self._scale
                self._confidence = min(0.7, self._confidence + 0.02)

        return ScaleEstimate(
            scale=self._scale,
            confidence=self._confidence,
            rangefinder_reading=rangefinder_reading,
            model_center=d_model,
            frozen=self._frozen,
            valid=True,
        )

    def _compute_warmup_scale(self) -> Tuple[float, bool]:
        """
        Compute scale from warmup samples using median ratio.

        Returns:
            (scale, success) - scale factor and whether computation succeeded
        """
        if len(self._warmup_samples) < self.min_samples:
            return self._scale, False

        model_vals = np.array([s[0] for s in self._warmup_samples])
        range_vals = np.array([s[1] for s in self._warmup_samples])

        # Check coefficient of variation - need sufficient depth variation
        mean_model = np.mean(model_vals)
        if mean_model < 0.01:
            print(f"[MVP Scale] Reject: mean_model={mean_model:.4f} too small")
            return self._scale, False

        cv = np.std(model_vals) / mean_model

        # Relaxed CV threshold: even 1% variation can give reasonable scale
        # At long range, model values are naturally more consistent
        if cv < self.min_depth_cv:
            # Log but don't reject immediately if we have many samples
            # With 20+ samples, even low CV can give decent estimate
            if len(self._warmup_samples) >= 40:
                print(f"[MVP Scale] Low CV={cv:.4f} but have {len(self._warmup_samples)} samples, proceeding")
            else:
                print(f"[MVP Scale] Reject: CV={cv:.4f} < {self.min_depth_cv} (samples={len(self._warmup_samples)})")
                return self._scale, False

        # Compute scale using median ratio (robust to outliers)
        ratios = range_vals / (model_vals + 1e-6)
        scale = float(np.median(ratios))

        # Debug: show range of ratios
        print(f"[MVP Scale] Ratios: min={np.min(ratios):.1f} max={np.max(ratios):.1f} "
              f"median={scale:.1f} CV={cv:.4f}")

        # Sanity check
        if scale <= 0 or scale > 500:
            print(f"[MVP Scale] Reject: scale={scale:.1f} out of bounds")
            return self._scale, False

        return scale, True

    def apply(
        self,
        model_output: np.ndarray,
        min_depth: float = 0.5,
        max_depth: Optional[float] = None,
    ) -> np.ndarray:
        """
        Apply scale to convert model output to metric depth.

        Args:
            model_output: Raw model output (H, W), normalized 0-1
            min_depth: Minimum output depth (meters)
            max_depth: Maximum output depth (meters), None for no upper clamp

        Returns:
            Metric depth map (H, W) in meters
        """
        metric_depth = self._scale * model_output
        metric_depth = np.maximum(metric_depth, min_depth)
        if max_depth is not None:
            metric_depth = np.minimum(metric_depth, max_depth)
        return metric_depth.astype(np.float32)

    def _sample_center(self, depth: np.ndarray) -> float:
        """
        Sample depth at image center region using median for robustness.
        """
        h, w = depth.shape[:2]
        margin_h = max(1, int(h * self.center_region / 2))
        margin_w = max(1, int(w * self.center_region / 2))

        center_h = h // 2
        center_w = w // 2

        # Extract center region
        region = depth[
            center_h - margin_h : center_h + margin_h + 1,
            center_w - margin_w : center_w + margin_w + 1,
        ]

        # Use median for robustness to outliers
        valid_mask = region > 0.001
        if valid_mask.sum() > 0:
            return float(np.median(region[valid_mask]))
        return 0.0

    @property
    def is_frozen(self) -> bool:
        """Whether scale has been frozen."""
        return self._frozen

    @property
    def scale(self) -> float:
        """Current scale factor."""
        return self._scale

    @property
    def confidence(self) -> float:
        """Current confidence in scale estimate."""
        return self._confidence

    @property
    def sample_count(self) -> int:
        """Number of warmup samples collected."""
        return len(self._warmup_samples)

    @property
    def frame_count(self) -> int:
        """Total frames processed."""
        return self._frame_count

    def reset(self):
        """Reset corrector state."""
        self._warmup_samples.clear()
        self._frame_count = 0
        self._scale = 1.0
        self._frozen = False
        self._confidence = 0.0
        self._warmup_complete = False

    def get_debug_info(self) -> dict:
        """Get debug information about current state."""
        return {
            "scale": self._scale,
            "confidence": self._confidence,
            "frozen": self._frozen,
            "frame_count": self._frame_count,
            "sample_count": len(self._warmup_samples),
            "warmup_complete": self._warmup_complete,
        }
