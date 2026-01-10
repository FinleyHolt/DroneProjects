"""
Affine depth correction for monocular depth estimation using rangefinder measurements.

Depth Anything V2 and similar models output AFFINE-INVARIANT inverse depth:
    model_output = a * true_inverse_depth + b

This means we need to recover BOTH scale AND shift parameters, not just scale.
The rangefinder provides ground truth metric distance at the image center,
which we accumulate over time to solve for both parameters via least-squares.

Metric depth conversion:
    metric_depth = scale * model_output + shift

where [scale, shift] = argmin ||scale * d_model + shift - d_rangefinder||^2
over a sliding window of accumulated (d_model, d_rangefinder) samples.

Author: Finley Holt
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Tuple


@dataclass
class AffineEstimate:
    """Result of affine parameter estimation."""
    scale: float
    shift: float
    confidence: float  # 0-1, based on sample count and fit quality
    rangefinder_reading: float
    model_center: float
    valid: bool


class AffineDepthCorrector:
    """
    Estimates scale AND shift for affine-invariant depth models.

    Uses least-squares fitting on a sliding window of (model_output, rangefinder)
    samples to recover both affine parameters. Includes EMA smoothing to prevent
    sudden parameter jumps.
    """

    def __init__(
        self,
        window_size: int = 20,
        alpha: float = 0.3,
        min_samples: int = 5,
        min_valid_range: float = 1.0,  # Minimum 1m - closer readings are unreliable
        max_valid_range: float = 200.0,  # Cap at 200m for stability
        center_region: float = 0.1,
        outlier_threshold: float = 2.0,
    ):
        """
        Initialize affine depth corrector.

        Args:
            window_size: Number of samples in sliding window for LS fitting
            alpha: EMA filter coefficient (0-1, higher = more responsive)
            min_samples: Minimum samples required before solving LS
            min_valid_range: Minimum valid rangefinder reading (meters)
            max_valid_range: Maximum valid rangefinder reading (meters)
            center_region: Fraction of image center to sample for model output
            outlier_threshold: Reject samples with residual > threshold * std
        """
        self.window_size = window_size
        self.alpha = alpha
        self.min_samples = min_samples
        self.min_valid_range = min_valid_range
        self.max_valid_range = max_valid_range
        self.center_region = center_region
        self.outlier_threshold = outlier_threshold

        # State
        self._scale: float = 1.0
        self._shift: float = 0.0
        self._confidence: float = 0.0
        self._sample_buffer: List[Tuple[float, float]] = []  # (d_model, d_rangefinder)
        self._fit_residual: float = 0.0

    def update(
        self,
        model_output: np.ndarray,
        rangefinder_reading: float,
        rangefinder_valid: bool = True,
    ) -> AffineEstimate:
        """
        Update affine parameters with new depth map and rangefinder reading.

        Args:
            model_output: Raw model output (H, W), normalized 0-1
            rangefinder_reading: Distance from rangefinder (meters)
            rangefinder_valid: Whether rangefinder reading is valid

        Returns:
            AffineEstimate with current scale, shift, and confidence
        """
        # Sample model output at image center
        d_model = self._sample_center(model_output)

        # Check validity
        valid = (
            rangefinder_valid
            and self.min_valid_range <= rangefinder_reading <= self.max_valid_range
            and d_model > 0.001  # Non-zero model output at center
        )

        if valid:
            # Add sample to buffer
            self._sample_buffer.append((d_model, rangefinder_reading))
            if len(self._sample_buffer) > self.window_size:
                self._sample_buffer.pop(0)

            # Solve least-squares if we have enough samples
            if len(self._sample_buffer) >= self.min_samples:
                scale, shift, residual = self._solve_least_squares()

                # Sanity check - scale must be positive and reasonable
                # For depth estimation, scale should typically be 1-100
                if scale <= 0 or scale > 500 or abs(shift) > 500:
                    # Invalid parameters - skip this update
                    self._sample_buffer.pop()  # Remove bad sample
                    return AffineEstimate(
                        scale=self._scale,
                        shift=self._shift,
                        confidence=self._confidence * 0.9,
                        rangefinder_reading=rangefinder_reading,
                        model_center=d_model,
                        valid=False,
                    )

                # Check for outliers - reject if this sample is too far from fit
                predicted = scale * d_model + shift
                error = abs(predicted - rangefinder_reading)
                if residual > 0 and error > self.outlier_threshold * residual:
                    # This sample is an outlier, remove it
                    self._sample_buffer.pop()
                else:
                    # Update parameters with EMA smoothing
                    self._scale = self.alpha * scale + (1 - self.alpha) * self._scale
                    self._shift = self.alpha * shift + (1 - self.alpha) * self._shift
                    self._fit_residual = residual

                # Confidence based on sample count and fit quality
                sample_conf = min(1.0, len(self._sample_buffer) / self.window_size)
                fit_conf = max(0.0, 1.0 - residual / 10.0) if residual > 0 else 0.5
                self._confidence = 0.7 * sample_conf + 0.3 * fit_conf

        else:
            # Decay confidence when we don't get valid readings
            self._confidence *= 0.98

        return AffineEstimate(
            scale=self._scale,
            shift=self._shift,
            confidence=self._confidence,
            rangefinder_reading=rangefinder_reading,
            model_center=d_model,
            valid=valid,
        )

    def apply(
        self,
        model_output: np.ndarray,
        min_depth: float = 0.5,
        max_depth: Optional[float] = None,
    ) -> np.ndarray:
        """
        Apply affine transformation to convert model output to metric depth.

        Args:
            model_output: Raw model output (H, W), normalized 0-1
            min_depth: Minimum output depth (meters)
            max_depth: Maximum output depth (meters), None for no upper clamp

        Returns:
            Metric depth map (H, W) in meters
        """
        # Apply affine transformation: metric = scale * model + shift
        metric_depth = self._scale * model_output + self._shift

        # Clamp to valid range
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

    def _sample_center_robust(
        self, depth: np.ndarray, gradient_threshold: float = 0.1
    ) -> float:
        """
        Sample center region with gradient-based edge rejection.

        Avoids depth discontinuities which can corrupt the scale estimate.
        """
        h, w = depth.shape[:2]
        margin = max(2, int(min(h, w) * self.center_region / 2))
        cy, cx = h // 2, w // 2

        region = depth[cy - margin : cy + margin, cx - margin : cx + margin]

        if region.size == 0:
            return self._sample_center(depth)

        # Compute gradients
        if region.shape[0] > 1 and region.shape[1] > 1:
            gy = np.abs(np.diff(region, axis=0))  # Shape: (H-1, W)
            gx = np.abs(np.diff(region, axis=1))  # Shape: (H, W-1)

            # Create edge mask (avoid depth discontinuities)
            edge_mask = np.zeros(region.shape, dtype=bool)

            # Vertical gradients affect rows 1 onward (where gradient was computed from)
            edge_mask[1:, :] |= gy > gradient_threshold * np.maximum(region[:-1, :], 0.01)

            # Horizontal gradients affect cols 1 onward
            edge_mask[:, 1:] |= gx > gradient_threshold * np.maximum(region[:, :-1], 0.01)

            # Use median of non-edge pixels
            valid = ~edge_mask & (region > 0.001)
            if valid.sum() > 5:
                return float(np.median(region[valid]))

        # Fallback to simple median
        valid = region > 0.001
        if valid.sum() > 0:
            return float(np.median(region[valid]))
        return 0.0

    def _solve_least_squares(self) -> Tuple[float, float, float]:
        """
        Solve least-squares for affine parameters.

        Solves: argmin ||scale * d_model + shift - d_rangefinder||^2

        Returns:
            Tuple of (scale, shift, residual_std)
        """
        if len(self._sample_buffer) < 2:
            return self._scale, self._shift, 0.0

        # Build design matrix and observation vector
        X = np.array([[d[0], 1.0] for d in self._sample_buffer])
        y = np.array([d[1] for d in self._sample_buffer])

        # Check for numerical issues: if all model outputs are too similar,
        # the design matrix is nearly rank-deficient and LS will be unstable.
        # We need sufficient variation to estimate both scale and shift.
        d_model_values = X[:, 0]
        if np.std(d_model_values) < 0.01:
            # Insufficient variation - try scale-only estimation using mean ratio
            # This is a fallback when we can't estimate both parameters
            d_mean = np.mean(d_model_values)
            y_mean = np.mean(y)
            if d_mean > 0.01:
                # Use simple ratio for scale, keep current shift
                rough_scale = y_mean / d_mean
                return rough_scale, self._shift, 0.0
            return self._scale, self._shift, 0.0

        # Solve (X^T X)^-1 X^T y using numpy's lstsq
        try:
            params, residuals, rank, s = np.linalg.lstsq(X, y, rcond=None)

            # Check rank deficiency
            if rank < 2:
                # Cannot solve affine transformation with rank < 2
                return self._scale, self._shift, 0.0

            scale, shift = params[0], params[1]

            # Validate results: check for extreme or invalid values
            if not np.isfinite(scale) or not np.isfinite(shift):
                return self._scale, self._shift, 0.0

            # Compute residual standard deviation
            predictions = X @ params
            residual_std = float(np.std(y - predictions))

            return scale, shift, residual_std

        except np.linalg.LinAlgError:
            # Fallback if lstsq fails (e.g., singular matrix)
            return self._scale, self._shift, 0.0

    @property
    def scale(self) -> float:
        """Current scale parameter."""
        return self._scale

    @property
    def shift(self) -> float:
        """Current shift parameter."""
        return self._shift

    @property
    def confidence(self) -> float:
        """Current confidence in affine estimate."""
        return self._confidence

    @property
    def is_initialized(self) -> bool:
        """Whether we have enough samples for reliable estimates."""
        return len(self._sample_buffer) >= self.min_samples

    @property
    def sample_count(self) -> int:
        """Number of samples in buffer."""
        return len(self._sample_buffer)

    def reset(self):
        """Reset estimator state."""
        self._scale = 1.0
        self._shift = 0.0
        self._confidence = 0.0
        self._sample_buffer.clear()
        self._fit_residual = 0.0

    def get_debug_info(self) -> dict:
        """Get debug information about current state."""
        return {
            "scale": self._scale,
            "shift": self._shift,
            "confidence": self._confidence,
            "sample_count": len(self._sample_buffer),
            "fit_residual": self._fit_residual,
            "is_initialized": self.is_initialized,
        }


class KalmanAffineFilter:
    """
    Kalman filter for smooth scale/shift estimation.

    Provides temporal smoothing of affine parameters with uncertainty estimates.
    State vector: [scale, shift]
    """

    def __init__(
        self,
        initial_scale: float = 1.0,
        initial_shift: float = 0.0,
        process_noise_scale: float = 0.001,
        process_noise_shift: float = 0.01,
        measurement_noise: float = 0.5,
    ):
        """
        Initialize Kalman filter.

        Args:
            initial_scale: Initial scale estimate
            initial_shift: Initial shift estimate
            process_noise_scale: Process noise for scale (lower = more stable)
            process_noise_shift: Process noise for shift (lower = more stable)
            measurement_noise: Measurement noise variance
        """
        # State: [scale, shift]
        self.x = np.array([initial_scale, initial_shift])

        # State covariance
        self.P = np.eye(2) * 1.0

        # Process noise covariance (scale is more stable than shift)
        self.Q = np.diag([process_noise_scale, process_noise_shift])

        # Measurement noise covariance
        self.R = np.array([[measurement_noise]])

        # Track innovation for confidence estimation
        self._innovation_history: List[float] = []
        self._max_history = 20

    def predict(self):
        """
        Predict step: state transition is identity (random walk model).
        """
        # State transition: x_k = x_{k-1} (parameters evolve slowly)
        # Covariance grows with process noise
        self.P = self.P + self.Q

    def update(
        self, d_model_center: float, rangefinder: float
    ) -> Tuple[float, float, np.ndarray]:
        """
        Update step with new measurement.

        Measurement model: z = scale * d_model + shift

        Args:
            d_model_center: Model output at image center
            rangefinder: Rangefinder measurement (meters)

        Returns:
            Tuple of (scale, shift, uncertainties)
        """
        # Input validation: skip update if inputs are invalid
        if not np.isfinite(d_model_center) or not np.isfinite(rangefinder):
            # Return current state without update
            return self.x[0], self.x[1], np.sqrt(np.diag(self.P))

        # If d_model_center is too small, we lose observability of scale
        if abs(d_model_center) < 0.001:
            # Cannot observe scale parameter, skip update
            return self.x[0], self.x[1], np.sqrt(np.diag(self.P))

        # Predict step
        self.predict()

        # Measurement matrix: z = H @ x where H = [d_model, 1]
        H = np.array([[d_model_center, 1.0]])
        z = np.array([rangefinder])

        # Innovation (measurement residual)
        y = z - H @ self.x

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Check for numerical stability
        S_val = S[0, 0]
        if S_val < 1e-10:
            # Innovation covariance too small, skip update
            return self.x[0], self.x[1], np.sqrt(np.diag(self.P))

        # Kalman gain (use scalar division for 1D measurement)
        K = (self.P @ H.T) / S_val

        # Update state
        self.x = self.x + (K * y[0]).flatten()

        # Update covariance using Joseph form for numerical stability
        I = np.eye(2)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        # Validate state: prevent extreme values
        if not np.all(np.isfinite(self.x)):
            self.reset()
            return self.x[0], self.x[1], np.sqrt(np.diag(self.P))

        # Track innovation for confidence
        self._innovation_history.append(float(np.abs(y[0])))
        if len(self._innovation_history) > self._max_history:
            self._innovation_history.pop(0)

        # Return scale, shift, and uncertainties (sqrt of diagonal of P)
        uncertainties = np.sqrt(np.diag(self.P))
        return self.x[0], self.x[1], uncertainties

    @property
    def scale(self) -> float:
        """Current scale estimate."""
        return self.x[0]

    @property
    def shift(self) -> float:
        """Current shift estimate."""
        return self.x[1]

    @property
    def confidence(self) -> float:
        """
        Confidence based on innovation consistency.

        Low innovation variance = high confidence in estimates.
        """
        if len(self._innovation_history) < 3:
            return 0.5

        innovation_std = np.std(self._innovation_history)
        # Map innovation std to confidence: low std = high confidence
        # std of 0 -> conf 1.0, std of 5+ -> conf 0.0
        return max(0.0, min(1.0, 1.0 - innovation_std / 5.0))

    def reset(self, scale: float = 1.0, shift: float = 0.0):
        """Reset filter state."""
        self.x = np.array([scale, shift])
        self.P = np.eye(2) * 1.0
        self._innovation_history.clear()


class HybridAffineCorrector:
    """
    Combines least-squares fitting with Kalman filtering.

    Uses LS for robust parameter estimation, then Kalman for temporal smoothing.
    """

    def __init__(
        self,
        window_size: int = 20,
        min_samples: int = 5,
        use_kalman: bool = True,
        use_robust_sampling: bool = True,
    ):
        """
        Initialize hybrid corrector.

        Args:
            window_size: Samples for LS fitting
            min_samples: Minimum samples before solving
            use_kalman: Whether to apply Kalman smoothing
            use_robust_sampling: Whether to use gradient-aware center sampling
        """
        self.ls_corrector = AffineDepthCorrector(
            window_size=window_size,
            min_samples=min_samples,
            alpha=1.0,  # No EMA in LS, Kalman handles smoothing
        )
        self.kalman = KalmanAffineFilter() if use_kalman else None
        self.use_robust_sampling = use_robust_sampling

        self._scale = 1.0
        self._shift = 0.0
        self._confidence = 0.0

    def update(
        self,
        model_output: np.ndarray,
        rangefinder_reading: float,
        rangefinder_valid: bool = True,
    ) -> AffineEstimate:
        """Update with new measurement."""
        # Get LS estimate
        ls_estimate = self.ls_corrector.update(
            model_output, rangefinder_reading, rangefinder_valid
        )

        if self.kalman is not None and ls_estimate.valid:
            # Apply Kalman smoothing
            d_model = (
                self.ls_corrector._sample_center_robust(model_output)
                if self.use_robust_sampling
                else self.ls_corrector._sample_center(model_output)
            )
            scale, shift, _ = self.kalman.update(d_model, rangefinder_reading)
            self._scale = scale
            self._shift = shift
            self._confidence = 0.5 * ls_estimate.confidence + 0.5 * self.kalman.confidence
        else:
            self._scale = ls_estimate.scale
            self._shift = ls_estimate.shift
            self._confidence = ls_estimate.confidence

        return AffineEstimate(
            scale=self._scale,
            shift=self._shift,
            confidence=self._confidence,
            rangefinder_reading=rangefinder_reading,
            model_center=ls_estimate.model_center,
            valid=ls_estimate.valid,
        )

    def apply(
        self,
        model_output: np.ndarray,
        min_depth: float = 0.5,
        max_depth: Optional[float] = None,
    ) -> np.ndarray:
        """Apply affine transformation to model output."""
        metric_depth = self._scale * model_output + self._shift
        metric_depth = np.maximum(metric_depth, min_depth)
        if max_depth is not None:
            metric_depth = np.minimum(metric_depth, max_depth)
        return metric_depth.astype(np.float32)

    @property
    def scale(self) -> float:
        return self._scale

    @property
    def shift(self) -> float:
        return self._shift

    @property
    def confidence(self) -> float:
        return self._confidence

    def reset(self):
        """Reset all state."""
        self.ls_corrector.reset()
        if self.kalman is not None:
            self.kalman.reset()
        self._scale = 1.0
        self._shift = 0.0
        self._confidence = 0.0
