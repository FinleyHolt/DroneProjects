"""
Unit tests for scale corrector.

Tests the rangefinder-based scale correction for monocular depth estimation.
"""

import numpy as np
import pytest

from flyby_depth.scale_corrector import ScaleCorrector, ScaleEstimate


class TestScaleCorrector:
    """Tests for ScaleCorrector class."""

    def test_initialization(self):
        """Test default initialization."""
        corrector = ScaleCorrector()

        assert not corrector.is_initialized
        assert corrector.scale_factor == 1.0
        assert corrector.confidence == 0.0

    def test_first_update_initializes_scale(self):
        """Test that first valid update initializes scale."""
        corrector = ScaleCorrector()

        # Create synthetic depth map with known center value
        depth = np.ones((100, 100), dtype=np.float32) * 0.5
        rangefinder_reading = 10.0  # meters

        result = corrector.update(depth, rangefinder_reading, True)

        assert corrector.is_initialized
        assert result.valid
        # scale = rangefinder / depth_at_center = 10 / 0.5 = 20
        assert abs(result.scale_factor - 20.0) < 0.1

    def test_invalid_rangefinder_doesnt_update(self):
        """Test that invalid rangefinder doesn't change scale."""
        corrector = ScaleCorrector()

        # First valid update
        depth = np.ones((100, 100), dtype=np.float32) * 0.5
        corrector.update(depth, 10.0, True)
        initial_scale = corrector.scale_factor

        # Invalid update (out of range)
        result = corrector.update(depth, 0.1, False)

        assert not result.valid
        assert corrector.scale_factor == initial_scale

    def test_scale_smoothing(self):
        """Test that scale is smoothed via EMA."""
        # Use high outlier_threshold to disable outlier rejection for this test
        corrector = ScaleCorrector(alpha=0.5, outlier_threshold=2.0)

        depth = np.ones((100, 100), dtype=np.float32) * 0.5

        # First update: scale = 10 / 0.5 = 20
        corrector.update(depth, 10.0, True)

        # Second update: instant scale = 20 / 0.5 = 40
        # EMA: 0.5 * 40 + 0.5 * 20 = 30
        result = corrector.update(depth, 20.0, True)

        assert abs(result.scale_factor - 30.0) < 0.1

    def test_outlier_rejection(self):
        """Test that large scale changes are dampened."""
        corrector = ScaleCorrector(alpha=0.3, outlier_threshold=0.5)

        depth = np.ones((100, 100), dtype=np.float32) * 0.5

        # Initial scale = 20
        corrector.update(depth, 10.0, True)

        # Large change: would be scale = 200 (10x change)
        # Should be heavily dampened
        result = corrector.update(depth, 100.0, True)

        # Scale should not jump to 200
        assert result.scale_factor < 100.0

    def test_apply_scale(self):
        """Test applying scale to depth map."""
        corrector = ScaleCorrector()

        depth = np.ones((100, 100), dtype=np.float32) * 0.5
        corrector.update(depth, 10.0, True)  # scale = 20

        metric_depth = corrector.apply_scale(depth, min_depth=0.5, max_depth=100.0)

        # 0.5 * 20 = 10
        assert metric_depth.shape == depth.shape
        assert abs(metric_depth[50, 50] - 10.0) < 0.1

    def test_apply_scale_clipping(self):
        """Test that apply_scale clips to min/max."""
        corrector = ScaleCorrector()

        depth = np.array([[0.001, 0.5, 10.0]], dtype=np.float32)
        corrector.update(np.ones((10, 10)) * 0.5, 10.0, True)  # scale = 20

        metric_depth = corrector.apply_scale(depth, min_depth=1.0, max_depth=50.0)

        assert metric_depth[0, 0] == 1.0  # Clipped to min
        assert metric_depth[0, 2] == 50.0  # Clipped to max

    def test_center_sampling_uses_region(self):
        """Test that center sampling uses configurable region."""
        corrector = ScaleCorrector(center_region=0.2)

        # Create depth map with different values
        depth = np.ones((100, 100), dtype=np.float32) * 1.0
        # Set center region to different value
        depth[40:60, 40:60] = 0.5

        result = corrector.update(depth, 10.0, True)

        # Should use center region value (0.5)
        assert abs(result.depth_at_center - 0.5) < 0.1

    def test_reset_clears_state(self):
        """Test that reset clears scale state."""
        corrector = ScaleCorrector()

        depth = np.ones((100, 100), dtype=np.float32) * 0.5
        corrector.update(depth, 10.0, True)

        assert corrector.is_initialized

        corrector.reset()

        assert not corrector.is_initialized
        assert corrector.confidence == 0.0

    def test_confidence_calculation(self):
        """Test that confidence reflects depth consistency at center region."""
        corrector = ScaleCorrector(center_region=0.2)  # 20% center region

        # Uniform depth at center = high confidence
        uniform_depth = np.ones((100, 100), dtype=np.float32) * 0.5
        result1 = corrector.update(uniform_depth, 10.0, True)

        corrector.reset()

        # Noisy depth at center = lower confidence
        # Don't set center to uniform - keep it noisy to test confidence
        np.random.seed(42)  # For reproducibility
        noisy_depth = np.random.rand(100, 100).astype(np.float32) * 0.5 + 0.25
        result2 = corrector.update(noisy_depth, 10.0, True)

        # Uniform center should have higher confidence than noisy center
        assert result1.confidence > result2.confidence


class TestScaleEstimate:
    """Tests for ScaleEstimate dataclass."""

    def test_scale_estimate_fields(self):
        """Test ScaleEstimate has correct fields."""
        estimate = ScaleEstimate(
            scale_factor=20.0,
            confidence=0.9,
            rangefinder_reading=10.0,
            depth_at_center=0.5,
            valid=True,
        )

        assert estimate.scale_factor == 20.0
        assert estimate.confidence == 0.9
        assert estimate.rangefinder_reading == 10.0
        assert estimate.depth_at_center == 0.5
        assert estimate.valid


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
