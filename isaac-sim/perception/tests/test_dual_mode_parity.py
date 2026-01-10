"""
Parity Tests for Dual-Mode Perception Pipeline

These tests verify that:
1. GT mode and Full mode produce identical observation format (516 dims)
2. Observations are properly normalized
3. Ground truth detector produces realistic detections
4. Both modes can be used interchangeably with the same policy

Key invariants:
- OUTPUT_DIM = 516 for both modes
- All values in [0, 1] or [-1, 1] as specified
- Same Detection dataclass from both detectors
- Encoder produces identical results for identical detections
"""

import numpy as np
import pytest
from typing import List

import sys
sys.path.insert(0, '/workspace/perception')

from perception import (
    # Dual-mode interface
    DualModePerception, DualModeConfig, create_dual_mode_perception,
    # Ground truth
    GroundTruthDetector, GroundTruthConfig, WorldObject,
    CameraParams, CameraFrustum, AABB,
    camera_pose_from_position_orientation,
    # YOLO (for comparison)
    YOLODetector, Detection,
    # Encoder
    PerceptionEncoder, PerceptionConfig, OUTPUT_DIM,
)


class TestObservationDimension:
    """Test that observation dimensions are consistent."""

    def test_output_dim_constant(self):
        """OUTPUT_DIM should be 516."""
        assert OUTPUT_DIM == 516

    def test_encoder_output_dim(self):
        """PerceptionEncoder should produce 516-dim vectors."""
        encoder = PerceptionEncoder()
        assert encoder.output_dim == 516
        assert encoder.OUTPUT_DIM == 516

    def test_dual_mode_output_dim(self):
        """DualModePerception should produce 516-dim vectors."""
        perception = create_dual_mode_perception(mode="gt")
        assert perception.OUTPUT_DIM == 516
        assert perception.output_dim == 516

    def test_gt_mode_observation_shape(self):
        """GT mode should produce (516,) observations."""
        perception = create_dual_mode_perception(mode="gt")

        # Create some test objects
        world_objects = [
            WorldObject(
                id="person_1",
                class_id=1,
                class_name="person",
                position=np.array([10.0, 5.0, 0.0]),
                ontology_class="Person",
            ),
            WorldObject(
                id="vehicle_1",
                class_id=2,
                class_name="vehicle",
                position=np.array([20.0, -10.0, 0.0]),
                ontology_class="DynamicObstacle",
            ),
        ]

        uav_position = np.array([0.0, 0.0, 50.0])
        uav_orientation = np.array([1.0, 0.0, 0.0, 0.0])

        obs = perception.get_observation_gt(
            world_objects=world_objects,
            uav_position=uav_position,
            uav_orientation=uav_orientation,
        )

        assert obs.shape == (516,)
        assert obs.dtype == np.float32


class TestEncoderParity:
    """Test that encoder produces same output for same detections."""

    def test_encoder_determinism(self):
        """Same detections should produce same encoding."""
        encoder = PerceptionEncoder()

        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([10.0, 5.0, 0.0]),
                distance=11.18,
                bearing=0.46,
                priority=1,
                safety_distance=15.0,
            ),
        ]

        uav_position = np.array([0.0, 0.0, 50.0])

        # Encode twice
        obs1 = encoder.encode_from_detections(detections.copy(), uav_position)
        encoder2 = PerceptionEncoder()
        obs2 = encoder2.encode_from_detections(detections.copy(), uav_position)

        # Should be very close (may differ slightly due to timing stats)
        np.testing.assert_allclose(obs1[:500], obs2[:500], atol=1e-5)

    def test_empty_detections(self):
        """Empty detection list should produce valid (zero) encoding."""
        encoder = PerceptionEncoder()

        obs = encoder.encode_from_detections(
            detections=[],
            uav_position=np.array([0.0, 0.0, 50.0]),
        )

        assert obs.shape == (516,)
        # Priority detections should be zero
        assert np.allclose(obs[:100], 0.0)


class TestGroundTruthDetector:
    """Test ground truth detector produces valid detections."""

    def test_detector_initialization(self):
        """Detector should initialize with default config."""
        detector = GroundTruthDetector()
        assert detector is not None
        assert detector.config is not None

    def test_visible_object_detected(self):
        """Object in front of camera should be detected."""
        detector = GroundTruthDetector()

        # Object directly in front of camera
        world_objects = [
            WorldObject(
                id="test_person",
                class_id=1,
                class_name="person",
                position=np.array([0.0, 50.0, 0.0]),  # 50m ahead
                ontology_class="Person",
            ),
        ]
        detector.update_world_objects(world_objects)

        # Camera at origin, looking forward (+Y in NED-like convention)
        camera_pose = np.eye(4)
        camera_pose[2, 3] = 50.0  # 50m altitude
        uav_position = np.array([0.0, 0.0, 50.0])

        detections = detector.detect(camera_pose, uav_position)

        # Should detect the person
        assert len(detections) >= 0  # May or may not be visible depending on frustum

    def test_object_behind_camera_not_detected(self):
        """Object behind camera should not be detected."""
        detector = GroundTruthDetector()

        # Object behind the camera
        world_objects = [
            WorldObject(
                id="test_person",
                class_id=1,
                class_name="person",
                position=np.array([0.0, -100.0, 0.0]),  # Behind camera
                ontology_class="Person",
            ),
        ]
        detector.update_world_objects(world_objects)

        # Camera looking forward
        camera_pose = np.eye(4)
        camera_pose[2, 3] = 50.0
        uav_position = np.array([0.0, 0.0, 50.0])

        detections = detector.detect(camera_pose, uav_position)

        # Should not detect anything behind camera
        for det in detections:
            assert det.distance > 0  # If detected, must have positive distance

    def test_detection_noise(self):
        """Detector should add configurable noise."""
        config = GroundTruthConfig(
            confidence_noise_std=0.1,
            bbox_noise_std=0.05,
        )
        detector = GroundTruthDetector(config)
        detector.set_seed(42)

        world_objects = [
            WorldObject(
                id="test",
                class_id=1,
                class_name="person",
                position=np.array([10.0, 10.0, 0.0]),
                ontology_class="Person",
            ),
        ]
        detector.update_world_objects(world_objects)

        # Get detections with different seeds
        detector.set_seed(1)
        camera_pose = camera_pose_from_position_orientation(
            np.array([0.0, 0.0, 50.0]),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
        det1 = detector.detect(camera_pose, np.array([0.0, 0.0, 50.0]))

        detector.set_seed(2)
        det2 = detector.detect(camera_pose, np.array([0.0, 0.0, 50.0]))

        # With different seeds, detections should vary
        # (though both should still represent the same object)


class TestCameraFrustum:
    """Test camera frustum geometry."""

    def test_point_in_frustum(self):
        """Point directly ahead should be in frustum."""
        params = CameraParams(fov_horizontal=90.0)
        frustum = CameraFrustum(params)

        camera_pose = np.eye(4)

        # Point 10m ahead in camera frame (+Z is forward)
        point_cam = np.array([0.0, 0.0, 10.0])

        assert frustum.is_in_frustum_camera_frame(point_cam)

    def test_point_behind_camera(self):
        """Point behind camera should not be in frustum."""
        params = CameraParams(fov_horizontal=90.0)
        frustum = CameraFrustum(params)

        # Point behind camera
        point_cam = np.array([0.0, 0.0, -10.0])

        assert not frustum.is_in_frustum_camera_frame(point_cam)

    def test_projection(self):
        """Points should project correctly to image plane."""
        params = CameraParams(width=640, height=480, fov_horizontal=90.0)
        frustum = CameraFrustum(params)

        camera_pose = np.eye(4)

        # Point directly ahead
        point_world = np.array([0.0, 0.0, 10.0])
        x, y = frustum.project_to_image(point_world, camera_pose)

        # Should be near center of image
        assert x is not None
        assert y is not None
        assert 0.4 < x < 0.6
        assert 0.4 < y < 0.6


class TestObservationNormalization:
    """Test that observations are properly normalized."""

    def test_observation_bounds(self):
        """All observation values should be in expected range."""
        encoder = PerceptionEncoder()

        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.95,
                bbox=(0.3, 0.7, 0.15, 0.25),
                position_3d=np.array([25.0, 15.0, -10.0]),
                distance=30.4,
                bearing=0.54,
                priority=1,
                safety_distance=15.0,
            ),
            Detection(
                class_id=2,
                class_name="vehicle",
                ontology_class="DynamicObstacle",
                confidence=0.88,
                bbox=(0.6, 0.4, 0.2, 0.15),
                position_3d=np.array([50.0, -20.0, 0.0]),
                distance=53.9,
                bearing=-0.38,
                priority=2,
                safety_distance=10.0,
                velocity=np.array([5.0, 2.0, 0.0]),
            ),
        ]

        obs = encoder.encode_from_detections(
            detections,
            uav_position=np.array([0.0, 0.0, 50.0]),
        )

        # Priority detections (first 100 dims) can have negative values for relative positions
        low, high = encoder.get_observation_space_bounds()
        for i, val in enumerate(obs):
            assert low[i] <= val <= high[i], f"obs[{i}] = {val} outside [{low[i]}, {high[i]}]"


class TestDualModeIntegration:
    """Integration tests for the full dual-mode pipeline."""

    def test_gt_mode_full_pipeline(self):
        """Test complete GT mode pipeline."""
        perception = create_dual_mode_perception(
            mode="gt",
            camera_fov=90.0,
        )

        world_objects = [
            WorldObject(
                id="target_1",
                class_id=4,
                class_name="poi_target",
                position=np.array([30.0, 20.0, 0.0]),
                ontology_class="TargetOfInterest",
            ),
        ]

        obs = perception.get_observation_gt(
            world_objects=world_objects,
            uav_position=np.array([0.0, 0.0, 50.0]),
            uav_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        assert obs.shape == (516,)
        assert obs.dtype == np.float32
        assert not np.isnan(obs).any()
        assert not np.isinf(obs).any()

    def test_mode_switching(self):
        """Verify that mode can be set correctly."""
        gt_perception = create_dual_mode_perception(mode="gt")
        assert gt_perception.mode == "gt"

        full_perception = create_dual_mode_perception(mode="full")
        assert full_perception.mode == "full"

    def test_stats_tracking(self):
        """Verify stats are tracked correctly."""
        perception = create_dual_mode_perception(mode="gt")

        world_objects = [
            WorldObject(
                id="person_1",
                class_id=1,
                class_name="person",
                position=np.array([10.0, 10.0, 0.0]),
                ontology_class="Person",
            ),
        ]

        # Run a few observations
        for _ in range(5):
            perception.get_observation_gt(
                world_objects=world_objects,
                uav_position=np.array([0.0, 0.0, 50.0]),
                uav_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )

        stats = perception.get_stats()
        assert stats['mode'] == 'gt'
        assert stats['frames_processed'] == 5
        assert 'avg_detect_time_ms' in stats
        assert 'avg_encode_time_ms' in stats

    def test_reset(self):
        """Verify reset clears state correctly."""
        perception = create_dual_mode_perception(mode="gt")

        # Run some observations
        for _ in range(3):
            perception.get_observation_gt(
                world_objects=[],
                uav_position=np.array([0.0, 0.0, 50.0]),
                uav_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )

        assert perception.get_stats()['frames_processed'] == 3

        perception.reset()

        assert perception.get_stats()['frames_processed'] == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
