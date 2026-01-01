"""
Integration tests for perception pipeline.
"""
import pytest
import numpy as np
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception_encoder import PerceptionEncoder, PerceptionConfig
from detector import Detection


class TestPerceptionEncoder:
    """Test perception encoder end-to-end."""

    @pytest.fixture
    def encoder(self):
        """Create perception encoder."""
        config = PerceptionConfig(detector_mode="ground_truth")
        return PerceptionEncoder(config)

    @pytest.fixture
    def sample_image(self):
        """Create dummy image."""
        return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    @pytest.fixture
    def sample_labels(self):
        """Create sample ground truth labels."""
        return [
            {
                'class_id': 1,
                'bbox': (0.3, 0.4, 0.1, 0.2),
                'position_3d': np.array([10.0, 5.0, 0.0]),
            },
            {
                'class_id': 3,
                'bbox': (0.7, 0.6, 0.15, 0.25),
                'position_3d': np.array([30.0, -10.0, 0.0]),
            },
        ]

    def test_output_shape(self, encoder, sample_image, sample_labels):
        """Test output has correct shape."""
        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=sample_labels,
        )

        assert obs.shape == (encoder.output_dim,)
        assert obs.dtype == np.float32

    def test_output_range(self, encoder, sample_image, sample_labels):
        """Test output values are normalized."""
        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=sample_labels,
        )

        # Most values should be in [-1, 1] or [0, 1]
        assert np.all(obs >= -1.1)
        assert np.all(obs <= 1.1)

    def test_person_detection_priority(self, encoder, sample_image):
        """Test persons are prioritized in observations."""
        labels = [
            {'class_id': 1, 'bbox': (0.5, 0.5, 0.1, 0.2), 'position_3d': np.array([5.0, 0.0, 0.0])},
            {'class_id': 3, 'bbox': (0.3, 0.3, 0.2, 0.3), 'position_3d': np.array([3.0, 0.0, 0.0])},
        ]

        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=labels,
        )

        # First detection should be person (class_id=1, normalized=0.1)
        assert obs[0] == pytest.approx(0.1, abs=0.01)

    def test_encoding_speed(self, encoder, sample_image, sample_labels):
        """Test encoding completes within budget."""
        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        # Warm up
        for _ in range(5):
            encoder.encode(
                image=sample_image,
                uav_position=uav_pos,
                uav_orientation=uav_ori,
                ground_truth_labels=sample_labels,
            )

        # Measure
        import time
        times = []
        for _ in range(20):
            start = time.perf_counter()
            encoder.encode(
                image=sample_image,
                uav_position=uav_pos,
                uav_orientation=uav_ori,
                ground_truth_labels=sample_labels,
            )
            times.append((time.perf_counter() - start) * 1000)

        avg_time = np.mean(times)
        print(f"Average encoding time: {avg_time:.2f} ms")

        # Should complete within 20ms budget (leaving 30ms for Vampire)
        assert avg_time < 20.0, f"Encoding too slow: {avg_time:.2f} ms"

    def test_empty_detections(self, encoder, sample_image):
        """Test handling of empty detection list."""
        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=[],
        )

        assert obs.shape == (encoder.output_dim,)
        # Priority detections should be zeros
        assert np.all(obs[:100] == 0)

    def test_many_detections(self, encoder, sample_image):
        """Test handling of many detections (crowd scenario)."""
        # Create 50 person detections
        labels = [
            {
                'class_id': 1,
                'bbox': (np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), 0.05, 0.1),
                'position_3d': np.array([
                    np.random.uniform(-50, 50),
                    np.random.uniform(-50, 50),
                    0.0
                ]),
            }
            for _ in range(50)
        ]

        uav_pos = np.array([0.0, 0.0, 50.0])
        uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

        obs = encoder.encode(
            image=sample_image,
            uav_position=uav_pos,
            uav_orientation=uav_ori,
            ground_truth_labels=labels,
        )

        assert obs.shape == (encoder.output_dim,)

        # Grid should capture all detections (person count should be high)
        grid_start = 100  # After priority detections
        grid_obs = obs[grid_start:grid_start + 384]

        # Some cells should have high person counts
        assert np.max(grid_obs) > 0.1

    def test_output_dimensions(self, encoder):
        """Test output dimensions match specification."""
        expected_dim = 100 + 384 + 32  # priority + grid + stats = 516
        assert encoder.output_dim == expected_dim


class TestTPTPGenerator:
    """Test TPTP fact generation."""

    @pytest.fixture
    def generator(self):
        from tptp_generator import TPTPGenerator
        return TPTPGenerator()

    def test_person_facts(self, generator):
        """Test person detection generates safety facts."""
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([10.0, 5.0, 0.0]),
                distance=11.2,
            )
        ]

        facts = generator.generate_facts(detections)

        # Should have observation, distance, and person-specific facts
        assert len(facts) >= 3
        assert any("observedObject" in f for f in facts)
        assert any("distanceTo" in f for f in facts)
        assert any("Person" in f for f in facts)

    def test_danger_zone_fact(self, generator):
        """Test person in danger zone generates warning fact."""
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([5.0, 0.0, 0.0]),
                distance=5.0,  # Within 15m danger zone
            )
        ]

        facts = generator.generate_facts(detections)

        assert any("personInDangerZone" in f for f in facts)

    def test_safety_query_format(self, generator):
        """Test complete safety query is valid TPTP."""
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([10.0, 5.0, 0.0]),
                distance=11.2,
            )
        ]

        state = {'x': 0.0, 'y': 0.0, 'z': 50.0, 'battery': 80.0, 'in_geofence': True}
        action = np.array([1.0, 0.0, 0.0, 0.0])

        query = generator.generate_safety_query(detections, state, action)

        # Should have proper TPTP structure
        assert "fof(" in query
        assert "axiom" in query
        assert "conjecture" in query
        assert "safety_query" in query


class TestSpatialEncoder:
    """Test spatial grid encoder."""

    @pytest.fixture
    def encoder(self):
        from spatial_encoder import SpatialGridEncoder
        return SpatialGridEncoder()

    def test_output_shape(self, encoder):
        """Test grid has correct shape."""
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                distance=10.0,
            )
        ]

        grid = encoder.encode(detections)
        assert grid.shape == (384,)  # 8 x 8 x 6

    def test_empty_grid(self, encoder):
        """Test empty detection list produces zeros."""
        grid = encoder.encode([])
        assert grid.shape == (384,)
        assert np.all(grid == 0)

    def test_person_in_correct_cell(self, encoder):
        """Test person detection appears in correct grid cell."""
        # Person at center (0.5, 0.5) should be in cell (4, 4)
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                distance=10.0,
            )
        ]

        grid = encoder.encode(detections)
        decoded = encoder.decode(grid)

        # Cell (4, 4), channel 0 (person count) should be > 0
        assert decoded[4, 4, 0] > 0


class TestTemporalTracker:
    """Test temporal tracking."""

    @pytest.fixture
    def tracker(self):
        from temporal_tracker import TemporalTracker
        return TemporalTracker()

    def test_new_track_creation(self, tracker):
        """Test new tracks are created for unmatched detections."""
        detections = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
            )
        ]

        updated = tracker.update(detections)

        assert len(tracker.tracks) == 1
        assert updated[0].track_id is not None

    def test_track_matching(self, tracker):
        """Test tracks are matched across frames."""
        # Frame 1
        det1 = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([10.0, 0.0, 0.0]),
            )
        ]
        tracker.update(det1)
        first_track_id = list(tracker.tracks.keys())[0]

        # Frame 2 - same object moved slightly
        det2 = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.52, 0.52, 0.1, 0.2),
                position_3d=np.array([10.5, 0.5, 0.0]),
            )
        ]
        updated = tracker.update(det2)

        # Should still have same track
        assert len(tracker.tracks) == 1
        assert updated[0].track_id == first_track_id

    def test_velocity_estimation(self, tracker):
        """Test velocity is estimated from position changes."""
        # Frame 1
        det1 = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.5, 0.5, 0.1, 0.2),
                position_3d=np.array([10.0, 0.0, 0.0]),
            )
        ]
        tracker.update(det1, dt=0.05)

        # Frame 2 - moved 1m in 0.05s = 20 m/s
        det2 = [
            Detection(
                class_id=1,
                class_name="person",
                ontology_class="Person",
                confidence=0.9,
                bbox=(0.52, 0.5, 0.1, 0.2),
                position_3d=np.array([11.0, 0.0, 0.0]),
            )
        ]
        updated = tracker.update(det2, dt=0.05)

        # Velocity should be estimated
        assert updated[0].velocity is not None
        assert updated[0].velocity[0] == pytest.approx(20.0, abs=0.1)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
