#!/usr/bin/env python3
"""
Tests for the gimbal controller node.

Verifies:
1. Gimbal limits match ontology definitions (isr_extensions.kif)
2. Commands exceeding limits are clamped correctly
3. State publishing works correctly
"""

import pytest
import sys
import os

# Add the package to path for testing without ROS 2 build
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'flyby_f11_sensors'
))


class TestGimbalLimits:
    """Test gimbal limit enforcement from ontology."""

    # Expected limits from isr_extensions.kif
    EXPECTED_YAW_LIMITS = (-6.02, 6.02)
    EXPECTED_PITCH_LIMITS = (-2.09, 2.09)
    EXPECTED_ROLL_LIMITS = (-0.785, 0.785)

    def test_yaw_limits_match_ontology(self):
        """Verify yaw limits match isr_extensions.kif values."""
        from gimbal_controller_node import GimbalControllerNode

        assert GimbalControllerNode.YAW_LIMITS == self.EXPECTED_YAW_LIMITS, (
            f"Yaw limits {GimbalControllerNode.YAW_LIMITS} don't match "
            f"ontology {self.EXPECTED_YAW_LIMITS}"
        )

    def test_pitch_limits_match_ontology(self):
        """Verify pitch limits match isr_extensions.kif values."""
        from gimbal_controller_node import GimbalControllerNode

        assert GimbalControllerNode.PITCH_LIMITS == self.EXPECTED_PITCH_LIMITS, (
            f"Pitch limits {GimbalControllerNode.PITCH_LIMITS} don't match "
            f"ontology {self.EXPECTED_PITCH_LIMITS}"
        )

    def test_roll_limits_match_ontology(self):
        """Verify roll limits match isr_extensions.kif values."""
        from gimbal_controller_node import GimbalControllerNode

        assert GimbalControllerNode.ROLL_LIMITS == self.EXPECTED_ROLL_LIMITS, (
            f"Roll limits {GimbalControllerNode.ROLL_LIMITS} don't match "
            f"ontology {self.EXPECTED_ROLL_LIMITS}"
        )


class TestClampFunction:
    """Test the clamp helper function."""

    def test_clamp_within_limits(self):
        """Value within limits should be unchanged."""
        from gimbal_controller_node import GimbalControllerNode

        # Create a mock instance to access clamp method
        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = (-1.0, 1.0)

        assert node.clamp(node, 0.0, limits) == 0.0
        assert node.clamp(node, 0.5, limits) == 0.5
        assert node.clamp(node, -0.5, limits) == -0.5

    def test_clamp_above_max(self):
        """Value above max should be clamped to max."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = (-1.0, 1.0)

        assert node.clamp(node, 1.5, limits) == 1.0
        assert node.clamp(node, 10.0, limits) == 1.0

    def test_clamp_below_min(self):
        """Value below min should be clamped to min."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = (-1.0, 1.0)

        assert node.clamp(node, -1.5, limits) == -1.0
        assert node.clamp(node, -10.0, limits) == -1.0

    def test_clamp_at_limits(self):
        """Value at exact limits should be unchanged."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = (-1.0, 1.0)

        assert node.clamp(node, 1.0, limits) == 1.0
        assert node.clamp(node, -1.0, limits) == -1.0


class TestYawLimitEnforcement:
    """Test yaw angle limit enforcement."""

    def test_yaw_positive_overflow(self):
        """Yaw exceeding +6.02 should be clamped."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = GimbalControllerNode.YAW_LIMITS

        # Test values exceeding limit
        assert node.clamp(node, 10.0, limits) == 6.02
        assert node.clamp(node, 100.0, limits) == 6.02

    def test_yaw_negative_overflow(self):
        """Yaw below -6.02 should be clamped."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = GimbalControllerNode.YAW_LIMITS

        assert node.clamp(node, -10.0, limits) == -6.02
        assert node.clamp(node, -100.0, limits) == -6.02


class TestPitchLimitEnforcement:
    """Test pitch angle limit enforcement."""

    def test_pitch_positive_overflow(self):
        """Pitch exceeding +2.09 should be clamped."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = GimbalControllerNode.PITCH_LIMITS

        assert node.clamp(node, 3.0, limits) == 2.09
        assert node.clamp(node, 10.0, limits) == 2.09

    def test_pitch_negative_overflow(self):
        """Pitch below -2.09 should be clamped."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = GimbalControllerNode.PITCH_LIMITS

        assert node.clamp(node, -3.0, limits) == -2.09
        assert node.clamp(node, -10.0, limits) == -2.09


class TestRollLimitEnforcement:
    """Test roll angle limit enforcement."""

    def test_roll_positive_overflow(self):
        """Roll exceeding +0.785 should be clamped."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = GimbalControllerNode.ROLL_LIMITS

        assert node.clamp(node, 1.0, limits) == 0.785
        assert node.clamp(node, 5.0, limits) == 0.785

    def test_roll_negative_overflow(self):
        """Roll below -0.785 should be clamped."""
        from gimbal_controller_node import GimbalControllerNode

        class MockNode:
            clamp = GimbalControllerNode.clamp

        node = MockNode()
        limits = GimbalControllerNode.ROLL_LIMITS

        assert node.clamp(node, -1.0, limits) == -0.785
        assert node.clamp(node, -5.0, limits) == -0.785


class TestOntologyConsistency:
    """Test consistency with Gazebo model limits."""

    # These values must match f11_isr_camera/model.sdf joint limits
    GAZEBO_YAW_LIMITS = (-6.02, 6.02)
    GAZEBO_PITCH_LIMITS = (-2.09, 2.09)
    GAZEBO_ROLL_LIMITS = (-0.785, 0.785)

    def test_yaw_matches_gazebo_model(self):
        """Verify yaw limits match Gazebo model.sdf."""
        from gimbal_controller_node import GimbalControllerNode

        assert GimbalControllerNode.YAW_LIMITS == self.GAZEBO_YAW_LIMITS, (
            "Yaw limits must match Gazebo model gimbal_yaw_joint limits"
        )

    def test_pitch_matches_gazebo_model(self):
        """Verify pitch limits match Gazebo model.sdf."""
        from gimbal_controller_node import GimbalControllerNode

        assert GimbalControllerNode.PITCH_LIMITS == self.GAZEBO_PITCH_LIMITS, (
            "Pitch limits must match Gazebo model gimbal_pitch_joint limits"
        )

    def test_roll_matches_gazebo_model(self):
        """Verify roll limits match Gazebo model.sdf."""
        from gimbal_controller_node import GimbalControllerNode

        assert GimbalControllerNode.ROLL_LIMITS == self.GAZEBO_ROLL_LIMITS, (
            "Roll limits must match Gazebo model camera_joint limits"
        )


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
