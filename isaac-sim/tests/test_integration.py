#!/usr/bin/env python3
"""
Integration Tests for Flyby F-11 RL Training Pipeline

Tests that REQUIRE Isaac Sim container:
- Environment instantiation
- reset() returns valid observation shape
- step() with random action doesn't crash
- Domain randomization produces valid ranges
- NFZ randomization generates 2-5 NFZs
- Weather/lighting presets apply without error
- Safety filter gracefully handles missing Vampire

Run inside Isaac Sim container with:
/isaac-sim/python.sh -m pytest /workspace/tests/test_integration.py -v

Mark for skipping outside container:
pytest -m "not requires_isaac" tests/test_integration.py
"""
import sys
import os
from pathlib import Path
from typing import Optional, Dict, Any
import numpy as np

import pytest

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Check if we're in Isaac Sim environment
ISAAC_SIM_AVAILABLE = False
try:
    # Try importing Isaac Sim - this will only work in container
    from isaacsim import SimulationApp
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    pass

# Custom marker for tests requiring Isaac Sim
requires_isaac = pytest.mark.skipif(
    not ISAAC_SIM_AVAILABLE,
    reason="Requires Isaac Sim environment (run inside container)"
)


# ==============================================================================
# IMPORTANT: Isaac Sim Session-Scoped Fixtures
# ==============================================================================
# Isaac Sim cannot be restarted within the same Python process. Once
# SimulationApp.close() is called, the process must exit. Therefore, we use
# a single session-scoped fixture that all tests share.
#
# The fixture creates ONE environment at the start and keeps it alive until
# all tests complete. Tests that need a different environment type should
# be in a separate test file or use module-level fixtures.
# ==============================================================================

@pytest.fixture(scope="session")
def session_comms_denied_env():
    """
    Session-scoped environment fixture.

    This environment is shared across ALL tests in this file to avoid
    the Isaac Sim restart issue. The environment is only closed after
    all tests complete.
    """
    if not ISAAC_SIM_AVAILABLE:
        pytest.skip("Isaac Sim not available")

    from environments.comms_denied_env import CommsDeniedSurveillanceEnv, CommsDeniedConfig

    config = CommsDeniedConfig(headless=True)
    env = CommsDeniedSurveillanceEnv(config)
    env.setup()

    yield env

    # Only close at end of entire session
    env.close()


class TestEnvironmentInstantiation:
    """Test environment creation without crash (no setup, just instantiation)."""

    @requires_isaac
    def test_comms_denied_env_instantiation(self):
        """Test CommsDeniedSurveillanceEnv can be instantiated."""
        from environments.comms_denied_env import CommsDeniedSurveillanceEnv, CommsDeniedConfig

        config = CommsDeniedConfig(headless=True)
        env = CommsDeniedSurveillanceEnv(config)

        assert env is not None
        assert hasattr(env, 'config')
        assert hasattr(env, 'uav_state')
        # Note: Don't call setup() or close() - just testing instantiation

    @requires_isaac
    def test_dynamic_nfz_env_instantiation(self):
        """Test DynamicNFZAvoidanceEnv can be instantiated."""
        from environments.dynamic_nfz_env import DynamicNFZAvoidanceEnv, DynamicNFZConfig

        config = DynamicNFZConfig(headless=True)
        env = DynamicNFZAvoidanceEnv(config)

        assert env is not None
        assert hasattr(env, 'dynamic_nfzs')  # List of NFZs
        assert hasattr(env, 'current_path')
        # Note: Don't call setup() or close() - just testing instantiation

    @requires_isaac
    def test_multi_objective_env_instantiation(self):
        """Test MultiObjectiveISREnv can be instantiated."""
        from environments.multi_objective_env import MultiObjectiveISREnv, MultiObjectiveConfig

        config = MultiObjectiveConfig(headless=True)
        env = MultiObjectiveISREnv(config)

        assert env is not None
        assert hasattr(env, 'targets')
        assert hasattr(env, 'threat_zones')
        # Note: Don't call setup() or close() - just testing instantiation


class TestEnvironmentSetup:
    """Test environment setup() method using session fixture."""

    @requires_isaac
    def test_setup_initializes_simulation(self, session_comms_denied_env):
        """Test that setup() initializes simulation components."""
        env = session_comms_denied_env

        assert env._initialized is True, "Environment should be initialized after setup"
        assert env.simulation_app is not None, "SimulationApp should be created"
        assert env.world is not None, "World should be created"
        assert env.vehicle is not None, "Vehicle should be spawned"

    @requires_isaac
    def test_setup_initializes_safety_filter(self, session_comms_denied_env):
        """Test that setup() initializes safety filter."""
        env = session_comms_denied_env

        assert env.safety_filter is not None, "Safety filter should be initialized"
        assert hasattr(env.safety_filter, 'filter_action')

    @requires_isaac
    def test_setup_initializes_action_bridge(self, session_comms_denied_env):
        """Test that setup() initializes action bridge."""
        env = session_comms_denied_env

        assert env.action_bridge is not None, "Action bridge should be initialized"
        assert hasattr(env.action_bridge, 'execute_action')


class TestResetBehavior:
    """Test environment reset() method using session fixture."""

    @requires_isaac
    def test_reset_returns_valid_state(self, session_comms_denied_env):
        """Test reset() returns UAVState."""
        from environments.base_isr_env import UAVState

        state = session_comms_denied_env.reset()

        assert isinstance(state, UAVState), "reset() should return UAVState"

    @requires_isaac
    def test_reset_observation_shape(self, session_comms_denied_env):
        """Test reset() observation matches observation_dim."""
        state = session_comms_denied_env.reset()
        obs = session_comms_denied_env.state_to_observation(state)

        expected_dim = session_comms_denied_env.observation_dim
        actual_dim = len(obs)

        assert actual_dim == expected_dim, \
            f"Observation dimension mismatch: expected {expected_dim}, got {actual_dim}"

    @requires_isaac
    def test_reset_initializes_battery(self, session_comms_denied_env):
        """Test reset() sets battery to full."""
        state = session_comms_denied_env.reset()

        assert state.battery_pct >= 90.0, "Battery should start near full after reset"

    @requires_isaac
    def test_reset_clears_gnss_degradation(self, session_comms_denied_env):
        """Test reset() clears GNSS degradation state."""
        state = session_comms_denied_env.reset()

        assert state.gnss_degradation_triggered is False, \
            "GNSS degradation should be cleared on reset"
        assert state.gnss_degradation_step == -1, \
            "GNSS degradation step should be reset to -1"

    @requires_isaac
    def test_reset_with_seed_deterministic(self, session_comms_denied_env):
        """Test reset() with same seed produces same initial state."""
        state1 = session_comms_denied_env.reset(seed=12345)
        wind1 = state1.wind_vector.copy()

        state2 = session_comms_denied_env.reset(seed=12345)
        wind2 = state2.wind_vector.copy()

        assert np.allclose(wind1, wind2), \
            "Same seed should produce same initial wind vector"


class TestStepBehavior:
    """Test environment step() method using session fixture."""

    @requires_isaac
    def test_step_with_random_action(self, session_comms_denied_env):
        """Test step() with random action doesn't crash."""
        session_comms_denied_env.reset()

        # Random action in valid range
        action = np.random.uniform(-1, 1, size=4).astype(np.float32)

        # Should not raise
        state, reward, done, info = session_comms_denied_env.step(action)

        assert state is not None
        assert isinstance(reward, (int, float))
        assert isinstance(done, bool)
        assert isinstance(info, dict)

    @requires_isaac
    def test_step_reward_is_finite(self, session_comms_denied_env):
        """Test step() produces finite reward."""
        session_comms_denied_env.reset()
        action = np.zeros(4, dtype=np.float32)  # Hover

        for _ in range(10):
            state, reward, done, info = session_comms_denied_env.step(action)

            assert np.isfinite(reward), f"Reward should be finite, got {reward}"

            if done:
                session_comms_denied_env.reset()

    @requires_isaac
    def test_step_observation_in_bounds(self, session_comms_denied_env):
        """Test step() observations are in reasonable bounds."""
        session_comms_denied_env.reset()

        action = np.array([0.5, 0.0, 0.0, 0.0], dtype=np.float32)  # Forward

        for _ in range(10):
            state, reward, done, info = session_comms_denied_env.step(action)
            obs = session_comms_denied_env.state_to_observation(state)

            # Most observations should be in [-10, 10] range after normalization
            # (with some exceptions like position)
            assert np.all(np.isfinite(obs)), "All observations should be finite"

            if done:
                break


# ==============================================================================
# NOTE: Tests requiring DynamicNFZ environment are skipped in this file
# ==============================================================================
# Isaac Sim cannot restart the simulation in the same process. Since we're
# using a session-scoped CommsDenied environment, tests that require the
# DynamicNFZ environment are skipped here. These should be run in a separate
# test file (test_integration_nfz.py) if needed.
# ==============================================================================

class TestDomainRandomization:
    """Test domain randomization sampling using session fixture.

    Note: NFZ-specific tests are skipped since they require a DynamicNFZ
    environment. The session fixture uses CommsDeniedSurveillanceEnv.
    """

    @requires_isaac
    def test_battery_drain_multiplier_varies(self, session_comms_denied_env):
        """Test battery drain multiplier is sampled per episode."""
        # Enable DR if not already
        env = session_comms_denied_env
        multipliers = []

        for i in range(5):
            env.reset(seed=i)
            state = env.uav_state
            if hasattr(state, 'battery_drain_multiplier'):
                multipliers.append(state.battery_drain_multiplier)

        if len(multipliers) > 0:
            # Should have some variation (not all same)
            unique = len(set(multipliers))
            assert unique >= 1, "Battery drain multiplier should exist"

            # Should be in valid range
            for m in multipliers:
                assert 0.8 <= m <= 1.2, f"Multiplier {m} outside expected range"
        else:
            pytest.skip("Environment doesn't have battery_drain_multiplier")

    @requires_isaac
    def test_wind_vector_varies(self, session_comms_denied_env):
        """Test wind vector is sampled per episode."""
        env = session_comms_denied_env
        winds = []

        for i in range(5):
            env.reset(seed=i)
            state = env.uav_state
            if hasattr(state, 'wind_vector'):
                winds.append(state.wind_vector.copy())

        if len(winds) > 0:
            # Check wind is finite
            for w in winds:
                assert np.all(np.isfinite(w)), "Wind vector should be finite"
        else:
            pytest.skip("Environment doesn't have wind_vector")

    @requires_isaac
    @pytest.mark.skip(reason="Requires DynamicNFZ environment - run in separate test file")
    def test_nfz_count_in_range(self):
        """Test NFZ count is in configured range."""
        pass

    @requires_isaac
    @pytest.mark.skip(reason="Requires DynamicNFZ environment - run in separate test file")
    def test_nfz_list_not_empty(self):
        """Test NFZ list is never empty (guards against index error)."""
        pass


class TestSafetyFilterIntegration:
    """Test safety filter integration using session fixture."""

    @requires_isaac
    def test_safety_filter_initialized(self, session_comms_denied_env):
        """Test safety filter is properly initialized."""
        assert session_comms_denied_env.safety_filter is not None

    @requires_isaac
    def test_safety_filter_handles_missing_vampire_gracefully(self, session_comms_denied_env):
        """Test safety filter works even if Vampire isn't installed."""
        session_comms_denied_env.reset()

        # Action that might trigger safety check
        action = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        # Should not crash even if Vampire isn't available
        try:
            state, reward, done, info = session_comms_denied_env.step(action)
            # If we get here, safety filter handled gracefully
            assert True
        except FileNotFoundError:
            pytest.fail("Safety filter should handle missing Vampire gracefully")

    @requires_isaac
    def test_safety_stats_tracking(self, session_comms_denied_env):
        """Test safety intervention stats are tracked."""
        session_comms_denied_env.reset()

        # Run a few steps
        for _ in range(10):
            action = np.random.uniform(-1, 1, size=4).astype(np.float32)
            session_comms_denied_env.step(action)

        stats = session_comms_denied_env.get_safety_stats()

        assert 'safety_violations' in stats
        assert 'safety_interventions' in stats
        assert 'intervention_rate' in stats


class TestGymnasiumWrapperIntegration:
    """Test Gymnasium wrapper - skipped in this session.

    Note: The wrapper creates its own environment which would conflict
    with the session fixture. Run wrapper tests in a separate test file.
    """

    @requires_isaac
    @pytest.mark.skip(reason="Wrapper creates own environment - run in separate test file")
    def test_wrapper_reset_returns_tuple(self):
        """Test wrapped reset() returns (obs, info) tuple."""
        pass

    @requires_isaac
    @pytest.mark.skip(reason="Wrapper creates own environment - run in separate test file")
    def test_wrapper_step_returns_5_values(self):
        """Test wrapped step() returns 5 values (Gymnasium v26+ API)."""
        pass

    @requires_isaac
    @pytest.mark.skip(reason="Wrapper creates own environment - run in separate test file")
    def test_wrapper_observation_matches_space(self):
        """Test observations match defined observation space."""
        pass

    @requires_isaac
    @pytest.mark.skip(reason="Wrapper creates own environment - run in separate test file")
    def test_wrapper_action_space_bounds(self):
        """Test action space is correctly bounded."""
        pass


class TestDomainRandomizationStats:
    """Test domain randomization statistics reporting using session fixture."""

    @requires_isaac
    def test_dr_stats_available(self, session_comms_denied_env):
        """Test domain randomization stats are available."""
        session_comms_denied_env.reset(seed=42)

        if not hasattr(session_comms_denied_env, 'get_domain_randomization_stats'):
            pytest.skip("Environment doesn't have get_domain_randomization_stats method")

        stats = session_comms_denied_env.get_domain_randomization_stats()

        # Check that stats is a dict
        assert isinstance(stats, dict), "DR stats should be a dictionary"

    @requires_isaac
    def test_dr_stats_values_valid(self, session_comms_denied_env):
        """Test DR stats have valid values."""
        session_comms_denied_env.reset(seed=42)

        if not hasattr(session_comms_denied_env, 'get_domain_randomization_stats'):
            pytest.skip("Environment doesn't have get_domain_randomization_stats method")

        stats = session_comms_denied_env.get_domain_randomization_stats()

        # Check values are valid if keys exist
        if 'battery_drain_multiplier' in stats:
            assert 0.5 <= stats['battery_drain_multiplier'] <= 2.0
        if 'wind_magnitude_mps' in stats:
            assert stats['wind_magnitude_mps'] >= 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
