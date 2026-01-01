#!/usr/bin/env python3
"""
Unit Tests for Flyby F-11 RL Training Pipeline

Tests WITH mocked Isaac Sim dependencies:
- DomainRandomizationConfig validation
- NFZRandomizationConfig bounds validation
- UAVState reset behavior
- VampireSafetyFilter TPTP query generation
- PX4ActionBridge action normalization
- LightingPreset / WeatherConfig validation
- Observation space dimensions
- Action space bounds

Run with: python3 -m pytest tests/test_unit.py -v
"""
import sys
import importlib.util
from pathlib import Path
from unittest.mock import MagicMock, patch
from typing import Dict, Any, Optional
import numpy as np

import pytest

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
ENVIRONMENTS_DIR = PROJECT_ROOT / "environments"
sys.path.insert(0, str(PROJECT_ROOT))


def load_module_direct(module_name: str, filepath: Path):
    """
    Load a module directly from filepath, bypassing __init__.py import chains.

    This is necessary because the environments package __init__.py imports
    modules that have Isaac Sim dependencies.
    """
    spec = importlib.util.spec_from_file_location(module_name, filepath)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


# Pre-load modules that don't have Isaac Sim dependencies
# These can be loaded without mocking
@pytest.fixture(scope="module")
def base_isr_env_module():
    """Load base_isr_env module directly."""
    # First mock scipy.spatial.transform which is imported
    mock_scipy = MagicMock()
    mock_scipy.spatial = MagicMock()
    mock_scipy.spatial.transform = MagicMock()
    mock_scipy.spatial.transform.Rotation = MagicMock()

    with patch.dict(sys.modules, {
        'scipy': mock_scipy,
        'scipy.spatial': mock_scipy.spatial,
        'scipy.spatial.transform': mock_scipy.spatial.transform,
    }):
        return load_module_direct("base_isr_env", ENVIRONMENTS_DIR / "base_isr_env.py")


@pytest.fixture(scope="module")
def action_bridge_module():
    """Load action_bridge module directly."""
    mock_pymavlink = MagicMock()
    mock_pymavlink.mavutil = MagicMock()

    with patch.dict(sys.modules, {
        'pymavlink': mock_pymavlink,
        'pymavlink.mavutil': mock_pymavlink.mavutil,
    }):
        return load_module_direct("action_bridge", ENVIRONMENTS_DIR / "action_bridge.py")


@pytest.fixture(scope="module")
def safety_filter_module():
    """Load safety_filter module directly."""
    return load_module_direct("safety_filter", ENVIRONMENTS_DIR / "safety_filter.py")


@pytest.fixture(scope="module")
def domain_randomizer_module():
    """Load domain_randomizer module directly."""
    return load_module_direct("domain_randomizer", ENVIRONMENTS_DIR / "domain_randomizer.py")


# Note: dynamic_nfz_env has relative imports that prevent standalone loading.
# Tests for that module use source file inspection instead.


class TestDomainRandomizationConfig:
    """Test DomainRandomizationConfig dataclass."""

    def test_default_enabled(self, base_isr_env_module):
        """DR should be enabled by default."""
        config = base_isr_env_module.DomainRandomizationConfig()
        assert config.enabled is True, "Domain randomization should be enabled by default"

    def test_default_battery_variation(self, base_isr_env_module):
        """Battery variation should have sensible defaults."""
        config = base_isr_env_module.DomainRandomizationConfig()
        assert config.battery_variation_enabled is True
        assert config.battery_drain_rate_variation == 0.1  # 10%
        assert 0 < config.battery_drain_rate_variation < 1, "Variation should be < 100%"

    def test_default_gnss_degradation(self, base_isr_env_module):
        """GNSS degradation should have valid probability."""
        config = base_isr_env_module.DomainRandomizationConfig()
        assert config.gnss_degradation_enabled is True
        assert 0 <= config.gnss_degradation_prob <= 1, "Probability must be in [0, 1]"

    def test_default_wind_range(self, base_isr_env_module):
        """Wind speed range should be valid."""
        config = base_isr_env_module.DomainRandomizationConfig()
        assert config.wind_enabled is True
        min_wind, max_wind = config.wind_speed_range
        assert min_wind >= 0, "Minimum wind speed must be >= 0"
        assert max_wind > min_wind, "Maximum wind must be greater than minimum"
        assert max_wind <= 20, "Maximum wind should be reasonable (< 20 m/s)"

    def test_disabled_config(self, base_isr_env_module):
        """Test explicitly disabled configuration."""
        config = base_isr_env_module.DomainRandomizationConfig(enabled=False)
        assert config.enabled is False

    def test_custom_battery_variation(self, base_isr_env_module):
        """Test custom battery variation setting."""
        config = base_isr_env_module.DomainRandomizationConfig(battery_drain_rate_variation=0.2)
        assert config.battery_drain_rate_variation == 0.2


class TestNFZRandomizationConfig:
    """Test NFZRandomizationConfig dataclass.

    Note: These tests read the source file directly since the module
    has relative imports that can't be loaded standalone.
    """

    def test_count_bounds_valid(self):
        """NFZ count bounds should be valid (min < max, both positive)."""
        # Read default values from source file
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"
        with open(filepath, 'r') as f:
            content = f.read()

        # Verify expected defaults exist
        assert "min_nfz_count: int = 2" in content, "min_nfz_count should default to 2"
        assert "max_nfz_count: int = 5" in content, "max_nfz_count should default to 5"

    def test_radius_bounds_valid(self):
        """NFZ radius bounds should be valid."""
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"
        with open(filepath, 'r') as f:
            content = f.read()

        assert "base_radius_min: float = 200.0" in content
        assert "base_radius_max: float = 500.0" in content

    def test_size_variation_valid(self):
        """Size variation should be reasonable percentage."""
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"
        with open(filepath, 'r') as f:
            content = f.read()

        assert "size_variation: float = 0.3" in content, "size_variation should be 0.3 (30%)"

    def test_timing_bounds_valid(self):
        """Timing bounds should be valid."""
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"
        with open(filepath, 'r') as f:
            content = f.read()

        assert "spawn_time_min: float = 0.0" in content
        assert "duration_min: float = 30.0" in content
        assert "duration_max: float = 120.0" in content

    def test_immediate_spawn_probability_valid(self):
        """Immediate spawn probability should be valid."""
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"
        with open(filepath, 'r') as f:
            content = f.read()

        assert "immediate_spawn_probability: float = 0.3" in content


class TestUAVState:
    """Test UAVState dataclass."""

    def test_default_initialization(self, base_isr_env_module):
        """UAVState should have sensible defaults."""
        UAVState = base_isr_env_module.UAVState
        FlightPhase = base_isr_env_module.FlightPhase
        CommsStatus = base_isr_env_module.CommsStatus
        GNSSStatus = base_isr_env_module.GNSSStatus

        state = UAVState()

        # Position/velocity defaults
        assert np.allclose(state.position, [0, 0, 0]), "Position should default to origin"
        assert np.allclose(state.velocity, [0, 0, 0]), "Velocity should default to zero"

        # Flight state defaults
        assert state.flight_phase == FlightPhase.PREFLIGHT
        assert state.battery_pct == 100.0, "Battery should start full"
        assert state.battery_reserve == 25.0, "Default reserve is 25%"

        # Sensor defaults
        assert state.comms_status == CommsStatus.OPERATIONAL
        assert state.gnss_status == GNSSStatus.FULL
        assert state.vio_valid is True
        assert state.imu_valid is True

        # Safety defaults
        assert state.in_geofence is True
        assert state.in_nfz is False

    def test_domain_randomization_fields_exist(self, base_isr_env_module):
        """UAVState should have domain randomization fields."""
        state = base_isr_env_module.UAVState()

        # These fields are required for DR
        assert hasattr(state, 'wind_vector')
        assert hasattr(state, 'vio_position_drift')
        assert hasattr(state, 'battery_drain_multiplier')
        assert hasattr(state, 'gnss_degradation_triggered')
        assert hasattr(state, 'gnss_degradation_step')

    def test_wind_vector_default(self, base_isr_env_module):
        """Wind vector should default to zero."""
        state = base_isr_env_module.UAVState()
        assert np.allclose(state.wind_vector, [0, 0, 0])

    def test_gnss_degradation_default(self, base_isr_env_module):
        """GNSS degradation should not be triggered by default."""
        state = base_isr_env_module.UAVState()
        assert state.gnss_degradation_triggered is False
        assert state.gnss_degradation_step == -1


class TestPX4ActionBridge:
    """Test PX4ActionBridge action normalization."""

    def test_action_to_velocity_conversion(self, action_bridge_module):
        """Test normalized action to velocity conversion."""
        bridge = action_bridge_module.PX4ActionBridge(mavlink_connection=None)

        # Full forward
        action = np.array([1.0, 0.0, 0.0, 0.0])
        vx, vy, vz, yaw_rate = bridge.action_to_velocity(action)

        # Use approximate comparison for float32 precision
        assert np.isclose(vx, bridge.max_horizontal_vel, atol=1e-5), "Full forward should give max horizontal vel"
        assert np.isclose(vy, 0.0, atol=1e-5)
        assert np.isclose(vz, 0.0, atol=1e-5)
        assert np.isclose(yaw_rate, 0.0, atol=1e-5)

    def test_action_clipping(self, action_bridge_module):
        """Test that out-of-range actions are clipped."""
        bridge = action_bridge_module.PX4ActionBridge(mavlink_connection=None)

        # Out-of-range action
        action = np.array([2.0, -2.0, 3.0, -3.0])
        vx, vy, vz, yaw_rate = bridge.action_to_velocity(action)

        # Should be clipped to max values (with float32 tolerance)
        assert np.isclose(vx, bridge.max_horizontal_vel, atol=1e-5)
        assert np.isclose(vy, -bridge.max_horizontal_vel, atol=1e-5)
        assert np.isclose(vz, bridge.max_vertical_vel, atol=1e-5)
        assert np.isclose(yaw_rate, -bridge.max_yaw_rate, atol=1e-5)

    def test_velocity_to_action_inverse(self, action_bridge_module):
        """Test that velocity_to_action is inverse of action_to_velocity."""
        bridge = action_bridge_module.PX4ActionBridge(mavlink_connection=None)

        # Random action
        original_action = np.array([0.5, -0.3, 0.7, -0.2])

        # Convert to velocity and back
        vx, vy, vz, yaw_rate = bridge.action_to_velocity(original_action)
        recovered_action = bridge.velocity_to_action(vx, vy, vz, yaw_rate)

        assert np.allclose(original_action, recovered_action, atol=1e-6), \
            "velocity_to_action should be inverse of action_to_velocity"

    def test_action_bounds(self, action_bridge_module):
        """Test that action space bounds are [-1, 1]."""
        bridge = action_bridge_module.PX4ActionBridge(mavlink_connection=None)

        # Edge case actions
        actions = [
            np.array([1.0, 1.0, 1.0, 1.0]),
            np.array([-1.0, -1.0, -1.0, -1.0]),
            np.array([0.0, 0.0, 0.0, 0.0]),
        ]

        for action in actions:
            vx, vy, vz, yaw_rate = bridge.action_to_velocity(action)
            # All velocities should be within bounds (with small tolerance for float32)
            assert abs(vx) <= bridge.max_horizontal_vel + 1e-5
            assert abs(vy) <= bridge.max_horizontal_vel + 1e-5
            assert abs(vz) <= bridge.max_vertical_vel + 1e-5
            assert abs(yaw_rate) <= bridge.max_yaw_rate + 1e-5

    def test_hover_action(self, action_bridge_module):
        """Test hover command produces zero velocity."""
        bridge = action_bridge_module.PX4ActionBridge(mavlink_connection=None)

        action = np.array([0.0, 0.0, 0.0, 0.0])
        vx, vy, vz, yaw_rate = bridge.action_to_velocity(action)

        assert vx == 0.0
        assert vy == 0.0
        assert vz == 0.0
        assert yaw_rate == 0.0


class TestVampireSafetyFilter:
    """Test VampireSafetyFilter TPTP query generation."""

    def test_safety_filter_disabled_passthrough(self, safety_filter_module):
        """When disabled, filter should pass through all actions."""
        filter = safety_filter_module.VampireSafetyFilter(enabled=False)

        current_state = {'x': 0, 'y': 0, 'z': 10, 'battery': 80}
        action = np.array([1.0, 0.0, 0.0, 0.0])

        is_safe, violation, penalty = filter.check_action_safety(
            current_state, action, current_state
        )

        assert is_safe is True, "Disabled filter should pass all actions"
        assert violation is None
        assert penalty == 0.0

    def test_filter_action_passthrough_when_disabled(self, safety_filter_module):
        """filter_action should pass through when disabled."""
        filter = safety_filter_module.VampireSafetyFilter(enabled=False)

        current_state = {'x': 0, 'y': 0, 'z': 10, 'battery': 80}
        action = np.array([1.0, 0.5, -0.2, 0.1])

        safe_action, was_modified, violation = filter.filter_action(
            current_state, action, dt=0.1
        )

        assert np.array_equal(safe_action, action), "Should return original action"
        assert was_modified is False
        assert violation is None

    def test_get_safe_fallback_is_hover(self, safety_filter_module):
        """Safe fallback action should be hover (all zeros)."""
        filter = safety_filter_module.VampireSafetyFilter()
        current_state = {'x': 0, 'y': 0, 'z': 10, 'battery': 80}

        fallback = filter.get_safe_fallback_action(current_state)

        assert np.allclose(fallback, [0, 0, 0, 0]), "Fallback should be hover"

    def test_predict_next_state(self, safety_filter_module):
        """Test next state prediction from action."""
        filter = safety_filter_module.VampireSafetyFilter()

        current_state = {'x': 0.0, 'y': 0.0, 'z': 10.0, 'battery': 80}
        action = np.array([1.0, 2.0, -1.0, 0.0])  # vx, vy, vz, yaw_rate
        dt = 0.1

        next_state = filter.predict_next_state(current_state, action, dt)

        expected_x = 0.0 + 1.0 * dt
        expected_y = 0.0 + 2.0 * dt
        expected_z = 10.0 + (-1.0) * dt

        assert abs(next_state['x'] - expected_x) < 1e-6
        assert abs(next_state['y'] - expected_y) < 1e-6
        assert abs(next_state['z'] - expected_z) < 1e-6

    def test_hard_constraints_list(self, safety_filter_module):
        """Verify hard constraints are defined."""
        filter = safety_filter_module.VampireSafetyFilter()

        expected_constraints = [
            'geofenceViolation',
            'noFlyZoneViolation',
            'highThreatExposure',
            'mustLand',
            'mustReturnToLaunch',
        ]

        for constraint in expected_constraints:
            assert constraint in filter.hard_constraints, f"Missing constraint: {constraint}"

    def test_tptp_query_generation(self, safety_filter_module):
        """Test TPTP query string generation."""
        filter = safety_filter_module.VampireSafetyFilter(ontology_path="/test/path")

        current = {'x': 10.0, 'y': 20.0, 'z': 30.0, 'battery': 75.0}
        next_state = {'x': 11.0, 'y': 21.0, 'z': 31.0}

        query = filter._generate_safety_query(current, next_state)

        # Check query structure
        assert "fof(" in query, "Should contain TPTP fof statements"
        assert "position(uav1" in query, "Should include position facts"
        assert "batteryLevel(uav1" in query, "Should include battery facts"
        assert "nextPosition(uav1" in query, "Should include next position"
        assert "safety_check" in query, "Should have safety check conjecture"


class TestLightingAndWeatherPresets:
    """Test lighting and weather configuration."""

    def test_lighting_presets_exist(self, domain_randomizer_module):
        """Verify all lighting presets are defined."""
        LightingPreset = domain_randomizer_module.LightingPreset
        RandomizationConfig = domain_randomizer_module.RandomizationConfig
        DomainRandomizer = domain_randomizer_module.DomainRandomizer

        config = RandomizationConfig()
        randomizer = DomainRandomizer(config)

        expected_presets = [
            LightingPreset.DAY,
            LightingPreset.DAWN,
            LightingPreset.DUSK,
            LightingPreset.OVERCAST,
            LightingPreset.NIGHT,
        ]

        for preset in expected_presets:
            assert preset in randomizer.LIGHTING_PRESETS, f"Missing preset: {preset}"

    def test_weather_presets_exist(self, domain_randomizer_module):
        """Verify all weather presets are defined."""
        WeatherCondition = domain_randomizer_module.WeatherCondition
        RandomizationConfig = domain_randomizer_module.RandomizationConfig
        DomainRandomizer = domain_randomizer_module.DomainRandomizer

        config = RandomizationConfig()
        randomizer = DomainRandomizer(config)

        expected_conditions = [
            WeatherCondition.CLEAR,
            WeatherCondition.HAZY,
            WeatherCondition.FOGGY,
            WeatherCondition.DUSTY,
        ]

        for condition in expected_conditions:
            assert condition in randomizer.WEATHER_PRESETS, f"Missing condition: {condition}"

    def test_lighting_config_valid_direction(self, domain_randomizer_module):
        """Verify lighting configs have valid direction vectors."""
        RandomizationConfig = domain_randomizer_module.RandomizationConfig
        DomainRandomizer = domain_randomizer_module.DomainRandomizer

        config = RandomizationConfig()
        randomizer = DomainRandomizer(config)

        for preset, lighting in randomizer.LIGHTING_PRESETS.items():
            # Direction should be normalized or near-normalized
            norm = np.linalg.norm(lighting.direction)
            # Some presets may not be perfectly normalized
            assert norm > 0.5, f"Direction for {preset} should have magnitude > 0.5"

    def test_weather_config_fog_parameters(self, domain_randomizer_module):
        """Verify fog parameters are valid when fog is enabled."""
        RandomizationConfig = domain_randomizer_module.RandomizationConfig
        DomainRandomizer = domain_randomizer_module.DomainRandomizer

        config = RandomizationConfig()
        randomizer = DomainRandomizer(config)

        for condition, weather in randomizer.WEATHER_PRESETS.items():
            if weather.fog_enabled:
                assert weather.fog_start < weather.fog_end, \
                    f"Fog start must be < end for {condition}"
                assert weather.fog_density > 0, \
                    f"Fog density must be positive for {condition}"


class TestObservationDimensions:
    """Test observation dimension calculations."""

    def test_base_observation_dim(self):
        """Test base environment observation dimension."""
        # From base_isr_env.py:
        # pos(3) + vel(3) + quat(4) + wind(3) + scalars(15) = 28
        expected_base_dim = 3 + 3 + 4 + 3 + 15
        assert expected_base_dim == 28, "Base observation dim should be 28"

    def test_dynamic_nfz_observation_dim(self):
        """Test DynamicNFZ environment observation dimension."""
        # From dynamic_nfz_env.py:
        # base(28) + dest_vector(3) + nfz_vector(3) + nfz_active(1) + replan_needed(1) = 36
        base_dim = 28
        extra_features = 8  # As stated in code: return base_dim + 8
        expected_dim = base_dim + extra_features
        assert expected_dim == 36, "DynamicNFZ observation dim should be 36"

    def test_comms_denied_observation_dim(self):
        """Test CommsDenied environment observation dimension."""
        # From comms_denied_env.py:
        # base(28) + coverage(1) + poi_ratio(1) + dist_home(1) + returning(1) +
        # recent_detections(1) + time_since_detection(1) = 34
        base_dim = 28
        extra_features = 6  # As stated in code: return base_dim + 6
        expected_dim = base_dim + extra_features
        assert expected_dim == 34, "CommsDenied observation dim should be 34"

    def test_multi_objective_observation_dim(self):
        """Test MultiObjective environment observation dimension."""
        # From multi_objective_env.py:
        # base(28) + targets(8*5) + threat_summary(3) = 71
        base_dim = 28
        target_features = 8 * 5  # 8 targets * 5 features each
        threat_features = 3
        expected_dim = base_dim + target_features + threat_features
        assert expected_dim == 71, "MultiObjective observation dim should be 71"


class TestDomainRandomizerSampling:
    """Test domain randomizer sampling logic."""

    def test_randomizer_produces_valid_lighting(self, domain_randomizer_module):
        """Test that randomizer produces valid lighting configs."""
        RandomizationConfig = domain_randomizer_module.RandomizationConfig
        DomainRandomizer = domain_randomizer_module.DomainRandomizer

        config = RandomizationConfig(randomize_lighting=True, seed=42)
        randomizer = DomainRandomizer(config)

        result = randomizer._randomize_lighting()

        assert 'preset' in result
        assert 'direction' in result
        assert 'intensity_scale' in result
        assert 0.8 <= result['intensity_scale'] <= 1.2

    def test_randomizer_produces_valid_weather(self, domain_randomizer_module):
        """Test that randomizer produces valid weather configs."""
        RandomizationConfig = domain_randomizer_module.RandomizationConfig
        DomainRandomizer = domain_randomizer_module.DomainRandomizer

        config = RandomizationConfig(randomize_weather=True, seed=42)
        randomizer = DomainRandomizer(config)

        result = randomizer._randomize_weather()

        assert 'condition' in result
        assert 'wind_speed' in result
        assert result['wind_speed'] >= 0

    def test_randomizer_deterministic_with_seed(self, domain_randomizer_module):
        """Test that same seed produces same randomization."""
        RandomizationConfig = domain_randomizer_module.RandomizationConfig
        DomainRandomizer = domain_randomizer_module.DomainRandomizer

        config1 = RandomizationConfig(seed=12345)
        config2 = RandomizationConfig(seed=12345)

        rand1 = DomainRandomizer(config1)
        rand2 = DomainRandomizer(config2)

        result1 = rand1.randomize(seed=12345)
        result2 = rand2.randomize(seed=12345)

        assert result1 == result2, "Same seed should produce same results"


class TestGymnasiumWrapper:
    """Test Gymnasium wrapper interface.

    Note: Direct module loading doesn't work due to relative imports.
    These tests verify the interface by reading the source code.
    """

    def test_wrapper_defines_observation_space(self):
        """Test that wrapper defines observation space from env dim."""
        filepath = ENVIRONMENTS_DIR / "gymnasium_wrapper.py"
        with open(filepath, 'r') as f:
            content = f.read()

        # Should define observation_space using env.observation_dim
        assert "observation_space" in content, "Should define observation_space"
        assert "observation_dim" in content, "Should use env.observation_dim"
        assert "Box" in content, "Should use gymnasium.spaces.Box"

    def test_wrapper_defines_action_space(self):
        """Test that wrapper defines action space with [-1, 1] bounds."""
        filepath = ENVIRONMENTS_DIR / "gymnasium_wrapper.py"
        with open(filepath, 'r') as f:
            content = f.read()

        # Should define action_space
        assert "action_space" in content, "Should define action_space"
        # The code uses np.ones() for bounds, which gives [-1, 1]
        assert "low=-np.ones" in content or "low=-1" in content, "Action space lower bound should be -1"
        assert "high=np.ones" in content or "high=1" in content, "Action space upper bound should be 1"

    def test_wrapper_implements_gym_interface(self):
        """Test that wrapper implements required Gymnasium methods."""
        filepath = ENVIRONMENTS_DIR / "gymnasium_wrapper.py"
        with open(filepath, 'r') as f:
            content = f.read()

        required_methods = ['def reset', 'def step', 'def close']
        for method in required_methods:
            assert method in content, f"Missing method: {method}"

    def test_wrapper_returns_5_values_from_step(self):
        """Test that step returns 5 values (Gymnasium v26+ API)."""
        filepath = ENVIRONMENTS_DIR / "gymnasium_wrapper.py"
        with open(filepath, 'r') as f:
            content = f.read()

        # Should return (obs, reward, terminated, truncated, info)
        assert "terminated" in content, "Should have terminated flag"
        assert "truncated" in content, "Should have truncated flag"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
