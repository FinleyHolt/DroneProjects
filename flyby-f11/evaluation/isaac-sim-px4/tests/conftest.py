#!/usr/bin/env python3
"""
Pytest configuration and shared fixtures for Flyby F-11 tests.
"""
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch
from typing import Dict, Any

import pytest
import numpy as np


# ==============================================================================
# Path Setup
# ==============================================================================

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


# ==============================================================================
# Isaac Sim Detection
# ==============================================================================

def is_isaac_sim_available() -> bool:
    """Check if we're running inside Isaac Sim environment."""
    try:
        from isaacsim import SimulationApp
        return True
    except ImportError:
        return False


ISAAC_SIM_AVAILABLE = is_isaac_sim_available()


# ==============================================================================
# Custom Markers
# ==============================================================================

def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line(
        "markers", "requires_isaac: mark test as requiring Isaac Sim (skip outside container)"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow (skip with -m 'not slow')"
    )
    config.addinivalue_line(
        "markers", "e2e: mark test as end-to-end test"
    )


def pytest_collection_modifyitems(config, items):
    """Auto-skip tests requiring Isaac Sim when not available."""
    if ISAAC_SIM_AVAILABLE:
        return

    skip_isaac = pytest.mark.skip(reason="Requires Isaac Sim (run inside container)")

    for item in items:
        if "requires_isaac" in item.keywords:
            item.add_marker(skip_isaac)


# ==============================================================================
# Isaac Sim Mocking Fixtures
# ==============================================================================

@pytest.fixture(scope="session")
def isaac_sim_mocks() -> Dict[str, MagicMock]:
    """
    Create comprehensive mocks for Isaac Sim and related dependencies.

    Returns dictionary of mock modules that can be patched into sys.modules.
    """
    mocks = {
        # Isaac Sim core
        'isaacsim': MagicMock(),
        'isaacsim.SimulationApp': MagicMock(),
        'isaacsim.core': MagicMock(),
        'isaacsim.core.api': MagicMock(),
        'isaacsim.core.api.world': MagicMock(),

        # Omni modules
        'omni': MagicMock(),
        'omni.timeline': MagicMock(),
        'omni.isaac': MagicMock(),
        'omni.isaac.core': MagicMock(),
        'omni.isaac.core.utils': MagicMock(),
        'omni.isaac.core.prims': MagicMock(),

        # Pegasus simulator
        'pegasus': MagicMock(),
        'pegasus.simulator': MagicMock(),
        'pegasus.simulator.params': MagicMock(),
        'pegasus.simulator.logic': MagicMock(),
        'pegasus.simulator.logic.backends': MagicMock(),
        'pegasus.simulator.logic.backends.px4_mavlink_backend': MagicMock(),
        'pegasus.simulator.logic.vehicles': MagicMock(),
        'pegasus.simulator.logic.vehicles.multirotor': MagicMock(),
        'pegasus.simulator.logic.interface': MagicMock(),
        'pegasus.simulator.logic.interface.pegasus_interface': MagicMock(),

        # External dependencies
        'scipy': MagicMock(),
        'scipy.spatial': MagicMock(),
        'scipy.spatial.transform': MagicMock(),
        'pymavlink': MagicMock(),
        'pymavlink.mavutil': MagicMock(),
    }

    # Configure Pegasus params
    mocks['pegasus.simulator.params'].ROBOTS = {'Iris': '/path/to/iris'}
    mocks['pegasus.simulator.params'].SIMULATION_ENVIRONMENTS = {
        'Curved Gridroom': '/path/to/gridroom'
    }

    return mocks


@pytest.fixture
def mock_isaac_modules(isaac_sim_mocks):
    """
    Fixture that patches all Isaac Sim modules for the duration of a test.

    Usage:
        def test_something(mock_isaac_modules):
            from environments.base_isr_env import SomeClass
            # Isaac Sim imports are mocked
    """
    with patch.dict(sys.modules, isaac_sim_mocks):
        yield isaac_sim_mocks


# ==============================================================================
# Environment Fixtures (for unit tests with mocks)
# ==============================================================================

@pytest.fixture
def mock_uav_state(mock_isaac_modules):
    """Create a UAVState with default values for testing."""
    from environments.base_isr_env import UAVState, FlightPhase, CommsStatus, GNSSStatus

    return UAVState(
        position=np.array([0.0, 0.0, 50.0]),
        velocity=np.array([1.0, 0.0, 0.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),  # quaternion
        flight_phase=FlightPhase.TRANSIT,
        battery_pct=85.0,
        battery_reserve=25.0,
        comms_status=CommsStatus.OPERATIONAL,
        gnss_status=GNSSStatus.FULL,
        vio_valid=True,
        imu_valid=True,
        in_geofence=True,
        in_nfz=False,
    )


@pytest.fixture
def mock_action_bridge(mock_isaac_modules):
    """Create an action bridge for testing."""
    from environments.action_bridge import PX4ActionBridge

    return PX4ActionBridge(mavlink_connection=None)


@pytest.fixture
def mock_safety_filter(mock_isaac_modules):
    """Create a safety filter for testing."""
    from environments.safety_filter import VampireSafetyFilter

    return VampireSafetyFilter(enabled=True, timeout_ms=50)


@pytest.fixture
def mock_domain_randomizer(mock_isaac_modules):
    """Create a domain randomizer for testing."""
    from environments.domain_randomizer import DomainRandomizer, RandomizationConfig

    config = RandomizationConfig(seed=42)
    return DomainRandomizer(config)


# ==============================================================================
# Test Data Fixtures
# ==============================================================================

@pytest.fixture
def sample_actions():
    """Sample actions for testing."""
    return [
        np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32),  # hover
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),  # full forward
        np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32),  # full right
        np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float32),  # full up
        np.array([0.5, 0.5, 0.0, 0.2], dtype=np.float32),  # diagonal with yaw
        np.array([-1.0, -1.0, -1.0, -1.0], dtype=np.float32),  # all negative
    ]


@pytest.fixture
def sample_nfz_config():
    """Sample NFZ configuration for testing."""
    return {
        'id': 'test_nfz_1',
        'center': np.array([100.0, 200.0, 50.0]),
        'radius': 50.0,
        'buffer_distance': 20.0,
        'is_dynamic': False,
        'is_active': True,
    }


@pytest.fixture
def sample_target_config():
    """Sample target configuration for testing."""
    return {
        'id': 'target_1',
        'position': np.array([150.0, 250.0, 0.0]),
        'priority': 1,
        'value': 100.0,
        'battery_cost': 8.0,
    }


# ==============================================================================
# Assertion Helpers
# ==============================================================================

def assert_observation_valid(obs: np.ndarray, expected_dim: int):
    """Assert observation is valid."""
    assert obs is not None, "Observation is None"
    assert isinstance(obs, np.ndarray), f"Expected ndarray, got {type(obs)}"
    assert obs.shape == (expected_dim,), f"Expected shape ({expected_dim},), got {obs.shape}"
    assert np.all(np.isfinite(obs)), "Observation contains non-finite values"


def assert_reward_valid(reward: float):
    """Assert reward is valid."""
    assert isinstance(reward, (int, float)), f"Expected numeric reward, got {type(reward)}"
    assert np.isfinite(reward), f"Reward is not finite: {reward}"


def assert_action_in_bounds(action: np.ndarray, low: float = -1.0, high: float = 1.0):
    """Assert action is within bounds."""
    assert isinstance(action, np.ndarray), f"Expected ndarray, got {type(action)}"
    assert np.all(action >= low), f"Action below lower bound: {action}"
    assert np.all(action <= high), f"Action above upper bound: {action}"


# ==============================================================================
# Test Utilities
# ==============================================================================

@pytest.fixture
def temp_file(tmp_path):
    """Create a temporary file for testing."""
    def _create_temp_file(content: str, name: str = "test.txt") -> Path:
        filepath = tmp_path / name
        filepath.write_text(content)
        return filepath
    return _create_temp_file


@pytest.fixture
def capture_print(capsys):
    """Capture print output."""
    def _capture():
        return capsys.readouterr().out
    return _capture
