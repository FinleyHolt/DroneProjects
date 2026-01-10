#!/usr/bin/env python3
"""
Unit tests for OntologyBehaviorController.

Tests the ontology-triggered behaviors:
- RTL triggers (battery reserve, comms-denied)
- Emergency landing triggers (critical battery, lost localization)
- Geofence recovery
- NFZ avoidance
- Integration with Gymnasium wrapper

Author: Finley Holt
"""

import pytest
import numpy as np
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


class TestOntologyBehaviorController:
    """Tests for the OntologyBehaviorController class."""

    @pytest.fixture
    def mock_modules(self, isaac_sim_mocks):
        """Patch Isaac Sim modules for isolated testing."""
        with patch.dict(sys.modules, isaac_sim_mocks):
            yield

    @pytest.fixture
    def controller(self, mock_modules):
        """Create an OntologyBehaviorController for testing."""
        from environments.ontology_behavior_controller import (
            OntologyBehaviorController,
            OntologyControllerConfig,
        )
        config = OntologyControllerConfig(
            use_vampire=False,  # Don't require Vampire for unit tests
            battery_rtl_threshold=25.0,
            battery_rtl_comms_denied_threshold=30.0,
            battery_emergency_threshold=15.0,
        )
        controller = OntologyBehaviorController(config)
        controller.set_home_position(np.array([0.0, 0.0, 0.0]))
        return controller

    @pytest.fixture
    def healthy_state(self, mock_modules):
        """Create a healthy UAV state (no violations)."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        return UAVState(
            position=np.array([100.0, 50.0, 30.0]),
            velocity=np.array([5.0, 0.0, 0.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=80.0,
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

    @pytest.fixture
    def low_battery_state(self, mock_modules):
        """Create state with low battery (should trigger RTL).

        Battery calculation: required = (distance_km * 2.5) + 10 + altitude_cost
        At 4km from home: (4 * 2.5) + 10 = 20% required
        With 18% battery, should trigger RTL.
        """
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        return UAVState(
            position=np.array([4000.0, 0.0, 50.0]),  # 4km from home
            velocity=np.array([5.0, 0.0, 0.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=18.0,  # Below 20% required for 4km distance
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

    @pytest.fixture
    def comms_denied_low_battery_state(self, mock_modules):
        """Create state with comms denied and moderately low battery."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        return UAVState(
            position=np.array([100.0, 50.0, 30.0]),
            velocity=np.array([5.0, 0.0, 0.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=28.0,  # Below 30% comms-denied threshold
            comms_status=CommsStatus.DENIED,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

    @pytest.fixture
    def critical_battery_state(self, mock_modules):
        """Create state with critical battery (should trigger emergency land)."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        return UAVState(
            position=np.array([100.0, 50.0, 30.0]),
            velocity=np.array([5.0, 0.0, 0.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=12.0,  # Below 15% emergency threshold
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

    @pytest.fixture
    def lost_localization_state(self, mock_modules):
        """Create state with lost localization (should trigger emergency land)."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        return UAVState(
            position=np.array([100.0, 50.0, 30.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=80.0,
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.DENIED,
            vio_valid=False,  # Both GNSS and VIO failed
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

    @pytest.fixture
    def geofence_violation_state(self, mock_modules):
        """Create state outside geofence."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        return UAVState(
            position=np.array([200.0, 100.0, 30.0]),
            velocity=np.array([5.0, 0.0, 0.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=80.0,
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=False,  # Outside geofence!
            in_nfz=False,
            nfz_distance=100.0,
        )

    @pytest.fixture
    def nfz_violation_state(self, mock_modules):
        """Create state inside NFZ."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        return UAVState(
            position=np.array([100.0, 50.0, 30.0]),
            velocity=np.array([5.0, 0.0, 0.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=80.0,
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=True,  # Inside NFZ!
            current_nfz_id="test_nfz",
            nfz_distance=0.0,
        )

    # === Test: Healthy state (no preemption) ===

    def test_healthy_state_no_preemption(self, controller, healthy_state):
        """Controller should not preempt RL when state is healthy."""
        from environments.ontology_behavior_controller import OntologyBehavior

        command = controller.update(healthy_state)

        assert command is None, "Should not preempt RL for healthy state"
        assert not controller.rtl_triggered
        assert not controller.emergency_triggered
        assert len(controller.active_violations) == 0

    # === Test: RTL triggers ===

    def test_low_battery_triggers_rtl(self, controller, low_battery_state):
        """Low battery far from home should trigger RTL."""
        from environments.ontology_behavior_controller import OntologyBehavior

        command = controller.update(low_battery_state)

        assert command is not None, "Should trigger behavior for low battery"
        assert command.behavior == OntologyBehavior.RTL
        assert command.preempts_rl
        assert controller.rtl_triggered
        assert "batteryReserveReturn" in [v.axiom_name for v in controller.active_violations]

    def test_comms_denied_low_battery_triggers_rtl(self, controller, comms_denied_low_battery_state):
        """Comms denied with battery below 30% should trigger RTL."""
        from environments.ontology_behavior_controller import OntologyBehavior

        command = controller.update(comms_denied_low_battery_state)

        assert command is not None, "Should trigger RTL for comms-denied low battery"
        assert command.behavior == OntologyBehavior.RTL
        assert command.preempts_rl
        assert controller.rtl_triggered
        assert "commsDeniedBatteryReserve" in [v.axiom_name for v in controller.active_violations]

    def test_rtl_command_has_home_target(self, controller, low_battery_state):
        """RTL command should target home position."""
        command = controller.update(low_battery_state)

        assert command is not None
        assert command.target_position is not None
        np.testing.assert_array_almost_equal(command.target_position, np.array([0.0, 0.0, 0.0]))

    # === Test: Emergency landing triggers ===

    def test_critical_battery_triggers_emergency_land(self, controller, critical_battery_state):
        """Critical battery should trigger emergency landing."""
        from environments.ontology_behavior_controller import OntologyBehavior

        command = controller.update(critical_battery_state)

        assert command is not None
        assert command.behavior == OntologyBehavior.EMERGENCY_LAND
        assert command.preempts_rl
        assert controller.emergency_triggered

    def test_lost_localization_triggers_emergency_land(self, controller, lost_localization_state):
        """Lost localization (VIO + GNSS) should trigger emergency landing."""
        from environments.ontology_behavior_controller import OntologyBehavior

        command = controller.update(lost_localization_state)

        assert command is not None
        assert command.behavior == OntologyBehavior.EMERGENCY_LAND
        assert command.preempts_rl
        assert controller.emergency_triggered
        assert "localizationLost" in [v.axiom_name for v in controller.active_violations]

    def test_imu_failure_triggers_emergency_land(self, mock_modules, controller):
        """IMU failure should trigger emergency landing."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        from environments.ontology_behavior_controller import OntologyBehavior

        state = UAVState(
            position=np.array([100.0, 50.0, 30.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=80.0,
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=False,  # IMU failed!
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

        command = controller.update(state)

        assert command is not None
        assert command.behavior == OntologyBehavior.EMERGENCY_LAND
        assert "imuFailure" in [v.axiom_name for v in controller.active_violations]

    # === Test: Geofence recovery ===

    def test_geofence_violation_triggers_recovery(self, controller, geofence_violation_state):
        """Geofence violation should trigger recovery behavior."""
        from environments.ontology_behavior_controller import OntologyBehavior

        command = controller.update(geofence_violation_state)

        assert command is not None
        assert command.behavior == OntologyBehavior.GEOFENCE_RECOVERY
        assert command.preempts_rl
        assert command.target_position is not None  # Should have recovery target
        assert "geofenceViolation" in [v.axiom_name for v in controller.active_violations]

    # === Test: NFZ avoidance ===

    def test_nfz_violation_triggers_avoidance(self, controller, nfz_violation_state):
        """NFZ entry should trigger avoidance behavior."""
        from environments.ontology_behavior_controller import OntologyBehavior

        command = controller.update(nfz_violation_state)

        assert command is not None
        assert command.behavior == OntologyBehavior.NFZ_AVOIDANCE
        assert command.preempts_rl
        assert "noFlyZoneViolation" in [v.axiom_name for v in controller.active_violations]

    def test_nfz_proximity_triggers_hover(self, mock_modules, controller):
        """Close proximity to NFZ should trigger hover."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        from environments.ontology_behavior_controller import OntologyBehavior

        state = UAVState(
            position=np.array([100.0, 50.0, 30.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=80.0,
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=5.0,  # Very close to NFZ
        )

        command = controller.update(state)

        assert command is not None
        assert command.behavior == OntologyBehavior.HOVER
        assert command.preempts_rl

    # === Test: Priority ordering ===

    def test_emergency_takes_priority_over_rtl(self, mock_modules, controller):
        """Emergency conditions should take priority over RTL conditions."""
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )
        from environments.ontology_behavior_controller import OntologyBehavior

        # State with both low battery AND lost localization
        state = UAVState(
            position=np.array([500.0, 300.0, 50.0]),
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=12.0,  # Critical battery (emergency)
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.DENIED,
            vio_valid=False,  # Lost localization (emergency)
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

        command = controller.update(state)

        # Emergency should take priority
        assert command is not None
        assert command.behavior == OntologyBehavior.EMERGENCY_LAND

    # === Test: Controller reset ===

    def test_reset_clears_state(self, controller, low_battery_state):
        """Reset should clear all triggered states."""
        # Trigger RTL
        controller.update(low_battery_state)
        assert controller.rtl_triggered

        # Reset
        controller.reset()

        assert not controller.rtl_triggered
        assert not controller.emergency_triggered
        assert len(controller.active_violations) == 0
        assert controller.active_behavior is None

    # === Test: Statistics tracking ===

    def test_stats_tracking(self, controller, low_battery_state, nfz_violation_state):
        """Controller should track statistics."""
        # Trigger RTL
        controller.update(low_battery_state)
        stats = controller.get_stats()
        assert stats['rtl_triggers'] == 1

        # Reset and trigger NFZ
        controller.reset()
        controller.set_home_position(np.array([0.0, 0.0, 0.0]))
        controller.update(nfz_violation_state)
        stats = controller.get_stats()
        assert stats['nfz_avoidances'] == 1

    # === Test: Distance-based battery calculation ===

    def test_distance_based_battery_reserve(self, mock_modules, controller):
        """Battery reserve should scale with distance from home.

        Battery calculation: required = (distance_km * 2.5) + 10 + altitude_cost

        At 0.5km:  required = (0.5 * 2.5) + 10 = 11.25%
        At 4.0km:  required = (4.0 * 2.5) + 10 = 20.0%
        At 8.0km:  required = (8.0 * 2.5) + 10 = 30.0%

        With 12% battery:
        - Close (0.5km): 12% > 11.25% → OK (barely, but no RTL)
        - Far (5km): 12% < 22.5% → trigger RTL
        """
        from environments.base_isr_env import (
            UAVState, FlightPhase, CommsStatus, GNSSStatus
        )

        # Close to home - lower reserve needed
        # 500m = 0.5km → required = 11.25%, with 16% battery = OK (above required and emergency)
        close_state = UAVState(
            position=np.array([500.0, 0.0, 30.0]),  # 500m from home
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=16.0,  # Above emergency (15%) and required (11.25%)
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

        # Far from home - higher reserve needed
        # 5000m = 5km → required = 22.5%, with 16% battery = RTL (not emergency)
        far_state = UAVState(
            position=np.array([5000.0, 0.0, 30.0]),  # 5km from home
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=16.0,  # Above emergency (15%) but below required (22.5%)
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

        # Close state should NOT trigger RTL (can make it back)
        close_command = controller.update(close_state)
        assert close_command is None, "Should not trigger RTL when close to home"

        controller.reset()
        controller.set_home_position(np.array([0.0, 0.0, 0.0]))

        # Far state SHOULD trigger RTL (not enough battery for distance)
        far_command = controller.update(far_state)
        assert far_command is not None, "Should trigger RTL when far from home"
        from environments.ontology_behavior_controller import OntologyBehavior
        assert far_command.behavior == OntologyBehavior.RTL


class TestOntologyBehaviorExecutor:
    """Tests for the OntologyBehaviorExecutor class."""

    @pytest.fixture
    def mock_modules(self, isaac_sim_mocks):
        """Patch Isaac Sim modules."""
        with patch.dict(sys.modules, isaac_sim_mocks):
            yield

    @pytest.fixture
    def executor(self, mock_modules):
        """Create an executor for testing."""
        from environments.ontology_behavior_controller import OntologyBehaviorExecutor
        return OntologyBehaviorExecutor()

    @pytest.fixture
    def rtl_command(self, mock_modules):
        """Create an RTL command."""
        from environments.ontology_behavior_controller import (
            BehaviorCommand, OntologyBehavior, AxiomPriority
        )
        return BehaviorCommand(
            behavior=OntologyBehavior.RTL,
            priority=AxiomPriority.HIGH,
            reason="Low battery",
            target_position=np.array([0.0, 0.0, 0.0]),
        )

    @pytest.fixture
    def emergency_command(self, mock_modules):
        """Create an emergency land command."""
        from environments.ontology_behavior_controller import (
            BehaviorCommand, OntologyBehavior, AxiomPriority
        )
        return BehaviorCommand(
            behavior=OntologyBehavior.EMERGENCY_LAND,
            priority=AxiomPriority.CRITICAL,
            reason="Critical failure",
        )

    def test_rtl_execution_returns_velocity_toward_home(self, mock_modules, executor, rtl_command):
        """RTL execution should return velocity toward home."""
        from environments.base_isr_env import UAVState, FlightPhase

        state = UAVState(
            position=np.array([100.0, 0.0, 30.0]),  # 100m East of home
            flight_phase=FlightPhase.MISSION_EXECUTION,
        )

        action = executor.execute(rtl_command, state)

        # Should have negative X component (moving West toward home)
        assert action[0] < 0, "Should fly toward home (negative X)"
        assert np.abs(action[1]) < 0.1, "Minimal Y movement"

    def test_emergency_land_descends(self, mock_modules, executor, emergency_command):
        """Emergency landing should command descent."""
        from environments.base_isr_env import UAVState, FlightPhase

        state = UAVState(
            position=np.array([100.0, 50.0, 30.0]),  # At 30m altitude
            flight_phase=FlightPhase.MISSION_EXECUTION,
        )

        action = executor.execute(emergency_command, state)

        # In Z-up (Isaac Sim): negative vz = descend
        # Default executor uses z_up=True
        assert action[2] < 0, "Should descend (negative Z in Z-up frame)"
        # Minimal horizontal movement
        assert np.abs(action[0]) < 0.1
        assert np.abs(action[1]) < 0.1

    def test_emergency_land_completes_on_ground(self, mock_modules, executor, emergency_command):
        """Emergency landing should mark complete when on ground."""
        from environments.base_isr_env import UAVState, FlightPhase

        state = UAVState(
            position=np.array([100.0, 50.0, 0.3]),  # Nearly on ground
            flight_phase=FlightPhase.LANDING,
        )

        executor.execute(emergency_command, state)

        assert executor.is_complete, "Should mark complete when on ground"


class TestGymnasiumWrapperOntologyIntegration:
    """Tests for ontology integration in the Gymnasium wrapper."""

    @pytest.fixture
    def mock_modules(self, isaac_sim_mocks):
        """Patch Isaac Sim modules."""
        with patch.dict(sys.modules, isaac_sim_mocks):
            yield

    def test_wrapper_creates_ontology_controller(self, mock_modules):
        """Wrapper should create ontology controller on init."""
        from environments.gymnasium_wrapper import IsaacSimGymWrapper
        from environments.base_isr_env import BaseISREnvironment, EnvironmentConfig

        # Create mock environment
        mock_env = MagicMock(spec=BaseISREnvironment)
        mock_env.observation_dim = 28
        mock_env.action_dim = 4
        mock_env.config = MagicMock()
        mock_env.config.spawn_position = np.array([0.0, 0.0, 0.0])

        wrapper = IsaacSimGymWrapper(mock_env, {})

        assert wrapper.ontology_controller is not None
        assert wrapper.behavior_executor is not None

    def test_wrapper_reset_resets_ontology_controller(self, mock_modules):
        """Wrapper reset should reset ontology controller."""
        from environments.gymnasium_wrapper import IsaacSimGymWrapper
        from environments.base_isr_env import UAVState, FlightPhase

        # Create mock environment
        mock_env = MagicMock()
        mock_env.observation_dim = 28
        mock_env.action_dim = 4
        mock_env.config = MagicMock()
        mock_env.config.spawn_position = np.array([0.0, 0.0, 0.0])
        mock_env.reset.return_value = UAVState(position=np.array([0.0, 0.0, 0.0]))
        mock_env.state_to_observation.return_value = np.zeros(28, dtype=np.float32)

        wrapper = IsaacSimGymWrapper(mock_env, {})

        # Trigger some state
        wrapper.ontology_controller._rtl_triggered = True

        # Reset
        obs, info = wrapper.reset()

        assert not wrapper.ontology_controller.rtl_triggered
        assert "ontology_preempting" in info
        assert not info["ontology_preempting"]

    def test_wrapper_step_checks_ontology(self, mock_modules):
        """Wrapper step should check ontology and potentially preempt RL.

        Battery calculation: required = (distance_km * 2.5) + 10 + altitude_cost
        At 4km: required = 20%, with 18% battery = RTL
        """
        from environments.gymnasium_wrapper import IsaacSimGymWrapper
        from environments.base_isr_env import UAVState, FlightPhase, CommsStatus, GNSSStatus

        # Create mock environment with low battery state
        # 4km from home, 18% battery < 20% required → triggers RTL
        low_battery_state = UAVState(
            position=np.array([4000.0, 0.0, 50.0]),  # 4km from home
            flight_phase=FlightPhase.MISSION_EXECUTION,
            battery_pct=18.0,  # Below 20% required for 4km
            comms_status=CommsStatus.OPERATIONAL,
            gnss_status=GNSSStatus.FULL,
            vio_valid=True,
            imu_valid=True,
            in_geofence=True,
            in_nfz=False,
            nfz_distance=100.0,
        )

        mock_env = MagicMock()
        mock_env.observation_dim = 28
        mock_env.action_dim = 4
        mock_env.config = MagicMock()
        mock_env.config.spawn_position = np.array([0.0, 0.0, 0.0])
        mock_env.uav_state = low_battery_state
        mock_env.step.return_value = (low_battery_state, 0.0, False, {})
        mock_env.state_to_observation.return_value = np.zeros(28, dtype=np.float32)

        wrapper = IsaacSimGymWrapper(mock_env, {})
        wrapper.ontology_controller.set_home_position(np.array([0.0, 0.0, 0.0]))

        # Step with arbitrary action
        action = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        obs, reward, terminated, truncated, info = wrapper.step(action)

        # Should be preempting
        assert info["ontology_preempting"], "Should preempt for low battery"
        assert info["ontology_behavior"] == "RTL"
        assert info["ontology_rtl_triggered"]
