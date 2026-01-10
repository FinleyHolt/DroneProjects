#!/usr/bin/env python3
"""
test_mission_state_machine.py - Unit tests for MissionStateMachine

Tests state transition validation, thread safety, and context management.
"""

import threading
import time
import pytest

import sys
sys.path.insert(0, str(__file__).rsplit('/', 2)[0])

from mission_orchestrator.mission_state_machine import (
    MissionStateMachine, MissionState, MissionContext, Waypoint
)


class TestMissionStateTransitions:
    """Test valid and invalid state transitions."""

    def test_initial_state_is_idle(self):
        """State machine starts in IDLE state."""
        sm = MissionStateMachine()
        assert sm.state == MissionState.IDLE

    def test_valid_transition_idle_to_receiving(self):
        """Can transition from IDLE to RECEIVING."""
        sm = MissionStateMachine()
        assert sm.can_transition_to(MissionState.RECEIVING)
        assert sm.transition_to(MissionState.RECEIVING)
        assert sm.state == MissionState.RECEIVING

    def test_invalid_transition_idle_to_executing(self):
        """Cannot jump from IDLE to EXECUTING."""
        sm = MissionStateMachine()
        assert not sm.can_transition_to(MissionState.EXECUTING)
        assert not sm.transition_to(MissionState.EXECUTING)
        assert sm.state == MissionState.IDLE

    def test_full_mission_lifecycle(self):
        """Test complete mission lifecycle: IDLE -> COMPLETED -> IDLE."""
        sm = MissionStateMachine()

        # Mission upload and verification
        assert sm.transition_to(MissionState.RECEIVING)
        assert sm.transition_to(MissionState.VERIFYING)
        assert sm.transition_to(MissionState.VERIFIED)

        # Start mission
        assert sm.transition_to(MissionState.ARMING)
        assert sm.transition_to(MissionState.EXECUTING)

        # Complete mission
        assert sm.transition_to(MissionState.COMPLETED)
        assert sm.transition_to(MissionState.IDLE)

    def test_abort_from_executing(self):
        """Can abort from EXECUTING state."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        assert sm.transition_to(MissionState.ABORTED, reason='Test abort')
        assert sm.state == MissionState.ABORTED
        assert sm.context.abort_reason == 'Test abort'

    def test_pause_and_resume(self):
        """Can pause and resume execution."""
        sm = MissionStateMachine()

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        # Pause
        assert sm.transition_to(MissionState.PAUSED)
        assert sm.state == MissionState.PAUSED

        # Resume
        assert sm.transition_to(MissionState.EXECUTING)
        assert sm.state == MissionState.EXECUTING

    def test_return_to_launch_sequence(self):
        """Test RTL sequence: EXECUTING -> RETURNING -> LANDING -> COMPLETED."""
        sm = MissionStateMachine()

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        assert sm.transition_to(MissionState.RETURNING)
        assert sm.transition_to(MissionState.LANDING)
        assert sm.transition_to(MissionState.COMPLETED)

    def test_cannot_skip_verification(self):
        """Cannot go directly from RECEIVING to ARMING."""
        sm = MissionStateMachine()

        sm.transition_to(MissionState.RECEIVING)
        assert not sm.transition_to(MissionState.ARMING)
        assert sm.state == MissionState.RECEIVING


class TestMissionContext:
    """Test mission context management."""

    def test_context_cleared_on_idle(self):
        """Context is cleared when returning to IDLE."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        assert sm.context is not None

        sm.transition_to(MissionState.IDLE)
        assert sm.context is None

    def test_waypoint_management(self):
        """Test waypoint tracking."""
        sm = MissionStateMachine()
        sm.context = MissionContext(
            mission_id='test_001',
            waypoints=[
                Waypoint(latitude=47.0, longitude=-122.0, altitude=100.0),
                Waypoint(latitude=47.1, longitude=-122.1, altitude=100.0),
                Waypoint(latitude=47.2, longitude=-122.2, altitude=100.0),
            ]
        )

        assert sm.get_remaining_waypoints() == 3
        assert sm.get_current_waypoint().latitude == 47.0

        assert sm.advance_waypoint()
        assert sm.context.current_waypoint_idx == 1
        assert sm.get_remaining_waypoints() == 2

        assert sm.advance_waypoint()
        assert sm.context.current_waypoint_idx == 2
        assert sm.get_remaining_waypoints() == 1

        # Cannot advance past last waypoint
        assert not sm.advance_waypoint()

    def test_safety_violation_recording(self):
        """Test safety violation tracking in context."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.record_safety_violation('geofence_violation')
        sm.record_safety_violation('battery_low')

        assert len(sm.context.safety_violations) == 2
        assert 'geofence_violation' in sm.context.safety_violations

    def test_verification_issue_recording(self):
        """Test verification issue tracking."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.record_verification_issue('Path intersects NFZ')
        assert 'Path intersects NFZ' in sm.context.verification_issues


class TestSafetyMonitoringStates:
    """Test safety monitoring state detection."""

    def test_safety_active_during_executing(self):
        """Safety monitoring is active during EXECUTING."""
        sm = MissionStateMachine()
        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        assert sm.is_safety_monitoring_active()

    def test_safety_active_during_paused(self):
        """Safety monitoring is active during PAUSED."""
        sm = MissionStateMachine()
        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)
        sm.transition_to(MissionState.PAUSED)

        assert sm.is_safety_monitoring_active()

    def test_safety_inactive_during_idle(self):
        """Safety monitoring is inactive during IDLE."""
        sm = MissionStateMachine()
        assert not sm.is_safety_monitoring_active()

    def test_safety_inactive_during_verification(self):
        """Safety monitoring is inactive during VERIFYING."""
        sm = MissionStateMachine()
        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)

        assert not sm.is_safety_monitoring_active()

    def test_flight_active_states(self):
        """Test flight active detection."""
        sm = MissionStateMachine()

        assert not sm.is_flight_active()

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        assert sm.is_flight_active()

        sm.transition_to(MissionState.RETURNING)
        assert sm.is_flight_active()

        sm.transition_to(MissionState.LANDING)
        assert sm.is_flight_active()


class TestTransitionCallback:
    """Test state transition callback mechanism."""

    def test_callback_invoked_on_transition(self):
        """Callback is invoked on state transition."""
        transitions = []

        def on_transition(old, new):
            transitions.append((old, new))

        sm = MissionStateMachine(on_transition=on_transition)

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)

        assert len(transitions) == 2
        assert transitions[0] == (MissionState.IDLE, MissionState.RECEIVING)
        assert transitions[1] == (MissionState.RECEIVING, MissionState.VERIFYING)

    def test_callback_not_invoked_on_invalid_transition(self):
        """Callback not invoked when transition fails."""
        transitions = []

        def on_transition(old, new):
            transitions.append((old, new))

        sm = MissionStateMachine(on_transition=on_transition)

        # Invalid transition should not invoke callback
        sm.transition_to(MissionState.EXECUTING)

        assert len(transitions) == 0


class TestThreadSafety:
    """Test thread-safe operations."""

    def test_concurrent_state_reads(self):
        """Multiple threads can read state concurrently."""
        sm = MissionStateMachine()
        sm.transition_to(MissionState.RECEIVING)

        errors = []

        def read_state():
            try:
                for _ in range(100):
                    _ = sm.state
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=read_state) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0

    def test_concurrent_transitions(self):
        """Concurrent transitions are handled safely."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        # Setup to VERIFIED state
        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        # Multiple threads try to record violations
        def record_violations():
            for i in range(10):
                sm.record_safety_violation(f'violation_{threading.current_thread().name}_{i}')

        threads = [threading.Thread(target=record_violations, name=f'thread_{i}')
                   for i in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # All violations should be recorded
        assert len(sm.context.safety_violations) == 50


class TestTransitionHistory:
    """Test transition history tracking."""

    def test_history_recorded(self):
        """Transitions are recorded in history."""
        sm = MissionStateMachine()

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)

        history = sm.get_transition_history()
        assert len(history) == 3
        assert history[0] == (MissionState.IDLE, MissionState.RECEIVING)

    def test_history_limit(self):
        """History is limited to prevent unbounded growth."""
        sm = MissionStateMachine()
        sm._max_history = 5

        # Make more transitions than the limit
        for _ in range(10):
            sm.transition_to(MissionState.RECEIVING)
            sm.transition_to(MissionState.IDLE)

        history = sm.get_transition_history()
        assert len(history) <= 5


class TestReset:
    """Test force reset functionality."""

    def test_reset_to_idle(self):
        """Reset forces state to IDLE."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        sm.reset()

        assert sm.state == MissionState.IDLE
        assert sm.context is None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
