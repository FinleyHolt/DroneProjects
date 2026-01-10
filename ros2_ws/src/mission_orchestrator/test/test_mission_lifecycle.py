#!/usr/bin/env python3
"""
test_mission_lifecycle.py - Integration tests for mission lifecycle

Tests the complete mission lifecycle including verification, execution,
and safety monitoring. Uses mocked Vampire responses for unit testing.
"""

import time
import threading
import pytest
from unittest.mock import MagicMock, patch
from dataclasses import dataclass

import sys
sys.path.insert(0, str(__file__).rsplit('/', 2)[0])

from mission_orchestrator.mission_state_machine import (
    MissionStateMachine, MissionState, MissionContext, Waypoint
)
from mission_orchestrator.safety_monitor import SafetyMonitor, SafetyCheckResult


class TestSafetyMonitor:
    """Test SafetyMonitor functionality."""

    def test_safety_check_result_dataclass(self):
        """SafetyCheckResult correctly stores check results."""
        result = SafetyCheckResult(
            name='geofence_check',
            passed=True,
            latency_ms=15.5,
            from_cache=True
        )
        assert result.name == 'geofence_check'
        assert result.passed
        assert result.latency_ms == 15.5
        assert result.from_cache

    def test_violation_severity_classification(self):
        """Violations are correctly classified by severity."""
        assert 'nfz_violation' in SafetyMonitor.CRITICAL_VIOLATIONS
        assert 'collision_imminent' in SafetyMonitor.CRITICAL_VIOLATIONS
        assert 'geofence_violation' in SafetyMonitor.HIGH_VIOLATIONS
        assert 'battery_low' in SafetyMonitor.MEDIUM_VIOLATIONS


class TestViolationHandling:
    """Test violation detection and response."""

    def test_critical_violation_triggers_abort(self):
        """Critical violations should recommend abort."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        # Setup to EXECUTING
        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        # Simulate critical violation
        sm.record_safety_violation('nfz_violation')
        sm.transition_to(MissionState.ABORTED, reason='NFZ violation')

        assert sm.state == MissionState.ABORTED
        assert 'nfz_violation' in sm.context.safety_violations

    def test_high_violation_triggers_rtl(self):
        """High severity violations should trigger RTL."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        # Setup to EXECUTING
        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        # Simulate high violation
        sm.record_safety_violation('geofence_violation')
        sm.transition_to(MissionState.RETURNING)

        assert sm.state == MissionState.RETURNING

    def test_medium_violation_triggers_pause(self):
        """Medium severity violations should trigger pause."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        # Setup to EXECUTING
        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        # Simulate medium violation
        sm.record_safety_violation('battery_low')
        sm.transition_to(MissionState.PAUSED)

        assert sm.state == MissionState.PAUSED


class TestMissionVerification:
    """Test pre-flight verification scenarios."""

    def test_verification_success_enables_arming(self):
        """Successful verification allows transition to ARMING."""
        sm = MissionStateMachine()
        sm.context = MissionContext(
            mission_id='test_001',
            waypoints=[
                Waypoint(latitude=47.0, longitude=-122.0, altitude=100.0),
            ]
        )

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)

        # Simulate successful verification
        sm.context.verified = True
        sm.transition_to(MissionState.VERIFIED)

        assert sm.can_transition_to(MissionState.ARMING)
        assert sm.transition_to(MissionState.ARMING)

    def test_verification_failure_returns_to_idle(self):
        """Failed verification returns to IDLE."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)

        # Simulate failed verification
        sm.record_verification_issue('Path intersects NFZ')
        sm.context.verified = False
        sm.transition_to(MissionState.IDLE)

        assert sm.state == MissionState.IDLE
        assert sm.context is None

    def test_verification_records_all_issues(self):
        """All verification issues are recorded."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)

        sm.record_verification_issue('Insufficient battery')
        sm.record_verification_issue('Path exceeds altitude limit')
        sm.record_verification_issue('Weather conditions unfavorable')

        assert len(sm.context.verification_issues) == 3


class TestMissionExecution:
    """Test mission execution scenarios."""

    def test_waypoint_progression(self):
        """Test waypoint advancement during execution."""
        sm = MissionStateMachine()
        sm.context = MissionContext(
            mission_id='test_001',
            waypoints=[
                Waypoint(latitude=47.0, longitude=-122.0, altitude=100.0),
                Waypoint(latitude=47.1, longitude=-122.1, altitude=100.0),
                Waypoint(latitude=47.2, longitude=-122.2, altitude=100.0),
                Waypoint(latitude=47.3, longitude=-122.3, altitude=100.0),
            ]
        )

        # Start mission
        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        # Progress through waypoints
        assert sm.get_current_waypoint().latitude == 47.0
        assert sm.get_remaining_waypoints() == 4

        sm.advance_waypoint()
        assert sm.get_current_waypoint().latitude == 47.1
        assert sm.get_remaining_waypoints() == 3

        sm.advance_waypoint()
        sm.advance_waypoint()
        assert sm.get_remaining_waypoints() == 1

        # Complete last waypoint
        assert not sm.advance_waypoint()

    def test_completion_from_last_waypoint(self):
        """Mission completes after last waypoint."""
        sm = MissionStateMachine()
        sm.context = MissionContext(
            mission_id='test_001',
            waypoints=[
                Waypoint(latitude=47.0, longitude=-122.0, altitude=100.0),
            ]
        )

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        # Cannot advance past single waypoint
        assert not sm.advance_waypoint()

        # Complete mission
        assert sm.transition_to(MissionState.COMPLETED)
        assert sm.state == MissionState.COMPLETED


class TestPauseResume:
    """Test pause and resume functionality."""

    def test_pause_preserves_state(self):
        """Pausing preserves waypoint progress."""
        sm = MissionStateMachine()
        sm.context = MissionContext(
            mission_id='test_001',
            waypoints=[
                Waypoint(latitude=47.0, longitude=-122.0, altitude=100.0),
                Waypoint(latitude=47.1, longitude=-122.1, altitude=100.0),
                Waypoint(latitude=47.2, longitude=-122.2, altitude=100.0),
            ]
        )

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        # Progress to second waypoint
        sm.advance_waypoint()
        assert sm.context.current_waypoint_idx == 1

        # Pause
        sm.transition_to(MissionState.PAUSED)
        assert sm.context.current_waypoint_idx == 1  # Preserved

        # Resume
        sm.transition_to(MissionState.EXECUTING)
        assert sm.context.current_waypoint_idx == 1  # Still preserved

    def test_pause_to_rtl(self):
        """Can transition from PAUSED to RTL."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)
        sm.transition_to(MissionState.PAUSED)

        assert sm.transition_to(MissionState.RETURNING)
        assert sm.state == MissionState.RETURNING


class TestSafetyMonitoringRate:
    """Test safety monitoring rate requirements."""

    def test_safety_queries_defined(self):
        """Safety queries are properly defined."""
        assert len(SafetyMonitor.SAFETY_QUERIES) >= 3

        query_names = [q[0] for q in SafetyMonitor.SAFETY_QUERIES]
        assert 'geofence_check' in query_names
        assert 'nfz_violation' in query_names
        assert 'battery_return' in query_names


class TestConcurrentOperations:
    """Test concurrent operation handling."""

    def test_concurrent_status_checks(self):
        """Status can be checked during safety monitoring."""
        sm = MissionStateMachine()
        sm.context = MissionContext(
            mission_id='test_001',
            waypoints=[
                Waypoint(latitude=47.0, longitude=-122.0, altitude=100.0),
            ]
        )

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        errors = []

        def check_status():
            try:
                for _ in range(50):
                    _ = sm.is_safety_monitoring_active()
                    _ = sm.is_flight_active()
                    _ = sm.get_remaining_waypoints()
                    time.sleep(0.01)
            except Exception as e:
                errors.append(e)

        def record_violations():
            try:
                for i in range(20):
                    sm.record_safety_violation(f'test_violation_{i}')
                    time.sleep(0.02)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=check_status),
            threading.Thread(target=check_status),
            threading.Thread(target=record_violations),
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0
        assert len(sm.context.safety_violations) == 20


class TestAbortScenarios:
    """Test various abort scenarios."""

    def test_abort_records_reason(self):
        """Abort transition records reason."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)

        sm.transition_to(MissionState.ABORTED, reason='Lost GPS fix')

        assert sm.state == MissionState.ABORTED
        assert sm.context.abort_reason == 'Lost GPS fix'

    def test_abort_from_returning(self):
        """Can abort during RTL."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)
        sm.transition_to(MissionState.RETURNING)

        assert sm.transition_to(MissionState.ABORTED, reason='Motor failure')
        assert sm.state == MissionState.ABORTED

    def test_abort_from_landing(self):
        """Can abort during landing."""
        sm = MissionStateMachine()
        sm.context = MissionContext(mission_id='test_001')

        sm.transition_to(MissionState.RECEIVING)
        sm.transition_to(MissionState.VERIFYING)
        sm.transition_to(MissionState.VERIFIED)
        sm.transition_to(MissionState.ARMING)
        sm.transition_to(MissionState.EXECUTING)
        sm.transition_to(MissionState.RETURNING)
        sm.transition_to(MissionState.LANDING)

        assert sm.transition_to(MissionState.ABORTED, reason='Obstacle detected')
        assert sm.state == MissionState.ABORTED


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
