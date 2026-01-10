"""
mission_state_machine.py - State machine for UAV mission lifecycle management

Implements a finite state machine for mission orchestration with the following
states and transitions:

    IDLE -> RECEIVING -> VERIFYING -> VERIFIED -> ARMING -> EXECUTING
                                                      |
                                        PAUSED <------+
                                          |           |
                                          +---> RETURNING -> LANDING -> COMPLETED
                                          |                      |
                                          +-------> ABORTED <----+

State Descriptions:
    IDLE: No active mission, ready to receive new mission plan
    RECEIVING: Mission plan upload in progress
    VERIFYING: Pre-flight verification via Vampire planning queries
    VERIFIED: Mission passed all verification checks, ready to arm
    ARMING: Vehicle arming sequence in progress
    EXECUTING: Active flight, 20Hz safety monitoring enabled
    PAUSED: Mission temporarily paused (minor safety violation)
    RETURNING: Return-to-launch commanded
    LANDING: Landing sequence in progress
    COMPLETED: Mission completed successfully
    ABORTED: Mission aborted due to critical safety violation

Thread Safety:
    All state transitions are protected by a threading lock to ensure
    atomicity in concurrent access scenarios (e.g., safety timer callback
    vs service handler).
"""

import threading
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional


class MissionState(Enum):
    """Mission lifecycle states."""
    IDLE = auto()
    RECEIVING = auto()
    VERIFYING = auto()
    VERIFIED = auto()
    ARMING = auto()
    EXECUTING = auto()
    PAUSED = auto()
    RETURNING = auto()
    LANDING = auto()
    COMPLETED = auto()
    ABORTED = auto()


@dataclass
class Waypoint:
    """Single waypoint in a mission plan."""
    latitude: float
    longitude: float
    altitude: float
    hold_time_sec: float = 0.0
    acceptance_radius_m: float = 5.0


@dataclass
class MissionContext:
    """
    Context data associated with an active mission.

    Holds mission parameters, verification results, and execution state
    that persists across state transitions.
    """
    mission_id: str
    waypoints: list[Waypoint] = field(default_factory=list)
    verified: bool = False
    verification_issues: list[str] = field(default_factory=list)
    current_waypoint_idx: int = 0
    safety_violations: list[str] = field(default_factory=list)
    abort_reason: Optional[str] = None

    # Mission parameters from verification
    geofence_id: Optional[str] = None
    home_position: Optional[tuple[float, float, float]] = None
    battery_required_percent: float = 30.0


class MissionStateMachine:
    """
    Manages mission lifecycle with validated state transitions.

    The state machine enforces valid transition paths and maintains
    mission context throughout the lifecycle. Thread-safe for concurrent
    access from service handlers and timer callbacks.

    Attributes:
        state: Current MissionState
        context: MissionContext for active mission (None when IDLE)

    Example:
        >>> sm = MissionStateMachine()
        >>> sm.transition_to(MissionState.RECEIVING)
        True
        >>> sm.context = MissionContext(mission_id='mission_001')
        >>> sm.transition_to(MissionState.VERIFYING)
        True
    """

    # Valid state transitions map
    VALID_TRANSITIONS: dict[MissionState, list[MissionState]] = {
        MissionState.IDLE: [MissionState.RECEIVING],
        MissionState.RECEIVING: [MissionState.VERIFYING, MissionState.IDLE],
        MissionState.VERIFYING: [MissionState.VERIFIED, MissionState.IDLE],
        MissionState.VERIFIED: [MissionState.ARMING, MissionState.IDLE],
        MissionState.ARMING: [MissionState.EXECUTING, MissionState.IDLE],
        MissionState.EXECUTING: [
            MissionState.PAUSED,
            MissionState.RETURNING,
            MissionState.COMPLETED,
            MissionState.ABORTED
        ],
        MissionState.PAUSED: [
            MissionState.EXECUTING,
            MissionState.RETURNING,
            MissionState.ABORTED
        ],
        MissionState.RETURNING: [MissionState.LANDING, MissionState.ABORTED],
        MissionState.LANDING: [MissionState.COMPLETED, MissionState.ABORTED],
        MissionState.COMPLETED: [MissionState.IDLE],
        MissionState.ABORTED: [MissionState.IDLE],
    }

    # States where safety monitoring should be active (20Hz queries)
    SAFETY_ACTIVE_STATES: set[MissionState] = {
        MissionState.EXECUTING,
        MissionState.PAUSED,
        MissionState.RETURNING,
        MissionState.LANDING,
    }

    def __init__(self, on_transition: Optional[Callable[[MissionState, MissionState], None]] = None):
        """
        Initialize the state machine.

        Args:
            on_transition: Optional callback invoked on each state transition.
                          Signature: (old_state, new_state) -> None
        """
        self._state = MissionState.IDLE
        self._context: Optional[MissionContext] = None
        self._lock = threading.RLock()
        self._on_transition = on_transition

        # Transition history for debugging
        self._transition_history: list[tuple[MissionState, MissionState]] = []
        self._max_history = 100

    @property
    def state(self) -> MissionState:
        """Return current state (thread-safe)."""
        with self._lock:
            return self._state

    @property
    def context(self) -> Optional[MissionContext]:
        """Return mission context (thread-safe)."""
        with self._lock:
            return self._context

    @context.setter
    def context(self, value: Optional[MissionContext]) -> None:
        """Set mission context (thread-safe)."""
        with self._lock:
            self._context = value

    def can_transition_to(self, new_state: MissionState) -> bool:
        """
        Check if transition to new_state is valid from current state.

        Args:
            new_state: Target state to check

        Returns:
            True if transition is valid, False otherwise
        """
        with self._lock:
            valid_targets = self.VALID_TRANSITIONS.get(self._state, [])
            return new_state in valid_targets

    def transition_to(self, new_state: MissionState, reason: Optional[str] = None) -> bool:
        """
        Attempt to transition to a new state.

        Thread-safe state transition with optional reason tracking.
        If transitioning to ABORTED, the reason is stored in context.

        Args:
            new_state: Target state
            reason: Optional reason for transition (stored for ABORTED)

        Returns:
            True if transition succeeded, False if invalid transition
        """
        with self._lock:
            if not self.can_transition_to(new_state):
                return False

            old_state = self._state
            self._state = new_state

            # Track abort reason
            if new_state == MissionState.ABORTED and self._context and reason:
                self._context.abort_reason = reason

            # Track transition history
            self._transition_history.append((old_state, new_state))
            if len(self._transition_history) > self._max_history:
                self._transition_history.pop(0)

            # Clear context on return to IDLE
            if new_state == MissionState.IDLE:
                self._context = None

        # Invoke callback outside lock to prevent deadlocks
        if self._on_transition:
            self._on_transition(old_state, new_state)

        return True

    def is_safety_monitoring_active(self) -> bool:
        """Check if safety monitoring should be running at 20Hz."""
        with self._lock:
            return self._state in self.SAFETY_ACTIVE_STATES

    def is_flight_active(self) -> bool:
        """Check if vehicle is in an active flight state."""
        with self._lock:
            return self._state in {
                MissionState.EXECUTING,
                MissionState.RETURNING,
                MissionState.LANDING
            }

    def record_safety_violation(self, violation: str) -> None:
        """
        Record a safety violation in the context.

        Args:
            violation: Description of the safety violation
        """
        with self._lock:
            if self._context:
                self._context.safety_violations.append(violation)

    def record_verification_issue(self, issue: str) -> None:
        """
        Record a verification issue in the context.

        Args:
            issue: Description of the verification issue
        """
        with self._lock:
            if self._context:
                self._context.verification_issues.append(issue)

    def advance_waypoint(self) -> bool:
        """
        Advance to the next waypoint in the mission.

        Returns:
            True if advanced, False if at end of mission
        """
        with self._lock:
            if self._context is None:
                return False

            next_idx = self._context.current_waypoint_idx + 1
            if next_idx >= len(self._context.waypoints):
                return False

            self._context.current_waypoint_idx = next_idx
            return True

    def get_current_waypoint(self) -> Optional[Waypoint]:
        """Return the current target waypoint, or None if no active mission."""
        with self._lock:
            if self._context is None:
                return None
            if self._context.current_waypoint_idx >= len(self._context.waypoints):
                return None
            return self._context.waypoints[self._context.current_waypoint_idx]

    def get_remaining_waypoints(self) -> int:
        """Return number of remaining waypoints."""
        with self._lock:
            if self._context is None:
                return 0
            return max(0, len(self._context.waypoints) - self._context.current_waypoint_idx)

    def get_transition_history(self) -> list[tuple[MissionState, MissionState]]:
        """Return recent state transition history for debugging."""
        with self._lock:
            return self._transition_history.copy()

    def reset(self) -> None:
        """
        Force reset to IDLE state.

        Use only for emergency recovery or testing. Normal mission
        completion/abort should use standard transitions.
        """
        with self._lock:
            old_state = self._state
            self._state = MissionState.IDLE
            self._context = None

        if self._on_transition and old_state != MissionState.IDLE:
            self._on_transition(old_state, MissionState.IDLE)

    def __repr__(self) -> str:
        with self._lock:
            mission_id = self._context.mission_id if self._context else 'None'
            return f'MissionStateMachine(state={self._state.name}, mission={mission_id})'
