"""
Ontology Behavior Controller for Flyby F-11 UAV

This controller monitors UAV state and triggers behaviors based on ontology axioms.
It serves as the "source of truth" for safety-critical behaviors, preempting RL
control when necessary.

Ontology-triggered behaviors (automatic, not learned):
- Takeoff sequence (PREFLIGHT → ARMED → TAKEOFF → MISSION_EXECUTION)
- Return-to-launch (RTL) when battery reserve violated
- Emergency landing when critical failures detected
- Landing sequence (RTL → LANDING → DISARMED)

The controller interfaces with the Vampire ATP for formal constraint verification
and uses cached rule-based checks for real-time performance.

Author: Finley Holt
"""

import subprocess
import tempfile
import os
import time
import logging
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Callable, Dict, Any
from enum import Enum, auto
import numpy as np

# Module logger
logger = logging.getLogger(__name__)

from .base_isr_env import (
    FlightPhase,
    CommsStatus,
    GNSSStatus,
    UAVState,
)


class OntologyBehavior(Enum):
    """Behaviors that can be triggered by ontology axioms."""
    NONE = auto()
    TAKEOFF = auto()           # Auto-takeoff to mission altitude
    RTL = auto()               # Return to launch
    EMERGENCY_LAND = auto()    # Immediate landing
    HOVER = auto()             # Hold position
    GEOFENCE_RECOVERY = auto() # Return to geofence
    NFZ_AVOIDANCE = auto()     # Avoid no-fly zone
    PERSON_AVOIDANCE = auto()  # Stop/retreat from person


class AxiomPriority(Enum):
    """Priority levels for ontology axioms. Higher priority preempts lower."""
    LOW = 1        # Soft constraints, penalized but allowed
    MEDIUM = 2     # Should trigger behavior soon
    HIGH = 3       # Must trigger behavior
    CRITICAL = 4   # Immediate action required (safety of flight)


@dataclass
class AxiomViolation:
    """Represents an ontology axiom violation."""
    axiom_name: str
    priority: AxiomPriority
    triggered_behavior: OntologyBehavior
    description: str
    timestamp: float = 0.0
    verified_by_vampire: bool = False


@dataclass
class BehaviorCommand:
    """Command issued by the ontology controller."""
    behavior: OntologyBehavior
    priority: AxiomPriority
    reason: str
    target_position: Optional[np.ndarray] = None
    target_velocity: Optional[np.ndarray] = None
    preempts_rl: bool = True  # Whether this command should override RL


@dataclass
class OntologyControllerConfig:
    """Configuration for the OntologyBehaviorController."""
    # Paths
    ontology_path: str = "/workspace/ontology/planning_mode"

    # Takeoff parameters (ontology-driven)
    takeoff_altitude: float = 10.0  # meters AGL for mission start
    takeoff_ascent_rate: float = 2.0  # m/s upward velocity
    takeoff_altitude_tolerance: float = 1.0  # meters
    ground_altitude_threshold: float = 0.5  # meters to consider on ground

    # Battery thresholds (from ontology axioms)
    # These are base thresholds - actual reserve is computed dynamically
    battery_rtl_threshold: float = 25.0  # Normal operations (legacy fallback)
    battery_rtl_comms_denied_threshold: float = 30.0  # More conservative
    battery_emergency_threshold: float = 15.0  # Must land immediately

    # F-11 flight parameters for distance-based battery calculation
    # From ontology Section 13: batteryPerKm=2.5%, batteryHeadroom=10%
    # Formula: required = (distance_km * battery_per_km) + headroom
    battery_per_km: float = 2.5  # % battery per km of flight
    battery_headroom: float = 10.0  # % base safety margin
    min_reserve: float = 10.0  # Hard floor for landing

    # Geofence
    geofence_recovery_buffer: float = 10.0  # meters

    # NFZ avoidance
    nfz_warning_distance: float = 30.0  # meters (start avoiding)
    nfz_critical_distance: float = 10.0  # meters (hard stop)

    # Person safety (from ontology minPersonDistance axiom)
    # UAV must maintain minimum distance from detected persons
    person_critical_distance: float = 10.0  # meters (hard stop, hover)
    person_warning_distance: float = 15.0  # meters (slow down, prepare to stop)
    person_detection_enabled: bool = True  # Enable person proximity checks

    # RTL behavior parameters
    rtl_switch_to_landing_distance: float = 3.0  # meters from home to begin landing
    rtl_max_horizontal_speed: float = 4.0  # m/s max speed during RTL
    rtl_completion_distance: float = 5.0  # meters to consider RTL complete
    rtl_completion_altitude: float = 2.0  # meters AGL to consider RTL complete
    safe_landing_altitude: float = 10.0  # meters AGL to begin final descent

    # Emergency landing parameters
    emergency_descent_rate: float = 2.0  # m/s descent rate
    ground_altitude_threshold: float = 0.5  # meters to consider on ground

    # Velocity limits (for normalization)
    max_horizontal_velocity: float = 5.0  # m/s
    max_vertical_velocity: float = 3.0  # m/s

    # Coordinate frame: Isaac Sim uses Z-up (ENU-like), PX4 uses NED
    # When sending to action bridge, Z velocities may need inversion
    isaac_sim_z_up: bool = True  # Set False if using NED-native simulation

    # Vampire ATP settings
    use_vampire: bool = True
    vampire_timeout_ms: int = 50  # Real-time constraint
    vampire_verification_interval: float = 1.0  # Verify with Vampire every N seconds

    # Behavior timing
    rtl_warning_time: float = 5.0  # Seconds of warning before RTL trigger


class OntologyBehaviorController:
    """
    Monitors UAV state and triggers behaviors based on ontology axioms.

    This controller is the "source of truth" for safety-critical behaviors.
    When an ontology axiom is violated, this controller issues a BehaviorCommand
    that preempts RL control.

    The controller uses a layered approach:
    1. Fast rule-based checks (every step, <1ms)
    2. Vampire ATP verification (periodic, validates rule-based decisions)
    3. Behavior execution via MAVLink commands

    Axiom mappings:
    - batteryReserveReturn: battery < compute_required_reserve() → RTL
    - commsDeniedBatteryReserve: comms_denied + battery < 30% → RTL
    - mustLand: critical failure (no localization, battery critical) → EMERGENCY_LAND
    - geofenceViolation: outside geofence → GEOFENCE_RECOVERY
    - noFlyZoneViolation: inside NFZ → NFZ_AVOIDANCE
    """

    def __init__(self, config: Optional[OntologyControllerConfig] = None):
        self.config = config or OntologyControllerConfig()
        self.ontology_path = Path(self.config.ontology_path)

        # Current behavior state
        self._active_behavior: Optional[BehaviorCommand] = None
        self._behavior_start_time: float = 0.0
        self._behavior_complete: bool = False

        # Violation tracking
        self._active_violations: List[AxiomViolation] = []
        self._violation_history: List[AxiomViolation] = []

        # Vampire verification state
        self._last_vampire_check: float = 0.0
        self._vampire_available = self._check_vampire_available()

        # Takeoff state (ontology-driven)
        self._takeoff_complete: bool = False
        self._takeoff_in_progress: bool = False

        # RTL state
        self._rtl_triggered: bool = False
        self._rtl_reason: str = ""
        self._home_position: np.ndarray = np.zeros(3)

        # Emergency state
        self._emergency_triggered: bool = False
        self._emergency_reason: str = ""

        # Callbacks for behavior execution
        self._behavior_callbacks: Dict[OntologyBehavior, Callable] = {}

        # Person detection state
        self._person_detections: List[Dict[str, Any]] = []  # Updated by perception pipeline
        self._person_avoidance_active: bool = False

        # Statistics
        self._stats = {
            'takeoff_triggers': 0,
            'rtl_triggers': 0,
            'emergency_triggers': 0,
            'nfz_avoidances': 0,
            'geofence_recoveries': 0,
            'person_avoidances': 0,
            'vampire_verifications': 0,
            'vampire_confirmations': 0,
        }

    def _check_vampire_available(self) -> bool:
        """Check if Vampire theorem prover is available."""
        import shutil
        return shutil.which('vampire') is not None

    def set_home_position(self, position: np.ndarray) -> None:
        """Set the home/launch position for RTL calculations."""
        self._home_position = position.copy()

    def _check_takeoff_required(self, state: UAVState) -> Optional[BehaviorCommand]:
        """
        Check if ontology-driven takeoff is required.

        Takeoff is triggered when:
        - Flight phase is PREFLIGHT, ARMED, or TAKEOFF
        - Altitude is below takeoff_altitude
        - Takeoff has not yet completed this episode

        This implements the ontology flight phase transitions:
        PREFLIGHT → ARMED → TAKEOFF → HOVER → MISSION_EXECUTION

        Args:
            state: Current UAV state

        Returns:
            BehaviorCommand for takeoff, or None if takeoff not needed
        """
        # If takeoff already completed, RL has control
        if self._takeoff_complete:
            return None

        # Check if we're at mission altitude
        current_altitude = state.position[2]
        target_altitude = self.config.takeoff_altitude
        tolerance = self.config.takeoff_altitude_tolerance

        if current_altitude >= target_altitude - tolerance:
            # Reached mission altitude - takeoff complete
            self._takeoff_complete = True
            self._takeoff_in_progress = False
            logger.info(
                f"[ONTOLOGY] Takeoff complete at {current_altitude:.1f}m. "
                f"RL agent has control."
            )
            return None

        # Takeoff required - drone is below mission altitude
        if not self._takeoff_in_progress:
            self._takeoff_in_progress = True
            self._stats['takeoff_triggers'] += 1
            logger.info(
                f"[ONTOLOGY] Initiating takeoff: {current_altitude:.1f}m → "
                f"{target_altitude:.1f}m"
            )

        # Return takeoff command
        return BehaviorCommand(
            behavior=OntologyBehavior.TAKEOFF,
            priority=AxiomPriority.HIGH,
            reason=f"Ascending to mission altitude ({target_altitude}m)",
            target_position=np.array([
                state.position[0],
                state.position[1],
                target_altitude
            ]),
            preempts_rl=True
        )

    def register_behavior_callback(
        self,
        behavior: OntologyBehavior,
        callback: Callable[[BehaviorCommand], bool]
    ) -> None:
        """
        Register a callback for executing a specific behavior.

        Args:
            behavior: The behavior type
            callback: Function(BehaviorCommand) -> bool (success)
        """
        self._behavior_callbacks[behavior] = callback

    def update(self, state: UAVState, dt: float = 1/60.0) -> Optional[BehaviorCommand]:
        """
        Main update loop. Check all ontology axioms and return behavior command if needed.

        This is called every simulation step. It performs fast rule-based checks
        and periodically verifies with Vampire ATP.

        Args:
            state: Current UAV state
            dt: Time step in seconds

        Returns:
            BehaviorCommand if ontology requires action, None if RL has control
        """
        current_time = time.time()

        # Clear previous violations
        self._active_violations = []

        # === Priority 0: Ontology-driven takeoff (prerequisite behavior) ===
        # UAV must reach mission altitude before RL gets control
        # This implements: PREFLIGHT → ARMED → TAKEOFF → MISSION_EXECUTION
        takeoff_command = self._check_takeoff_required(state)
        if takeoff_command is not None:
            return takeoff_command

        # === Priority 1: Critical safety checks (CRITICAL priority) ===

        # Check for must-land conditions (critical failures)
        if self._check_must_land(state):
            return self._create_emergency_land_command(state)

        # === Priority 2: Battery reserve checks (HIGH priority) ===

        # Compute distance-based battery reserve requirement
        required_reserve = self._compute_required_battery_reserve(state)

        # Check battery axiom violations
        battery_violation = self._check_battery_axioms(state, required_reserve)
        if battery_violation:
            return self._create_rtl_command(state, battery_violation)

        # === Priority 3: Airspace containment (HIGH/CRITICAL priority) ===

        # Check NFZ FIRST - being inside NFZ is CRITICAL priority
        # If we're both outside geofence AND inside NFZ, we need to exit NFZ first
        # (geofence recovery might fly us deeper into the NFZ)
        nfz_command = self._check_nfz_proximity(state)
        if nfz_command and nfz_command.priority == AxiomPriority.CRITICAL:
            # Inside NFZ - must exit immediately, even if outside geofence
            return nfz_command

        # Check geofence violation (HIGH priority, but lower than NFZ CRITICAL)
        if self._check_geofence_violation(state):
            return self._create_geofence_recovery_command(state)

        # Check NFZ proximity warnings (HIGH/MEDIUM priority)
        if nfz_command:
            return nfz_command

        # === Priority 4: Person proximity check (HIGH priority) ===
        # From ontology: minPersonDistance axiom requires 15m clearance
        person_command = self._check_person_proximity(state)
        if person_command:
            return person_command

        # === Priority 5: Periodic Vampire verification ===

        if (self.config.use_vampire and
            self._vampire_available and
            current_time - self._last_vampire_check > self.config.vampire_verification_interval):
            self._verify_with_vampire(state)
            self._last_vampire_check = current_time

        # === No violations - RL has control ===

        # If we had an active behavior that completed, clear it
        if self._behavior_complete:
            self._active_behavior = None
            self._behavior_complete = False

        return None

    def _compute_required_battery_reserve(self, state: UAVState) -> float:
        """
        Compute required battery reserve based on distance to home.

        This implements the ontology's distance-based battery reserve calculation
        from uav_domain.kif Section 13:

            required = (distance_km * battery_per_km) + headroom

        Where:
        - battery_per_km: 2.5% per km (F-11 spec with payload)
        - headroom: 10.0% base safety margin

        Examples from ontology:
        - At 2km from home: (2 * 2.5) + 10 = 15% required
        - At 5km from home: (5 * 2.5) + 10 = 22.5% required
        - At 8km from home: (8 * 2.5) + 10 = 30% required

        For comms-denied operations, applies additional 20% buffer.

        Note: Uses 2D horizontal distance plus altitude cost. A quadcopter doesn't
        fly straight 3D paths - it climbs/descends and flies horizontally. Using
        3D distance would underestimate battery for high-altitude positions close
        to home horizontally.

        Args:
            state: Current UAV state

        Returns:
            Required battery reserve percentage
        """
        # Calculate 2D horizontal distance (xy plane) - quadcopters fly
        # horizontal routes, not straight 3D diagonals
        horizontal_dist_m = np.linalg.norm(state.position[:2] - self._home_position[:2])
        horizontal_dist_km = horizontal_dist_m / 1000.0

        # Add altitude cost: descending to home altitude requires battery
        # Assume 0.5% per 100m altitude difference (descent is cheaper than climb)
        altitude_diff_m = abs(state.position[2] - self._home_position[2])
        altitude_cost_pct = altitude_diff_m / 100.0 * 0.5

        # Apply ontology formula: (distance_km * battery_per_km) + headroom + altitude_cost
        required = (horizontal_dist_km * self.config.battery_per_km) + \
                   self.config.battery_headroom + altitude_cost_pct

        # Apply domain randomization battery drain multiplier ADDITIVELY
        # (not multiplicatively to avoid compounding with comms-denied)
        if hasattr(state, 'battery_drain_multiplier') and state.battery_drain_multiplier != 1.0:
            drain_factor = state.battery_drain_multiplier - 1.0  # e.g., 1.1 -> 0.1
            required += required * drain_factor

        # Apply comms-denied buffer ADDITIVELY (not multiplicatively)
        if state.comms_status == CommsStatus.DENIED:
            required += self.config.battery_headroom * 0.2  # Add 20% of headroom

        return max(self.config.min_reserve, required)

    def _check_battery_axioms(
        self,
        state: UAVState,
        required_reserve: float
    ) -> Optional[AxiomViolation]:
        """
        Check battery-related ontology axioms.

        Axioms checked:
        - batteryReserveReturn: battery < required_reserve → RTL
        - commsDeniedBatteryReserve: comms_denied + battery < 30% → RTL

        Args:
            state: Current UAV state
            required_reserve: Computed required battery reserve

        Returns:
            AxiomViolation if battery constraint violated, None otherwise
        """
        # Skip if already in RTL or emergency
        if state.flight_phase in (FlightPhase.RTL, FlightPhase.LANDING,
                                   FlightPhase.DISARMED, FlightPhase.PREFLIGHT):
            return None

        # Dynamic battery reserve check (primary axiom)
        if state.battery_pct < required_reserve:
            return AxiomViolation(
                axiom_name="batteryReserveReturn",
                priority=AxiomPriority.HIGH,
                triggered_behavior=OntologyBehavior.RTL,
                description=(
                    f"Battery {state.battery_pct:.1f}% below required reserve "
                    f"{required_reserve:.1f}% for distance "
                    f"{np.linalg.norm(state.position - self._home_position):.0f}m"
                ),
                timestamp=time.time()
            )

        # Comms-denied specific check (ontology axiom)
        # More conservative threshold when operating autonomously
        if (state.comms_status == CommsStatus.DENIED and
            state.battery_pct < self.config.battery_rtl_comms_denied_threshold):
            return AxiomViolation(
                axiom_name="commsDeniedBatteryReserve",
                priority=AxiomPriority.HIGH,
                triggered_behavior=OntologyBehavior.RTL,
                description=(
                    f"Comms-denied with battery {state.battery_pct:.1f}% below "
                    f"{self.config.battery_rtl_comms_denied_threshold}% threshold"
                ),
                timestamp=time.time()
            )

        return None

    def _check_must_land(self, state: UAVState) -> bool:
        """
        Check if must-land condition is triggered.

        Ontology mustLand triggers:
        - Battery critically low (emergency threshold)
        - Lost localization (no valid position)
        - Critical sensor failure

        Args:
            state: Current UAV state

        Returns:
            True if must land immediately
        """
        # Skip if already landing or disarmed
        if state.flight_phase in (FlightPhase.LANDING, FlightPhase.DISARMED):
            return False

        # Critical battery - must land now
        if state.battery_pct < self.config.battery_emergency_threshold:
            self._active_violations.append(AxiomViolation(
                axiom_name="criticalBattery",
                priority=AxiomPriority.CRITICAL,
                triggered_behavior=OntologyBehavior.EMERGENCY_LAND,
                description=f"Critical battery {state.battery_pct:.1f}%",
                timestamp=time.time()
            ))
            return True

        # Lost localization - must land
        # VIO invalid AND (GNSS denied OR degraded) = no valid position
        # GNSS degraded with no VIO means position accuracy is unacceptable
        # for autonomous navigation near obstacles
        gnss_unreliable = state.gnss_status in (GNSSStatus.DENIED, GNSSStatus.DEGRADED)
        if not state.vio_valid and gnss_unreliable:
            self._active_violations.append(AxiomViolation(
                axiom_name="localizationLost",
                priority=AxiomPriority.CRITICAL,
                triggered_behavior=OntologyBehavior.EMERGENCY_LAND,
                description=f"Lost localization: VIO invalid, GNSS {state.gnss_status.name}",
                timestamp=time.time()
            ))
            return True

        # IMU failure - cannot maintain attitude
        if not state.imu_valid:
            self._active_violations.append(AxiomViolation(
                axiom_name="imuFailure",
                priority=AxiomPriority.CRITICAL,
                triggered_behavior=OntologyBehavior.EMERGENCY_LAND,
                description="IMU failure - attitude estimation compromised",
                timestamp=time.time()
            ))
            return True

        return False

    def _check_geofence_violation(self, state: UAVState) -> bool:
        """
        Check if UAV has violated geofence boundary.

        Args:
            state: Current UAV state

        Returns:
            True if outside geofence
        """
        if not state.in_geofence:
            self._active_violations.append(AxiomViolation(
                axiom_name="geofenceViolation",
                priority=AxiomPriority.HIGH,
                triggered_behavior=OntologyBehavior.GEOFENCE_RECOVERY,
                description=f"UAV outside geofence at position {state.position}",
                timestamp=time.time()
            ))
            return True
        return False

    def _check_nfz_proximity(self, state: UAVState) -> Optional[BehaviorCommand]:
        """
        Check NFZ proximity and return avoidance command if needed.

        Args:
            state: Current UAV state

        Returns:
            BehaviorCommand for NFZ avoidance, or None
        """
        if state.in_nfz:
            # Critical: already inside NFZ
            self._active_violations.append(AxiomViolation(
                axiom_name="noFlyZoneViolation",
                priority=AxiomPriority.CRITICAL,
                triggered_behavior=OntologyBehavior.NFZ_AVOIDANCE,
                description=f"UAV inside NFZ {state.current_nfz_id}",
                timestamp=time.time()
            ))
            self._stats['nfz_avoidances'] += 1

            return BehaviorCommand(
                behavior=OntologyBehavior.NFZ_AVOIDANCE,
                priority=AxiomPriority.CRITICAL,
                reason=f"Inside NFZ {state.current_nfz_id} - immediate exit required",
                preempts_rl=True
            )

        elif state.nfz_distance < self.config.nfz_critical_distance:
            # High priority: very close to NFZ
            self._active_violations.append(AxiomViolation(
                axiom_name="nfzProximityWarning",
                priority=AxiomPriority.HIGH,
                triggered_behavior=OntologyBehavior.NFZ_AVOIDANCE,
                description=f"UAV {state.nfz_distance:.1f}m from NFZ (critical)",
                timestamp=time.time()
            ))

            return BehaviorCommand(
                behavior=OntologyBehavior.HOVER,
                priority=AxiomPriority.HIGH,
                reason=f"NFZ proximity critical: {state.nfz_distance:.1f}m",
                preempts_rl=True
            )

        elif state.nfz_distance < self.config.nfz_warning_distance:
            # Medium priority: approaching NFZ (allow RL but constrain)
            self._active_violations.append(AxiomViolation(
                axiom_name="nfzProximityWarning",
                priority=AxiomPriority.MEDIUM,
                triggered_behavior=OntologyBehavior.NONE,
                description=f"UAV {state.nfz_distance:.1f}m from NFZ (warning)",
                timestamp=time.time()
            ))
            # Don't preempt RL, but safety filter will constrain actions
            return None

        return None

    def _check_person_proximity(self, state: UAVState) -> Optional[BehaviorCommand]:
        """
        Check person proximity and return avoidance command if needed.

        Implements ontology axiom: minPersonDistance (15m clearance required)
        From ONTOLOGY_FOUNDATION.md Section 14: Safety Constraints

        This check uses person detections from the perception pipeline.
        Call update_person_detections() before update() to provide fresh data.

        Args:
            state: Current UAV state

        Returns:
            BehaviorCommand for person avoidance, or None
        """
        if not self.config.person_detection_enabled:
            return None

        if not self._person_detections:
            # No person detections - safe to proceed
            if self._person_avoidance_active:
                self._person_avoidance_active = False
                logger.info("[ONTOLOGY] Person avoidance cleared - no persons detected")
            return None

        # Find closest person
        uav_pos = state.position
        closest_person = None
        closest_distance = float('inf')

        for person in self._person_detections:
            # Person detection has 'position_3d' or 'distance' field
            if 'position_3d' in person and person['position_3d'] is not None:
                person_pos = np.array(person['position_3d'])
                distance = float(np.linalg.norm(person_pos - uav_pos))
            elif 'distance' in person:
                distance = person['distance']
            else:
                continue

            if distance < closest_distance:
                closest_distance = distance
                closest_person = person

        if closest_person is None:
            return None

        # Check against critical threshold (hard stop)
        if closest_distance < self.config.person_critical_distance:
            self._active_violations.append(AxiomViolation(
                axiom_name="minPersonDistance",
                priority=AxiomPriority.CRITICAL,
                triggered_behavior=OntologyBehavior.PERSON_AVOIDANCE,
                description=f"Person {closest_distance:.1f}m away (< {self.config.person_critical_distance}m critical)",
                timestamp=time.time()
            ))

            if not self._person_avoidance_active:
                self._person_avoidance_active = True
                self._stats['person_avoidances'] += 1
                logger.warning(
                    f"[ONTOLOGY] CRITICAL: Person detected at {closest_distance:.1f}m - "
                    f"hovering immediately"
                )

            return BehaviorCommand(
                behavior=OntologyBehavior.HOVER,
                priority=AxiomPriority.CRITICAL,
                reason=f"Person within critical distance ({closest_distance:.1f}m)",
                preempts_rl=True
            )

        # Check against warning threshold (prepare to stop)
        elif closest_distance < self.config.person_warning_distance:
            self._active_violations.append(AxiomViolation(
                axiom_name="minPersonDistance",
                priority=AxiomPriority.HIGH,
                triggered_behavior=OntologyBehavior.PERSON_AVOIDANCE,
                description=f"Person {closest_distance:.1f}m away (< {self.config.person_warning_distance}m warning)",
                timestamp=time.time()
            ))

            if not self._person_avoidance_active:
                self._person_avoidance_active = True
                self._stats['person_avoidances'] += 1
                logger.warning(
                    f"[ONTOLOGY] WARNING: Person detected at {closest_distance:.1f}m - "
                    f"reducing speed"
                )

            # High priority but allows slow movement
            return BehaviorCommand(
                behavior=OntologyBehavior.PERSON_AVOIDANCE,
                priority=AxiomPriority.HIGH,
                reason=f"Person within warning distance ({closest_distance:.1f}m)",
                preempts_rl=True
            )

        # Person detected but at safe distance
        if self._person_avoidance_active:
            self._person_avoidance_active = False
            logger.info(
                f"[ONTOLOGY] Person avoidance cleared - "
                f"person now at {closest_distance:.1f}m"
            )

        return None

    def update_person_detections(self, detections: List[Dict[str, Any]]) -> None:
        """
        Update person detections from perception pipeline.

        Call this each step before calling update() to provide fresh
        person detection data.

        Args:
            detections: List of detection dicts with keys:
                - class_name or class_id: Should be 'person' or 1
                - position_3d: [x, y, z] world position (optional)
                - distance: Distance from UAV in meters (optional)
                - confidence: Detection confidence (optional)
        """
        # Filter to only person detections
        self._person_detections = [
            d for d in detections
            if d.get('class_name', '').lower() == 'person' or d.get('class_id') == 1
        ]

    def _create_rtl_command(
        self,
        state: UAVState,
        violation: AxiomViolation
    ) -> BehaviorCommand:
        """Create RTL behavior command."""
        self._rtl_triggered = True
        self._rtl_reason = violation.description
        self._active_violations.append(violation)
        self._stats['rtl_triggers'] += 1

        return BehaviorCommand(
            behavior=OntologyBehavior.RTL,
            priority=violation.priority,
            reason=violation.description,
            target_position=self._home_position.copy(),
            preempts_rl=True
        )

    def _create_emergency_land_command(self, state: UAVState) -> BehaviorCommand:
        """Create emergency landing behavior command."""
        self._emergency_triggered = True
        self._emergency_reason = self._active_violations[-1].description if self._active_violations else "Unknown"
        self._stats['emergency_triggers'] += 1

        return BehaviorCommand(
            behavior=OntologyBehavior.EMERGENCY_LAND,
            priority=AxiomPriority.CRITICAL,
            reason=self._emergency_reason,
            target_position=state.position.copy(),  # Land at current position
            preempts_rl=True
        )

    def _create_geofence_recovery_command(self, state: UAVState) -> BehaviorCommand:
        """Create geofence recovery behavior command."""
        self._stats['geofence_recoveries'] += 1

        # Calculate direction back to geofence center
        direction_to_home = self._home_position - state.position
        if np.linalg.norm(direction_to_home) > 0:
            direction_to_home = direction_to_home / np.linalg.norm(direction_to_home)

        # Target is inside geofence, towards home
        recovery_target = state.position + direction_to_home * self.config.geofence_recovery_buffer * 2

        return BehaviorCommand(
            behavior=OntologyBehavior.GEOFENCE_RECOVERY,
            priority=AxiomPriority.HIGH,
            reason="Geofence violation - returning to safe area",
            target_position=recovery_target,
            preempts_rl=True
        )

    def _verify_with_vampire(self, state: UAVState) -> bool:
        """
        Verify current state with Vampire theorem prover.

        This is called periodically to validate rule-based decisions
        against the formal ontology.

        Args:
            state: Current UAV state

        Returns:
            True if Vampire verification succeeded
        """
        if not self._vampire_available:
            return False

        self._stats['vampire_verifications'] += 1

        # Generate TPTP query for current state
        query = self._generate_vampire_query(state)

        try:
            # Write query to temp file
            with tempfile.NamedTemporaryFile(
                mode='w', suffix='.tptp', delete=False
            ) as f:
                f.write(query)
                query_file = f.name

            try:
                result = subprocess.run(
                    [
                        'vampire',
                        '--input_syntax', 'tptp',
                        '--time_limit', str(max(1, self.config.vampire_timeout_ms // 1000)),
                        query_file
                    ],
                    capture_output=True,
                    text=True,
                    timeout=self.config.vampire_timeout_ms / 1000 + 1
                )

                output = result.stdout + result.stderr

                # Check if Vampire confirms our rule-based analysis
                if 'Theorem' in output or 'proved' in output.lower():
                    self._stats['vampire_confirmations'] += 1
                    return True

            finally:
                os.unlink(query_file)

        except subprocess.TimeoutExpired:
            logger.warning(
                "Vampire ATP timed out after %dms. Rule-based checks still valid.",
                self.config.vampire_timeout_ms
            )
            self._stats['vampire_timeouts'] = self._stats.get('vampire_timeouts', 0) + 1
        except FileNotFoundError:
            logger.error(
                "Vampire binary not found. Disabling Vampire verification."
            )
            self._vampire_available = False
        except Exception as e:
            logger.error(
                "Vampire verification failed: %s. Rule-based checks still valid.",
                str(e),
                exc_info=True
            )
            self._stats['vampire_errors'] = self._stats.get('vampire_errors', 0) + 1

        return False

    def _generate_vampire_query(self, state: UAVState) -> str:
        """
        Generate TPTP query to verify current state against ontology.

        Args:
            state: Current UAV state

        Returns:
            TPTP query string
        """
        # Distance to home for battery calculation
        distance_to_home = np.linalg.norm(state.position - self._home_position)
        distance_km = distance_to_home / 1000.0

        # Compute required battery using ontology formula
        required_battery = (distance_km * 2.5) + 10.0  # batteryPerKm * distance + headroom

        query = f"""
% Flyby F-11 UAV State Verification
% Generated by OntologyBehaviorController

% UAV type hierarchy
fof(uav_is_aircraft, axiom, ![X]: (uav(X) => aircraft(X))).
fof(flyby_f11_is_uav, axiom, ![X]: (flyby_f11(X) => uav(X))).
fof(test_uav_is_flyby, axiom, flyby_f11(uav1)).

% Current state facts
fof(current_battery, axiom, current_battery_level(uav1, {state.battery_pct:.1f})).
fof(current_position, axiom, position(uav1, {state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f})).
fof(distance_to_home, axiom, distance_to_home(uav1, {distance_km:.3f})).
fof(required_battery, axiom, required_battery_for_return(uav1, {required_battery:.1f})).

% Comms status
fof(comms_status, axiom, comms_status(uav1, {state.comms_status.name.lower()})).

% Battery reserve axiom - must return if below computed reserve
fof(battery_reserve_axiom, axiom,
    ![UAV, Current, Required]:
      ((uav(UAV) &
        current_battery_level(UAV, Current) &
        required_battery_for_return(UAV, Required) &
        less_than(Current, Required))
       => must_return_to_launch(UAV))).

% Comms-denied battery axiom - lower threshold when autonomous
fof(comms_denied_battery_axiom, axiom,
    ![UAV, Battery]:
      ((uav(UAV) &
        comms_status(UAV, denied) &
        current_battery_level(UAV, Battery) &
        less_than(Battery, 30.0))
       => must_return_to_launch(UAV))).

% NFZ violation axiom
fof(in_nfz_fact, axiom, {"in_nfz(uav1)" if state.in_nfz else "~in_nfz(uav1)"}).
fof(nfz_violation_axiom, axiom,
    ![UAV]: (in_nfz(UAV) => nfz_violation(UAV))).

% Geofence axiom
fof(in_geofence_fact, axiom, {"in_geofence(uav1)" if state.in_geofence else "~in_geofence(uav1)"}).
fof(geofence_axiom, axiom,
    ![UAV]: (~in_geofence(UAV) => geofence_violation(UAV))).

% Arithmetic comparison (simplified)
fof(battery_comparison, axiom,
    {"less_than(" + str(state.battery_pct) + ", " + str(required_battery) + ")"
     if state.battery_pct < required_battery
     else "~less_than(" + str(state.battery_pct) + ", " + str(required_battery) + ")"}).

% Query: Check if any violation exists
fof(safety_check, conjecture,
    ~(must_return_to_launch(uav1) | nfz_violation(uav1) | geofence_violation(uav1))
).
"""
        return query

    # === Public accessors ===

    @property
    def active_behavior(self) -> Optional[BehaviorCommand]:
        """Get the currently active behavior command."""
        return self._active_behavior

    @property
    def is_preempting_rl(self) -> bool:
        """Check if ontology controller is currently preempting RL."""
        return self._active_behavior is not None and self._active_behavior.preempts_rl

    @property
    def active_violations(self) -> List[AxiomViolation]:
        """Get list of current axiom violations."""
        return self._active_violations.copy()

    @property
    def rtl_triggered(self) -> bool:
        """Check if RTL has been triggered."""
        return self._rtl_triggered

    @property
    def emergency_triggered(self) -> bool:
        """Check if emergency landing has been triggered."""
        return self._emergency_triggered

    def get_stats(self) -> Dict[str, Any]:
        """Get controller statistics."""
        return self._stats.copy()

    @property
    def takeoff_complete(self) -> bool:
        """Check if takeoff has completed."""
        return self._takeoff_complete

    def reset(self) -> None:
        """Reset controller state for new episode."""
        self._active_behavior = None
        self._behavior_complete = False
        self._active_violations = []
        self._takeoff_complete = False
        self._takeoff_in_progress = False
        self._rtl_triggered = False
        self._rtl_reason = ""
        self._emergency_triggered = False
        self._emergency_reason = ""
        self._behavior_start_time = 0.0


class OntologyBehaviorExecutor:
    """
    Executes behavior commands issued by OntologyBehaviorController.

    This class translates high-level behavior commands (RTL, EMERGENCY_LAND, etc.)
    into MAVLink commands or direct control inputs.

    Coordinate Frame Notes:
    - Isaac Sim uses Z-up (ENU-like): +X=East, +Y=North, +Z=Up
    - Output actions are [vx, vy, vz, yaw_rate] normalized to [-1, 1]
    - vz: negative = descend, positive = ascend (Z-up convention)
    - The action_bridge handles conversion to PX4 NED if needed
    """

    # Executor configuration constants
    TAKEOFF_ASCENT_RATE_NORMALIZED: float = 0.4  # normalized ascent rate for takeoff
    RTL_LANDING_DISTANCE: float = 3.0  # meters from home to begin landing
    RTL_MAX_SPEED: float = 4.0  # m/s max cruise speed
    RTL_APPROACH_GAIN: float = 0.5  # speed = distance * gain (clamped)
    GEOFENCE_RECOVERY_DISTANCE: float = 5.0  # meters to consider back in safe area
    GEOFENCE_RECOVERY_SPEED: float = 3.0  # m/s recovery speed
    DESCENT_RATE_NORMALIZED: float = 0.4  # normalized descent rate
    MAX_VELOCITY: float = 5.0  # m/s for normalization
    GROUND_ALTITUDE: float = 0.5  # meters to consider on ground

    def __init__(self, mav_connection=None, action_bridge=None, z_up: bool = True):
        """
        Initialize executor.

        Args:
            mav_connection: PyMAVLink connection
            action_bridge: PX4ActionBridge instance
            z_up: If True, use Z-up convention (positive Z = up)
                  If False, use NED convention (positive Z = down)
        """
        self._mav = mav_connection
        self._action_bridge = action_bridge
        self._z_up = z_up
        self._current_behavior: Optional[OntologyBehavior] = None
        self._behavior_complete: bool = False

    def execute(self, command: BehaviorCommand, state: UAVState) -> np.ndarray:
        """
        Execute a behavior command and return action for this step.

        Args:
            command: Behavior command from ontology controller
            state: Current UAV state

        Returns:
            Action array [vx, vy, vz, yaw_rate] normalized to [-1, 1]
        """
        self._current_behavior = command.behavior
        self._behavior_complete = False

        if command.behavior == OntologyBehavior.TAKEOFF:
            return self._execute_takeoff(command, state)
        elif command.behavior == OntologyBehavior.RTL:
            return self._execute_rtl(command, state)
        elif command.behavior == OntologyBehavior.EMERGENCY_LAND:
            return self._execute_emergency_land(command, state)
        elif command.behavior == OntologyBehavior.HOVER:
            return self._execute_hover(state)
        elif command.behavior == OntologyBehavior.GEOFENCE_RECOVERY:
            return self._execute_geofence_recovery(command, state)
        elif command.behavior == OntologyBehavior.NFZ_AVOIDANCE:
            return self._execute_nfz_avoidance(command, state)
        else:
            # Default: hover
            return self._execute_hover(state)

    def _execute_takeoff(self, command: BehaviorCommand, state: UAVState) -> np.ndarray:
        """
        Execute ontology-driven takeoff behavior.

        Ascends vertically at constant rate until target altitude is reached.
        Uses Z-up convention: positive vz = ascend.

        Args:
            command: Takeoff command with target_position containing target altitude
            state: Current UAV state

        Returns:
            Action array [vx, vy, vz, yaw_rate] normalized to [-1, 1]
        """
        if command.target_position is None:
            # No target - hover
            return self._execute_hover(state)

        target_altitude = command.target_position[2]
        current_altitude = state.position[2]
        altitude_error = target_altitude - current_altitude

        if altitude_error <= 0:
            # At or above target - behavior complete
            self._behavior_complete = True
            return self._execute_hover(state)

        # Ascend at constant rate
        # In Z-up (Isaac Sim): positive vz = ascend
        ascent_vz = self.TAKEOFF_ASCENT_RATE_NORMALIZED if self._z_up else -self.TAKEOFF_ASCENT_RATE_NORMALIZED

        # Hold horizontal position (zero vx, vy)
        return np.array([0.0, 0.0, ascent_vz, 0.0], dtype=np.float32)

    def _execute_rtl(self, command: BehaviorCommand, state: UAVState) -> np.ndarray:
        """Execute return-to-launch behavior."""
        if command.target_position is None:
            return self._execute_hover(state)

        # Calculate direction to home (2D horizontal first)
        to_home_2d = command.target_position[:2] - state.position[:2]
        horizontal_distance = np.linalg.norm(to_home_2d)

        if horizontal_distance < self.RTL_LANDING_DISTANCE:
            # Close enough to home horizontally - begin landing descent
            # In Z-up: negative vz = descend
            descent_vz = -self.DESCENT_RATE_NORMALIZED if self._z_up else self.DESCENT_RATE_NORMALIZED
            return np.array([0.0, 0.0, descent_vz, 0.0], dtype=np.float32)

        # Fly towards home horizontally
        direction_2d = to_home_2d / horizontal_distance if horizontal_distance > 0 else np.zeros(2)

        # Cruise at moderate speed, slowing as approaching
        speed = min(self.RTL_MAX_SPEED, horizontal_distance * self.RTL_APPROACH_GAIN)
        velocity_2d = direction_2d * speed

        # Handle altitude: maintain altitude during cruise, or adjust if needed
        # For simplicity, maintain current altitude during horizontal approach
        vz = 0.0

        # Normalize to [-1, 1]
        vx_norm = velocity_2d[0] / self.MAX_VELOCITY
        vy_norm = velocity_2d[1] / self.MAX_VELOCITY

        return np.array([vx_norm, vy_norm, vz, 0.0], dtype=np.float32)

    def _execute_emergency_land(self, command: BehaviorCommand, state: UAVState) -> np.ndarray:
        """Execute emergency landing (descend at current position)."""
        if state.position[2] < self.GROUND_ALTITUDE:
            # On ground - stop descent
            self._behavior_complete = True
            return np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        # Descend at safe rate
        # In Z-up (Isaac Sim): negative vz = descend
        # In NED: positive vz = descend
        descent_vz = -self.DESCENT_RATE_NORMALIZED if self._z_up else self.DESCENT_RATE_NORMALIZED
        return np.array([0.0, 0.0, descent_vz, 0.0], dtype=np.float32)

    def _execute_hover(self, state: UAVState) -> np.ndarray:
        """Execute hover in place."""
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    def _execute_geofence_recovery(self, command: BehaviorCommand, state: UAVState) -> np.ndarray:
        """Execute geofence recovery (return to safe area)."""
        if command.target_position is None:
            return self._execute_hover(state)

        # Calculate 2D horizontal distance to target
        to_target_2d = command.target_position[:2] - state.position[:2]
        horizontal_distance = np.linalg.norm(to_target_2d)

        if horizontal_distance < self.GEOFENCE_RECOVERY_DISTANCE:
            # Back in safe area
            self._behavior_complete = True
            return self._execute_hover(state)

        # Fly horizontally toward safe area
        direction_2d = to_target_2d / horizontal_distance if horizontal_distance > 0 else np.zeros(2)
        velocity_2d = direction_2d * self.GEOFENCE_RECOVERY_SPEED

        # Maintain current altitude during recovery
        vx_norm = velocity_2d[0] / self.MAX_VELOCITY
        vy_norm = velocity_2d[1] / self.MAX_VELOCITY

        return np.array([vx_norm, vy_norm, 0.0, 0.0], dtype=np.float32)

    def _execute_nfz_avoidance(self, command: BehaviorCommand, state: UAVState) -> np.ndarray:
        """Execute NFZ avoidance (back away from NFZ)."""
        # If we have NFZ info, back away from it
        # For now, just hover - safety filter will handle constraint
        return self._execute_hover(state)

    @property
    def is_complete(self) -> bool:
        """Check if current behavior execution is complete."""
        return self._behavior_complete
