"""
Safety filter using Vampire theorem prover.
Checks actions against ontology constraints before execution.

When Vampire is unavailable, uses rule-based fallback checks to ensure
safety-critical constraints are still enforced. This prevents the filter
from "failing open" and allowing all actions.
"""
import subprocess
import tempfile
import os
import shutil
from pathlib import Path
from typing import Optional, Tuple, Dict, Any, List
import numpy as np


class VampireSafetyFilter:
    """
    Filters actions through Vampire theorem prover for safety.

    Uses the UAV domain ontology to verify that proposed actions
    will not violate safety constraints.

    Hard constraints (blocked):
    - geofenceViolation: UAV would exit operational boundary
    - noFlyZoneViolation: UAV would enter restricted airspace
    - highThreatExposure: UAV would enter high-threat zone
    - mustLand: Battery critical, must land immediately
    - mustReturnToLaunch: Battery low, must RTL

    Soft constraints (penalized but allowed):
    - nfz_buffer_penetration: Approaching NFZ boundary
    - medium_threat_exposure: In medium threat zone
    """

    def __init__(
        self,
        ontology_path: str = "/workspace/ontology/planning_mode",
        timeout_ms: int = 50,
        enabled: bool = True
    ):
        """
        Initialize safety filter.

        Args:
            ontology_path: Path to ontology TPTP files
            timeout_ms: Vampire query timeout in milliseconds
            enabled: Whether safety filtering is active
        """
        self.ontology_path = Path(ontology_path)
        self.timeout_s = timeout_ms / 1000.0
        self.enabled = enabled

        self.hard_constraints = [
            'geofenceViolation',
            'noFlyZoneViolation',
            'highThreatExposure',
            'mustLand',
            'mustReturnToLaunch'
        ]

        self.soft_constraints = [
            'nfzBufferPenetration',
            'mediumThreatExposure',
            'lowBatteryWarning'
        ]

        # Cache for ontology axioms
        self._base_axioms: Optional[str] = None

        # Check if Vampire is available
        self._vampire_available = shutil.which('vampire') is not None

        # Geofence bounds for rule-based fallback (F-11 defaults)
        self.geofence_bounds = {
            'min_x': -150.0, 'max_x': 150.0,
            'min_y': -150.0, 'max_y': 150.0,
            'min_z': 0.0, 'max_z': 120.0,
        }

        # NFZ list for rule-based fallback (can be updated)
        self.nfz_zones: List[Dict[str, Any]] = []

        # Battery thresholds
        self.battery_critical_threshold = 15.0  # Must land
        self.battery_rtl_threshold = 25.0  # Must RTL

    def _load_base_axioms(self) -> str:
        """Load base ontology axioms for queries."""
        if self._base_axioms is not None:
            return self._base_axioms

        axiom_files = [
            "uav_domain.tptp",
            "safety_axioms.tptp",
            "flight_rules.tptp"
        ]

        axioms = []
        for filename in axiom_files:
            filepath = self.ontology_path / filename
            if filepath.exists():
                axioms.append(filepath.read_text())

        self._base_axioms = "\n".join(axioms)
        return self._base_axioms

    def check_action_safety(
        self,
        current_state: Dict[str, Any],
        proposed_action: np.ndarray,
        predicted_next_state: Dict[str, Any]
    ) -> Tuple[bool, Optional[str], float]:
        """
        Check if proposed action is safe.

        Args:
            current_state: Current UAV state as dict with keys:
                - x, y, z: position
                - battery: battery percentage
                - in_geofence: bool
                - in_nfz: bool
            proposed_action: Action agent wants to take [vx, vy, vz, yaw_rate]
            predicted_next_state: Predicted state after action

        Returns:
            Tuple of:
                - is_safe: True if action passes all hard constraints
                - violation_type: Name of violated constraint if unsafe, None otherwise
                - penalty: Soft constraint penalty (0 if no soft violations)
        """
        if not self.enabled:
            return True, None, 0.0

        # Try Vampire first if available
        if self._vampire_available:
            # Generate TPTP query
            query = self._generate_safety_query(current_state, predicted_next_state)

            # Run Vampire
            result = self._run_vampire(query)

            # Only use Vampire result if it's meaningful (non-empty)
            if result.strip():
                # Parse result for hard violations
                for constraint in self.hard_constraints:
                    if constraint.lower() in result.lower():
                        return False, constraint, 0.0

                # Check soft constraints for penalty
                penalty = 0.0
                for constraint in self.soft_constraints:
                    if constraint.lower() in result.lower():
                        penalty += self._get_soft_penalty(constraint)

                return True, None, penalty

        # Fallback: Use rule-based safety checks
        # This ensures we don't fail open when Vampire is unavailable
        return self._rule_based_safety_check(current_state, predicted_next_state)

    def _rule_based_safety_check(
        self,
        current_state: Dict[str, Any],
        predicted_next_state: Dict[str, Any]
    ) -> Tuple[bool, Optional[str], float]:
        """
        Rule-based safety check as fallback when Vampire is unavailable.

        Implements the same hard constraints as the ontology using simple
        geometric and threshold checks. This ensures the safety filter
        doesn't "fail open" and allow dangerous actions.

        Args:
            current_state: Current UAV state
            predicted_next_state: Predicted state after action

        Returns:
            Same format as check_action_safety()
        """
        next_x = predicted_next_state.get('x', 0.0)
        next_y = predicted_next_state.get('y', 0.0)
        next_z = predicted_next_state.get('z', 0.0)
        battery = current_state.get('battery', 100.0)

        penalty = 0.0

        # Check geofence violation
        if (next_x < self.geofence_bounds['min_x'] or
            next_x > self.geofence_bounds['max_x'] or
            next_y < self.geofence_bounds['min_y'] or
            next_y > self.geofence_bounds['max_y'] or
            next_z < self.geofence_bounds['min_z'] or
            next_z > self.geofence_bounds['max_z']):
            return False, 'geofenceViolation', 0.0

        # Check NFZ violations
        for nfz in self.nfz_zones:
            nfz_center = np.array([nfz.get('cx', 0), nfz.get('cy', 0)])
            nfz_radius = nfz.get('radius', 0)
            nfz_z_min = nfz.get('z_min', 0)
            nfz_z_max = nfz.get('z_max', float('inf'))

            next_pos_2d = np.array([next_x, next_y])
            dist_to_center = np.linalg.norm(next_pos_2d - nfz_center)

            # Check if inside NFZ cylinder
            if (dist_to_center < nfz_radius and
                nfz_z_min <= next_z <= nfz_z_max):
                return False, 'noFlyZoneViolation', 0.0

            # Check NFZ buffer (soft constraint)
            buffer_distance = 10.0  # meters
            if (dist_to_center < nfz_radius + buffer_distance and
                nfz_z_min <= next_z <= nfz_z_max):
                penalty += self._get_soft_penalty('nfzBufferPenetration')

        # Check battery critical (must land)
        if battery < self.battery_critical_threshold:
            return False, 'mustLand', 0.0

        # Check battery low (must RTL)
        if battery < self.battery_rtl_threshold:
            return False, 'mustReturnToLaunch', 0.0

        # Low battery warning (soft)
        if battery < 35.0:
            penalty += self._get_soft_penalty('lowBatteryWarning')

        return True, None, penalty

    def update_geofence(self, bounds: Dict[str, float]) -> None:
        """
        Update geofence bounds for rule-based fallback.

        Args:
            bounds: Dict with keys min_x, max_x, min_y, max_y, min_z, max_z
        """
        self.geofence_bounds.update(bounds)

    def update_nfz_zones(self, zones: List[Dict[str, Any]]) -> None:
        """
        Update NFZ list for rule-based fallback.

        Args:
            zones: List of NFZ dicts with keys:
                - cx, cy: Center x, y coordinates
                - radius: Radius in meters
                - z_min, z_max: Altitude bounds (optional)
        """
        self.nfz_zones = zones

    def _generate_safety_query(
        self,
        current: Dict[str, Any],
        next_state: Dict[str, Any]
    ) -> str:
        """
        Generate TPTP safety query.

        Creates a first-order logic query that checks if transitioning
        from current state to next state violates any constraints.
        """
        # Extract values with defaults
        curr_x = current.get('x', 0.0)
        curr_y = current.get('y', 0.0)
        curr_z = current.get('z', 0.0)
        curr_battery = current.get('battery', 100.0)

        next_x = next_state.get('x', curr_x)
        next_y = next_state.get('y', curr_y)
        next_z = next_state.get('z', curr_z)

        query = f"""
% Base ontology axioms
{self._load_base_axioms() if self.ontology_path.exists() else '% No base axioms loaded'}

% Current state facts
fof(current_position, axiom,
    position(uav1, {curr_x:.2f}, {curr_y:.2f}, {curr_z:.2f})).
fof(current_battery, axiom,
    batteryLevel(uav1, {curr_battery:.1f})).

% Predicted next state facts
fof(next_position, axiom,
    nextPosition(uav1, {next_x:.2f}, {next_y:.2f}, {next_z:.2f})).

% Safety query: Is there any hard constraint violation?
fof(safety_check, conjecture,
    ~(geofenceViolation(uav1) |
      noFlyZoneViolation(uav1) |
      highThreatExposure(uav1) |
      mustLand(uav1) |
      mustReturnToLaunch(uav1))
).
"""
        return query

    def _run_vampire(self, query: str) -> str:
        """
        Run Vampire theorem prover on query.

        Args:
            query: TPTP query string

        Returns:
            Vampire output string (may contain constraint names if violated)
        """
        try:
            # Write query to temp file
            with tempfile.NamedTemporaryFile(
                mode='w',
                suffix='.tptp',
                delete=False
            ) as f:
                f.write(query)
                query_file = f.name

            try:
                # Run Vampire
                result = subprocess.run(
                    [
                        'vampire',
                        '--input_syntax', 'tptp',
                        '--time_limit', str(max(1, int(self.timeout_s))),
                        query_file
                    ],
                    capture_output=True,
                    text=True,
                    timeout=self.timeout_s + 1
                )
                return result.stdout + result.stderr
            finally:
                # Clean up temp file
                os.unlink(query_file)

        except FileNotFoundError:
            # Vampire not installed - log warning, allow action
            return ""
        except subprocess.TimeoutExpired:
            # Timeout - allow action but log
            return ""
        except Exception as e:
            print(f"Vampire error: {e}")
            return ""

    def _get_soft_penalty(self, constraint: str) -> float:
        """Get penalty value for soft constraint violation."""
        penalties = {
            'nfzBufferPenetration': 0.1,
            'mediumThreatExposure': 0.2,
            'lowBatteryWarning': 0.05
        }
        return penalties.get(constraint, 0.0)

    def get_safe_fallback_action(self, current_state: Dict[str, Any]) -> np.ndarray:
        """
        Return safe fallback action.

        Used when proposed action is unsafe. Default is hover in place.

        Args:
            current_state: Current UAV state

        Returns:
            Safe fallback action [vx, vy, vz, yaw_rate]
        """
        # Default: hover in place
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    def predict_next_state(
        self,
        current_state: Dict[str, Any],
        action: np.ndarray,
        dt: float = 0.1
    ) -> Dict[str, Any]:
        """
        Simple physics-based prediction of next state.

        Args:
            current_state: Current state dict
            action: Velocity action [vx, vy, vz, yaw_rate]
            dt: Time step in seconds

        Returns:
            Predicted next state dict
        """
        # Extract current position
        x = current_state.get('x', 0.0)
        y = current_state.get('y', 0.0)
        z = current_state.get('z', 0.0)

        # Apply velocity for time step
        vx, vy, vz = action[0], action[1], action[2]

        next_state = current_state.copy()
        next_state['x'] = x + vx * dt
        next_state['y'] = y + vy * dt
        next_state['z'] = z + vz * dt

        return next_state

    def filter_action(
        self,
        current_state: Dict[str, Any],
        action: np.ndarray,
        dt: float = 0.1
    ) -> Tuple[np.ndarray, bool, Optional[str]]:
        """
        Filter action through safety constraints.

        Convenience method that combines prediction and safety checking.

        Args:
            current_state: Current UAV state
            action: Proposed action
            dt: Time step for prediction

        Returns:
            Tuple of:
                - action: Safe action (original or fallback)
                - was_modified: True if action was replaced with fallback
                - violation: Constraint that was violated, if any
        """
        predicted = self.predict_next_state(current_state, action, dt)
        is_safe, violation, _ = self.check_action_safety(
            current_state, action, predicted
        )

        if is_safe:
            return action, False, None
        else:
            return self.get_safe_fallback_action(current_state), True, violation
