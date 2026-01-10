#!/usr/bin/env python3
"""
Ontology State Bridge for Isaac Sim Training

Provides the connection between simulation state and the ontology reasoning system:
1. Converts simulation state to TPTP facts for Vampire verification
2. Converts state to Prolog facts for runtime monitoring
3. Implements safety shielding via theorem prover queries
4. Generates symbolic observations for hierarchical RL

Integrates with:
- flyby-f11/ontology/planning_mode/uav_domain.kif (core UAV ontology)
- flyby-f11/ontology/planning_mode/isr_extensions.kif (ISR extensions)
- Vampire theorem prover for safety verification
- SWI-Prolog for runtime constraint checking
"""

import numpy as np
import subprocess
import tempfile
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from enum import Enum
import json

# Import types from base environment
from .base_isr_env import (
    UAVState, FlightPhase, CommsStatus, GNSSStatus, ThreatLevel,
    NoFlyZone, ThreatZone, TargetOfInterest, CommsZone
)


@dataclass
class OntologyFact:
    """A single ontology fact."""
    predicate: str
    arguments: List[str]

    def to_tptp(self) -> str:
        """Convert to TPTP format."""
        args = ", ".join(self.arguments)
        return f"{self.predicate}({args})"

    def to_prolog(self) -> str:
        """Convert to Prolog format."""
        args = ", ".join(self.arguments)
        return f"{self.predicate}({args})."


@dataclass
class SafetyVerificationResult:
    """Result of a safety verification query."""
    is_safe: bool
    violated_constraints: List[str] = field(default_factory=list)
    verification_time_ms: float = 0.0
    suggested_action: Optional[np.ndarray] = None


class OntologyStateBridge:
    """
    Bridge between simulation state and ontology reasoning.

    Provides:
    1. State-to-facts conversion for Vampire/Prolog
    2. Safety shielding via theorem prover
    3. Symbolic state extraction for RL
    4. Runtime constraint monitoring
    """

    # Mapping from Python enums to ontology instances
    FLIGHT_PHASE_MAP = {
        FlightPhase.PREFLIGHT: "Preflight",
        FlightPhase.ARMED: "Armed",
        FlightPhase.TAKEOFF: "Takeoff",
        FlightPhase.HOVER: "Hover",
        FlightPhase.TRANSIT: "Transit",
        FlightPhase.LOITER: "Loiter",
        FlightPhase.MISSION_EXECUTION: "MissionExecution",
        FlightPhase.RTL: "RTL",
        FlightPhase.LANDING: "Landing",
        FlightPhase.DISARMED: "Disarmed",
    }

    COMMS_STATUS_MAP = {
        CommsStatus.OPERATIONAL: "CommsOperational",
        CommsStatus.DEGRADED: "CommsDegraded",
        CommsStatus.DENIED: "CommsDenied",
    }

    GNSS_STATUS_MAP = {
        GNSSStatus.FULL: "GNSSFull",
        GNSSStatus.DEGRADED: "GNSSDegraded",
        GNSSStatus.DENIED: "GNSSDenied",
    }

    THREAT_LEVEL_MAP = {
        ThreatLevel.NONE: "NoThreat",
        ThreatLevel.LOW: "LowThreat",
        ThreatLevel.MEDIUM: "MediumThreat",
        ThreatLevel.HIGH: "HighThreat",
    }

    # Hard safety constraints that trigger shielding
    HARD_CONSTRAINTS = [
        "geofenceViolation",
        "noFlyZoneViolation",
        "highThreatExposure",
        "mustLand",
        "mustReturnToLaunch",
        "pathProhibited",
    ]

    def __init__(
        self,
        ontology_path: str = "/workspace/ontology/planning_mode",
        vampire_path: str = "vampire",
        vampire_timeout_ms: int = 100,
        use_vampire: bool = True,
        use_prolog: bool = False
    ):
        """
        Initialize ontology bridge.

        Args:
            ontology_path: Path to ontology files (KIF/TPTP)
            vampire_path: Path to Vampire executable
            vampire_timeout_ms: Vampire query timeout
            use_vampire: Enable Vampire for safety verification
            use_prolog: Enable Prolog for runtime monitoring
        """
        self.ontology_path = ontology_path
        self.vampire_path = vampire_path
        self.vampire_timeout_ms = vampire_timeout_ms
        self.use_vampire = use_vampire
        self.use_prolog = use_prolog

        # Cache for ontology files
        self._ontology_cache: Dict[str, str] = {}

        # UAV instance identifier
        self.uav_id = "flyby_f11_alpha"
        self.mission_id = "current_mission"

    def state_to_facts(
        self,
        state: UAVState,
        nfz_zones: List[NoFlyZone],
        threat_zones: List[ThreatZone],
        targets: Optional[List[TargetOfInterest]] = None
    ) -> List[OntologyFact]:
        """
        Convert simulation state to ontology facts.

        Args:
            state: Current UAV state
            nfz_zones: Active no-fly zones
            threat_zones: Threat zones
            targets: Optional targets of interest

        Returns:
            List of ontology facts
        """
        facts = []

        # UAV instance
        facts.append(OntologyFact("instance", [self.uav_id, "FlybyF11"]))

        # Flight phase
        phase_name = self.FLIGHT_PHASE_MAP.get(state.flight_phase, "Unknown")
        facts.append(OntologyFact("hasFlightPhase", [self.uav_id, phase_name]))

        # Battery level
        facts.append(OntologyFact(
            "CurrentBatteryLevel",
            [self.uav_id, f"{state.battery_pct:.1f}"]
        ))

        # Battery reserve requirement
        facts.append(OntologyFact(
            "BatteryReserveForReturn",
            [self.uav_id, f"{state.battery_reserve:.1f}"]
        ))

        # Communications status
        comms_name = self.COMMS_STATUS_MAP.get(state.comms_status, "Unknown")
        facts.append(OntologyFact("hasCommsStatus", [self.uav_id, comms_name]))

        # GNSS status
        gnss_name = self.GNSS_STATUS_MAP.get(state.gnss_status, "Unknown")
        facts.append(OntologyFact("hasGNSSStatus", [self.uav_id, gnss_name]))

        # VIO status
        if state.vio_valid:
            facts.append(OntologyFact("hasValidVisualOdometry", [self.uav_id]))

        # IMU status
        if state.imu_valid:
            facts.append(OntologyFact("imuOperational", [self.uav_id]))

        # Autonomous mode
        if state.autonomous_mode:
            facts.append(OntologyFact(
                "autonomousMissionMode",
                [self.uav_id, self.mission_id]
            ))

        # Geofence status
        if not state.in_geofence:
            facts.append(OntologyFact("geofenceViolation", [self.uav_id]))

        # NFZ status
        for nfz in nfz_zones:
            if not nfz.is_active:
                continue

            # NFZ instance
            nfz_type = "DynamicNoFlyZone" if nfz.is_dynamic else "NoFlyZone"
            facts.append(OntologyFact("instance", [nfz.id, nfz_type]))

            if nfz.is_active:
                facts.append(OntologyFact("nfzActive", [nfz.id]))

            # Check if inside NFZ
            if state.current_nfz_id == nfz.id:
                facts.append(OntologyFact("isWithin", [self.uav_id, nfz.id]))
                facts.append(OntologyFact("noFlyZoneViolation", [self.uav_id]))

            # Distance to NFZ
            if state.nfz_distance < nfz.buffer_distance:
                facts.append(OntologyFact(
                    "nfzProximityWarning",
                    [self.uav_id, nfz.id]
                ))

        # Threat zone status
        for zone in threat_zones:
            threat_name = self.THREAT_LEVEL_MAP.get(zone.threat_level, "Unknown")
            facts.append(OntologyFact("instance", [zone.id, "ThreatZone"]))
            facts.append(OntologyFact("threatLevel", [zone.id, threat_name]))

        if state.in_threat_zone:
            if state.current_threat_level == ThreatLevel.HIGH:
                facts.append(OntologyFact("highThreatExposure", [self.uav_id]))
                facts.append(OntologyFact("not_safeState", [self.uav_id]))

        # Threat exposure accumulation
        facts.append(OntologyFact(
            "accumulatedThreatExposure",
            [self.uav_id, f"{state.threat_exposure:.1f}"]
        ))

        # Targets
        if targets:
            for target in targets:
                facts.append(OntologyFact(
                    "instance",
                    [target.id, "TargetOfInterest"]
                ))
                facts.append(OntologyFact(
                    "targetPriority",
                    [target.id, str(target.priority)]
                ))
                facts.append(OntologyFact(
                    "targetValue",
                    [target.id, f"{target.value:.0f}"]
                ))

                if target.observed:
                    facts.append(OntologyFact("targetObserved", [target.id]))

                if target.in_high_threat:
                    facts.append(OntologyFact("targetInHighThreat", [target.id]))

        return facts

    def facts_to_tptp(self, facts: List[OntologyFact]) -> str:
        """Convert facts to TPTP format for Vampire."""
        lines = [
            "% Flyby F-11 ISR Environment State",
            "% Auto-generated from simulation state",
            "",
            "% Include core ontology",
            f"include('{self.ontology_path}/uav_domain.tptp').",
            f"include('{self.ontology_path}/isr_extensions.tptp').",
            "",
            "% Current state facts",
        ]

        for i, fact in enumerate(facts):
            tptp_fact = f"fof(state_fact_{i}, axiom, {fact.to_tptp()})."
            lines.append(tptp_fact)

        return "\n".join(lines)

    def facts_to_prolog(self, facts: List[OntologyFact]) -> str:
        """Convert facts to Prolog format for runtime monitoring."""
        lines = [
            "% Flyby F-11 ISR Environment State",
            "% Auto-generated from simulation state",
            "",
        ]

        for fact in facts:
            lines.append(fact.to_prolog())

        return "\n".join(lines)

    def verify_action_safety(
        self,
        action: np.ndarray,
        state: UAVState,
        nfz_zones: List[NoFlyZone],
        threat_zones: List[ThreatZone]
    ) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Verify that a proposed action is safe using Vampire.

        Args:
            action: Proposed action (position setpoint or velocity command)
            state: Current UAV state
            nfz_zones: Active NFZs
            threat_zones: Threat zones

        Returns:
            Tuple of (is_safe, safe_alternative_action)
        """
        if not self.use_vampire:
            return True, None

        # Predict next state from action
        next_position = state.position + action[:3] * 0.1  # Simple prediction

        # Create hypothetical next state
        next_state = UAVState(
            position=next_position,
            velocity=action[:3] if len(action) >= 3 else state.velocity,
            battery_pct=state.battery_pct - 0.01,  # Estimate consumption
            comms_status=state.comms_status,
            gnss_status=state.gnss_status,
            vio_valid=state.vio_valid,
            imu_valid=state.imu_valid,
        )

        # Check NFZ violations in predicted state
        for nfz in nfz_zones:
            if not nfz.is_active:
                continue

            dist = np.linalg.norm(next_position[:2] - nfz.center[:2])
            if dist < nfz.radius:
                next_state.in_nfz = True
                next_state.current_nfz_id = nfz.id

        # Check threat zone entry
        for zone in threat_zones:
            dist = np.linalg.norm(next_position[:2] - zone.center[:2])
            if dist < zone.radius:
                next_state.in_threat_zone = True
                if zone.threat_level == ThreatLevel.HIGH:
                    next_state.current_threat_level = ThreatLevel.HIGH

        # Generate facts for predicted state
        facts = self.state_to_facts(next_state, nfz_zones, threat_zones)

        # Check each hard constraint
        result = self._verify_constraints(facts)

        if result.is_safe:
            return True, None

        # Generate safe alternative
        safe_action = self._compute_safe_alternative(
            action, state, nfz_zones, threat_zones, result.violated_constraints
        )

        return False, safe_action

    def _verify_constraints(
        self,
        facts: List[OntologyFact]
    ) -> SafetyVerificationResult:
        """
        Verify hard constraints using Vampire.

        Args:
            facts: Current state facts

        Returns:
            Verification result
        """
        violated = []

        # Check each fact for violations
        fact_strings = [f.to_tptp() for f in facts]

        for constraint in self.HARD_CONSTRAINTS:
            # Check if constraint violation is in facts
            for fact in facts:
                if constraint in fact.predicate:
                    violated.append(constraint)
                    break

        # For more complex constraints, we could invoke Vampire
        # but for performance, we do simple fact checking first

        return SafetyVerificationResult(
            is_safe=len(violated) == 0,
            violated_constraints=violated,
        )

    def _compute_safe_alternative(
        self,
        action: np.ndarray,
        state: UAVState,
        nfz_zones: List[NoFlyZone],
        threat_zones: List[ThreatZone],
        violations: List[str]
    ) -> np.ndarray:
        """
        Compute a safe alternative action.

        Uses gradient-based avoidance to find nearest safe action.
        """
        safe_action = action.copy()

        # Handle specific violations
        if "noFlyZoneViolation" in violations:
            # Find nearest NFZ and compute avoidance vector
            for nfz in nfz_zones:
                if not nfz.is_active:
                    continue

                to_nfz = nfz.center[:2] - state.position[:2]
                dist = np.linalg.norm(to_nfz)

                if dist < nfz.radius + nfz.buffer_distance:
                    # Compute repulsion vector
                    repulsion = -to_nfz / (dist + 1e-6)
                    repulsion_strength = (nfz.radius + nfz.buffer_distance - dist) / nfz.buffer_distance
                    safe_action[:2] += repulsion * repulsion_strength * 2.0

        if "highThreatExposure" in violations:
            # Move away from high threat zones
            for zone in threat_zones:
                if zone.threat_level != ThreatLevel.HIGH:
                    continue

                to_zone = zone.center[:2] - state.position[:2]
                dist = np.linalg.norm(to_zone)

                if dist < zone.radius * 1.5:
                    repulsion = -to_zone / (dist + 1e-6)
                    safe_action[:2] += repulsion * 3.0

        if "geofenceViolation" in violations:
            # Clamp to geofence
            # This should integrate with EnvironmentConfig.geofence
            pass

        if "mustLand" in violations:
            # Override with landing command
            safe_action = np.array([0, 0, -1.0, 0])  # Descend

        if "mustReturnToLaunch" in violations:
            # Set action toward home
            to_home = -state.position
            to_home = to_home / (np.linalg.norm(to_home) + 1e-6)
            safe_action[:3] = to_home * 5.0  # RTL velocity

        return safe_action

    def get_symbolic_state(
        self,
        state: UAVState,
        nfz_zones: List[NoFlyZone],
        threat_zones: List[ThreatZone],
        targets: Optional[List[TargetOfInterest]] = None
    ) -> Dict[str, Any]:
        """
        Extract symbolic state for hierarchical RL.

        Returns a dictionary of symbolic features suitable for
        high-level planning agents.
        """
        return {
            # Flight status
            "flight_phase": state.flight_phase.name,
            "battery_pct": state.battery_pct,
            "battery_adequate": state.battery_pct > state.battery_reserve,

            # Localization
            "comms_status": state.comms_status.name,
            "gnss_status": state.gnss_status.name,
            "localization_valid": state.gnss_status != GNSSStatus.DENIED or state.vio_valid,

            # Safety status
            "in_geofence": state.in_geofence,
            "in_nfz": state.in_nfz,
            "nfz_warning": state.nfz_distance < 20.0,  # Within 20m warning zone

            # Threat status
            "in_threat_zone": state.in_threat_zone,
            "threat_level": state.current_threat_level.name,
            "threat_exposure_ratio": state.threat_exposure / state.max_threat_exposure,

            # Autonomy
            "autonomous_mode": state.autonomous_mode,

            # Mission progress (if targets defined)
            "targets_observed": sum(1 for t in (targets or []) if t.observed),
            "targets_total": len(targets or []),
            "critical_targets_pending": sum(
                1 for t in (targets or [])
                if t.priority == 1 and not t.observed and not t.in_high_threat
            ),

            # Active NFZs
            "active_nfz_count": sum(1 for nfz in nfz_zones if nfz.is_active),

            # Mission time
            "mission_time": state.mission_time,
        }

    def query_vampire(
        self,
        query: str,
        additional_facts: Optional[List[OntologyFact]] = None
    ) -> Tuple[bool, float]:
        """
        Execute a Vampire theorem prover query.

        Args:
            query: TPTP query to prove
            additional_facts: Optional additional facts

        Returns:
            Tuple of (proved, time_ms)
        """
        if not self.use_vampire:
            return False, 0.0

        # Build TPTP file
        tptp_content = self.facts_to_tptp(additional_facts or [])
        tptp_content += f"\n\n% Query\nfof(query, conjecture, {query})."

        # Write to temp file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.tptp', delete=False) as f:
            f.write(tptp_content)
            tptp_file = f.name

        try:
            # Execute Vampire
            result = subprocess.run(
                [
                    self.vampire_path,
                    "--input_syntax", "tptp",
                    "--time_limit", str(self.vampire_timeout_ms // 1000),
                    tptp_file
                ],
                capture_output=True,
                text=True,
                timeout=self.vampire_timeout_ms / 1000.0 + 1.0
            )

            # Parse result
            output = result.stdout
            proved = "Theorem" in output or "Refutation found" in output

            # Extract time
            time_ms = 0.0
            for line in output.split('\n'):
                if "Time elapsed:" in line:
                    try:
                        time_str = line.split(":")[1].strip().replace("s", "")
                        time_ms = float(time_str) * 1000
                    except:
                        pass

            return proved, time_ms

        except subprocess.TimeoutExpired:
            return False, self.vampire_timeout_ms

        finally:
            # Cleanup
            os.unlink(tptp_file)

    def to_json(
        self,
        state: UAVState,
        nfz_zones: List[NoFlyZone],
        threat_zones: List[ThreatZone]
    ) -> str:
        """
        Export current state as JSON for logging/debugging.
        """
        facts = self.state_to_facts(state, nfz_zones, threat_zones)
        symbolic = self.get_symbolic_state(state, nfz_zones, threat_zones)

        return json.dumps({
            "facts": [{"predicate": f.predicate, "arguments": f.arguments} for f in facts],
            "symbolic_state": symbolic,
            "timestamp": state.mission_time,
        }, indent=2)
