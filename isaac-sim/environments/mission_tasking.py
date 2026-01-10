"""
Mission Tasking System for ISR Operations.

This module defines how an operator would task a drone for ISR missions,
following real-world military/commercial UAS tasking conventions.

Key Concepts:
- Named Area of Interest (NAI): A geographic region requiring surveillance
- Priority Intelligence Requirement (PIR): What the operator wants to find
- Operating Constraints: Airspace, altitude, time restrictions
- Mission Parameters: Coverage requirements, return conditions

The tasking flows through the system as:
    Operator Tasking → Ontology Parser → Flight Plan → RL Execution

Example real-world tasking:
    "Scout NAI ALPHA and NAI BRAVO for enemy vehicle concentrations.
     Priority to ALPHA. Avoid restricted airspace RESTRICTED-1.
     Achieve 80% coverage. Return with 25% battery reserve."
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Any
from enum import Enum, auto
import numpy as np
import json


class NAIPriority(Enum):
    """Priority level for Named Areas of Interest."""
    CRITICAL = 1      # Must surveil, mission failure if missed
    HIGH = 2          # High priority, surveil if possible
    MEDIUM = 3        # Standard priority
    LOW = 4           # Opportunistic, surveil if convenient


class TargetType(Enum):
    """Types of targets the operator wants to find."""
    VEHICLE = auto()          # Any vehicle
    MILITARY_VEHICLE = auto() # Tanks, APCs, military trucks
    CIVILIAN_VEHICLE = auto() # Cars, trucks, civilian
    PERSONNEL = auto()        # People/troops
    STRUCTURE = auto()        # Buildings, camps
    EQUIPMENT = auto()        # Weapons, supplies


class SearchPattern(Enum):
    """Search patterns for surveilling an NAI."""
    PARALLEL = auto()      # Parallel tracks (lawnmower)
    EXPANDING_SQUARE = auto()  # Spiral outward from center
    SECTOR = auto()        # Pie-slice sectors from center
    CREEPING_LINE = auto() # Parallel with offset
    RANDOM = auto()        # RL-determined (no fixed pattern)


@dataclass
class NamedAreaOfInterest:
    """
    A geographic region requiring surveillance.

    In real operations, NAIs are typically:
    - Intersections, bridges, or chokepoints
    - Suspected enemy positions
    - Areas of reported activity
    - Key terrain features
    """
    id: str                          # Unique identifier (e.g., "NAI_ALPHA")
    name: str                        # Human-readable name
    center: np.ndarray               # (x, y) center position in meters
    radius: float                    # Radius of interest in meters
    priority: NAIPriority = NAIPriority.MEDIUM
    required_coverage: float = 80.0  # Minimum % coverage required
    search_pattern: SearchPattern = SearchPattern.RANDOM  # Let RL decide
    altitude_agl: float = 50.0       # Suggested altitude AGL
    dwell_time: float = 0.0          # Minimum time to spend (0 = no minimum)

    # Intelligence requirements specific to this NAI
    target_types: List[TargetType] = field(default_factory=lambda: [TargetType.VEHICLE, TargetType.PERSONNEL])

    # Last known intel (affects spawn distribution)
    reported_activity: str = ""      # E.g., "Vehicle convoy observed 0600Z"
    confidence: float = 0.5          # 0-1, how confident in the intel
    time_since_report: float = 0.0   # Hours since last report

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "id": self.id,
            "name": self.name,
            "center": self.center.tolist(),
            "radius": self.radius,
            "priority": self.priority.name,
            "required_coverage": self.required_coverage,
            "search_pattern": self.search_pattern.name,
            "altitude_agl": self.altitude_agl,
            "dwell_time": self.dwell_time,
            "target_types": [t.name for t in self.target_types],
            "reported_activity": self.reported_activity,
            "confidence": self.confidence,
            "time_since_report": self.time_since_report,
        }


@dataclass
class RestrictedZone:
    """
    Airspace to avoid (No-Fly Zones, restricted areas).

    Can be:
    - Permanent (friendly positions, civilian areas)
    - Temporary (active fire zones, weather)
    - Dynamic (pop-up threats detected during mission)
    """
    id: str
    name: str
    center: np.ndarray              # (x, y) center
    radius: float                   # Radius in meters
    floor_altitude: float = 0.0     # Lower altitude limit (0 = ground)
    ceiling_altitude: float = 500.0 # Upper altitude limit
    is_hard_constraint: bool = True # True = never enter, False = avoid if possible
    activation_time: float = 0.0    # Mission time when zone activates (0 = always active)
    deactivation_time: float = -1.0 # When zone deactivates (-1 = never)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "center": self.center.tolist(),
            "radius": self.radius,
            "floor_altitude": self.floor_altitude,
            "ceiling_altitude": self.ceiling_altitude,
            "is_hard_constraint": self.is_hard_constraint,
            "activation_time": self.activation_time,
            "deactivation_time": self.deactivation_time,
        }


@dataclass
class MissionTasking:
    """
    Complete mission tasking from operator to drone.

    This is the top-level structure that contains everything the drone
    needs to execute an ISR mission autonomously.
    """
    # Mission identification
    mission_id: str
    mission_name: str

    # Geographic areas
    nais: List[NamedAreaOfInterest]
    restricted_zones: List[RestrictedZone] = field(default_factory=list)

    # Home/launch position
    launch_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))

    # Operating constraints
    max_altitude_agl: float = 120.0      # meters (regulatory limit)
    min_altitude_agl: float = 20.0       # meters (safety minimum)
    max_mission_time: float = 900.0      # seconds
    battery_reserve_pct: float = 25.0    # Must RTL with this much battery

    # Coverage requirements
    overall_coverage_target: float = 85.0  # % of all NAIs combined

    # Comms parameters
    comms_loss_behavior: str = "CONTINUE"  # CONTINUE, RTL, HOLD
    expected_comms_loss_time: float = -1.0 # -1 = don't expect loss

    # Priority intelligence requirements (what to look for)
    primary_pirs: List[TargetType] = field(default_factory=lambda: [TargetType.MILITARY_VEHICLE])
    secondary_pirs: List[TargetType] = field(default_factory=lambda: [TargetType.PERSONNEL])

    # Weather/environment notes
    weather_conditions: str = "CLEAR"
    visibility_km: float = 10.0

    # Free-text operator notes
    operator_notes: str = ""

    def get_nais_by_priority(self) -> List[NamedAreaOfInterest]:
        """Return NAIs sorted by priority (highest first)."""
        return sorted(self.nais, key=lambda n: n.priority.value)

    def get_primary_nai(self) -> Optional[NamedAreaOfInterest]:
        """Get the highest priority NAI."""
        if not self.nais:
            return None
        return self.get_nais_by_priority()[0]

    def get_total_survey_area(self) -> float:
        """Total area to survey in square meters."""
        return sum(np.pi * nai.radius ** 2 for nai in self.nais)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization/logging."""
        return {
            "mission_id": self.mission_id,
            "mission_name": self.mission_name,
            "nais": [nai.to_dict() for nai in self.nais],
            "restricted_zones": [rz.to_dict() for rz in self.restricted_zones],
            "launch_position": self.launch_position.tolist(),
            "max_altitude_agl": self.max_altitude_agl,
            "min_altitude_agl": self.min_altitude_agl,
            "max_mission_time": self.max_mission_time,
            "battery_reserve_pct": self.battery_reserve_pct,
            "overall_coverage_target": self.overall_coverage_target,
            "comms_loss_behavior": self.comms_loss_behavior,
            "primary_pirs": [p.name for p in self.primary_pirs],
            "secondary_pirs": [p.name for p in self.secondary_pirs],
            "weather_conditions": self.weather_conditions,
            "visibility_km": self.visibility_km,
            "operator_notes": self.operator_notes,
        }

    def to_natural_language(self) -> str:
        """
        Convert tasking to natural language for logging/debugging.

        This is what an operator might say verbally.
        """
        lines = [
            f"MISSION: {self.mission_name} ({self.mission_id})",
            "",
            "OBJECTIVE:",
        ]

        for nai in self.get_nais_by_priority():
            target_str = ", ".join(t.name.lower().replace("_", " ") for t in nai.target_types)
            lines.append(f"  - Survey {nai.name} ({nai.id}) for {target_str}")
            lines.append(f"    Priority: {nai.priority.name}, Coverage: {nai.required_coverage}%")
            if nai.reported_activity:
                lines.append(f"    Intel: {nai.reported_activity}")

        lines.append("")
        lines.append("RESTRICTIONS:")
        for rz in self.restricted_zones:
            constraint = "HARD" if rz.is_hard_constraint else "SOFT"
            lines.append(f"  - AVOID {rz.name} ({rz.id}), {constraint} constraint")

        lines.append("")
        lines.append("CONSTRAINTS:")
        lines.append(f"  - Max mission time: {self.max_mission_time / 60:.0f} minutes")
        lines.append(f"  - Battery reserve: {self.battery_reserve_pct}%")
        lines.append(f"  - Altitude: {self.min_altitude_agl}-{self.max_altitude_agl}m AGL")
        lines.append(f"  - On comms loss: {self.comms_loss_behavior}")

        if self.operator_notes:
            lines.append("")
            lines.append(f"NOTES: {self.operator_notes}")

        return "\n".join(lines)


# =============================================================================
# CANONICAL PROBLEM TASKING TEMPLATES
# =============================================================================

def create_comms_denied_tasking(
    surveillance_center: Tuple[float, float] = (100.0, 100.0),
    surveillance_radius: float = 150.0,
    secondary_area: Optional[Tuple[float, float]] = None,
    expected_comms_loss_time: float = 30.0,
    seed: Optional[int] = None,
) -> MissionTasking:
    """
    Create tasking for Comms-Denied Area Surveillance problem.

    Scenario: Scout assigned areas for enemy activity. Comms will be lost
    at some point - continue mission autonomously.

    Args:
        surveillance_center: Primary NAI center (x, y)
        surveillance_radius: Radius of primary search area
        secondary_area: Optional secondary NAI center
        expected_comms_loss_time: When comms loss is expected (seconds)
        seed: Random seed for variation

    Returns:
        MissionTasking configured for comms-denied surveillance
    """
    rng = np.random.default_rng(seed)

    # Primary NAI - where the intel says enemy activity is
    primary_nai = NamedAreaOfInterest(
        id="NAI_ALPHA",
        name="Primary Surveillance Zone",
        center=np.array(surveillance_center),
        radius=surveillance_radius,
        priority=NAIPriority.HIGH,
        required_coverage=80.0,
        altitude_agl=50.0,
        target_types=[TargetType.MILITARY_VEHICLE, TargetType.PERSONNEL],
        reported_activity="Possible enemy vehicle staging area, reported 2 hours ago",
        confidence=0.7,
        time_since_report=2.0,
    )

    nais = [primary_nai]

    # Secondary NAI if provided
    if secondary_area is not None:
        secondary_nai = NamedAreaOfInterest(
            id="NAI_BRAVO",
            name="Secondary Surveillance Zone",
            center=np.array(secondary_area),
            radius=surveillance_radius * 0.7,  # Smaller secondary area
            priority=NAIPriority.MEDIUM,
            required_coverage=60.0,
            altitude_agl=60.0,
            target_types=[TargetType.VEHICLE, TargetType.PERSONNEL],
            reported_activity="Possible supply route, low confidence",
            confidence=0.4,
            time_since_report=6.0,
        )
        nais.append(secondary_nai)

    return MissionTasking(
        mission_id=f"CD-{rng.integers(1000, 9999)}",
        mission_name="Comms-Denied Area Surveillance",
        nais=nais,
        restricted_zones=[],  # No restricted zones for this problem
        launch_position=np.array([0.0, 0.0, 0.0]),
        max_mission_time=900.0,  # 15 minutes
        battery_reserve_pct=30.0,  # Higher reserve for autonomous ops
        overall_coverage_target=85.0,
        comms_loss_behavior="CONTINUE",  # Key: continue mission on comms loss
        expected_comms_loss_time=expected_comms_loss_time,
        primary_pirs=[TargetType.MILITARY_VEHICLE],
        secondary_pirs=[TargetType.PERSONNEL, TargetType.CIVILIAN_VEHICLE],
        operator_notes="Expect comms loss mid-mission. Continue surveillance autonomously. "
                      "RTL when coverage complete or battery critical.",
    )


def create_dynamic_nfz_tasking(
    destination: Tuple[float, float, float] = (3000.0, 500.0, 80.0),
    nfz_positions: Optional[List[Tuple[float, float, float]]] = None,
    seed: Optional[int] = None,
) -> MissionTasking:
    """
    Create tasking for Dynamic No-Fly Zone Avoidance problem.

    Scenario: Transit to destination while avoiding pop-up NFZs that
    appear during the mission.

    Args:
        destination: Target destination (x, y, z)
        nfz_positions: Pre-planned NFZ positions (some may activate mid-mission)
        seed: Random seed for variation

    Returns:
        MissionTasking configured for NFZ avoidance transit
    """
    rng = np.random.default_rng(seed)

    # Destination NAI (the endpoint we need to reach)
    destination_nai = NamedAreaOfInterest(
        id="NAI_DEST",
        name="Destination Rally Point",
        center=np.array([destination[0], destination[1]]),
        radius=50.0,  # Small radius - just need to reach it
        priority=NAIPriority.CRITICAL,
        required_coverage=100.0,  # Must reach destination
        altitude_agl=destination[2],
        target_types=[],  # No targets to find, just transit
        dwell_time=10.0,  # Hold at destination briefly
    )

    # Default NFZ positions if not provided
    if nfz_positions is None:
        nfz_positions = [
            (1000.0, 400.0, 200.0),  # Along route, activates mid-mission
            (2000.0, 600.0, 150.0),  # Along route, activates later
        ]

    # Create restricted zones (some activate dynamically)
    restricted_zones = []
    for i, (x, y, radius) in enumerate(nfz_positions):
        # First NFZ activates at mission start, others activate later
        activation_time = 0.0 if i == 0 else 180.0 + i * 60.0

        rz = RestrictedZone(
            id=f"NFZ_{i+1:02d}",
            name=f"Restricted Zone {chr(65 + i)}",  # A, B, C, ...
            center=np.array([x, y]),
            radius=radius,
            floor_altitude=0.0,
            ceiling_altitude=500.0,
            is_hard_constraint=True,
            activation_time=activation_time,
        )
        restricted_zones.append(rz)

    return MissionTasking(
        mission_id=f"NFZ-{rng.integers(1000, 9999)}",
        mission_name="Dynamic NFZ Transit",
        nais=[destination_nai],
        restricted_zones=restricted_zones,
        launch_position=np.array([0.0, 0.0, 0.0]),
        max_mission_time=600.0,  # 10 minutes for transit
        battery_reserve_pct=25.0,
        overall_coverage_target=100.0,  # Must reach destination
        comms_loss_behavior="RTL",  # Return home if comms lost during transit
        primary_pirs=[],  # No targets, just transit
        operator_notes="Transit to destination. Dynamic NFZs may appear. "
                      "Replan route as needed. Reach destination within time limit.",
    )


def create_multi_objective_tasking(
    nai_positions: Optional[List[Tuple[float, float, float, str]]] = None,
    threat_positions: Optional[List[Tuple[float, float, float]]] = None,
    seed: Optional[int] = None,
) -> MissionTasking:
    """
    Create tasking for Multi-Objective ISR with Threat Avoidance.

    Scenario: Multiple surveillance objectives with varying priorities,
    while avoiding threat zones and managing resources.

    Args:
        nai_positions: List of (x, y, radius, priority) for NAIs
        threat_positions: List of (x, y, radius) for threat zones
        seed: Random seed for variation

    Returns:
        MissionTasking configured for multi-objective ISR
    """
    rng = np.random.default_rng(seed)

    # Default NAI positions if not provided
    if nai_positions is None:
        nai_positions = [
            (200.0, 150.0, 100.0, "HIGH"),      # Close, high priority
            (400.0, -100.0, 80.0, "MEDIUM"),    # Medium distance
            (600.0, 200.0, 120.0, "HIGH"),      # Farther, also high priority
            (300.0, 300.0, 60.0, "LOW"),        # Low priority
        ]

    # Default threat positions if not provided
    if threat_positions is None:
        threat_positions = [
            (350.0, 50.0, 80.0),   # Between NAIs
            (500.0, 100.0, 100.0), # Near high-priority NAI
        ]

    # Create NAIs
    nais = []
    for i, (x, y, radius, priority_str) in enumerate(nai_positions):
        priority = NAIPriority[priority_str]
        nai = NamedAreaOfInterest(
            id=f"NAI_{chr(65 + i)}",  # NAI_A, NAI_B, etc.
            name=f"Objective {chr(65 + i)}",
            center=np.array([x, y]),
            radius=radius,
            priority=priority,
            required_coverage=90.0 if priority == NAIPriority.HIGH else 70.0,
            altitude_agl=40.0 + i * 10.0,  # Vary altitude per NAI
            target_types=[TargetType.MILITARY_VEHICLE, TargetType.PERSONNEL],
            reported_activity=f"Suspected activity, priority {priority.name}",
            confidence=0.8 if priority == NAIPriority.HIGH else 0.5,
        )
        nais.append(nai)

    # Create threat zones (soft constraints - avoid if possible)
    restricted_zones = []
    for i, (x, y, radius) in enumerate(threat_positions):
        rz = RestrictedZone(
            id=f"THREAT_{i+1:02d}",
            name=f"Threat Zone {i+1}",
            center=np.array([x, y]),
            radius=radius,
            is_hard_constraint=False,  # Soft constraint - can enter briefly if needed
            floor_altitude=0.0,
            ceiling_altitude=100.0,  # Can fly over if high enough
        )
        restricted_zones.append(rz)

    return MissionTasking(
        mission_id=f"MO-{rng.integers(1000, 9999)}",
        mission_name="Multi-Objective ISR",
        nais=nais,
        restricted_zones=restricted_zones,
        launch_position=np.array([0.0, 0.0, 0.0]),
        max_mission_time=1200.0,  # 20 minutes
        battery_reserve_pct=25.0,
        overall_coverage_target=80.0,
        comms_loss_behavior="CONTINUE",
        primary_pirs=[TargetType.MILITARY_VEHICLE],
        secondary_pirs=[TargetType.PERSONNEL, TargetType.EQUIPMENT],
        operator_notes="Multiple objectives with varying priorities. "
                      "Minimize exposure to threat zones. "
                      "High-priority NAIs must be surveilled. "
                      "Low-priority NAIs are opportunistic.",
    )


# =============================================================================
# WORLD GENERATION FROM TASKING
# =============================================================================

def generate_poi_distribution_from_tasking(
    tasking: MissionTasking,
    seed: Optional[int] = None,
) -> List[Dict[str, Any]]:
    """
    Generate POI (target) positions based on mission tasking.

    This creates a realistic distribution of targets that:
    1. Clusters targets near NAI centers (where intel says they are)
    2. Adds uncertainty based on intel confidence
    3. Distributes target types according to PIRs

    Args:
        tasking: Mission tasking with NAIs and intel
        seed: Random seed for reproducibility

    Returns:
        List of POI dictionaries with position, type, and cluster info
    """
    rng = np.random.default_rng(seed)
    pois = []

    for nai in tasking.nais:
        # Number of targets scales with:
        # - NAI radius (larger area = more targets)
        # - Intel confidence (higher confidence = more targets where expected)
        # - Time since report (older intel = more spread/uncertainty)

        base_count = int(nai.radius / 20)  # ~1 target per 20m radius
        confidence_factor = 0.5 + 0.5 * nai.confidence
        target_count = max(1, int(base_count * confidence_factor))

        # Spread increases with time since report and decreases with confidence
        spread_factor = 1.0 + nai.time_since_report * 0.1
        effective_spread = nai.radius * 0.5 * spread_factor / max(0.3, nai.confidence)

        for i in range(target_count):
            # Sample position near NAI center with Gaussian spread
            offset = rng.normal(0, effective_spread, 2)
            position = nai.center + offset

            # Clamp to NAI radius (targets should be in the area we're told)
            dist = np.linalg.norm(position - nai.center)
            if dist > nai.radius * 1.2:  # Allow 20% outside
                position = nai.center + (position - nai.center) * (nai.radius / dist)

            # Select target type from NAI's expected types
            if nai.target_types:
                target_type = rng.choice(nai.target_types)
            else:
                target_type = TargetType.VEHICLE

            pois.append({
                "position": position.tolist(),
                "type": target_type.name,
                "nai_id": nai.id,
                "cluster_center": nai.center.tolist(),
                "confidence": nai.confidence,
            })

    # Add some "surprise" targets outside NAIs (fog of war)
    # 10-20% of targets may be in unexpected locations
    num_surprise = max(1, int(len(pois) * rng.uniform(0.1, 0.2)))

    for _ in range(num_surprise):
        # Random position within overall mission area
        if tasking.nais:
            all_centers = np.array([nai.center for nai in tasking.nais])
            area_center = np.mean(all_centers, axis=0)
            area_radius = np.max([np.linalg.norm(nai.center - area_center) + nai.radius
                                  for nai in tasking.nais])
        else:
            area_center = np.array([0.0, 0.0])
            area_radius = 200.0

        angle = rng.uniform(0, 2 * np.pi)
        dist = rng.uniform(0, area_radius)
        position = area_center + dist * np.array([np.cos(angle), np.sin(angle)])

        target_type = rng.choice(list(TargetType))

        pois.append({
            "position": position.tolist(),
            "type": target_type.name,
            "nai_id": "UNKNOWN",  # Not in any known NAI
            "cluster_center": None,
            "confidence": 0.0,  # Unknown target
        })

    return pois


def tasking_to_ontology_facts(tasking: MissionTasking) -> str:
    """
    Convert mission tasking to ontology facts for the reasoning engine.

    Returns TPTP-format facts that can be loaded by Vampire.
    """
    facts = []
    facts.append("% Mission tasking facts")
    facts.append(f"% Mission: {tasking.mission_name}")
    facts.append("")

    # NAI facts
    for nai in tasking.nais:
        facts.append(f"% NAI: {nai.name}")
        facts.append(f"fof(nai_{nai.id.lower()}, axiom, is_nai({nai.id.lower()})).")
        facts.append(f"fof(nai_{nai.id.lower()}_center, axiom, "
                    f"nai_center({nai.id.lower()}, {nai.center[0]:.1f}, {nai.center[1]:.1f})).")
        facts.append(f"fof(nai_{nai.id.lower()}_radius, axiom, "
                    f"nai_radius({nai.id.lower()}, {nai.radius:.1f})).")
        facts.append(f"fof(nai_{nai.id.lower()}_priority, axiom, "
                    f"nai_priority({nai.id.lower()}, {nai.priority.value})).")
        facts.append(f"fof(nai_{nai.id.lower()}_coverage, axiom, "
                    f"nai_required_coverage({nai.id.lower()}, {nai.required_coverage:.1f})).")
        facts.append("")

    # Restricted zone facts
    for rz in tasking.restricted_zones:
        facts.append(f"% Restricted Zone: {rz.name}")
        facts.append(f"fof(rz_{rz.id.lower()}, axiom, is_restricted_zone({rz.id.lower()})).")
        facts.append(f"fof(rz_{rz.id.lower()}_center, axiom, "
                    f"rz_center({rz.id.lower()}, {rz.center[0]:.1f}, {rz.center[1]:.1f})).")
        facts.append(f"fof(rz_{rz.id.lower()}_radius, axiom, "
                    f"rz_radius({rz.id.lower()}, {rz.radius:.1f})).")
        constraint_type = "hard" if rz.is_hard_constraint else "soft"
        facts.append(f"fof(rz_{rz.id.lower()}_type, axiom, "
                    f"rz_constraint_type({rz.id.lower()}, {constraint_type})).")
        facts.append("")

    # Mission constraints
    facts.append("% Mission constraints")
    facts.append(f"fof(mission_max_time, axiom, max_mission_time({tasking.max_mission_time:.1f})).")
    facts.append(f"fof(mission_battery_reserve, axiom, "
                f"required_battery_reserve({tasking.battery_reserve_pct:.1f})).")
    facts.append(f"fof(mission_coverage_target, axiom, "
                f"coverage_target({tasking.overall_coverage_target:.1f})).")

    return "\n".join(facts)
