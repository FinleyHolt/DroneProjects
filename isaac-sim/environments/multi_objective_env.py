#!/usr/bin/env python3
"""
Canonical Problem 3: Multi-Objective ISR with Threat Avoidance

Mission: Conduct ISR collection on multiple targets of interest (TOIs)
with varying priority levels while avoiding elevated threat areas and
maintaining sufficient battery for safe return.

Key Challenges:
- Multiple targets with different priorities and values
- Threat zone avoidance (High = prohibited, Medium = time-limited)
- Battery budget optimization across targets
- Risk-reward tradeoffs

Per ONTOLOGY_FOUNDATION.qmd Section 1.3
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple

from .base_isr_env import (
    BaseISREnvironment, EnvironmentConfig, UAVState,
    FlightPhase, CommsStatus, GNSSStatus, ThreatLevel,
    NoFlyZone, ThreatZone, TargetOfInterest, GeofenceConfig
)
from .domain_randomizer import DomainRandomizer, RandomizationConfig
from .ontology_bridge import OntologyStateBridge


@dataclass
class TargetConfig:
    """Configuration for a single target."""
    id: str
    position: np.ndarray
    priority: int  # 1=Critical, 2=High, 3=Medium, 4=Low
    value: float
    battery_cost: float  # % to reach and observe
    threat_level: ThreatLevel = ThreatLevel.NONE


@dataclass
class ThreatZoneConfig:
    """Configuration for a threat zone."""
    id: str
    center: np.ndarray
    radius: float
    threat_level: ThreatLevel
    risk_rate: float = 1.0  # Penalty per second


@dataclass
class MultiObjectiveConfig(EnvironmentConfig):
    """Configuration for Multi-Objective ISR environment."""
    # Mission parameters
    max_mission_time: float = 1200.0  # 20 minutes

    # Battery
    initial_battery: float = 92.0
    min_battery_at_landing: float = 15.0

    # Threat exposure limits
    max_medium_threat_time: float = 30.0  # seconds

    # Target distribution (from ontology document)
    targets: List[TargetConfig] = field(default_factory=lambda: [
        TargetConfig("toi_1", np.array([200, 300, 0]), 1, 100, 8, ThreatLevel.MEDIUM),
        TargetConfig("toi_2", np.array([400, -200, 0]), 1, 100, 6, ThreatLevel.LOW),
        TargetConfig("toi_3", np.array([-300, 400, 0]), 2, 75, 15, ThreatLevel.HIGH),
        TargetConfig("toi_4", np.array([100, -400, 0]), 2, 75, 5, ThreatLevel.LOW),
        TargetConfig("toi_5", np.array([-200, -100, 0]), 3, 50, 9, ThreatLevel.MEDIUM),
        TargetConfig("toi_6", np.array([350, 350, 0]), 3, 50, 18, ThreatLevel.HIGH),
        TargetConfig("toi_7", np.array([-100, 200, 0]), 4, 25, 4, ThreatLevel.LOW),
        TargetConfig("toi_8", np.array([50, -300, 0]), 4, 25, 10, ThreatLevel.MEDIUM),
    ])

    # Threat zones
    threat_zones_config: List[ThreatZoneConfig] = field(default_factory=lambda: [
        ThreatZoneConfig("high_threat_north", np.array([-300, 400, 0]), 100, ThreatLevel.HIGH),
        ThreatZoneConfig("high_threat_east", np.array([350, 350, 0]), 80, ThreatLevel.HIGH),
        ThreatZoneConfig("medium_threat_center", np.array([0, 0, 0]), 150, ThreatLevel.MEDIUM, 2.0),
    ])

    # Success criteria
    target_value_threshold: float = 0.8  # 80% of achievable

    # Comms status (intermittent)
    comms_availability: float = 0.5  # 50% availability


class MultiObjectiveISREnv(BaseISREnvironment):
    """
    Multi-Objective ISR Training Environment.

    Implements Canonical Problem 3 from ONTOLOGY_FOUNDATION.qmd.

    Reward Structure (from ontology document):
    - High threat entry: -500 (catastrophic)
    - Medium threat exposure: -risk_rate * dt
    - Target observation: value * priority_multiplier + critical bonus
    - Resource efficiency: +0.5 * min(value/battery, 20)
    - Mission success (80%): +100
    """

    # Priority multipliers per ontology document
    PRIORITY_MULTIPLIERS = {
        1: 2.0,   # Critical
        2: 1.5,   # High
        3: 1.0,   # Medium
        4: 0.75,  # Low
    }

    def __init__(self, config: Optional[MultiObjectiveConfig] = None):
        if config is None:
            config = MultiObjectiveConfig()

        super().__init__(config)

        self.config: MultiObjectiveConfig = config

        # Target tracking
        self.targets: List[TargetOfInterest] = []
        self.total_value: float = 0.0
        self.achievable_value: float = 0.0
        self.collected_value: float = 0.0

        # Threat tracking
        self.medium_threat_time: float = 0.0
        self.threat_zones: List[ThreatZone] = []

        # Comms simulation
        self.comms_check_interval: float = 5.0
        self.last_comms_check: float = 0.0

    def _setup_environment(self) -> None:
        """Setup multi-objective ISR environment."""
        # Create targets
        self.targets = []
        self.total_value = 0.0
        self.achievable_value = 0.0

        for tc in self.config.targets:
            is_in_high_threat = tc.threat_level == ThreatLevel.HIGH

            target = TargetOfInterest(
                id=tc.id,
                position=tc.position.copy(),
                priority=tc.priority,
                value=tc.value,
                battery_cost=tc.battery_cost,
                observed=False,
                in_high_threat=is_in_high_threat,
            )
            self.targets.append(target)
            self.total_value += tc.value

            if not is_in_high_threat:
                self.achievable_value += tc.value

        print(f"  Targets: {len(self.targets)}")
        print(f"  Total value: {self.total_value}")
        print(f"  Achievable value (avoid high threat): {self.achievable_value}")

        # Create threat zones
        self.threat_zones = []
        for tz in self.config.threat_zones_config:
            zone = ThreatZone(
                id=tz.id,
                center=tz.center.copy(),
                radius=tz.radius,
                threat_level=tz.threat_level,
                risk_rate=tz.risk_rate,
            )
            self.threat_zones.append(zone)

        print(f"  Threat zones: {len(self.threat_zones)}")

        # Setup geofence
        self.config.geofence = GeofenceConfig(
            min_x=-500.0,
            max_x=500.0,
            min_y=-500.0,
            max_y=500.0,
            min_z=0.0,
            max_z=120.0,
        )

        # No NFZs for this problem (threats instead)
        self.nfz_zones = []

        # Initialize domain randomizer
        rand_config = RandomizationConfig(
            randomize_lighting=self.config.randomize_lighting,
            randomize_weather=self.config.randomize_weather,
            randomize_sensors=self.config.randomize_sensors,
        )
        self.randomizer = DomainRandomizer(rand_config)

        # Initialize ontology bridge
        self.ontology_bridge = OntologyStateBridge()

    def reset(self, seed: Optional[int] = None) -> UAVState:
        """Reset environment for new episode."""
        state = super().reset(seed)

        # Reset targets
        for target in self.targets:
            target.observed = False

        self.collected_value = 0.0
        self.medium_threat_time = 0.0
        self.last_comms_check = 0.0

        # Set initial battery
        state.battery_pct = self.config.initial_battery
        state.flight_phase = FlightPhase.MISSION_EXECUTION

        # Set intermittent comms
        state.comms_status = CommsStatus.OPERATIONAL

        # Randomize target positions if enabled
        if self.randomizer and self.config.randomize_obstacles:
            self._randomize_targets(seed)

        return state

    def _randomize_targets(self, seed: Optional[int] = None) -> None:
        """Randomize target positions."""
        if seed is not None:
            np.random.seed(seed)

        for target in self.targets:
            # Add position noise
            noise = np.random.normal(0, 20.0, size=3)
            noise[2] = 0  # Keep on ground
            target.position = target.position + noise

    def _update_state(self) -> None:
        """Update state including threat and target tracking."""
        super()._update_state()

        # Update threat exposure
        self._update_threat_exposure()

        # Check target observations
        self._check_target_observations()

        # Update comms status (intermittent)
        self._update_comms_status()

        # Calculate remaining battery budget
        self._calculate_battery_budget()

    def _update_threat_exposure(self) -> None:
        """Update threat zone exposure tracking."""
        pos = self.uav_state.position
        dt = 1.0 / 60.0  # Assuming 60Hz

        for zone in self.threat_zones:
            dist = np.linalg.norm(pos[:2] - zone.center[:2])

            if dist < zone.radius:
                if zone.threat_level == ThreatLevel.MEDIUM:
                    self.medium_threat_time += dt
                    self.uav_state.threat_exposure = self.medium_threat_time

    def _check_target_observations(self) -> None:
        """Check if any targets are observed by camera."""
        pos = self.uav_state.position
        altitude = pos[2]

        if altitude < 10.0:
            return  # Too low for target observation

        # Camera footprint (ISR camera has longer range)
        footprint_radius = altitude * 0.8  # Wider FOV for ISR

        for target in self.targets:
            if target.observed:
                continue

            dist = np.linalg.norm(pos[:2] - target.position[:2])

            if dist < footprint_radius:
                # Check if we're in high threat zone (shouldn't be possible)
                in_high_threat = False
                for zone in self.threat_zones:
                    if zone.threat_level == ThreatLevel.HIGH:
                        if np.linalg.norm(pos[:2] - zone.center[:2]) < zone.radius:
                            in_high_threat = True
                            break

                if in_high_threat:
                    continue  # Can't observe from high threat zone

                # Observe target
                target.observed = True
                self.collected_value += target.value
                print(f">>> Target {target.id} observed (Priority {target.priority}, Value {target.value})")

    def _update_comms_status(self) -> None:
        """Simulate intermittent communications."""
        if self.uav_state.mission_time - self.last_comms_check >= self.comms_check_interval:
            self.last_comms_check = self.uav_state.mission_time

            # Random comms availability
            if np.random.random() < self.config.comms_availability:
                self.uav_state.comms_status = CommsStatus.OPERATIONAL
            else:
                self.uav_state.comms_status = CommsStatus.DEGRADED

    def _calculate_battery_budget(self) -> None:
        """Calculate remaining battery budget for target collection."""
        # Current battery minus reserve
        available = self.uav_state.battery_pct - self.config.min_battery_at_landing

        # Check which targets are still reachable
        for target in self.targets:
            if target.observed or target.in_high_threat:
                continue

            # Update reachability based on battery
            target.in_high_threat = target.battery_cost > available

    def compute_reward(self, state: UAVState, next_state: UAVState) -> float:
        """
        Compute reward per ONTOLOGY_FOUNDATION.qmd reward structure.
        """
        reward = 0.0

        # ==========================================
        # Threat avoidance (critical)
        # ==========================================

        # High threat entry is catastrophic
        if next_state.current_threat_level == ThreatLevel.HIGH:
            return -500.0

        # Medium threat exposure penalty
        if next_state.current_threat_level == ThreatLevel.MEDIUM:
            # Find active zone's risk rate
            for zone in self.threat_zones:
                if zone.threat_level == ThreatLevel.MEDIUM:
                    dist = np.linalg.norm(next_state.position[:2] - zone.center[:2])
                    if dist < zone.radius:
                        dt = 1.0 / 60.0
                        reward -= zone.risk_rate * dt
                        break

        # ==========================================
        # Target observation (weighted by priority)
        # ==========================================

        # Check for newly observed targets
        prev_observed = {t.id: t.observed for t in self.targets}

        for target in self.targets:
            if target.observed and not prev_observed.get(target.id, True):
                multiplier = self.PRIORITY_MULTIPLIERS.get(target.priority, 1.0)
                reward += target.value * multiplier

                # Critical target bonus
                if target.priority == 1:
                    reward += 50.0

        # ==========================================
        # Resource efficiency
        # ==========================================

        battery_used = state.battery_pct - next_state.battery_pct

        if battery_used > 0.1:
            # Value gained this step
            prev_value = sum(
                t.value for t in self.targets
                if prev_observed.get(t.id, False)
            )
            curr_value = self.collected_value
            value_gained = curr_value - prev_value

            if value_gained > 0:
                efficiency = min(value_gained / battery_used, 20.0)
                reward += 0.5 * efficiency

        # ==========================================
        # Mission success threshold (80%)
        # ==========================================

        if self.achievable_value > 0:
            completion_ratio = self.collected_value / self.achievable_value

            if completion_ratio >= self.config.target_value_threshold:
                # Only give bonus once when threshold crossed
                prev_ratio = (self.collected_value - (
                    sum(t.value for t in self.targets if t.observed) -
                    sum(t.value for t in self.targets if prev_observed.get(t.id, False))
                )) / self.achievable_value

                if prev_ratio < self.config.target_value_threshold:
                    reward += 100.0

        # ==========================================
        # Safety penalties
        # ==========================================

        if not next_state.in_geofence:
            reward -= 100.0

        # Threat exposure limit exceeded
        if self.medium_threat_time > self.config.max_medium_threat_time:
            # Continuous penalty for exceeding limit
            excess = self.medium_threat_time - self.config.max_medium_threat_time
            reward -= 5.0 * excess

        return reward

    def is_done(self, state: UAVState) -> bool:
        """Check if episode is complete."""
        # High threat zone entry (terminal)
        if state.current_threat_level == ThreatLevel.HIGH:
            return True

        # All reachable targets observed
        all_observed = all(
            target.observed or target.in_high_threat
            for target in self.targets
        )
        if all_observed:
            return True

        # Mission time exceeded
        if state.mission_time >= self.config.max_mission_time:
            return True

        # Battery depleted
        if state.battery_pct < 10.0:
            return True

        # Geofence violation
        if not state.in_geofence:
            return True

        # Excessive threat exposure triggers abort
        if self.medium_threat_time > self.config.max_medium_threat_time * 2:
            return True

        return False

    def get_info(self, state: UAVState) -> Dict[str, Any]:
        """Get additional info about current state."""
        # Target statistics
        observed_count = sum(1 for t in self.targets if t.observed)
        critical_observed = sum(1 for t in self.targets if t.observed and t.priority == 1)
        critical_total = sum(1 for t in self.targets if t.priority == 1 and not t.in_high_threat)

        # Value statistics
        completion_ratio = (
            self.collected_value / self.achievable_value
            if self.achievable_value > 0 else 0
        )

        return {
            # Target metrics
            "targets_observed": observed_count,
            "targets_total": len(self.targets),
            "critical_observed": critical_observed,
            "critical_total": critical_total,

            # Value metrics
            "collected_value": self.collected_value,
            "achievable_value": self.achievable_value,
            "total_value": self.total_value,
            "completion_ratio": completion_ratio,

            # Threat metrics
            "in_threat_zone": state.in_threat_zone,
            "current_threat": state.current_threat_level.name,
            "medium_threat_time": self.medium_threat_time,
            "threat_limit": self.config.max_medium_threat_time,

            # Battery
            "battery_pct": state.battery_pct,
            "battery_budget": state.battery_pct - self.config.min_battery_at_landing,

            # Success criteria
            "value_threshold_met": completion_ratio >= self.config.target_value_threshold,
            "threat_limit_ok": self.medium_threat_time <= self.config.max_medium_threat_time,
            "high_threat_entries": 1 if state.current_threat_level == ThreatLevel.HIGH else 0,

            # Mission success
            "mission_success": (
                completion_ratio >= self.config.target_value_threshold and
                self.medium_threat_time <= self.config.max_medium_threat_time and
                state.current_threat_level != ThreatLevel.HIGH
            ),

            # Per-target status
            "target_status": [
                {
                    "id": t.id,
                    "priority": t.priority,
                    "value": t.value,
                    "observed": t.observed,
                    "in_high_threat": t.in_high_threat,
                    "position": t.position.tolist(),
                }
                for t in self.targets
            ],

            # Ontology state
            "ontology_state": self.ontology_bridge.get_symbolic_state(
                state, self.nfz_zones, self.threat_zones, self.targets
            ) if self.ontology_bridge else None,
        }

    @property
    def observation_dim(self) -> int:
        """Extended observation dimension for multi-objective ISR."""
        base_dim = super().observation_dim
        # Add: per-target features (8 targets * 5 features) + threat summary
        return base_dim + 8 * 5 + 3

    def state_to_observation(self, state: UAVState) -> np.ndarray:
        """Convert state to observation with target and threat features."""
        base_obs = super().state_to_observation(state)

        # Target features
        target_features = []
        for target in self.targets:
            # Direction to target (normalized)
            to_target = target.position - state.position
            dist = np.linalg.norm(to_target)
            direction = to_target / (dist + 1e-6)

            target_features.extend([
                direction[0], direction[1],  # Direction XY
                np.clip(dist / 500.0, 0, 1),  # Normalized distance
                float(target.observed),
                float(target.in_high_threat),
            ])

        # Threat summary
        threat_features = [
            float(state.in_threat_zone),
            state.current_threat_level.value / 3.0,
            self.medium_threat_time / self.config.max_medium_threat_time,
        ]

        extra_features = np.array(
            target_features + threat_features,
            dtype=np.float32
        )

        return np.concatenate([base_obs, extra_features])

    def get_target_priorities(self) -> Dict[str, int]:
        """Get mapping of target IDs to priorities for hierarchical planning."""
        return {t.id: t.priority for t in self.targets}

    def get_reachable_targets(self) -> List[TargetOfInterest]:
        """Get list of targets that are still reachable."""
        return [
            t for t in self.targets
            if not t.observed and not t.in_high_threat
        ]

    def get_critical_targets_pending(self) -> List[TargetOfInterest]:
        """Get critical targets that haven't been observed."""
        return [
            t for t in self.targets
            if t.priority == 1 and not t.observed and not t.in_high_threat
        ]
