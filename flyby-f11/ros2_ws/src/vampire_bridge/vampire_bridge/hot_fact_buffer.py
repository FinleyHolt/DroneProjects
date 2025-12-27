"""
hot_fact_buffer.py - Thread-safe buffer for volatile state facts

Implements the HotFactBuffer from ADR-002: State Persistence Architecture.
Stores rapidly-changing facts (position, battery, comms, sensor status) for
atomic snapshot retrieval during Vampire query construction.

Design constraints (from ADR):
    - Query rate: 20Hz (50ms period)
    - Fact injection latency: <0.5ms
    - Thread-safe for concurrent queries
    - Memory budget: <20KB for hot facts
"""

import threading
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class FactMetadata:
    """Metadata for a hot fact entry."""
    tptp_axiom: str
    timestamp: float
    source_topic: Optional[str] = None


class HotFactBuffer:
    """
    Thread-safe buffer for volatile state facts updated at sensor rate.

    Hot facts are state that changes frequently (10-50Hz) and must be
    injected into each Vampire query for accurate reasoning. The buffer
    provides atomic snapshots to ensure query consistency.

    Fact Schema (from ADR-002):
        - battery_status: has_battery_status(uav, status)
        - position: position(uav, pos_id)
        - comms_status: has_comms_status(uav, status)
        - obstacle_distance: min_obstacle_distance(uav, dist)
        - sensor_health: sensor_operational(uav, sensor)
        - localization_quality: has_position_quality(uav, quality)
        - distance_to_home: distance_to_home(uav, dist)
        - current_battery_level: current_battery_level(uav, percent)
    """

    # TPTP fact templates for each fact type
    FACT_SCHEMA = {
        'battery_status': 'has_battery_status({uav}, {status})',
        'current_battery_level': 'current_battery_level({uav}, {level})',
        'position': 'position({uav}, {pos_id})',
        'comms_status': 'has_comms_status({uav}, {status})',
        'obstacle_distance': 'min_obstacle_distance({uav}, {dist})',
        'sensor_health': 'sensor_operational({uav}, {sensor})',
        'localization_quality': 'has_position_quality({uav}, {quality})',
        'distance_to_home': 'distance_to_home({uav}, {dist})',
        'armed_status': 'is_armed({uav}, {armed})',
        'flight_mode': 'has_flight_mode({uav}, {mode})',
    }

    # Default staleness thresholds by fact type (seconds)
    DEFAULT_STALE_THRESHOLDS = {
        'battery_status': 5.0,         # Battery updates at ~1Hz
        'current_battery_level': 5.0,
        'position': 0.5,               # Position critical, expect at 50Hz
        'comms_status': 2.0,           # Comms status at ~1Hz
        'obstacle_distance': 0.5,      # Safety critical
        'sensor_health': 2.0,
        'localization_quality': 1.0,
        'distance_to_home': 1.0,
        'armed_status': 2.0,
        'flight_mode': 2.0,
    }

    def __init__(
        self,
        uav_id: str = 'drone_alpha',
        default_stale_threshold: float = 1.0
    ):
        """
        Initialize the hot fact buffer.

        Args:
            uav_id: Identifier for the UAV in TPTP facts
            default_stale_threshold: Default staleness threshold in seconds
        """
        self._uav_id = uav_id
        self._default_stale_threshold = default_stale_threshold
        self._facts: dict[str, FactMetadata] = {}
        self._lock = threading.RLock()

        # Configurable staleness thresholds
        self._stale_thresholds = self.DEFAULT_STALE_THRESHOLDS.copy()

        # Statistics
        self._update_count = 0
        self._snapshot_count = 0

    @property
    def uav_id(self) -> str:
        """Return the UAV identifier."""
        return self._uav_id

    def update(self, fact_type: str, source_topic: Optional[str] = None, **kwargs) -> bool:
        """
        Update a hot fact with new values.

        Thread-safe update using schema templates. Unknown fact types are
        silently ignored to prevent runtime errors from malformed data.

        Args:
            fact_type: Type of fact from FACT_SCHEMA
            source_topic: Optional ROS 2 topic that sourced this fact
            **kwargs: Template parameters for the fact type

        Returns:
            True if fact was updated, False if fact_type unknown

        Example:
            buffer.update('battery_status', status='battery_low')
            buffer.update('distance_to_home', dist='3.2')
        """
        template = self.FACT_SCHEMA.get(fact_type)
        if template is None:
            return False

        # Build TPTP axiom from template
        try:
            predicate = template.format(uav=self._uav_id, **kwargs)
            axiom = f"fof({fact_type}_state, axiom, {predicate})."
        except KeyError as e:
            # Missing template parameter - don't update
            return False

        with self._lock:
            self._facts[fact_type] = FactMetadata(
                tptp_axiom=axiom,
                timestamp=time.time(),
                source_topic=source_topic
            )
            self._update_count += 1

        return True

    def update_raw(self, fact_name: str, tptp_axiom: str, source_topic: Optional[str] = None) -> None:
        """
        Update with a pre-formed TPTP axiom (for custom facts).

        Use this when the fact doesn't fit the standard schema or when
        constructing complex facts externally.

        Args:
            fact_name: Unique identifier for this fact
            tptp_axiom: Complete TPTP axiom string
            source_topic: Optional ROS 2 topic that sourced this fact
        """
        with self._lock:
            self._facts[fact_name] = FactMetadata(
                tptp_axiom=tptp_axiom,
                timestamp=time.time(),
                source_topic=source_topic
            )
            self._update_count += 1

    def get_snapshot(self) -> str:
        """
        Return atomic snapshot of all hot facts as TPTP block.

        This is the primary method for query construction. The lock ensures
        all facts represent a consistent state at a single point in time.

        Returns:
            Multi-line string of TPTP axioms, empty string if no facts
        """
        with self._lock:
            self._snapshot_count += 1
            if not self._facts:
                return ''
            return '\n'.join(
                meta.tptp_axiom for meta in self._facts.values()
            )

    def get_snapshot_dict(self) -> dict[str, str]:
        """
        Return atomic snapshot as dictionary mapping fact_type to axiom.

        Useful for debugging and selective fact retrieval.

        Returns:
            Dict mapping fact names to their TPTP axioms
        """
        with self._lock:
            return {
                name: meta.tptp_axiom
                for name, meta in self._facts.items()
            }

    def get_stale_facts(self, max_age_sec: Optional[float] = None) -> list[str]:
        """
        Return list of facts older than their staleness threshold.

        Uses per-fact-type thresholds from STALE_THRESHOLDS, falling back
        to max_age_sec or default_stale_threshold if not specified.

        Args:
            max_age_sec: Override threshold for all facts (optional)

        Returns:
            List of fact names that are stale
        """
        now = time.time()
        stale = []

        with self._lock:
            for name, meta in self._facts.items():
                if max_age_sec is not None:
                    threshold = max_age_sec
                else:
                    threshold = self._stale_thresholds.get(
                        name, self._default_stale_threshold
                    )

                if now - meta.timestamp > threshold:
                    stale.append(name)

        return stale

    def get_fact_age(self, fact_type: str) -> Optional[float]:
        """
        Get the age of a specific fact in seconds.

        Args:
            fact_type: The fact type to check

        Returns:
            Age in seconds, or None if fact doesn't exist
        """
        with self._lock:
            meta = self._facts.get(fact_type)
            if meta is None:
                return None
            return time.time() - meta.timestamp

    def is_fact_stale(self, fact_type: str) -> bool:
        """
        Check if a specific fact is stale.

        Args:
            fact_type: The fact type to check

        Returns:
            True if stale or missing, False if fresh
        """
        age = self.get_fact_age(fact_type)
        if age is None:
            return True  # Missing facts are considered stale

        threshold = self._stale_thresholds.get(
            fact_type, self._default_stale_threshold
        )
        return age > threshold

    def set_stale_threshold(self, fact_type: str, threshold_sec: float) -> None:
        """
        Configure staleness threshold for a fact type.

        Args:
            fact_type: The fact type to configure
            threshold_sec: Staleness threshold in seconds
        """
        with self._lock:
            self._stale_thresholds[fact_type] = threshold_sec

    def remove(self, fact_type: str) -> bool:
        """
        Remove a fact from the buffer.

        Args:
            fact_type: The fact type to remove

        Returns:
            True if fact was removed, False if not found
        """
        with self._lock:
            if fact_type in self._facts:
                del self._facts[fact_type]
                return True
            return False

    def clear(self) -> None:
        """Clear all facts from the buffer."""
        with self._lock:
            self._facts.clear()

    def get_stats(self) -> dict:
        """
        Get buffer statistics.

        Returns:
            Dict with buffer metrics
        """
        with self._lock:
            stale_count = len(self.get_stale_facts())
            return {
                'fact_count': len(self._facts),
                'update_count': self._update_count,
                'snapshot_count': self._snapshot_count,
                'stale_count': stale_count,
                'fact_types': list(self._facts.keys()),
            }

    def __len__(self) -> int:
        """Return number of facts in buffer."""
        with self._lock:
            return len(self._facts)
