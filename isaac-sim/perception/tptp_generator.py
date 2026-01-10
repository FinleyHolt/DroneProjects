"""
TPTP fact generator for Vampire theorem prover integration.

Converts ALL detections (not just top-10) into TPTP facts for safety checking.
This ensures Vampire can reason about every detected object in the scene.
"""
import numpy as np
from typing import List, Dict
from dataclasses import dataclass
import hashlib
from collections import deque
import time

try:
    from .detector import Detection
except ImportError:
    from detector import Detection


@dataclass
class TPTPConfig:
    """Configuration for TPTP fact generation."""
    max_facts_per_query: int = 100
    dedup_window_ms: float = 500.0
    include_velocity: bool = True
    include_tracking: bool = True


class TPTPGenerator:
    """
    Generates TPTP facts from detections for Vampire safety queries.

    Fact types generated:
    1. observedObject - Object detected with class and position
    2. distanceTo - Distance from UAV to object
    3. personDetected - Special fact for safety-critical person detections
    4. approachingObject - Object moving toward UAV
    5. crowdDensity - High density of persons in area
    """

    def __init__(self, config: TPTPConfig = None):
        self.config = config or TPTPConfig()
        self._fact_counter = 0
        self._recent_facts = deque(maxlen=1000)

    def generate_facts(
        self,
        detections: List[Detection],
        uav_id: str = "f11_uav",
        timestamp: float = None
    ) -> List[str]:
        """
        Generate TPTP facts from detections.

        Args:
            detections: All detections from perception
            uav_id: UAV identifier for facts
            timestamp: Current timestamp (for temporal facts)

        Returns:
            List of TPTP fact strings
        """
        if timestamp is None:
            timestamp = time.time()

        facts = []
        persons = []

        for det in detections:
            self._fact_counter += 1
            fact_id = self._fact_counter
            object_id = f"{det.class_name}_{fact_id}"

            # Skip if duplicate (same class + similar position recently)
            if self._is_duplicate(det):
                continue

            # 1. Basic observation fact
            if det.position_3d is not None:
                x, y, z = det.position_3d
                facts.append(
                    f"fof(obs_{fact_id}, axiom, "
                    f"observedObject({object_id}, {det.ontology_class}, "
                    f"position({x:.2f}, {y:.2f}, {z:.2f}), "
                    f"confidence({det.confidence:.3f}), "
                    f"time({timestamp:.3f})))."
                )

                # 2. Distance fact
                if det.distance > 0:
                    facts.append(
                        f"fof(dist_{fact_id}, axiom, "
                        f"distanceTo({uav_id}, {object_id}, {det.distance:.2f}))."
                    )

            # 3. Person-specific facts (safety critical)
            if det.class_id == 1:  # Person
                persons.append(det)
                facts.append(
                    f"fof(person_{fact_id}, axiom, "
                    f"(and "
                    f"(instance({object_id}, Person)) "
                    f"(distanceTo({uav_id}, {object_id}, {det.distance:.2f}))))."
                )

                # Check if person is in danger zone
                if det.distance < 15.0:  # minPersonDistance
                    facts.append(
                        f"fof(person_danger_{fact_id}, axiom, "
                        f"personInDangerZone({uav_id}, {object_id}))."
                    )

            # 4. Approaching object facts
            if self.config.include_velocity and det.velocity is not None:
                vel = det.velocity
                speed = np.linalg.norm(vel)

                if speed > 0.5:  # Moving > 0.5 m/s
                    vel_z = vel[2] if len(vel) > 2 else 0.0
                    facts.append(
                        f"fof(vel_{fact_id}, axiom, "
                        f"objectVelocity({object_id}, "
                        f"velocity({vel[0]:.2f}, {vel[1]:.2f}, {vel_z:.2f})))."
                    )

                    # Check if approaching UAV
                    if det.position_3d is not None and det.distance > 0:
                        radial_vel = -np.dot(vel[:2], det.position_3d[:2]) / det.distance
                        if radial_vel > 1.0:  # Approaching at > 1 m/s
                            facts.append(
                                f"fof(approach_{fact_id}, axiom, "
                                f"approachingObject({object_id}, {uav_id}, {radial_vel:.2f}))."
                            )

            # 5. Track confidence facts
            if self.config.include_tracking and det.track_id is not None:
                facts.append(
                    f"fof(track_{fact_id}, axiom, "
                    f"trackedObject({object_id}, track_{det.track_id}))."
                )

        # 6. Crowd density fact
        if len(persons) >= 3:
            facts.append(
                f"fof(crowd_{self._fact_counter}, axiom, "
                f"crowdPresent({uav_id}, {len(persons)}))."
            )

        # Limit facts
        if len(facts) > self.config.max_facts_per_query:
            # Prioritize person-related facts
            person_facts = [f for f in facts if 'person' in f.lower() or 'Person' in f]
            other_facts = [f for f in facts if f not in person_facts]
            facts = person_facts + other_facts[:self.config.max_facts_per_query - len(person_facts)]

        return facts

    def generate_safety_query(
        self,
        detections: List[Detection],
        current_state: Dict,
        proposed_action: np.ndarray,
        uav_id: str = "f11_uav"
    ) -> str:
        """
        Generate complete TPTP safety query for Vampire.

        Args:
            detections: All detections
            current_state: Current UAV state dict
            proposed_action: Proposed action vector
            uav_id: UAV identifier

        Returns:
            Complete TPTP query string
        """
        # Generate perception facts
        perception_facts = self.generate_facts(detections, uav_id)

        # Add state facts
        state_facts = [
            f"fof(state_pos, axiom, "
            f"position({uav_id}, {current_state['x']:.2f}, {current_state['y']:.2f}, {current_state['z']:.2f})).",

            f"fof(state_battery, axiom, "
            f"batteryLevel({uav_id}, {current_state['battery']:.1f})).",

            f"fof(state_geofence, axiom, "
            f"{'inGeofence' if current_state.get('in_geofence', True) else 'outsideGeofence'}({uav_id})).",
        ]

        # Add predicted next state
        vx, vy, vz = proposed_action[0], proposed_action[1], proposed_action[2]
        dt = 0.05  # 20Hz control loop
        next_x = current_state['x'] + vx * dt
        next_y = current_state['y'] + vy * dt
        next_z = current_state['z'] + vz * dt

        state_facts.append(
            f"fof(next_pos, axiom, "
            f"nextPosition({uav_id}, {next_x:.2f}, {next_y:.2f}, {next_z:.2f}))."
        )

        # Safety query conjecture
        conjecture = """
% Query: Is the proposed action safe?
fof(safety_query, conjecture,
    ~(geofenceViolation(f11_uav) |
      noFlyZoneViolation(f11_uav) |
      personInDangerZone(f11_uav, _) |
      highThreatExposure(f11_uav))
).
"""

        # Combine all parts
        query = "% Perception facts\n"
        query += "\n".join(perception_facts)
        query += "\n\n% State facts\n"
        query += "\n".join(state_facts)
        query += "\n\n" + conjecture

        return query

    def _is_duplicate(self, det: Detection) -> bool:
        """Check if detection is duplicate of recent fact."""
        current_time = time.time() * 1000  # ms

        # Create hash of detection
        det_hash = hashlib.md5(
            f"{det.class_id}_{det.bbox[0]:.2f}_{det.bbox[1]:.2f}".encode()
        ).hexdigest()[:16]

        # Clean old entries
        while self._recent_facts and (current_time - self._recent_facts[0][0]) > self.config.dedup_window_ms:
            self._recent_facts.popleft()

        # Check for duplicate
        for _, h in self._recent_facts:
            if h == det_hash:
                return True

        # Add new fact
        self._recent_facts.append((current_time, det_hash))
        return False
