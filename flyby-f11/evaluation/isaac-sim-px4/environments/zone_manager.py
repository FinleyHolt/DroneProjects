"""
Zone Manager - NFZ, threat zone, and comms zone tracking.

Single responsibility: Track zone states and check zone violations.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .base_isr_env import (
    NoFlyZone,
    ThreatZone,
    CommsZone,
    ThreatLevel,
    CommsStatus,
    GeofenceConfig,
)


@dataclass
class ZoneState:
    """Current zone state for the UAV."""
    in_geofence: bool = True
    in_nfz: bool = False
    current_nfz_id: Optional[str] = None
    nfz_distance: float = float('inf')
    in_threat_zone: bool = False
    current_threat_level: ThreatLevel = ThreatLevel.NONE
    threat_exposure: float = 0.0
    comms_status: CommsStatus = CommsStatus.OPERATIONAL


class ZoneManager:
    """
    Manages zone tracking and violation detection.

    Responsibilities:
    - Geofence boundary checking
    - NFZ proximity and violation detection
    - Threat zone tracking and exposure accumulation
    - Communications zone status
    """

    def __init__(self, geofence: GeofenceConfig):
        """
        Initialize ZoneManager.

        Args:
            geofence: Geofence configuration
        """
        self.geofence = geofence
        self.nfz_zones: List[NoFlyZone] = []
        self.threat_zones: List[ThreatZone] = []
        self.comms_zones: List[CommsZone] = []

    def add_nfz(self, nfz: NoFlyZone) -> None:
        """Add a no-fly zone."""
        self.nfz_zones.append(nfz)

    def add_threat_zone(self, zone: ThreatZone) -> None:
        """Add a threat zone."""
        self.threat_zones.append(zone)

    def add_comms_zone(self, zone: CommsZone) -> None:
        """Add a communications zone."""
        self.comms_zones.append(zone)

    def clear_zones(self) -> None:
        """Clear all zones."""
        self.nfz_zones.clear()
        self.threat_zones.clear()
        self.comms_zones.clear()

    def update(
        self,
        position: np.ndarray,
        current_exposure: float,
        dt: float,
    ) -> ZoneState:
        """
        Update zone state based on current position.

        Args:
            position: Current UAV position
            current_exposure: Current threat exposure
            dt: Time delta for exposure accumulation

        Returns:
            Updated zone state
        """
        state = ZoneState()

        # Check geofence
        state.in_geofence = self._check_geofence(position)

        # Check NFZs
        nfz_result = self._check_nfz(position)
        state.in_nfz = nfz_result[0]
        state.current_nfz_id = nfz_result[1]
        state.nfz_distance = nfz_result[2]

        # Check threat zones
        threat_result = self._check_threat_zones(position, current_exposure, dt)
        state.in_threat_zone = threat_result[0]
        state.current_threat_level = threat_result[1]
        state.threat_exposure = threat_result[2]

        # Check comms zones
        state.comms_status = self._check_comms(position)

        return state

    def _check_geofence(self, position: np.ndarray) -> bool:
        """Check if position is within geofence."""
        gf = self.geofence
        return (
            gf.min_x <= position[0] <= gf.max_x and
            gf.min_y <= position[1] <= gf.max_y and
            gf.min_z <= position[2] <= gf.max_z
        )

    def _check_nfz(
        self,
        position: np.ndarray,
    ) -> Tuple[bool, Optional[str], float]:
        """
        Check NFZ proximity and violations.

        Returns:
            Tuple of (in_nfz, nfz_id, min_distance)
        """
        min_dist = float('inf')
        in_nfz = False
        current_nfz = None

        for nfz in self.nfz_zones:
            if not nfz.is_active:
                continue

            # Distance to NFZ cylinder
            horizontal_dist = np.linalg.norm(position[:2] - nfz.center[:2])
            dist = horizontal_dist - nfz.radius

            if dist < min_dist:
                min_dist = dist

            # Check if inside NFZ
            if horizontal_dist < nfz.radius:
                z_min = nfz.center[2] - nfz.height / 2
                z_max = nfz.center[2] + nfz.height / 2
                if z_min <= position[2] <= z_max:
                    in_nfz = True
                    current_nfz = nfz.id

        return in_nfz, current_nfz, max(0, min_dist)

    def _check_threat_zones(
        self,
        position: np.ndarray,
        current_exposure: float,
        dt: float,
    ) -> Tuple[bool, ThreatLevel, float]:
        """
        Check threat zone state and accumulate exposure.

        Returns:
            Tuple of (in_threat, threat_level, exposure)
        """
        max_threat = ThreatLevel.NONE
        in_threat = False
        exposure = current_exposure

        for zone in self.threat_zones:
            dist = np.linalg.norm(position[:2] - zone.center[:2])
            if dist < zone.radius:
                in_threat = True
                if zone.threat_level.value > max_threat.value:
                    max_threat = zone.threat_level

                    # Accumulate exposure for medium threats
                    if zone.threat_level == ThreatLevel.MEDIUM:
                        exposure += zone.risk_rate * dt

        return in_threat, max_threat, exposure

    def _check_comms(self, position: np.ndarray) -> CommsStatus:
        """Check communications status based on zones."""
        if not self.comms_zones:
            return CommsStatus.OPERATIONAL

        for zone in self.comms_zones:
            dist = np.linalg.norm(position[:2] - zone.center[:2])
            if dist < zone.radius:
                return zone.status

        # Outside all zones = denied
        return CommsStatus.DENIED

    def get_nearest_nfz(self, position: np.ndarray) -> Optional[NoFlyZone]:
        """Get the nearest active NFZ."""
        nearest = None
        min_dist = float('inf')

        for nfz in self.nfz_zones:
            if not nfz.is_active:
                continue
            dist = np.linalg.norm(position[:2] - nfz.center[:2]) - nfz.radius
            if dist < min_dist:
                min_dist = dist
                nearest = nfz

        return nearest

    def get_escape_vector(self, position: np.ndarray) -> Optional[np.ndarray]:
        """
        Get escape vector from current NFZ.

        Returns:
            Unit vector pointing away from NFZ center, or None if not in NFZ
        """
        for nfz in self.nfz_zones:
            if not nfz.is_active:
                continue

            horizontal_dist = np.linalg.norm(position[:2] - nfz.center[:2])
            if horizontal_dist < nfz.radius:
                # Inside NFZ - compute escape direction
                direction = position[:2] - nfz.center[:2]
                if np.linalg.norm(direction) > 0.01:
                    direction = direction / np.linalg.norm(direction)
                else:
                    # At center - pick random direction
                    direction = np.array([1.0, 0.0])

                return np.array([direction[0], direction[1], 0.0])

        return None
