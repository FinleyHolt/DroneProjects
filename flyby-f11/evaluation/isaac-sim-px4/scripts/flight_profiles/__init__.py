"""
Flight Profiles Module

Modular flight test profiles for drone simulation testing.
Each profile is a focused test targeting specific capabilities.

Available profiles:
    - full: Complete ~12 min test (default, backward compatible)
    - detection: 360° pan for YOLO/GT overlay testing (~2-3 min)
    - tracking: Linear flight over vehicle clusters for ByteTrack (~2-3 min)
    - gimbal: Hover + pan/tilt sequences for gimbal testing (~1-2 min)
    - nav: Waypoint navigation patterns (~3-4 min)
    - stress: Aggressive maneuvers for attitude control (~2 min)

Usage:
    from flight_profiles import get_profile, PROFILES

    profile = get_profile("detection")
    results = profile.run(simulation_context)
"""

from .base import TestProfile, ProfileConfig, ProfileResult
from .detection_profile import DetectionProfile
from .tracking_profile import TrackingProfile
from .gimbal_profile import GimbalProfile
from .nav_profile import NavProfile
from .stress_profile import StressProfile
from .full_profile import FullProfile

PROFILES = {
    "full": FullProfile,
    "detection": DetectionProfile,
    "tracking": TrackingProfile,
    "gimbal": GimbalProfile,
    "nav": NavProfile,
    "stress": StressProfile,
}

def get_profile(name: str) -> TestProfile:
    """
    Get a flight profile by name.

    Args:
        name: Profile name (full, detection, tracking, gimbal, nav, stress)

    Returns:
        Instantiated TestProfile

    Raises:
        ValueError: If profile name is unknown
    """
    if name not in PROFILES:
        available = ", ".join(PROFILES.keys())
        raise ValueError(f"Unknown profile '{name}'. Available: {available}")

    return PROFILES[name]()

def list_profiles() -> dict:
    """Return dict of profile names to descriptions."""
    return {
        "full": "Complete ~12 min test (world gen, takeoff, nav, gimbal, stress, RTH, land)",
        "detection": "360° pan for YOLO/GT overlay testing (~2-3 min)",
        "tracking": "Linear flight over vehicle clusters for ByteTrack (~2-3 min)",
        "gimbal": "Hover + pan/tilt sequences (~1-2 min)",
        "nav": "Waypoint navigation patterns (~3-4 min)",
        "stress": "Aggressive maneuvers for attitude control (~2 min)",
    }

__all__ = [
    "TestProfile",
    "ProfileConfig",
    "ProfileResult",
    "DetectionProfile",
    "TrackingProfile",
    "GimbalProfile",
    "NavProfile",
    "StressProfile",
    "FullProfile",
    "PROFILES",
    "get_profile",
    "list_profiles",
]
