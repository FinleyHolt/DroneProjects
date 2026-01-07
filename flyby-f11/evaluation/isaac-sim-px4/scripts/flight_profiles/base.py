"""
Base Test Profile Classes

Defines the abstract base class and common data structures for flight test profiles.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any, Optional, Tuple
import numpy as np
import time


@dataclass
class ProfileConfig:
    """Configuration for a test profile."""

    # World generation
    terrain_size: Tuple[float, float] = (500.0, 500.0)
    tree_density: float = 0.15
    vehicle_clusters: int = 10
    vehicles_per_cluster: int = 6
    people_per_cluster: int = 4

    # Flight parameters
    takeoff_altitude: float = 50.0
    cruise_speed: float = 3.0
    position_tolerance: float = 2.0

    # Camera/gimbal
    camera_fov: float = 90.0
    default_gimbal_tilt: float = -30.0

    # Output
    video_suffix: str = ""  # Appended to output filename

    def __post_init__(self):
        """Validate config."""
        if self.takeoff_altitude < 10.0:
            raise ValueError("Takeoff altitude must be >= 10m")
        if self.terrain_size[0] < 100.0 or self.terrain_size[1] < 100.0:
            raise ValueError("Terrain size must be >= 100m")


@dataclass
class TestResult:
    """Result of a single test phase."""

    name: str
    passed: bool
    message: str = ""
    duration_seconds: float = 0.0
    data: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ProfileResult:
    """Complete result from running a profile."""

    profile_name: str
    started_at: str
    duration_seconds: float
    tests: List[TestResult]
    frames_recorded: int = 0
    total_detections: int = 0
    tracking_stats: Dict[str, Any] = field(default_factory=dict)

    @property
    def all_passed(self) -> bool:
        return all(t.passed for t in self.tests)

    @property
    def passed_count(self) -> int:
        return sum(1 for t in self.tests if t.passed)

    @property
    def failed_count(self) -> int:
        return sum(1 for t in self.tests if not t.passed)

    def to_dict(self) -> dict:
        return {
            "profile_name": self.profile_name,
            "started_at": self.started_at,
            "duration_seconds": self.duration_seconds,
            "tests_passed": self.passed_count,
            "tests_total": len(self.tests),
            "all_passed": self.all_passed,
            "frames_recorded": self.frames_recorded,
            "total_detections": self.total_detections,
            "tracking_stats": self.tracking_stats,
            "tests": [asdict(t) for t in self.tests],
        }


class SimulationContext:
    """
    Holds all initialized simulation components.

    This is passed to profiles so they don't need to handle initialization.
    """

    def __init__(self):
        self.simulation_app = None
        self.world = None
        self.stage = None
        self.timeline = None
        self.vehicle = None
        self.camera = None
        self.gimbal = None
        self.flight_controller = None
        self.video_recorder = None
        self.world_generator = None
        self.perception_manager = None

        # Output paths
        self.output_dir = "/workspace/output"
        self.video_path = None
        self.report_path = None

        # Config
        self.headless = True
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fps = 30


class TestProfile(ABC):
    """
    Abstract base class for flight test profiles.

    Each profile defines:
    - A configuration for world generation
    - A flight sequence to execute
    - What tests to record/validate
    """

    # Profile metadata
    name: str = "base"
    description: str = "Base profile (do not use directly)"
    estimated_duration: str = "unknown"

    def __init__(self, config: Optional[ProfileConfig] = None):
        """
        Initialize profile with optional custom config.

        Args:
            config: Custom profile configuration. If None, uses default_config().
        """
        self.config = config or self.default_config()
        self.results: List[TestResult] = []
        self._start_time = None
        self._current_phase = "Initializing"

    @abstractmethod
    def default_config(self) -> ProfileConfig:
        """Return the default configuration for this profile."""
        pass

    @abstractmethod
    def run_flight(self, ctx: SimulationContext) -> List[TestResult]:
        """
        Execute the flight test sequence.

        This is the main entry point called after simulation is initialized.
        Should return list of TestResult for each phase of the test.

        Args:
            ctx: Initialized simulation context with all components ready

        Returns:
            List of TestResult objects
        """
        pass

    def get_world_config(self) -> dict:
        """
        Return WorldConfig parameters for this profile.

        Profiles can override this to customize world generation.
        Returns dict that will be passed to WorldConfig.
        """
        return {
            "terrain_size": self.config.terrain_size,
            "terrain_roughness": 2.0,
            "terrain_material": "forest_floor",
            "tree_density": self.config.tree_density,
            "tree_proportions": {"Birch": 30, "Spruce": 40, "Pine": 30},
            "undergrowth_density": 0.0,
            "randomize_lighting": False,
            "time_of_day": "noon",
            "weather": "clear",
            "seed": None,  # Random seed for variety
        }

    def get_vehicle_clusters(self) -> List[Tuple[float, float]]:
        """
        Return list of (x, y) positions for vehicle cluster spawning.

        Profiles can override to customize target placement.
        """
        clusters = []
        n = self.config.vehicle_clusters

        if n <= 0:
            return clusters

        # Distribute clusters in concentric rings
        # Inner ring at 100m, outer at terrain_size/3
        terrain_radius = min(self.config.terrain_size) / 3

        if n >= 1:
            clusters.append((0.0, 0.0))  # Center

        if n >= 5:
            # Cardinal directions at half radius
            r = terrain_radius / 2
            clusters.extend([
                (r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)
            ])

        if n >= 9:
            # Diagonals at half radius
            r = terrain_radius / 2 * 0.707
            clusters.extend([
                (r, r), (-r, r), (-r, -r), (r, -r)
            ])

        if n >= 13:
            # Outer ring cardinal
            r = terrain_radius
            clusters.extend([
                (r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)
            ])

        return clusters[:n]

    def get_output_suffix(self) -> str:
        """Return suffix for output files (e.g., '_detection' for detection profile)."""
        if self.config.video_suffix:
            return self.config.video_suffix
        return f"_{self.name}" if self.name != "full" else ""

    def set_phase(self, phase: str):
        """Set current test phase for overlay display."""
        self._current_phase = phase

    def get_phase(self) -> str:
        """Get current test phase."""
        return self._current_phase

    def add_result(self, result: TestResult):
        """Add a test result."""
        self.results.append(result)

    def create_result(
        self,
        name: str,
        passed: bool,
        message: str = "",
        start_time: float = None,
        data: dict = None,
    ) -> TestResult:
        """Helper to create and add a test result."""
        duration = time.time() - start_time if start_time else 0.0
        result = TestResult(
            name=name,
            passed=passed,
            message=message,
            duration_seconds=duration,
            data=data or {},
        )
        self.add_result(result)
        return result

    def __str__(self):
        return f"{self.__class__.__name__}(name={self.name})"
