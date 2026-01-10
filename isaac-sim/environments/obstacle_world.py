"""
Procedural obstacle world generator for navigation testing.

Creates test environments with configurable obstacle layouts:
- Single large obstacles (buildings, towers)
- Random obstacle fields
- Narrow passages
- Dense obstacle clusters

Used by nav_obstacle_test.py for headless simulation testing.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum


class ObstacleType(Enum):
    """Types of obstacles that can be generated."""
    BOX = "box"
    CYLINDER = "cylinder"
    RANDOM_BOX = "random_box"
    BUILDING = "building"


@dataclass
class Obstacle:
    """Single obstacle definition."""
    obstacle_type: ObstacleType
    position: Tuple[float, float, float]  # x, y, z (z is base)
    size: Tuple[float, float, float]  # width, depth, height
    rotation: float = 0.0  # yaw rotation in radians
    color: Tuple[float, float, float] = (0.5, 0.5, 0.5)  # RGB
    name: str = ""


@dataclass
class NavigationScenario:
    """Complete navigation test scenario."""
    name: str
    description: str
    start_position: Tuple[float, float, float]  # x, y, z
    goal_position: Tuple[float, float, float]  # x, y, z
    obstacles: List[Obstacle] = field(default_factory=list)

    # Success criteria
    max_time_seconds: float = 120.0
    min_clearance_meters: float = 5.0
    goal_tolerance_meters: float = 5.0

    # Environment settings
    ground_size: Tuple[float, float] = (1000.0, 1000.0)  # x, y extent


class ObstacleWorldGenerator:
    """
    Generates obstacle worlds for navigation testing.

    Can create predefined scenarios or random layouts.
    """

    def __init__(self, seed: Optional[int] = None):
        """
        Initialize generator.

        Args:
            seed: Random seed for reproducibility
        """
        self.rng = np.random.default_rng(seed)

    def create_scenario(self, scenario_name: str) -> NavigationScenario:
        """
        Create a named scenario.

        Args:
            scenario_name: One of: simple_transit, obstacle_field,
                          narrow_passage, stress_test

        Returns:
            NavigationScenario with obstacles
        """
        scenarios = {
            "simple_transit": self._create_simple_transit,
            "obstacle_field": self._create_obstacle_field,
            "narrow_passage": self._create_narrow_passage,
            "stress_test": self._create_stress_test,
            "building_canyon": self._create_building_canyon,
            "pillar_field": self._create_pillar_field,
        }

        if scenario_name not in scenarios:
            raise ValueError(
                f"Unknown scenario: {scenario_name}. "
                f"Available: {list(scenarios.keys())}"
            )

        return scenarios[scenario_name]()

    def _create_simple_transit(self) -> NavigationScenario:
        """
        Simple A to B with one large obstacle in the middle.

        Tests basic obstacle detection and avoidance.
        """
        return NavigationScenario(
            name="simple_transit",
            description="Single obstacle between start and goal",
            start_position=(0.0, 0.0, 30.0),
            goal_position=(500.0, 0.0, 30.0),
            obstacles=[
                Obstacle(
                    obstacle_type=ObstacleType.BOX,
                    position=(250.0, 0.0, 0.0),
                    size=(30.0, 30.0, 100.0),
                    color=(0.6, 0.3, 0.2),
                    name="building_1",
                ),
            ],
            max_time_seconds=90.0,
            min_clearance_meters=5.0,
        )

    def _create_obstacle_field(self) -> NavigationScenario:
        """
        Random field of 10 tall obstacles.

        Tests navigation through complex environment.
        """
        obstacles = []
        num_obstacles = 10

        # Create obstacles along path with random offsets
        for i in range(num_obstacles):
            # Spread along X axis
            x = 50.0 + i * 45.0 + self.rng.uniform(-10, 10)
            # Random Y offset
            y = self.rng.uniform(-50, 50)

            # Random size
            width = self.rng.uniform(15, 35)
            depth = self.rng.uniform(15, 35)
            height = self.rng.uniform(60, 150)

            obstacles.append(Obstacle(
                obstacle_type=ObstacleType.BOX,
                position=(x, y, 0.0),
                size=(width, depth, height),
                rotation=self.rng.uniform(0, np.pi / 4),
                color=(
                    self.rng.uniform(0.4, 0.7),
                    self.rng.uniform(0.4, 0.7),
                    self.rng.uniform(0.4, 0.7),
                ),
                name=f"obstacle_{i}",
            ))

        return NavigationScenario(
            name="obstacle_field",
            description="Field of 10 random tall obstacles",
            start_position=(0.0, 0.0, 30.0),
            goal_position=(500.0, 0.0, 30.0),
            obstacles=obstacles,
            max_time_seconds=120.0,
            min_clearance_meters=5.0,
        )

    def _create_narrow_passage(self) -> NavigationScenario:
        """
        Two large obstacles forming a narrow gap.

        Tests precise navigation through constrained space.
        """
        gap_width = 20.0  # meters between obstacles

        return NavigationScenario(
            name="narrow_passage",
            description="Two obstacles forming narrow passage",
            start_position=(0.0, 0.0, 30.0),
            goal_position=(400.0, 0.0, 30.0),
            obstacles=[
                # Left obstacle
                Obstacle(
                    obstacle_type=ObstacleType.BOX,
                    position=(200.0, -gap_width / 2 - 25.0, 0.0),
                    size=(50.0, 50.0, 120.0),
                    color=(0.5, 0.4, 0.3),
                    name="wall_left",
                ),
                # Right obstacle
                Obstacle(
                    obstacle_type=ObstacleType.BOX,
                    position=(200.0, gap_width / 2 + 25.0, 0.0),
                    size=(50.0, 50.0, 120.0),
                    color=(0.5, 0.4, 0.3),
                    name="wall_right",
                ),
            ],
            max_time_seconds=90.0,
            min_clearance_meters=3.0,  # Tighter clearance for passage
        )

    def _create_stress_test(self) -> NavigationScenario:
        """
        Dense field of 20+ obstacles.

        Tests performance under high obstacle density.
        """
        obstacles = []
        num_obstacles = 20

        # Create dense obstacle field
        for i in range(num_obstacles):
            x = 40.0 + i * 22.0 + self.rng.uniform(-8, 8)
            y = self.rng.uniform(-60, 60)

            width = self.rng.uniform(10, 25)
            depth = self.rng.uniform(10, 25)
            height = self.rng.uniform(50, 120)

            obstacles.append(Obstacle(
                obstacle_type=ObstacleType.BOX,
                position=(x, y, 0.0),
                size=(width, depth, height),
                rotation=self.rng.uniform(0, np.pi / 2),
                color=(0.5, 0.5, 0.5),
                name=f"stress_obstacle_{i}",
            ))

        return NavigationScenario(
            name="stress_test",
            description="Dense field of 20 obstacles",
            start_position=(0.0, 0.0, 30.0),
            goal_position=(500.0, 0.0, 30.0),
            obstacles=obstacles,
            max_time_seconds=180.0,
            min_clearance_meters=3.0,
        )

    def _create_building_canyon(self) -> NavigationScenario:
        """
        Two rows of tall buildings forming a canyon.

        Tests navigation in urban-like environment.
        """
        obstacles = []

        # Left row of buildings
        for i in range(5):
            x = 100.0 + i * 80.0
            y = -60.0

            obstacles.append(Obstacle(
                obstacle_type=ObstacleType.BOX,
                position=(x, y, 0.0),
                size=(40.0, 40.0, self.rng.uniform(80, 150)),
                color=(0.6, 0.6, 0.65),
                name=f"building_left_{i}",
            ))

        # Right row of buildings
        for i in range(5):
            x = 100.0 + i * 80.0
            y = 60.0

            obstacles.append(Obstacle(
                obstacle_type=ObstacleType.BOX,
                position=(x, y, 0.0),
                size=(40.0, 40.0, self.rng.uniform(80, 150)),
                color=(0.6, 0.6, 0.65),
                name=f"building_right_{i}",
            ))

        return NavigationScenario(
            name="building_canyon",
            description="Urban canyon between two rows of buildings",
            start_position=(0.0, 0.0, 30.0),
            goal_position=(500.0, 0.0, 30.0),
            obstacles=obstacles,
            max_time_seconds=120.0,
            min_clearance_meters=5.0,
        )

    def _create_pillar_field(self) -> NavigationScenario:
        """
        Field of massive pillars requiring weaving navigation.

        Tests precise maneuvering between large cylindrical obstacles.
        8 pillars arranged in 4 rows with varying positions.
        """
        obstacles = []

        # Row 1 (x=60): Single 15m pillar at center
        obstacles.append(Obstacle(
            obstacle_type=ObstacleType.CYLINDER,
            position=(60.0, 0.0, 0.0),
            size=(15.0, 15.0, 180.0),
            color=(0.7, 0.7, 0.75),
            name="pillar_r1_center",
        ))

        # Row 2 (x=120): 12m at y=-30, 18m at y=+35
        obstacles.append(Obstacle(
            obstacle_type=ObstacleType.CYLINDER,
            position=(120.0, -30.0, 0.0),
            size=(12.0, 12.0, 160.0),
            color=(0.65, 0.65, 0.7),
            name="pillar_r2_left",
        ))
        obstacles.append(Obstacle(
            obstacle_type=ObstacleType.CYLINDER,
            position=(120.0, 35.0, 0.0),
            size=(18.0, 18.0, 200.0),
            color=(0.65, 0.65, 0.7),
            name="pillar_r2_right",
        ))

        # Row 3 (x=180): 10m at y=-15, 14m at y=+20 (narrow passage)
        obstacles.append(Obstacle(
            obstacle_type=ObstacleType.CYLINDER,
            position=(180.0, -15.0, 0.0),
            size=(10.0, 10.0, 150.0),
            color=(0.6, 0.6, 0.65),
            name="pillar_r3_left",
        ))
        obstacles.append(Obstacle(
            obstacle_type=ObstacleType.CYLINDER,
            position=(180.0, 20.0, 0.0),
            size=(14.0, 14.0, 170.0),
            color=(0.6, 0.6, 0.65),
            name="pillar_r3_right",
        ))

        # Row 4 (x=240): 16m at center, 12m at y=-40, 10m at y=+45
        obstacles.append(Obstacle(
            obstacle_type=ObstacleType.CYLINDER,
            position=(240.0, 0.0, 0.0),
            size=(16.0, 16.0, 190.0),
            color=(0.55, 0.55, 0.6),
            name="pillar_r4_center",
        ))
        obstacles.append(Obstacle(
            obstacle_type=ObstacleType.CYLINDER,
            position=(240.0, -40.0, 0.0),
            size=(12.0, 12.0, 140.0),
            color=(0.55, 0.55, 0.6),
            name="pillar_r4_left",
        ))
        obstacles.append(Obstacle(
            obstacle_type=ObstacleType.CYLINDER,
            position=(240.0, 45.0, 0.0),
            size=(10.0, 10.0, 145.0),
            color=(0.55, 0.55, 0.6),
            name="pillar_r4_right",
        ))

        return NavigationScenario(
            name="pillar_field",
            description="8 massive pillars requiring weaving navigation",
            start_position=(0.0, 0.0, 30.0),
            goal_position=(300.0, 0.0, 30.0),
            obstacles=obstacles,
            max_time_seconds=120.0,
            min_clearance_meters=5.0,
        )

    def create_random_scenario(
        self,
        num_obstacles: int = 10,
        area_size: float = 500.0,
        obstacle_height_range: Tuple[float, float] = (50.0, 150.0),
        obstacle_width_range: Tuple[float, float] = (15.0, 40.0),
    ) -> NavigationScenario:
        """
        Create a randomized scenario.

        Args:
            num_obstacles: Number of obstacles to generate
            area_size: Size of the navigation area (meters)
            obstacle_height_range: Min/max obstacle height
            obstacle_width_range: Min/max obstacle width

        Returns:
            Randomized NavigationScenario
        """
        obstacles = []

        for i in range(num_obstacles):
            # Random position (avoid start/goal areas)
            x = self.rng.uniform(area_size * 0.1, area_size * 0.9)
            y = self.rng.uniform(-area_size * 0.3, area_size * 0.3)

            # Random size
            width = self.rng.uniform(*obstacle_width_range)
            depth = self.rng.uniform(*obstacle_width_range)
            height = self.rng.uniform(*obstacle_height_range)

            obstacles.append(Obstacle(
                obstacle_type=ObstacleType.BOX,
                position=(x, y, 0.0),
                size=(width, depth, height),
                rotation=self.rng.uniform(0, np.pi),
                color=(
                    self.rng.uniform(0.3, 0.7),
                    self.rng.uniform(0.3, 0.7),
                    self.rng.uniform(0.3, 0.7),
                ),
                name=f"random_obstacle_{i}",
            ))

        return NavigationScenario(
            name="random",
            description=f"Random scenario with {num_obstacles} obstacles",
            start_position=(0.0, 0.0, 30.0),
            goal_position=(area_size, 0.0, 30.0),
            obstacles=obstacles,
            max_time_seconds=180.0,
            min_clearance_meters=5.0,
            ground_size=(area_size * 1.2, area_size * 0.8),
        )


def spawn_obstacles_isaac_sim(scenario: NavigationScenario, stage) -> List:
    """
    Spawn obstacles in Isaac Sim.

    Args:
        scenario: NavigationScenario with obstacles
        stage: USD stage to spawn into

    Returns:
        List of spawned prim paths
    """
    from pxr import UsdGeom, Gf
    import omni.isaac.core.utils.prims as prim_utils

    prim_paths = []

    for obstacle in scenario.obstacles:
        # Create unique prim path
        prim_path = f"/World/Obstacles/{obstacle.name or f'obstacle_{len(prim_paths)}'}"

        # Create box primitive
        if obstacle.obstacle_type in [ObstacleType.BOX, ObstacleType.RANDOM_BOX, ObstacleType.BUILDING]:
            cube_prim = prim_utils.create_prim(
                prim_path=prim_path,
                prim_type="Cube",
                position=np.array([
                    obstacle.position[0],
                    obstacle.position[1],
                    obstacle.position[2] + obstacle.size[2] / 2,  # Center Z
                ]),
                scale=np.array([
                    obstacle.size[0] / 2,  # Cube scale is half-extent
                    obstacle.size[1] / 2,
                    obstacle.size[2] / 2,
                ]),
                orientation=np.array([
                    np.cos(obstacle.rotation / 2),
                    0, 0,
                    np.sin(obstacle.rotation / 2),
                ]),
            )

        elif obstacle.obstacle_type == ObstacleType.CYLINDER:
            cylinder_prim = prim_utils.create_prim(
                prim_path=prim_path,
                prim_type="Cylinder",
                position=np.array([
                    obstacle.position[0],
                    obstacle.position[1],
                    obstacle.position[2] + obstacle.size[2] / 2,
                ]),
                scale=np.array([
                    obstacle.size[0] / 2,
                    obstacle.size[1] / 2,
                    obstacle.size[2] / 2,
                ]),
            )

        # Set color/material
        # (Material application would go here for production use)

        prim_paths.append(prim_path)

    return prim_paths


def create_ground_plane(
    scenario: NavigationScenario,
    stage,
    ground_height: float = 0.0,
) -> str:
    """
    Create ground plane for the scenario.

    Args:
        scenario: NavigationScenario for size reference
        stage: USD stage
        ground_height: Z position of ground

    Returns:
        Prim path of ground plane
    """
    import omni.isaac.core.utils.prims as prim_utils

    prim_path = "/World/Ground"

    ground_prim = prim_utils.create_prim(
        prim_path=prim_path,
        prim_type="Plane",
        position=np.array([
            scenario.ground_size[0] / 2,
            0.0,
            ground_height,
        ]),
        scale=np.array([
            scenario.ground_size[0],
            scenario.ground_size[1],
            1.0,
        ]),
    )

    return prim_path
