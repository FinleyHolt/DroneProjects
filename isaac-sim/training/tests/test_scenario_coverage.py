#!/usr/bin/env python3
"""
Test suite for Phase 1: Scenario Infrastructure and Coverage Tracking

Tests:
1. ScenarioType enum and config dataclasses
2. Coverage grid initialization and update
3. Mission state tracking (battery, comms, time)
4. New observation components
5. World bounds configuration

Run with:
    pytest training/tests/test_scenario_coverage.py -v

Note: Full environment tests require Isaac Sim and should be run via:
    ./scripts/run_phase1_test.sh
"""

import pytest
import torch
import numpy as np


class TestScenarioConfigs:
    """Test scenario configuration dataclasses."""

    def test_scenario_type_enum(self):
        """Test ScenarioType enum values."""
        from training.environments import ScenarioType

        assert ScenarioType.AREA_COVERAGE.value == "area_coverage"
        assert ScenarioType.NFZ_AVOIDANCE.value == "nfz_avoidance"
        assert ScenarioType.MULTI_OBJECTIVE.value == "multi_objective"

    def test_world_config_defaults(self):
        """Test WorldConfig default values."""
        from training.environments import WorldConfig

        cfg = WorldConfig()

        # Default 500m x 500m world
        assert cfg.bounds[0] == -250.0  # x_min
        assert cfg.bounds[1] == 250.0   # x_max
        assert cfg.bounds[2] == -250.0  # y_min
        assert cfg.bounds[3] == 250.0   # y_max
        assert cfg.bounds[4] == 0.0     # z_min
        assert cfg.bounds[5] == 120.0   # z_max

        # 10m coverage resolution
        assert cfg.coverage_resolution == 10.0

        # Launch at origin
        assert cfg.launch_position == (0.0, 0.0, 0.0)

    def test_world_config_custom(self):
        """Test WorldConfig with custom values."""
        from training.environments import WorldConfig

        cfg = WorldConfig(
            bounds=(-500, 3500, -500, 1500, 0, 150),  # 4km corridor
            coverage_resolution=20.0,
            launch_position=(100.0, 100.0, 0.0),
        )

        assert cfg.bounds[1] - cfg.bounds[0] == 4000  # 4km x-range
        assert cfg.coverage_resolution == 20.0
        assert cfg.launch_position[0] == 100.0

    def test_mission_config_defaults(self):
        """Test MissionConfig default values."""
        from training.environments import MissionConfig, ScenarioType

        cfg = MissionConfig()

        assert cfg.scenario == ScenarioType.AREA_COVERAGE
        assert cfg.max_episode_steps == 37500  # ~5 minutes
        assert cfg.target_coverage_pct == 85.0
        assert cfg.battery_reserve_pct == 25.0

    def test_nfz_config_defaults(self):
        """Test NFZConfig default values."""
        from training.environments import NFZConfig

        cfg = NFZConfig()

        assert cfg.enabled is False
        assert cfg.max_nfz == 10
        assert cfg.nfz_violation_reward == -100.0
        assert cfg.nfz_buffer_distance == 50.0

    def test_threat_config_defaults(self):
        """Test ThreatConfig default values."""
        from training.environments import ThreatConfig

        cfg = ThreatConfig()

        assert cfg.enabled is False
        assert cfg.max_threat_zones == 5
        assert cfg.max_medium_exposure == 30.0
        assert cfg.high_threat_terminates is True

    def test_isr_training_config_with_scenario(self):
        """Test ISRTrainingConfig with scenario configs."""
        from training.environments import (
            ISRTrainingConfig,
            WorldConfig,
            MissionConfig,
            NFZConfig,
            ScenarioType,
        )

        # Create config for NFZ avoidance scenario
        cfg = ISRTrainingConfig(
            num_envs=64,
            world=WorldConfig(bounds=(-500, 3500, -500, 1500, 0, 150)),
            mission=MissionConfig(
                scenario=ScenarioType.NFZ_AVOIDANCE,
                max_episode_steps=37500,
            ),
            nfz=NFZConfig(enabled=True, max_nfz=5),
        )

        assert cfg.num_envs == 64
        assert cfg.world.bounds[1] == 3500
        assert cfg.mission.scenario == ScenarioType.NFZ_AVOIDANCE
        assert cfg.nfz.enabled is True

    def test_backwards_compatibility(self):
        """Test backwards compatibility properties."""
        from training.environments import ISRTrainingConfig, WorldConfig, MissionConfig

        cfg = ISRTrainingConfig(
            world=WorldConfig(bounds=(-100, 100, -100, 100, 0, 80)),
            mission=MissionConfig(max_episode_steps=5000),
        )

        # world_bounds property should return world.bounds
        assert cfg.world_bounds == (-100, 100, -100, 100, 0, 80)

        # max_episode_steps property should return mission.max_episode_steps
        assert cfg.max_episode_steps == 5000


class TestObservationDimensions:
    """Test observation space dimensions."""

    def test_base_observation_dim(self):
        """Test base observation dimension calculation."""
        from training.environments import ISRTrainingConfig

        cfg = ISRTrainingConfig(num_envs=4)

        # Base: 13 (drone) + 2 (gimbal) + 16*4 (targets) + 6 (mission) = 85
        expected_dim = 21 + cfg.targets.max_targets * 4
        assert expected_dim == 85

    def test_observation_dim_with_nfz(self):
        """Test observation dimension with NFZ enabled."""
        from training.environments import ISRTrainingConfig, NFZConfig

        cfg = ISRTrainingConfig(
            num_envs=4,
            nfz=NFZConfig(enabled=True),
        )

        # Base (85) + NFZ (7) = 92
        expected_base = 21 + cfg.targets.max_targets * 4
        expected_with_nfz = expected_base + 7
        assert expected_with_nfz == 92

    def test_observation_dim_with_threat(self):
        """Test observation dimension with threat zones enabled."""
        from training.environments import ISRTrainingConfig, ThreatConfig

        cfg = ISRTrainingConfig(
            num_envs=4,
            threat=ThreatConfig(enabled=True),
        )

        # Base (85) + Threat (6) = 91
        expected_base = 21 + cfg.targets.max_targets * 4
        expected_with_threat = expected_base + 6
        assert expected_with_threat == 91

    def test_observation_dim_full(self):
        """Test observation dimension with all features enabled."""
        from training.environments import ISRTrainingConfig, NFZConfig, ThreatConfig

        cfg = ISRTrainingConfig(
            num_envs=4,
            nfz=NFZConfig(enabled=True),
            threat=ThreatConfig(enabled=True),
        )

        # Base (85) + NFZ (7) + Threat (6) = 98
        expected_base = 21 + cfg.targets.max_targets * 4
        expected_full = expected_base + 7 + 6
        assert expected_full == 98


class TestCoverageGridLogic:
    """Test coverage grid calculations (without Isaac Sim)."""

    def test_coverage_grid_size_calculation(self):
        """Test coverage grid size based on world bounds and resolution."""
        from training.environments import WorldConfig

        # 500m x 500m world with 10m resolution = 50x50 grid
        cfg = WorldConfig(
            bounds=(-250, 250, -250, 250, 0, 120),
            coverage_resolution=10.0,
        )

        grid_x = int((cfg.bounds[1] - cfg.bounds[0]) / cfg.coverage_resolution)
        grid_y = int((cfg.bounds[3] - cfg.bounds[2]) / cfg.coverage_resolution)

        assert grid_x == 50
        assert grid_y == 50

    def test_coverage_grid_size_large_world(self):
        """Test coverage grid for 4km corridor."""
        from training.environments import WorldConfig

        # 4km x 2km world with 20m resolution
        cfg = WorldConfig(
            bounds=(-500, 3500, -500, 1500, 0, 150),
            coverage_resolution=20.0,
        )

        grid_x = int((cfg.bounds[1] - cfg.bounds[0]) / cfg.coverage_resolution)
        grid_y = int((cfg.bounds[3] - cfg.bounds[2]) / cfg.coverage_resolution)

        assert grid_x == 200  # 4000m / 20m
        assert grid_y == 100  # 2000m / 20m

    def test_position_to_grid_index(self):
        """Test converting world position to grid index."""
        from training.environments import WorldConfig

        cfg = WorldConfig(
            bounds=(-250, 250, -250, 250, 0, 120),
            coverage_resolution=10.0,
        )

        # Position at origin should map to center of grid
        pos_x, pos_y = 0.0, 0.0
        grid_x = int((pos_x - cfg.bounds[0]) / cfg.coverage_resolution)
        grid_y = int((pos_y - cfg.bounds[2]) / cfg.coverage_resolution)

        assert grid_x == 25  # Center of 50-cell grid
        assert grid_y == 25

        # Position at corner
        pos_x, pos_y = -250.0, -250.0
        grid_x = int((pos_x - cfg.bounds[0]) / cfg.coverage_resolution)
        grid_y = int((pos_y - cfg.bounds[2]) / cfg.coverage_resolution)

        assert grid_x == 0
        assert grid_y == 0

    def test_coverage_percentage_calculation(self):
        """Test coverage percentage from grid."""
        device = torch.device("cpu")

        # 10x10 grid, 25 cells covered = 25%
        grid = torch.zeros((1, 10, 10), dtype=torch.bool, device=device)
        grid[0, :5, :5] = True  # Cover 5x5 = 25 cells

        total_cells = 100
        covered_cells = grid.float().sum(dim=(1, 2))
        coverage_pct = (covered_cells / total_cells) * 100.0

        assert coverage_pct[0].item() == 25.0


class TestMissionStateLogic:
    """Test mission state calculations (without Isaac Sim)."""

    def test_battery_drain(self):
        """Test battery drain calculation."""
        from training.environments import MissionConfig

        cfg = MissionConfig(
            initial_battery_pct=100.0,
            battery_drain_rate=0.001,  # 0.1% per step
        )

        battery = cfg.initial_battery_pct
        steps = 1000

        for _ in range(steps):
            battery -= cfg.battery_drain_rate

        # After 1000 steps at 0.001 drain rate = 1% drained
        assert abs(battery - 99.0) < 0.01

    def test_mission_time_calculation(self):
        """Test mission time from step count."""
        physics_dt = 1.0 / 250.0
        decimation = 2
        step_time = physics_dt * decimation  # 8ms per step

        steps = 37500  # ~5 minutes worth
        mission_time = steps * step_time

        # Should be approximately 300 seconds (5 minutes)
        assert abs(mission_time - 300.0) < 1.0


class TestNFZLogic:
    """Test NFZ calculations (without Isaac Sim)."""

    def test_nfz_distance_calculation(self):
        """Test distance to NFZ edge calculation."""
        device = torch.device("cpu")

        # NFZ at (100, 100) with radius 50
        nfz_center = torch.tensor([[100.0, 100.0]], device=device)
        nfz_radius = torch.tensor([50.0], device=device)

        # Drone at (120, 100) - should be 30m from edge (20m from center, 50m radius)
        drone_pos = torch.tensor([[120.0, 100.0]], device=device)

        dist_to_center = torch.norm(drone_pos - nfz_center, dim=-1)
        dist_to_edge = dist_to_center - nfz_radius

        assert abs(dist_to_edge[0].item() - (-30.0)) < 0.01  # Inside by 30m

    def test_nfz_inside_check(self):
        """Test checking if drone is inside NFZ."""
        device = torch.device("cpu")

        nfz_center = torch.tensor([[100.0, 100.0]], device=device)
        nfz_radius = torch.tensor([50.0], device=device)

        # Inside NFZ
        drone_inside = torch.tensor([[110.0, 110.0]], device=device)
        dist = torch.norm(drone_inside - nfz_center, dim=-1)
        in_nfz = dist < nfz_radius
        assert in_nfz[0].item() is True

        # Outside NFZ
        drone_outside = torch.tensor([[200.0, 200.0]], device=device)
        dist = torch.norm(drone_outside - nfz_center, dim=-1)
        in_nfz = dist < nfz_radius
        assert in_nfz[0].item() is False


class TestThreatLogic:
    """Test threat zone calculations (without Isaac Sim)."""

    def test_threat_level_max(self):
        """Test getting maximum threat level when in multiple zones."""
        device = torch.device("cpu")

        # Two threat zones
        threat_centers = torch.tensor([[[50.0, 50.0], [100.0, 100.0]]], device=device)
        threat_radii = torch.tensor([[30.0, 40.0]], device=device)
        threat_levels = torch.tensor([[2, 3]], dtype=torch.int32, device=device)  # MEDIUM, HIGH

        # Drone at (60, 60) - inside first zone only
        drone_pos = torch.tensor([[60.0, 60.0]], device=device)

        dist = torch.norm(drone_pos.unsqueeze(1) - threat_centers, dim=-1)
        in_threat = dist < threat_radii

        masked_levels = torch.where(in_threat, threat_levels, torch.zeros_like(threat_levels))
        max_threat = masked_levels.max(dim=-1).values

        assert max_threat[0].item() == 2  # MEDIUM

    def test_threat_exposure_accumulation(self):
        """Test threat exposure time accumulation."""
        dt = 0.008  # 8ms per step
        exposure = 0.0
        steps_in_medium = 1000

        for _ in range(steps_in_medium):
            exposure += dt

        # 1000 steps * 8ms = 8 seconds
        assert abs(exposure - 8.0) < 0.01


class TestMissionTaskingIntegration:
    """Test mission tasking integration with training environment."""

    def test_area_coverage_tasking_creation(self):
        """Test that area coverage tasking can be created."""
        try:
            from environments.mission_tasking import (
                create_comms_denied_tasking,
            )
        except ImportError:
            pytest.skip("mission_tasking module not available")

        # We reuse the comms_denied factory for area coverage since
        # the core functionality (NAI-based target spawning) is the same
        tasking = create_comms_denied_tasking(
            surveillance_center=(100.0, 100.0),
            surveillance_radius=150.0,
            seed=42,
        )

        assert len(tasking.nais) >= 1
        assert tasking.nais[0].center[0] == 100.0
        assert tasking.nais[0].radius == 150.0

    def test_poi_generation_from_tasking(self):
        """Test POI generation clusters targets around NAIs."""
        try:
            from environments.mission_tasking import (
                create_comms_denied_tasking,
                generate_poi_distribution_from_tasking,
            )
        except ImportError:
            pytest.skip("mission_tasking module not available")

        tasking = create_comms_denied_tasking(
            surveillance_center=(200.0, 200.0),
            surveillance_radius=100.0,
            seed=42,
        )

        pois = generate_poi_distribution_from_tasking(tasking, seed=42)

        # Should generate some POIs
        assert len(pois) > 0

        # Most POIs should be near the NAI center (within 1.2x radius)
        nai_center = np.array([200.0, 200.0])
        near_center_count = 0
        for poi in pois:
            pos = np.array(poi["position"])
            dist = np.linalg.norm(pos - nai_center)
            if dist < 100.0 * 1.2:  # Within 1.2x radius
                near_center_count += 1

        # At least 70% should be near center
        assert near_center_count / len(pois) >= 0.7

    def test_dynamic_nfz_tasking_creation(self):
        """Test dynamic NFZ tasking has restricted zones."""
        try:
            from environments.mission_tasking import create_dynamic_nfz_tasking
        except ImportError:
            pytest.skip("mission_tasking module not available")

        tasking = create_dynamic_nfz_tasking(
            destination=(3000.0, 500.0, 80.0),
            seed=42,
        )

        assert tasking.mission_name == "Dynamic NFZ Transit"
        assert len(tasking.restricted_zones) >= 1

        # First zone should be active from start
        assert tasking.restricted_zones[0].activation_time == 0.0

        # Later zones should activate mid-mission
        if len(tasking.restricted_zones) > 1:
            assert tasking.restricted_zones[1].activation_time > 0.0

    def test_multi_objective_tasking_creation(self):
        """Test multi-objective tasking has varying priorities."""
        try:
            from environments.mission_tasking import (
                create_multi_objective_tasking,
                NAIPriority,
            )
        except ImportError:
            pytest.skip("mission_tasking module not available")

        tasking = create_multi_objective_tasking(seed=42)

        assert tasking.mission_name == "Multi-Objective ISR"
        assert len(tasking.nais) >= 2

        # Should have NAIs with different priorities
        priorities = [nai.priority for nai in tasking.nais]
        assert len(set(priorities)) > 1  # Multiple different priorities

        # Should have soft-constraint threat zones
        assert len(tasking.restricted_zones) >= 1
        has_soft_constraint = any(
            not rz.is_hard_constraint for rz in tasking.restricted_zones
        )
        assert has_soft_constraint

    def test_priority_value_mapping(self):
        """Test that higher priority NAIs give higher target values."""
        try:
            from environments.mission_tasking import NAIPriority
        except ImportError:
            pytest.skip("mission_tasking module not available")

        # Priority value mapping from the implementation
        value_mult = {1: 4.0, 2: 2.0, 3: 1.5, 4: 1.0}

        # CRITICAL (1) should give 4x base value
        assert value_mult[NAIPriority.CRITICAL.value] == 4.0

        # HIGH (2) should give 2x base value
        assert value_mult[NAIPriority.HIGH.value] == 2.0

        # MEDIUM (3) should give 1.5x base value
        assert value_mult[NAIPriority.MEDIUM.value] == 1.5

        # LOW (4) should give 1x base value
        assert value_mult[NAIPriority.LOW.value] == 1.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
