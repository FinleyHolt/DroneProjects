#!/usr/bin/env python3
"""
Search Policy Training Environment

Specialized environment for training the SearchPolicy used by the BT node.
Focused observation space (40 dims) and action space (4 dims) optimized
for area coverage and target detection.

Observation Space (40 dims):
- Drone state (12): pos(3), vel(3), quat(4), ang_vel(3) - using rotation matrix
- Coverage map features (15): coverage percentage, frontier centroids, uncovered area stats
- Mission state (3): battery, time, coverage_pct
- Context info (10): home direction, targets detected, environmental context

Action Space (4 dims):
- Forward velocity [-1, 1] -> [-max_vel, +max_vel] m/s
- Lateral velocity [-1, 1] -> [-max_vel, +max_vel] m/s
- Vertical velocity [-1, 1] -> [-max_vel_z, +max_vel_z] m/s
- Heading rate [-1, 1] -> [-max_yaw_rate, +max_yaw_rate] rad/s

Key Differences from ISRTrainingEnv:
- No gimbal control (fixed downward-looking camera during search)
- Coverage-focused observations (frontier points, coverage grid features)
- Reward optimized for coverage efficiency and target detection
- Fixed camera angle during search (gimbal control happens in DwellPolicy)

Author: Finley Holt
"""

import torch
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any

from .isr_training_env import (
    ISRTrainingEnv,
    ISRTrainingConfig,
    CameraConfig,
    TargetConfig,
    DroneConfig,
    WorldConfig,
    MissionConfig,
    RewardConfig,
    ScenarioType,
)


@dataclass
class SearchRewardConfig:
    """Reward configuration optimized for search behavior."""
    # Coverage rewards
    coverage_progress: float = 2.0           # Per % coverage increase
    coverage_completion_bonus: float = 100.0 # Bonus for reaching coverage target

    # Target detection
    target_detection: float = 15.0           # Per new target detected
    target_revisit_penalty: float = -0.5     # Penalty for revisiting detected targets

    # Efficiency
    time_penalty: float = -0.001             # Per step (encourages efficiency)
    battery_efficiency: float = 0.1          # Reward for coverage/battery ratio

    # Flight quality
    smooth_action_penalty: float = 0.02      # Action smoothness regularization
    altitude_deviation_penalty: float = 0.01 # Penalty per meter from optimal altitude
    optimal_altitude: float = 50.0           # Optimal search altitude

    # Frontier rewards (encourage exploring uncovered areas)
    frontier_progress: float = 0.5           # Reward for moving toward frontier

    # Mission outcomes
    mission_success: float = 100.0
    crash_penalty: float = -100.0
    battery_critical: float = -50.0


@dataclass
class SearchTrainingConfig:
    """Configuration for Search Policy training environment."""
    # Parallelization
    num_envs: int = 1024
    device: str = "cuda:0"

    # Physics
    physics_dt: float = 1.0 / 250.0
    decimation: int = 2

    # Drone (velocity limits for search - slightly slower for better coverage)
    max_horizontal_vel: float = 8.0      # m/s
    max_vertical_vel: float = 3.0        # m/s
    max_yaw_rate: float = 0.5            # rad/s

    # Search-specific settings
    search_altitude: float = 50.0        # Fixed search altitude
    coverage_target_pct: float = 85.0    # Coverage target
    coverage_grid_resolution: float = 10.0  # Meters per grid cell

    # Frontier detection
    num_frontier_samples: int = 5        # Number of frontier points in observation

    # Episode limits
    max_episode_steps: int = 20000       # ~3 minutes at 125Hz effective

    # Rewards
    rewards: SearchRewardConfig = field(default_factory=SearchRewardConfig)

    # Spawn configuration
    spawn_height_range: Tuple[float, float] = (40.0, 60.0)
    spawn_area_radius: float = 50.0

    # World bounds
    world_bounds: Tuple[float, ...] = (-200.0, 200.0, -200.0, 200.0, 20.0, 100.0)

    # Target spawning
    num_targets: int = 8

    # Domain randomization
    mass_randomization: float = 0.03     # ±3% mass variation
    drag_randomization: float = 0.05     # ±5% drag variation
    wind_enabled: bool = True
    wind_speed_range: Tuple[float, float] = (0.0, 3.0)

    # Prim path prefix (allows multiple envs in same stage)
    prim_path_prefix: str = "Train"


class SearchTrainingEnv:
    """
    Specialized training environment for SearchPolicy.

    Simplified from ISRTrainingEnv to focus on:
    - Area coverage optimization
    - Target detection during search
    - Efficient flight patterns

    Key simplifications:
    - No gimbal control (fixed camera)
    - Coverage-focused observation space
    - 4-DOF action space (velocity + yaw rate)
    """

    # Class-level observation/action dimensions
    OBS_DIM = 40
    ACTION_DIM = 4

    def __init__(self, config: SearchTrainingConfig = None):
        self.cfg = config or SearchTrainingConfig()
        self.device = torch.device(self.cfg.device)
        self.num_envs = self.cfg.num_envs

        self._initialized = False

        # Will be set during setup
        self.simulation_app = None
        self.world = None
        self._robot_view = None

        # State tensors
        self._drone_pos = None      # [num_envs, 3]
        self._drone_vel = None      # [num_envs, 3]
        self._drone_quat = None     # [num_envs, 4]
        self._drone_ang_vel = None  # [num_envs, 3]

        # Target state
        self._target_positions = None   # [num_envs, num_targets, 3]
        self._target_detected = None    # [num_envs, num_targets] bool

        # Coverage grid
        self._coverage_grid = None      # [num_envs, grid_x, grid_y] bool
        self._coverage_pct = None       # [num_envs]
        self._prev_coverage_pct = None  # [num_envs]

        # Mission state
        self._battery_pct = None        # [num_envs]
        self._mission_time = None       # [num_envs]
        self._step_count = None         # [num_envs]
        self._episode_rewards = None    # [num_envs]

        # Frontier tracking (uncovered areas)
        self._frontier_centroids = None # [num_envs, num_frontier_samples, 2]

        # Action smoothness tracking
        self._prev_action = None        # [num_envs, 4]

        # Home position for RTB calculations
        self._home_position = torch.zeros(3, device=self.device)

        # Domain randomization state
        self._mass_scale = None         # [num_envs]
        self._drag_scale = None         # [num_envs]

        # Precompute grid dimensions
        x_min, x_max, y_min, y_max = self.cfg.world_bounds[:4]
        self._grid_x = int((x_max - x_min) / self.cfg.coverage_grid_resolution)
        self._grid_y = int((y_max - y_min) / self.cfg.coverage_grid_resolution)

    @property
    def observation_space_shape(self) -> Tuple[int]:
        """Return observation space shape."""
        return (self.OBS_DIM,)

    @property
    def action_space_shape(self) -> Tuple[int]:
        """Return action space shape."""
        return (self.ACTION_DIM,)

    def setup(self) -> None:
        """Initialize the simulation environment."""
        if self._initialized:
            return

        print("=" * 60)
        print("Search Training Environment Setup")
        print(f"  Parallel Environments: {self.num_envs}")
        print(f"  Observation Dim: {self.OBS_DIM}")
        print(f"  Action Dim: {self.ACTION_DIM}")
        print(f"  Device: {self.device}")
        print("=" * 60)

        # Initialize Isaac Sim
        self._init_isaac_sim()

        # Create world and spawn entities
        self._init_world()
        self._spawn_drones()
        self._spawn_targets()

        # Initialize tensors
        self._init_tensors()

        self._initialized = True
        print("Search Training Environment Ready")

    def _init_isaac_sim(self) -> None:
        """Initialize Isaac Sim application.

        If simulation_app is already set (passed via set_simulation_app),
        this method does nothing. Otherwise creates a new SimulationApp.
        """
        if self.simulation_app is not None:
            # Already initialized externally
            return

        from isaacsim import SimulationApp

        config = {
            "headless": True,
            "width": 1280,
            "height": 720,
        }
        self.simulation_app = SimulationApp(config)

    def set_simulation_app(self, app) -> None:
        """Set an externally created SimulationApp.

        This allows the training script to create the SimulationApp before
        importing CUDA libraries (torch, numpy), avoiding conflicts with
        Carbonite extension loading.
        """
        self.simulation_app = app

    def _init_world(self) -> None:
        """Initialize simulation world."""
        from omni.isaac.core.world import World

        self.world = World(
            physics_dt=self.cfg.physics_dt,
            rendering_dt=self.cfg.physics_dt,
            stage_units_in_meters=1.0,
            backend="torch",
            device=str(self.device),
        )
        self.world.scene.add_default_ground_plane()

    def _spawn_drones(self) -> None:
        """Spawn drone rigid bodies."""
        from pxr import UsdGeom, UsdPhysics, Gf
        import omni.isaac.core.utils.prims as prim_utils
        from omni.isaac.core.prims import RigidPrimView
        from isaacsim.core.utils.stage import get_current_stage

        self._stage = get_current_stage()
        prefix = self.cfg.prim_path_prefix

        for i in range(self.num_envs):
            env_path = f"/World/{prefix}_Env_{i}"
            drone_path = f"{env_path}/Drone"

            prim_utils.create_prim(env_path, "Xform")
            prim_utils.create_prim(drone_path, "Xform")
            drone_prim = self._stage.GetPrimAtPath(drone_path)

            # Visual
            body_vis = UsdGeom.Cube.Define(self._stage, f"{drone_path}/body_visual")
            body_vis.GetSizeAttr().Set(0.3)

            # Physics
            UsdPhysics.RigidBodyAPI.Apply(drone_prim)
            mass_api = UsdPhysics.MassAPI.Apply(drone_prim)
            mass_api.CreateMassAttr(6.3)  # F-11 mass
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(0.115, 0.115, 0.175))

            # Collision
            collision = UsdGeom.Sphere.Define(self._stage, f"{drone_path}/collision")
            collision.GetRadiusAttr().Set(0.25)
            UsdPhysics.CollisionAPI.Apply(collision.GetPrim())

            # Position
            drone_xform = UsdGeom.Xformable(drone_prim)
            drone_xform.ClearXformOpOrder()
            translate_op = drone_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(Gf.Vec3d(
                (i % 32) * 10.0 - 160.0,
                (i // 32) * 10.0 - 160.0,
                50.0
            ))
            orient_op = drone_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
            orient_op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))

        # Create view for batched operations
        self._robot_view = RigidPrimView(
            prim_paths_expr=f"/World/{prefix}_Env_.*/Drone",
            name=f"{prefix}_drone_view",
        )
        self.world.scene.add(self._robot_view)
        self.world.reset()
        self._robot_view.initialize()
        self.world.step(render=False)

    def _spawn_targets(self) -> None:
        """Spawn target positions (ground truth, no physics)."""
        # Targets are just positions tracked in tensors, no visual prims needed
        pass

    def _init_tensors(self) -> None:
        """Initialize all state tensors."""
        n = self.num_envs

        # Drone state
        self._drone_pos = torch.zeros((n, 3), device=self.device)
        self._drone_vel = torch.zeros((n, 3), device=self.device)
        self._drone_quat = torch.zeros((n, 4), device=self.device)
        self._drone_quat[:, 3] = 1.0  # Identity quaternion (w=1)
        self._drone_ang_vel = torch.zeros((n, 3), device=self.device)

        # Targets
        self._target_positions = torch.zeros((n, self.cfg.num_targets, 3), device=self.device)
        self._target_detected = torch.zeros((n, self.cfg.num_targets), dtype=torch.bool, device=self.device)

        # Coverage
        self._coverage_grid = torch.zeros((n, self._grid_x, self._grid_y), dtype=torch.bool, device=self.device)
        self._coverage_pct = torch.zeros(n, device=self.device)
        self._prev_coverage_pct = torch.zeros(n, device=self.device)

        # Mission
        self._battery_pct = torch.ones(n, device=self.device) * 100.0
        self._mission_time = torch.zeros(n, device=self.device)
        self._step_count = torch.zeros(n, dtype=torch.long, device=self.device)
        self._episode_rewards = torch.zeros(n, device=self.device)

        # Frontier
        self._frontier_centroids = torch.zeros((n, self.cfg.num_frontier_samples, 2), device=self.device)

        # Action history
        self._prev_action = torch.zeros((n, self.ACTION_DIM), device=self.device)

        # Domain randomization
        self._mass_scale = torch.ones(n, device=self.device)
        self._drag_scale = torch.ones(n, device=self.device)

    def reset(self, env_ids: Optional[torch.Tensor] = None) -> torch.Tensor:
        """Reset specified environments."""
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)

        n = len(env_ids)

        # Randomize spawn position
        x_min, x_max, y_min, y_max, z_min, z_max = self.cfg.world_bounds
        spawn_x = torch.rand(n, device=self.device) * self.cfg.spawn_area_radius * 2 - self.cfg.spawn_area_radius
        spawn_y = torch.rand(n, device=self.device) * self.cfg.spawn_area_radius * 2 - self.cfg.spawn_area_radius
        spawn_z = torch.rand(n, device=self.device) * (self.cfg.spawn_height_range[1] - self.cfg.spawn_height_range[0]) + self.cfg.spawn_height_range[0]

        self._drone_pos[env_ids, 0] = spawn_x
        self._drone_pos[env_ids, 1] = spawn_y
        self._drone_pos[env_ids, 2] = spawn_z
        self._drone_vel[env_ids] = 0
        self._drone_quat[env_ids] = torch.tensor([0.0, 0.0, 0.0, 1.0], device=self.device)
        self._drone_ang_vel[env_ids] = 0

        # Reset targets (random positions within world bounds)
        for i in range(self.cfg.num_targets):
            self._target_positions[env_ids, i, 0] = torch.rand(n, device=self.device) * (x_max - x_min) + x_min
            self._target_positions[env_ids, i, 1] = torch.rand(n, device=self.device) * (y_max - y_min) + y_min
            self._target_positions[env_ids, i, 2] = 0  # Ground level
        self._target_detected[env_ids] = False

        # Reset coverage
        self._coverage_grid[env_ids] = False
        self._coverage_pct[env_ids] = 0
        self._prev_coverage_pct[env_ids] = 0

        # Reset mission
        self._battery_pct[env_ids] = 100.0
        self._mission_time[env_ids] = 0
        self._step_count[env_ids] = 0
        self._episode_rewards[env_ids] = 0

        # Reset action history
        self._prev_action[env_ids] = 0

        # Apply domain randomization
        if self.cfg.mass_randomization > 0:
            self._mass_scale[env_ids] = 1.0 + (torch.rand(n, device=self.device) * 2 - 1) * self.cfg.mass_randomization
        if self.cfg.drag_randomization > 0:
            self._drag_scale[env_ids] = 1.0 + (torch.rand(n, device=self.device) * 2 - 1) * self.cfg.drag_randomization

        # Write to simulation
        self._write_state_to_sim(env_ids)

        return self._get_observations()

    def step(self, actions: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, Dict]:
        """
        Execute one environment step.

        Actions: [num_envs, 4]
            [0]: Forward velocity [-1, 1]
            [1]: Lateral velocity [-1, 1]
            [2]: Vertical velocity [-1, 1]
            [3]: Yaw rate [-1, 1]
        """
        actions = torch.clamp(actions, -1.0, 1.0)

        # Physics steps
        for _ in range(self.cfg.decimation):
            self._apply_velocity_commands(actions)
            self.world.step(render=False)

        # Update state
        self._read_state_from_sim()
        self._update_coverage()
        self._update_target_detection()
        self._update_mission_state()

        # Compute outputs
        obs = self._get_observations()
        rewards = self._compute_rewards(actions)
        dones = self._check_termination()

        # Update counters
        self._step_count += 1
        self._episode_rewards += rewards
        self._prev_action = actions.clone()

        # Auto-reset done environments
        done_env_ids = torch.where(dones)[0]
        if len(done_env_ids) > 0:
            self.reset(done_env_ids)

        info = {
            "coverage_pct": self._coverage_pct.clone(),
            "battery_pct": self._battery_pct.clone(),
            "targets_detected": self._target_detected.sum(dim=1),
        }

        return obs, rewards, dones, info

    def _apply_velocity_commands(self, actions: torch.Tensor) -> None:
        """Apply velocity commands through PD controller."""
        # Scale actions to velocity setpoints
        vel_setpoint = torch.zeros((self.num_envs, 3), device=self.device)
        vel_setpoint[:, 0] = actions[:, 0] * self.cfg.max_horizontal_vel
        vel_setpoint[:, 1] = actions[:, 1] * self.cfg.max_horizontal_vel
        vel_setpoint[:, 2] = actions[:, 2] * self.cfg.max_vertical_vel

        yaw_rate_setpoint = actions[:, 3] * self.cfg.max_yaw_rate

        # Velocity error
        vel_error = vel_setpoint - self._drone_vel

        # PD control (simplified)
        mass = 6.3 * self._mass_scale
        force_xy = vel_error[:, :2] * 5.0 * mass.unsqueeze(-1)
        force_z = mass * (9.81 + vel_error[:, 2] * 8.0)

        # Assemble forces
        forces = torch.zeros((self.num_envs, 3), device=self.device)
        forces[:, :2] = force_xy
        forces[:, 2] = force_z

        # Yaw moment
        yaw_error = yaw_rate_setpoint - self._drone_ang_vel[:, 2]
        moments = torch.zeros((self.num_envs, 3), device=self.device)
        moments[:, 2] = yaw_error * 4.0 * 0.175  # Izz

        # Apply to simulation
        self._robot_view.apply_forces_and_torques_at_pos(
            forces=forces,
            torques=moments,
            is_global=True,
        )

    def _read_state_from_sim(self) -> None:
        """Read state from simulation."""
        poses = self._robot_view.get_world_poses()
        vels = self._robot_view.get_velocities()

        if poses is not None:
            self._drone_pos = poses[0]
            self._drone_quat = poses[1]
        if vels is not None:
            self._drone_vel = vels[:, :3]
            self._drone_ang_vel = vels[:, 3:]

    def _write_state_to_sim(self, env_ids: torch.Tensor) -> None:
        """Write state to simulation."""
        self._robot_view.set_world_poses(
            positions=self._drone_pos[env_ids],
            orientations=self._drone_quat[env_ids],
            indices=env_ids,
        )
        self._robot_view.set_velocities(
            velocities=torch.cat([
                self._drone_vel[env_ids],
                self._drone_ang_vel[env_ids]
            ], dim=-1),
            indices=env_ids,
        )

    def _update_coverage(self) -> None:
        """Update coverage grid based on drone position."""
        x_min, x_max, y_min, y_max = self.cfg.world_bounds[:4]
        res = self.cfg.coverage_grid_resolution

        # Convert position to grid indices
        grid_x = ((self._drone_pos[:, 0] - x_min) / res).long().clamp(0, self._grid_x - 1)
        grid_y = ((self._drone_pos[:, 1] - y_min) / res).long().clamp(0, self._grid_y - 1)

        # Mark coverage (with some radius based on altitude and FOV)
        # Simplified: mark current cell and neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                gx = (grid_x + dx).clamp(0, self._grid_x - 1)
                gy = (grid_y + dy).clamp(0, self._grid_y - 1)
                self._coverage_grid[torch.arange(self.num_envs, device=self.device), gx, gy] = True

        # Update coverage percentage
        self._prev_coverage_pct = self._coverage_pct.clone()
        self._coverage_pct = self._coverage_grid.float().mean(dim=(1, 2)) * 100.0

    def _update_target_detection(self) -> None:
        """Check for new target detections."""
        # Simple distance-based detection (within FOV would be more accurate)
        detection_range = 50.0  # meters

        for i in range(self.cfg.num_targets):
            # Distance to each target
            target_pos = self._target_positions[:, i, :]
            dist = torch.norm(self._drone_pos - target_pos, dim=-1)

            # Check altitude (must be above target with downward camera)
            alt_ok = self._drone_pos[:, 2] > target_pos[:, 2]

            # Mark as detected if in range and not already detected
            new_detection = (dist < detection_range) & alt_ok & ~self._target_detected[:, i]
            self._target_detected[:, i] |= new_detection

    def _update_mission_state(self) -> None:
        """Update battery and mission time."""
        dt = self.cfg.physics_dt * self.cfg.decimation
        self._mission_time += dt
        self._battery_pct -= 0.001  # Simple drain rate

    def _compute_frontier_centroids(self) -> None:
        """Compute centroids of uncovered frontier regions."""
        # Simplified: find uncovered cells and sample
        x_min, x_max, y_min, y_max = self.cfg.world_bounds[:4]
        res = self.cfg.coverage_grid_resolution

        for i in range(self.num_envs):
            uncovered = ~self._coverage_grid[i]
            uncovered_indices = torch.nonzero(uncovered)

            if len(uncovered_indices) > 0:
                # Sample random uncovered cells
                n_samples = min(self.cfg.num_frontier_samples, len(uncovered_indices))
                perm = torch.randperm(len(uncovered_indices))[:n_samples]
                sampled = uncovered_indices[perm].float()

                # Convert to world coordinates
                self._frontier_centroids[i, :n_samples, 0] = sampled[:, 0] * res + x_min
                self._frontier_centroids[i, :n_samples, 1] = sampled[:, 1] * res + y_min

    def _get_observations(self) -> torch.Tensor:
        """
        Compute 40-dimensional observation vector.

        Layout:
        - [0:3]   Position (normalized by 500m)
        - [3:6]   Velocity (normalized by 20 m/s)
        - [6:15]  Rotation matrix (flattened 3x3, from quaternion)
        - [15:18] Angular velocity (normalized by 10 rad/s)
        - [18:19] Coverage percentage (0-1)
        - [19:29] Frontier centroids (5 x 2, normalized)
        - [29:30] Uncovered ratio in nearby region
        - [30:31] Battery percentage (0-1)
        - [31:32] Mission time ratio (0-1)
        - [32:34] Home direction (unit vector XY)
        - [34:35] Home distance (normalized)
        - [35:36] Targets detected ratio
        - [36:38] Mean uncovered direction (to encourage exploration)
        - [38:40] Velocity magnitude and heading error
        """
        # Update frontier centroids
        self._compute_frontier_centroids()

        # Compute rotation matrix from quaternion
        rot_matrix = self._quat_to_rotation_matrix(self._drone_quat)  # [N, 3, 3]

        # Home direction and distance
        home_vec = self._home_position[:2].unsqueeze(0) - self._drone_pos[:, :2]
        home_dist = torch.norm(home_vec, dim=-1, keepdim=True)
        home_dir = home_vec / (home_dist + 1e-6)

        # Max time for normalization
        max_time = self.cfg.max_episode_steps * self.cfg.physics_dt * self.cfg.decimation

        # Build observation
        obs = torch.cat([
            self._drone_pos / 500.0,                                    # 3
            self._drone_vel / 20.0,                                     # 3
            rot_matrix.view(self.num_envs, 9),                          # 9
            self._drone_ang_vel / 10.0,                                 # 3
            (self._coverage_pct / 100.0).unsqueeze(-1),                 # 1
            (self._frontier_centroids / 200.0).view(self.num_envs, -1), # 10
            self._compute_nearby_uncovered().unsqueeze(-1),             # 1
            (self._battery_pct / 100.0).unsqueeze(-1),                  # 1
            (self._mission_time / max_time).unsqueeze(-1),              # 1
            home_dir,                                                    # 2
            home_dist / 500.0,                                          # 1
            (self._target_detected.float().mean(dim=1)).unsqueeze(-1),  # 1
            self._compute_exploration_direction(),                       # 2
            self._compute_velocity_features(),                           # 2
        ], dim=-1)

        return obs

    def _quat_to_rotation_matrix(self, quat: torch.Tensor) -> torch.Tensor:
        """Convert quaternion (xyzw) to rotation matrix."""
        x, y, z, w = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]

        R = torch.zeros((len(quat), 3, 3), device=self.device)
        R[:, 0, 0] = 1 - 2*(y*y + z*z)
        R[:, 0, 1] = 2*(x*y - z*w)
        R[:, 0, 2] = 2*(x*z + y*w)
        R[:, 1, 0] = 2*(x*y + z*w)
        R[:, 1, 1] = 1 - 2*(x*x + z*z)
        R[:, 1, 2] = 2*(y*z - x*w)
        R[:, 2, 0] = 2*(x*z - y*w)
        R[:, 2, 1] = 2*(y*z + x*w)
        R[:, 2, 2] = 1 - 2*(x*x + y*y)

        return R

    def _compute_nearby_uncovered(self) -> torch.Tensor:
        """Compute ratio of uncovered cells in nearby region."""
        x_min, x_max, y_min, y_max = self.cfg.world_bounds[:4]
        res = self.cfg.coverage_grid_resolution

        # Current grid position
        grid_x = ((self._drone_pos[:, 0] - x_min) / res).long().clamp(0, self._grid_x - 1)
        grid_y = ((self._drone_pos[:, 1] - y_min) / res).long().clamp(0, self._grid_y - 1)

        # Check 5x5 region
        uncovered = torch.zeros(self.num_envs, device=self.device)
        count = 0
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                gx = (grid_x + dx).clamp(0, self._grid_x - 1)
                gy = (grid_y + dy).clamp(0, self._grid_y - 1)
                uncovered += (~self._coverage_grid[torch.arange(self.num_envs, device=self.device), gx, gy]).float()
                count += 1

        return uncovered / count

    def _compute_exploration_direction(self) -> torch.Tensor:
        """Compute direction toward uncovered areas."""
        # Simplified: use first frontier centroid
        frontier = self._frontier_centroids[:, 0, :]  # [N, 2]
        direction = frontier - self._drone_pos[:, :2]
        dist = torch.norm(direction, dim=-1, keepdim=True)
        return direction / (dist + 1e-6)

    def _compute_velocity_features(self) -> torch.Tensor:
        """Compute velocity magnitude and heading alignment."""
        speed = torch.norm(self._drone_vel[:, :2], dim=-1, keepdim=True) / 10.0

        # Heading (yaw from quaternion)
        yaw = torch.atan2(
            2 * (self._drone_quat[:, 3] * self._drone_quat[:, 2] + self._drone_quat[:, 0] * self._drone_quat[:, 1]),
            1 - 2 * (self._drone_quat[:, 1]**2 + self._drone_quat[:, 2]**2)
        )

        # Velocity heading
        vel_heading = torch.atan2(self._drone_vel[:, 1], self._drone_vel[:, 0])
        heading_error = torch.cos(vel_heading - yaw).unsqueeze(-1)

        return torch.cat([speed, heading_error], dim=-1)

    def _compute_rewards(self, actions: torch.Tensor) -> torch.Tensor:
        """Compute per-step rewards."""
        cfg = self.cfg.rewards
        rewards = torch.zeros(self.num_envs, device=self.device)

        # Coverage progress
        coverage_delta = self._coverage_pct - self._prev_coverage_pct
        rewards += coverage_delta * cfg.coverage_progress

        # Target detection (check for newly detected this step)
        new_detections = self._target_detected.sum(dim=1) - (self._step_count > 0).float() * self._target_detected.sum(dim=1)
        rewards += new_detections * cfg.target_detection

        # Time penalty
        rewards += cfg.time_penalty

        # Action smoothness
        action_diff = torch.norm(actions - self._prev_action, dim=-1)
        rewards -= action_diff * cfg.smooth_action_penalty

        # Altitude deviation
        alt_error = torch.abs(self._drone_pos[:, 2] - cfg.optimal_altitude)
        rewards -= alt_error * cfg.altitude_deviation_penalty

        # Coverage completion bonus
        coverage_done = self._coverage_pct >= self.cfg.coverage_target_pct
        rewards += coverage_done.float() * cfg.coverage_completion_bonus

        return rewards

    def _check_termination(self) -> torch.Tensor:
        """Check for episode termination."""
        dones = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        # Max steps
        dones |= self._step_count >= self.cfg.max_episode_steps

        # Coverage target reached
        dones |= self._coverage_pct >= self.cfg.coverage_target_pct

        # Crash (altitude too low)
        dones |= self._drone_pos[:, 2] < 2.0

        # Battery critical
        dones |= self._battery_pct < 10.0

        # Out of bounds
        x_min, x_max, y_min, y_max, z_min, z_max = self.cfg.world_bounds
        dones |= (self._drone_pos[:, 0] < x_min - 50) | (self._drone_pos[:, 0] > x_max + 50)
        dones |= (self._drone_pos[:, 1] < y_min - 50) | (self._drone_pos[:, 1] > y_max + 50)

        return dones

    def close(self) -> None:
        """Clean up environment."""
        if self.simulation_app is not None:
            self.simulation_app.close()
