#!/usr/bin/env python3
"""
Dwell Policy Training Environment

Specialized environment for training the DwellPolicy used by the BT node.
Focused observation space (25 dims) and action space (6 dims) optimized
for target tracking with 2-axis gimbal control.

Observation Space (25 dims):
- Target relative state (10): relative position, velocity, image-space position
- Image space features (8): target bounding box, image center offset, frame rate
- UAV state (5): velocity, altitude, yaw rate
- Gimbal state (2): pan, tilt angles

Action Space (6 dims):
- Forward velocity [-1, 1] -> [-max_vel, +max_vel] m/s
- Lateral velocity [-1, 1] -> [-max_vel, +max_vel] m/s
- Vertical velocity [-1, 1] -> [-max_vel_z, +max_vel_z] m/s
- Yaw rate [-1, 1] -> [-max_yaw_rate, +max_yaw_rate] rad/s
- Gimbal pan rate [-1, 1] -> [-45, +45] deg/s
- Gimbal tilt rate [-1, 1] -> [-45, +45] deg/s

Key Features:
- 2-axis gimbal control from the start (per plan specification)
- Image-space target tracking observations
- Smooth gimbal motion rewards
- Dwell time accumulation tracking
- Track quality metrics (GSD, frame rate, stability)

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
)


@dataclass
class DwellRewardConfig:
    """Reward configuration optimized for dwell/tracking behavior."""
    # Track quality rewards
    target_centered: float = 0.5         # Per step when target near image center
    target_in_frame: float = 0.1         # Per step when target in FOV
    track_stability: float = 0.3         # Reward for stable tracking (low jitter)

    # Dwell time
    dwell_progress: float = 1.0          # Per second of accumulated dwell time
    dwell_complete_bonus: float = 50.0   # Bonus when dwell requirement met

    # Gimbal smoothness
    gimbal_smooth_bonus: float = 0.1     # Reward for smooth gimbal motion
    gimbal_jerk_penalty: float = 0.05    # Penalty for jerky gimbal commands

    # Image quality
    gsd_quality_bonus: float = 0.2       # Reward for good ground sample distance
    optimal_range: float = 50.0          # Optimal range for GSD

    # Flight quality
    velocity_smooth_penalty: float = 0.01 # Penalty for jerky flight
    altitude_stable_bonus: float = 0.05   # Reward for stable altitude

    # Mission outcomes
    track_lost_penalty: float = -20.0     # Penalty when track is lost
    dwell_complete_reward: float = 100.0  # Success reward


@dataclass
class DwellTrainingConfig:
    """Configuration for Dwell Policy training environment."""
    # Parallelization
    num_envs: int = 1024
    device: str = "cuda:0"

    # Physics
    physics_dt: float = 1.0 / 250.0
    decimation: int = 2

    # Drone velocity limits (slower for precise tracking)
    max_horizontal_vel: float = 5.0      # m/s
    max_vertical_vel: float = 2.0        # m/s
    max_yaw_rate: float = 0.3            # rad/s

    # Gimbal configuration (2-axis from the start)
    gimbal_pan_rate_limit: float = 0.785   # rad/s (~45 deg/s)
    gimbal_tilt_rate_limit: float = 0.785  # rad/s (~45 deg/s)
    gimbal_pan_limits: Tuple[float, float] = (-3.14, 3.14)  # rad (±180°)
    gimbal_tilt_limits: Tuple[float, float] = (-1.57, 0.52)  # rad (-90° to +30°)

    # Camera configuration
    camera_fov_h: float = 90.0           # degrees
    camera_fov_v: float = 60.0           # degrees
    camera_min_range: float = 5.0        # meters
    camera_max_range: float = 200.0      # meters

    # Dwell requirements
    min_dwell_time: float = 30.0         # seconds
    track_quality_threshold: float = 0.7 # Minimum quality for dwell accumulation
    track_lost_timeout: float = 5.0      # seconds before declaring track lost

    # Episode limits
    max_episode_steps: int = 15000       # ~2 minutes at 125Hz effective

    # Rewards
    rewards: DwellRewardConfig = field(default_factory=DwellRewardConfig)

    # Spawn configuration
    spawn_height_range: Tuple[float, float] = (30.0, 70.0)
    initial_target_distance_range: Tuple[float, float] = (30.0, 80.0)

    # Target motion (for moving target scenarios)
    target_stationary_prob: float = 0.5  # Probability of stationary target
    target_max_speed: float = 3.0        # m/s for moving targets

    # Domain randomization
    mass_randomization: float = 0.03
    drag_randomization: float = 0.05
    gimbal_noise: float = 0.01           # rad noise on gimbal readings

    # Prim path prefix (allows multiple envs in same stage)
    prim_path_prefix: str = "Train"


class DwellTrainingEnv:
    """
    Specialized training environment for DwellPolicy.

    Focused on:
    - Target tracking with 2-axis gimbal
    - Dwell time accumulation
    - Image-space optimization
    - Smooth gimbal and flight control

    Key features:
    - 6-DOF action space (4 velocity + 2 gimbal)
    - Image-space target observations
    - Track quality estimation
    """

    # Class-level observation/action dimensions
    OBS_DIM = 25
    ACTION_DIM = 6

    def __init__(self, config: DwellTrainingConfig = None):
        self.cfg = config or DwellTrainingConfig()
        self.device = torch.device(self.cfg.device)
        self.num_envs = self.cfg.num_envs

        self._initialized = False

        # Will be set during setup
        self.simulation_app = None
        self.world = None
        self._robot_view = None

        # State tensors
        self._drone_pos = None          # [num_envs, 3]
        self._drone_vel = None          # [num_envs, 3]
        self._drone_quat = None         # [num_envs, 4]
        self._drone_ang_vel = None      # [num_envs, 3]

        # Gimbal state (2-axis)
        self._gimbal_pan = None         # [num_envs] rad
        self._gimbal_tilt = None        # [num_envs] rad

        # Target state
        self._target_pos = None         # [num_envs, 3]
        self._target_vel = None         # [num_envs, 3]
        self._target_in_frame = None    # [num_envs] bool
        self._target_image_pos = None   # [num_envs, 2] normalized image coordinates

        # Tracking state
        self._track_quality = None      # [num_envs] float 0-1
        self._accumulated_dwell = None  # [num_envs] float seconds
        self._time_since_track = None   # [num_envs] float seconds since good track

        # Mission state
        self._step_count = None
        self._episode_rewards = None
        self._mission_time = None

        # Action history for smoothness
        self._prev_action = None        # [num_envs, 6]
        self._prev_gimbal_cmd = None    # [num_envs, 2]

        # Domain randomization
        self._mass_scale = None
        self._drag_scale = None

        # Precompute camera parameters
        self._fov_h_rad = np.radians(self.cfg.camera_fov_h)
        self._fov_v_rad = np.radians(self.cfg.camera_fov_v)
        self._half_fov_h = self._fov_h_rad / 2
        self._half_fov_v = self._fov_v_rad / 2

    @property
    def observation_space_shape(self) -> Tuple[int]:
        return (self.OBS_DIM,)

    @property
    def action_space_shape(self) -> Tuple[int]:
        return (self.ACTION_DIM,)

    def setup(self) -> None:
        """Initialize the simulation environment."""
        if self._initialized:
            return

        print("=" * 60)
        print("Dwell Training Environment Setup")
        print(f"  Parallel Environments: {self.num_envs}")
        print(f"  Observation Dim: {self.OBS_DIM}")
        print(f"  Action Dim: {self.ACTION_DIM} (including 2-axis gimbal)")
        print(f"  Device: {self.device}")
        print("=" * 60)

        self._init_isaac_sim()
        self._init_world()
        self._spawn_drones()
        self._init_tensors()

        self._initialized = True
        print("Dwell Training Environment Ready")

    def set_simulation_app(self, app) -> None:
        """Set an externally created SimulationApp.

        Use this to share a SimulationApp between training and eval environments,
        or when the app must be created before importing torch/numpy.
        """
        self.simulation_app = app

    def _init_isaac_sim(self) -> None:
        """Initialize Isaac Sim application.

        If simulation_app is already set (passed via set_simulation_app),
        skip creation. This allows sharing a single SimulationApp instance.
        """
        if self.simulation_app is not None:
            return  # Already initialized externally

        from isaacsim import SimulationApp

        config = {
            "headless": True,
            "width": 1280,
            "height": 720,
        }
        self.simulation_app = SimulationApp(config)

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

            body_vis = UsdGeom.Cube.Define(self._stage, f"{drone_path}/body_visual")
            body_vis.GetSizeAttr().Set(0.3)

            UsdPhysics.RigidBodyAPI.Apply(drone_prim)
            mass_api = UsdPhysics.MassAPI.Apply(drone_prim)
            mass_api.CreateMassAttr(6.3)
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(0.115, 0.115, 0.175))

            collision = UsdGeom.Sphere.Define(self._stage, f"{drone_path}/collision")
            collision.GetRadiusAttr().Set(0.25)
            UsdPhysics.CollisionAPI.Apply(collision.GetPrim())

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

        self._robot_view = RigidPrimView(
            prim_paths_expr=f"/World/{prefix}_Env_.*/Drone",
            name=f"{prefix}_drone_view",
        )
        self.world.scene.add(self._robot_view)
        self.world.reset()
        self._robot_view.initialize()
        self.world.step(render=False)

    def _init_tensors(self) -> None:
        """Initialize all state tensors."""
        n = self.num_envs

        # Drone state
        self._drone_pos = torch.zeros((n, 3), device=self.device)
        self._drone_vel = torch.zeros((n, 3), device=self.device)
        self._drone_quat = torch.zeros((n, 4), device=self.device)
        self._drone_quat[:, 3] = 1.0
        self._drone_ang_vel = torch.zeros((n, 3), device=self.device)

        # Gimbal state
        self._gimbal_pan = torch.zeros(n, device=self.device)
        self._gimbal_tilt = torch.full((n,), -0.5, device=self.device)  # Default look down

        # Target state
        self._target_pos = torch.zeros((n, 3), device=self.device)
        self._target_vel = torch.zeros((n, 3), device=self.device)
        self._target_in_frame = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._target_image_pos = torch.zeros((n, 2), device=self.device)

        # Tracking state
        self._track_quality = torch.zeros(n, device=self.device)
        self._accumulated_dwell = torch.zeros(n, device=self.device)
        self._time_since_track = torch.zeros(n, device=self.device)

        # Mission state
        self._step_count = torch.zeros(n, dtype=torch.long, device=self.device)
        self._episode_rewards = torch.zeros(n, device=self.device)
        self._mission_time = torch.zeros(n, device=self.device)

        # Action history
        self._prev_action = torch.zeros((n, self.ACTION_DIM), device=self.device)
        self._prev_gimbal_cmd = torch.zeros((n, 2), device=self.device)

        # Domain randomization
        self._mass_scale = torch.ones(n, device=self.device)
        self._drag_scale = torch.ones(n, device=self.device)

    def reset(self, env_ids: Optional[torch.Tensor] = None) -> torch.Tensor:
        """Reset specified environments."""
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)

        n = len(env_ids)

        # Spawn drone at random position and altitude
        spawn_x = torch.randn(n, device=self.device) * 20
        spawn_y = torch.randn(n, device=self.device) * 20
        spawn_z = torch.rand(n, device=self.device) * (
            self.cfg.spawn_height_range[1] - self.cfg.spawn_height_range[0]
        ) + self.cfg.spawn_height_range[0]

        self._drone_pos[env_ids, 0] = spawn_x
        self._drone_pos[env_ids, 1] = spawn_y
        self._drone_pos[env_ids, 2] = spawn_z
        self._drone_vel[env_ids] = 0
        self._drone_quat[env_ids] = torch.tensor([0.0, 0.0, 0.0, 1.0], device=self.device)
        self._drone_ang_vel[env_ids] = 0

        # Reset gimbal to default (looking down)
        self._gimbal_pan[env_ids] = 0
        self._gimbal_tilt[env_ids] = -0.5  # ~-30 degrees

        # Spawn target at random distance from drone
        dist_range = self.cfg.initial_target_distance_range
        target_dist = torch.rand(n, device=self.device) * (dist_range[1] - dist_range[0]) + dist_range[0]
        target_angle = torch.rand(n, device=self.device) * 2 * np.pi

        self._target_pos[env_ids, 0] = spawn_x + target_dist * torch.cos(target_angle)
        self._target_pos[env_ids, 1] = spawn_y + target_dist * torch.sin(target_angle)
        self._target_pos[env_ids, 2] = 0  # Ground level

        # Target velocity (some stationary, some moving)
        is_moving = torch.rand(n, device=self.device) > self.cfg.target_stationary_prob
        speed = torch.rand(n, device=self.device) * self.cfg.target_max_speed * is_moving.float()
        move_angle = torch.rand(n, device=self.device) * 2 * np.pi
        self._target_vel[env_ids, 0] = speed * torch.cos(move_angle)
        self._target_vel[env_ids, 1] = speed * torch.sin(move_angle)
        self._target_vel[env_ids, 2] = 0

        # Reset tracking state
        self._target_in_frame[env_ids] = False
        self._target_image_pos[env_ids] = 0
        self._track_quality[env_ids] = 0
        self._accumulated_dwell[env_ids] = 0
        self._time_since_track[env_ids] = 0

        # Reset mission state
        self._step_count[env_ids] = 0
        self._episode_rewards[env_ids] = 0
        self._mission_time[env_ids] = 0

        # Reset action history
        self._prev_action[env_ids] = 0
        self._prev_gimbal_cmd[env_ids] = 0

        # Domain randomization
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

        Actions: [num_envs, 6]
            [0]: Forward velocity [-1, 1]
            [1]: Lateral velocity [-1, 1]
            [2]: Vertical velocity [-1, 1]
            [3]: Yaw rate [-1, 1]
            [4]: Gimbal pan rate [-1, 1]
            [5]: Gimbal tilt rate [-1, 1]
        """
        actions = torch.clamp(actions, -1.0, 1.0)

        # Physics steps (drone movement)
        for _ in range(self.cfg.decimation):
            self._apply_velocity_commands(actions[:, :4])
            self.world.step(render=False)

        # Update state
        self._read_state_from_sim()

        # Update gimbal (after physics to apply gimbal commands)
        self._update_gimbal(actions[:, 4], actions[:, 5])

        # Update target (may be moving)
        self._update_target()

        # Update tracking state (check if target in FOV, compute image position)
        self._update_tracking()

        # Accumulate dwell time if track quality is good
        self._update_dwell()

        # Update mission time
        dt = self.cfg.physics_dt * self.cfg.decimation
        self._mission_time += dt

        # Compute outputs
        obs = self._get_observations()
        rewards = self._compute_rewards(actions)
        dones = self._check_termination()

        # Update counters
        self._step_count += 1
        self._episode_rewards += rewards
        self._prev_action = actions.clone()
        self._prev_gimbal_cmd = actions[:, 4:6].clone()

        # Auto-reset
        done_env_ids = torch.where(dones)[0]
        if len(done_env_ids) > 0:
            self.reset(done_env_ids)

        info = {
            "accumulated_dwell": self._accumulated_dwell.clone(),
            "track_quality": self._track_quality.clone(),
            "target_in_frame": self._target_in_frame.clone(),
        }

        return obs, rewards, dones, info

    def _apply_velocity_commands(self, actions: torch.Tensor) -> None:
        """Apply velocity commands (4 DOF, no gimbal)."""
        vel_setpoint = torch.zeros((self.num_envs, 3), device=self.device)
        vel_setpoint[:, 0] = actions[:, 0] * self.cfg.max_horizontal_vel
        vel_setpoint[:, 1] = actions[:, 1] * self.cfg.max_horizontal_vel
        vel_setpoint[:, 2] = actions[:, 2] * self.cfg.max_vertical_vel

        yaw_rate_setpoint = actions[:, 3] * self.cfg.max_yaw_rate

        vel_error = vel_setpoint - self._drone_vel

        mass = 6.3 * self._mass_scale
        force_xy = vel_error[:, :2] * 5.0 * mass.unsqueeze(-1)
        force_z = mass * (9.81 + vel_error[:, 2] * 8.0)

        forces = torch.zeros((self.num_envs, 3), device=self.device)
        forces[:, :2] = force_xy
        forces[:, 2] = force_z

        yaw_error = yaw_rate_setpoint - self._drone_ang_vel[:, 2]
        moments = torch.zeros((self.num_envs, 3), device=self.device)
        moments[:, 2] = yaw_error * 4.0 * 0.175

        self._robot_view.apply_forces_and_torques_at_pos(
            forces=forces,
            torques=moments,
            is_global=True,
        )

    def _update_gimbal(self, pan_rate_cmd: torch.Tensor, tilt_rate_cmd: torch.Tensor) -> None:
        """Update gimbal angles based on rate commands."""
        dt = self.cfg.physics_dt * self.cfg.decimation

        # Scale rate commands
        pan_rate = pan_rate_cmd * self.cfg.gimbal_pan_rate_limit
        tilt_rate = tilt_rate_cmd * self.cfg.gimbal_tilt_rate_limit

        # Update angles
        self._gimbal_pan += pan_rate * dt
        self._gimbal_tilt += tilt_rate * dt

        # Clamp to limits
        self._gimbal_pan = torch.clamp(
            self._gimbal_pan,
            self.cfg.gimbal_pan_limits[0],
            self.cfg.gimbal_pan_limits[1]
        )
        self._gimbal_tilt = torch.clamp(
            self._gimbal_tilt,
            self.cfg.gimbal_tilt_limits[0],
            self.cfg.gimbal_tilt_limits[1]
        )

    def _update_target(self) -> None:
        """Update target position (for moving targets)."""
        dt = self.cfg.physics_dt * self.cfg.decimation
        self._target_pos += self._target_vel * dt

    def _update_tracking(self) -> None:
        """Update tracking state - check if target is in FOV and compute image position."""
        # Vector from drone to target
        to_target = self._target_pos - self._drone_pos
        target_dist = torch.norm(to_target, dim=-1)

        # Get drone yaw from quaternion
        drone_yaw = torch.atan2(
            2 * (self._drone_quat[:, 3] * self._drone_quat[:, 2] + self._drone_quat[:, 0] * self._drone_quat[:, 1]),
            1 - 2 * (self._drone_quat[:, 1]**2 + self._drone_quat[:, 2]**2)
        )

        # Camera pointing direction (drone yaw + gimbal pan, gimbal tilt)
        cam_yaw = drone_yaw + self._gimbal_pan
        cam_pitch = self._gimbal_tilt

        # Convert to_target to camera frame
        # Rotate around Z by -cam_yaw
        cos_yaw = torch.cos(-cam_yaw)
        sin_yaw = torch.sin(-cam_yaw)
        target_cam_x = to_target[:, 0] * cos_yaw - to_target[:, 1] * sin_yaw
        target_cam_y = to_target[:, 0] * sin_yaw + to_target[:, 1] * cos_yaw
        target_cam_z = to_target[:, 2]

        # Compute angles to target in camera frame
        target_azimuth = torch.atan2(target_cam_y, target_cam_x)
        target_elevation = torch.atan2(-target_cam_z, torch.sqrt(target_cam_x**2 + target_cam_y**2))

        # Adjust for camera pitch
        target_elevation_cam = target_elevation - cam_pitch

        # Check if in FOV
        in_fov_h = torch.abs(target_azimuth) < self._half_fov_h
        in_fov_v = torch.abs(target_elevation_cam) < self._half_fov_v
        in_range = (target_dist > self.cfg.camera_min_range) & (target_dist < self.cfg.camera_max_range)

        self._target_in_frame = in_fov_h & in_fov_v & in_range

        # Compute normalized image position [-1, 1] for targets in frame
        self._target_image_pos[:, 0] = target_azimuth / self._half_fov_h
        self._target_image_pos[:, 1] = target_elevation_cam / self._half_fov_v

        # Compute track quality based on centering and range
        center_error = torch.norm(self._target_image_pos, dim=-1)
        range_quality = 1.0 - torch.abs(target_dist - self.cfg.rewards.optimal_range) / 100.0
        range_quality = torch.clamp(range_quality, 0.0, 1.0)

        centering_quality = 1.0 - center_error
        centering_quality = torch.clamp(centering_quality, 0.0, 1.0)

        self._track_quality = self._target_in_frame.float() * centering_quality * range_quality

    def _update_dwell(self) -> None:
        """Accumulate dwell time when track quality is sufficient."""
        dt = self.cfg.physics_dt * self.cfg.decimation

        # Accumulate dwell time for good tracks
        good_track = self._track_quality >= self.cfg.track_quality_threshold
        self._accumulated_dwell += good_track.float() * dt

        # Update time since last good track
        self._time_since_track = torch.where(
            good_track,
            torch.zeros_like(self._time_since_track),
            self._time_since_track + dt
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

    def _get_observations(self) -> torch.Tensor:
        """
        Compute 25-dimensional observation vector.

        Layout:
        - [0:3]   Target relative position (normalized)
        - [3:6]   Target relative velocity (normalized)
        - [6:8]   Target image position (normalized [-1, 1])
        - [8:10]  Target distance and angle (normalized)
        - [10:13] Drone velocity (normalized)
        - [13:14] Drone altitude (normalized)
        - [14:15] Drone yaw rate (normalized)
        - [15:17] Gimbal angles (normalized)
        - [17:19] Gimbal rates (from previous command, normalized)
        - [19:20] Track quality
        - [20:21] Target in frame flag
        - [21:22] Accumulated dwell ratio
        - [22:23] Time since track (normalized)
        - [23:25] Distance to optimal tracking position
        """
        # Target relative position
        rel_pos = self._target_pos - self._drone_pos
        rel_dist = torch.norm(rel_pos, dim=-1, keepdim=True)

        # Target relative velocity
        rel_vel = self._target_vel - self._drone_vel

        # Dwell progress
        dwell_ratio = (self._accumulated_dwell / self.cfg.min_dwell_time).unsqueeze(-1)

        # Time since track (normalized by timeout)
        track_time_norm = (self._time_since_track / self.cfg.track_lost_timeout).unsqueeze(-1)

        # Optimal tracking direction (toward target at optimal range)
        to_target_unit = rel_pos / (rel_dist + 1e-6)
        optimal_pos = self._target_pos - to_target_unit * self.cfg.rewards.optimal_range
        dist_to_optimal = (self._drone_pos - optimal_pos)[:, :2] / 100.0  # XY only

        obs = torch.cat([
            rel_pos / 100.0,                                # 3
            rel_vel / 10.0,                                 # 3
            self._target_image_pos,                         # 2
            rel_dist / 100.0,                               # 1
            torch.atan2(rel_pos[:, 1], rel_pos[:, 0]).unsqueeze(-1) / np.pi,  # 1
            self._drone_vel / 10.0,                         # 3
            (self._drone_pos[:, 2] / 100.0).unsqueeze(-1),  # 1
            (self._drone_ang_vel[:, 2] / 2.0).unsqueeze(-1),  # 1
            (self._gimbal_pan / np.pi).unsqueeze(-1),       # 1
            (self._gimbal_tilt / np.pi).unsqueeze(-1),      # 1
            self._prev_gimbal_cmd,                          # 2
            self._track_quality.unsqueeze(-1),              # 1
            self._target_in_frame.float().unsqueeze(-1),    # 1
            dwell_ratio,                                     # 1
            track_time_norm,                                 # 1
            dist_to_optimal,                                 # 2
        ], dim=-1)

        return obs

    def _compute_rewards(self, actions: torch.Tensor) -> torch.Tensor:
        """Compute per-step rewards."""
        cfg = self.cfg.rewards
        rewards = torch.zeros(self.num_envs, device=self.device)

        # Target centering reward
        center_error = torch.norm(self._target_image_pos, dim=-1)
        rewards += self._target_in_frame.float() * (1.0 - center_error) * cfg.target_centered

        # Target in frame reward
        rewards += self._target_in_frame.float() * cfg.target_in_frame

        # Track stability (low jitter in image position)
        # Approximated by track quality
        rewards += self._track_quality * cfg.track_stability

        # Dwell progress
        dt = self.cfg.physics_dt * self.cfg.decimation
        good_track = self._track_quality >= self.cfg.track_quality_threshold
        rewards += good_track.float() * cfg.dwell_progress * dt

        # Gimbal smoothness
        gimbal_cmd = actions[:, 4:6]
        gimbal_diff = torch.norm(gimbal_cmd - self._prev_gimbal_cmd, dim=-1)
        rewards -= gimbal_diff * cfg.gimbal_jerk_penalty
        rewards += (1.0 - gimbal_diff) * self._target_in_frame.float() * cfg.gimbal_smooth_bonus

        # GSD quality (based on range)
        rel_pos = self._target_pos - self._drone_pos
        target_dist = torch.norm(rel_pos, dim=-1)
        range_quality = 1.0 - torch.abs(target_dist - cfg.optimal_range) / 50.0
        range_quality = torch.clamp(range_quality, 0.0, 1.0)
        rewards += self._target_in_frame.float() * range_quality * cfg.gsd_quality_bonus

        # Velocity smoothness
        vel_diff = torch.norm(actions[:, :4] - self._prev_action[:, :4], dim=-1)
        rewards -= vel_diff * cfg.velocity_smooth_penalty

        # Dwell complete bonus
        dwell_complete = self._accumulated_dwell >= self.cfg.min_dwell_time
        rewards += dwell_complete.float() * cfg.dwell_complete_bonus

        return rewards

    def _check_termination(self) -> torch.Tensor:
        """Check for episode termination."""
        dones = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        # Max steps
        dones |= self._step_count >= self.cfg.max_episode_steps

        # Dwell complete
        dones |= self._accumulated_dwell >= self.cfg.min_dwell_time

        # Track lost (timeout exceeded)
        dones |= self._time_since_track >= self.cfg.track_lost_timeout

        # Crash
        dones |= self._drone_pos[:, 2] < 2.0

        # Target too far (gave up)
        rel_dist = torch.norm(self._target_pos - self._drone_pos, dim=-1)
        dones |= rel_dist > 300.0

        return dones

    def close(self) -> None:
        """Clean up environment."""
        if self.simulation_app is not None:
            self.simulation_app.close()
