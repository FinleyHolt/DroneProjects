#!/usr/bin/env python3
"""
ISR Training Environment - Fast RL Training with Ground Truth Perception

Phase 2 implementation using Isaac Lab DirectRLEnv pattern:
- Uses set_external_force_and_torque() for drone dynamics (no PX4 SITL)
- Ground truth target observations with frustum filtering
- Gimbal control via articulation joints
- Target: 10k-50k FPS with 1024 parallel environments

Key Features:
- Frustum-filtered ground truth: Only targets visible in camera FOV
- Gimbal as articulated joint: 2-DOF pitch/yaw control
- Compatible observation/action space with evaluation env

Usage:
    from training.environments import ISRTrainingEnv, ISRTrainingConfig

    config = ISRTrainingConfig(num_envs=1024)
    env = ISRTrainingEnv(config)
    env.setup()

    obs = env.reset()
    for _ in range(1000):
        action = policy(obs)
        obs, reward, done, info = env.step(action)

Author: Finley Holt
"""

import torch
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from enum import Enum

# Import mission tasking for scenario-based spawning
import sys
import os
# Add parent directories to path for mission_tasking import
_current_dir = os.path.dirname(os.path.abspath(__file__))
_environments_dir = os.path.dirname(os.path.dirname(_current_dir))
if _environments_dir not in sys.path:
    sys.path.insert(0, _environments_dir)

try:
    from environments.mission_tasking import (
        MissionTasking,
        create_comms_denied_tasking,
        create_dynamic_nfz_tasking,
        create_multi_objective_tasking,
        generate_poi_distribution_from_tasking,
        NAIPriority,
    )
    _MISSION_TASKING_AVAILABLE = True
except ImportError:
    _MISSION_TASKING_AVAILABLE = False


@dataclass
class CameraConfig:
    """Camera/gimbal configuration for perception."""
    # Field of view
    fov_horizontal: float = 90.0  # degrees
    fov_vertical: float = 60.0    # degrees

    # Range limits
    min_range: float = 2.0        # meters
    max_range: float = 200.0      # meters

    # Gimbal limits
    gimbal_pitch_min: float = -90.0   # degrees (looking down)
    gimbal_pitch_max: float = 30.0    # degrees (looking up)
    gimbal_yaw_min: float = -180.0    # degrees
    gimbal_yaw_max: float = 180.0     # degrees

    # Default gimbal position
    gimbal_pitch_default: float = -30.0  # degrees
    gimbal_yaw_default: float = 0.0      # degrees

    # Gimbal dynamics
    gimbal_rate_limit: float = 60.0  # degrees/second

    # Altitude-dependent detection (GSD modeling)
    # Higher altitude = larger coverage but worse GSD (ground sample distance)
    optimal_altitude: float = 50.0        # meters - best detection altitude
    altitude_detection_falloff: float = 0.02  # Detection quality drops per meter deviation
    min_detection_quality: float = 0.1    # Floor on detection quality multiplier
    max_detection_altitude: float = 500.0 # meters - targets undetectable above this

    # Range-dependent detection (inverse-square falloff)
    optimal_range: float = 50.0           # meters - best detection range
    range_detection_falloff: float = 1.5  # Exponent for falloff beyond optimal


@dataclass
class TargetConfig:
    """Target/POI configuration."""
    max_targets: int = 16          # Maximum targets to track in observation
    detection_reward: float = 10.0  # Reward for detecting target
    tracking_reward: float = 0.1    # Reward per step while tracking

    # Dwell time tracking - sustained observation for quality intelligence
    min_dwell_time: float = 2.0           # Seconds for "quality" detection
    dwell_reward_rate: float = 0.5        # Reward per second of continuous dwell
    quality_detection_bonus: float = 5.0  # One-time bonus when dwell threshold met


@dataclass
class DroneConfig:
    """
    Drone physical parameters (F-11 ISR variant).

    Control Architecture:
    The RL policy outputs velocity commands (vx, vy, vz, yaw_rate) which are
    tracked by a simple PD controller that computes forces/torques. This matches
    the deployment architecture where the policy sends velocity setpoints to
    PX4/ArduPilot's velocity controller.

    Physics Notes:
    - Mass: 6.3kg total (airframe + batteries + payload)
    - Inertia: Estimated for ~50cm arm span quadcopter
    - Velocity limits match typical ISR operational envelope

    For sim-to-real transfer, these values should be validated against
    actual F-11 flight data.
    """
    mass: float = 6.3              # kg
    inertia_xx: float = 0.115      # kg*m^2 (kept lower for faster training dynamics)
    inertia_yy: float = 0.115      # Can increase to ~0.25 for realistic sim
    inertia_zz: float = 0.175

    # Velocity limits (m/s) - matches PX4 MPC_XY_VEL_MAX and MPC_Z_VEL_MAX
    max_horizontal_vel: float = 12.0  # m/s horizontal (typical ISR cruise)
    max_vertical_vel: float = 5.0     # m/s vertical (ascent/descent)
    max_yaw_rate: float = 1.0         # rad/s yaw rate

    # Velocity controller gains (PD controller tracking velocity setpoints)
    # These simulate PX4's velocity controller response
    vel_p_gain_xy: float = 5.0        # Proportional gain for horizontal velocity
    vel_d_gain_xy: float = 0.5        # Derivative gain for horizontal velocity
    vel_p_gain_z: float = 8.0         # Proportional gain for vertical velocity
    vel_d_gain_z: float = 1.0         # Derivative gain for vertical velocity
    yaw_p_gain: float = 4.0           # Proportional gain for yaw rate

    # Thrust parameters (used internally by velocity controller)
    thrust_to_weight: float = 2.0     # Max thrust / weight ratio
    moment_scale: float = 2.0         # Max moment for attitude control

    # Drag coefficient: F_drag = -drag_coeff * v * |v|
    drag_coeff: float = 0.1           # Quadratic drag coefficient


class ScenarioType(Enum):
    """Training scenario types corresponding to canonical ISR problems."""
    AREA_COVERAGE = "area_coverage"      # Problem 1: Area surveillance for coverage
    NFZ_AVOIDANCE = "nfz_avoidance"      # Problem 2: Dynamic no-fly zone avoidance
    MULTI_OBJECTIVE = "multi_objective"  # Problem 3: Multi-objective ISR with threats


@dataclass
class WorldConfig:
    """World/environment configuration."""
    # World bounds (x_min, x_max, y_min, y_max, z_min, z_max)
    bounds: Tuple[float, float, float, float, float, float] = (
        -250.0, 250.0,   # x: 500m total
        -250.0, 250.0,   # y: 500m total
        0.0, 120.0       # z: 120m ceiling
    )

    # Coverage grid resolution (meters per cell)
    coverage_resolution: float = 10.0

    # Launch/home position
    launch_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Wind disturbances (domain randomization for sim-to-real)
    wind_enabled: bool = True
    wind_speed_range: Tuple[float, float] = (0.0, 5.0)  # m/s min/max
    wind_gust_probability: float = 0.01  # Per-step probability of gust
    wind_gust_magnitude: float = 3.0     # Additional m/s during gust

    # Observation noise (sensor modeling for sim-to-real)
    position_noise_std: float = 0.5    # meters (GPS noise)
    velocity_noise_std: float = 0.1    # m/s (IMU noise)
    attitude_noise_std: float = 0.01   # radians (IMU noise)


@dataclass
class MissionConfig:
    """Mission-specific configuration."""
    # Scenario type
    scenario: ScenarioType = ScenarioType.AREA_COVERAGE

    # Episode limits
    max_episode_steps: int = 37500  # ~5 minutes at 125Hz effective rate

    # Coverage requirements (Scenario 1)
    target_coverage_pct: float = 85.0

    # Battery
    initial_battery_pct: float = 100.0
    battery_reserve_pct: float = 25.0
    battery_drain_rate: float = 0.001  # % per step (rough estimate)

    # RTB (Return to Base) configuration
    # Ontology-enforced RTB when battery can't safely return home
    rtb_speed: float = 10.0           # m/s assumed return speed
    rtb_headroom: float = 1.2         # 20% safety margin on RTB battery estimate
    rtb_reward: float = 50.0          # Bonus for successful RTB
    rtb_progress_reward: float = 0.1  # Per-meter reward when heading home in RTB mode

    # Success/failure rewards
    mission_success_reward: float = 100.0
    mission_failure_reward: float = -50.0


@dataclass
class NFZConfig:
    """No-fly zone configuration (Scenario 2)."""
    enabled: bool = False
    max_nfz: int = 10

    # NFZ violation penalties
    nfz_violation_reward: float = -100.0
    nfz_buffer_reward: float = -0.1  # Soft penalty for being close
    nfz_buffer_distance: float = 50.0  # Buffer zone in meters


@dataclass
class ThreatConfig:
    """Threat zone configuration (Scenario 3)."""
    enabled: bool = False
    max_threat_zones: int = 5

    # Threat levels: 0=none, 1=LOW, 2=MEDIUM, 3=HIGH
    max_medium_exposure: float = 30.0  # Max seconds in MEDIUM zones
    high_threat_terminates: bool = True  # HIGH zone = episode termination

    # Penalties
    medium_threat_rate: float = -0.1  # Per-second penalty in MEDIUM
    high_threat_reward: float = -200.0  # Instant penalty for HIGH


@dataclass
class RewardConfig:
    """Reward weights for training. Loaded from scenario_configs.yaml."""
    # Detection/tracking
    target_detection: float = 10.0     # Reward per new target detected
    target_tracking: float = 0.1       # Reward per step tracking target

    # Coverage (Scenario 1)
    coverage_progress: float = 1.0     # Reward per % coverage increase

    # Flight quality
    height_penalty: float = 0.001      # Penalty per meter altitude deviation
    velocity_penalty: float = 0.0001   # Penalty per m/s velocity
    target_altitude: float = 50.0      # Target operational altitude

    # Action smoothness - penalize jerky commands for sim-to-real transfer
    action_smoothness_penalty: float = 0.01   # Penalty per unit velocity command change
    gimbal_smoothness_penalty: float = 0.02   # Higher penalty for gimbal jerk (affects imagery)

    # Mission outcomes
    mission_success: float = 100.0     # Bonus for mission success
    crash: float = -100.0              # Penalty for crashing
    battery_critical: float = -50.0    # Penalty when battery < reserve

    # NFZ specific (Scenario 2)
    progress_to_dest: float = 0.5      # Reward for getting closer to destination
    dest_reached: float = 200.0        # Bonus for reaching destination
    nfz_violation: float = -100.0      # Penalty for NFZ violation
    nfz_buffer: float = -0.1           # Soft penalty for being in buffer zone

    # Threat specific (Scenario 3)
    efficiency_bonus: float = 5.0      # Bonus for value/battery efficiency
    exposure_warning: float = -1.0     # Warning when approaching exposure limit


@dataclass
class ISRTrainingConfig:
    """Configuration for ISR training environment."""
    # Parallelization
    num_envs: int = 256  # RTX 5090 can handle high parallelization
    device: str = "cuda:0"

    # Physics
    physics_dt: float = 1.0 / 250.0
    decimation: int = 2  # Action repeat

    # Camera/perception
    camera: CameraConfig = field(default_factory=CameraConfig)

    # Targets
    targets: TargetConfig = field(default_factory=TargetConfig)

    # Drone
    drone: DroneConfig = field(default_factory=DroneConfig)

    # World configuration (replaces world_bounds)
    world: WorldConfig = field(default_factory=WorldConfig)

    # Mission configuration
    mission: MissionConfig = field(default_factory=MissionConfig)

    # NFZ configuration (Scenario 2)
    nfz: NFZConfig = field(default_factory=NFZConfig)

    # Threat configuration (Scenario 3)
    threat: ThreatConfig = field(default_factory=ThreatConfig)

    # Reward configuration
    rewards: RewardConfig = field(default_factory=RewardConfig)

    # Spawn configuration
    spawn_height_range: Tuple[float, float] = (30.0, 60.0)
    spawn_area_radius: float = 50.0

    # Termination
    crash_height: float = 2.0
    crash_reward: float = -100.0

    # Mission tasking integration
    use_mission_tasking: bool = True  # Use NAI-based target spawning
    base_seed: int = 42  # Base random seed for reproducibility

    # Backwards compatibility property
    @property
    def world_bounds(self) -> Tuple[float, float, float, float, float, float]:
        """Backwards compatible access to world bounds."""
        return self.world.bounds

    @property
    def max_episode_steps(self) -> int:
        """Backwards compatible access to max episode steps."""
        return self.mission.max_episode_steps


class ISRTrainingEnv:
    """
    Fast ISR training environment using Isaac Lab patterns.

    Key design decisions:
    1. Uses set_external_force_and_torque() instead of PX4 SITL
    2. Ground truth targets with frustum filtering (simulates camera FOV)
    3. Gimbal as 2-DOF articulation for realistic camera control
    4. Batched operations for massive parallelization
    """

    def __init__(self, config: ISRTrainingConfig = None):
        self.cfg = config or ISRTrainingConfig()
        self.device = torch.device(self.cfg.device)

        self.num_envs = self.cfg.num_envs
        self._initialized = False

        # Will be set during setup
        self.simulation_app = None
        self.world = None
        self._robot = None
        self._robot_view = None
        self._use_gpu_tensors = None  # Cached after first detection

        # State tensors (batched across all envs)
        self._drone_pos = None      # [num_envs, 3]
        self._drone_vel = None      # [num_envs, 3]
        self._drone_quat = None     # [num_envs, 4] xyzw (Isaac Sim convention)
        self._drone_ang_vel = None  # [num_envs, 3]

        # Gimbal state
        self._gimbal_pitch = None   # [num_envs]
        self._gimbal_yaw = None     # [num_envs]

        # Target state
        self._target_positions = None  # [num_envs, max_targets, 3]
        self._target_active = None     # [num_envs, max_targets] bool
        self._target_detected = None   # [num_envs, max_targets] bool
        self._target_priorities = None # [num_envs, max_targets] int (1-4, 1=critical)
        self._target_values = None     # [num_envs, max_targets] float

        # Episode tracking
        self._step_count = None        # [num_envs]
        self._episode_rewards = None   # [num_envs]

        # Mission state
        self._battery_pct = None       # [num_envs] float
        self._mission_time = None      # [num_envs] float (seconds)

        # Coverage tracking (Phase 1 - Scenario 1)
        self._coverage_grid = None     # [num_envs, grid_x, grid_y] bool
        self._coverage_pct = None      # [num_envs] float
        self._coverage_grid_size = None  # (grid_x, grid_y) tuple
        self._prev_coverage_pct = None # [num_envs] float - for delta reward

        # Reward tracking
        self._newly_detected = None    # [num_envs, max_targets] bool - targets detected this step
        self._prev_dest_distance = None # [num_envs] float - for progress reward

        # NFZ state (Phase 2 - Scenario 2)
        self._nfz_centers = None       # [num_envs, max_nfz, 2] float (XY)
        self._nfz_radii = None         # [num_envs, max_nfz] float
        self._nfz_activation_step = None  # [num_envs, max_nfz] int (-1 = always active)
        self._nfz_active = None        # [num_envs, max_nfz] bool
        self._nearest_nfz_dist = None  # [num_envs] float
        self._in_nfz = None            # [num_envs] bool
        self._destination = None       # [num_envs, 3] float
        self._dest_distance = None     # [num_envs] float

        # Threat state (Phase 3 - Scenario 3)
        self._threat_centers = None    # [num_envs, max_threat, 2] float (XY)
        self._threat_radii = None      # [num_envs, max_threat] float
        self._threat_levels = None     # [num_envs, max_threat] int (0-3)
        self._threat_exposure = None   # [num_envs] float (accumulated seconds in MEDIUM)
        self._current_threat_level = None  # [num_envs] int
        self._collected_value = None   # [num_envs] float

        # Actions
        self._thrust = None         # [num_envs, 1, 3]
        self._moment = None         # [num_envs, 1, 3]

        # Episode tracking for mission tasking
        self._episode_seed = self.cfg.base_seed  # Incremented each episode for variation

        # Precomputed constants
        self._gravity = 9.81
        self._robot_weight = self.cfg.drone.mass * self._gravity
        self._body_id = None

        # Frustum precomputation
        self._fov_h_rad = np.radians(self.cfg.camera.fov_horizontal)
        self._fov_v_rad = np.radians(self.cfg.camera.fov_vertical)
        self._half_fov_h = self._fov_h_rad / 2
        self._half_fov_v = self._fov_v_rad / 2

        # Launch position for home calculations
        self._launch_position = torch.tensor(
            self.cfg.world.launch_position, device=self.device
        )

    def setup(self) -> None:
        """Initialize the simulation environment."""
        if self._initialized:
            return

        print("=" * 60)
        print("ISR Training Environment Setup")
        print(f"  Parallel Environments: {self.num_envs}")
        print(f"  Device: {self.device}")
        print("=" * 60)

        # Initialize Isaac Sim
        print("\n[1/5] Initializing Isaac Sim...")
        self._init_isaac_sim()

        # Create world
        print("\n[2/5] Creating world...")
        self._init_world()

        # Spawn drones
        print("\n[3/5] Spawning drones...")
        self._spawn_drones()

        # Spawn targets
        print("\n[4/5] Spawning targets...")
        self._spawn_targets()

        # Initialize tensors
        print("\n[5/5] Initializing state tensors...")
        self._init_tensors()

        self._initialized = True
        print("\n" + "=" * 60)
        print("ISR Training Environment Ready")
        print("=" * 60)

    def _init_isaac_sim(self) -> None:
        """Initialize Isaac Sim application."""
        from isaacsim import SimulationApp

        # Headless for training
        config = {
            "headless": True,
            "width": 1280,
            "height": 720,
        }

        self.simulation_app = SimulationApp(config)

    def _init_world(self) -> None:
        """Initialize Isaac Sim world with physics."""
        from omni.isaac.core.world import World
        from pxr import UsdPhysics, PhysxSchema, Gf

        # Enable GPU pipeline for high performance
        self.world = World(
            physics_dt=self.cfg.physics_dt,
            rendering_dt=self.cfg.physics_dt,
            stage_units_in_meters=1.0,
            backend="torch",  # Use torch tensors instead of numpy
            device=str(self.device),  # Run physics on GPU
        )

        # Get stage
        from isaacsim.core.utils.stage import get_current_stage
        self._stage = get_current_stage()

        # Configure physics scene with GPU dynamics
        physics_scene_path = "/World/PhysicsScene"
        if not self._stage.GetPrimAtPath(physics_scene_path):
            scene = UsdPhysics.Scene.Define(self._stage, physics_scene_path)
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(self._gravity)

        # Enable GPU dynamics on PhysX scene
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(self._stage.GetPrimAtPath(physics_scene_path))
        physx_scene.CreateEnableGPUDynamicsAttr().Set(True)
        physx_scene.CreateBroadphaseTypeAttr().Set("GPU")
        physx_scene.CreateGpuFoundLostPairsCapacityAttr().Set(1024 * 1024)
        physx_scene.CreateGpuCollisionStackSizeAttr().Set(64 * 1024 * 1024)

        # Add ground plane
        self.world.scene.add_default_ground_plane()

    def _spawn_drones(self) -> None:
        """Spawn drone rigid bodies for all environments."""
        from pxr import UsdGeom, UsdPhysics, Gf
        import omni.isaac.core.utils.prims as prim_utils
        from omni.isaac.core.prims import RigidPrimView

        # Create drone instances
        drone_paths = []
        for i in range(self.num_envs):
            env_path = f"/World/Env_{i}"
            drone_path = f"{env_path}/Drone"

            # Create env prim
            prim_utils.create_prim(env_path, "Xform")

            # Create drone body
            prim_utils.create_prim(drone_path, "Xform")
            drone_prim = self._stage.GetPrimAtPath(drone_path)

            # Add visual geometry (simple box)
            body_vis = UsdGeom.Cube.Define(self._stage, f"{drone_path}/body_visual")
            body_vis.GetSizeAttr().Set(0.3)
            body_xform = UsdGeom.Xformable(body_vis.GetPrim())
            body_xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 0.4))

            # Apply rigid body physics
            UsdPhysics.RigidBodyAPI.Apply(drone_prim)

            # Mass properties
            mass_api = UsdPhysics.MassAPI.Apply(drone_prim)
            mass_api.CreateMassAttr(self.cfg.drone.mass)
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(
                self.cfg.drone.inertia_xx,
                self.cfg.drone.inertia_yy,
                self.cfg.drone.inertia_zz,
            ))

            # Collision
            collision_vis = UsdGeom.Sphere.Define(self._stage, f"{drone_path}/collision")
            collision_vis.GetRadiusAttr().Set(0.25)
            UsdPhysics.CollisionAPI.Apply(collision_vis.GetPrim())

            # Set initial position - clear existing ops first and use explicit precision
            drone_xform = UsdGeom.Xformable(drone_prim)
            drone_xform.ClearXformOpOrder()
            translate_op = drone_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(Gf.Vec3d(
                (i % 32) * 10.0 - 160.0,  # Spread in X
                (i // 32) * 10.0 - 160.0,  # Spread in Y
                50.0
            ))
            orient_op = drone_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
            orient_op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))

            drone_paths.append(drone_path)

        # Create RigidPrimView for batched operations
        self._robot_view = RigidPrimView(
            prim_paths_expr="/World/Env_.*/Drone",
            name="drone_view",
            track_contact_forces=False,
            prepare_contact_sensors=False,
        )
        self.world.scene.add(self._robot_view)

        # Reset world to initialize physics
        self.world.reset()

        # Initialize the RigidPrimView (required for GPU pipeline)
        self._robot_view.initialize()

        # Step physics once to ensure simulation is fully initialized
        # This is required for forces to be properly applied
        self.world.step(render=False)

        # Detect backend type now so _apply_actions works on first call
        pos, _ = self._robot_view.get_world_poses()
        self._use_gpu_tensors = not isinstance(pos, np.ndarray)
        print(f"  Backend: {'GPU tensors' if self._use_gpu_tensors else 'CPU numpy'}")

        self._body_id = 0  # Body ID for force application

        print(f"  Spawned {self.num_envs} drones")

    def _spawn_targets(self) -> None:
        """Spawn target objects (vehicles, people, etc.)."""
        from pxr import UsdGeom, Gf
        import omni.isaac.core.utils.prims as prim_utils

        max_targets = self.cfg.targets.max_targets

        # For now, create simple visual markers for targets
        # In full implementation, use WorldGenerator for realistic assets
        for env_idx in range(self.num_envs):
            env_path = f"/World/Env_{env_idx}"

            for target_idx in range(max_targets):
                target_path = f"{env_path}/Target_{target_idx}"

                # Create target prim (visual marker)
                prim_utils.create_prim(target_path, "Xform")
                marker = UsdGeom.Sphere.Define(self._stage, f"{target_path}/marker")
                marker.GetRadiusAttr().Set(1.0)

                # Random position in world
                x = np.random.uniform(-100, 100)
                y = np.random.uniform(-100, 100)
                z = 0.0  # Ground level

                target_xform = UsdGeom.Xformable(self._stage.GetPrimAtPath(target_path))
                target_xform.ClearXformOpOrder()
                target_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, z))

        print(f"  Spawned {max_targets} targets per env")

    def _init_tensors(self) -> None:
        """Initialize state tensors on device."""
        n = self.num_envs
        max_t = self.cfg.targets.max_targets
        bounds = self.cfg.world.bounds

        # Drone state
        self._drone_pos = torch.zeros((n, 3), device=self.device)
        self._drone_vel = torch.zeros((n, 3), device=self.device)
        self._drone_quat = torch.zeros((n, 4), device=self.device)
        self._drone_quat[:, 3] = 1.0  # w=1 for identity quaternion (xyzw: [0,0,0,1])
        self._drone_ang_vel = torch.zeros((n, 3), device=self.device)

        # Gimbal state
        self._gimbal_pitch = torch.full(
            (n,), np.radians(self.cfg.camera.gimbal_pitch_default),
            device=self.device
        )
        self._gimbal_yaw = torch.zeros((n,), device=self.device)

        # Target state
        self._target_positions = torch.zeros((n, max_t, 3), device=self.device)
        self._target_active = torch.ones((n, max_t), dtype=torch.bool, device=self.device)
        self._target_detected = torch.zeros((n, max_t), dtype=torch.bool, device=self.device)
        self._target_priorities = torch.full((n, max_t), 3, dtype=torch.int32, device=self.device)  # Default priority 3 (MEDIUM)
        self._target_values = torch.full((n, max_t), 25.0, device=self.device)  # Default value

        # Randomize initial target positions based on world bounds
        world_x_range = bounds[1] - bounds[0]
        world_y_range = bounds[3] - bounds[2]
        self._target_positions[:, :, 0] = torch.rand((n, max_t), device=self.device) * world_x_range * 0.8 + bounds[0] * 0.8
        self._target_positions[:, :, 1] = torch.rand((n, max_t), device=self.device) * world_y_range * 0.8 + bounds[2] * 0.8
        self._target_positions[:, :, 2] = 0.0

        # Episode state
        self._step_count = torch.zeros((n,), dtype=torch.int32, device=self.device)
        self._episode_rewards = torch.zeros((n,), device=self.device)

        # Mission state
        self._battery_pct = torch.full((n,), self.cfg.mission.initial_battery_pct, device=self.device)
        self._mission_time = torch.zeros((n,), device=self.device)

        # Coverage tracking (Phase 1 - Scenario 1)
        # Grid size based on world bounds and resolution
        grid_x = int((bounds[1] - bounds[0]) / self.cfg.world.coverage_resolution)
        grid_y = int((bounds[3] - bounds[2]) / self.cfg.world.coverage_resolution)
        self._coverage_grid_size = (grid_x, grid_y)
        self._coverage_grid = torch.zeros((n, grid_x, grid_y), dtype=torch.bool, device=self.device)
        self._coverage_pct = torch.zeros((n,), device=self.device)
        self._prev_coverage_pct = torch.zeros((n,), device=self.device)
        print(f"  Coverage grid: {grid_x}x{grid_y} cells ({self.cfg.world.coverage_resolution}m resolution)")

        # Reward tracking
        self._newly_detected = torch.zeros((n, max_t), dtype=torch.bool, device=self.device)
        self._prev_dest_distance = torch.zeros((n,), device=self.device)

        # NFZ state (Phase 2 - Scenario 2)
        max_nfz = self.cfg.nfz.max_nfz
        self._nfz_centers = torch.zeros((n, max_nfz, 2), device=self.device)
        self._nfz_radii = torch.zeros((n, max_nfz), device=self.device)
        self._nfz_activation_step = torch.full((n, max_nfz), -1, dtype=torch.int32, device=self.device)  # -1 = inactive
        self._nfz_active = torch.zeros((n, max_nfz), dtype=torch.bool, device=self.device)
        self._nearest_nfz_dist = torch.full((n,), float('inf'), device=self.device)
        self._in_nfz = torch.zeros((n,), dtype=torch.bool, device=self.device)
        self._destination = torch.zeros((n, 3), device=self.device)
        self._dest_distance = torch.zeros((n,), device=self.device)

        # Threat state (Phase 3 - Scenario 3)
        max_threat = self.cfg.threat.max_threat_zones
        self._threat_centers = torch.zeros((n, max_threat, 2), device=self.device)
        self._threat_radii = torch.zeros((n, max_threat), device=self.device)
        self._threat_levels = torch.zeros((n, max_threat), dtype=torch.int32, device=self.device)
        self._threat_exposure = torch.zeros((n,), device=self.device)
        self._current_threat_level = torch.zeros((n,), dtype=torch.int32, device=self.device)
        self._collected_value = torch.zeros((n,), device=self.device)

        # Action tensors - [num_envs, 3] for apply_forces_and_torques_at_pos
        self._thrust = torch.zeros((n, 3), device=self.device)
        self._moment = torch.zeros((n, 3), device=self.device)

        # === Phase 5: Ontology-Constrained RL Improvements ===

        # 5.1 RTB (Return to Base) state
        self._rtb_mode = torch.zeros((n,), dtype=torch.bool, device=self.device)
        self._rtb_battery_threshold = torch.zeros((n,), device=self.device)
        self._prev_home_distance = torch.zeros((n,), device=self.device)

        # 5.2 Dwell time tracking per target
        self._target_dwell_time = torch.zeros((n, max_t), device=self.device)
        self._target_quality_detected = torch.zeros((n, max_t), dtype=torch.bool, device=self.device)
        self._newly_quality_detected = torch.zeros((n, max_t), dtype=torch.bool, device=self.device)

        # 5.3 Action smoothness tracking
        self._prev_actions = torch.zeros((n, 6), device=self.device)  # 6 action dims

        # 5.6 Wind state
        self._wind_velocity = torch.zeros((n, 3), device=self.device)

        # Pre-allocate observation buffer to reduce memory fragmentation
        # This will be filled in _get_observations() instead of cat-ing every step
        self._obs_buffer = torch.zeros((n, self.observation_dim), device=self.device)

    def reset(self, env_ids: Optional[torch.Tensor] = None) -> torch.Tensor:
        """
        Reset specified environments.

        Args:
            env_ids: Indices of environments to reset. If None, reset all.

        Returns:
            Observation tensor
        """
        if not self._initialized:
            self.setup()

        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)

        n_reset = len(env_ids)

        # Reset drone positions
        spawn_height = torch.rand((n_reset,), device=self.device) * (
            self.cfg.spawn_height_range[1] - self.cfg.spawn_height_range[0]
        ) + self.cfg.spawn_height_range[0]

        spawn_x = torch.rand((n_reset,), device=self.device) * self.cfg.spawn_area_radius * 2 - self.cfg.spawn_area_radius
        spawn_y = torch.rand((n_reset,), device=self.device) * self.cfg.spawn_area_radius * 2 - self.cfg.spawn_area_radius

        self._drone_pos[env_ids, 0] = spawn_x
        self._drone_pos[env_ids, 1] = spawn_y
        self._drone_pos[env_ids, 2] = spawn_height

        # Reset velocities
        self._drone_vel[env_ids] = 0.0
        self._drone_ang_vel[env_ids] = 0.0

        # Reset orientation (identity) - xyzw convention: [0, 0, 0, 1]
        self._drone_quat[env_ids] = torch.tensor([0.0, 0.0, 0.0, 1.0], device=self.device)

        # Reset gimbal
        self._gimbal_pitch[env_ids] = np.radians(self.cfg.camera.gimbal_pitch_default)
        self._gimbal_yaw[env_ids] = 0.0

        # Reset targets - use mission tasking if enabled, otherwise random
        self._target_detected[env_ids] = False
        self._target_active[env_ids] = True
        if self.cfg.use_mission_tasking and _MISSION_TASKING_AVAILABLE:
            self._spawn_targets_from_tasking(env_ids)
        else:
            self._spawn_targets_random(env_ids)

        # Reset episode counters
        self._step_count[env_ids] = 0
        self._episode_rewards[env_ids] = 0.0

        # Reset mission state
        self._battery_pct[env_ids] = self.cfg.mission.initial_battery_pct
        self._mission_time[env_ids] = 0.0

        # Reset coverage grid (Phase 1 - Scenario 1)
        self._coverage_grid[env_ids] = False
        self._coverage_pct[env_ids] = 0.0
        self._prev_coverage_pct[env_ids] = 0.0

        # Reset reward tracking
        self._newly_detected[env_ids] = False
        self._prev_dest_distance[env_ids] = 0.0

        # Reset NFZ state (Phase 2 - Scenario 2)
        self._nfz_active[env_ids] = False
        self._nearest_nfz_dist[env_ids] = float('inf')
        self._in_nfz[env_ids] = False
        self._dest_distance[env_ids] = 0.0

        # Reset threat state (Phase 3 - Scenario 3)
        self._threat_exposure[env_ids] = 0.0
        self._current_threat_level[env_ids] = 0
        self._collected_value[env_ids] = 0.0

        # Reset Phase 5 state (Ontology-Constrained RL Improvements)
        # 5.1 RTB state
        self._rtb_mode[env_ids] = False
        self._rtb_battery_threshold[env_ids] = 0.0
        # Initialize home distance based on spawn position
        home_vec = self._drone_pos[env_ids, :2] - self._launch_position[:2].unsqueeze(0)
        self._prev_home_distance[env_ids] = torch.norm(home_vec, dim=-1)

        # 5.2 Dwell time state
        self._target_dwell_time[env_ids] = 0.0
        self._target_quality_detected[env_ids] = False
        self._newly_quality_detected[env_ids] = False

        # 5.3 Action smoothness state
        self._prev_actions[env_ids] = 0.0

        # 5.6 Wind state - randomize initial wind direction per episode
        if self.cfg.world.wind_enabled:
            wind_speed = torch.rand((len(env_ids),), device=self.device) * \
                (self.cfg.world.wind_speed_range[1] - self.cfg.world.wind_speed_range[0]) + \
                self.cfg.world.wind_speed_range[0]
            wind_dir = torch.randn((len(env_ids), 3), device=self.device)
            wind_dir = wind_dir / torch.norm(wind_dir, dim=-1, keepdim=True)
            self._wind_velocity[env_ids] = wind_dir * wind_speed.unsqueeze(-1)

        # Apply to simulation
        self._write_state_to_sim(env_ids)

        # Read back state to ensure consistency
        self._read_state_from_sim()

        return self._get_observations()

    def step(self, actions: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, Dict]:
        """
        Execute one environment step.

        Action space (velocity commands - matches PX4/ArduPilot interface):
            actions: [num_envs, 6] tensor
                [0]: vx command (-1 to 1) -> scaled to [-max_horizontal_vel, +max_horizontal_vel]
                [1]: vy command (-1 to 1) -> scaled to [-max_horizontal_vel, +max_horizontal_vel]
                [2]: vz command (-1 to 1) -> scaled to [-max_vertical_vel, +max_vertical_vel]
                [3]: yaw_rate command (-1 to 1) -> scaled to [-max_yaw_rate, +max_yaw_rate]
                [4]: gimbal pitch rate command (-1 to 1)
                [5]: gimbal yaw rate command (-1 to 1)

        The velocity commands are tracked by an internal PD controller that
        computes the necessary forces/torques. This matches the deployment
        architecture where the RL policy sends velocity setpoints to the
        flight controller.

        Returns:
            observations, rewards, dones, info
        """
        # Clamp actions
        actions = torch.clamp(actions, -1.0, 1.0)

        # Apply physics for decimation steps
        for _ in range(self.cfg.decimation):
            self._apply_velocity_commands(actions)
            self.world.step(render=False)

        # Update state from simulation
        self._read_state_from_sim()

        # Update gimbal
        self._update_gimbal(actions[:, 4], actions[:, 5])

        # Update mission state (Phase 1)
        self._update_mission_state()

        # Update coverage grid (Phase 1 - Scenario 1)
        self._update_coverage()

        # Update NFZ state (Phase 2 - Scenario 2)
        if self.cfg.nfz.enabled:
            self._update_nfz_state()

        # Update threat state (Phase 3 - Scenario 3)
        if self.cfg.threat.enabled:
            self._update_threat_state()

        # === Phase 5: Ontology-Constrained RL Updates ===

        # 5.1 Update RTB mode based on battery feasibility
        self._update_rtb_mode()

        # 5.6 Update wind (slowly varying + gusts)
        if self.cfg.world.wind_enabled:
            self._update_wind()

        # Compute frustum-filtered observations
        obs = self._get_observations()

        # Compute per-step rewards (includes 5.1 RTB, 5.2 dwell, 5.3 smoothness, 5.4/5.5 detection quality)
        rewards = self._compute_rewards(actions)

        # Check termination and get detailed info
        dones, term_info = self._check_termination()

        # Add terminal rewards based on termination type
        rewards = self._add_terminal_rewards(rewards, term_info)

        # Update counters
        self._step_count += 1
        self._episode_rewards += rewards

        # Auto-reset done environments
        done_env_ids = torch.where(dones)[0]
        if len(done_env_ids) > 0:
            self.reset(done_env_ids)

        info = {
            "step_count": self._step_count.clone(),
            "episode_reward": self._episode_rewards.clone(),
            "coverage_pct": self._coverage_pct.clone(),
            "battery_pct": self._battery_pct.clone(),
            "term_info": term_info,  # Include termination details
        }

        return obs, rewards, dones, info

    def _apply_velocity_commands(self, actions: torch.Tensor) -> None:
        """
        Apply velocity commands using a PD controller to compute forces/torques.

        This simulates PX4/ArduPilot's velocity controller. The RL policy outputs
        velocity setpoints, and this controller tracks them by computing appropriate
        thrust and moments.

        Action mapping:
        - actions[:, 0]: vx command [-1, 1] -> [-max_horizontal_vel, +max_horizontal_vel] m/s
        - actions[:, 1]: vy command [-1, 1] -> [-max_horizontal_vel, +max_horizontal_vel] m/s
        - actions[:, 2]: vz command [-1, 1] -> [-max_vertical_vel, +max_vertical_vel] m/s
        - actions[:, 3]: yaw_rate command [-1, 1] -> [-max_yaw_rate, +max_yaw_rate] rad/s

        The controller computes:
        1. Horizontal forces (world frame) to track vx, vy
        2. Vertical thrust to track vz (plus gravity compensation)
        3. Yaw moment to track yaw_rate
        """
        cfg = self.cfg.drone

        # Scale actions to velocity setpoints
        vel_setpoint = torch.zeros((self.num_envs, 3), device=self.device)
        vel_setpoint[:, 0] = actions[:, 0] * cfg.max_horizontal_vel  # vx
        vel_setpoint[:, 1] = actions[:, 1] * cfg.max_horizontal_vel  # vy
        vel_setpoint[:, 2] = actions[:, 2] * cfg.max_vertical_vel    # vz

        yaw_rate_setpoint = actions[:, 3] * cfg.max_yaw_rate  # rad/s

        # Velocity error (world frame)
        vel_error = vel_setpoint - self._drone_vel

        # =================================================================
        # PD Controller for horizontal velocity (world frame forces)
        # =================================================================
        # Proportional term
        force_xy = vel_error[:, :2] * cfg.vel_p_gain_xy * cfg.mass

        # We don't have acceleration directly, so use velocity error derivative
        # approximation (simplified - could add state for proper derivative)
        # For now, just use P control for simplicity (D term would need history)

        # =================================================================
        # PD Controller for vertical velocity (thrust)
        # =================================================================
        # Vertical force = mass * (g + P * vel_error_z)
        # gravity compensation + proportional control
        gravity = 9.81
        force_z = cfg.mass * (gravity + vel_error[:, 2] * cfg.vel_p_gain_z)

        # Clamp thrust to physical limits
        max_thrust = cfg.thrust_to_weight * cfg.mass * gravity
        force_z = torch.clamp(force_z, 0.0, max_thrust)

        # =================================================================
        # Yaw rate controller (moment about Z)
        # =================================================================
        # Current yaw rate from angular velocity (world Z component)
        current_yaw_rate = self._drone_ang_vel[:, 2]
        yaw_rate_error = yaw_rate_setpoint - current_yaw_rate
        moment_z = yaw_rate_error * cfg.yaw_p_gain * cfg.inertia_zz

        # Clamp moment
        moment_z = torch.clamp(moment_z, -cfg.moment_scale, cfg.moment_scale)

        # =================================================================
        # Assemble force and moment vectors
        # =================================================================
        # World-frame horizontal forces + body-frame vertical thrust
        # For simplicity, we apply all forces in world frame
        self._thrust[:, 0] = force_xy[:, 0]
        self._thrust[:, 1] = force_xy[:, 1]
        self._thrust[:, 2] = force_z

        # Moments (only yaw for now - roll/pitch handled implicitly by world-frame forces)
        self._moment.zero_()
        self._moment[:, 2] = moment_z

        # Add drag force
        vel_magnitude = torch.norm(self._drone_vel, dim=-1, keepdim=True).clamp(min=1e-6)
        drag_force = -cfg.drag_coeff * self._drone_vel * vel_magnitude

        # Update wind and add wind force (5.6)
        if self.cfg.world.wind_enabled:
            self._update_wind()
            # Wind creates apparent velocity error, modeled as external force
            # F_wind = 0.5 * rho * Cd * A * v_wind^2 (simplified to linear for training stability)
            wind_force = self._wind_velocity * cfg.drag_coeff * cfg.mass * 0.5
        else:
            wind_force = torch.zeros_like(self._thrust)

        # Total world-frame force
        total_force = self._thrust + drag_force + wind_force

        # Apply forces to simulation (all in world frame for velocity control)
        if self._use_gpu_tensors:
            self._robot_view.apply_forces_and_torques_at_pos(
                forces=total_force,
                torques=self._moment,
                positions=None,  # Apply at CoM
                is_global=True,  # World frame for velocity control
            )
        else:
            # CPU path
            self._robot_view.apply_forces_and_torques_at_pos(
                forces=total_force.cpu().numpy(),
                torques=self._moment.cpu().numpy(),
                positions=None,
                is_global=True,
            )

    def _update_gimbal(self, pitch_cmd: torch.Tensor, yaw_cmd: torch.Tensor) -> None:
        """Update gimbal angles based on rate commands."""
        dt = self.cfg.physics_dt * self.cfg.decimation
        rate_limit = np.radians(self.cfg.camera.gimbal_rate_limit)

        # Rate commands
        pitch_rate = pitch_cmd * rate_limit
        yaw_rate = yaw_cmd * rate_limit

        # Integrate
        self._gimbal_pitch += pitch_rate * dt
        self._gimbal_yaw += yaw_rate * dt

        # Clamp to limits
        self._gimbal_pitch = torch.clamp(
            self._gimbal_pitch,
            np.radians(self.cfg.camera.gimbal_pitch_min),
            np.radians(self.cfg.camera.gimbal_pitch_max),
        )
        self._gimbal_yaw = torch.clamp(
            self._gimbal_yaw,
            np.radians(self.cfg.camera.gimbal_yaw_min),
            np.radians(self.cfg.camera.gimbal_yaw_max),
        )

    def _update_mission_state(self) -> None:
        """Update mission-related state (battery, time)."""
        # Update mission time
        dt = self.cfg.physics_dt * self.cfg.decimation
        self._mission_time += dt

        # Drain battery
        self._battery_pct -= self.cfg.mission.battery_drain_rate

    def _update_coverage(self) -> None:
        """Update coverage grid based on camera footprint. GPU-optimized."""
        # Store previous coverage for reward computation
        self._prev_coverage_pct = self._coverage_pct.clone()

        bounds = self.cfg.world.bounds
        resolution = self.cfg.world.coverage_resolution

        # Camera footprint radius approximation: altitude * tan(fov/2)
        # Using vertical FOV for conservative estimate
        footprint_radius = self._drone_pos[:, 2] * np.tan(self._half_fov_v)

        # Convert drone XY position to grid coordinates
        grid_x = ((self._drone_pos[:, 0] - bounds[0]) / resolution).long()
        grid_y = ((self._drone_pos[:, 1] - bounds[2]) / resolution).long()

        # Clamp to valid grid indices
        grid_x = grid_x.clamp(0, self._coverage_grid_size[0] - 1)
        grid_y = grid_y.clamp(0, self._coverage_grid_size[1] - 1)

        # Mark cells as covered - VECTORIZED (single GPU kernel)
        # Uses advanced indexing instead of Python for-loop
        env_indices = torch.arange(self.num_envs, device=self.device)
        self._coverage_grid[env_indices, grid_x, grid_y] = True

        # Compute coverage percentage
        total_cells = max(self._coverage_grid_size[0] * self._coverage_grid_size[1], 1)
        covered_cells = self._coverage_grid.float().sum(dim=(1, 2))
        self._coverage_pct = (covered_cells / total_cells) * 100.0

    def _update_nfz_state(self) -> None:
        """Update NFZ activation and distance calculations. GPU-optimized."""
        # Activate NFZs based on step count
        # NFZs with activation_step >= 0 activate when step_count reaches that value
        # NFZs with activation_step == -1 stay inactive
        should_activate = (
            (self._step_count.unsqueeze(-1) >= self._nfz_activation_step) &
            (self._nfz_activation_step >= 0)
        )
        self._nfz_active = self._nfz_active | should_activate

        # Compute distance to nearest active NFZ edge
        drone_xy = self._drone_pos[:, :2].unsqueeze(1)  # [N, 1, 2]
        nfz_xy = self._nfz_centers  # [N, max_nfz, 2]

        dist_to_center = torch.norm(drone_xy - nfz_xy, dim=-1)  # [N, max_nfz]
        dist_to_edge = dist_to_center - self._nfz_radii

        # Mask inactive NFZs with infinity
        dist_to_edge = torch.where(
            self._nfz_active,
            dist_to_edge,
            torch.full_like(dist_to_edge, float('inf'))
        )

        # Get nearest NFZ distance and check if inside any NFZ
        self._nearest_nfz_dist = dist_to_edge.min(dim=-1).values
        self._in_nfz = (dist_to_edge < 0).any(dim=-1)

        # Update destination distance if destination is set
        if self._destination.abs().sum() > 0:
            self._dest_distance = torch.norm(
                self._drone_pos - self._destination, dim=-1
            )

    def _update_threat_state(self) -> None:
        """Update threat zone exposure tracking. GPU-optimized."""
        dt = self.cfg.physics_dt * self.cfg.decimation

        # Compute distance to threat zone centers
        drone_xy = self._drone_pos[:, :2].unsqueeze(1)  # [N, 1, 2]
        threat_xy = self._threat_centers  # [N, max_threat, 2]

        dist = torch.norm(drone_xy - threat_xy, dim=-1)  # [N, max_threat]
        in_threat = dist < self._threat_radii  # [N, max_threat]

        # Get max threat level currently inside
        # Mask threat levels by whether we're inside
        masked_levels = torch.where(
            in_threat,
            self._threat_levels,
            torch.zeros_like(self._threat_levels)
        )
        self._current_threat_level = masked_levels.max(dim=-1).values

        # Accumulate exposure time for MEDIUM threats (level 2)
        medium_exposure = (self._current_threat_level == 2).float()
        self._threat_exposure += medium_exposure * dt

    # === Phase 5: Ontology-Constrained RL Helper Functions ===

    def _compute_rtb_battery_threshold(self) -> torch.Tensor:
        """
        Compute minimum battery needed to RTB from current position.

        Formula: battery_needed = (distance_to_home / rtb_speed) * drain_rate * headroom + reserve

        This is a simplified model - real implementation would account for:
        - Wind conditions
        - Altitude changes (climbing costs more)
        - Payload weight

        Returns:
            [num_envs] tensor of battery thresholds
        """
        # Distance to home (XY plane)
        home_distance = torch.norm(
            self._drone_pos[:, :2] - self._launch_position[:2].unsqueeze(0),
            dim=-1
        )
        time_to_home = home_distance / self.cfg.mission.rtb_speed

        # Battery drain is per-step, convert to per-second
        dt = self.cfg.physics_dt * self.cfg.decimation
        drain_per_second = self.cfg.mission.battery_drain_rate / dt

        battery_needed = time_to_home * drain_per_second * self.cfg.mission.rtb_headroom
        battery_needed += self.cfg.mission.battery_reserve_pct  # Must land with reserve

        return battery_needed

    def _update_rtb_mode(self) -> None:
        """Check if drone should enter RTB mode based on battery feasibility."""
        threshold = self._compute_rtb_battery_threshold()
        self._rtb_battery_threshold = threshold

        # Enter RTB mode when battery drops to threshold
        should_rtb = self._battery_pct <= threshold

        # Once in RTB mode, stay in RTB mode (no flip-flopping)
        self._rtb_mode = self._rtb_mode | should_rtb

    def _update_target_dwell(self, in_frustum: torch.Tensor) -> None:
        """
        Update dwell time for targets currently in frustum.

        Dwell time requirements model real ISR operations where sustained
        observation is needed for quality intelligence collection.

        Args:
            in_frustum: [num_envs, max_targets] boolean mask of targets in camera FOV
        """
        dt = self.cfg.physics_dt * self.cfg.decimation

        # Accumulate dwell time for targets in frustum
        self._target_dwell_time += in_frustum.float() * dt

        # Reset dwell time for targets that left frustum (must be continuous observation)
        left_frustum = ~in_frustum & (self._target_dwell_time > 0)
        self._target_dwell_time[left_frustum] = 0.0

        # Check for quality detection threshold
        newly_quality = (
            (self._target_dwell_time >= self.cfg.targets.min_dwell_time) &
            ~self._target_quality_detected &
            self._target_active
        )
        self._target_quality_detected = self._target_quality_detected | newly_quality

        # Store for reward computation this step
        self._newly_quality_detected = newly_quality

    def _compute_detection_quality(
        self,
        slant_range: torch.Tensor,
        altitude_above_target: torch.Tensor
    ) -> torch.Tensor:
        """
        Compute detection quality multiplier based on altitude and range.

        Models real-world GSD (Ground Sample Distance) limitations:
        - Too high: Can't resolve target details
        - Too far: Inverse-square imaging degradation
        - Optimal: Best balance of coverage and resolution

        Args:
            slant_range: [num_envs, max_targets] distance to targets
            altitude_above_target: [num_envs, max_targets] height above targets

        Returns:
            [num_envs, max_targets] quality multiplier in [0, 1]
        """
        cfg = self.cfg.camera

        # Altitude-based quality (GSD modeling)
        altitude_error = torch.abs(altitude_above_target - cfg.optimal_altitude)
        altitude_quality = 1.0 - altitude_error * cfg.altitude_detection_falloff
        altitude_quality = torch.clamp(altitude_quality, cfg.min_detection_quality, 1.0)

        # Hard cutoff at max detection altitude
        too_high = altitude_above_target > cfg.max_detection_altitude
        altitude_quality[too_high] = 0.0

        # Range-based quality (inverse-square falloff)
        range_ratio = slant_range / cfg.optimal_range
        range_quality = torch.where(
            range_ratio <= 1.0,
            torch.ones_like(range_ratio),  # Full quality at/below optimal
            1.0 / (range_ratio ** cfg.range_detection_falloff)  # Gradual falloff
        )

        # Hard cutoffs for range
        range_quality[slant_range < cfg.min_range] = 0.0
        range_quality[slant_range > cfg.max_range] = 0.0

        # Combined quality (multiply factors)
        combined_quality = altitude_quality * range_quality

        return torch.clamp(combined_quality, 0.0, 1.0)

    def _update_wind(self) -> None:
        """Update wind velocity with random walk and gusts."""
        cfg = self.cfg.world

        # Slowly varying base wind (random walk)
        wind_change = torch.randn_like(self._wind_velocity) * 0.1
        self._wind_velocity += wind_change

        # Clamp to range
        wind_speed = torch.norm(self._wind_velocity, dim=-1, keepdim=True)
        max_speed = cfg.wind_speed_range[1]
        scale = torch.where(
            wind_speed > max_speed,
            max_speed / (wind_speed + 1e-6),
            torch.ones_like(wind_speed)
        )
        self._wind_velocity = self._wind_velocity * scale

        # Random gusts
        gust_trigger = torch.rand((self.num_envs,), device=self.device) < cfg.wind_gust_probability
        if gust_trigger.any():
            gust_direction = torch.randn((self.num_envs, 3), device=self.device)
            gust_direction = gust_direction / (torch.norm(gust_direction, dim=-1, keepdim=True) + 1e-6)
            gust_velocity = gust_direction * cfg.wind_gust_magnitude
            self._wind_velocity[gust_trigger] += gust_velocity[gust_trigger]

    def _spawn_targets_from_tasking(self, env_ids: torch.Tensor) -> None:
        """
        Spawn targets using mission tasking system for realistic NAI-clustered distribution.

        This integrates with environments/mission_tasking.py to generate targets
        based on Named Areas of Interest (NAI) and intelligence requirements.

        OPTIMIZATION NOTE: This function still uses Python loops due to the
        NumPy-based mission_tasking.py. However, we:
        1. Convert env_ids to CPU list once at the start (single sync)
        2. Batch tensor assignments where possible
        3. Pre-compute scenario-independent values

        For maximum performance, mission_tasking.py would need PyTorch rewrite.

        Args:
            env_ids: Environment indices to spawn targets for
        """
        if not _MISSION_TASKING_AVAILABLE:
            # Fall back to random spawning if mission_tasking not available
            self._spawn_targets_random(env_ids)
            return

        scenario = self.cfg.mission.scenario
        bounds = self.cfg.world.bounds
        max_t = self.cfg.targets.max_targets

        # Pre-compute scenario-specific constants (computed once, not per-env)
        if scenario == ScenarioType.AREA_COVERAGE:
            center_x = (bounds[0] + bounds[1]) / 2 + (bounds[1] - bounds[0]) * 0.2
            center_y = (bounds[2] + bounds[3]) / 2 + (bounds[3] - bounds[2]) * 0.2
            radius = min(bounds[1] - bounds[0], bounds[3] - bounds[2]) * 0.3
        elif scenario == ScenarioType.NFZ_AVOIDANCE:
            dest_x = bounds[1] * 0.9
            dest_y = (bounds[2] + bounds[3]) / 2

        # OPTIMIZATION: Convert to CPU list once to avoid repeated .item() calls
        # This is a single GPU->CPU sync instead of one per environment
        env_ids_list = env_ids.tolist() if isinstance(env_ids, torch.Tensor) else list(env_ids)
        n_reset = len(env_ids_list)

        # Pre-allocate batch buffers for accumulating results
        batch_positions = []
        batch_priorities = []
        batch_values = []
        batch_active_counts = []

        # Priority value multipliers (precompute)
        value_mult = {1: 4.0, 2: 2.0, 3: 1.5, 4: 1.0}

        for i, env_idx_int in enumerate(env_ids_list):
            seed = self._episode_seed + i  # Use loop index, not env_idx for determinism

            # Create scenario-appropriate tasking
            if scenario == ScenarioType.AREA_COVERAGE:
                tasking = create_comms_denied_tasking(
                    surveillance_center=(center_x, center_y),
                    surveillance_radius=radius,
                    seed=seed,
                )

            elif scenario == ScenarioType.NFZ_AVOIDANCE:
                tasking = create_dynamic_nfz_tasking(
                    destination=(dest_x, dest_y, 80.0),
                    seed=seed,
                )

                # Set destination for this environment (batch later if needed)
                self._destination[env_idx_int, 0] = dest_x
                self._destination[env_idx_int, 1] = dest_y
                self._destination[env_idx_int, 2] = 80.0

                # Populate NFZ tensors from restricted zones
                self._populate_nfz_from_tasking(env_idx_int, tasking)

            elif scenario == ScenarioType.MULTI_OBJECTIVE:
                tasking = create_multi_objective_tasking(seed=seed)

                # Populate threat zones from restricted zones (soft constraints)
                self._populate_threats_from_tasking(env_idx_int, tasking)

            else:
                # Unknown scenario - use comms denied as default
                tasking = create_comms_denied_tasking(seed=seed)

            # Generate POI distribution from tasking
            pois = generate_poi_distribution_from_tasking(tasking, seed=seed)

            # Build NAI lookup dict for O(1) priority lookup
            nai_lookup = {nai.id: nai.priority.value for nai in tasking.nais} if tasking.nais else {}

            # Collect per-environment results
            env_positions = []
            env_priorities = []
            env_values = []

            for j, poi in enumerate(pois[:max_t]):
                pos = poi["position"]
                env_positions.append([pos[0], pos[1], 0.0])

                # Get priority from NAI lookup
                nai_id = poi.get("nai_id", "UNKNOWN")
                priority = nai_lookup.get(nai_id, 4)  # Default to low priority
                env_priorities.append(priority)
                env_values.append(25.0 * value_mult.get(priority, 1.0))

            # Pad to max_targets
            active_count = len(env_positions)
            while len(env_positions) < max_t:
                env_positions.append([0.0, 0.0, 0.0])
                env_priorities.append(0)
                env_values.append(0.0)

            batch_positions.append(env_positions)
            batch_priorities.append(env_priorities)
            batch_values.append(env_values)
            batch_active_counts.append(active_count)

        # BATCH ASSIGNMENT: Convert lists to tensors and assign all at once
        # This minimizes individual tensor writes
        positions_tensor = torch.tensor(batch_positions, device=self.device, dtype=torch.float32)
        priorities_tensor = torch.tensor(batch_priorities, device=self.device, dtype=torch.int32)
        values_tensor = torch.tensor(batch_values, device=self.device, dtype=torch.float32)

        # Batch write to target tensors
        self._target_positions[env_ids] = positions_tensor
        self._target_priorities[env_ids] = priorities_tensor
        self._target_values[env_ids] = values_tensor

        # Set active flags based on active counts
        for i, (env_idx_int, active_count) in enumerate(zip(env_ids_list, batch_active_counts)):
            if active_count < max_t:
                self._target_active[env_idx_int, active_count:] = False

        # Increment episode seed for next reset
        self._episode_seed += n_reset

    def _spawn_targets_random(self, env_ids: torch.Tensor) -> None:
        """Fall back to random uniform target spawning."""
        n_reset = len(env_ids)
        max_t = self.cfg.targets.max_targets
        bounds = self.cfg.world.bounds

        world_x_range = bounds[1] - bounds[0]
        world_y_range = bounds[3] - bounds[2]

        self._target_positions[env_ids, :, 0] = (
            torch.rand((n_reset, max_t), device=self.device) * world_x_range * 0.8 + bounds[0] * 0.8
        )
        self._target_positions[env_ids, :, 1] = (
            torch.rand((n_reset, max_t), device=self.device) * world_y_range * 0.8 + bounds[2] * 0.8
        )
        self._target_positions[env_ids, :, 2] = 0.0
        self._target_detected[env_ids] = False
        self._target_active[env_ids] = True
        self._target_priorities[env_ids] = 3  # Default MEDIUM priority
        self._target_values[env_ids] = 25.0

    def _populate_nfz_from_tasking(self, env_idx: int, tasking: 'MissionTasking') -> None:
        """
        Populate NFZ tensors from mission tasking restricted zones.

        Args:
            env_idx: Environment index
            tasking: Mission tasking with restricted zones
        """
        max_nfz = self.cfg.nfz.max_nfz
        dt = self.cfg.physics_dt * self.cfg.decimation

        for j, rz in enumerate(tasking.restricted_zones[:max_nfz]):
            self._nfz_centers[env_idx, j, 0] = rz.center[0]
            self._nfz_centers[env_idx, j, 1] = rz.center[1]
            self._nfz_radii[env_idx, j] = rz.radius

            # Convert activation time (seconds) to step count
            if rz.activation_time <= 0:
                # Always active from start
                self._nfz_activation_step[env_idx, j] = 0
                self._nfz_active[env_idx, j] = True
            else:
                # Activates at specified time
                activation_step = int(rz.activation_time / dt)
                self._nfz_activation_step[env_idx, j] = activation_step
                self._nfz_active[env_idx, j] = False

        # Mark remaining NFZ slots as inactive
        num_zones = len(tasking.restricted_zones)
        if num_zones < max_nfz:
            self._nfz_activation_step[env_idx, num_zones:] = -1
            self._nfz_active[env_idx, num_zones:] = False
            self._nfz_radii[env_idx, num_zones:] = 0.0

    def _populate_threats_from_tasking(self, env_idx: int, tasking: 'MissionTasking') -> None:
        """
        Populate threat zone tensors from mission tasking restricted zones.

        For multi-objective scenarios, restricted zones with is_hard_constraint=False
        become MEDIUM threat zones (time-limited exposure), while hard constraints
        become HIGH threat zones (episode termination).

        Args:
            env_idx: Environment index
            tasking: Mission tasking with restricted zones as threat areas
        """
        max_threat = self.cfg.threat.max_threat_zones

        for j, rz in enumerate(tasking.restricted_zones[:max_threat]):
            self._threat_centers[env_idx, j, 0] = rz.center[0]
            self._threat_centers[env_idx, j, 1] = rz.center[1]
            self._threat_radii[env_idx, j] = rz.radius

            # Threat level: HIGH (3) for hard constraints, MEDIUM (2) for soft
            if rz.is_hard_constraint:
                self._threat_levels[env_idx, j] = 3  # HIGH
            else:
                self._threat_levels[env_idx, j] = 2  # MEDIUM

        # Mark remaining slots as inactive (level 0)
        num_zones = len(tasking.restricted_zones)
        if num_zones < max_threat:
            self._threat_levels[env_idx, num_zones:] = 0
            self._threat_radii[env_idx, num_zones:] = 0.0

    def _read_state_from_sim(self) -> None:
        """Read drone state from simulation."""
        # Get state from RigidPrimView (returns torch tensors with GPU backend, numpy otherwise)
        pos, quat = self._robot_view.get_world_poses()
        vel = self._robot_view.get_velocities()

        # Backend detection is done in _spawn_drones, but handle edge case
        if self._use_gpu_tensors is None:
            self._use_gpu_tensors = not isinstance(pos, np.ndarray)

        # Handle both torch tensors and numpy arrays
        if not self._use_gpu_tensors:
            self._drone_pos = torch.from_numpy(pos).to(self.device)
            self._drone_quat = torch.from_numpy(quat).to(self.device)
            self._drone_vel = torch.from_numpy(vel[:, :3]).to(self.device)
            self._drone_ang_vel = torch.from_numpy(vel[:, 3:]).to(self.device)
        else:
            # Already torch tensors from GPU backend - avoid .to() if already on device
            self._drone_pos = pos if pos.device == self.device else pos.to(self.device)
            self._drone_quat = quat if quat.device == self.device else quat.to(self.device)
            self._drone_vel = vel[:, :3] if vel.device == self.device else vel[:, :3].to(self.device)
            self._drone_ang_vel = vel[:, 3:] if vel.device == self.device else vel[:, 3:].to(self.device)

    def _write_state_to_sim(self, env_ids: torch.Tensor) -> None:
        """Write drone state to simulation."""
        # Backend detection is done in _spawn_drones, but handle edge case
        if self._use_gpu_tensors is None:
            pos, _ = self._robot_view.get_world_poses()
            self._use_gpu_tensors = not isinstance(pos, np.ndarray)

        velocities = torch.cat([
            self._drone_vel[env_ids],
            self._drone_ang_vel[env_ids]
        ], dim=-1)

        if self._use_gpu_tensors:
            # GPU backend - pass torch tensors directly
            self._robot_view.set_world_poses(
                positions=self._drone_pos[env_ids],
                orientations=self._drone_quat[env_ids],
                indices=env_ids,
            )
            self._robot_view.set_velocities(velocities, indices=env_ids)
        else:
            # CPU backend - convert to numpy
            env_ids_np = env_ids.cpu().numpy()
            self._robot_view.set_world_poses(
                positions=self._drone_pos[env_ids].cpu().numpy(),
                orientations=self._drone_quat[env_ids].cpu().numpy(),
                indices=env_ids_np,
            )
            self._robot_view.set_velocities(velocities.cpu().numpy(), indices=env_ids_np)

    def _get_observations(self) -> torch.Tensor:
        """
        Compute observations including frustum-filtered targets and mission state.

        All observations are normalized to approximately [-1, 1] or [0, 1] range
        for stable RL training.

        Normalization scales:
        - Position: /500.0 (world size ~500m)
        - Velocity: /20.0 (max expected ~20 m/s)
        - Angular velocity: /10.0 (max expected ~10 rad/s)
        - Gimbal angles: /pi (radians)
        - Target positions: /200.0 (max frustum range)

        Phase 5 additions:
        - Observation noise (5.7) for sim-to-real robustness
        - RTB state (5.1) for battery-distance feasibility awareness

        Returns:
            [num_envs, obs_dim] tensor containing:
            - Drone state (position, velocity, orientation, angular velocity): 13
            - Gimbal state (pitch, yaw): 2
            - Visible targets (relative position, priority, value, in_frustum): max_targets * 6
            - Mission state (coverage, battery, time, home): 6
            - RTB state (mode, threshold): 2
            - NFZ state (if enabled): 7
            - Threat state (if enabled): 6
        """
        # Compute camera pose from drone pose + gimbal
        cam_pose = self._compute_camera_pose()

        # Filter targets by frustum
        visible_targets, in_frustum = self._frustum_filter_targets(cam_pose)

        # =================================================================
        # Apply observation noise (5.7) for sim-to-real robustness
        # =================================================================
        cfg_w = self.cfg.world

        # Position with GPS-like noise
        if cfg_w.position_noise_std > 0:
            pos_noise = torch.randn_like(self._drone_pos) * cfg_w.position_noise_std
            noisy_pos = self._drone_pos + pos_noise
        else:
            noisy_pos = self._drone_pos

        # Velocity with IMU-like noise
        if cfg_w.velocity_noise_std > 0:
            vel_noise = torch.randn_like(self._drone_vel) * cfg_w.velocity_noise_std
            noisy_vel = self._drone_vel + vel_noise
        else:
            noisy_vel = self._drone_vel

        # Angular velocity with gyro-like noise
        if cfg_w.attitude_noise_std > 0:
            ang_vel_noise = torch.randn_like(self._drone_ang_vel) * cfg_w.attitude_noise_std * 10.0
            noisy_ang_vel = self._drone_ang_vel + ang_vel_noise
        else:
            noisy_ang_vel = self._drone_ang_vel

        # Compute home direction and distance (XY only) - use noisy position
        home_vec = self._launch_position[:2].unsqueeze(0) - noisy_pos[:, :2]
        home_distance = torch.norm(home_vec, dim=-1, keepdim=True)
        home_direction = home_vec / (home_distance + 1e-6)  # Normalize, avoid div by zero

        # Mission time ratio (0 to 1)
        max_time = self.cfg.mission.max_episode_steps * self.cfg.physics_dt * self.cfg.decimation
        time_ratio = (self._mission_time / max_time).unsqueeze(-1)

        # Build observation tensor
        # NORMALIZED OBSERVATIONS with noise applied
        obs_parts = [
            # Drone state (13) - all normalized, with noise
            noisy_pos / 500.0,                         # 3 - position normalized by world size
            noisy_vel / 20.0,                          # 3 - velocity normalized by max expected
            self._drone_quat,                          # 4 - quaternion already in [-1, 1]
            noisy_ang_vel / 10.0,                      # 3 - angular velocity normalized

            # Gimbal state (2) - normalized to [-1, 1]
            self._gimbal_pitch.unsqueeze(-1) / np.pi,  # 1
            self._gimbal_yaw.unsqueeze(-1) / np.pi,    # 1

            # Target observations (max_targets * 6) - normalized
            visible_targets.view(self.num_envs, -1) / 200.0,  # max_targets * 3 - normalized by max range
            (self._target_priorities.float() / 4.0).view(self.num_envs, -1),  # max_targets - priority [0, 1]
            (self._target_values / 100.0).view(self.num_envs, -1),  # max_targets - value normalized
            in_frustum.float().view(self.num_envs, -1),  # max_targets - binary

            # Mission state (6) - already normalized
            self._coverage_pct.unsqueeze(-1) / 100.0,  # 1 - normalized 0-1
            self._battery_pct.unsqueeze(-1) / 100.0,   # 1 - normalized 0-1
            time_ratio,                                # 1 - normalized 0-1
            home_direction,                            # 2 - unit vector
            home_distance / 500.0,                     # 1 - normalized by typical world size

            # RTB state (5.1) - 2 dims
            self._rtb_mode.float().unsqueeze(-1),                    # 1 - binary RTB mode flag
            self._rtb_battery_threshold.unsqueeze(-1) / 100.0,       # 1 - battery threshold normalized
        ]

        # Add NFZ observations if enabled (7 dims)
        if self.cfg.nfz.enabled:
            # Destination direction and distance
            dest_vec = self._destination - self._drone_pos
            dest_dist = torch.norm(dest_vec, dim=-1, keepdim=True)
            dest_direction = dest_vec / (dest_dist + 1e-6)

            # NFZ buffer check
            in_buffer = (self._nearest_nfz_dist < self.cfg.nfz.nfz_buffer_distance) & \
                       (self._nearest_nfz_dist >= 0)

            obs_parts.extend([
                dest_direction,                                    # 3
                dest_dist / 1000.0,                               # 1 - normalized
                self._nearest_nfz_dist.unsqueeze(-1) / 500.0,     # 1 - normalized
                in_buffer.float().unsqueeze(-1),                  # 1
                self._nfz_active.float().sum(dim=-1, keepdim=True) / self.cfg.nfz.max_nfz,  # 1
            ])

        # Add threat observations if enabled (6 dims)
        if self.cfg.threat.enabled:
            max_exposure = self.cfg.threat.max_medium_exposure
            max_value = self._target_values.sum(dim=-1, keepdim=True)

            obs_parts.extend([
                self._collected_value.unsqueeze(-1) / (max_value + 1e-6),  # 1 - value ratio
                (self._target_priorities == 1).float().sum(dim=-1, keepdim=True) / max_t,  # 1 - critical remaining
                self._threat_exposure.unsqueeze(-1) / max_exposure,  # 1 - exposure ratio
                self._current_threat_level.float().unsqueeze(-1) / 3.0,  # 1 - current threat
                self._battery_pct.unsqueeze(-1) / 100.0,  # 1 - battery budget (duplicate for emphasis)
                (self._collected_value / (100.0 - self._battery_pct + 1e-6)).unsqueeze(-1),  # 1 - efficiency
            ])

        return torch.cat(obs_parts, dim=-1)

    def _compute_camera_pose(self) -> torch.Tensor:
        """
        Compute 4x4 camera-to-world transformation matrices.

        Following the same approach as perception/frustum.py:
        - Build camera pose as 4x4 matrix
        - Camera convention: +Z forward, +X right, +Y down

        Returns:
            cam_pose: [num_envs, 4, 4] camera-to-world transforms
        """
        n = self.num_envs

        # Camera offset from drone CoG (belly-mounted gimbal)
        cam_offset_body = torch.tensor([0.1, 0.0, -0.05], device=self.device)

        # Drone rotation matrix from quaternion
        drone_rot = self._quat_to_rotation_matrix(self._drone_quat)  # [n, 3, 3]

        # Gimbal rotation (pitch then yaw)
        gimbal_rot = self._gimbal_rotation_matrix(self._gimbal_pitch, self._gimbal_yaw)  # [n, 3, 3]

        # Camera rotation = drone rotation * gimbal rotation
        cam_rot = torch.bmm(drone_rot, gimbal_rot)  # [n, 3, 3]

        # Camera position in world frame
        cam_offset_batched = cam_offset_body.view(1, 3, 1).expand(n, 3, 1)
        cam_offset_world = torch.bmm(drone_rot, cam_offset_batched).squeeze(-1)
        cam_pos = self._drone_pos + cam_offset_world  # [n, 3]

        # Build 4x4 camera-to-world matrix
        cam_pose = torch.zeros((n, 4, 4), device=self.device)
        cam_pose[:, :3, :3] = cam_rot
        cam_pose[:, :3, 3] = cam_pos
        cam_pose[:, 3, 3] = 1.0

        return cam_pose

    def _frustum_filter_targets(
        self,
        cam_pose: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Filter targets to only include those visible in camera frustum.

        Following the same approach as perception/frustum.py:
        - Use 4x4 matrix inversion for world-to-camera transform
        - Camera convention: +Z forward, +X right, +Y down

        Args:
            cam_pose: [num_envs, 4, 4] camera-to-world transforms

        Returns:
            rel_positions: [num_envs, max_targets, 3] relative positions in camera frame
            in_frustum: [num_envs, max_targets] boolean mask
        """
        n = self.num_envs
        max_t = self.cfg.targets.max_targets

        # Invert camera pose to get world-to-camera transform
        # For rigid transforms: inv([R|t]) = [R^T | -R^T @ t]
        cam_rot = cam_pose[:, :3, :3]  # [n, 3, 3]
        cam_pos = cam_pose[:, :3, 3]   # [n, 3]
        cam_rot_inv = cam_rot.transpose(1, 2)  # [n, 3, 3]

        # Transform target positions to camera frame
        # point_cam = R^T @ (point_world - cam_pos)
        rel_pos_world = self._target_positions - cam_pos.unsqueeze(1)  # [n, max_t, 3]

        # Batched rotation: rel_pos_cam = rel_pos_world @ R^T
        # Using einsum: rel_pos_cam[n,t,i] = sum_j rel_pos_world[n,t,j] * R^T[n,j,i]
        rel_pos_cam = torch.einsum('ntj,nji->nti', rel_pos_world, cam_rot_inv)

        # In camera frame: +Z is forward, +X is right, +Y is down
        # Check if target is in front of camera
        z = rel_pos_cam[:, :, 2]
        in_front = z > 0

        # Horizontal angle
        x = rel_pos_cam[:, :, 0]
        angle_h = torch.atan2(x, z)
        in_h_fov = torch.abs(angle_h) < self._half_fov_h

        # Vertical angle
        y = rel_pos_cam[:, :, 1]
        angle_v = torch.atan2(y, z)
        in_v_fov = torch.abs(angle_v) < self._half_fov_v

        # Range check (distance is same in camera frame as world frame due to rotation invariance)
        # Using camera frame for consistency with FOV checks
        distance = torch.norm(rel_pos_cam, dim=-1)
        in_range = (distance >= self.cfg.camera.min_range) & (distance <= self.cfg.camera.max_range)

        # Combine all checks
        in_frustum = in_front & in_h_fov & in_v_fov & in_range & self._target_active

        # Mark newly detected targets and store for reward computation
        self._newly_detected = in_frustum & ~self._target_detected
        self._target_detected = self._target_detected | in_frustum

        # Update dwell time tracking (5.2)
        self._update_target_dwell(in_frustum)

        return rel_pos_cam, in_frustum

    def _compute_rewards(self, actions: torch.Tensor) -> torch.Tensor:
        """
        Compute rewards for all environments.

        This is the full implementation using config values and scenario-specific rewards:
        - Priority-weighted detection rewards (P1=4x, P2=2x, P3=1.5x, P4=1x)
        - Coverage progress rewards (Scenario 1)
        - Destination progress rewards (Scenario 2)
        - NFZ violation/buffer penalties (Scenario 2)
        - Threat zone penalties (Scenario 3)
        - Flight quality penalties (altitude, velocity)

        Phase 5 additions:
        - RTB progress rewards (5.1)
        - Dwell time rewards (5.2)
        - Action smoothness penalties (5.3)
        - Detection quality scaling by altitude/range (5.4, 5.5)

        Args:
            actions: Current step actions for smoothness penalty calculation
        """
        rewards = torch.zeros((self.num_envs,), device=self.device)
        cfg_r = self.cfg.rewards
        scenario = self.cfg.mission.scenario
        dt = self.cfg.physics_dt * self.cfg.decimation

        # Priority weights: 0=inactive, P1=4x, P2=2x, P3=1.5x, P4=1x
        priority_weights = torch.tensor(
            [0.0, 4.0, 2.0, 1.5, 1.0], device=self.device
        )

        # Get priority weights for each target [num_envs, max_targets]
        target_priority_weights = priority_weights[self._target_priorities]

        # =================================================================
        # 1. Priority-weighted detection rewards with quality scaling (5.4, 5.5)
        # =================================================================

        # Compute detection quality based on altitude and range
        drone_altitude = self._drone_pos[:, 2].unsqueeze(-1)  # [N, 1]
        target_altitude = drone_altitude - self._target_positions[:, :, 2]  # Height above target

        # Compute slant range to each target
        rel_pos = self._target_positions - self._drone_pos.unsqueeze(1)  # [N, max_t, 3]
        slant_range = torch.norm(rel_pos, dim=-1)  # [N, max_t]

        # Get detection quality multiplier (0-1)
        detection_quality = self._compute_detection_quality(slant_range, target_altitude)

        # Weighted detection reward for newly detected targets (scaled by quality)
        detection_values = target_priority_weights * self._newly_detected.float() * detection_quality
        rewards += detection_values.sum(dim=-1) * cfg_r.target_detection

        # Tracking reward for targets currently in frustum
        tracking_values = self._target_detected.float()
        rewards += tracking_values.sum(dim=-1) * cfg_r.target_tracking

        # =================================================================
        # 2. Dwell time rewards (5.2)
        # =================================================================
        # Continuous reward while tracking targets in frustum
        # Note: _newly_detected is computed in _frustum_filter_targets
        # We also call _update_target_dwell there

        # Dwell reward: reward sustained observation
        dwell_reward = self._target_dwell_time.sum(dim=-1) * self.cfg.targets.dwell_reward_rate * dt * 0.1
        rewards += dwell_reward

        # Quality detection bonus (one-time, priority-weighted)
        quality_values = target_priority_weights * self._newly_quality_detected.float()
        rewards += quality_values.sum(dim=-1) * self.cfg.targets.quality_detection_bonus

        # =================================================================
        # 3. Scenario-specific rewards
        # =================================================================

        if scenario == ScenarioType.AREA_COVERAGE:
            # Coverage progress reward (delta coverage)
            coverage_delta = self._coverage_pct - self._prev_coverage_pct
            rewards += coverage_delta * cfg_r.coverage_progress

        elif scenario == ScenarioType.NFZ_AVOIDANCE:
            # Progress-to-destination reward (getting closer)
            has_dest = self._destination.abs().sum(dim=-1) > 0

            current_dist = torch.norm(
                self._drone_pos - self._destination, dim=-1
            )

            distance_delta = self._prev_dest_distance - current_dist
            rewards += has_dest.float() * distance_delta * cfg_r.progress_to_dest

            self._prev_dest_distance = current_dist.clone()

            # Destination reached bonus (within 20m)
            dest_reached = has_dest & (current_dist < 20.0)
            rewards += dest_reached.float() * cfg_r.dest_reached

        elif scenario == ScenarioType.MULTI_OBJECTIVE:
            # Coverage still matters for multi-objective
            coverage_delta = self._coverage_pct - self._prev_coverage_pct
            rewards += coverage_delta * cfg_r.coverage_progress * 0.5

            # Efficiency bonus
            value_collected = (
                target_priority_weights * self._target_detected.float()
            ).sum(dim=-1)
            battery_spent = 100.0 - self._battery_pct
            efficiency = value_collected / (battery_spent.clamp(min=1.0))
            rewards += efficiency * cfg_r.efficiency_bonus * 0.01

        # =================================================================
        # 4. RTB rewards (5.1)
        # =================================================================
        # Compute home distance
        home_distance = torch.norm(
            self._drone_pos[:, :2] - self._launch_position[:2].unsqueeze(0),
            dim=-1
        )

        # RTB progress reward (when in RTB mode, reward getting closer to home)
        rtb_progress = self._prev_home_distance - home_distance
        rewards += self._rtb_mode.float() * rtb_progress * self.cfg.mission.rtb_progress_reward

        # Update prev home distance
        self._prev_home_distance = home_distance.clone()

        # RTB success (reached home with battery remaining)
        home_reached = home_distance < 10.0  # 10m threshold
        rtb_success = self._rtb_mode & home_reached & (self._battery_pct > 0)
        rewards += rtb_success.float() * self.cfg.mission.rtb_reward

        # =================================================================
        # 5. NFZ penalties (Scenario 2)
        # =================================================================
        if self.cfg.nfz.enabled:
            rewards += self._in_nfz.float() * cfg_r.nfz_violation

            in_buffer = (
                (self._nearest_nfz_dist < self.cfg.nfz.nfz_buffer_distance) &
                ~self._in_nfz
            )
            rewards += in_buffer.float() * cfg_r.nfz_buffer

        # =================================================================
        # 6. Threat penalties (Scenario 3)
        # =================================================================
        if self.cfg.threat.enabled:
            in_medium = (self._current_threat_level == 2)
            rewards += in_medium.float() * self.cfg.threat.medium_threat_rate * dt

            exposure_warning_threshold = 0.8 * self.cfg.threat.max_medium_exposure
            near_limit = self._threat_exposure > exposure_warning_threshold
            rewards += near_limit.float() * cfg_r.exposure_warning * dt

        # =================================================================
        # 7. Flight quality penalties
        # =================================================================
        # Height penalty
        height_error = torch.abs(self._drone_pos[:, 2] - cfg_r.target_altitude)
        rewards -= cfg_r.height_penalty * height_error

        # Velocity penalty
        vel_magnitude = torch.norm(self._drone_vel, dim=-1)
        rewards -= cfg_r.velocity_penalty * vel_magnitude

        # =================================================================
        # 8. Action smoothness penalties (5.3)
        # =================================================================
        action_delta = torch.abs(actions - self._prev_actions)

        # Velocity command smoothness (actions 0-3)
        vel_jerk = action_delta[:, :4].sum(dim=-1)
        rewards -= vel_jerk * cfg_r.action_smoothness_penalty

        # Gimbal smoothness (actions 4-5) - higher penalty for imagery quality
        gimbal_jerk = action_delta[:, 4:6].sum(dim=-1)
        rewards -= gimbal_jerk * cfg_r.gimbal_smoothness_penalty

        # Store actions for next step
        self._prev_actions = actions.clone()

        # =================================================================
        # 9. Battery critical penalty
        # =================================================================
        battery_critical = self._battery_pct < self.cfg.mission.battery_reserve_pct
        rewards += battery_critical.float() * cfg_r.battery_critical * 0.01

        return rewards

    def _add_terminal_rewards(
        self,
        rewards: torch.Tensor,
        term_info: Dict[str, torch.Tensor]
    ) -> torch.Tensor:
        """
        Add one-time terminal rewards/penalties based on episode outcome.

        Args:
            rewards: Current step rewards
            term_info: Dictionary with termination flags from _check_termination

        Returns:
            Updated rewards tensor with terminal bonuses/penalties
        """
        cfg_r = self.cfg.rewards

        # Mission success bonus
        rewards += term_info["mission_success"].float() * cfg_r.mission_success

        # Crash penalty
        crash_mask = term_info["crashed"] | term_info["out_of_bounds"]
        rewards += crash_mask.float() * cfg_r.crash

        # NFZ violation penalty (if terminating due to NFZ)
        rewards += term_info["nfz_violation"].float() * cfg_r.nfz_violation

        # High threat zone penalty
        if self.cfg.threat.enabled:
            rewards += term_info["high_threat"].float() * self.cfg.threat.high_threat_reward

        # Battery depleted penalty (didn't make it back)
        rewards += term_info["battery_depleted"].float() * cfg_r.crash * 0.5  # Half crash penalty

        # Divergence penalty (simulation instability)
        rewards += term_info["diverged"].float() * cfg_r.crash  # Same as crash

        return rewards

    def _check_termination(self) -> Tuple[torch.Tensor, Dict[str, torch.Tensor]]:
        """
        Check termination conditions.

        Returns:
            Tuple of (done mask, termination_info dict with specific flags)
        """
        # Crash (too low)
        crashed = self._drone_pos[:, 2] < self.cfg.crash_height

        # Out of bounds
        bounds = self.cfg.world_bounds
        out_of_bounds = (
            (self._drone_pos[:, 0] < bounds[0]) |
            (self._drone_pos[:, 0] > bounds[1]) |
            (self._drone_pos[:, 1] < bounds[2]) |
            (self._drone_pos[:, 1] > bounds[3]) |
            (self._drone_pos[:, 2] > bounds[5])
        )

        # Timeout
        timeout = self._step_count >= self.cfg.max_episode_steps

        # Battery depleted
        battery_depleted = self._battery_pct <= 0.0

        # Mission success conditions (scenario-specific)
        scenario = self.cfg.mission.scenario
        mission_success = torch.zeros(
            (self.num_envs,), dtype=torch.bool, device=self.device
        )

        if scenario == ScenarioType.AREA_COVERAGE:
            # Success: coverage target met AND battery above reserve
            mission_success = (
                (self._coverage_pct >= self.cfg.mission.target_coverage_pct) &
                (self._battery_pct >= self.cfg.mission.battery_reserve_pct)
            )
        elif scenario == ScenarioType.NFZ_AVOIDANCE:
            # Success: destination reached (checked via distance)
            has_dest = self._destination.abs().sum(dim=-1) > 0
            dest_distance = torch.norm(
                self._drone_pos - self._destination, dim=-1
            )
            mission_success = has_dest & (dest_distance < 20.0)
        elif scenario == ScenarioType.MULTI_OBJECTIVE:
            # Success: all P1 targets detected OR coverage + battery constraints met
            p1_mask = self._target_priorities == 1  # Priority 1 targets
            p1_detected = (self._target_detected & p1_mask).all(dim=-1)
            mission_success = p1_detected

        # High threat termination (Scenario 3)
        high_threat = torch.zeros(
            (self.num_envs,), dtype=torch.bool, device=self.device
        )
        if self.cfg.threat.enabled and self.cfg.threat.high_threat_terminates:
            high_threat = self._current_threat_level == 3

        # NFZ violation termination (optional - could be just penalty)
        nfz_violation = torch.zeros(
            (self.num_envs,), dtype=torch.bool, device=self.device
        )
        if self.cfg.nfz.enabled:
            nfz_violation = self._in_nfz

        # Divergence detection: catch NaN values or simulation explosions
        # This prevents training on corrupted data
        diverged = (
            torch.isnan(self._drone_pos).any(dim=-1) |
            torch.isnan(self._drone_vel).any(dim=-1) |
            torch.isnan(self._drone_quat).any(dim=-1) |
            (torch.norm(self._drone_vel, dim=-1) > 100.0) |  # Velocity explosion
            (torch.norm(self._drone_ang_vel, dim=-1) > 50.0)  # Angular velocity explosion
        )

        # Combine all termination conditions
        done = (
            crashed | out_of_bounds | timeout | battery_depleted |
            mission_success | high_threat | nfz_violation | diverged
        )

        # Return detailed termination info for reward shaping
        term_info = {
            "crashed": crashed,
            "out_of_bounds": out_of_bounds,
            "timeout": timeout,
            "battery_depleted": battery_depleted,
            "mission_success": mission_success,
            "high_threat": high_threat,
            "nfz_violation": nfz_violation,
            "diverged": diverged,
        }

        return done, term_info

    # Utility functions
    def _quat_to_rotation_matrix(self, quat: torch.Tensor) -> torch.Tensor:
        """
        Convert quaternion to rotation matrix [3,3].

        IMPORTANT: Isaac Sim returns quaternions in [x,y,z,w] order (Hamilton convention).
        This function expects that convention.

        Args:
            quat: [num_envs, 4] quaternion tensor in [x, y, z, w] order

        Returns:
            [num_envs, 3, 3] rotation matrices
        """
        # Isaac Sim convention: [x, y, z, w]
        x, y, z, w = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]

        # Rotation matrix elements
        r00 = 1 - 2*(y*y + z*z)
        r01 = 2*(x*y - w*z)
        r02 = 2*(x*z + w*y)
        r10 = 2*(x*y + w*z)
        r11 = 1 - 2*(x*x + z*z)
        r12 = 2*(y*z - w*x)
        r20 = 2*(x*z - w*y)
        r21 = 2*(y*z + w*x)
        r22 = 1 - 2*(x*x + y*y)

        rot = torch.stack([
            torch.stack([r00, r01, r02], dim=-1),
            torch.stack([r10, r11, r12], dim=-1),
            torch.stack([r20, r21, r22], dim=-1),
        ], dim=1)

        return rot

    def _gimbal_rotation_matrix(
        self,
        pitch: torch.Tensor,
        yaw: torch.Tensor
    ) -> torch.Tensor:
        """
        Compute gimbal rotation matrix from pitch and yaw angles.

        Args:
            pitch: [num_envs] pitch angles (radians)
            yaw: [num_envs] yaw angles (radians)

        Returns:
            [num_envs, 3, 3] rotation matrices
        """
        n = pitch.shape[0]

        # Gimbal rotation for camera mounted on drone:
        #
        # Isaac Sim body frame: +X forward, +Y left, +Z up
        # Camera frame convention: +X right, +Y down, +Z forward (optical axis)
        #
        # Step 1: Base rotation to align camera frame with body frame
        #   - Camera +Z (forward) should align with body +X (drone forward)
        #   - Camera +Y (down) should align with body -Z (world down)
        #   - Camera +X (right) should align with body -Y (world right)
        # This is Ry(-90°) @ Rx(90°) or equivalently a rotation that maps:
        #   cam_X -> -body_Y, cam_Y -> -body_Z, cam_Z -> body_X
        #
        # Step 2: Apply gimbal pitch (negative = looking down)
        #   - Pitch rotates around camera X axis
        #   - Negative pitch tilts camera forward axis down
        #
        # Combined rotation matrix for camera-to-body:
        # R_base maps camera axes to body axes for forward-looking camera
        # Then R_pitch applies the gimbal pitch

        # Base rotation: camera forward (+Z) along body forward (+X)
        # This is a -90° rotation around Y then considerations for other axes
        # Simpler: directly construct the rotation that gives us:
        #   body = R_base @ camera
        # where camera (1,0,0) -> body (0,-1,0)  [cam right -> body left... no, right]
        #       camera (0,1,0) -> body (0,0,-1)  [cam down -> body down]
        #       camera (0,0,1) -> body (1,0,0)   [cam forward -> body forward]
        R_base = torch.zeros((n, 3, 3), device=self.device)
        R_base[:, 0, 2] = 1   # body_x = cam_z (forward)
        R_base[:, 1, 0] = -1  # body_y = -cam_x (left = -right)
        R_base[:, 2, 1] = -1  # body_z = -cam_y (up = -down)

        # Pitch rotation around camera X axis (which becomes -body_Y after base rotation)
        # For pitch < 0 (looking down), rotate camera +Z toward camera +Y
        # Rx(theta): [[1,0,0], [0,cos,-sin], [0,sin,cos]]
        # With negative pitch, we want forward to tilt down
        cp, sp = torch.cos(pitch), torch.sin(pitch)
        R_pitch = torch.zeros((n, 3, 3), device=self.device)
        R_pitch[:, 0, 0] = 1
        R_pitch[:, 1, 1] = cp
        R_pitch[:, 1, 2] = -sp
        R_pitch[:, 2, 1] = sp
        R_pitch[:, 2, 2] = cp

        # Yaw rotation around camera Z axis (forward axis)
        cy, sy = torch.cos(yaw), torch.sin(yaw)
        R_yaw = torch.zeros((n, 3, 3), device=self.device)
        R_yaw[:, 0, 0] = cy
        R_yaw[:, 0, 1] = -sy
        R_yaw[:, 1, 0] = sy
        R_yaw[:, 1, 1] = cy
        R_yaw[:, 2, 2] = 1

        # Combined: base @ pitch @ yaw (apply yaw, then pitch, then base)
        # This gives camera-to-body rotation
        R_gimbal = torch.bmm(R_base, torch.bmm(R_pitch, R_yaw))
        return R_gimbal

    @property
    def observation_dim(self) -> int:
        """
        Dimension of observation space.

        Observation structure:
        - Drone state: 13 (pos:3 + vel:3 + quat:4 + ang_vel:3)
        - Gimbal: 2 (pitch + yaw)
        - Per-target: 6 (rel_pos:3 + priority:1 + value:1 + in_frustum:1) x max_targets
        - Mission: 6 (coverage + battery + time + home_dir:2 + home_dist)
        - RTB state (5.1): 2 (rtb_mode + rtb_threshold)
        - NFZ (if enabled): 7
        - Threat (if enabled): 6
        """
        # Base: 13 (drone) + 2 (gimbal) + 6 (mission) + 2 (RTB) = 23
        # Per-target: 6 dims (rel_pos:3 + priority:1 + value:1 + in_frustum:1)
        base_dim = 23 + self.cfg.targets.max_targets * 6

        # Add NFZ observations if enabled
        if self.cfg.nfz.enabled:
            base_dim += 7

        # Add threat observations if enabled
        if self.cfg.threat.enabled:
            base_dim += 6

        return base_dim

    @property
    def action_dim(self) -> int:
        """
        Dimension of action space (velocity commands).

        Action space:
            [0]: vx - horizontal velocity X (scaled by max_horizontal_vel)
            [1]: vy - horizontal velocity Y (scaled by max_horizontal_vel)
            [2]: vz - vertical velocity (scaled by max_vertical_vel)
            [3]: yaw_rate - yaw angular velocity (scaled by max_yaw_rate)
            [4]: gimbal_pitch_rate - gimbal pitch rate command
            [5]: gimbal_yaw_rate - gimbal yaw rate command

        This matches the PX4/ArduPilot velocity command interface, making
        sim-to-real transfer straightforward.
        """
        return 6  # vx, vy, vz, yaw_rate, gimbal_pitch_rate, gimbal_yaw_rate

    def close(self) -> None:
        """Clean up resources."""
        if self._initialized:
            self.world.stop()
            self.simulation_app.close()
            self._initialized = False
