#!/usr/bin/env python3
"""
Direct Dynamics Environment for Fast RL Training

Phase 2 implementation: Bypasses PX4 SITL to apply forces/torques directly to drone rigid body.
This removes the ~250Hz MAVLink/PX4 overhead that limits training speed.

Key differences from PX4-based environment:
- No MAVLink communication
- No PX4 lockstep synchronization
- Direct force/torque application to Isaac Sim rigid body
- Physics runs as fast as GPU allows
- Target: 500+ FPS without rendering, 200+ FPS with render_interval=4

Usage:
    from training.environments.direct_dynamics_env import DirectDynamicsEnv, DirectDynamicsConfig

    config = DirectDynamicsConfig()
    env = DirectDynamicsEnv(config)
    env.setup()

    obs = env.reset()
    for _ in range(1000):
        action = np.random.uniform(-1, 1, 4)
        obs, reward, done, info = env.step(action)

Author: Finley Holt
"""

import sys
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from enum import Enum

# Add paths for imports
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)


@dataclass
class QuadrotorDynamicsParams:
    """Physical parameters for simplified quadrotor dynamics."""
    # Mass and inertia (F-11 ISR camera variant)
    mass: float = 6.3  # kg
    inertia_xx: float = 0.115  # kg*m^2
    inertia_yy: float = 0.115
    inertia_zz: float = 0.175

    # Geometry
    arm_length: float = 0.195  # m (motor-to-motor)

    # Motor/propeller characteristics
    k_thrust: float = 8.54858e-06  # thrust coefficient (N/(rad/s)^2)
    k_torque: float = 1.3677e-07   # torque coefficient (N*m/(rad/s)^2)
    max_rpm: float = 10000.0       # maximum motor RPM
    min_rpm: float = 1000.0        # minimum motor RPM (idle)

    # Aerodynamic drag
    drag_coeff_xy: float = 0.25    # horizontal drag coefficient
    drag_coeff_z: float = 0.15     # vertical drag coefficient

    # Gravity
    gravity: float = 9.81  # m/s^2

    @property
    def inertia_matrix(self) -> np.ndarray:
        return np.diag([self.inertia_xx, self.inertia_yy, self.inertia_zz])

    @property
    def hover_rpm(self) -> float:
        """RPM required to hover (all motors equal)."""
        hover_thrust_per_motor = (self.mass * self.gravity) / 4.0
        omega = np.sqrt(hover_thrust_per_motor / self.k_thrust)
        return omega * 60.0 / (2.0 * np.pi)


class ActionMode(Enum):
    """Action space modes for direct dynamics."""
    MOTOR_SPEEDS = "motor_speeds"      # [m1, m2, m3, m4] normalized motor speeds
    THRUST_RATES = "thrust_rates"      # [thrust, roll_rate, pitch_rate, yaw_rate]
    VELOCITY = "velocity"              # [vx, vy, vz, yaw_rate] velocity commands


@dataclass
class DirectDynamicsConfig:
    """Configuration for direct dynamics environment."""
    # Simulation parameters
    headless: bool = True
    physics_dt: float = 1.0 / 250.0  # 250 Hz physics
    render_interval: int = 4          # Render every Nth frame

    # Action space
    action_mode: ActionMode = ActionMode.THRUST_RATES

    # Dynamics parameters
    dynamics: QuadrotorDynamicsParams = field(default_factory=QuadrotorDynamicsParams)

    # Domain randomization
    position_noise_std: float = 0.0     # Position noise per step
    velocity_noise_std: float = 0.0     # Velocity noise per step
    mass_variation: float = 0.0         # Mass variation factor (0.1 = +/-10%)

    # Action limits for THRUST_RATES mode
    max_thrust: float = 2.0             # Max thrust as multiple of hover thrust
    max_roll_rate: float = 180.0        # deg/s
    max_pitch_rate: float = 180.0       # deg/s
    max_yaw_rate: float = 90.0          # deg/s

    # Action limits for VELOCITY mode
    max_velocity_xy: float = 10.0       # m/s
    max_velocity_z: float = 5.0         # m/s
    velocity_response_time: float = 0.5 # Time constant for velocity tracking (s)

    # Environment bounds
    world_bounds: Tuple[float, float, float, float, float, float] = (-100, 100, -100, 100, 0, 150)
    spawn_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 50.0]))
    spawn_heading: float = 0.0  # radians

    # Perception
    camera_enabled: bool = True
    camera_resolution: Tuple[int, int] = (640, 480)
    camera_fov: float = 90.0
    gimbal_tilt: float = -30.0  # degrees

    # World generation
    terrain_size: Tuple[float, float] = (150.0, 150.0)
    tree_density: float = 0.01
    vehicle_count: int = 8

    # Termination
    max_episode_steps: int = 10000


class DirectDynamicsEnv:
    """
    Fast training environment with direct dynamics control.

    Bypasses PX4 SITL by directly applying forces and torques to the drone's
    rigid body in Isaac Sim. This removes MAVLink overhead and allows the
    simulation to run as fast as the GPU can compute physics.

    Supports three action modes:
    1. MOTOR_SPEEDS: Direct motor speed commands [m1, m2, m3, m4]
    2. THRUST_RATES: Collective thrust + angular rates [T, p, q, r]
    3. VELOCITY: Velocity commands with internal rate controller [vx, vy, vz, yaw_rate]
    """

    # Motor positions relative to CoG (X-configuration)
    # Order: FR, BL, FL, BR (matching PX4 motor numbering)
    MOTOR_POSITIONS = np.array([
        [+0.195, -0.195, 0.0],  # Motor 0: Front-Right (CW)
        [-0.195, +0.195, 0.0],  # Motor 1: Back-Left (CW)
        [+0.195, +0.195, 0.0],  # Motor 2: Front-Left (CCW)
        [-0.195, -0.195, 0.0],  # Motor 3: Back-Right (CCW)
    ])

    # Motor spin directions (1 = CW produces positive yaw torque, -1 = CCW)
    MOTOR_DIRECTIONS = np.array([1, 1, -1, -1])

    def __init__(self, config: DirectDynamicsConfig = None):
        self.config = config or DirectDynamicsConfig()
        self._initialized = False
        self._step_count = 0

        # Simulation objects
        self.simulation_app = None
        self.world = None
        self.timeline = None
        self.stage = None

        # Drone objects
        self.drone_prim = None
        self.drone_rb = None  # Rigid body handle
        self.drone_path = "/World/Drone"

        # Camera
        self.camera = None
        self.camera_path = "/World/Camera"
        self.gimbal = None

        # State
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz quaternion
        self.angular_velocity = np.zeros(3)

        # Episode randomization
        self._episode_mass = self.config.dynamics.mass

        # World generator
        self.world_gen = None

        # Computed constants
        self._precompute_dynamics_constants()

    def _precompute_dynamics_constants(self):
        """Precompute constants used in dynamics calculations."""
        params = self.config.dynamics

        # Hover thrust
        self.hover_thrust = params.mass * params.gravity

        # Motor mixing matrix: maps [thrust, roll, pitch, yaw] to motor thrusts
        # For X-configuration quadrotor
        L = params.arm_length
        k_t = params.k_thrust
        k_q = params.k_torque

        # Each row maps command to motor contribution
        # thrust: all motors contribute equally
        # roll: differential left-right
        # pitch: differential front-back
        # yaw: differential CW-CCW
        self.mix_matrix = np.array([
            [1.0, -1.0, -1.0,  1.0],  # Motor 0 FR (CW)
            [1.0,  1.0,  1.0,  1.0],  # Motor 1 BL (CW)
            [1.0,  1.0, -1.0, -1.0],  # Motor 2 FL (CCW)
            [1.0, -1.0,  1.0, -1.0],  # Motor 3 BR (CCW)
        ]) / 4.0

        # Inverse mix for going from motor speeds to commands
        self.mix_matrix_inv = np.linalg.pinv(self.mix_matrix)

    def setup(self) -> None:
        """Initialize the simulation environment."""
        if self._initialized:
            return

        print("=" * 60)
        print("Direct Dynamics Environment Setup")
        print("=" * 60)

        # Phase 1: Isaac Sim
        print("\n[1/5] Initializing Isaac Sim...")
        self._init_isaac_sim()

        # Phase 2: World
        print("\n[2/5] Creating world...")
        self._init_world()

        # Phase 3: Drone
        print("\n[3/5] Spawning drone...")
        self._spawn_drone()

        # Phase 4: Camera
        print("\n[4/5] Setting up camera...")
        self._setup_camera()

        # Phase 5: Environment
        print("\n[5/5] Generating environment...")
        self._generate_environment()

        # Warm up
        print("\n[Warmup] Running warmup steps...")
        for _ in range(100):
            self.world.step(render=True)

        self._initialized = True
        print("\n" + "=" * 60)
        print("Direct Dynamics Environment Ready")
        print("=" * 60)

    def _init_isaac_sim(self) -> None:
        """Initialize Isaac Sim application."""
        from isaacsim import SimulationApp

        simulation_config = {
            "headless": self.config.headless,
            "renderer": "RayTracedLighting",
            "anti_aliasing": 0,
            "width": 1280,
            "height": 720,
        }

        self.simulation_app = SimulationApp(simulation_config)
        print("  Isaac Sim ready")

    def _init_world(self) -> None:
        """Initialize Isaac Sim world."""
        import omni.timeline
        from omni.isaac.core.world import World
        from isaacsim.core.utils.stage import get_current_stage
        from pxr import UsdPhysics, Sdf, Gf

        self.timeline = omni.timeline.get_timeline_interface()

        # Create world with physics settings
        self.world = World(
            physics_dt=self.config.physics_dt,
            rendering_dt=self.config.physics_dt,
            stage_units_in_meters=1.0,
        )

        self.stage = get_current_stage()

        # Ensure physics scene exists
        physics_path = "/World/PhysicsScene"
        if not self.stage.GetPrimAtPath(physics_path):
            scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path(physics_path))
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(self.config.dynamics.gravity)

        print("  World ready")

    def _spawn_drone(self) -> None:
        """Spawn a simple drone rigid body for direct dynamics control."""
        from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf
        import omni.isaac.core.utils.prims as prim_utils

        # Create drone prim
        prim_utils.create_prim(self.drone_path, "Xform")
        self.drone_prim = self.stage.GetPrimAtPath(self.drone_path)

        # Create visual geometry - simple box representing drone body
        body_path = f"{self.drone_path}/body"
        body_mesh = UsdGeom.Cube.Define(self.stage, body_path)
        body_mesh.GetSizeAttr().Set(0.3)  # 30cm cube

        # Apply material for visibility
        body_prim = self.stage.GetPrimAtPath(body_path)
        body_xform = UsdGeom.Xformable(body_prim)
        body_xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 0.4))  # Flatten for drone shape

        # Add visual arms
        arm_positions = [
            (0.15, 0.15, 0), (0.15, -0.15, 0),
            (-0.15, 0.15, 0), (-0.15, -0.15, 0)
        ]
        for i, pos in enumerate(arm_positions):
            arm_path = f"{self.drone_path}/arm_{i}"
            arm_mesh = UsdGeom.Cylinder.Define(self.stage, arm_path)
            arm_mesh.GetRadiusAttr().Set(0.02)
            arm_mesh.GetHeightAttr().Set(0.05)
            arm_xform = UsdGeom.Xformable(arm_mesh.GetPrim())
            arm_xform.AddTranslateOp().Set(Gf.Vec3d(*pos))

        # Apply rigid body physics
        UsdPhysics.RigidBodyAPI.Apply(self.drone_prim)

        # Mass properties
        mass_api = UsdPhysics.MassAPI.Apply(self.drone_prim)
        mass_api.CreateMassAttr(self.config.dynamics.mass)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(
            self.config.dynamics.inertia_xx,
            self.config.dynamics.inertia_yy,
            self.config.dynamics.inertia_zz,
        ))

        # Collision geometry
        collision_path = f"{self.drone_path}/collision"
        collision_mesh = UsdGeom.Sphere.Define(self.stage, collision_path)
        collision_mesh.GetRadiusAttr().Set(0.25)
        UsdPhysics.CollisionAPI.Apply(collision_mesh.GetPrim())

        # Set initial position
        drone_xform = UsdGeom.Xformable(self.drone_prim)
        drone_xform.ClearXformOpOrder()
        self._translate_op = drone_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
        self._orient_op = drone_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)

        self._translate_op.Set(Gf.Vec3d(*self.config.spawn_position))
        self._orient_op.Set(Gf.Quatd(1, 0, 0, 0))

        self.position = self.config.spawn_position.copy()

        print(f"  Drone spawned at {self.position}")

    def _setup_camera(self) -> None:
        """Setup camera for observations."""
        if not self.config.camera_enabled:
            print("  Camera: DISABLED")
            return

        from isaacsim.sensors.camera import Camera
        import isaacsim.core.utils.numpy.rotations as rot_utils

        self.camera = Camera(
            prim_path=self.camera_path,
            position=self.config.spawn_position + np.array([0, 0, 0.5]),
            frequency=30,
            resolution=self.config.camera_resolution,
            orientation=rot_utils.euler_angles_to_quats(
                np.array([90, 0, 0]), degrees=True
            ),
        )
        self.camera.initialize()
        self.simulation_app.update()
        self.camera.initialize()
        self.simulation_app.update()

        # Setup gimbal controller for camera orientation
        from pxr import UsdGeom, Gf
        from scipy.spatial.transform import Rotation

        camera_prim = self.stage.GetPrimAtPath(self.camera_path)
        camera_xform = UsdGeom.Xformable(camera_prim)

        # Store camera ops for updates
        self._camera_translate_op = None
        self._camera_orient_op = None

        for op in camera_xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                self._camera_translate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                self._camera_orient_op = op

        if self._camera_translate_op is None:
            self._camera_translate_op = camera_xform.AddTranslateOp()
        if self._camera_orient_op is None:
            self._camera_orient_op = camera_xform.AddOrientOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            )

        # Set initial gimbal angle
        self._update_camera_orientation(self.config.gimbal_tilt)

        print(f"  Camera ready: {self.config.camera_resolution}")

    def _update_camera_orientation(self, tilt_degrees: float) -> None:
        """Update camera gimbal orientation."""
        from scipy.spatial.transform import Rotation
        from pxr import Gf

        tilt_rad = np.radians(tilt_degrees)

        # Camera looks forward and down
        cos_tilt, sin_tilt = np.cos(tilt_rad), np.sin(tilt_rad)
        look_dir = np.array([cos_tilt, 0, sin_tilt])
        look_dir = look_dir / np.linalg.norm(look_dir)

        # Construct rotation matrix
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(look_dir, world_up)
        right_norm = np.linalg.norm(right)
        if right_norm < 1e-6:
            right = np.array([0.0, 1.0, 0.0])
        else:
            right = right / right_norm

        up = np.cross(right, look_dir)
        up = up / np.linalg.norm(up)

        R_cam_to_world = np.column_stack([right, up, -look_dir])
        world_rot = Rotation.from_matrix(R_cam_to_world)
        quat_xyzw = world_rot.as_quat()
        quat_wxyz = Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])

        if self._camera_orient_op is not None:
            self._camera_orient_op.Set(quat_wxyz)

    def _generate_environment(self) -> None:
        """Generate procedural environment with terrain and objects."""
        sys.path.insert(0, "/workspace")
        sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")

        try:
            from flyby.world_generator.world_generator import WorldGenerator, WorldConfig

            models_path = "/workspace/extensions/forest_generator/models"
            world_config = WorldConfig(
                terrain_size=self.config.terrain_size,
                tree_density=self.config.tree_density,
            )

            self.world_gen = WorldGenerator(models_path, world_config)
            self.world_gen.generate_terrain()
            self.world_gen.setup_lighting()
            self.world_gen.generate_forest(
                density=self.config.tree_density,
                include_undergrowth=False
            )

            # Spawn target vehicles
            if self.config.vehicle_count > 0:
                vehicle_paths = self.world_gen.vehicles.spawn_vehicle_group(
                    vehicle_types=["sedan2", "suv", "tank", "taxi"],
                    count=self.config.vehicle_count,
                    clustering=0.3,
                    center=(30.0, 30.0),
                )
                print(f"  Spawned {len(vehicle_paths)} vehicles")

            for _ in range(10):
                self.simulation_app.update()

            print("  Environment generated")

        except Exception as e:
            print(f"  Warning: Could not generate environment: {e}")
            print("  Continuing with empty world...")

    def reset(self, seed: int = None) -> np.ndarray:
        """Reset the environment for a new episode."""
        if not self._initialized:
            self.setup()

        if seed is not None:
            np.random.seed(seed)

        self._step_count = 0

        # Apply domain randomization
        if self.config.mass_variation > 0:
            mass_factor = 1.0 + np.random.uniform(
                -self.config.mass_variation,
                self.config.mass_variation
            )
            self._episode_mass = self.config.dynamics.mass * mass_factor
        else:
            self._episode_mass = self.config.dynamics.mass

        # Reset drone state
        self.position = self.config.spawn_position.copy()
        self.velocity = np.zeros(3)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz
        self.angular_velocity = np.zeros(3)

        # Update drone prim
        from pxr import Gf
        self._translate_op.Set(Gf.Vec3d(*self.position))

        heading_quat = self._euler_to_quat(0, 0, self.config.spawn_heading)
        self._orient_op.Set(Gf.Quatd(
            heading_quat[0], heading_quat[1], heading_quat[2], heading_quat[3]
        ))
        self.orientation = heading_quat

        # Reset velocity on rigid body
        self._set_rigid_body_velocity(np.zeros(3), np.zeros(3))

        # Step simulation to settle
        if not self.timeline.is_playing():
            self.timeline.play()

        for _ in range(50):
            self.world.step(render=True)

        # Update state from simulation
        self._read_state_from_sim()

        return self._get_observation()

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """
        Execute one environment step.

        Args:
            action: Action array, interpretation depends on action_mode:
                - MOTOR_SPEEDS: [m1, m2, m3, m4] normalized [-1, 1]
                - THRUST_RATES: [thrust, roll_rate, pitch_rate, yaw_rate] normalized [-1, 1]
                - VELOCITY: [vx, vy, vz, yaw_rate] normalized [-1, 1]

        Returns:
            observation, reward, done, info
        """
        self._step_count += 1

        # Convert action to forces/torques
        force, torque = self._action_to_wrench(action)

        # Apply wrench to rigid body
        self._apply_wrench(force, torque)

        # Step physics with optional render skipping
        need_render = (self._step_count % self.config.render_interval == 0)
        self.world.step(render=need_render)

        # Read state from simulation
        self._read_state_from_sim()

        # Update camera to follow drone
        if self.camera is not None and self._camera_translate_op is not None:
            from pxr import Gf
            camera_pos = Gf.Vec3d(
                self.position[0],
                self.position[1],
                self.position[2] + 0.5
            )
            self._camera_translate_op.Set(camera_pos)

        # Compute observation, reward, done
        obs = self._get_observation()
        reward = self._compute_reward()
        done = self._check_done()

        info = {
            "step": self._step_count,
            "rendered": need_render,
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
            "force_applied": force.copy(),
            "torque_applied": torque.copy(),
        }

        return obs, reward, done, info

    def _action_to_wrench(self, action: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert action to force and torque in body frame.

        Returns:
            force: [fx, fy, fz] in body frame
            torque: [tx, ty, tz] in body frame
        """
        action = np.clip(action, -1, 1)
        params = self.config.dynamics

        if self.config.action_mode == ActionMode.MOTOR_SPEEDS:
            # Direct motor speed control
            # Map [-1, 1] to [min_rpm, max_rpm]
            motor_rpms = (action + 1) / 2 * (params.max_rpm - params.min_rpm) + params.min_rpm
            motor_omegas = motor_rpms * 2 * np.pi / 60.0

            # Calculate thrust per motor
            thrusts = params.k_thrust * motor_omegas**2

            # Total thrust (body Z-up)
            total_thrust = np.sum(thrusts)
            force = np.array([0, 0, total_thrust])

            # Torques from thrust differential
            # Roll: differential left-right
            roll_torque = params.arm_length * (
                (thrusts[2] + thrusts[1]) - (thrusts[0] + thrusts[3])
            )
            # Pitch: differential front-back
            pitch_torque = params.arm_length * (
                (thrusts[0] + thrusts[2]) - (thrusts[1] + thrusts[3])
            )
            # Yaw: reaction torque from motor spin
            yaw_torque = params.k_torque * (
                self.MOTOR_DIRECTIONS @ motor_omegas**2
            )

            torque = np.array([roll_torque, pitch_torque, yaw_torque])

        elif self.config.action_mode == ActionMode.THRUST_RATES:
            # Thrust + angular rate control
            thrust_normalized = (action[0] + 1) / 2  # [0, 1]
            thrust = thrust_normalized * self.config.max_thrust * self.hover_thrust

            # Angular rates (rad/s)
            roll_rate = action[1] * np.radians(self.config.max_roll_rate)
            pitch_rate = action[2] * np.radians(self.config.max_pitch_rate)
            yaw_rate = action[3] * np.radians(self.config.max_yaw_rate)

            # Force in body frame (Z-up)
            force = np.array([0, 0, thrust])

            # Compute required torque for angular acceleration
            # Using P control to track desired rates
            rate_error = np.array([roll_rate, pitch_rate, yaw_rate]) - self.angular_velocity

            # PD gains for rate control
            kp_rate = np.array([5.0, 5.0, 2.0])  # Proportional gain

            # Required torque: I * alpha = I * kp * error
            inertia = params.inertia_matrix
            torque = inertia @ (kp_rate * rate_error)

        elif self.config.action_mode == ActionMode.VELOCITY:
            # Velocity command with internal controller
            target_vel = np.array([
                action[0] * self.config.max_velocity_xy,
                action[1] * self.config.max_velocity_xy,
                action[2] * self.config.max_velocity_z,
            ])
            target_yaw_rate = action[3] * np.radians(self.config.max_yaw_rate)

            # Transform target velocity to body frame
            R = self._quat_to_rotation_matrix(self.orientation)
            body_vel = R.T @ self.velocity
            target_body_vel = R.T @ target_vel

            # Velocity error in body frame
            vel_error = target_body_vel - body_vel

            # PD controller for velocity
            tau = self.config.velocity_response_time
            kp_vel = 2.0 / tau

            # Desired acceleration
            desired_accel = kp_vel * vel_error

            # Required force (body frame)
            # F = m * a + m * g (gravity compensation in world frame -> body frame)
            gravity_world = np.array([0, 0, -params.gravity])
            gravity_body = R.T @ gravity_world
            force = self._episode_mass * (desired_accel - gravity_body)

            # Limit lateral force by tilting
            # The main thrust is Z, we create desired tilt from x,y force components
            desired_roll = -np.arctan2(force[1], max(force[2], 0.1))
            desired_pitch = np.arctan2(force[0], max(force[2], 0.1))

            # Limit tilt angles
            max_tilt = np.radians(30)
            desired_roll = np.clip(desired_roll, -max_tilt, max_tilt)
            desired_pitch = np.clip(desired_pitch, -max_tilt, max_tilt)

            # Current Euler angles
            current_roll, current_pitch, current_yaw = self._quat_to_euler(self.orientation)

            # Angular rate commands to achieve desired orientation
            kp_angle = 4.0
            roll_rate_cmd = kp_angle * (desired_roll - current_roll)
            pitch_rate_cmd = kp_angle * (desired_pitch - current_pitch)

            # Rate control
            rate_cmd = np.array([roll_rate_cmd, pitch_rate_cmd, target_yaw_rate])
            rate_error = rate_cmd - self.angular_velocity

            kp_rate = np.array([5.0, 5.0, 2.0])
            inertia = params.inertia_matrix
            torque = inertia @ (kp_rate * rate_error)

            # Thrust is mainly vertical in body frame
            thrust_magnitude = np.linalg.norm(force)
            force = np.array([0, 0, max(thrust_magnitude, 0)])

        else:
            raise ValueError(f"Unknown action mode: {self.config.action_mode}")

        return force, torque

    def _apply_wrench(self, force_body: np.ndarray, torque_body: np.ndarray) -> None:
        """Apply force and torque to the drone rigid body."""
        from pxr import Gf, PhysxSchema

        # Transform force from body to world frame
        R = self._quat_to_rotation_matrix(self.orientation)
        force_world = R @ force_body

        # Apply gravity in world frame
        gravity_force = np.array([0, 0, -self._episode_mass * self.config.dynamics.gravity])
        force_world = force_world + gravity_force

        # Add aerodynamic drag
        drag = self._compute_drag()
        force_world = force_world + drag

        # Apply domain randomization noise
        if self.config.velocity_noise_std > 0:
            force_world += np.random.normal(0, self.config.velocity_noise_std * self._episode_mass, 3)

        # Get rigid body and apply forces
        # Use PhysX direct API for force application
        from omni.physx import get_physx_interface

        physx = get_physx_interface()

        # Apply force at center of mass
        physx.apply_force_at_pos(
            self.drone_path,
            Gf.Vec3f(*force_world),
            Gf.Vec3f(*self.position),
        )

        # Apply torque in world frame
        torque_world = R @ torque_body
        # Note: PhysX applies torque directly without needing transformation
        # But we want torque in world frame for consistency

        # For torque, we use the PhysX torque application
        # This requires accessing the rigid body handle differently
        # Use velocity-based approach instead for simplicity

        # Update angular velocity directly based on torque
        # angular_accel = I^-1 @ torque (in body frame)
        I_inv = np.linalg.inv(self.config.dynamics.inertia_matrix)
        angular_accel_body = I_inv @ torque_body

        # Convert to world frame
        angular_accel_world = R @ angular_accel_body

        # Integrate angular velocity
        dt = self.config.physics_dt
        new_angular_vel = self.angular_velocity + angular_accel_world * dt

        # Clamp angular velocity
        max_angular_vel = np.radians(360)  # 360 deg/s max
        new_angular_vel = np.clip(new_angular_vel, -max_angular_vel, max_angular_vel)

        # Set the new angular velocity
        self._set_rigid_body_velocity(None, new_angular_vel)

    def _compute_drag(self) -> np.ndarray:
        """Compute aerodynamic drag force in world frame."""
        params = self.config.dynamics

        # Quadratic drag model
        vel_sq = self.velocity**2 * np.sign(self.velocity)

        drag = np.array([
            -params.drag_coeff_xy * vel_sq[0],
            -params.drag_coeff_xy * vel_sq[1],
            -params.drag_coeff_z * vel_sq[2],
        ])

        return drag

    def _set_rigid_body_velocity(
        self,
        linear_vel: Optional[np.ndarray],
        angular_vel: Optional[np.ndarray]
    ) -> None:
        """Set rigid body velocities directly."""
        from pxr import Gf, UsdPhysics

        rb_api = UsdPhysics.RigidBodyAPI(self.drone_prim)

        if linear_vel is not None:
            rb_api.GetVelocityAttr().Set(Gf.Vec3f(*linear_vel))

        if angular_vel is not None:
            rb_api.GetAngularVelocityAttr().Set(Gf.Vec3f(*angular_vel))

    def _read_state_from_sim(self) -> None:
        """Read drone state from Isaac Sim."""
        from pxr import Gf, UsdPhysics, UsdGeom

        # Read transform
        xform = UsdGeom.Xformable(self.drone_prim)
        world_transform = xform.ComputeLocalToWorldTransform(0)

        # Extract position
        translation = world_transform.ExtractTranslation()
        self.position = np.array([translation[0], translation[1], translation[2]])

        # Extract orientation as quaternion
        rotation = world_transform.ExtractRotationQuat()
        # USD Quatd is (real, i, j, k) = (w, x, y, z)
        self.orientation = np.array([
            rotation.GetReal(),
            rotation.GetImaginary()[0],
            rotation.GetImaginary()[1],
            rotation.GetImaginary()[2],
        ])

        # Read velocities from rigid body
        rb_api = UsdPhysics.RigidBodyAPI(self.drone_prim)

        lin_vel = rb_api.GetVelocityAttr().Get()
        if lin_vel is not None:
            self.velocity = np.array([lin_vel[0], lin_vel[1], lin_vel[2]])

        ang_vel = rb_api.GetAngularVelocityAttr().Get()
        if ang_vel is not None:
            self.angular_velocity = np.array([ang_vel[0], ang_vel[1], ang_vel[2]])

    def _get_observation(self) -> np.ndarray:
        """Get observation vector."""
        # Normalize position relative to world bounds
        bounds = self.config.world_bounds
        pos_normalized = np.array([
            (self.position[0] - bounds[0]) / (bounds[1] - bounds[0]) * 2 - 1,
            (self.position[1] - bounds[2]) / (bounds[3] - bounds[2]) * 2 - 1,
            (self.position[2] - bounds[4]) / (bounds[5] - bounds[4]) * 2 - 1,
        ])

        # Normalize velocity
        max_vel = 15.0
        vel_normalized = self.velocity / max_vel

        # Angular velocity (normalized)
        max_ang_vel = np.radians(360)
        ang_vel_normalized = self.angular_velocity / max_ang_vel

        obs = np.concatenate([
            pos_normalized,
            vel_normalized,
            self.orientation,
            ang_vel_normalized,
        ]).astype(np.float32)

        return obs

    def _compute_reward(self) -> float:
        """Compute reward (placeholder - override in subclass)."""
        # Simple reward: stay alive and stay near center
        pos_penalty = -0.01 * np.linalg.norm(self.position[:2])
        alive_bonus = 0.1

        # Penalty for extreme velocities
        vel_penalty = -0.01 * max(0, np.linalg.norm(self.velocity) - 10)

        return alive_bonus + pos_penalty + vel_penalty

    def _check_done(self) -> bool:
        """Check if episode is done."""
        # Check world bounds
        bounds = self.config.world_bounds
        if (self.position[0] < bounds[0] or self.position[0] > bounds[1] or
            self.position[1] < bounds[2] or self.position[1] > bounds[3] or
            self.position[2] < bounds[4] or self.position[2] > bounds[5]):
            return True

        # Check max steps
        if self._step_count >= self.config.max_episode_steps:
            return True

        # Check for crash (too low)
        if self.position[2] < 1.0:
            return True

        return False

    # Utility functions
    def _euler_to_quat(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert Euler angles (radians) to quaternion [w, x, y, z]."""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([w, x, y, z])

    def _quat_to_euler(self, quat: np.ndarray) -> Tuple[float, float, float]:
        """Convert quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw) in radians."""
        w, x, y, z = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _quat_to_rotation_matrix(self, quat: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
        w, x, y, z = quat

        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ])

        return R

    def get_camera_image(self) -> Optional[np.ndarray]:
        """Get current camera image."""
        if self.camera is None:
            return None
        try:
            return self.camera.get_rgba()
        except Exception:
            return None

    def close(self) -> None:
        """Clean up resources."""
        if self._initialized:
            try:
                self.timeline.stop()
            except Exception:
                pass
            self._initialized = False

    @property
    def observation_dim(self) -> int:
        """Dimension of observation space."""
        return 13  # 3 pos + 3 vel + 4 quat + 3 ang_vel

    @property
    def action_dim(self) -> int:
        """Dimension of action space."""
        return 4
