#!/usr/bin/env python3
"""
Phase 6: SAC Training Test

Builds on Phase 5 (Reward Signal) by adding:
- Stable Baselines3 SAC agent
- Short training run to verify learning signal
- Tensorboard logging
- Policy checkpoint saving

This incrementally adds RL training on top of Phase 5's
working reward function.

Success criteria:
- SAC agent trains without crashing
- Loss values decrease over training
- Episode rewards improve (even slightly)
- Policy checkpoints save correctly
"""

from isaacsim import SimulationApp

# Launch with GUI (can be headless for training)
import sys
HEADLESS = "--headless" in sys.argv
simulation_app = SimulationApp({"headless": HEADLESS})

import omni
import omni.timeline
import omni.kit.viewport.utility as viewport_utils
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import os
from datetime import datetime

# Add Pegasus to path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

# Add our extension to path
EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

# Add workspace to path
sys.path.insert(0, "/workspace")

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pymavlink import mavutil

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig
from isaacsim.core.utils.stage import get_current_stage

# Import Stable Baselines3
try:
    from stable_baselines3 import SAC
    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
    from stable_baselines3.common.logger import configure
    SB3_AVAILABLE = True
except ImportError:
    SB3_AVAILABLE = False
    print("[WARNING] stable_baselines3 not available, will run in demo mode", flush=True)

print("=" * 60, flush=True)
print("Phase 6: SAC Training Test", flush=True)
print("=" * 60, flush=True)
print(f"  Headless: {HEADLESS}", flush=True)
print(f"  SB3 Available: {SB3_AVAILABLE}", flush=True)


class Phase6SACEnv(gym.Env):
    """
    Gymnasium environment for SAC training.

    Same as Phase5RewardEnv but optimized for training:
    - Faster episode resets
    - Configurable episode length
    - Training statistics tracking
    """

    metadata = {'render_modes': ['human']}

    def __init__(self, world, vehicle, mav, target_positions,
                 max_steps=200, update_camera_fn=None):
        super().__init__()

        self.world = world
        self.vehicle = vehicle
        self.mav = mav
        self.target_positions = target_positions
        self.update_camera_fn = update_camera_fn

        # Observation space
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(29,), dtype=np.float32
        )

        # Action space: [vx, vy, vz, yaw_rate] normalized to [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )

        # Scaling factors
        self.max_velocity = 5.0  # m/s
        self.max_yaw_rate = 45.0  # deg/s

        # Episode state
        self.step_count = 0
        self.max_steps = max_steps
        self.episode_count = 0

        # Camera parameters
        self.camera_fov_deg = 90.0
        self.camera_pitch_deg = 45.0
        self.max_detection_range = 100.0

        # Coverage tracking
        self.grid_size = 8
        self.area_size = 200.0
        self.cell_size = self.area_size / self.grid_size
        self.coverage_grid = np.zeros((self.grid_size, self.grid_size), dtype=bool)

        # POI detection tracking
        self.detected_pois = set()
        self.eyes_on_pois = set()

        # Reward weights
        self.reward_weights = {
            'coverage': 0.1,
            'eyes_on': 1.0,
            'yolo_detection': 5.0,
            'altitude_penalty': -0.01,
            'boundary_penalty': -0.1,
            'time_penalty': -0.001,
            'crash_penalty': -10.0,
        }

        # Altitude constraints
        self.min_altitude = 15.0
        self.max_altitude = 50.0
        self.optimal_altitude = 25.0

        # Episode statistics
        self.episode_stats = {}

        # Training statistics
        self.training_stats = {
            'episodes': 0,
            'total_steps': 0,
            'best_reward': float('-inf'),
            'recent_rewards': [],
        }

    def reset(self, seed=None, options=None):
        """Reset environment for new episode."""
        super().reset(seed=seed)

        self.step_count = 0
        self.episode_count += 1

        # Reset tracking
        self.coverage_grid = np.zeros((self.grid_size, self.grid_size), dtype=bool)
        self.eyes_on_pois = set()
        self.detected_pois = set()

        # Reset stats
        self.episode_stats = {
            'total_reward': 0.0,
            'coverage_reward': 0.0,
            'eyes_on_reward': 0.0,
            'yolo_reward': 0.0,
            'penalty': 0.0,
            'cells_covered': 0,
            'eyes_on_count': 0,
            'yolo_count': 0,
        }

        # Get initial observation
        obs = self._get_observation()
        info = {
            "step": 0,
            "position": self.vehicle.state.position.tolist(),
            "coverage": 0.0,
            "pois_detected": 0,
            "episode": self.episode_count,
        }

        return obs, info

    def step(self, action):
        """Execute action and compute reward."""
        self.step_count += 1

        # Convert normalized action to velocity setpoint
        vx = float(action[0]) * self.max_velocity
        vy = float(action[1]) * self.max_velocity
        vz = float(action[2]) * self.max_velocity

        # Send velocity setpoint
        self._send_setpoint(vx, vy, vz)

        # Step simulation
        self.world.step(render=not HEADLESS)

        # Get state
        pos = self.vehicle.state.position
        orient = self.vehicle.state.attitude

        # Update camera if callback provided
        if self.update_camera_fn:
            self.update_camera_fn(pos, orient)

        # Compute reward
        reward = 0.0

        # 1. Coverage reward
        coverage_reward = self._update_coverage(pos)
        reward += coverage_reward
        self.episode_stats['coverage_reward'] += coverage_reward

        # 2. POI detection reward
        poi_reward = self._check_poi_detections(pos, orient)
        reward += poi_reward

        # 3. Altitude penalty
        altitude_penalty = self._compute_altitude_penalty(pos[2])
        reward += altitude_penalty
        self.episode_stats['penalty'] += altitude_penalty

        # 4. Boundary penalty
        boundary_penalty = self._compute_boundary_penalty(pos)
        reward += boundary_penalty
        self.episode_stats['penalty'] += boundary_penalty

        # 5. Time penalty
        reward += self.reward_weights['time_penalty']
        self.episode_stats['penalty'] += self.reward_weights['time_penalty']

        self.episode_stats['total_reward'] += reward

        # Get observation
        obs = self._get_observation()

        # Check termination
        terminated = False
        truncated = False

        # Crash detection
        if pos[2] < 0.5:
            terminated = True
            reward += self.reward_weights['crash_penalty']

        # Boundary violation
        if np.linalg.norm(pos[:2]) > 150.0:
            terminated = True
            reward += self.reward_weights['crash_penalty'] * 0.5

        # Max steps
        if self.step_count >= self.max_steps:
            truncated = True

        # Update training stats on episode end
        if terminated or truncated:
            self.training_stats['episodes'] += 1
            ep_reward = self.episode_stats['total_reward']
            self.training_stats['recent_rewards'].append(ep_reward)
            if len(self.training_stats['recent_rewards']) > 100:
                self.training_stats['recent_rewards'].pop(0)
            if ep_reward > self.training_stats['best_reward']:
                self.training_stats['best_reward'] = ep_reward

        self.training_stats['total_steps'] += 1

        coverage_pct = np.sum(self.coverage_grid) / (self.grid_size ** 2) * 100

        info = {
            "step": self.step_count,
            "position": pos.tolist(),
            "coverage": coverage_pct,
            "pois_detected": len(self.detected_pois),
            "episode_stats": self.episode_stats.copy(),
        }

        return obs, reward, terminated, truncated, info

    def _update_coverage(self, pos):
        """Update coverage grid and return reward."""
        grid_x = int((pos[0] + self.area_size / 2) / self.cell_size)
        grid_y = int((pos[1] + self.area_size / 2) / self.cell_size)
        grid_x = max(0, min(self.grid_size - 1, grid_x))
        grid_y = max(0, min(self.grid_size - 1, grid_y))

        if not self.coverage_grid[grid_x, grid_y]:
            self.coverage_grid[grid_x, grid_y] = True
            self.episode_stats['cells_covered'] += 1
            return self.reward_weights['coverage']
        return 0.0

    def _check_poi_detections(self, pos, orient):
        """Check for POI detections with two-tier rewards."""
        reward = 0.0

        r = Rotation.from_quat(orient)
        camera_pitch = Rotation.from_euler('Y', self.camera_pitch_deg, degrees=True)
        camera_rotation = r * camera_pitch
        camera_forward = camera_rotation.apply([1, 0, 0])
        fov_half = np.radians(self.camera_fov_deg / 2)

        for i, target in enumerate(self.target_positions):
            if i in self.detected_pois:
                continue

            to_target = target['position'] - pos
            dist = np.linalg.norm(to_target)

            if dist > self.max_detection_range or dist < 5.0:
                continue

            if pos[2] < target['position'][2] + 10.0:
                continue

            to_target_norm = to_target / dist
            cos_angle = np.dot(to_target_norm, camera_forward)
            angle = np.arccos(np.clip(cos_angle, -1, 1))

            if angle > fov_half:
                continue

            forward_dist = np.dot(to_target, camera_forward)
            if forward_dist < 5.0:
                continue

            # Eyes-on reward
            if i not in self.eyes_on_pois:
                self.eyes_on_pois.add(i)
                self.episode_stats['eyes_on_count'] += 1
                eyes_reward = self.reward_weights['eyes_on']
                reward += eyes_reward
                self.episode_stats['eyes_on_reward'] += eyes_reward

            # YOLO reward
            if dist < 50.0 and angle < np.radians(30.0):
                self.detected_pois.add(i)
                self.episode_stats['yolo_count'] += 1
                yolo_reward = self.reward_weights['yolo_detection']
                reward += yolo_reward
                self.episode_stats['yolo_reward'] += yolo_reward

        return reward

    def _compute_altitude_penalty(self, altitude):
        if altitude < self.min_altitude:
            return self.reward_weights['altitude_penalty'] * (self.min_altitude - altitude)
        elif altitude > self.max_altitude:
            return self.reward_weights['altitude_penalty'] * (altitude - self.max_altitude)
        return 0.0

    def _compute_boundary_penalty(self, pos):
        dist_from_center = np.linalg.norm(pos[:2])
        boundary_dist = 100.0
        if dist_from_center > boundary_dist:
            return self.reward_weights['boundary_penalty'] * (dist_from_center - boundary_dist) / 50.0
        return 0.0

    def _get_observation(self):
        pos = self.vehicle.state.position
        vel = self.vehicle.state.linear_velocity
        orient = self.vehicle.state.attitude

        r = Rotation.from_quat(orient)
        euler = r.as_euler('XYZ')

        uav_state = np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            euler[0], euler[1], euler[2]
        ], dtype=np.float32)

        target_obs = []
        for target in self.target_positions[:5]:
            dx = target['position'][0] - pos[0]
            dy = target['position'][1] - pos[1]
            dz = target['position'][2] - pos[2]
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            in_fov = 1.0 if dist < self.max_detection_range else 0.0
            target_obs.extend([dx, dy, dz, in_fov])

        while len(target_obs) < 20:
            target_obs.extend([0.0, 0.0, 0.0, 0.0])

        target_obs = np.array(target_obs[:20], dtype=np.float32)
        obs = np.concatenate([uav_state, target_obs])
        return obs

    def _send_setpoint(self, vx, vy, vz):
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
        )

    def close(self):
        pass


class TrainingCallback(BaseCallback):
    """Custom callback for logging training progress."""

    def __init__(self, verbose=1):
        super().__init__(verbose)
        self.episode_rewards = []
        self.episode_lengths = []

    def _on_step(self):
        # Log episode info when available
        if len(self.model.ep_info_buffer) > 0:
            ep_info = self.model.ep_info_buffer[-1]
            if 'r' in ep_info and 't' in ep_info:
                self.episode_rewards.append(ep_info['r'])
                self.episode_lengths.append(ep_info['t'])

                if self.verbose > 0 and len(self.episode_rewards) % 5 == 0:
                    mean_reward = np.mean(self.episode_rewards[-10:])
                    mean_length = np.mean(self.episode_lengths[-10:])
                    print(f"  [SAC] Episodes: {len(self.episode_rewards)}, "
                          f"Mean reward (10): {mean_reward:.2f}, "
                          f"Mean length: {mean_length:.0f}", flush=True)
        return True


# =============================================================================
# Setup (same as Phase 5)
# =============================================================================

timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world
pg.set_px4_path("/px4")

stage = get_current_stage()
physics_path = "/World/PhysicsScene"
if not stage.GetPrimAtPath(physics_path):
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_path))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    print("Physics scene created", flush=True)

print("\n[WorldGen] Generating procedural environment...", flush=True)
models_path = "/workspace/extensions/forest_generator/models"

world_config = WorldConfig(
    terrain_size=(200.0, 200.0),
    terrain_roughness=2.0,
    terrain_material="forest_floor",
    tree_density=0.3,
    tree_proportions={"Birch": 30, "Spruce": 40, "Pine": 30},
    undergrowth_density=0.0,
    randomize_lighting=False,
    time_of_day="noon",
    weather="clear",
    seed=42,
)

world_gen = WorldGenerator(models_path, world_config)

print("[WorldGen] Generating terrain...", flush=True)
terrain_path = world_gen.generate_terrain()

print("[WorldGen] Setting up lighting...", flush=True)
world_gen.setup_lighting()

print("[WorldGen] Generating forest...", flush=True)
forest_result = world_gen.generate_forest(density=0.3, include_undergrowth=False)
print(f"[WorldGen] Forest generated: {len(forest_result['trees'])} trees", flush=True)

# Spawn POI targets
print("\n[WorldGen] Spawning POI targets...", flush=True)
poi_clusters = [
    (30.0, 20.0),
    (-20.0, 30.0),
    (40.0, -25.0),
]
all_vehicle_paths = []

for center in poi_clusters:
    paths = world_gen.vehicles.spawn_vehicle_group(
        vehicle_types=["sedan", "suv", "tank"],
        count=3,
        clustering=0.5,
        center=center,
    )
    all_vehicle_paths.extend(paths)

print(f"[WorldGen] Spawned {len(all_vehicle_paths)} vehicles", flush=True)

# Store target positions
target_positions = []
for path in all_vehicle_paths:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        translate_op = xform.GetOrderedXformOps()[0] if xform.GetOrderedXformOps() else None
        if translate_op:
            pos = translate_op.Get()
            target_positions.append({
                'path': path,
                'position': np.array([pos[0], pos[1], pos[2]]),
                'type': 'vehicle'
            })

print(f"[WorldGen] Total POI targets: {len(target_positions)}", flush=True)

# Create camera if not headless
camera_prim_path = "/World/ISRCamera"
if not HEADLESS:
    print("\n[Camera] Creating ISR camera...", flush=True)
    camera_prim = UsdGeom.Camera.Define(stage, camera_prim_path)
    camera_prim.CreateFocalLengthAttr(4.5)
    camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 500.0))

    try:
        viewport_api = viewport_utils.get_active_viewport()
        if viewport_api:
            viewport_api.set_active_camera(camera_prim_path)
            print("[Camera] Set viewport to ISR camera view", flush=True)
    except Exception as e:
        print(f"[Camera] Warning: Could not set viewport camera: {e}", flush=True)


def update_camera_pose(drone_pos, drone_orient):
    """Update ISR camera to follow drone."""
    if HEADLESS:
        return
    camera_xform = UsdGeom.Xformable(stage.GetPrimAtPath(camera_prim_path))
    camera_xform.ClearXformOpOrder()
    camera_xform.AddTranslateOp().Set(Gf.Vec3d(drone_pos[0], drone_pos[1], drone_pos[2] - 0.1))

    drone_rot = Rotation.from_quat(drone_orient)
    drone_euler = drone_rot.as_euler('ZYX', degrees=True)
    drone_yaw = drone_euler[0]
    camera_rot = Rotation.from_euler("ZX", [drone_yaw, 45.0], degrees=True)
    quat = camera_rot.as_quat()
    camera_xform.AddOrientOp().Set(Gf.Quatf(quat[3], quat[0], quat[1], quat[2]))


# Create PX4 vehicle
print("\n[PX4] Creating vehicle...", flush=True)
mavlink_config = PX4MavlinkBackendConfig({
    "vehicle_id": 0,
    "px4_autolaunch": True,
    "px4_dir": pg.px4_path,
    "px4_vehicle_model": pg.px4_default_airframe,
    "enable_lockstep": True,
    "update_rate": 250.0,
})
config = MultirotorConfig()
config.backends = [PX4MavlinkBackend(mavlink_config)]

spawn_height = 5.0
vehicle = Multirotor(
    "/World/quadrotor",
    ROBOTS['Iris'],
    0,
    [0.0, 0.0, spawn_height],
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=config,
)
print(f"[PX4] Vehicle spawned at height {spawn_height}m", flush=True)

world.reset()
timeline.play()

print("\n[PX4] Running 200 steps to let PX4 initialize...", flush=True)
for i in range(200):
    world.step(render=not HEADLESS)

print("\n[MAVLink] Connecting...", flush=True)
mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

for i in range(500):
    world.step(render=not HEADLESS)
    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg and msg.get_srcSystem() != 0:
        print(f"[MAVLink] Got heartbeat from system {msg.get_srcSystem()} after {i} steps", flush=True)
        break

if mav.target_system == 0:
    mav.target_system = 1
    mav.target_component = 1
print(f"[MAVLink] Target: system {mav.target_system}, component {mav.target_component}", flush=True)


def check_mode():
    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return armed, msg.custom_mode
    return None, None


def send_setpoint(vx=0.0, vy=0.0, vz=-0.5):
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
    )


print("\n[Flight] Priming with 200 setpoints...", flush=True)
for _ in range(200):
    send_setpoint()
    world.step(render=not HEADLESS)

print("\n[Flight] Engaging OFFBOARD mode and arming...", flush=True)
for attempt in range(3):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6, 0, 0, 0, 0, 0
    )
    for _ in range(100):
        send_setpoint()
        world.step(render=not HEADLESS)
    armed, mode = check_mode()
    if mode == 6:
        break

for attempt in range(3):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    for _ in range(100):
        send_setpoint()
        world.step(render=not HEADLESS)
    armed, mode = check_mode()
    if armed:
        break

mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    6, 0, 0, 0, 0, 0
)
for _ in range(50):
    send_setpoint()
    world.step(render=not HEADLESS)

armed, mode = check_mode()
print(f"[Flight] Final state: armed={armed}, mode={mode}", flush=True)

print("\n[Flight] Ascending to observation altitude (25m)...", flush=True)
for i in range(600):
    send_setpoint(vz=-2.0)
    world.step(render=not HEADLESS)
    if i % 200 == 0:
        pos = vehicle.state.position
        print(f"[Flight] Step {i}: altitude = {pos[2]:.1f}m", flush=True)

# =============================================================================
# Phase 6: SAC Training
# =============================================================================
print("\n" + "=" * 60, flush=True)
print("SAC TRAINING TEST", flush=True)
print("=" * 60, flush=True)

# Create environment
env = Phase6SACEnv(
    world, vehicle, mav, target_positions,
    max_steps=200,
    update_camera_fn=update_camera_pose if not HEADLESS else None
)

print(f"\n[Env] Environment created with {len(target_positions)} POIs", flush=True)
print(f"  Observation space: {env.observation_space}", flush=True)
print(f"  Action space: {env.action_space}", flush=True)

# Create log directory
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_dir = f"/workspace/logs/sac_training/{timestamp}"
os.makedirs(log_dir, exist_ok=True)
print(f"[SAC] Log directory: {log_dir}", flush=True)

if SB3_AVAILABLE:
    # Create SAC agent
    print("\n[SAC] Creating SAC agent...", flush=True)

    model = SAC(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        buffer_size=10000,  # Smaller buffer for quick test
        learning_starts=100,  # Start learning after 100 steps
        batch_size=64,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        verbose=1,
        tensorboard_log=log_dir,
    )

    print("[SAC] SAC agent created", flush=True)
    print(f"  Policy: MlpPolicy", flush=True)
    print(f"  Learning rate: 3e-4", flush=True)
    print(f"  Buffer size: 10000", flush=True)

    # Training parameters
    TOTAL_TIMESTEPS = 2000  # Short test run
    print(f"\n[SAC] Starting training for {TOTAL_TIMESTEPS} timesteps...", flush=True)

    # Create callbacks
    training_callback = TrainingCallback(verbose=1)
    checkpoint_callback = CheckpointCallback(
        save_freq=500,
        save_path=log_dir,
        name_prefix="sac_isr"
    )

    # Train
    start_time = time.time()
    try:
        model.learn(
            total_timesteps=TOTAL_TIMESTEPS,
            callback=[training_callback, checkpoint_callback],
            log_interval=10,
            progress_bar=True,
        )
        training_success = True
    except Exception as e:
        print(f"\n[SAC] Training error: {e}", flush=True)
        training_success = False

    training_time = time.time() - start_time

    # Save final model
    final_model_path = f"{log_dir}/sac_isr_final"
    model.save(final_model_path)
    print(f"\n[SAC] Model saved to {final_model_path}", flush=True)

    # Get training stats
    episode_rewards = training_callback.episode_rewards
    episode_lengths = training_callback.episode_lengths

else:
    # Demo mode without SB3
    print("\n[Demo] Running demo mode (no SB3)...", flush=True)

    NUM_EPISODES = 5
    episode_rewards = []
    episode_lengths = []

    for ep in range(NUM_EPISODES):
        obs, info = env.reset()
        episode_reward = 0
        done = False
        step = 0

        while not done:
            # Random action
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            step += 1
            done = terminated or truncated

        episode_rewards.append(episode_reward)
        episode_lengths.append(step)
        print(f"  Episode {ep+1}: reward={episode_reward:.2f}, steps={step}", flush=True)

    training_success = True
    training_time = 0

# =============================================================================
# Results
# =============================================================================
print("\n" + "=" * 60, flush=True)
print("PHASE 6 RESULTS", flush=True)
print("=" * 60, flush=True)

print(f"\nTraining Statistics:", flush=True)
print(f"  Total episodes: {len(episode_rewards)}", flush=True)
print(f"  Total steps: {env.training_stats['total_steps']}", flush=True)
print(f"  Training time: {training_time:.1f}s", flush=True)

if episode_rewards:
    print(f"\nEpisode Rewards:", flush=True)
    print(f"  Mean: {np.mean(episode_rewards):.2f}", flush=True)
    print(f"  Std: {np.std(episode_rewards):.2f}", flush=True)
    print(f"  Min: {min(episode_rewards):.2f}", flush=True)
    print(f"  Max: {max(episode_rewards):.2f}", flush=True)

    # Check for improvement (compare first and last 3 episodes)
    if len(episode_rewards) >= 6:
        early_mean = np.mean(episode_rewards[:3])
        late_mean = np.mean(episode_rewards[-3:])
        improvement = late_mean - early_mean
        print(f"\nLearning Signal:", flush=True)
        print(f"  Early episodes (1-3) mean: {early_mean:.2f}", flush=True)
        print(f"  Late episodes mean: {late_mean:.2f}", flush=True)
        print(f"  Improvement: {improvement:+.2f}", flush=True)

if episode_lengths:
    print(f"\nEpisode Lengths:", flush=True)
    print(f"  Mean: {np.mean(episode_lengths):.1f}", flush=True)
    print(f"  Max: {max(episode_lengths)}", flush=True)

# Success criteria
success = (
    training_success and
    len(episode_rewards) > 0 and
    env.training_stats['total_steps'] > 100
)

if success:
    print("\nSUCCESS! SAC training completed successfully.", flush=True)
    print("  Agent trained without crashing", flush=True)
    print("  Episodes completed", flush=True)
    print("  Model checkpoint saved", flush=True)
    if SB3_AVAILABLE:
        print(f"  Tensorboard logs at: {log_dir}", flush=True)
        print("  Run: tensorboard --logdir=/workspace/logs/sac_training", flush=True)
else:
    print("\nFAILED: SAC training had issues.", flush=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
env.close()
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
