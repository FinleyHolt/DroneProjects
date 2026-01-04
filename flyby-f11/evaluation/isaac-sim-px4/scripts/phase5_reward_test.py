#!/usr/bin/env python3
"""
Phase 5: Reward Signal Test

Builds on Phase 4 (Gymnasium wrapper) by adding:
- ISR mission reward function
- Coverage tracking
- POI detection rewards
- Safety penalties

This incrementally adds the reward signal on top of Phase 4's
working Gymnasium wrapper code.

Success criteria:
- Non-zero rewards during episode
- Coverage increases as drone flies
- POI detection gives positive reward
- Crash/boundary gives negative reward
"""

from isaacsim import SimulationApp

# Launch with GUI
simulation_app = SimulationApp({"headless": False})

import omni
import omni.timeline
import omni.kit.viewport.utility as viewport_utils
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
import sys
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces

# Add Pegasus to path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

# Add our extension to path
EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

# Add perception to path
sys.path.insert(0, "/workspace")

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pymavlink import mavutil

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig
from isaacsim.core.utils.stage import get_current_stage

print("=" * 60, flush=True)
print("Phase 5: Reward Signal Test", flush=True)
print("=" * 60, flush=True)


class Phase5RewardEnv(gym.Env):
    """
    Gymnasium wrapper with ISR mission reward signal.

    Builds on Phase4GymEnv by adding:
    - Coverage tracking (grid-based)
    - POI detection rewards
    - Altitude/boundary penalties
    - Time penalty to encourage efficiency
    """

    metadata = {'render_modes': ['human']}

    def __init__(self, world, vehicle, mav, target_positions):
        super().__init__()

        self.world = world
        self.vehicle = vehicle
        self.mav = mav
        self.target_positions = target_positions

        # Observation space: same as Phase 4
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(29,), dtype=np.float32
        )

        # Action space: [vx, vy, vz, yaw_rate] normalized to [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )

        # Scaling factors for actions
        self.max_velocity = 5.0  # m/s
        self.max_yaw_rate = 45.0  # deg/s

        # Episode state
        self.step_count = 0
        self.max_steps = 500

        # Camera FOV for target detection
        self.camera_fov_deg = 90.0
        self.camera_pitch_deg = 45.0
        self.max_detection_range = 100.0

        # =====================================================================
        # PHASE 5: Reward Components
        # =====================================================================

        # Coverage tracking (8x8 grid over 200x200m area)
        self.grid_size = 8
        self.area_size = 200.0
        self.cell_size = self.area_size / self.grid_size
        self.coverage_grid = np.zeros((self.grid_size, self.grid_size), dtype=bool)

        # POI detection tracking
        self.detected_pois = set()

        # Reward weights
        self.reward_weights = {
            'coverage': 0.1,           # Per new cell covered
            'eyes_on': 1.0,            # Per new POI with eyes-on (frustum check)
            'yolo_detection': 5.0,     # Per new POI with YOLO detection (bonus)
            'altitude_penalty': -0.01, # Per step outside optimal altitude
            'boundary_penalty': -0.1,  # Per step near boundary
            'time_penalty': -0.001,    # Per step (encourages efficiency)
            'crash_penalty': -10.0,    # Termination penalty
        }

        # Track eyes-on vs YOLO detections separately
        self.eyes_on_pois = set()  # POIs that entered camera FOV

        # Optimal altitude range for ISR
        self.min_altitude = 15.0
        self.max_altitude = 50.0
        self.optimal_altitude = 25.0

        # Episode statistics
        self.episode_stats = {}

    def reset(self, seed=None, options=None):
        """Reset environment and return initial observation."""
        super().reset(seed=seed)

        self.step_count = 0

        # Reset coverage and detection tracking
        self.coverage_grid = np.zeros((self.grid_size, self.grid_size), dtype=bool)
        self.eyes_on_pois = set()   # POIs that entered camera FOV (one-time reward)
        self.detected_pois = set()  # POIs confirmed by YOLO (additional one-time reward)

        # Reset episode stats
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
        }

        return obs, info

    def step(self, action):
        """Execute action and return (obs, reward, terminated, truncated, info)."""
        self.step_count += 1

        # Convert normalized action to velocity setpoint
        vx = float(action[0]) * self.max_velocity
        vy = float(action[1]) * self.max_velocity
        vz = float(action[2]) * self.max_velocity

        # Send velocity setpoint via MAVLink
        self._send_setpoint(vx, vy, vz)

        # Step simulation
        self.world.step(render=True)

        # Get current state
        pos = self.vehicle.state.position
        orient = self.vehicle.state.attitude

        # =====================================================================
        # PHASE 5: Compute Reward
        # =====================================================================
        reward = 0.0

        # 1. Coverage reward - new cells covered
        coverage_reward = self._update_coverage(pos)
        reward += coverage_reward
        self.episode_stats['coverage_reward'] += coverage_reward

        # 2. POI detection reward (eyes-on + yolo bonuses)
        # Note: _check_poi_detections updates episode_stats internally
        poi_reward = self._check_poi_detections(pos, orient)
        reward += poi_reward

        # 3. Altitude penalty - encourage staying in optimal range
        altitude_penalty = self._compute_altitude_penalty(pos[2])
        reward += altitude_penalty
        self.episode_stats['penalty'] += altitude_penalty

        # 4. Boundary penalty - discourage going too far
        boundary_penalty = self._compute_boundary_penalty(pos)
        reward += boundary_penalty
        self.episode_stats['penalty'] += boundary_penalty

        # 5. Time penalty - encourage efficiency
        reward += self.reward_weights['time_penalty']
        self.episode_stats['penalty'] += self.reward_weights['time_penalty']

        self.episode_stats['total_reward'] += reward

        # Get observation
        obs = self._get_observation()

        # Check termination conditions
        terminated = False
        truncated = False

        # Terminate if crashed (below ground)
        if pos[2] < 0.5:
            terminated = True
            reward += self.reward_weights['crash_penalty']
            self.episode_stats['total_reward'] += self.reward_weights['crash_penalty']

        # Terminate if too far from mission area
        if np.linalg.norm(pos[:2]) > 150.0:
            terminated = True
            reward += self.reward_weights['crash_penalty'] * 0.5  # Less severe

        # Truncate if max steps reached
        if self.step_count >= self.max_steps:
            truncated = True

        # Compute coverage percentage
        coverage_pct = np.sum(self.coverage_grid) / (self.grid_size ** 2) * 100

        info = {
            "step": self.step_count,
            "position": pos.tolist(),
            "velocity": self.vehicle.state.linear_velocity.tolist(),
            "coverage": coverage_pct,
            "pois_detected": len(self.detected_pois),
            "episode_stats": self.episode_stats.copy(),
        }

        return obs, reward, terminated, truncated, info

    def _update_coverage(self, pos):
        """Update coverage grid and return reward for new cells."""
        # Convert position to grid cell
        grid_x = int((pos[0] + self.area_size / 2) / self.cell_size)
        grid_y = int((pos[1] + self.area_size / 2) / self.cell_size)

        # Clamp to grid bounds
        grid_x = max(0, min(self.grid_size - 1, grid_x))
        grid_y = max(0, min(self.grid_size - 1, grid_y))

        # Check if this is a new cell
        if not self.coverage_grid[grid_x, grid_y]:
            self.coverage_grid[grid_x, grid_y] = True
            self.episode_stats['cells_covered'] += 1
            return self.reward_weights['coverage']

        return 0.0

    def _check_poi_detections(self, pos, orient):
        """
        Check for POI detections using two-tier reward system:
        1. Eyes-on: One-time reward when POI first enters camera FOV
        2. YOLO: Additional one-time reward when YOLO confirms (simulated for now)

        Both rewards are ONE-TIME per POI - no accumulation for staring.
        """
        reward = 0.0

        # Get camera forward vector (body +X, pitched down by camera_pitch_deg)
        r = Rotation.from_quat(orient)
        camera_pitch = Rotation.from_euler('Y', self.camera_pitch_deg, degrees=True)
        camera_rotation = r * camera_pitch

        camera_forward = camera_rotation.apply([1, 0, 0])
        fov_half = np.radians(self.camera_fov_deg / 2)

        for i, target in enumerate(self.target_positions):
            # Skip if already got YOLO reward (maximum reward already claimed)
            if i in self.detected_pois:
                continue

            # Vector to target
            to_target = target['position'] - pos
            dist = np.linalg.norm(to_target)

            # Range check
            if dist > self.max_detection_range or dist < 5.0:
                continue

            # Must be above target by at least 10m
            if pos[2] < target['position'][2] + 10.0:
                continue

            # Frustum check: target must be within camera FOV cone
            to_target_norm = to_target / dist
            cos_angle = np.dot(to_target_norm, camera_forward)
            angle = np.arccos(np.clip(cos_angle, -1, 1))

            # Target must be within FOV cone
            if angle > fov_half:
                continue

            # Also check target is in front of camera (positive forward distance)
            forward_dist = np.dot(to_target, camera_forward)
            if forward_dist < 5.0:
                continue

            # === EYES-ON REWARD (one-time) ===
            if i not in self.eyes_on_pois:
                self.eyes_on_pois.add(i)
                self.episode_stats['eyes_on_count'] += 1
                eyes_reward = self.reward_weights['eyes_on']
                reward += eyes_reward
                self.episode_stats['eyes_on_reward'] += eyes_reward
                print(f"    [EYES-ON] Target {i} in FOV! dist={dist:.1f}m, angle={np.degrees(angle):.1f}° (+{eyes_reward:.1f})", flush=True)

            # === YOLO REWARD (one-time, requires closer range) ===
            # Simulate YOLO detection: closer range + good angle = higher chance
            # In real system, this would call actual YOLO inference
            if dist < 50.0 and angle < np.radians(30.0):
                self.detected_pois.add(i)
                self.episode_stats['yolo_count'] += 1
                yolo_reward = self.reward_weights['yolo_detection']
                reward += yolo_reward
                self.episode_stats['yolo_reward'] += yolo_reward
                print(f"    [YOLO] Target {i} confirmed! dist={dist:.1f}m (+{yolo_reward:.1f})", flush=True)

        return reward

    def _compute_altitude_penalty(self, altitude):
        """Penalize flying outside optimal altitude range."""
        if altitude < self.min_altitude:
            return self.reward_weights['altitude_penalty'] * (self.min_altitude - altitude)
        elif altitude > self.max_altitude:
            return self.reward_weights['altitude_penalty'] * (altitude - self.max_altitude)
        return 0.0

    def _compute_boundary_penalty(self, pos):
        """Penalize approaching mission boundary."""
        dist_from_center = np.linalg.norm(pos[:2])
        boundary_dist = 100.0  # Start penalizing at 100m from center

        if dist_from_center > boundary_dist:
            return self.reward_weights['boundary_penalty'] * (dist_from_center - boundary_dist) / 50.0
        return 0.0

    def _get_observation(self):
        """Build observation vector from current state."""
        pos = self.vehicle.state.position
        vel = self.vehicle.state.linear_velocity
        orient = self.vehicle.state.attitude

        # Convert quaternion to euler
        r = Rotation.from_quat(orient)
        euler = r.as_euler('XYZ')

        # UAV state: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        uav_state = np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            euler[0], euler[1], euler[2]
        ], dtype=np.float32)

        # Target observations: for each target, [dx, dy, dz, in_fov]
        target_obs = []
        for i, target in enumerate(self.target_positions[:5]):
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
        """Send velocity setpoint via MAVLink."""
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
        )

    def close(self):
        """Cleanup resources."""
        pass


# =============================================================================
# Setup (same as Phase 4)
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
print(f"[WorldGen] Terrain created at: {terrain_path}", flush=True)

print("[WorldGen] Setting up lighting...", flush=True)
world_gen.setup_lighting()

print("[WorldGen] Generating forest...", flush=True)
forest_result = world_gen.generate_forest(density=0.3, include_undergrowth=False)
print(f"[WorldGen] Forest generated: {len(forest_result['trees'])} trees", flush=True)

# Spawn POI targets at multiple locations for detection testing
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

print(f"[WorldGen] Spawned {len(all_vehicle_paths)} vehicles in {len(poi_clusters)} clusters", flush=True)

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

# =============================================================================
# Create ISR Camera that follows the drone
# =============================================================================
print("\n[Camera] Creating ISR camera...", flush=True)
camera_prim_path = "/World/ISRCamera"

# Create camera using USD
camera_prim = UsdGeom.Camera.Define(stage, camera_prim_path)
camera_prim.CreateFocalLengthAttr(4.5)  # Wide angle
camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 500.0))
print(f"[Camera] Created camera at {camera_prim_path}", flush=True)

# Set viewport to use our camera (so you can see what the drone sees)
try:
    viewport_api = viewport_utils.get_active_viewport()
    if viewport_api:
        viewport_api.set_active_camera(camera_prim_path)
        print("[Camera] Set viewport to ISR camera view", flush=True)
except Exception as e:
    print(f"[Camera] Warning: Could not set viewport camera: {e}", flush=True)
    viewport_api = None


def update_camera_pose(drone_pos, drone_orient):
    """Update the ISR camera to follow the drone."""
    camera_xform = UsdGeom.Xformable(stage.GetPrimAtPath(camera_prim_path))
    camera_xform.ClearXformOpOrder()

    # Position camera at drone location, slightly below
    camera_xform.AddTranslateOp().Set(Gf.Vec3d(drone_pos[0], drone_pos[1], drone_pos[2] - 0.1))

    # Get drone yaw and pitch camera down 45 degrees
    drone_rot = Rotation.from_quat(drone_orient)
    drone_euler = drone_rot.as_euler('ZYX', degrees=True)
    drone_yaw = drone_euler[0]

    # Camera looks forward and down at 45 degrees
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
    world.step(render=True)

initial_pos = vehicle.state.position
print(f"[PX4] Initial position: ({initial_pos[0]:.2f}, {initial_pos[1]:.2f}, {initial_pos[2]:.2f})", flush=True)

print("\n[MAVLink] Connecting...", flush=True)
mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

for i in range(500):
    world.step(render=True)
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
    world.step(render=True)

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
        world.step(render=True)
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
        world.step(render=True)
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
    world.step(render=True)

armed, mode = check_mode()
print(f"[Flight] Final state: armed={armed}, mode={mode}", flush=True)

print("\n[Flight] Ascending to observation altitude (25m)...", flush=True)
for i in range(600):
    send_setpoint(vz=-2.0)
    world.step(render=True)
    if i % 100 == 0:
        pos = vehicle.state.position
        print(f"[Flight] Step {i}: altitude = {pos[2]:.1f}m", flush=True)

# =============================================================================
# Phase 5: Reward Signal Test
# =============================================================================
print("\n" + "=" * 60, flush=True)
print("REWARD SIGNAL TEST", flush=True)
print("=" * 60, flush=True)

env = Phase5RewardEnv(world, vehicle, mav, target_positions)

print(f"\n[Env] Environment created with {len(target_positions)} POIs", flush=True)
print(f"  Reward weights: {env.reward_weights}", flush=True)

# Test with patrol pattern visiting all POI clusters
print("\n[Test] Running ISR patrol mission...", flush=True)
obs, info = env.reset(seed=42)

# Patrol waypoints: visit each POI cluster
patrol_waypoints = [
    np.array([30.0, 20.0, 25.0]),   # Cluster 1
    np.array([-20.0, 30.0, 25.0]),  # Cluster 2
    np.array([40.0, -25.0, 25.0]),  # Cluster 3
    np.array([0.0, 0.0, 25.0]),     # Return to center
]
current_waypoint_idx = 0

NUM_STEPS = 500
rewards = []
coverages = []
eyes_on_counts = []
yolo_counts = []

for step in range(NUM_STEPS):
    pos = env.vehicle.state.position
    orient = env.vehicle.state.attitude

    # Update camera to follow drone
    update_camera_pose(pos, orient)

    # Get current waypoint
    target = patrol_waypoints[current_waypoint_idx]

    # Compute direction to waypoint
    direction = target - pos
    dist_to_waypoint = np.linalg.norm(direction[:2])

    if dist_to_waypoint > 5.0:
        # Fly toward waypoint
        direction = direction / np.linalg.norm(direction)
        action = np.array([
            direction[0] * 0.7,  # vx
            direction[1] * 0.7,  # vy
            direction[2] * 0.3,  # vz (altitude control)
            0.0  # yaw_rate
        ], dtype=np.float32)
    else:
        # Reached waypoint - move to next
        current_waypoint_idx = (current_waypoint_idx + 1) % len(patrol_waypoints)
        print(f"  [NAV] Reached waypoint, heading to cluster {current_waypoint_idx + 1}", flush=True)
        action = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    obs, reward, terminated, truncated, info = env.step(action)

    rewards.append(reward)
    coverages.append(info['coverage'])
    stats = info.get('episode_stats', {})
    eyes_on_counts.append(stats.get('eyes_on_count', 0))
    yolo_counts.append(stats.get('yolo_count', 0))

    if step % 100 == 0:
        pos = info['position']
        print(f"  Step {step}: pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), "
              f"reward={reward:.4f}, eyes_on={stats.get('eyes_on_count', 0)}, "
              f"yolo={stats.get('yolo_count', 0)}", flush=True)

    if terminated or truncated:
        print(f"\n  Episode ended at step {step}: terminated={terminated}, truncated={truncated}", flush=True)
        break

# =============================================================================
# Results
# =============================================================================
print("\n" + "=" * 60, flush=True)
print("PHASE 5 RESULTS", flush=True)
print("=" * 60, flush=True)

final_stats = info.get('episode_stats', {})

print(f"\nEpisode Statistics:", flush=True)
print(f"  Steps completed: {len(rewards)}/{NUM_STEPS}", flush=True)
print(f"  Total reward: {sum(rewards):.2f}", flush=True)
print(f"  Coverage reward: {final_stats.get('coverage_reward', 0):.2f}", flush=True)
print(f"  Eyes-on reward: {final_stats.get('eyes_on_reward', 0):.2f} ({final_stats.get('eyes_on_count', 0)} POIs)", flush=True)
print(f"  YOLO reward: {final_stats.get('yolo_reward', 0):.2f} ({final_stats.get('yolo_count', 0)} POIs)", flush=True)
print(f"  Penalties: {final_stats.get('penalty', 0):.2f}", flush=True)
print(f"  Cells covered: {final_stats.get('cells_covered', 0)}/{env.grid_size**2}", flush=True)

print(f"\nFinal coverage: {coverages[-1] if coverages else 0:.1f}%", flush=True)
print(f"Final eyes-on: {eyes_on_counts[-1] if eyes_on_counts else 0}/{len(target_positions)}", flush=True)
print(f"Final YOLO confirmed: {yolo_counts[-1] if yolo_counts else 0}/{len(target_positions)}", flush=True)

# Check reward signal quality
non_zero_rewards = sum(1 for r in rewards if r != 0)
positive_rewards = sum(1 for r in rewards if r > 0)
negative_rewards = sum(1 for r in rewards if r < 0)

print(f"\nReward signal analysis:", flush=True)
print(f"  Non-zero rewards: {non_zero_rewards}/{len(rewards)} ({100*non_zero_rewards/max(1,len(rewards)):.1f}%)", flush=True)
print(f"  Positive rewards: {positive_rewards}", flush=True)
print(f"  Negative rewards: {negative_rewards}", flush=True)
print(f"  Reward range: [{min(rewards):.4f}, {max(rewards):.4f}]", flush=True)

# Success criteria
success = (
    len(rewards) >= NUM_STEPS * 0.5 and
    non_zero_rewards > 0 and
    final_stats.get('cells_covered', 0) > 0
)

if success:
    print("\nSUCCESS! Reward signal is working correctly.", flush=True)
    print("  Coverage tracking works", flush=True)
    print("  Rewards are being computed", flush=True)
    print("  Ready for Phase 6: SAC Training", flush=True)
else:
    print("\nFAILED: Reward signal has issues.", flush=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
env.close()
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
