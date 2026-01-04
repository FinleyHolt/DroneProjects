#!/usr/bin/env python3
"""
Phase 7: Safety Filter Integration Test with Gimbal Control

Builds on Phase 6 (SAC Training) by adding:
- VampireSafetyFilter to check actions against ontology constraints
- Proper action execution with altitude safety
- Debug logging to verify drone is responding to commands
- Safe fallback actions when violations detected
- GIMBAL CONTROL: Agent can manipulate camera pitch/yaw
- YAW CONTROL: Agent can rotate the drone body

This tests the integration of the ontology-based safety filter
with the RL training loop, plus gimbal-controlled camera for ISR.

Success criteria:
- Safety filter catches constraint violations
- Drone responds to agent velocity commands (vx, vy, vz)
- Drone responds to yaw rate commands
- Gimbal pitch/yaw controls camera view
- Fallback actions are used when needed
- Episode doesn't crash immediately
"""

from isaacsim import SimulationApp

import sys
HEADLESS = "--headless" in sys.argv
simulation_app = SimulationApp({"headless": HEADLESS})

import omni
import omni.timeline
import omni.kit.viewport.utility as viewport_utils
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
from isaacsim.sensors.camera import Camera
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import os
from datetime import datetime

# Add paths
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

sys.path.insert(0, "/workspace")

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pymavlink import mavutil

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig
from isaacsim.core.utils.stage import get_current_stage

# Import safety filter
from environments.safety_filter import VampireSafetyFilter

print("=" * 60, flush=True)
print("Phase 7: Safety Filter Integration Test", flush=True)
print("=" * 60, flush=True)
print(f"  Headless: {HEADLESS}", flush=True)


class Phase7SafeEnv(gym.Env):
    """
    Gymnasium environment with integrated safety filter and gimbal control.

    Key improvements:
    1. Actions are filtered through VampireSafetyFilter before execution
    2. Altitude is protected - can't command rapid descent
    3. Geofence violations are blocked
    4. Debug logging shows what's happening
    5. GIMBAL CONTROL: Agent controls camera pitch/yaw independently of drone
    6. YAW CONTROL: Agent can rotate drone body
    """

    metadata = {'render_modes': ['human']}

    def __init__(self, world, vehicle, mav, target_positions,
                 max_steps=200, isr_camera=None):
        super().__init__()

        self.world = world
        self.vehicle = vehicle
        self.mav = mav
        self.target_positions = target_positions
        self.isr_camera = isr_camera  # Isaac Sim Camera object for gimbal control

        # Observation space: [x,y,z, vx,vy,vz, roll,pitch,yaw, gimbal_pitch, gimbal_yaw, target_obs(20)]
        # Added gimbal state to observation (31 total)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(31,), dtype=np.float32
        )

        # Action space: [vx, vy, vz_delta, yaw_rate, gimbal_pitch, gimbal_yaw] normalized to [-1, 1]
        # 6D action space: drone velocity + yaw + gimbal control
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32
        )

        # Scaling factors
        self.max_horizontal_velocity = 5.0  # m/s
        self.max_vertical_velocity = 2.0    # m/s (slower for safety)
        self.max_yaw_rate = 45.0            # deg/s

        # Gimbal limits (degrees)
        self.gimbal_pitch_min = 0.0    # Looking forward
        self.gimbal_pitch_max = 90.0   # Looking straight down
        self.gimbal_yaw_min = -60.0    # Left
        self.gimbal_yaw_max = 60.0     # Right

        # Current gimbal state (degrees)
        self.gimbal_pitch = 45.0  # Default: 45 degrees down
        self.gimbal_yaw = 0.0     # Default: centered

        # Target altitude for ISR operations
        self.target_altitude = 25.0  # meters
        self.altitude_tolerance = 5.0  # allowed deviation

        # Episode state
        self.step_count = 0
        self.max_steps = max_steps
        self.episode_count = 0

        # Camera parameters (for frustum calculations)
        self.camera_fov_deg = 90.0
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
            'safety_violation_penalty': -1.0,  # NEW: penalty for unsafe actions
        }

        # Altitude constraints
        self.min_altitude = 10.0
        self.max_altitude = 50.0

        # =====================================================================
        # PHASE 7: Safety Filter
        # =====================================================================
        self.safety_filter = VampireSafetyFilter(
            ontology_path="/workspace/ontology/planning_mode",
            timeout_ms=50,
            enabled=True
        )

        # Configure geofence matching our mission area
        self.safety_filter.update_geofence({
            'min_x': -100.0, 'max_x': 100.0,
            'min_y': -100.0, 'max_y': 100.0,
            'min_z': 5.0, 'max_z': 60.0,  # Altitude limits
        })

        # Track safety filter statistics
        self.safety_stats = {
            'total_checks': 0,
            'violations_blocked': 0,
            'soft_penalties': 0.0,
        }

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

        # Reset gimbal to default position
        self.gimbal_pitch = 45.0  # 45 degrees down
        self.gimbal_yaw = 0.0     # Centered
        self._update_gimbal()

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
            'safety_violations': 0,
        }

        # Get initial observation
        obs = self._get_observation()

        # Log episode start
        pos = self.vehicle.state.position
        print(f"\n[Episode {self.episode_count}] Starting at altitude {pos[2]:.1f}m, gimbal at pitch={self.gimbal_pitch:.0f}°", flush=True)

        info = {
            "step": 0,
            "position": pos.tolist(),
            "coverage": 0.0,
            "pois_detected": 0,
            "episode": self.episode_count,
        }

        return obs, info

    def step(self, action):
        """Execute action with safety filtering and gimbal control."""
        self.step_count += 1

        # Get current state for safety check
        pos = self.vehicle.state.position
        current_state = {
            'x': float(pos[0]),
            'y': float(pos[1]),
            'z': float(pos[2]),
            'battery': 80.0,  # Simulated battery (could track real consumption)
        }

        # =====================================================================
        # Parse 6D action: [vx, vy, vz_delta, yaw_rate, gimbal_pitch, gimbal_yaw]
        # =====================================================================

        # Drone velocity control (actions 0-2)
        vx = float(action[0]) * self.max_horizontal_velocity
        vy = float(action[1]) * self.max_horizontal_velocity

        # Vertical: relative to altitude hold (action[2] adjusts target altitude)
        altitude_error = self.target_altitude - pos[2]
        vz_agent = float(action[2]) * self.max_vertical_velocity
        vz = -altitude_error * 0.5 + vz_agent  # P-controller + agent adjustment
        vz = np.clip(vz, -self.max_vertical_velocity, self.max_vertical_velocity)

        # Yaw rate control (action 3) - NOW ACTUALLY USED
        yaw_rate = float(action[3]) * self.max_yaw_rate  # degrees/s
        yaw_rate_rad = np.radians(yaw_rate)

        # Gimbal control (actions 4-5) - pitch and yaw rates
        gimbal_pitch_rate = float(action[4]) * 30.0  # degrees/s
        gimbal_yaw_rate = float(action[5]) * 30.0    # degrees/s

        # Update gimbal state (integrate rate commands)
        self.gimbal_pitch += gimbal_pitch_rate * 0.1  # dt = 0.1s
        self.gimbal_yaw += gimbal_yaw_rate * 0.1

        # Clamp gimbal to limits
        self.gimbal_pitch = np.clip(self.gimbal_pitch, self.gimbal_pitch_min, self.gimbal_pitch_max)
        self.gimbal_yaw = np.clip(self.gimbal_yaw, self.gimbal_yaw_min, self.gimbal_yaw_max)

        raw_action = np.array([vx, vy, vz, yaw_rate_rad], dtype=np.float32)

        # =====================================================================
        # PHASE 7: Safety Filter Check
        # =====================================================================
        self.safety_stats['total_checks'] += 1

        safe_action, was_modified, violation = self.safety_filter.filter_action(
            current_state, raw_action, dt=0.1
        )

        safety_penalty = 0.0
        if was_modified:
            self.safety_stats['violations_blocked'] += 1
            self.episode_stats['safety_violations'] += 1
            safety_penalty = self.reward_weights['safety_violation_penalty']

            if self.step_count <= 10 or self.step_count % 50 == 0:
                print(f"  [SAFETY] Step {self.step_count}: Blocked {violation}, "
                      f"using fallback hover", flush=True)

        # Execute safe action (velocity + yaw rate)
        self._send_setpoint(safe_action[0], safe_action[1], safe_action[2], yaw_rate_rad)

        # Update gimbal camera pose
        self._update_gimbal()

        # Step simulation
        self.world.step(render=not HEADLESS)

        # Get new state
        pos = self.vehicle.state.position
        orient = self.vehicle.state.attitude

        # Compute reward
        reward = safety_penalty  # Start with safety penalty if any

        # 1. Coverage reward
        coverage_reward = self._update_coverage(pos)
        reward += coverage_reward
        self.episode_stats['coverage_reward'] += coverage_reward

        # 2. POI detection reward
        poi_reward = self._check_poi_detections(pos, orient)
        reward += poi_reward

        # 3. Altitude penalty (staying in optimal range)
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
        if pos[2] < 1.0:
            terminated = True
            reward += self.reward_weights['crash_penalty']
            print(f"  [CRASH] Episode {self.episode_count} crashed at step {self.step_count}, "
                  f"altitude={pos[2]:.2f}m", flush=True)

        # Boundary violation (shouldn't happen if safety filter works)
        if np.linalg.norm(pos[:2]) > 120.0:
            terminated = True
            reward += self.reward_weights['crash_penalty'] * 0.5
            print(f"  [BOUNDARY] Episode {self.episode_count} exceeded boundary at step {self.step_count}", flush=True)

        # Max steps
        if self.step_count >= self.max_steps:
            truncated = True

        # Debug logging with gimbal state
        if self.step_count % 50 == 0:
            print(f"  Step {self.step_count}: pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), "
                  f"vel=({vx:.1f}, {vy:.1f}, {vz:.1f}), yaw_rate={np.degrees(yaw_rate_rad):.1f}°/s, "
                  f"gimbal=({self.gimbal_pitch:.0f}°, {self.gimbal_yaw:.0f}°), reward={reward:.3f}", flush=True)

        # Update training stats on episode end
        if terminated or truncated:
            self.training_stats['episodes'] += 1
            ep_reward = self.episode_stats['total_reward']
            self.training_stats['recent_rewards'].append(ep_reward)
            if len(self.training_stats['recent_rewards']) > 100:
                self.training_stats['recent_rewards'].pop(0)
            if ep_reward > self.training_stats['best_reward']:
                self.training_stats['best_reward'] = ep_reward

            print(f"  [END] Episode {self.episode_count}: steps={self.step_count}, "
                  f"reward={ep_reward:.2f}, safety_violations={self.episode_stats['safety_violations']}", flush=True)

        self.training_stats['total_steps'] += 1

        coverage_pct = np.sum(self.coverage_grid) / (self.grid_size ** 2) * 100

        info = {
            "step": self.step_count,
            "position": pos.tolist(),
            "coverage": coverage_pct,
            "pois_detected": len(self.detected_pois),
            "episode_stats": self.episode_stats.copy(),
            "safety_stats": self.safety_stats.copy(),
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
        """Check for POI detections using gimbal-controlled camera direction."""
        reward = 0.0

        # Get drone body rotation
        drone_rot = Rotation.from_quat(orient)
        drone_euler = drone_rot.as_euler('ZYX', degrees=True)
        drone_yaw = drone_euler[0]

        # Camera direction = drone yaw + gimbal yaw, then pitch down by gimbal pitch
        # World-frame camera direction considering both drone heading and gimbal angles
        total_yaw = drone_yaw + self.gimbal_yaw
        camera_rotation = Rotation.from_euler("ZY", [total_yaw, self.gimbal_pitch], degrees=True)
        camera_forward = camera_rotation.apply([1, 0, 0])  # Forward in world frame

        fov_half = np.radians(self.camera_fov_deg / 2)

        for i, target in enumerate(self.target_positions):
            if i in self.detected_pois:
                continue

            to_target = target['position'] - pos
            dist = np.linalg.norm(to_target)

            if dist > self.max_detection_range or dist < 5.0:
                continue

            # Must be above the target
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
                print(f"    [EYES-ON] Target {i} at {dist:.1f}m (gimbal: p={self.gimbal_pitch:.0f}°, y={self.gimbal_yaw:.0f}°)", flush=True)

            # YOLO reward
            if dist < 50.0 and angle < np.radians(30.0):
                self.detected_pois.add(i)
                self.episode_stats['yolo_count'] += 1
                yolo_reward = self.reward_weights['yolo_detection']
                reward += yolo_reward
                self.episode_stats['yolo_reward'] += yolo_reward
                print(f"    [YOLO] Target {i} confirmed at {dist:.1f}m", flush=True)

        return reward

    def _compute_altitude_penalty(self, altitude):
        if altitude < self.min_altitude:
            return self.reward_weights['altitude_penalty'] * (self.min_altitude - altitude)
        elif altitude > self.max_altitude:
            return self.reward_weights['altitude_penalty'] * (altitude - self.max_altitude)
        return 0.0

    def _compute_boundary_penalty(self, pos):
        dist_from_center = np.linalg.norm(pos[:2])
        boundary_dist = 80.0  # Start penalizing earlier
        if dist_from_center > boundary_dist:
            return self.reward_weights['boundary_penalty'] * (dist_from_center - boundary_dist) / 40.0
        return 0.0

    def _get_observation(self):
        """Get observation including gimbal state."""
        pos = self.vehicle.state.position
        vel = self.vehicle.state.linear_velocity
        orient = self.vehicle.state.attitude

        r = Rotation.from_quat(orient)
        euler = r.as_euler('XYZ')

        # UAV state (9) + gimbal state (2) = 11
        uav_state = np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            euler[0], euler[1], euler[2],
            np.radians(self.gimbal_pitch),  # Gimbal pitch in radians
            np.radians(self.gimbal_yaw),    # Gimbal yaw in radians
        ], dtype=np.float32)

        # Target observations (5 targets x 4 values = 20)
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
        obs = np.concatenate([uav_state, target_obs])  # 11 + 20 = 31
        return obs

    def _send_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        """Send velocity + yaw rate setpoint via MAVLink (NED frame)."""
        # Bitmask: ignore position, use velocity and yaw_rate
        # Bit 0-2: position x,y,z (1 = ignore)
        # Bit 3-5: velocity x,y,z (0 = use)
        # Bit 6-8: accel x,y,z (1 = ignore)
        # Bit 9: force (1 = ignore)
        # Bit 10: yaw (1 = ignore)
        # Bit 11: yaw_rate (0 = use)
        # 0b0000011111000111 = use velocity + yaw_rate
        type_mask = 0b0000011111000111

        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,       # Position (ignored)
            vx, vy, vz,    # Velocity in NED
            0, 0, 0,       # Acceleration (ignored)
            0, yaw_rate    # Yaw (ignored), yaw_rate (used)
        )

    def _update_gimbal(self):
        """Update the ISR camera gimbal pose."""
        if self.isr_camera is None:
            return

        # Gimbal rotation: pitch down + yaw offset
        # In Isaac Sim camera convention:
        # - Camera default looks along -Z in its local frame
        # - We need to rotate to look down and forward
        # - Pitch: 0° = forward, 90° = straight down
        # - Yaw: 0° = forward, positive = right, negative = left
        #
        # Build rotation: first rotate camera to look forward (not up),
        # then apply gimbal pitch (down), then yaw
        gimbal_rot = Rotation.from_euler("ZYX", [self.gimbal_yaw, self.gimbal_pitch, 0.0], degrees=True)

        # Set camera local pose relative to drone body
        # Camera is mounted slightly below and forward of drone center
        local_pos = np.array([0.1, 0.0, -0.05])

        try:
            self.isr_camera.set_local_pose(local_pos, gimbal_rot.as_quat())
        except Exception as e:
            # Fallback if camera API fails
            if self.step_count == 1:
                print(f"  [WARN] Could not update gimbal: {e}", flush=True)

    def close(self):
        pass


# =============================================================================
# Setup (same as Phase 6)
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

# Create PX4 vehicle FIRST (camera will be attached after)
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

# =============================================================================
# Create ISR camera AFTER world.reset() but BEFORE timeline.play()
# Camera is attached to drone body and moves with it
# =============================================================================
isr_camera = None
camera_prim_path = "/World/quadrotor/body/isr_camera"

print("\n[Camera] Creating ISR gimbal camera attached to drone...", flush=True)
try:
    # Create camera as child of drone body - it will move with the drone
    isr_camera = Camera(
        prim_path=camera_prim_path,
        frequency=30,
        resolution=(640, 480)
    )

    # Initial gimbal position: 45 degrees down, centered
    # Local pose is relative to parent (drone body)
    initial_rot = Rotation.from_euler("ZYX", [0.0, 45.0, 0.0], degrees=True)
    isr_camera.set_local_pose(
        np.array([0.1, 0.0, -0.05]),  # Forward, centered, slightly below body
        initial_rot.as_quat()
    )
    isr_camera.initialize()
    isr_camera.set_focal_length(4.5)
    isr_camera.set_clipping_range(0.1, 500.0)
    print(f"[Camera] ISR camera attached at {camera_prim_path}", flush=True)
    print("[Camera] Gimbal control enabled: pitch 0-90°, yaw ±60°", flush=True)

except Exception as e:
    print(f"[Camera] Warning: Could not create camera: {e}", flush=True)
    print("[Camera] Gimbal control will be simulated but not rendered", flush=True)
    isr_camera = None

timeline.play()

# Set viewport to camera view AFTER timeline starts (GUI only)
if not HEADLESS and isr_camera is not None:
    try:
        viewport_api = viewport_utils.get_active_viewport()
        if viewport_api:
            viewport_api.set_active_camera(camera_prim_path)
            print("[Camera] Viewport set to ISR gimbal camera view", flush=True)
    except Exception as e:
        print(f"[Camera] Warning: Could not set viewport: {e}", flush=True)

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
# Phase 7: Safety Filter Test with Gimbal Control
# =============================================================================
print("\n" + "=" * 60, flush=True)
print("SAFETY FILTER + GIMBAL CONTROL TEST", flush=True)
print("=" * 60, flush=True)

# Create environment with gimbal-controlled camera
env = Phase7SafeEnv(
    world, vehicle, mav, target_positions,
    max_steps=300,  # Longer episodes
    isr_camera=isr_camera  # Pass camera for gimbal control
)

print(f"\n[Env] Environment created with {len(target_positions)} POIs", flush=True)
print(f"  Observation space: {env.observation_space.shape} (includes gimbal state)", flush=True)
print(f"  Action space: {env.action_space.shape} (vx, vy, vz, yaw_rate, gimbal_pitch, gimbal_yaw)", flush=True)
print(f"  Safety filter enabled: {env.safety_filter.enabled}", flush=True)
print(f"  Gimbal camera: {'attached' if isr_camera else 'simulated'}", flush=True)

# Run test episodes with random actions to verify safety filter works
NUM_EPISODES = 3
episode_rewards = []
episode_lengths = []
episode_violations = []

for ep in range(NUM_EPISODES):
    obs, info = env.reset()
    episode_reward = 0
    done = False
    step = 0

    while not done:
        # Random action (will test if safety filter catches bad ones)
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        episode_reward += reward
        step += 1
        done = terminated or truncated

    episode_rewards.append(episode_reward)
    episode_lengths.append(step)
    episode_violations.append(info.get('episode_stats', {}).get('safety_violations', 0))

# =============================================================================
# Results
# =============================================================================
print("\n" + "=" * 60, flush=True)
print("PHASE 7 RESULTS", flush=True)
print("=" * 60, flush=True)

print(f"\nEpisode Statistics:", flush=True)
print(f"  Total episodes: {len(episode_rewards)}", flush=True)
print(f"  Mean reward: {np.mean(episode_rewards):.2f}", flush=True)
print(f"  Mean length: {np.mean(episode_lengths):.1f} steps", flush=True)
print(f"  Total safety violations blocked: {sum(episode_violations)}", flush=True)

print(f"\nSafety Filter Statistics:", flush=True)
print(f"  Total checks: {env.safety_stats['total_checks']}", flush=True)
print(f"  Violations blocked: {env.safety_stats['violations_blocked']}", flush=True)
block_rate = (env.safety_stats['violations_blocked'] / max(1, env.safety_stats['total_checks'])) * 100
print(f"  Block rate: {block_rate:.1f}%", flush=True)

print(f"\nPer-Episode Details:", flush=True)
for i, (reward, length, violations) in enumerate(zip(episode_rewards, episode_lengths, episode_violations)):
    print(f"  Episode {i+1}: reward={reward:.2f}, steps={length}, violations={violations}", flush=True)

# Success criteria
success = (
    np.mean(episode_lengths) > 50 and  # Episodes last longer
    env.safety_stats['total_checks'] > 0 and  # Safety filter was used
    sum(episode_violations) > 0  # Some violations were caught (proves filter works)
)

if success:
    print("\nSUCCESS! Safety filter is working.", flush=True)
    print("  Episodes completed without immediate crash", flush=True)
    print("  Safety filter is checking actions", flush=True)
    print("  Unsafe actions are being blocked", flush=True)
else:
    if np.mean(episode_lengths) <= 50:
        print("\nFAILED: Episodes too short (drone still crashing)", flush=True)
    elif env.safety_stats['total_checks'] == 0:
        print("\nFAILED: Safety filter not being used", flush=True)
    else:
        print("\nPARTIAL: Safety filter running but no violations caught", flush=True)
        print("  This might be OK if random actions happen to be safe", flush=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
env.close()
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
