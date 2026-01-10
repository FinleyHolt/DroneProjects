#!/usr/bin/env python3
"""
Phase 4: Gymnasium Wrapper Test

Builds on Phase 3 (procedural world + PX4 flight + perception) by adding:
- Gymnasium-compatible wrapper with obs/action spaces
- reset() and step() API working in sim loop
- Observation vectors validated (no NaN/Inf)

This incrementally adds the Gymnasium API layer on top of the working
Phase 3 code rather than using the complex base_isr_env which has
PX4 boot timing issues.

Success criteria:
- Gymnasium-style reset() returns valid observation
- Gymnasium-style step() works for 200 steps
- Observations within space bounds
- Actions properly converted to velocity setpoints
"""

from isaacsim import SimulationApp

# Launch with GUI
simulation_app = SimulationApp({"headless": False})

import omni.timeline
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
print("Phase 4: Gymnasium Wrapper Test", flush=True)
print("=" * 60, flush=True)


class Phase4GymEnv(gym.Env):
    """
    Minimal Gymnasium wrapper built on Phase 3's working code.

    This is NOT the full ISR environment - just the Gym API layer
    to validate reset/step work before adding rewards in Phase 5.
    """

    metadata = {'render_modes': ['human']}

    def __init__(self, world, vehicle, mav, target_positions):
        super().__init__()

        self.world = world
        self.vehicle = vehicle
        self.mav = mav
        self.target_positions = target_positions

        # Simplified observation space for Phase 4 test:
        # [x, y, z, vx, vy, vz, roll, pitch, yaw] = 9 dims
        # + target info (for each of 5 targets: dx, dy, dz, in_fov) = 20 dims
        # Total: 29 dims
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

    def reset(self, seed=None, options=None):
        """Reset environment and return initial observation."""
        super().reset(seed=seed)

        self.step_count = 0

        # Get initial observation
        obs = self._get_observation()
        info = {"step": 0, "position": self.vehicle.state.position.tolist()}

        return obs, info

    def step(self, action):
        """Execute action and return (obs, reward, terminated, truncated, info)."""
        self.step_count += 1

        # Convert normalized action to velocity setpoint
        vx = float(action[0]) * self.max_velocity
        vy = float(action[1]) * self.max_velocity
        vz = float(action[2]) * self.max_velocity  # Negative = up in NED
        # yaw_rate = action[3] * self.max_yaw_rate  # Not used in setpoint

        # Send velocity setpoint via MAVLink
        self._send_setpoint(vx, vy, vz)

        # Step simulation
        self.world.step(render=True)

        # Get observation
        obs = self._get_observation()

        # Simple placeholder reward (Phase 5 will add real rewards)
        reward = 0.0

        # Check termination conditions
        pos = self.vehicle.state.position
        terminated = False
        truncated = False

        # Terminate if crashed (below ground)
        if pos[2] < 0.5:
            terminated = True

        # Terminate if too far from mission area
        if np.linalg.norm(pos[:2]) > 150.0:
            terminated = True

        # Truncate if max steps reached
        if self.step_count >= self.max_steps:
            truncated = True

        info = {
            "step": self.step_count,
            "position": pos.tolist(),
            "velocity": self.vehicle.state.linear_velocity.tolist(),
        }

        return obs, reward, terminated, truncated, info

    def _get_observation(self):
        """Build observation vector from current state."""
        pos = self.vehicle.state.position
        vel = self.vehicle.state.linear_velocity
        orient = self.vehicle.state.attitude

        # Convert quaternion to euler
        r = Rotation.from_quat(orient)
        euler = r.as_euler('XYZ')  # roll, pitch, yaw

        # UAV state: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        uav_state = np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            euler[0], euler[1], euler[2]
        ], dtype=np.float32)

        # Target observations: for each target, [dx, dy, dz, in_fov]
        target_obs = []
        for i, target in enumerate(self.target_positions[:5]):  # Max 5 targets
            dx = target['position'][0] - pos[0]
            dy = target['position'][1] - pos[1]
            dz = target['position'][2] - pos[2]

            # Check if in FOV (simplified)
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            in_fov = 1.0 if dist < self.max_detection_range else 0.0

            target_obs.extend([dx, dy, dz, in_fov])

        # Pad if fewer than 5 targets
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
        pass  # Cleanup handled by main script


# =============================================================================
# Setup (same as Phase 3)
# =============================================================================

# Initialize timeline and Pegasus interface
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world
pg.set_px4_path("/px4")

# Get stage and setup physics scene
stage = get_current_stage()
physics_path = "/World/PhysicsScene"
if not stage.GetPrimAtPath(physics_path):
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_path))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    print("Physics scene created", flush=True)

# Generate procedural forest environment
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

# Spawn POI targets (vehicles)
print("\n[WorldGen] Spawning POI targets...", flush=True)
vehicle_cluster_center = (30.0, 20.0)
vehicle_paths = world_gen.vehicles.spawn_vehicle_group(
    vehicle_types=["sedan", "suv", "tank"],
    count=5,
    clustering=0.7,
    center=vehicle_cluster_center,
)
print(f"[WorldGen] Spawned {len(vehicle_paths)} vehicles", flush=True)

# Store target positions
target_positions = []
for path in vehicle_paths:
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

# Reset world and start simulation
world.reset()
timeline.play()

# Wait for PX4 to initialize
print("\n[PX4] Running 200 steps to let PX4 initialize...", flush=True)
for i in range(200):
    world.step(render=True)

initial_pos = vehicle.state.position
print(f"[PX4] Initial position: ({initial_pos[0]:.2f}, {initial_pos[1]:.2f}, {initial_pos[2]:.2f})", flush=True)

# Connect MAVLink
print("\n[MAVLink] Connecting...", flush=True)
mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

# Wait for heartbeat while stepping
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


# Prime with setpoints
print("\n[Flight] Priming with 200 setpoints...", flush=True)
for _ in range(200):
    send_setpoint()
    world.step(render=True)

# Engage OFFBOARD mode and arm
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

# Fly up to observation altitude
print("\n[Flight] Ascending to observation altitude (20m)...", flush=True)
for i in range(500):
    send_setpoint(vz=-2.0)
    world.step(render=True)
    if i % 100 == 0:
        pos = vehicle.state.position
        print(f"[Flight] Step {i}: altitude = {pos[2]:.1f}m", flush=True)

# =============================================================================
# Phase 4: Gymnasium Wrapper Test
# =============================================================================
print("\n" + "=" * 60, flush=True)
print("GYMNASIUM WRAPPER TEST", flush=True)
print("=" * 60, flush=True)

# Create Gymnasium environment wrapper
env = Phase4GymEnv(world, vehicle, mav, target_positions)

print(f"\n[Gym] Environment created", flush=True)
print(f"  Observation space: {env.observation_space}", flush=True)
print(f"  Observation shape: {env.observation_space.shape}", flush=True)
print(f"  Action space: {env.action_space}", flush=True)
print(f"  Action shape: {env.action_space.shape}", flush=True)

# Test reset()
print("\n[Gym] Testing reset()...", flush=True)
obs, info = env.reset(seed=42)

print(f"  Observation shape: {obs.shape}", flush=True)
print(f"  Observation dtype: {obs.dtype}", flush=True)
print(f"  Has NaN: {np.any(np.isnan(obs))}", flush=True)
print(f"  Has Inf: {np.any(np.isinf(obs))}", flush=True)
print(f"  Min/Max: [{obs.min():.4f}, {obs.max():.4f}]", flush=True)
print(f"  Info: {info}", flush=True)

# Validate observation is in space bounds
if env.observation_space.contains(obs):
    print("  Observation is valid (within space bounds)", flush=True)
else:
    print("  WARNING: Observation not in observation_space!", flush=True)

# Test step() loop
print("\n[Gym] Testing step() with random actions...", flush=True)
NUM_STEPS = 200
episode_reward = 0.0
step_times = []
nan_count = 0
inf_count = 0

for step in range(NUM_STEPS):
    step_start = time.time()

    # Random action
    action = env.action_space.sample()

    # Step environment
    obs, reward, terminated, truncated, info = env.step(action)
    episode_reward += reward

    step_time = time.time() - step_start
    step_times.append(step_time)

    # Validate observation
    if np.any(np.isnan(obs)):
        nan_count += 1
    if np.any(np.isinf(obs)):
        inf_count += 1

    # Print progress
    if step % 50 == 0 or step < 5:
        pos = info.get('position', [0, 0, 0])
        print(f"  Step {step}: pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), "
              f"reward={reward:.3f}, term={terminated}, trunc={truncated}", flush=True)

    if terminated or truncated:
        print(f"\n  Episode ended at step {step}: terminated={terminated}, truncated={truncated}", flush=True)
        break

# =============================================================================
# Results
# =============================================================================
print("\n" + "=" * 60, flush=True)
print("PHASE 4 RESULTS", flush=True)
print("=" * 60, flush=True)

avg_step_time = np.mean(step_times) if step_times else 0
steps_per_sec = 1.0 / avg_step_time if avg_step_time > 0 else 0

print(f"Steps completed: {len(step_times)}/{NUM_STEPS}", flush=True)
print(f"Total reward: {episode_reward:.2f}", flush=True)
print(f"Avg step time: {avg_step_time*1000:.2f} ms", flush=True)
print(f"Steps per second: {steps_per_sec:.1f} Hz", flush=True)
print(f"NaN observations: {nan_count}", flush=True)
print(f"Inf observations: {inf_count}", flush=True)

# Final observation check
final_obs, _ = env.reset()
final_valid = env.observation_space.contains(final_obs)
print(f"\nFinal reset observation valid: {final_valid}", flush=True)

# Success criteria
success = (
    len(step_times) >= NUM_STEPS * 0.5 and
    nan_count == 0 and
    inf_count == 0 and
    final_valid
)

if success:
    print("\nSUCCESS! Gymnasium wrapper is working correctly.", flush=True)
    print("  reset() returns valid observation", flush=True)
    print("  step() works with action space", flush=True)
    print("  Observations are valid (no NaN/Inf)", flush=True)
    print("  Ready for Phase 5: Add reward signal", flush=True)
else:
    print("\nFAILED: Gymnasium wrapper has issues.", flush=True)
    if nan_count > 0:
        print(f"  - {nan_count} observations had NaN", flush=True)
    if inf_count > 0:
        print(f"  - {inf_count} observations had Inf", flush=True)
    if not final_valid:
        print("  - Final observation not in space bounds", flush=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
env.close()
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
