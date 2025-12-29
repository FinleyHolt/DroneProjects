#!/usr/bin/env python3
"""
Test ISR Training Environment with GUI

Simple test script to visually inspect the training environments.
Spawns the drone and demonstrates basic flight in one of the canonical environments.

Run with GUI:
  podman exec px4-sim /isaac-sim/python.sh /workspace/scripts/test_isr_env.py

Run headless:
  podman exec px4-sim /isaac-sim/python.sh /workspace/scripts/test_isr_env.py --headless
"""

import argparse
import sys
import os
import time
import numpy as np

# Force unbuffered output
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Add Pegasus to Python path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

# Parse args before importing Isaac Sim (some options affect initialization)
parser = argparse.ArgumentParser(description="Test ISR Training Environment")
parser.add_argument("--headless", action="store_true", help="Run headless (no GUI)")
parser.add_argument(
    "--env",
    type=str,
    choices=["comms_denied", "dynamic_nfz", "multi_objective", "basic"],
    default="basic",
    help="Environment to test"
)
parser.add_argument("--duration", type=int, default=60, help="Test duration in seconds")
args = parser.parse_args()

# Initialize Isaac Sim
print("=" * 60)
print("Flyby F-11 ISR Environment Test")
print("=" * 60)

print(f"\nEnvironment: {args.env}")
print(f"Headless: {args.headless}")
print(f"Duration: {args.duration}s")

print("\n[1/6] Initializing Isaac Sim...")
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "width": 1920,
    "height": 1080,
    "anti_aliasing": 0,  # Faster rendering
})
print("  Isaac Sim ready")

# Now import the rest
print("\n[2/6] Loading Pegasus extension...")
import omni.timeline
from isaacsim.core.api.world import World

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation
print("  Pegasus loaded")

# Initialize simulation
print("\n[3/6] Setting up world...")
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world

# Set coordinates (Camp Pendleton - MCTSSA)
pg.set_global_coordinates(33.3853, -117.5653, 100.0)

# Load environment based on selection
ENV_MAP = {
    "basic": "Curved Gridroom",
    "comms_denied": "Full Warehouse",  # Large area for surveillance
    "dynamic_nfz": "Warehouse with Forklifts",  # Obstacles for path planning
    "multi_objective": "Full Warehouse",  # Complex environment for multi-target
}

env_name = ENV_MAP.get(args.env, "Full Warehouse")
pg.load_environment(SIMULATION_ENVIRONMENTS[env_name])
print(f"  Loaded: {env_name}")

print("  Environment loaded")

# Spawn quadcopter with PX4
print("\n[4/6] Spawning Iris with PX4...")
print(f"  PX4 path: {pg.px4_path}")
print(f"  PX4 airframe: {pg.px4_default_airframe}")

px4_config = PX4MavlinkBackendConfig({
    "vehicle_id": 0,
    "px4_autolaunch": True,
    "px4_dir": pg.px4_path,
    "px4_vehicle_model": pg.px4_default_airframe,
})

iris_config = MultirotorConfig()
iris_config.backends = [PX4MavlinkBackend(px4_config)]

# Spawn position
spawn_pos = [0.0, 0.0, 0.1]
spawn_rot = Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat()

iris = Multirotor(
    "/World/flyby_f11",  # Using F-11 name even with Iris model
    ROBOTS["Iris"],
    0,
    spawn_pos,
    spawn_rot,
    config=iris_config,
)
print("  Vehicle spawned")

# No artificial markers - threat zones, NFZs, and targets are inferred
# from ontology reasoning based on intel, not hardcoded visuals
print("\n[5/6] Environment ready (no artificial markers)")
print("  Threat zones and NFZs will be inferred from ontology during training")

# Reset and start
world.reset()
timeline.play()

# Flight control - same as fly_mission.py
import threading
import subprocess

PX4_CMD = "/px4/build/px4_sitl_default/bin/px4-commander"


def px4_cmd_async(cmd_args, callback=None):
    """Execute PX4 commander command in background thread."""
    def run():
        try:
            result = subprocess.run(
                [PX4_CMD] + cmd_args.split(),
                capture_output=True,
                text=True,
                timeout=10,
                cwd="/px4"
            )
            if callback:
                callback(result.returncode == 0)
        except Exception as e:
            print(f"  PX4 cmd error: {e}")
            if callback:
                callback(False)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()


# State machine
class FlightState:
    INIT = 0
    WAIT_READY = 1
    ARM = 2
    TAKEOFF = 3
    SURVEY = 4
    LAND = 5
    DONE = 6


state = FlightState.INIT
step_count = 0
phase_start_step = 0
command_sent = False

# Timing in steps (60 steps ≈ 1 second at 60Hz)
STEPS_INIT = 300           # 5 seconds for PX4 to initialize
STEPS_WAIT_READY = 180     # 3 seconds to confirm ready
STEPS_ARM_WAIT = 120       # 2 seconds after arm command
STEPS_TAKEOFF = 480        # 8 seconds for takeoff to 10m
STEPS_SURVEY = args.duration * 60  # Survey duration
STEPS_LAND = 600           # 10 seconds for landing

print("\n[6/6] Starting simulation loop...")
print("=" * 60)
print("ISR Environment Test Running")
print()
print("Environment Features:")
print("  - Red cylinder: No-Fly Zone (prohibited)")
print("  - Orange cylinder: Threat Zone (time-limited)")
print("  - Green spheres: Targets of Interest")
print("  - White disk: Landing Zone")
print()
print("Flight Plan:")
print("  1. Initialize and wait for PX4")
print("  2. Arm vehicle")
print("  3. Takeoff to 10m altitude")
print("  4. Survey pattern (simulated)")
print("  5. Land")
print()
print("Press Ctrl+C to stop early.")
print("=" * 60)

# Survey waypoints (simple box pattern)
survey_waypoints = [
    (30.0, 30.0, 10.0),
    (30.0, -30.0, 10.0),
    (-30.0, -30.0, 10.0),
    (-30.0, 30.0, 10.0),
]
current_waypoint = 0

try:
    while simulation_app.is_running():
        world.step(render=True)
        step_count += 1

        # State machine
        if state == FlightState.INIT:
            if step_count >= STEPS_INIT:
                state = FlightState.WAIT_READY
                phase_start_step = step_count
                print("\n>>> Waiting for PX4 ready...")

        elif state == FlightState.WAIT_READY:
            if step_count - phase_start_step >= STEPS_WAIT_READY:
                state = FlightState.ARM
                phase_start_step = step_count
                command_sent = False
                print(">>> Sending arm command...")

        elif state == FlightState.ARM:
            if not command_sent:
                px4_cmd_async("arm")
                command_sent = True
            if step_count - phase_start_step >= STEPS_ARM_WAIT:
                state = FlightState.TAKEOFF
                phase_start_step = step_count
                command_sent = False
                print(">>> Sending takeoff command...")

        elif state == FlightState.TAKEOFF:
            if not command_sent:
                px4_cmd_async("takeoff")
                command_sent = True
            if step_count - phase_start_step >= STEPS_TAKEOFF:
                state = FlightState.SURVEY
                phase_start_step = step_count
                print(">>> Beginning survey pattern...")

        elif state == FlightState.SURVEY:
            # Simple hover for now (would use offboard mode for actual waypoints)
            if step_count - phase_start_step >= STEPS_SURVEY:
                state = FlightState.LAND
                phase_start_step = step_count
                command_sent = False
                print(">>> Survey complete, landing...")

        elif state == FlightState.LAND:
            if not command_sent:
                px4_cmd_async("land")
                command_sent = True
            if step_count - phase_start_step >= STEPS_LAND:
                state = FlightState.DONE
                print(">>> Flight complete!")
                print(">>> Simulation will continue. Press Ctrl+C to exit.")

        # Log position every 2 seconds
        if step_count % 120 == 0:
            pos = iris.state.position
            state_names = ["INIT", "WAIT_READY", "ARM", "TAKEOFF", "SURVEY", "LAND", "DONE"]
            t = step_count / 60.0
            print(f"t={t:6.1f}s | State: {state_names[state]:10s} | Pos: [{pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:7.2f}]")

except KeyboardInterrupt:
    print("\n\nStopping simulation...")

# Cleanup
timeline.stop()
simulation_app.close()
print("\nTest complete!")
