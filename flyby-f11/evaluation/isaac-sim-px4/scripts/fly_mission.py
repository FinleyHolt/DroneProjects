#!/usr/bin/env python3
"""
Isaac Sim + PX4 Autonomous Flight Demo
Takes off, flies a square pattern via offboard control, and lands.

Key insights:
- Simulation must step continuously - no time.sleep() in main loop
- PX4 commander CLI works for arm/takeoff/land commands
- MAVLink UDP is for offboard position control
"""

import argparse
import sys
import os
import threading
import subprocess

# Force unbuffered output
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Add Pegasus to Python path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

# Parse args before importing Isaac Sim
parser = argparse.ArgumentParser(description="PX4 Flight Demo")
parser.add_argument("--headless", action="store_true", help="Run headless")
args = parser.parse_args()

# Initialize Isaac Sim
print("=" * 60)
print("Isaac Sim + PX4 Autonomous Flight Demo")
print("=" * 60)

print("\n[1/5] Initializing Isaac Sim...")
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})
print("  Isaac Sim ready")

# Now import the rest
print("\n[2/5] Loading Pegasus extension...")
import omni.timeline
from isaacsim.core.api.world import World

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation
print("  Pegasus loaded")

# Initialize simulation
print("\n[3/5] Setting up world...")
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world

# Load environment
pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
print("  Environment loaded")

# Spawn quadcopter with PX4
print("\n[4/5] Spawning Iris with PX4...")
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

iris = Multirotor(
    "/World/quadrotor",
    ROBOTS["Iris"],
    0,
    [0.0, 0.0, 0.1],
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=iris_config,
)
print("  Iris spawned")

# Reset and start
world.reset()
timeline.play()

# PX4 Commander helper - runs in background thread to avoid blocking
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

# State machine for flight
class FlightState:
    INIT = 0
    WAIT_READY = 1
    ARM = 2
    TAKEOFF = 3
    HOVER = 4
    LAND = 5
    DONE = 6

state = FlightState.INIT
step_count = 0
phase_start_step = 0
command_sent = False

# Flight parameters
ALTITUDE = 3.0

# Timing in steps (60 steps ≈ 1 second at 60Hz physics)
STEPS_INIT = 300           # 5 seconds for PX4 to initialize
STEPS_WAIT_READY = 180     # 3 seconds to confirm ready
STEPS_ARM_WAIT = 120       # 2 seconds after arm command
STEPS_TAKEOFF = 360        # 6 seconds for takeoff
STEPS_HOVER = 600          # 10 seconds hover
STEPS_LAND = 600           # 10 seconds for landing

print("\n[5/5] Starting simulation loop...")
print("=" * 60)
print("Simulation running! Watch the drone:")
print("  1. Wait for PX4 ready")
print("  2. Arm")
print("  3. Takeoff to 2.5m")
print("  4. Hover for 10 seconds")
print("  5. Land")
print("Press Ctrl+C to stop.")
print("=" * 60)

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
                state = FlightState.HOVER
                phase_start_step = step_count
                print(">>> Hovering...")

        elif state == FlightState.HOVER:
            if step_count - phase_start_step >= STEPS_HOVER:
                state = FlightState.LAND
                phase_start_step = step_count
                command_sent = False
                print(">>> Sending land command...")

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
            state_names = ["INIT", "WAIT_READY", "ARM", "TAKEOFF", "HOVER", "LAND", "DONE"]
            print(f"step={step_count:5d} | State: {state_names[state]:10s} | Pos: [{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}]")

except KeyboardInterrupt:
    print("\nStopping simulation...")

# Cleanup
timeline.stop()
simulation_app.close()
print("\nSimulation complete!")
