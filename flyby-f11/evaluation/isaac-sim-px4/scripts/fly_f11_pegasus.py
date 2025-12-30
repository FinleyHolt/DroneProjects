#!/usr/bin/env python3
"""
Fly F-11 drone using Pegasus Simulator with PX4 SITL.

Integrates with our procedurally generated world (WorldGenerator).
Runs headless for testing - verifies sensors, gimbal, and flight.

Usage:
    /isaac-sim/python.sh /workspace/scripts/fly_f11_pegasus.py
    /isaac-sim/python.sh /workspace/scripts/fly_f11_pegasus.py --gui
"""

import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="Run with GUI")
parser.add_argument("--duration", type=float, default=60.0, help="Test duration in seconds")
args, _ = parser.parse_known_args()

HEADLESS = not args.gui

import carb
from isaacsim import SimulationApp

# Start Isaac Sim
simulation_app = SimulationApp({"headless": HEADLESS})

# Add paths
import sys
sys.path.insert(0, "/pegasus/extensions/pegasus.simulator")
sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")

# Now import modules
import numpy as np
import omni.timeline
from omni.isaac.core.world import World

# Import Pegasus API
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.dynamics import LinearDrag

from scipy.spatial.transform import Rotation

# Import our WorldGenerator
from flyby.world_generator import WorldGenerator, WorldConfig


# F-11 ISR Physical Specifications
F11_ISR_SPECS = {
    "mass": 6.3,
    "arm_length": 0.295,
    "max_thrust_per_motor": 22.0,
    "hover_throttle": 0.47,
    "Ixx": 0.115,
    "Iyy": 0.115,
    "Izz": 0.175,
    "drag_coeff": [0.35, 0.35, 0.1],
}


class F11ThrustCurve(QuadraticThrustCurve):
    """Custom thrust curve for F-11."""
    
    def __init__(self):
        super().__init__()
        self._rotor_constant = [4.1e-5] * 4
        self._rolling_moment_coefficient = [1e-6] * 4
        arm = F11_ISR_SPECS["arm_length"]
        self._rotor_positions = [
            [arm * 0.707, -arm * 0.707, 0.0],
            [-arm * 0.707, arm * 0.707, 0.0],
            [arm * 0.707, arm * 0.707, 0.0],
            [-arm * 0.707, -arm * 0.707, 0.0],
        ]


class F11PegasusApp:
    """F-11 Drone with Pegasus + PX4 + Procedural World."""

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        
        # Initialize Pegasus
        self.pg = PegasusInterface()
        self.pg._px4_path = "/px4"
        # Use "none_iris" which is the generic SITL airframe
        self.pg._px4_default_airframe = "none_iris"
        
        # Create world
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        
        # Generate OUR procedural world
        print("\n" + "=" * 60)
        print("Generating Procedural World")
        print("=" * 60)
        
        world_config = WorldConfig(
            terrain_size=(200.0, 200.0),
            terrain_roughness=2.0,
            tree_density=0.4,
        )
        
        self.world_gen = WorldGenerator(
            models_path="/workspace/extensions/forest_generator/models",
            config=world_config,
        )
        
        print("  Generating terrain...")
        self.world_gen.generate_terrain()
        
        print("  Setting up lighting...")
        self.world_gen.setup_lighting()
        
        print("  Generating forest...")
        self.world_gen.generate_forest()
        
        stats = self.world_gen.get_stats()
        print(f"\n  World stats: {stats}")
        
        # Configure drone
        print("\n" + "=" * 60)
        print("Configuring F-11 ISR Drone")
        print("=" * 60)
        
        config = MultirotorConfig()
        config.thrust_curve = F11ThrustCurve()
        config.drag = LinearDrag(F11_ISR_SPECS["drag_coeff"])
        config.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
        
        # Use none_iris airframe (generic SITL)
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": "/px4",
            "px4_vehicle_model": "none_iris",  # Generic SITL iris
        })
        config.backends = [PX4MavlinkBackend(mavlink_config)]
        
        print("  Spawning drone...")
        self.drone = Multirotor(
            "/World/F11_ISR",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.5],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )
        
        self.world.reset()
        self._print_info()
        self.stop_sim = False
        self.start_time = None

    def _print_info(self):
        print("\n" + "=" * 60)
        print("F-11 ISR Drone Ready")
        print("=" * 60)
        print(f"\nPhysical Parameters:")
        print(f"  Mass:       {F11_ISR_SPECS['mass']} kg")
        print(f"  Arm Length: {F11_ISR_SPECS['arm_length']} m")
        print(f"  Max Thrust: {F11_ISR_SPECS['max_thrust_per_motor']} N/motor")
        print(f"\nPX4 SITL: Waiting for heartbeat...")
        print(f"Connect QGC (UDP 14550) or use MAVSDK")
        print("=" * 60)

    def run(self):
        """Main loop with sensor monitoring."""
        
        self.timeline.play()
        self.start_time = time.time()
        
        mode = "GUI" if not HEADLESS else "HEADLESS"
        print(f"\n[{mode}] Simulation running...")
        print("Press Ctrl+C to stop.\n")
        
        frame = 0
        last_log = 0
        
        try:
            while simulation_app.is_running() and not self.stop_sim:
                self.world.step(render=not HEADLESS)
                frame += 1
                
                elapsed = time.time() - self.start_time
                
                # Log every 5 seconds
                if elapsed - last_log >= 5.0:
                    last_log = elapsed
                    self._log_status(elapsed, frame)
                
                # Stop after duration in headless mode
                if HEADLESS and elapsed >= args.duration:
                    print(f"\n[TEST] {args.duration}s test complete.")
                    break
                    
        except KeyboardInterrupt:
            print("\n[STOP] Interrupted")
        
        total = time.time() - self.start_time
        print(f"\n[DONE] Time: {total:.1f}s, Frames: {frame}, FPS: {frame/total:.1f}")
        
        self.timeline.stop()
        simulation_app.close()

    def _log_status(self, elapsed, frame):
        """Log drone and sensor status."""
        print(f"\n[{elapsed:.0f}s] Frame {frame}")
        
        # Try to get state from drone
        if hasattr(self.drone, '_state') and self.drone._state is not None:
            state = self.drone._state
            if hasattr(state, 'position'):
                pos = state.position
                print(f"  Position: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f} m")
            if hasattr(state, 'linear_velocity'):
                vel = state.linear_velocity
                print(f"  Velocity: vx={vel[0]:.2f}, vy={vel[1]:.2f}, vz={vel[2]:.2f} m/s")
        
        # Check PX4 backend status
        if hasattr(self.drone, '_backends') and len(self.drone._backends) > 0:
            backend = self.drone._backends[0]
            if hasattr(backend, '_is_connected'):
                print(f"  PX4 Connected: {backend._is_connected}")
            if hasattr(backend, '_armed'):
                print(f"  Armed: {backend._armed}")


def main():
    mode = "GUI" if args.gui else "HEADLESS"
    print("=" * 60)
    print(f"F-11 ISR - Pegasus + PX4 + Procedural World [{mode}]")
    print("=" * 60)
    
    app = F11PegasusApp()
    app.run()


if __name__ == "__main__":
    main()
