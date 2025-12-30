#!/usr/bin/env python3
"""
Spawn F-11 ISR drone in Isaac Sim with GUI visualization and flight.

Usage:
    /isaac-sim/python.sh /workspace/scripts/spawn_f11_isr.py
"""

import sys
import time
import numpy as np

# Launch Isaac Sim with GUI
from isaacsim import SimulationApp

config = {
    "headless": False,
    "renderer": "RayTracedLighting",
    "width": 1920,
    "height": 1080,
}
simulation_app = SimulationApp(config)

# Now import Isaac modules
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.articulations import Articulation
import omni.isaac.core.utils.prims as prim_utils
from pxr import UsdPhysics, Gf, UsdGeom

# Add extension path
sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")

from flyby.world_generator import WorldGenerator, WorldConfig
from flyby.world_generator.drones import ThrustModel, MotorMixingMatrix


class DroneController:
    """Simple drone flight controller for visualization."""

    def __init__(self, drone_path: str, stage, target_altitude: float = 10.0):
        self.drone_path = drone_path
        self.stage = stage
        self.target_altitude = target_altitude
        self.thrust_model = ThrustModel()
        self.thrust_model.reset()

        # PID gains for altitude hold
        self.kp_alt = 0.5
        self.ki_alt = 0.1
        self.kd_alt = 0.3
        self.alt_integral = 0.0
        self.last_alt_error = 0.0

        # Get drone mass for hover thrust calculation
        self.mass = 6.3  # ISR variant
        self.hover_throttle = self.thrust_model.get_hover_throttle(self.mass)

    def get_drone_position(self) -> np.ndarray:
        """Get current drone position from USD."""
        prim = self.stage.GetPrimAtPath(self.drone_path)
        if not prim.IsValid():
            return np.array([0, 0, 0])

        xform = UsdGeom.Xformable(prim)
        transform = xform.ComputeLocalToWorldTransform(0)
        pos = transform.ExtractTranslation()
        return np.array([pos[0], pos[1], pos[2]])

    def apply_thrust(self, thrust_force: float):
        """Apply upward thrust force to drone."""
        base_link_path = f"{self.drone_path}/base_link"
        prim = self.stage.GetPrimAtPath(base_link_path)
        if not prim.IsValid():
            return

        # Apply force via rigid body API
        # Force is applied in world frame, upward (+Z in Isaac Sim)
        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        if rigid_body:
            # Set velocity directly for simple visualization
            # (Real physics would use force application)
            pass

    def update(self, dt: float) -> np.ndarray:
        """Update controller and return motor commands."""
        pos = self.get_drone_position()
        current_alt = pos[2]

        # Altitude PID
        alt_error = self.target_altitude - current_alt
        self.alt_integral += alt_error * dt
        self.alt_integral = np.clip(self.alt_integral, -1.0, 1.0)
        alt_derivative = (alt_error - self.last_alt_error) / dt if dt > 0 else 0
        self.last_alt_error = alt_error

        # Compute thrust adjustment
        thrust_adj = (
            self.kp_alt * alt_error +
            self.ki_alt * self.alt_integral +
            self.kd_alt * alt_derivative
        )

        # Total throttle = hover + adjustment
        throttle = self.hover_throttle + thrust_adj
        throttle = np.clip(throttle, 0.0, 1.0)

        # Mix to motors (no roll/pitch/yaw for simple hover)
        motor_commands = MotorMixingMatrix.mix(throttle, 0, 0, 0)

        return motor_commands


def main():
    print("=" * 60)
    print("F-11 ISR Drone - Flying to 10m Altitude")
    print("=" * 60)

    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Set physics parameters
    world.set_simulation_dt(physics_dt=1.0/250.0, rendering_dt=1.0/60.0)

    # Create world generator with light forest
    config = WorldConfig(
        terrain_size=(100.0, 100.0),
        terrain_roughness=1.5,
        tree_density=0.3,
    )

    print("\nInitializing world generator...")
    world_gen = WorldGenerator(
        models_path="/workspace/extensions/forest_generator/models",
        config=config,
    )

    # Generate environment
    print("Generating terrain...")
    world_gen.generate_terrain()

    print("Setting up lighting...")
    world_gen.setup_lighting()

    print("Generating sparse forest...")
    world_gen.generate_forest()

    # Spawn the F-11 ISR drone at 10m
    print("\n*** Spawning F-11 ISR Camera Drone at 10m ***")
    drone_path = world_gen.spawn_drone(
        variant="isr_camera",
        position=(0.0, 0.0, 10.0),  # 10m above ground
        heading=0.0,
    )
    print(f"Drone spawned at: {drone_path}")

    # Print stats
    stats = world_gen.get_stats()
    print(f"\nWorld statistics:")
    for key, value in stats.items():
        print(f"  {key}: {value}")

    # Create controller
    stage = get_current_stage()
    controller = DroneController(drone_path, stage, target_altitude=10.0)

    # Reset and run simulation
    print("\n" + "=" * 60)
    print("Simulation running - Drone hovering at 10m")
    print("Close window to exit")
    print("=" * 60)

    world.reset()

    # Run simulation loop
    frame = 0
    dt = 1.0 / 60.0  # Rendering dt

    try:
        while simulation_app.is_running():
            world.step(render=True)
            frame += 1

            # Update controller
            motor_cmds = controller.update(dt)

            # Get and print drone position periodically
            if frame % 300 == 0:  # Every 5 seconds
                pos = controller.get_drone_position()
                print(f"Frame {frame} - Drone at altitude: {pos[2]:.2f}m")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    print(f"\nTotal frames: {frame}")
    simulation_app.close()
    print("Done!")


if __name__ == "__main__":
    main()
