#!/usr/bin/env python3
"""
Fly drone with live perception detection visualization.

This script:
1. Spawns a world with people and vehicles at known positions
2. Spawns the F-11 drone with camera
3. Takes off and lets you fly manually via PX4 SITL
4. Shows live detections from the perception pipeline
5. Compares detected positions to ground truth

Run in Isaac Sim container:
    /isaac-sim/python.sh /workspace/scripts/fly_with_perception.py

Controls:
    - Use QGroundControl or MAVSDK to control the drone
    - Or use the Pegasus keyboard interface if enabled
"""
import sys
import time
import numpy as np
import asyncio

# Add paths
sys.path.insert(0, '/workspace/perception')
sys.path.insert(0, '/workspace')

# Isaac Sim imports
import carb
from isaacsim import SimulationApp

# Launch Isaac Sim headless=False for visualization
simulation_app = SimulationApp({"headless": False, "anti_aliasing": 0})

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Gf, Sdf

# Pegasus imports
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Perception imports
from perception.perception_encoder import PerceptionEncoder, PerceptionConfig
from perception.detector import Detection
from perception.tptp_generator import TPTPGenerator


class PerceptionTestWorld:
    """World with known objects for perception testing."""

    def __init__(self):
        self.world = None
        self.drone = None
        self.pegasus_interface = None

        # Ground truth object positions
        self.people_positions = [
            [20, 30, 0],
            [50, -20, 0],
            [-30, 40, 0],
            [15, 15, 0],
            [-50, -50, 0],
            [80, 10, 0],
        ]

        self.vehicle_positions = [
            [40, 0, 0],
            [-60, 30, 0],
            [0, -70, 0],
        ]

        self.building_positions = [
            [0, 0, 0],
            [100, 100, 0],
            [-100, 50, 0],
        ]

        # Perception
        self.encoder = PerceptionEncoder(PerceptionConfig(detector_mode="ground_truth"))
        self.tptp_gen = TPTPGenerator()

        # Stats
        self.frame_count = 0
        self.detection_log = []

    async def setup(self):
        """Set up the world with objects and drone."""
        print("\n" + "="*60)
        print("Setting up Perception Test World")
        print("="*60 + "\n")

        # Create world
        self.world = World(stage_units_in_meters=1.0)
        await self.world.initialize_simulation_context_async()

        # Add ground plane
        self.world.scene.add_ground_plane()

        # Spawn people (using simple cubes for now - replace with person models)
        print("Spawning people...")
        for i, pos in enumerate(self.people_positions):
            prim_path = f"/World/Person_{i}"
            # Create a simple red cube to represent a person
            cube = UsdGeom.Cube.Define(self.world.stage, prim_path)
            cube.GetSizeAttr().Set(1.0)
            xform = XFormPrim(prim_path)
            xform.set_world_pose(position=np.array([pos[0], pos[1], 0.5]))
            # Set red color
            cube.GetDisplayColorAttr().Set([(1.0, 0.0, 0.0)])
            print(f"  Person {i}: [{pos[0]}, {pos[1]}, {pos[2]}]")

        # Spawn vehicles (blue boxes)
        print("\nSpawning vehicles...")
        for i, pos in enumerate(self.vehicle_positions):
            prim_path = f"/World/Vehicle_{i}"
            cube = UsdGeom.Cube.Define(self.world.stage, prim_path)
            cube.GetSizeAttr().Set(3.0)
            xform = XFormPrim(prim_path)
            xform.set_world_pose(position=np.array([pos[0], pos[1], 1.5]))
            cube.GetDisplayColorAttr().Set([(0.0, 0.0, 1.0)])
            print(f"  Vehicle {i}: [{pos[0]}, {pos[1]}, {pos[2]}]")

        # Spawn buildings (gray boxes)
        print("\nSpawning buildings...")
        for i, pos in enumerate(self.building_positions):
            prim_path = f"/World/Building_{i}"
            cube = UsdGeom.Cube.Define(self.world.stage, prim_path)
            cube.GetSizeAttr().Set(20.0)
            xform = XFormPrim(prim_path)
            xform.set_world_pose(position=np.array([pos[0], pos[1], 10.0]))
            cube.GetDisplayColorAttr().Set([(0.5, 0.5, 0.5)])
            print(f"  Building {i}: [{pos[0]}, {pos[1]}, {pos[2]}]")

        # Set up Pegasus interface
        self.pegasus_interface = PegasusInterface()

        # Spawn drone
        print("\nSpawning F-11 drone...")
        drone_config = MultirotorConfig()
        drone_config.backends = [
            MavlinkBackend(
                vehicle_id=1,
                connection_type="tcpin",
                connection_ip="localhost",
                connection_port=4560
            )
        ]

        self.drone = Multirotor(
            "/World/F11_Drone",
            ROBOTS["Iris"],  # Using Iris model
            init_pos=np.array([0.0, 0.0, 0.5]),
            init_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            config=drone_config,
        )

        self.world.scene.add(self.drone)

        # Reset world
        await self.world.reset_async()

        print("\n" + "="*60)
        print("World ready! Drone spawned at origin.")
        print("="*60)
        print("\nTo fly the drone:")
        print("  1. Start PX4 SITL: make px4_sitl none_iris")
        print("  2. Connect QGroundControl to localhost:14550")
        print("  3. Arm and takeoff!")
        print("\nPerception will show detections as you fly.")
        print("="*60 + "\n")

        return self.world

    def get_ground_truth_labels(self, drone_pos: np.ndarray) -> list:
        """Generate ground truth labels based on drone position and FOV."""
        labels = []

        # Check people
        for i, pos in enumerate(self.people_positions):
            rel_pos = np.array(pos) - drone_pos
            dist = np.linalg.norm(rel_pos)

            if dist < 150:  # Detection range
                # Simple projection to image coords
                if drone_pos[2] > 5:  # Only if drone is high enough
                    img_x = 0.5 + rel_pos[0] / (dist + 1) * 0.3
                    img_y = 0.5 + rel_pos[1] / (dist + 1) * 0.3

                    if 0.05 < img_x < 0.95 and 0.05 < img_y < 0.95:
                        labels.append({
                            'class_id': 1,  # person
                            'semantic_id': 1,
                            'bbox': (img_x, img_y, 0.05, 0.1),
                            'position_3d': np.array(pos, dtype=np.float32),
                        })

        # Check vehicles
        for i, pos in enumerate(self.vehicle_positions):
            rel_pos = np.array(pos) - drone_pos
            dist = np.linalg.norm(rel_pos)

            if dist < 150:
                if drone_pos[2] > 5:
                    img_x = 0.5 + rel_pos[0] / (dist + 1) * 0.3
                    img_y = 0.5 + rel_pos[1] / (dist + 1) * 0.3

                    if 0.05 < img_x < 0.95 and 0.05 < img_y < 0.95:
                        labels.append({
                            'class_id': 2,  # vehicle
                            'semantic_id': 2,
                            'bbox': (img_x, img_y, 0.1, 0.08),
                            'position_3d': np.array(pos, dtype=np.float32),
                        })

        # Check buildings
        for i, pos in enumerate(self.building_positions):
            rel_pos = np.array(pos) - drone_pos
            dist = np.linalg.norm(rel_pos)

            if dist < 200:
                img_x = 0.5 + rel_pos[0] / (dist + 1) * 0.3
                img_y = 0.5 + rel_pos[1] / (dist + 1) * 0.3

                if 0.0 < img_x < 1.0 and 0.0 < img_y < 1.0:
                    labels.append({
                        'class_id': 3,  # building
                        'semantic_id': 3,
                        'bbox': (img_x, img_y, 0.2, 0.3),
                        'position_3d': np.array(pos, dtype=np.float32),
                    })

        return labels

    def process_perception(self, drone_pos: np.ndarray, drone_ori: np.ndarray):
        """Run perception pipeline and display results."""
        self.frame_count += 1

        # Get ground truth labels
        labels = self.get_ground_truth_labels(drone_pos)

        # Dummy image (in real use, get from camera)
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Run perception encoder
        start = time.perf_counter()
        obs = self.encoder.encode(
            image=image,
            uav_position=drone_pos,
            uav_orientation=drone_ori,
            ground_truth_labels=labels,
        )
        encode_time = (time.perf_counter() - start) * 1000

        # Get raw detections
        detections = self.encoder.detector.detect(
            image, ground_truth_labels=labels, uav_position=drone_pos
        )

        # Print every 20 frames
        if self.frame_count % 20 == 0:
            self._print_detection_summary(drone_pos, detections, encode_time, obs)

        return obs, detections

    def _print_detection_summary(self, drone_pos, detections, encode_time, obs):
        """Print detection summary."""
        persons = [d for d in detections if d.class_id == 1]
        vehicles = [d for d in detections if d.class_id == 2]
        buildings = [d for d in detections if d.class_id == 3]

        print(f"\n{'='*60}")
        print(f"Frame {self.frame_count} | Encode: {encode_time:.2f}ms")
        print(f"Drone: [{drone_pos[0]:.1f}, {drone_pos[1]:.1f}, {drone_pos[2]:.1f}]")
        print(f"{'='*60}")

        print(f"\nDetections: {len(detections)} total")
        print(f"  👤 Persons:   {len(persons)}")
        print(f"  🚗 Vehicles:  {len(vehicles)}")
        print(f"  🏢 Buildings: {len(buildings)}")

        # Show closest objects
        print("\nNearest objects:")
        for det in sorted(detections, key=lambda d: d.distance)[:5]:
            icon = "👤" if det.class_id == 1 else "🚗" if det.class_id == 2 else "🏢"
            danger = " ⚠️ DANGER!" if det.class_id == 1 and det.distance < 15 else ""
            print(f"  {icon} {det.class_name:10} @ {det.distance:6.1f}m{danger}")

        # TPTP facts
        facts = self.tptp_gen.generate_facts(detections)
        danger_facts = [f for f in facts if 'Danger' in f]
        if danger_facts:
            print(f"\n⚠️  VAMPIRE SAFETY ALERT: {len(danger_facts)} danger zone violation(s)")

        # Grid coverage
        grid_obs = obs[100:484]
        grid_3d = grid_obs.reshape(8, 8, 6)
        cells_with_persons = np.sum(grid_3d[:, :, 0] > 0)
        cells_with_obstacles = np.sum(grid_3d[:, :, 1] > 0)
        print(f"\nSpatial Grid: {cells_with_persons} person cells, {cells_with_obstacles} obstacle cells")

    async def run(self):
        """Main simulation loop."""
        print("\nStarting simulation loop...")
        print("Fly the drone and watch detections!\n")

        while simulation_app.is_running():
            # Step simulation
            self.world.step(render=True)

            # Get drone state
            if self.drone is not None:
                try:
                    pos, ori = self.drone.get_world_pose()
                    if pos is not None:
                        # Run perception
                        self.process_perception(pos, ori)
                except Exception as e:
                    pass  # Drone not ready yet

            await asyncio.sleep(0.05)  # 20Hz

        print("\nSimulation ended.")


async def main():
    """Main entry point."""
    print("\n" + "#"*60)
    print("Flyby F-11 Perception Test")
    print("#"*60 + "\n")

    test_world = PerceptionTestWorld()
    await test_world.setup()
    await test_world.run()

    simulation_app.close()


if __name__ == "__main__":
    asyncio.run(main())
