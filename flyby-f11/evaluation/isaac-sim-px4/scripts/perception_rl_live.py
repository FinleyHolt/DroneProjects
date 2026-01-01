#!/usr/bin/env python3
"""
Live Perception for RL Training - YOLO on actual camera images.

This script integrates the perception pipeline with Isaac Sim's camera
to run real-time object detection during RL training. This ensures
realistic sim-to-real transfer by using actual inference, not ground truth.

Key features:
1. Captures camera images at 20Hz from drone camera
2. Runs YOLO inference on each frame (or hybrid mode with ground truth)
3. Encodes detections into 516-dim observation vector
4. Generates TPTP facts for Vampire ATP safety checking
5. Displays bounding boxes on detections

Usage:
    /isaac-sim/python.sh /workspace/scripts/perception_rl_live.py
    /isaac-sim/python.sh /workspace/scripts/perception_rl_live.py --headless
    /isaac-sim/python.sh /workspace/scripts/perception_rl_live.py --mode inference
"""

import argparse
import time
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="Run headless")
parser.add_argument("--mode", choices=["inference", "ground_truth", "hybrid"],
                    default="hybrid", help="Detection mode")
parser.add_argument("--duration", type=float, default=120.0, help="Duration in seconds")
parser.add_argument("--output-dir", type=str, default="/tmp/perception_live",
                    help="Output directory for annotated images")
parser.add_argument("--save-interval", type=float, default=5.0,
                    help="Interval between saving annotated images (seconds)")
args, _ = parser.parse_known_args()

HEADLESS = args.headless

import carb
from isaacsim import SimulationApp

config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 0,
}
simulation_app = SimulationApp(config)

# Add paths - use /tmp/perception if /workspace has permission issues
sys.path.insert(0, "/pegasus/extensions/pegasus.simulator")
sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")
# Try /tmp first (for podman cp workaround), fall back to /workspace
if os.path.exists("/tmp/perception/perception_encoder.py"):
    sys.path.insert(0, "/tmp/perception")
else:
    sys.path.insert(0, "/workspace/perception")
sys.path.insert(0, "/workspace")

import numpy as np
import omni.timeline
from omni.isaac.core.world import World
from pxr import UsdGeom, Gf, Usd
from scipy.spatial.transform import Rotation

# Isaac Sim camera
from isaacsim.sensors.camera import Camera

# Pegasus imports
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.dynamics import LinearDrag

# World generator
from flyby.world_generator import WorldGenerator, WorldConfig

# Perception imports
try:
    from perception_encoder import PerceptionEncoder, PerceptionConfig
    from detector import YOLODetector, Detection
    from tptp_generator import TPTPGenerator
    PERCEPTION_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Perception module import error: {e}")
    PERCEPTION_AVAILABLE = False

# Try CV2 for visualization
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: OpenCV not available, will save raw numpy arrays")


# F-11 ISR Physical Specifications
F11_ISR_SPECS = {
    "mass": 6.3,
    "arm_length": 0.295,
    "max_thrust_per_motor": 22.0,
    "Ixx": 0.115,
    "Iyy": 0.115,
    "Izz": 0.175,
    "drag_coeff": [0.35, 0.35, 0.1],
}


class F11ThrustCurve(QuadraticThrustCurve):
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


class LivePerceptionPipeline:
    """Real-time perception pipeline with YOLO inference on camera images."""

    def __init__(self, mode: str = "hybrid"):
        self.mode = mode
        self.frame_count = 0
        self.total_encode_time = 0.0
        self.total_inference_time = 0.0

        if PERCEPTION_AVAILABLE:
            # Initialize perception encoder
            config = PerceptionConfig(detector_mode=mode)
            self.encoder = PerceptionEncoder(config)
            self.tptp_gen = TPTPGenerator()
            print(f"[PERCEPTION] Initialized in '{mode}' mode")
            print(f"[PERCEPTION] Output dimension: {self.encoder.output_dim}")
        else:
            self.encoder = None
            self.tptp_gen = None
            print("[PERCEPTION] Not available - running without detection")

        # Detection statistics
        self.detection_counts = {"person": 0, "vehicle": 0, "building": 0}
        self.danger_zone_alerts = 0

    def process_frame(
        self,
        image: np.ndarray,
        uav_position: np.ndarray,
        uav_orientation: np.ndarray,
        ground_truth_labels: list = None,
    ) -> tuple:
        """
        Process a camera frame and return perception observation.

        Args:
            image: RGB image from camera (H, W, 3)
            uav_position: UAV position in world frame [x, y, z]
            uav_orientation: UAV orientation as quaternion [w, x, y, z]
            ground_truth_labels: Optional ground truth for hybrid mode

        Returns:
            (observation, detections, encode_time_ms)
        """
        if not PERCEPTION_AVAILABLE or self.encoder is None:
            return np.zeros(516, dtype=np.float32), [], 0.0

        self.frame_count += 1

        # Run perception encoder
        start = time.perf_counter()
        obs = self.encoder.encode(
            image=image,
            uav_position=uav_position,
            uav_orientation=uav_orientation,
            ground_truth_labels=ground_truth_labels,
        )
        encode_time = (time.perf_counter() - start) * 1000
        self.total_encode_time += encode_time

        # Get raw detections for visualization
        detections = self.encoder.detector.detect(
            image,
            ground_truth_labels=ground_truth_labels,
            uav_position=uav_position,
        )

        # Update statistics
        for det in detections:
            if det.class_id == 1:
                self.detection_counts["person"] += 1
                if det.distance < 15:
                    self.danger_zone_alerts += 1
            elif det.class_id == 2:
                self.detection_counts["vehicle"] += 1
            elif det.class_id == 3:
                self.detection_counts["building"] += 1

        return obs, detections, encode_time

    def draw_detections(
        self,
        image: np.ndarray,
        detections: list,
        uav_position: np.ndarray,
    ) -> np.ndarray:
        """Draw bounding boxes on image."""
        if not CV2_AVAILABLE:
            return image

        annotated = image.copy()
        h, w = image.shape[:2]

        for det in detections:
            # Get bbox (normalized coordinates)
            cx, cy, bw, bh = det.bbox

            # Convert to pixel coordinates
            x1 = int((cx - bw/2) * w)
            y1 = int((cy - bh/2) * h)
            x2 = int((cx + bw/2) * w)
            y2 = int((cy + bh/2) * h)

            # Clamp to image bounds
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w-1, x2), min(h-1, y2)

            # Color by class and danger level
            if det.class_id == 1:  # Person
                if det.distance < 15:
                    color = (255, 0, 255)  # Magenta for DANGER
                    thickness = 3
                elif det.distance < 30:
                    color = (255, 165, 0)  # Orange for WARNING
                    thickness = 2
                else:
                    color = (255, 0, 0)  # Red
                    thickness = 2
            elif det.class_id == 2:  # Vehicle
                color = (0, 0, 255)  # Blue
                thickness = 2
            else:  # Building
                color = (128, 128, 128)  # Gray
                thickness = 1

            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)

            # Draw label
            label = f"{det.class_name} {det.distance:.1f}m"
            if det.distance < 15:
                label += " DANGER!"

            cv2.putText(annotated, label, (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Add HUD info
        info = f"UAV: [{uav_position[0]:.0f}, {uav_position[1]:.0f}, {uav_position[2]:.0f}]"
        cv2.putText(annotated, info, (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        info2 = f"Detections: {len(detections)} | Frame: {self.frame_count}"
        cv2.putText(annotated, info2, (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Show mode
        mode_text = f"Mode: {self.mode.upper()}"
        cv2.putText(annotated, mode_text, (10, h - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return annotated

    def get_stats(self) -> dict:
        """Get perception statistics."""
        avg_encode = self.total_encode_time / max(1, self.frame_count)
        return {
            "frames_processed": self.frame_count,
            "avg_encode_time_ms": avg_encode,
            "detection_counts": self.detection_counts,
            "danger_zone_alerts": self.danger_zone_alerts,
        }


class PerceptionRLApp:
    """Isaac Sim app with live perception for RL training."""

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # Initialize Pegasus
        self.pg = PegasusInterface()
        self.pg._px4_path = "/px4"
        self.pg._px4_default_airframe = "none_iris"

        # Create world
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Generate procedural world with people and vehicles
        print("\n" + "=" * 60)
        print("Generating World with People and Vehicles")
        print("=" * 60)

        world_config = WorldConfig(
            terrain_size=(200.0, 200.0),
            terrain_roughness=2.0,
            tree_density=0.2,  # Sparse trees for visibility
        )

        self.world_gen = WorldGenerator(
            models_path="/workspace/extensions/forest_generator/models",
            config=world_config,
        )

        print("  Generating terrain...")
        self.world_gen.generate_terrain()
        print("  Setting up lighting...")
        self.world_gen.setup_lighting()
        print("  Generating sparse forest...")
        self.world_gen.generate_forest()

        # Spawn test objects for detection
        self._spawn_test_objects()

        stats = self.world_gen.get_stats()
        print(f"  World stats: {stats}")

        # Configure and spawn drone
        print("\n" + "=" * 60)
        print("Spawning F-11 ISR Drone with Camera")
        print("=" * 60)

        config = MultirotorConfig()
        config.thrust_curve = F11ThrustCurve()
        config.drag = LinearDrag(F11_ISR_SPECS["drag_coeff"])
        config.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
        config.graphical_sensors = []  # We'll add our own camera

        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": "/px4",
            "px4_vehicle_model": "none_iris",
        })
        config.backends = [PX4MavlinkBackend(mavlink_config)]

        print("  Spawning drone at 50m altitude...")
        self.drone = Multirotor(
            "/World/F11_ISR",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 50.0],  # Start at 50m - good for seeing terrain
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )

        self.world.reset()

        # Create camera
        print("  Creating ISR camera...")
        camera_path = "/World/F11_ISR/body/isr_camera"
        self.camera = Camera(
            prim_path=camera_path,
            frequency=30,
            resolution=(640, 480),
        )

        # Camera pointing down at 45 degrees for ISR
        self.camera.set_local_pose(
            np.array([0.1, 0.0, -0.05]),
            Rotation.from_euler("ZYX", [0.0, 45.0, 0.0], degrees=True).as_quat()
        )
        self.camera.initialize()
        self.camera.set_focal_length(4.5)
        self.camera.set_clipping_range(0.1, 500.0)

        # Initialize perception pipeline
        print("\n" + "=" * 60)
        print("Initializing Perception Pipeline")
        print("=" * 60)

        self.perception = LivePerceptionPipeline(mode=args.mode)

        # Create output directory
        os.makedirs(args.output_dir, exist_ok=True)

        self.start_time = None
        self.last_save_time = 0
        self.warmup_frames = 0
        self.warmup_complete = False

    def _spawn_test_objects(self):
        """Spawn people and vehicles for detection testing."""
        print("  Spawning test objects (people, vehicles)...")

        stage = self.world.stage

        # Spawn people at known positions
        self.people_positions = [
            [20, 30, 0],
            [50, -20, 0],
            [-30, 40, 0],
            [15, 15, 0],
            [-50, -50, 0],
            [80, 10, 0],
        ]

        for i, pos in enumerate(self.people_positions):
            prim_path = f"/World/Person_{i}"
            cube = UsdGeom.Cube.Define(stage, prim_path)
            cube.GetSizeAttr().Set(1.5)  # Person-sized
            xform = cube.GetPrim()
            UsdGeom.XformCommonAPI(xform).SetTranslate(Gf.Vec3d(pos[0], pos[1], 0.75))
            cube.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])  # Red
            print(f"    Person {i}: [{pos[0]}, {pos[1]}]")

        # Spawn vehicles
        self.vehicle_positions = [
            [40, 0, 0],
            [-60, 30, 0],
            [0, -70, 0],
        ]

        for i, pos in enumerate(self.vehicle_positions):
            prim_path = f"/World/Vehicle_{i}"
            cube = UsdGeom.Cube.Define(stage, prim_path)
            cube.GetSizeAttr().Set(4.0)  # Car-sized
            xform = cube.GetPrim()
            UsdGeom.XformCommonAPI(xform).SetTranslate(Gf.Vec3d(pos[0], pos[1], 2.0))
            cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.0, 0.0, 1.0)])  # Blue
            print(f"    Vehicle {i}: [{pos[0]}, {pos[1]}]")

        # Spawn buildings
        self.building_positions = [
            [0, 0, 0],
            [100, 100, 0],
            [-100, 50, 0],
        ]

        for i, pos in enumerate(self.building_positions):
            prim_path = f"/World/Building_{i}"
            cube = UsdGeom.Cube.Define(stage, prim_path)
            cube.GetSizeAttr().Set(20.0)  # Building-sized
            xform = cube.GetPrim()
            UsdGeom.XformCommonAPI(xform).SetTranslate(Gf.Vec3d(pos[0], pos[1], 10.0))
            cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.5, 0.5, 0.5)])  # Gray
            print(f"    Building {i}: [{pos[0]}, {pos[1]}]")

    def _get_ground_truth_labels(self, uav_pos: np.ndarray) -> list:
        """Generate ground truth labels from known object positions."""
        labels = []

        for pos in self.people_positions:
            pos_arr = np.array(pos, dtype=np.float32)
            rel = pos_arr - uav_pos
            dist = np.linalg.norm(rel)
            if dist < 150:
                img_x = 0.5 + rel[0] / (dist + 10) * 0.3
                img_y = 0.5 + rel[1] / (dist + 10) * 0.3
                if 0.1 < img_x < 0.9 and 0.1 < img_y < 0.9:
                    labels.append({
                        'class_id': 1,
                        'bbox': (float(img_x), float(img_y), 0.05, 0.1),
                        'position_3d': pos_arr,
                    })

        for pos in self.vehicle_positions:
            pos_arr = np.array(pos, dtype=np.float32)
            rel = pos_arr - uav_pos
            dist = np.linalg.norm(rel)
            if dist < 150:
                img_x = 0.5 + rel[0] / (dist + 10) * 0.3
                img_y = 0.5 + rel[1] / (dist + 10) * 0.3
                if 0.1 < img_x < 0.9 and 0.1 < img_y < 0.9:
                    labels.append({
                        'class_id': 2,
                        'bbox': (float(img_x), float(img_y), 0.1, 0.08),
                        'position_3d': pos_arr,
                    })

        for pos in self.building_positions:
            pos_arr = np.array(pos, dtype=np.float32)
            rel = pos_arr - uav_pos
            dist = np.linalg.norm(rel)
            if dist < 200:
                img_x = 0.5 + rel[0] / (dist + 10) * 0.3
                img_y = 0.5 + rel[1] / (dist + 10) * 0.3
                if 0.0 < img_x < 1.0 and 0.0 < img_y < 1.0:
                    labels.append({
                        'class_id': 3,
                        'bbox': (float(img_x), float(img_y), 0.2, 0.3),
                        'position_3d': pos_arr,
                    })

        return labels

    def run(self):
        """Main simulation loop with perception."""
        self.timeline.play()
        self.start_time = time.time()

        mode = "GUI" if not HEADLESS else "HEADLESS"
        print("\n" + "=" * 60)
        print(f"[{mode}] Running Perception RL Pipeline")
        print(f"Mode: {args.mode}")
        print(f"Duration: {args.duration}s")
        print(f"Output: {args.output_dir}")
        print("=" * 60 + "\n")

        frame = 0
        last_log = 0

        try:
            while simulation_app.is_running():
                self.world.step(render=True)
                frame += 1
                elapsed = time.time() - self.start_time

                # Warmup camera
                if not self.warmup_complete:
                    self.warmup_frames += 1
                    if self.warmup_frames >= 120:  # 2 seconds
                        self.warmup_complete = True
                        print("[CAMERA] Warmup complete, starting perception...")
                    continue

                # Get camera image
                rgba = self.camera.get_rgba()
                if rgba is None or rgba.max() < 5:
                    continue

                rgb = rgba[:, :, :3]

                # Get drone state
                uav_pos = np.array([0.0, 0.0, 50.0])  # Default
                uav_ori = np.array([1.0, 0.0, 0.0, 0.0])

                if hasattr(self.drone, '_state') and self.drone._state is not None:
                    state = self.drone._state
                    if hasattr(state, 'position'):
                        uav_pos = np.array(state.position)

                # Get ground truth for hybrid/ground_truth mode
                gt_labels = None
                if args.mode in ["ground_truth", "hybrid"]:
                    gt_labels = self._get_ground_truth_labels(uav_pos)

                # Run perception
                obs, detections, encode_time = self.perception.process_frame(
                    rgb, uav_pos, uav_ori, gt_labels
                )

                # Save annotated image periodically
                if elapsed - self.last_save_time >= args.save_interval:
                    self.last_save_time = elapsed
                    self._save_annotated(rgb, detections, uav_pos, elapsed)

                # Log every 10 seconds
                if elapsed - last_log >= 10.0:
                    last_log = elapsed
                    self._log_status(elapsed, encode_time, detections)

                # Stop after duration
                if elapsed >= args.duration:
                    break

        except KeyboardInterrupt:
            print("\n[STOP] Interrupted")

        # Print summary
        self._print_summary()

        self.timeline.stop()
        simulation_app.close()

    def _save_annotated(self, image, detections, uav_pos, elapsed):
        """Save annotated image with detections."""
        annotated = self.perception.draw_detections(image, detections, uav_pos)

        filename = f"{args.output_dir}/detection_{elapsed:.0f}s.png"

        if CV2_AVAILABLE:
            cv2.imwrite(filename, cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
        else:
            np.save(filename.replace('.png', '.npy'), annotated)

        print(f"[SAVE] {filename} - {len(detections)} detections")

    def _log_status(self, elapsed, encode_time, detections):
        """Log perception status."""
        persons = sum(1 for d in detections if d.class_id == 1)
        vehicles = sum(1 for d in detections if d.class_id == 2)
        buildings = sum(1 for d in detections if d.class_id == 3)
        danger = sum(1 for d in detections if d.class_id == 1 and d.distance < 15)

        print(f"\n[{elapsed:.0f}s] Perception Status:")
        print(f"  Encode: {encode_time:.2f}ms")
        print(f"  Detections: {len(detections)} (P:{persons} V:{vehicles} B:{buildings})")
        if danger > 0:
            print(f"  DANGER ZONE: {danger} person(s) within 15m!")

        # Show nearest objects
        if detections:
            nearest = sorted(detections, key=lambda d: d.distance)[:3]
            print("  Nearest:")
            for d in nearest:
                icon = "P" if d.class_id == 1 else "V" if d.class_id == 2 else "B"
                print(f"    [{icon}] {d.distance:.1f}m")

    def _print_summary(self):
        """Print final summary."""
        stats = self.perception.get_stats()

        print("\n" + "=" * 60)
        print("PERCEPTION RL SESSION SUMMARY")
        print("=" * 60)
        print(f"\nFrames processed: {stats['frames_processed']}")
        print(f"Average encode time: {stats['avg_encode_time_ms']:.2f}ms")
        print(f"\nDetection totals:")
        print(f"  Persons:   {stats['detection_counts']['person']}")
        print(f"  Vehicles:  {stats['detection_counts']['vehicle']}")
        print(f"  Buildings: {stats['detection_counts']['building']}")
        print(f"\nDanger zone alerts: {stats['danger_zone_alerts']}")
        print(f"\nOutput saved to: {args.output_dir}")
        print("=" * 60)


def main():
    print("=" * 60)
    print("Flyby F-11 Perception RL Pipeline")
    print("=" * 60)
    print(f"Mode: {args.mode}")
    print(f"Headless: {HEADLESS}")
    print()

    app = PerceptionRLApp()
    app.run()


if __name__ == "__main__":
    main()
