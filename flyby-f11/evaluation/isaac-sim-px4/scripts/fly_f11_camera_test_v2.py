#!/usr/bin/env python3
"""
F-11 drone with camera capture and gimbal control test.

Tests:
1. Camera image capture (save frames to disk)
2. Gimbal manipulation (pitch/yaw control)
3. Sensor data publishing verification

Uses Isaac Sim's Camera API properly for headless rendering.

Usage:
    /isaac-sim/python.sh /workspace/scripts/fly_f11_camera_test_v2.py
    /isaac-sim/python.sh /workspace/scripts/fly_f11_camera_test_v2.py --gui
"""

import argparse
import time
import os

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="Run with GUI")
parser.add_argument("--duration", type=float, default=60.0, help="Test duration")
parser.add_argument("--output-dir", type=str, default="/tmp/f11_camera_output", help="Output directory for images")
args, _ = parser.parse_known_args()

HEADLESS = not args.gui

import carb
from isaacsim import SimulationApp

# Enable rendering in headless mode
config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",  # Required for camera rendering
    "anti_aliasing": 0,
}
simulation_app = SimulationApp(config)

# Add paths
import sys
sys.path.insert(0, "/pegasus/extensions/pegasus.simulator")
sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")

import numpy as np
import omni.timeline
from omni.isaac.core.world import World
from pxr import UsdGeom, Gf, Usd

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

from scipy.spatial.transform import Rotation

# Our WorldGenerator
from flyby.world_generator import WorldGenerator, WorldConfig


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


class GimbalController:
    """Simple gimbal controller for testing with actual camera rotation."""

    def __init__(self, camera: Camera, initial_orientation: np.ndarray):
        self.camera = camera
        self.base_orientation = initial_orientation.copy()
        self.pitch = 0.0  # -90 to +30 degrees
        self.yaw = 0.0    # -180 to +180 degrees

    def set_gimbal_angles(self, pitch_deg: float, yaw_deg: float):
        """Set gimbal pitch and yaw angles by rotating the camera."""
        self.pitch = np.clip(pitch_deg, -90, 30)
        self.yaw = np.clip(yaw_deg, -180, 180)

        # Compute new orientation
        # Base orientation + gimbal rotation
        base_rot = Rotation.from_euler("ZYX", self.base_orientation, degrees=True)
        gimbal_rot = Rotation.from_euler("YZ", [self.pitch, self.yaw], degrees=True)
        combined = base_rot * gimbal_rot

        # Apply to camera (local pose)
        try:
            pos, _ = self.camera.get_local_pose()
            self.camera.set_local_pose(pos, combined.as_quat())
        except Exception as e:
            pass  # Camera may not be ready yet

    def sweep_test(self, elapsed: float) -> tuple:
        """Generate a sweep pattern for gimbal testing."""
        # Pitch oscillates between -45 and 0
        pitch = -22.5 + 22.5 * np.sin(elapsed * 0.5)
        # Yaw oscillates between -30 and 30
        yaw = 30 * np.sin(elapsed * 0.3)
        return pitch, yaw


class CameraCapture:
    """Camera frame capture utility with proper headless support."""

    def __init__(self, camera: Camera, output_dir: str):
        self.camera = camera
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.frame_count = 0
        self.last_capture_time = 0
        self.capture_interval = 2.0  # Capture every 2 seconds
        self._warmup_frames = 0
        self._warmup_complete = False

    def warmup(self):
        """Call during warmup period to let camera initialize."""
        self._warmup_frames += 1
        if self._warmup_frames >= 120:  # 2 seconds at 60fps
            self._warmup_complete = True
            print("  [CAMERA] Warmup complete, ready for capture")

    def should_capture(self, elapsed: float) -> bool:
        if not self._warmup_complete:
            return False
        if elapsed - self.last_capture_time >= self.capture_interval:
            self.last_capture_time = elapsed
            return True
        return False

    def capture_frame(self, elapsed: float) -> bool:
        """Capture and save a camera frame."""
        if not self._warmup_complete:
            return False

        try:
            # Get RGBA image from camera
            rgba = self.camera.get_rgba()
            if rgba is None:
                print("  [CAMERA] No RGBA data available yet")
                return False

            # Check if image has content (not all zeros)
            if rgba.max() < 5:
                print(f"  [CAMERA] Image too dark (max={rgba.max()}), waiting...")
                return False

            rgb = rgba[:, :, :3]

            # Save as numpy file
            filename = f"{self.output_dir}/frame_{self.frame_count:04d}_{elapsed:.1f}s.npy"
            np.save(filename, rgb)

            self.frame_count += 1
            print(f"  [CAMERA] Captured frame {self.frame_count}: {rgb.shape}, max={rgb.max()} -> {filename}")

            # Also try to get depth if available
            try:
                depth = self.camera.get_depth()
                if depth is not None and depth.max() > 0:
                    depth_file = f"{self.output_dir}/depth_{self.frame_count:04d}_{elapsed:.1f}s.npy"
                    np.save(depth_file, depth)
                    print(f"  [CAMERA] Captured depth: {depth.shape}, range={depth.min():.2f}-{depth.max():.2f}m")
            except Exception as e:
                pass

            return True
        except Exception as e:
            print(f"  [CAMERA] Capture error: {e}")
            return False


class F11CameraTestApp:
    """F-11 with camera capture and gimbal test."""

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # Initialize Pegasus
        self.pg = PegasusInterface()
        self.pg._px4_path = "/px4"
        self.pg._px4_default_airframe = "none_iris"

        # Create world
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Generate procedural world
        print("\n" + "=" * 60)
        print("Generating Procedural World")
        print("=" * 60)

        world_config = WorldConfig(
            terrain_size=(200.0, 200.0),
            terrain_roughness=2.0,
            tree_density=0.3,  # Sparse for visibility
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
        print(f"  World stats: {stats}")

        # Configure drone (without Pegasus camera - we'll add our own)
        print("\n" + "=" * 60)
        print("Configuring F-11 ISR with Camera")
        print("=" * 60)

        config = MultirotorConfig()
        config.thrust_curve = F11ThrustCurve()
        config.drag = LinearDrag(F11_ISR_SPECS["drag_coeff"])

        # Standard sensors
        config.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # Don't use Pegasus graphical sensors - we'll add our own camera
        config.graphical_sensors = []

        # PX4 backend
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": "/px4",
            "px4_vehicle_model": "none_iris",
        })
        config.backends = [PX4MavlinkBackend(mavlink_config)]

        print("  Spawning drone...")
        self.drone = Multirotor(
            "/World/F11_ISR",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 10.0],  # Start at 10m altitude
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )

        self.world.reset()

        # Create camera directly attached to drone body
        print("  Creating ISR camera...")
        camera_path = "/World/F11_ISR/body/isr_camera"
        self.camera = Camera(
            prim_path=camera_path,
            frequency=30,
            resolution=(640, 480),
        )

        # Set camera position: forward and slightly below, pitched down 45 degrees
        camera_orientation = np.array([0.0, 45.0, 0.0])  # Pitch down
        self.camera.set_local_pose(
            np.array([0.1, 0.0, -0.05]),  # 10cm forward, 5cm down
            Rotation.from_euler("ZYX", camera_orientation, degrees=True).as_quat()
        )

        # Initialize camera
        self.camera.initialize()

        # Set camera properties
        self.camera.set_focal_length(4.5)  # Wide angle
        self.camera.set_clipping_range(0.1, 500.0)

        # Add depth capability
        self.camera.add_distance_to_image_plane_to_frame()

        # Initialize camera capture and gimbal controller
        self.camera_capture = CameraCapture(self.camera, args.output_dir)
        self.gimbal = GimbalController(self.camera, camera_orientation)

        self._print_info()
        self.stop_sim = False
        self.start_time = None

    def _print_info(self):
        print("\n" + "=" * 60)
        print("F-11 ISR Camera Test Ready")
        print("=" * 60)
        print(f"\nCamera Configuration:")
        print(f"  Resolution: 640x480")
        print(f"  FOV: ~90° (4.5mm focal length)")
        print(f"  Depth: Enabled")
        print(f"  Output: {args.output_dir}")
        print(f"\nGimbal Test:")
        print(f"  Sweep pattern: pitch -45°..0°, yaw -30°..+30°")
        print(f"\nPX4 SITL: Waiting for heartbeat...")
        print("=" * 60)

    def run(self):
        """Main loop with camera capture and gimbal testing."""

        self.timeline.play()
        self.start_time = time.time()

        mode = "GUI" if not HEADLESS else "HEADLESS"
        print(f"\n[{mode}] Simulation running with camera test...")
        print(f"Capturing frames to: {args.output_dir}")
        print("Warming up camera (2 seconds)...")
        print("Press Ctrl+C to stop.\n")

        frame = 0
        last_log = 0

        try:
            while simulation_app.is_running() and not self.stop_sim:
                self.world.step(render=True)  # Always render for camera
                frame += 1

                elapsed = time.time() - self.start_time

                # Warmup the camera
                if not self.camera_capture._warmup_complete:
                    self.camera_capture.warmup()
                    continue

                # Gimbal sweep test
                if frame % 60 == 0:  # Every second at 60fps
                    pitch, yaw = self.gimbal.sweep_test(elapsed)
                    self.gimbal.set_gimbal_angles(pitch, yaw)

                # Try to capture camera frames
                if self.camera_capture.should_capture(elapsed):
                    self.camera_capture.capture_frame(elapsed)

                # Log every 5 seconds
                if elapsed - last_log >= 5.0:
                    last_log = elapsed
                    self._log_status(elapsed, frame)

                # Stop after duration
                if elapsed >= args.duration:
                    print(f"\n[TEST] {args.duration}s test complete.")
                    break

        except KeyboardInterrupt:
            print("\n[STOP] Interrupted")

        total = time.time() - self.start_time
        print(f"\n[DONE] Time: {total:.1f}s, Frames: {frame}")
        print(f"Camera captures: {self.camera_capture.frame_count}")
        print(f"Output directory: {args.output_dir}")

        self.timeline.stop()
        simulation_app.close()

    def _log_status(self, elapsed, frame):
        """Log drone and sensor status."""
        print(f"\n[{elapsed:.0f}s] Frame {frame}")

        # Drone state
        if hasattr(self.drone, '_state') and self.drone._state is not None:
            state = self.drone._state
            if hasattr(state, 'position'):
                pos = state.position
                print(f"  Position: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f} m")

        # PX4 status
        if hasattr(self.drone, '_backends') and len(self.drone._backends) > 0:
            backend = self.drone._backends[0]
            if hasattr(backend, '_is_connected'):
                print(f"  PX4 Connected: {backend._is_connected}")

        # Camera status
        print(f"  Camera frames captured: {self.camera_capture.frame_count}")

        # Gimbal status
        print(f"  Gimbal: pitch={self.gimbal.pitch:.1f}°, yaw={self.gimbal.yaw:.1f}°")


def main():
    mode = "GUI" if args.gui else "HEADLESS"
    print("=" * 60)
    print(f"F-11 ISR Camera Test v2 [{mode}]")
    print("=" * 60)

    app = F11CameraTestApp()
    app.run()


if __name__ == "__main__":
    main()
