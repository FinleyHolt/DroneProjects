#!/usr/bin/env python3
"""
F-11 ISR Full Sensor Suite Test

Tests all F-11 sensors in Isaac Sim:
- EO Camera (belly-mounted, 1080p RGB + depth)
- Thermal Camera (simulated, belly-mounted)
- 3-axis Gimbal control
- Navigation sensors (GPS, IMU, Barometer, Magnetometer)

Cameras are belly-mounted at z=-0.15m to avoid body obstruction.

Usage:
    /isaac-sim/python.sh /workspace/scripts/fly_f11_sensor_test.py
    /isaac-sim/python.sh /workspace/scripts/fly_f11_sensor_test.py --gui
    /isaac-sim/python.sh /workspace/scripts/fly_f11_sensor_test.py --gui --duration 120
"""

import argparse
import time
import os

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="Run with GUI")
parser.add_argument("--duration", type=float, default=60.0, help="Test duration in seconds")
parser.add_argument("--output-dir", type=str, default="/tmp/f11_sensor_output", help="Output directory")
args, _ = parser.parse_known_args()

HEADLESS = not args.gui

from isaacsim import SimulationApp

config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 0,
}
simulation_app = SimulationApp(config)

# Add paths
import sys
sys.path.insert(0, "/pegasus/extensions/pegasus.simulator")
sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")
sys.path.insert(0, "/workspace/scripts")

import numpy as np
import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation

# Pegasus imports
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.dynamics import LinearDrag

# WorldGenerator
from flyby.world_generator import WorldGenerator, WorldConfig

# F-11 Sensor Suite
from f11_sensor_suite import F11SensorSuite, SensorConfig


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


class CaptureManager:
    """Manages sensor data capture and storage."""

    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(f"{output_dir}/eo", exist_ok=True)
        os.makedirs(f"{output_dir}/thermal", exist_ok=True)
        os.makedirs(f"{output_dir}/depth", exist_ok=True)

        self.eo_count = 0
        self.thermal_count = 0
        self.depth_count = 0
        self.last_capture_time = 0
        self.capture_interval = 2.0
        self.warmup_frames = 0
        self.warmup_complete = False

    def warmup(self):
        self.warmup_frames += 1
        if self.warmup_frames >= 150:  # 2.5 seconds at 60fps
            self.warmup_complete = True
            print("  [CAPTURE] Warmup complete, sensors ready")

    def should_capture(self, elapsed: float) -> bool:
        if not self.warmup_complete:
            return False
        if elapsed - self.last_capture_time >= self.capture_interval:
            self.last_capture_time = elapsed
            return True
        return False

    def capture_all(self, sensors: F11SensorSuite, elapsed: float, gimbal_state: dict):
        """Capture from all sensors."""
        if not self.warmup_complete:
            return

        # EO Camera (RGB)
        rgb = sensors.get_eo_rgb()
        if rgb is not None and rgb.size > 0 and rgb.max() > 5:
            filename = f"{self.output_dir}/eo/frame_{self.eo_count:04d}_{elapsed:.1f}s.npy"
            np.save(filename, rgb)
            self.eo_count += 1
            print(f"  [EO] Captured frame {self.eo_count}: {rgb.shape}, mean={rgb.mean():.1f}")

        # Depth
        depth = sensors.get_eo_depth()
        if depth is not None and depth.size > 0 and depth.max() > 0:
            filename = f"{self.output_dir}/depth/depth_{self.depth_count:04d}_{elapsed:.1f}s.npy"
            np.save(filename, depth)
            self.depth_count += 1
            print(f"  [DEPTH] Captured: range {depth.min():.1f}-{depth.max():.1f}m")

        # Thermal
        thermal = sensors.get_thermal_image()
        if thermal is not None and thermal.size > 0:
            filename = f"{self.output_dir}/thermal/thermal_{self.thermal_count:04d}_{elapsed:.1f}s.npy"
            np.save(filename, thermal)
            self.thermal_count += 1
            print(f"  [THERMAL] Captured: {thermal.shape}")

        # Log gimbal state
        if gimbal_state:
            print(f"  [GIMBAL] pitch={gimbal_state['pitch']:.1f}°, yaw={gimbal_state['yaw']:.1f}°")


class F11SensorTestApp:
    """F-11 sensor suite test application."""

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
            tree_density=0.3,
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

        # Configure drone (without graphical sensors - we add our own)
        print("\n" + "=" * 60)
        print("Configuring F-11 ISR Drone")
        print("=" * 60)

        config = MultirotorConfig()
        config.thrust_curve = F11ThrustCurve()
        config.drag = LinearDrag(F11_ISR_SPECS["drag_coeff"])

        # Standard navigation sensors
        config.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # No Pegasus graphical sensors - we use F11SensorSuite instead
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
            [0.0, 0.0, 15.0],  # Start at 15m altitude
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )

        self.world.reset()

        # Initialize F-11 sensor suite
        print("\n" + "=" * 60)
        print("Initializing F-11 Sensor Suite")
        print("=" * 60)

        sensor_config = SensorConfig(
            drone_body_path="/World/F11_ISR/body",
            eo_enabled=True,
            eo_resolution=(1920, 1080),
            eo_depth_enabled=True,
            thermal_enabled=True,
            lidar_enabled=False,
            gimbal_mount_position=(0.0, 0.0, -0.15),  # Belly mount
            gimbal_default_pitch=45.0,
        )

        self.sensors = F11SensorSuite(sensor_config)
        self.sensors.initialize(self.world)

        # Initialize capture manager
        self.capture = CaptureManager(args.output_dir)

        self._print_info()
        self.stop_sim = False
        self.start_time = None

    def _print_info(self):
        print("\n" + "=" * 60)
        print("F-11 ISR Sensor Test Ready")
        print("=" * 60)
        print("\nSensor Configuration:")
        status = self.sensors.get_sensor_status()
        print(f"  EO Camera: {'Ready' if status['eo_camera'] else 'Not Available'}")
        print(f"  Thermal Camera: {'Ready' if status['thermal_camera'] else 'Not Available'}")
        print(f"  LiDAR: {'Ready' if status['lidar'] else 'Not Available'}")
        print(f"  Gimbal: {'Ready' if status['gimbal'] else 'Not Available'}")
        print(f"\nMount Position: Belly (z=-0.15m)")
        print(f"Default Pitch: 45° down")
        print(f"Output: {args.output_dir}")
        print("\nGimbal Test Pattern:")
        print("  0-20s: Forward look (0°)")
        print("  20-40s: Nadir survey (-90°)")
        print("  40-60s: Sweep pattern")
        print("\nPX4 SITL: Waiting for heartbeat...")
        print("=" * 60)

    def run(self):
        """Main simulation loop."""
        self.timeline.play()
        self.start_time = time.time()

        mode = "GUI" if not HEADLESS else "HEADLESS"
        print(f"\n[{mode}] Simulation running...")
        print(f"Capturing to: {args.output_dir}")
        print("Warming up sensors (2.5 seconds)...")
        print("Press Ctrl+C to stop.\n")

        frame = 0
        last_log = 0

        try:
            while simulation_app.is_running() and not self.stop_sim:
                self.world.step(render=True)
                frame += 1

                elapsed = time.time() - self.start_time

                # Warmup
                if not self.capture.warmup_complete:
                    self.capture.warmup()
                    continue

                # Gimbal test pattern
                self._run_gimbal_pattern(elapsed)

                # Capture sensor data
                if self.capture.should_capture(elapsed):
                    gimbal_state = None
                    if self.sensors.gimbal:
                        gimbal_state = {
                            "pitch": self.sensors.gimbal.state.pitch,
                            "yaw": self.sensors.gimbal.state.yaw,
                            "roll": self.sensors.gimbal.state.roll,
                        }
                    self.capture.capture_all(self.sensors, elapsed, gimbal_state)

                # Log every 10 seconds
                if elapsed - last_log >= 10.0:
                    last_log = elapsed
                    self._log_status(elapsed, frame)

                # Stop after duration
                if elapsed >= args.duration:
                    print(f"\n[TEST] {args.duration}s test complete.")
                    break

        except KeyboardInterrupt:
            print("\n[STOP] Interrupted")

        self._print_summary()
        self.timeline.stop()
        simulation_app.close()

    def _run_gimbal_pattern(self, elapsed: float):
        """Execute gimbal test pattern based on elapsed time."""
        if self.sensors.gimbal is None:
            return

        if elapsed < 20.0:
            # Forward look
            self.sensors.gimbal.look_forward()
        elif elapsed < 40.0:
            # Nadir survey
            self.sensors.gimbal.nadir_survey()
        else:
            # Sweep pattern
            pitch = -45 + 30 * np.sin(elapsed * 0.3)
            yaw = 30 * np.sin(elapsed * 0.2)
            self.sensors.gimbal.set_angles(pitch=pitch, yaw=yaw)

    def _log_status(self, elapsed: float, frame: int):
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

        # Capture counts
        print(f"  EO frames: {self.capture.eo_count}")
        print(f"  Thermal frames: {self.capture.thermal_count}")
        print(f"  Depth frames: {self.capture.depth_count}")

        # Gimbal state
        if self.sensors.gimbal:
            g = self.sensors.gimbal.state
            print(f"  Gimbal: pitch={g.pitch:.1f}°, yaw={g.yaw:.1f}°, roll={g.roll:.1f}°")

    def _print_summary(self):
        """Print test summary."""
        total = time.time() - self.start_time
        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        print(f"Duration: {total:.1f}s")
        print(f"\nCaptures:")
        print(f"  EO Camera: {self.capture.eo_count} frames")
        print(f"  Thermal Camera: {self.capture.thermal_count} frames")
        print(f"  Depth: {self.capture.depth_count} frames")
        print(f"\nOutput directory: {args.output_dir}")
        print("=" * 60)


def main():
    mode = "GUI" if args.gui else "HEADLESS"
    print("=" * 60)
    print(f"F-11 ISR Sensor Suite Test [{mode}]")
    print("=" * 60)

    app = F11SensorTestApp()
    app.run()


if __name__ == "__main__":
    main()
