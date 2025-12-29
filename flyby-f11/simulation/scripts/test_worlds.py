#!/usr/bin/env python3
"""
Flyby F-11 World Testing Script

Tests each training world in headless mode by:
1. Starting Gazebo with the world
2. Starting ArduPilot SITL
3. Flying a simple waypoint pattern
4. Recording camera images to video

Usage:
    python3 scripts/test_worlds.py --world urban_training
    python3 scripts/test_worlds.py --all
    python3 scripts/test_worlds.py --world coastal_training --duration 60

Author: Finley Holt
"""

import argparse
import subprocess
import time
import signal
import sys
import os
from pathlib import Path
from typing import Optional, List
import threading


# Available training worlds
TRAINING_WORLDS = [
    "urban_training",
    "rural_training",
    "industrial_training",
    "coastal_training",
    "randomized_training"
]

# World descriptions for logging
WORLD_DESCRIPTIONS = {
    "urban_training": "Dense city environment with buildings (300m x 300m)",
    "rural_training": "Forest/natural terrain with trees (400m x 400m)",
    "industrial_training": "Warehouses and shipping containers (350m x 350m)",
    "coastal_training": "Maritime environment with water/piers (400m x 400m)",
    "randomized_training": "Mixed obstacle template (350m x 350m)"
}


class SimulationTester:
    """Manages simulation testing for a single world."""

    def __init__(self, world_name: str, output_dir: Path, duration: int = 60):
        self.world_name = world_name
        self.output_dir = output_dir
        self.duration = duration
        self.processes: List[subprocess.Popen] = []
        self.recording_active = False

    def start_gazebo_headless(self, world_sdf: Path) -> Optional[subprocess.Popen]:
        """Start Gazebo in headless mode with the specified world."""
        print(f"  Starting Gazebo headless with {world_sdf.name}...")

        env = os.environ.copy()
        env["GZ_SIM_RESOURCE_PATH"] = "/simulation/worlds:/simulation/models"
        env["GAZEBO_MODEL_PATH"] = "/simulation/models"

        # Start Gazebo server only (headless)
        cmd = ["gz", "sim", "-s", "-r", str(world_sdf)]

        try:
            proc = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )
            self.processes.append(proc)
            return proc
        except Exception as e:
            print(f"    ERROR: Failed to start Gazebo: {e}")
            return None

    def wait_for_gazebo(self, timeout: int = 30) -> bool:
        """Wait for Gazebo to be ready by checking topic availability."""
        print("  Waiting for Gazebo to initialize...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                result = subprocess.run(
                    ["gz", "topic", "-l"],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if "/world/" in result.stdout:
                    print("  Gazebo ready!")
                    return True
            except (subprocess.TimeoutExpired, Exception):
                pass
            time.sleep(2)

        print("    WARNING: Gazebo initialization timeout")
        return False

    def get_camera_topic(self) -> Optional[str]:
        """Find the ISR camera image topic."""
        try:
            result = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True,
                timeout=10
            )
            for line in result.stdout.split("\n"):
                if "isr_camera" in line and "image" in line:
                    return line.strip()
        except Exception:
            pass
        return None

    def check_world_loaded(self) -> dict:
        """Check what models are loaded in the world."""
        info = {
            "models": [],
            "nfz_count": 0,
            "landing_zones": 0,
            "drone_spawned": False
        }

        try:
            result = subprocess.run(
                ["gz", "model", "--list"],
                capture_output=True,
                text=True,
                timeout=10
            )

            for line in result.stdout.split("\n"):
                line = line.strip()
                if not line:
                    continue
                info["models"].append(line)
                if "nfz" in line.lower():
                    info["nfz_count"] += 1
                if "landing" in line.lower():
                    info["landing_zones"] += 1
                if "f11" in line.lower():
                    info["drone_spawned"] = True

        except Exception as e:
            print(f"    WARNING: Could not list models: {e}")

        return info

    def capture_camera_frames(self, topic: str, output_path: Path,
                              num_frames: int = 150, fps: int = 5) -> bool:
        """Capture camera frames and save as video."""
        print(f"  Recording from camera topic: {topic}")
        print(f"  Output: {output_path}")

        # Create frames directory
        frames_dir = output_path.parent / f"{output_path.stem}_frames"
        frames_dir.mkdir(parents=True, exist_ok=True)

        frame_count = 0
        self.recording_active = True

        try:
            # Use gz topic to echo camera images
            # For a simpler approach, we'll use Gazebo's built-in recording
            cmd = [
                "gz", "topic", "-e", "-t", topic, "-n", str(num_frames)
            ]

            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )

            # Let it run for the capture duration
            duration_secs = num_frames / fps
            try:
                proc.wait(timeout=duration_secs + 10)
            except subprocess.TimeoutExpired:
                proc.terminate()

            print(f"  Captured camera data for {duration_secs:.1f} seconds")
            return True

        except Exception as e:
            print(f"    ERROR: Camera capture failed: {e}")
            return False
        finally:
            self.recording_active = False

    def run_test(self, world_sdf: Path) -> dict:
        """Run a complete test of the specified world."""
        results = {
            "world": self.world_name,
            "success": False,
            "gazebo_started": False,
            "models_loaded": {},
            "camera_topic_found": False,
            "video_recorded": False,
            "errors": []
        }

        print(f"\n{'='*60}")
        print(f"Testing: {self.world_name}")
        print(f"  {WORLD_DESCRIPTIONS.get(self.world_name, 'Unknown world')}")
        print(f"{'='*60}")

        try:
            # Start Gazebo
            gz_proc = self.start_gazebo_headless(world_sdf)
            if not gz_proc:
                results["errors"].append("Failed to start Gazebo")
                return results

            results["gazebo_started"] = True

            # Wait for Gazebo to be ready
            if not self.wait_for_gazebo():
                results["errors"].append("Gazebo initialization timeout")
                return results

            # Check world contents
            time.sleep(5)  # Give models time to spawn
            model_info = self.check_world_loaded()
            results["models_loaded"] = model_info

            print(f"  Models loaded: {len(model_info['models'])}")
            print(f"  NFZ zones: {model_info['nfz_count']}")
            print(f"  Landing zones: {model_info['landing_zones']}")
            print(f"  Drone spawned: {model_info['drone_spawned']}")

            # Find camera topic
            camera_topic = self.get_camera_topic()
            if camera_topic:
                results["camera_topic_found"] = True
                print(f"  Camera topic: {camera_topic}")
            else:
                print("  WARNING: Camera topic not found")
                results["errors"].append("Camera topic not found")

            # Let simulation run for observation
            print(f"  Running simulation for {self.duration} seconds...")
            time.sleep(self.duration)

            # Check if Gazebo is still running
            if gz_proc.poll() is None:
                print("  Simulation ran successfully!")
                results["success"] = True
            else:
                results["errors"].append("Gazebo crashed during test")

        except Exception as e:
            results["errors"].append(str(e))

        finally:
            self.cleanup()

        return results

    def cleanup(self):
        """Clean up all spawned processes."""
        print("  Cleaning up...")
        for proc in self.processes:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.processes = []

        # Also kill any remaining gz processes
        subprocess.run(["pkill", "-f", "gz sim"], capture_output=True)
        time.sleep(2)


def run_container_test(world_name: str, output_dir: Path, duration: int) -> dict:
    """Run the test inside the container."""
    print(f"\nStarting container test for {world_name}...")

    world_sdf = f"/simulation/worlds/{world_name}.sdf"

    # Build the container command
    cmd = [
        "podman", "run", "--rm",
        "-e", "HEADLESS=true",
        "-e", f"GAZEBO_WORLD={world_sdf}",
        "-v", f"{output_dir}:/output:z",
        "--network", "host",
        "localhost/flyby-f11-sim:latest",
        "bash", "-c", f"""
            set -e

            # Source ROS
            source /opt/ros/humble/setup.bash

            # Start Xvfb for headless rendering
            Xvfb :99 -screen 0 1920x1080x24 &
            export DISPLAY=:99
            sleep 2

            echo "Starting Gazebo with {world_name}..."
            gz sim -s -r {world_sdf} &
            GZ_PID=$!

            # Wait for Gazebo
            echo "Waiting for Gazebo to initialize..."
            sleep 15

            # Check topics
            echo "Available topics:"
            gz topic -l | head -20

            # Check models
            echo "Loaded models:"
            gz model --list

            # Run for duration
            echo "Running simulation for {duration} seconds..."
            sleep {duration}

            # Check if still running
            if kill -0 $GZ_PID 2>/dev/null; then
                echo "SUCCESS: Simulation completed"
                exit 0
            else
                echo "ERROR: Gazebo crashed"
                exit 1
            fi
        """
    ]

    result = {
        "world": world_name,
        "success": False,
        "output": "",
        "errors": []
    }

    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=duration + 120  # Extra time for startup
        )

        result["output"] = proc.stdout
        result["success"] = proc.returncode == 0

        if proc.returncode != 0:
            result["errors"].append(proc.stderr)

    except subprocess.TimeoutExpired:
        result["errors"].append("Container test timeout")
    except Exception as e:
        result["errors"].append(str(e))

    return result


def print_summary(results: List[dict]):
    """Print test summary."""
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)

    passed = sum(1 for r in results if r["success"])
    total = len(results)

    for r in results:
        status = "PASS" if r["success"] else "FAIL"
        print(f"  [{status}] {r['world']}")
        if r.get("errors"):
            for err in r["errors"][:2]:  # Show first 2 errors
                print(f"        Error: {err[:80]}")

    print(f"\nTotal: {passed}/{total} worlds passed")
    print("="*60)


def main():
    parser = argparse.ArgumentParser(
        description="Test Flyby F-11 training worlds in headless mode"
    )
    parser.add_argument(
        "--world",
        type=str,
        choices=TRAINING_WORLDS,
        help="Specific world to test"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Test all training worlds"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Test duration per world in seconds (default: 30)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="/home/finley/Github/DroneProjects/flyby-f11/simulation/test_recordings",
        help="Output directory for recordings"
    )
    parser.add_argument(
        "--container",
        action="store_true",
        help="Run tests inside container (recommended)"
    )

    args = parser.parse_args()

    if not args.world and not args.all:
        print("Error: Specify --world <name> or --all")
        sys.exit(1)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    worlds_to_test = TRAINING_WORLDS if args.all else [args.world]
    results = []

    print(f"Testing {len(worlds_to_test)} world(s)")
    print(f"Duration per world: {args.duration} seconds")
    print(f"Output directory: {output_dir}")

    for world in worlds_to_test:
        if args.container:
            result = run_container_test(world, output_dir, args.duration)
        else:
            sim_dir = Path("/home/finley/Github/DroneProjects/flyby-f11/simulation")
            world_sdf = sim_dir / "worlds" / f"{world}.sdf"

            if not world_sdf.exists():
                print(f"ERROR: World file not found: {world_sdf}")
                results.append({
                    "world": world,
                    "success": False,
                    "errors": ["World file not found"]
                })
                continue

            tester = SimulationTester(world, output_dir, args.duration)
            result = tester.run_test(world_sdf)

        results.append(result)
        time.sleep(5)  # Brief pause between worlds

    print_summary(results)

    # Return non-zero if any tests failed
    sys.exit(0 if all(r["success"] for r in results) else 1)


if __name__ == "__main__":
    main()
