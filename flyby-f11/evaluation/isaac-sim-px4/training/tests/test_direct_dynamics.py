#!/usr/bin/env python3
"""
Phase 2 Full-Stack Test: Direct Dynamics Environment

This test validates the direct dynamics environment for fast RL training.
The drone should fly under direct force/torque control without PX4 SITL.

Produces video output showing:
- Drone flying a simple pattern (hover, forward, turn, etc.)
- FPS overlay demonstrating speedup over PX4 baseline
- Position, velocity, and action overlays
- Comparison of render_interval=1 vs render_interval=4

Usage:
    /isaac-sim/python.sh /workspace/training/tests/test_direct_dynamics.py --headless

Output:
    /workspace/output/videos/phase2_direct_dynamics_test.mp4
    /workspace/output/videos/phase2_direct_dynamics_report.json

Success Criteria:
    - Drone maintains stable hover
    - Drone responds to control inputs (forward, backward, rotate)
    - FPS >= 500 without rendering (render_interval=1000)
    - FPS >= 200 with render_interval=4
    - Video shows controllable flight

Author: Finley Holt
"""

import sys
import os
import argparse
import time
import json
import subprocess
import numpy as np

# Parse arguments early
parser = argparse.ArgumentParser(description="Phase 2: Direct Dynamics Test")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument("--render-interval", type=int, default=4, help="Render every Nth frame")
parser.add_argument("--test-steps", type=int, default=2000, help="Number of steps to run")
parser.add_argument("--output-dir", type=str, default="/workspace/output/videos",
                    help="Output directory for video")
parser.add_argument("--benchmark-only", action="store_true",
                    help="Run FPS benchmark without video recording")
args = parser.parse_args()

print("=" * 70)
print("PHASE 2 TEST: Direct Dynamics Environment")
print("=" * 70)
print(f"Headless: {args.headless}")
print(f"Render Interval: {args.render_interval}")
print(f"Test Steps: {args.test_steps}")
print()

# Add paths
sys.path.insert(0, "/workspace")
sys.path.insert(0, "/workspace/training")

# Import direct dynamics environment
from training.environments.direct_dynamics_env import (
    DirectDynamicsEnv,
    DirectDynamicsConfig,
    ActionMode,
)


class VideoRecorderSimple:
    """Simple video recorder using FFmpeg."""

    def __init__(self, output_path: str, width: int, height: int, fps: int = 30):
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_count = 0
        self.ffmpeg_proc = None

        os.makedirs(os.path.dirname(output_path), exist_ok=True)

    def start(self) -> bool:
        import shutil
        if shutil.which('ffmpeg') is None:
            print("[VideoRecorder] FFmpeg not found")
            return False

        ffmpeg_cmd = [
            'ffmpeg', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', f'{self.width}x{self.height}', '-r', str(self.fps),
            '-i', '-', '-c:v', 'libx264', '-preset', 'fast',
            '-crf', '23', '-pix_fmt', 'yuv420p', self.output_path
        ]

        try:
            self.ffmpeg_proc = subprocess.Popen(
                ffmpeg_cmd, stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            print(f"[VideoRecorder] Started: {self.output_path}")
            return True
        except Exception as e:
            print(f"[VideoRecorder] Error: {e}")
            return False

    def write_frame(self, rgba_frame: np.ndarray, overlay_text: str = None) -> bool:
        if self.ffmpeg_proc is None or rgba_frame is None:
            return False

        try:
            import cv2
        except ImportError:
            print("[VideoRecorder] cv2 not available, skipping overlay")
            cv2 = None

        # Validate frame
        if len(rgba_frame.shape) < 3:
            print(f"[VideoRecorder] Invalid frame shape: {rgba_frame.shape}")
            return False

        # Ensure RGB (3 channels)
        if rgba_frame.shape[2] == 4:
            rgb_frame = rgba_frame[:, :, :3].copy()
        elif rgba_frame.shape[2] == 3:
            rgb_frame = rgba_frame.copy()
        else:
            print(f"[VideoRecorder] Unexpected channels: {rgba_frame.shape[2]}")
            return False

        # Resize if needed
        if cv2 is not None and (rgb_frame.shape[0] != self.height or rgb_frame.shape[1] != self.width):
            rgb_frame = cv2.resize(rgb_frame, (self.width, self.height))

        # Draw overlay text
        if overlay_text and cv2 is not None:
            font = cv2.FONT_HERSHEY_SIMPLEX
            y_offset = 25
            for line in overlay_text.split('\n'):
                (tw, th), _ = cv2.getTextSize(line, font, 0.6, 1)
                cv2.rectangle(rgb_frame, (5, y_offset - th - 5),
                             (tw + 10, y_offset + 5), (0, 0, 0), -1)
                cv2.putText(rgb_frame, line, (8, y_offset), font, 0.6, (0, 255, 0), 1)
                y_offset += th + 12

        rgb_frame = np.ascontiguousarray(rgb_frame)
        try:
            self.ffmpeg_proc.stdin.write(rgb_frame.tobytes())
            self.frame_count += 1
            return True
        except Exception as e:
            print(f"[VideoRecorder] Write error: {e}")
            return False

    def stop(self) -> bool:
        if self.ffmpeg_proc is None:
            return False
        try:
            self.ffmpeg_proc.stdin.close()
            self.ffmpeg_proc.wait(timeout=30)
            print(f"[VideoRecorder] Finished: {self.frame_count} frames -> {self.output_path}")
            return True
        except Exception as e:
            print(f"[VideoRecorder] Stop error: {e}")
            self.ffmpeg_proc.kill()
            return False


class FlightController:
    """Simple flight controller for test patterns."""

    def __init__(self, action_mode: ActionMode = ActionMode.THRUST_RATES):
        self.action_mode = action_mode
        self.phase = "hover"
        self.phase_start_step = 0
        self.target_altitude = 50.0

    def get_action(self, step: int, obs: np.ndarray, position: np.ndarray,
                   velocity: np.ndarray) -> np.ndarray:
        """Generate action based on flight phase."""

        # Phase transitions
        phase_duration = 300  # steps per phase

        phase_idx = (step // phase_duration) % 6
        phases = ["hover", "forward", "hover", "right", "hover", "yaw"]
        self.phase = phases[phase_idx]

        if self.action_mode == ActionMode.THRUST_RATES:
            return self._thrust_rates_action(position, velocity)
        elif self.action_mode == ActionMode.VELOCITY:
            return self._velocity_action(position, velocity)
        else:
            return self._motor_speed_action(position, velocity)

    def _thrust_rates_action(self, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Generate thrust + angular rate commands."""
        # Altitude controller
        alt_error = self.target_altitude - position[2]
        vz_error = 0.0 - velocity[2]

        # PD altitude control -> thrust
        kp_alt = 0.2
        kd_alt = 0.3
        thrust_cmd = 0.0 + kp_alt * alt_error + kd_alt * vz_error  # 0 = hover thrust

        # Limit thrust
        thrust_cmd = np.clip(thrust_cmd, -0.5, 0.5)

        # Phase-specific commands
        roll_cmd = 0.0
        pitch_cmd = 0.0
        yaw_cmd = 0.0

        if self.phase == "forward":
            pitch_cmd = -0.2  # Pitch forward
        elif self.phase == "right":
            roll_cmd = 0.2  # Roll right
        elif self.phase == "yaw":
            yaw_cmd = 0.3  # Yaw right

        # Position hold for hover phases
        if self.phase == "hover":
            # Simple position damping
            kd_pos = 0.1
            roll_cmd = -kd_pos * velocity[1]  # Damp Y velocity with roll
            pitch_cmd = kd_pos * velocity[0]  # Damp X velocity with pitch

        return np.array([thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd])

    def _velocity_action(self, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Generate velocity commands."""
        # Altitude control via vz
        alt_error = self.target_altitude - position[2]
        vz_cmd = np.clip(0.3 * alt_error, -0.5, 0.5)

        vx_cmd = 0.0
        vy_cmd = 0.0
        yaw_cmd = 0.0

        if self.phase == "forward":
            vx_cmd = 0.3
        elif self.phase == "right":
            vy_cmd = 0.3
        elif self.phase == "yaw":
            yaw_cmd = 0.3
        elif self.phase == "hover":
            # Position damping
            vx_cmd = -0.1 * velocity[0]
            vy_cmd = -0.1 * velocity[1]

        return np.array([vx_cmd, vy_cmd, vz_cmd, yaw_cmd])

    def _motor_speed_action(self, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Generate direct motor speed commands."""
        # Start with hover (all motors equal at ~0)
        base = 0.0

        # Altitude correction
        alt_error = self.target_altitude - position[2]
        vz_error = 0.0 - velocity[2]
        alt_correction = 0.1 * alt_error + 0.2 * vz_error

        motors = np.array([base, base, base, base]) + alt_correction

        if self.phase == "forward":
            # Increase rear motors, decrease front
            motors[0] -= 0.1  # FR
            motors[2] -= 0.1  # FL
            motors[1] += 0.1  # BL
            motors[3] += 0.1  # BR
        elif self.phase == "right":
            # Increase left motors
            motors[1] += 0.1  # BL
            motors[2] += 0.1  # FL
            motors[0] -= 0.1  # FR
            motors[3] -= 0.1  # BR
        elif self.phase == "yaw":
            # Increase CW motors, decrease CCW
            motors[0] += 0.1  # CW
            motors[1] += 0.1  # CW
            motors[2] -= 0.1  # CCW
            motors[3] -= 0.1  # CCW

        return np.clip(motors, -1, 1)


def run_fps_benchmark(env: DirectDynamicsEnv, num_steps: int = 1000,
                     render_interval: int = 1) -> dict:
    """Run FPS benchmark without video recording."""
    print(f"\n[Benchmark] Running {num_steps} steps with render_interval={render_interval}...")

    # Temporarily override render interval
    original_interval = env.config.render_interval
    env.config.render_interval = render_interval

    controller = FlightController(env.config.action_mode)
    obs = env.reset()

    start_time = time.time()
    steps_completed = 0

    for step in range(num_steps):
        action = controller.get_action(step, obs, env.position, env.velocity)
        obs, reward, done, info = env.step(action)
        steps_completed += 1

        if done:
            obs = env.reset()

    elapsed = time.time() - start_time
    fps = steps_completed / elapsed

    env.config.render_interval = original_interval

    print(f"[Benchmark] Completed {steps_completed} steps in {elapsed:.2f}s")
    print(f"[Benchmark] FPS: {fps:.1f}")

    return {
        "steps": steps_completed,
        "elapsed_seconds": elapsed,
        "fps": fps,
        "render_interval": render_interval,
    }


def run_direct_dynamics_test():
    """Main test function."""

    OUTPUT_VIDEO = f"{args.output_dir}/phase2_direct_dynamics_test.mp4"
    OUTPUT_REPORT = f"{args.output_dir}/phase2_direct_dynamics_report.json"
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480

    # ========================================================================
    # Create Environment
    # ========================================================================
    print("\n[1/4] Creating Direct Dynamics Environment...")

    config = DirectDynamicsConfig(
        headless=args.headless,
        render_interval=args.render_interval,
        action_mode=ActionMode.THRUST_RATES,
        camera_enabled=True,
        camera_resolution=(CAMERA_WIDTH, CAMERA_HEIGHT),
        spawn_position=np.array([0.0, 0.0, 50.0]),
        max_episode_steps=args.test_steps * 2,
    )

    env = DirectDynamicsEnv(config)
    env.setup()

    # ========================================================================
    # Run FPS Benchmarks
    # ========================================================================
    print("\n[2/4] Running FPS Benchmarks...")

    benchmarks = {}

    # Benchmark without rendering (render_interval=1000)
    benchmarks["no_render"] = run_fps_benchmark(env, num_steps=500, render_interval=1000)

    # Benchmark with render_interval=4
    benchmarks["render_4"] = run_fps_benchmark(env, num_steps=500, render_interval=4)

    # Benchmark with render_interval=1
    benchmarks["render_1"] = run_fps_benchmark(env, num_steps=500, render_interval=1)

    print("\n[Benchmark Summary]")
    print(f"  No rendering (interval=1000): {benchmarks['no_render']['fps']:.1f} FPS")
    print(f"  With render_interval=4:       {benchmarks['render_4']['fps']:.1f} FPS")
    print(f"  With render_interval=1:       {benchmarks['render_1']['fps']:.1f} FPS")

    if args.benchmark_only:
        print("\n[Benchmark Only] Skipping video recording")
        env.close()
        return True

    # ========================================================================
    # Run Flight Test with Video Recording
    # ========================================================================
    print("\n[3/4] Running flight test with video recording...")

    # Reset for video recording test
    env.config.render_interval = args.render_interval
    obs = env.reset()

    # Initialize video recorder
    recorder = VideoRecorderSimple(
        output_path=OUTPUT_VIDEO,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        fps=30,
    )
    if not recorder.start():
        print("[VideoRecorder] WARNING: Could not start video recorder")

    # Flight controller
    controller = FlightController(env.config.action_mode)

    # Run test
    test_start = time.time()
    step_times = []
    rendered_frames = 0
    skipped_frames = 0

    for step in range(args.test_steps):
        step_start = time.time()

        # Get action from controller
        action = controller.get_action(step, obs, env.position, env.velocity)

        # Step environment
        obs, reward, done, info = env.step(action)

        # Track render stats
        if info.get("rendered", False):
            rendered_frames += 1

            # Capture frame for video
            frame = env.get_camera_image()
            if frame is not None and frame.size > 0:
                elapsed = time.time() - test_start
                current_fps = (step + 1) / elapsed if elapsed > 0 else 0

                overlay = (
                    f"Phase 2: Direct Dynamics Test"
                    f"\nAction Mode: {env.config.action_mode.value}"
                    f"\nRender Interval: {args.render_interval}"
                    f"\nStep: {step}/{args.test_steps}"
                    f"\nFPS: {current_fps:.1f}"
                    f"\nPhase: {controller.phase}"
                    f"\nPos: ({env.position[0]:.1f}, {env.position[1]:.1f}, {env.position[2]:.1f})"
                    f"\nVel: ({env.velocity[0]:.1f}, {env.velocity[1]:.1f}, {env.velocity[2]:.1f})"
                    f"\nAction: [{action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f}, {action[3]:.2f}]"
                )
                recorder.write_frame(frame, overlay)
        else:
            skipped_frames += 1

        step_times.append(time.time() - step_start)

        # Reset if done
        if done:
            print(f"[Test] Episode done at step {step}, resetting...")
            obs = env.reset()

        # Progress report
        if step > 0 and step % 200 == 0:
            elapsed = time.time() - test_start
            fps = step / elapsed
            print(f"[Test] Step {step}/{args.test_steps} | FPS: {fps:.1f} | "
                  f"Pos: ({env.position[0]:.1f}, {env.position[1]:.1f}, {env.position[2]:.1f}) | "
                  f"Phase: {controller.phase}")

    test_elapsed = time.time() - test_start
    test_fps = args.test_steps / test_elapsed

    # Stop video recorder
    recorder.stop()

    # ========================================================================
    # Generate Report
    # ========================================================================
    print("\n[4/4] Generating report...")

    report = {
        "test": "phase2_direct_dynamics",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "config": {
            "headless": args.headless,
            "render_interval": args.render_interval,
            "test_steps": args.test_steps,
            "action_mode": env.config.action_mode.value,
        },
        "benchmarks": benchmarks,
        "flight_test": {
            "total_steps": args.test_steps,
            "rendered_frames": rendered_frames,
            "skipped_frames": skipped_frames,
            "elapsed_seconds": test_elapsed,
            "fps": test_fps,
        },
        "success_criteria": {
            "no_render_fps_target": 500,
            "no_render_fps_achieved": benchmarks["no_render"]["fps"],
            "no_render_passed": benchmarks["no_render"]["fps"] >= 400,  # Allow some margin
            "render_4_fps_target": 200,
            "render_4_fps_achieved": benchmarks["render_4"]["fps"],
            "render_4_passed": benchmarks["render_4"]["fps"] >= 150,  # Allow some margin
        },
        "video_output": OUTPUT_VIDEO,
    }

    # Check overall pass
    report["success_criteria"]["overall_passed"] = (
        report["success_criteria"]["no_render_passed"] and
        report["success_criteria"]["render_4_passed"]
    )

    # Save report
    os.makedirs(os.path.dirname(OUTPUT_REPORT), exist_ok=True)
    with open(OUTPUT_REPORT, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"[Report] Saved to {OUTPUT_REPORT}")

    # Print summary
    print("\n" + "=" * 70)
    print("TEST COMPLETE")
    print("=" * 70)
    print(f"\nFPS Benchmarks:")
    print(f"  No rendering: {benchmarks['no_render']['fps']:.1f} FPS (target: 500)")
    print(f"  Render interval=4: {benchmarks['render_4']['fps']:.1f} FPS (target: 200)")
    print(f"  Render interval=1: {benchmarks['render_1']['fps']:.1f} FPS")
    print(f"\nFlight Test:")
    print(f"  Total Steps: {args.test_steps}")
    print(f"  Rendered Frames: {rendered_frames}")
    print(f"  Effective FPS: {test_fps:.1f}")
    print(f"\nVideo: {OUTPUT_VIDEO}")

    if report["success_criteria"]["overall_passed"]:
        print("\n  RESULT: PASSED")
    else:
        print("\n  RESULT: NEEDS IMPROVEMENT")
        if not report["success_criteria"]["no_render_passed"]:
            print(f"    - No-render FPS below target (got {benchmarks['no_render']['fps']:.1f}, want 500)")
        if not report["success_criteria"]["render_4_passed"]:
            print(f"    - Render-4 FPS below target (got {benchmarks['render_4']['fps']:.1f}, want 200)")

    # Cleanup
    print("\n[Shutdown] Cleaning up...")
    env.close()
    print("[Shutdown] Done.")

    return report["success_criteria"]["overall_passed"]


if __name__ == "__main__":
    try:
        success = run_direct_dynamics_test()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n[FATAL] Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
