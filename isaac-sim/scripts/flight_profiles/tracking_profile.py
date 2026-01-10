"""
Tracking Profile - Linear flight for ByteTrack temporal coherence testing

This profile is optimized for testing object tracking consistency:
- Takeoff to 60m
- Fly straight line over multiple vehicle clusters
- Maintain consistent speed for track continuity
- End after reaching final waypoint (skip landing)

Uses Isaac Sim's native bounding_box_2d_tight annotator for pixel-perfect GT.

Estimated duration: 2-3 minutes
"""

import time
import numpy as np
from typing import List

from .base import TestProfile, ProfileConfig, TestResult, SimulationContext


class TrackingProfile(TestProfile):
    """Tracking-focused flight profile for ByteTrack temporal coherence."""

    name = "tracking"
    description = "Linear flight over vehicle clusters for ByteTrack testing"
    estimated_duration = "2-3 min"

    def default_config(self) -> ProfileConfig:
        return ProfileConfig(
            terrain_size=(400.0, 400.0),
            tree_density=0.1,
            vehicle_clusters=8,
            vehicles_per_cluster=10,
            people_per_cluster=5,
            takeoff_altitude=60.0,
            cruise_speed=2.5,
            video_suffix="_tracking",
        )

    def get_vehicle_clusters(self) -> List[tuple]:
        """Linear cluster pattern along flight path."""
        return [
            (-150.0, 0.0),
            (-100.0, 0.0),
            (-50.0, 0.0),
            (0.0, 0.0),
            (50.0, 0.0),
            (100.0, 0.0),
            (150.0, 0.0),
            (0.0, 50.0),
        ]

    def run_flight(self, ctx: SimulationContext) -> List[TestResult]:
        """Execute tracking test: takeoff, linear flight, end."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal
        results = []

        # === TAKEOFF ===
        self.set_phase("TAKEOFF")
        print("\n--- Tracking Profile: Takeoff ---")
        takeoff_start = time.time()

        success = fc.takeoff(
            self.config.takeoff_altitude,
            callback=lambda step, state: self._record_frame(ctx, step)
        )

        results.append(TestResult(
            name="Takeoff",
            passed=success,
            message=f"Reached {fc.state.altitude:.1f}m",
            duration_seconds=time.time() - takeoff_start,
        ))

        if not success:
            return results

        # === POSITION AT START ===
        self.set_phase("START POS")
        print("--- Tracking Profile: Moving to start position ---")

        start_pos = np.array([-180.0, 0.0, self.config.takeoff_altitude])
        fc.goto_position(
            start_pos, tolerance=3.0, max_steps=1000,
            callback=lambda step, state, idx=None: self._record_frame(ctx, step)
        )

        gimbal.set_angles_immediate(0, -45)

        # === LINEAR FLIGHT ===
        self.set_phase("LINEAR TRACK")
        print("--- Tracking Profile: Linear flight over clusters ---")
        flight_start = time.time()

        end_pos = np.array([180.0, 0.0, self.config.takeoff_altitude])

        track_success = self._fly_straight_line(ctx, start_pos, end_pos, speed=self.config.cruise_speed)

        results.append(TestResult(
            name="Linear Tracking Flight",
            passed=track_success,
            message=f"Flew 360m at {self.config.cruise_speed}m/s",
            duration_seconds=time.time() - flight_start,
            data={"distance": 360.0, "speed": self.config.cruise_speed},
        ))

        # === RETURN PASS ===
        self.set_phase("RETURN PASS")
        print("--- Tracking Profile: Return pass ---")
        return_start = time.time()

        gimbal.set_angles(180, -45)
        for _ in range(45):
            fc._step()
            self._update_gimbal_from_vehicle(ctx)
            gimbal.update()
            self._record_frame(ctx, 0)

        return_success = self._fly_straight_line(ctx, end_pos, start_pos, speed=self.config.cruise_speed)

        results.append(TestResult(
            name="Return Tracking Pass",
            passed=return_success,
            message="Completed return pass",
            duration_seconds=time.time() - return_start,
        ))

        print("--- Tracking Profile: Complete (skipping landing) ---")

        return results

    def _fly_straight_line(
        self,
        ctx: SimulationContext,
        start: np.ndarray,
        end: np.ndarray,
        speed: float,
    ) -> bool:
        """Fly a straight line at constant velocity."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        direction = end - start
        distance = np.linalg.norm(direction)
        unit_dir = direction / distance

        expected_time = distance / speed
        max_steps = int(expected_time * 60 * 1.5)

        print(f"[Tracking] Flying {distance:.0f}m at {speed}m/s (~{expected_time:.0f}s)")

        for step in range(max_steps):
            current_pos = fc.state.position
            dist_remaining = np.linalg.norm(end - current_pos)

            if dist_remaining < 3.0:
                print("[Tracking] Reached end position")
                return True

            velocity = unit_dir * speed
            fc._send_velocity_setpoint(velocity[1], velocity[0], -velocity[2])
            fc._step()
            fc._update_state()

            self._update_gimbal_from_vehicle(ctx)
            gimbal.update()
            self._record_frame(ctx, step)

            if step % 120 == 0:
                print(f"[Tracking] Progress: {distance - dist_remaining:.0f}/{distance:.0f}m")

        return False

    def _update_gimbal_from_vehicle(self, ctx: SimulationContext):
        """Update gimbal with current vehicle attitude."""
        try:
            from scipy.spatial.transform import Rotation
            quat = ctx.vehicle.state.attitude
            if quat is not None and len(quat) == 4:
                rot = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
                euler = rot.as_euler('xyz', degrees=True)
                ctx.gimbal.update_drone_attitude(euler[0], euler[1], euler[2])
        except Exception:
            pass

    def _record_frame(self, ctx: SimulationContext, step: int) -> int:
        """Record a frame with overlay."""
        if ctx.video_recorder is None or ctx.camera is None:
            return 0

        try:
            rgba = ctx.camera.get_rgba()
            if rgba is None or rgba.size == 0:
                return 0

            fc = ctx.flight_controller
            gimbal = ctx.gimbal

            drone_roll, drone_pitch, drone_yaw = gimbal.get_drone_attitude()
            gimbal_pan, gimbal_tilt = gimbal.get_gimbal_angles()

            overlay = f"TEST: {self.get_phase()}"
            overlay += f"\nAlt: {fc.state.altitude:.1f}m | Mode: {fc.state.mode_name}"
            overlay += f"\nPos: ({fc.state.position[0]:.1f}, {fc.state.position[1]:.1f})"

            right_overlay = "=== TRACKING TEST ==="
            right_overlay += f"\nDRONE:"
            right_overlay += f"\n  Roll:  {drone_roll:+6.1f}"
            right_overlay += f"\n  Pitch: {drone_pitch:+6.1f}"
            right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f}"
            right_overlay += f"\nGIMBAL:"
            right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f}"
            right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f}"

            ground_truth = self._get_ground_truth(ctx)
            return ctx.video_recorder.write_frame(rgba, overlay, right_overlay, ground_truth)

        except Exception:
            return 0

    def _get_ground_truth(self, ctx: SimulationContext):
        """Get ground truth detections using native annotator."""
        if ctx.perception_manager is None:
            return None

        if not ctx.perception_manager.initialized:
            return None

        try:
            fc = ctx.flight_controller
            uav_pos = fc.state.position
            gt_config = getattr(ctx, 'gt_config', None)

            return ctx.perception_manager.get_ground_truth_detections(
                uav_position=uav_pos,
                gt_config=gt_config,
                stage=ctx.stage,
            )
        except Exception:
            return None
