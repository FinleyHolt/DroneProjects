"""
Detection Profile - 360° pan for YOLO/GT overlay testing

This profile is optimized for testing object detection and ground truth:
- Takeoff to 50m over dense vehicle cluster
- Hover and pan camera 360° slowly
- Tilt up/down while panning
- End immediately after pan complete (skip landing for speed)

Uses Isaac Sim's native bounding_box_2d_tight annotator for pixel-perfect GT.

Estimated duration: 2-3 minutes
"""

import time
import numpy as np
from typing import List

from .base import TestProfile, ProfileConfig, TestResult, SimulationContext


class DetectionProfile(TestProfile):
    """Detection-focused flight profile for YOLO/GT overlay testing."""

    name = "detection"
    description = "360° pan for detection/GT overlay testing"
    estimated_duration = "2-3 min"

    def default_config(self) -> ProfileConfig:
        return ProfileConfig(
            terrain_size=(300.0, 300.0),
            tree_density=0.1,
            vehicle_clusters=5,
            vehicles_per_cluster=12,
            people_per_cluster=6,
            takeoff_altitude=50.0,
            cruise_speed=2.0,
            video_suffix="_detection",
        )

    def get_vehicle_clusters(self) -> List[tuple]:
        """Dense cluster pattern centered under hover point."""
        return [
            (0.0, 0.0),
            (30.0, 0.0),
            (-30.0, 0.0),
            (0.0, 30.0),
            (0.0, -30.0),
        ]

    def run_flight(self, ctx: SimulationContext) -> List[TestResult]:
        """Execute detection test: takeoff, 360° pan, end."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        results = []
        total_detections = 0

        # === TAKEOFF ===
        self.set_phase("TAKEOFF")
        print("\n--- Detection Profile: Takeoff ---")
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
            data={"target": self.config.takeoff_altitude, "reached": fc.state.altitude},
        ))

        if not success:
            return results

        # === POSITION OVER TARGET ===
        self.set_phase("POSITION")
        print("--- Detection Profile: Positioning over target cluster ---")

        center_pos = np.array([0.0, 0.0, self.config.takeoff_altitude])
        fc.goto_position(
            center_pos, tolerance=3.0, max_steps=500,
            callback=lambda step, state, idx=None: self._record_frame(ctx, step)
        )

        # === 360° PAN TEST ===
        self.set_phase("360 PAN")
        print("--- Detection Profile: 360° Pan Sequence ---")
        pan_start = time.time()

        pan_steps = 8
        frames_per_position = 60
        tilt_angles = [-30, -45, -60, -45, -30]

        gimbal.set_angles_immediate(0, -45)

        for pan_idx in range(pan_steps):
            pan_angle = pan_idx * 45
            tilt_angle = tilt_angles[pan_idx % len(tilt_angles)]

            print(f"[Detection] Pan {pan_angle}° Tilt {tilt_angle}°")
            gimbal.set_angles(pan_angle, tilt_angle)

            for frame in range(frames_per_position):
                fc._send_position_setpoint(center_pos[0], center_pos[1], center_pos[2])
                fc._step()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                detections = self._record_frame(ctx, frame)
                total_detections += detections

        # Complete the circle
        gimbal.set_angles(0, -30)
        for frame in range(30):
            fc._send_position_setpoint(center_pos[0], center_pos[1], center_pos[2])
            fc._step()
            self._update_gimbal_from_vehicle(ctx)
            gimbal.update()
            self._record_frame(ctx, frame)

        results.append(TestResult(
            name="360° Pan",
            passed=True,
            message=f"Completed {pan_steps} pan positions",
            duration_seconds=time.time() - pan_start,
            data={"positions": pan_steps, "frames_per_position": frames_per_position},
        ))

        # === TILT SWEEP ===
        self.set_phase("TILT SWEEP")
        print("--- Detection Profile: Tilt Sweep ---")
        tilt_start = time.time()

        tilt_sequence = [-90, -75, -60, -45, -30, -15, 0, -15, -30, -45]
        frames_per_tilt = 45

        for tilt in tilt_sequence:
            gimbal.set_angles(0, tilt)

            for frame in range(frames_per_tilt):
                fc._send_position_setpoint(center_pos[0], center_pos[1], center_pos[2])
                fc._step()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                detections = self._record_frame(ctx, frame)
                total_detections += detections

        results.append(TestResult(
            name="Tilt Sweep",
            passed=True,
            message=f"Completed tilt sweep: {tilt_sequence}",
            duration_seconds=time.time() - tilt_start,
        ))

        print("--- Detection Profile: Complete (skipping landing) ---")

        if hasattr(ctx, 'total_detections'):
            ctx.total_detections = total_detections

        return results

    def _update_gimbal_from_vehicle(self, ctx: SimulationContext):
        """Update gimbal with current vehicle attitude for stabilization."""
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
        """Record a frame with overlay. Returns detection count."""
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

            # Left overlay - test info
            overlay = f"TEST: {self.get_phase()}"
            overlay += f"\nAlt: {fc.state.altitude:.1f}m | Mode: {fc.state.mode_name}"
            overlay += f"\nPos: ({fc.state.position[0]:.1f}, {fc.state.position[1]:.1f})"

            # Right overlay - stabilization
            right_overlay = "=== STABILIZATION ==="
            right_overlay += f"\nDRONE BODY:"
            right_overlay += f"\n  Roll:  {drone_roll:+6.1f} deg"
            right_overlay += f"\n  Pitch: {drone_pitch:+6.1f} deg"
            right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f} deg"
            right_overlay += f"\nGIMBAL (world):"
            right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f} deg"
            right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f} deg"

            # Get ground truth using native annotator
            ground_truth = self._get_ground_truth(ctx)

            return ctx.video_recorder.write_frame(rgba, overlay, right_overlay, ground_truth)

        except Exception as e:
            if step % 100 == 0:
                print(f"[Detection] Frame error: {e}")
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
