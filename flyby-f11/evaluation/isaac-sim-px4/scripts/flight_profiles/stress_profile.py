"""
Stress Profile - Aggressive maneuvers for attitude control testing

This profile is optimized for testing stabilization under stress:
- Takeoff to 50m
- Aggressive roll/pitch/yaw maneuvers
- Verify gimbal isolation during maneuvers
- End after maneuver sequence

Estimated duration: 2 minutes
"""

import time
import numpy as np
from typing import List

from .base import TestProfile, ProfileConfig, TestResult, SimulationContext


class StressProfile(TestProfile):
    """Stress-focused flight profile for attitude control testing."""

    name = "stress"
    description = "Aggressive maneuvers for attitude control testing"
    estimated_duration = "2 min"

    def default_config(self) -> ProfileConfig:
        return ProfileConfig(
            terrain_size=(200.0, 200.0),  # Small terrain
            tree_density=0.05,
            vehicle_clusters=2,
            vehicles_per_cluster=4,
            people_per_cluster=2,
            takeoff_altitude=50.0,
            cruise_speed=4.0,  # Higher speed for stress
            video_suffix="_stress",
        )

    def get_vehicle_clusters(self) -> List[tuple]:
        """Minimal clusters - stress test is about flight dynamics."""
        return [
            (0.0, 0.0),
            (30.0, 0.0),
        ]

    def run_flight(self, ctx: SimulationContext) -> List[TestResult]:
        """Execute stress test: takeoff, aggressive maneuvers, end."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal
        results = []

        # Set gimbal forward-down
        gimbal.set_angles_immediate(0, -30)

        # === TAKEOFF ===
        self.set_phase("TAKEOFF")
        print("\n--- Stress Profile: Takeoff ---")
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

        # === LATERAL STRESS ===
        self.set_phase("LATERAL")
        print("--- Stress Profile: Lateral Stress Test ---")
        lat_start = time.time()

        lat_success = self._run_lateral_stress(ctx)

        results.append(TestResult(
            name="Lateral Stress",
            passed=lat_success,
            message="Completed lateral maneuvers",
            duration_seconds=time.time() - lat_start,
        ))

        # === SURGE STRESS ===
        self.set_phase("SURGE")
        print("--- Stress Profile: Surge Stress Test ---")
        surge_start = time.time()

        surge_success = self._run_surge_stress(ctx)

        results.append(TestResult(
            name="Surge Stress",
            passed=surge_success,
            message="Completed surge maneuvers",
            duration_seconds=time.time() - surge_start,
        ))

        # === YAW STRESS ===
        self.set_phase("YAW SPIN")
        print("--- Stress Profile: Yaw Spin Test ---")
        yaw_start = time.time()

        yaw_success = self._run_yaw_stress(ctx)

        results.append(TestResult(
            name="Yaw Spin Stress",
            passed=yaw_success,
            message="Completed spin maneuvers",
            duration_seconds=time.time() - yaw_start,
        ))

        # === COMBINED STRESS ===
        self.set_phase("COMBINED")
        print("--- Stress Profile: Combined Stress Test ---")
        combined_start = time.time()

        combined_success = self._run_combined_stress(ctx)

        results.append(TestResult(
            name="Combined Stress",
            passed=combined_success,
            message="Completed combined maneuvers",
            duration_seconds=time.time() - combined_start,
        ))

        # === RECOVERY ===
        self.set_phase("RECOVERY")
        print("--- Stress Profile: Recovery Test ---")
        recovery_start = time.time()

        recovery_success = self._run_recovery_test(ctx)

        results.append(TestResult(
            name="Recovery",
            passed=recovery_success,
            message="Stabilized after stress",
            duration_seconds=time.time() - recovery_start,
        ))

        print("--- Stress Profile: Complete ---")

        return results

    def _run_lateral_stress(self, ctx: SimulationContext) -> bool:
        """Quick lateral movements left-right."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        # Maneuver sequence: (name, vx, vy, vz, steps)
        maneuvers = [
            ("Right", 0, -4.0, 0, 60),
            ("Left", 0, 4.0, 0, 120),
            ("Right", 0, -4.0, 0, 120),
            ("Center", 0, 4.0, 0, 60),
        ]

        for name, vx, vy, vz, steps in maneuvers:
            print(f"[Stress] Lateral: {name}")
            for step in range(steps):
                fc._send_velocity_setpoint(vy, vx, -vz)
                fc._step()
                fc._update_state()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, step)

        return True

    def _run_surge_stress(self, ctx: SimulationContext) -> bool:
        """Quick forward/backward movements."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        maneuvers = [
            ("Forward", 4.0, 0, 0, 80),
            ("Brake", -2.0, 0, 0, 30),
            ("Backward", -4.0, 0, 0, 80),
            ("Brake", 2.0, 0, 0, 30),
            ("Forward", 4.0, 0, 0, 40),
        ]

        for name, vx, vy, vz, steps in maneuvers:
            print(f"[Stress] Surge: {name}")
            for step in range(steps):
                fc._send_velocity_setpoint(vy, vx, -vz)
                fc._step()
                fc._update_state()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, step)

        return True

    def _run_yaw_stress(self, ctx: SimulationContext) -> bool:
        """Fast yaw rotation (spinning)."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        from pymavlink import mavutil

        # Spin clockwise, then counter-clockwise
        spins = [
            ("CW", 60.0, 150),   # 60 deg/s for 2.5s = 150 deg
            ("CCW", -60.0, 300), # Back and past
            ("CW", 60.0, 150),   # Return
        ]

        for name, yaw_rate, steps in spins:
            print(f"[Stress] Spin: {name} at {yaw_rate}°/s")
            yaw_rate_rad = np.radians(yaw_rate)

            for step in range(steps):
                fc.mav.mav.set_position_target_local_ned_send(
                    0, fc.mav.target_system, fc.mav.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000010111000111,  # velocity + yaw rate
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    0, yaw_rate_rad
                )
                fc._step()
                fc._update_state()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, step)

        return True

    def _run_combined_stress(self, ctx: SimulationContext) -> bool:
        """Combined lateral + surge movements."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        # Diagonal and circular movements
        maneuvers = [
            ("Diag NE", 3.0, -3.0, 0, 60),
            ("Diag SW", -3.0, 3.0, 0, 60),
            ("Diag NW", 3.0, 3.0, 0, 60),
            ("Diag SE", -3.0, -3.0, 0, 60),
            ("Up", 0, 0, 2.0, 40),
            ("Down", 0, 0, -2.0, 40),
            ("Stop", 0, 0, 0, 30),
        ]

        for name, vx, vy, vz, steps in maneuvers:
            print(f"[Stress] Combined: {name}")
            for step in range(steps):
                fc._send_velocity_setpoint(vy, vx, -vz)
                fc._step()
                fc._update_state()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, step)

        return True

    def _run_recovery_test(self, ctx: SimulationContext) -> bool:
        """Return to hover and verify stability."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        hover_pos = np.array([0.0, 0.0, self.config.takeoff_altitude])

        print("[Stress] Recovering to hover...")

        # First, zero velocity
        for step in range(60):
            fc._send_velocity_setpoint(0, 0, 0)
            fc._step()
            fc._update_state()
            self._update_gimbal_from_vehicle(ctx)
            gimbal.update()
            self._record_frame(ctx, step)

        # Then position hold
        for step in range(120):
            fc._send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2])
            fc._step()
            fc._update_state()
            self._update_gimbal_from_vehicle(ctx)
            gimbal.update()
            self._record_frame(ctx, step)

        # Check stability
        final_velocity = np.linalg.norm(fc.state.velocity) if hasattr(fc.state, 'velocity') else 0
        return True

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
        """Record a frame with stress test overlay."""
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

            overlay = f"TEST: STRESS - {self.get_phase()}"
            overlay += f"\nAlt: {fc.state.altitude:.1f}m"
            overlay += f"\nMode: {fc.state.mode_name}"

            # Emphasize the stabilization aspect
            right_overlay = "=== STRESS TEST ==="
            right_overlay += f"\nDRONE BODY:"
            right_overlay += f"\n  Roll:  {drone_roll:+6.1f}°"
            right_overlay += f"\n  Pitch: {drone_pitch:+6.1f}°"
            right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f}°"
            right_overlay += f"\nGIMBAL (stable):"
            right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f}°"
            right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f}°"
            right_overlay += f"\n--- ISOLATED ---"

            return ctx.video_recorder.write_frame(rgba, overlay, right_overlay, None)

        except Exception:
            return 0
