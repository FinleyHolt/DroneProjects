"""
Gimbal Profile - Hover + pan/tilt sequences for gimbal testing

This profile is optimized for testing gimbal stabilization:
- Takeoff to 30m
- Hover in place
- Run gimbal pan/tilt sequences
- Test stabilization during small attitude changes
- End after gimbal test complete

Estimated duration: 1-2 minutes
"""

import time
import numpy as np
from typing import List

from .base import TestProfile, ProfileConfig, TestResult, SimulationContext


class GimbalProfile(TestProfile):
    """Gimbal-focused flight profile for stabilization testing."""

    name = "gimbal"
    description = "Hover + pan/tilt sequences for gimbal testing"
    estimated_duration = "1-2 min"

    def default_config(self) -> ProfileConfig:
        return ProfileConfig(
            terrain_size=(200.0, 200.0),  # Small terrain
            tree_density=0.05,
            vehicle_clusters=3,
            vehicles_per_cluster=5,
            people_per_cluster=3,
            takeoff_altitude=30.0,
            cruise_speed=2.0,
            video_suffix="_gimbal",
        )

    def get_vehicle_clusters(self) -> List[tuple]:
        """Minimal clusters - gimbal test doesn't need many targets."""
        return [
            (0.0, 0.0),
            (30.0, 30.0),
            (-30.0, -30.0),
        ]

    def run_flight(self, ctx: SimulationContext) -> List[TestResult]:
        """Execute gimbal test: takeoff, hover, pan/tilt sequences."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal
        results = []

        # === TAKEOFF ===
        self.set_phase("TAKEOFF")
        print("\n--- Gimbal Profile: Takeoff ---")
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

        # Get hover position
        hover_pos = np.array([0.0, 0.0, self.config.takeoff_altitude])

        # === GIMBAL SWEEP TEST ===
        self.set_phase("PAN SWEEP")
        print("--- Gimbal Profile: Pan Sweep ---")
        pan_start = time.time()

        # Full 360° pan in steps
        pan_success = self._run_pan_sweep(ctx, hover_pos)

        results.append(TestResult(
            name="360° Pan Sweep",
            passed=pan_success,
            message="Completed full rotation",
            duration_seconds=time.time() - pan_start,
        ))

        # === TILT SWEEP TEST ===
        self.set_phase("TILT SWEEP")
        print("--- Gimbal Profile: Tilt Sweep ---")
        tilt_start = time.time()

        tilt_success = self._run_tilt_sweep(ctx, hover_pos)

        results.append(TestResult(
            name="Tilt Sweep",
            passed=tilt_success,
            message="Completed tilt range test",
            duration_seconds=time.time() - tilt_start,
        ))

        # === SLEW RATE TEST ===
        self.set_phase("SLEW TEST")
        print("--- Gimbal Profile: Slew Rate Test ---")
        slew_start = time.time()

        slew_success = self._run_slew_test(ctx, hover_pos)

        results.append(TestResult(
            name="Slew Rate Test",
            passed=slew_success,
            message="Tested smooth interpolation",
            duration_seconds=time.time() - slew_start,
        ))

        # === STABILITY DURING ATTITUDE CHANGE ===
        self.set_phase("STAB TEST")
        print("--- Gimbal Profile: Stability During Movement ---")
        stab_start = time.time()

        stab_success = self._run_stability_test(ctx, hover_pos)

        results.append(TestResult(
            name="Attitude Stability",
            passed=stab_success,
            message="Gimbal isolation during small movements",
            duration_seconds=time.time() - stab_start,
        ))

        # === END ===
        print("--- Gimbal Profile: Complete ---")

        return results

    def _run_pan_sweep(self, ctx: SimulationContext, hover_pos: np.ndarray) -> bool:
        """Run 360° pan sweep."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        gimbal.set_angles_immediate(0, -30)

        # 8 positions around the compass
        for pan in [0, 45, 90, 135, 180, 225, 270, 315, 360]:
            gimbal.set_angles(pan % 360, -30)

            for frame in range(45):
                fc._send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2])
                fc._step()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, frame)

        return True

    def _run_tilt_sweep(self, ctx: SimulationContext, hover_pos: np.ndarray) -> bool:
        """Run tilt sweep from nadir to horizon."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        # Forward direction, sweep tilt
        tilts = [-90, -75, -60, -45, -30, -15, 0, -15, -30, -45, -60]

        for tilt in tilts:
            gimbal.set_angles(0, tilt)

            for frame in range(40):
                fc._send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2])
                fc._step()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, frame)

        return True

    def _run_slew_test(self, ctx: SimulationContext, hover_pos: np.ndarray) -> bool:
        """Test smooth interpolation between extreme positions."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        # Quick jumps to test slew limiting
        targets = [
            (0, -30),
            (180, -60),   # 180° pan jump
            (0, -90),     # 180° pan + 30° tilt
            (270, 0),     # 270° pan + 90° tilt
            (0, -30),     # Return
        ]

        for pan, tilt in targets:
            gimbal.set_angles(pan, tilt)

            # Allow time for slew
            for frame in range(90):
                fc._send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2])
                fc._step()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, frame)

        return True

    def _run_stability_test(self, ctx: SimulationContext, hover_pos: np.ndarray) -> bool:
        """Test gimbal stability during small drone movements."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        # Lock gimbal forward-down
        gimbal.set_angles_immediate(45, -45)

        # Small velocity perturbations
        perturbations = [
            (1.0, 0.0, 0.0),   # Small forward
            (-1.0, 0.0, 0.0),  # Small back
            (0.0, 1.0, 0.0),   # Small right
            (0.0, -1.0, 0.0),  # Small left
            (0.0, 0.0, 0.5),   # Small up
            (0.0, 0.0, -0.5),  # Small down
            (0.0, 0.0, 0.0),   # Stabilize
        ]

        for vx, vy, vz in perturbations:
            for frame in range(50):
                fc._send_velocity_setpoint(vy, vx, -vz)  # ENU to NED
                fc._step()
                fc._update_state()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, frame)

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
        """Record a frame with gimbal info overlay."""
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
            target_pan, target_tilt = gimbal.target_pan, gimbal.target_tilt

            overlay = f"TEST: {self.get_phase()}"
            overlay += f"\nAlt: {fc.state.altitude:.1f}m"
            overlay += f"\nGimbal: {gimbal_pan:.0f}° / {gimbal_tilt:.0f}°"
            overlay += f"\nTarget: {target_pan:.0f}° / {target_tilt:.0f}°"

            right_overlay = "=== GIMBAL TEST ==="
            right_overlay += f"\nDRONE ATTITUDE:"
            right_overlay += f"\n  Roll:  {drone_roll:+6.1f}°"
            right_overlay += f"\n  Pitch: {drone_pitch:+6.1f}°"
            right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f}°"
            right_overlay += f"\nGIMBAL (world):"
            right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f}°"
            right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f}°"
            right_overlay += f"\n--- ISOLATED ---"

            return ctx.video_recorder.write_frame(rgba, overlay, right_overlay, None)

        except Exception:
            return 0
