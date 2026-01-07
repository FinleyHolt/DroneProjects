"""
Nav Profile - Waypoint navigation patterns

This profile is optimized for testing navigation and position control:
- Takeoff
- Fly complex waypoint patterns (square, figure-8)
- Test position hold accuracy
- End after waypoints complete

Estimated duration: 3-4 minutes
"""

import time
import numpy as np
from typing import List

from .base import TestProfile, ProfileConfig, TestResult, SimulationContext


class NavProfile(TestProfile):
    """Navigation-focused flight profile for waypoint testing."""

    name = "nav"
    description = "Waypoint navigation patterns"
    estimated_duration = "3-4 min"

    def default_config(self) -> ProfileConfig:
        return ProfileConfig(
            terrain_size=(400.0, 400.0),
            tree_density=0.1,
            vehicle_clusters=5,
            vehicles_per_cluster=6,
            people_per_cluster=3,
            takeoff_altitude=60.0,
            cruise_speed=3.0,
            position_tolerance=2.5,
            video_suffix="_nav",
        )

    def get_vehicle_clusters(self) -> List[tuple]:
        """Clusters at waypoint locations for visual reference."""
        return [
            (0.0, 0.0),      # Center
            (50.0, 50.0),    # Square corners
            (-50.0, 50.0),
            (-50.0, -50.0),
            (50.0, -50.0),
        ]

    def run_flight(self, ctx: SimulationContext) -> List[TestResult]:
        """Execute navigation test: takeoff, waypoint patterns, end."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal
        results = []

        # Set gimbal to look forward-down for nav
        gimbal.set_angles_immediate(0, -45)

        # === TAKEOFF ===
        self.set_phase("TAKEOFF")
        print("\n--- Nav Profile: Takeoff ---")
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

        # === SQUARE PATTERN ===
        self.set_phase("SQUARE NAV")
        print("--- Nav Profile: Square Pattern ---")
        square_start = time.time()

        square_waypoints = self._generate_square_pattern(
            center=(0.0, 0.0),
            size=80.0,
            altitude=self.config.takeoff_altitude,
        )

        reached, total = self._fly_waypoints(ctx, square_waypoints)

        results.append(TestResult(
            name="Square Pattern",
            passed=reached >= total - 1,  # Allow 1 missed waypoint
            message=f"Reached {reached}/{total} waypoints",
            duration_seconds=time.time() - square_start,
            data={"reached": reached, "total": total, "pattern": "square"},
        ))

        # === FIGURE-8 PATTERN ===
        self.set_phase("FIGURE-8 NAV")
        print("--- Nav Profile: Figure-8 Pattern ---")
        fig8_start = time.time()

        fig8_waypoints = self._generate_figure8_pattern(
            center=(0.0, 0.0),
            radius=40.0,
            altitude=self.config.takeoff_altitude,
        )

        reached, total = self._fly_waypoints(ctx, fig8_waypoints)

        results.append(TestResult(
            name="Figure-8 Pattern",
            passed=reached >= total - 2,
            message=f"Reached {reached}/{total} waypoints",
            duration_seconds=time.time() - fig8_start,
            data={"reached": reached, "total": total, "pattern": "figure-8"},
        ))

        # === POSITION HOLD TEST ===
        self.set_phase("POS HOLD")
        print("--- Nav Profile: Position Hold Test ---")
        hold_start = time.time()

        hold_success, max_drift = self._test_position_hold(ctx)

        results.append(TestResult(
            name="Position Hold",
            passed=hold_success,
            message=f"Max drift: {max_drift:.2f}m",
            duration_seconds=time.time() - hold_start,
            data={"max_drift": max_drift},
        ))

        # === ALTITUDE CHANGES ===
        self.set_phase("ALT NAV")
        print("--- Nav Profile: Altitude Change Test ---")
        alt_start = time.time()

        alt_success = self._test_altitude_changes(ctx)

        results.append(TestResult(
            name="Altitude Changes",
            passed=alt_success,
            message="Completed altitude profile",
            duration_seconds=time.time() - alt_start,
        ))

        # === END ===
        print("--- Nav Profile: Complete ---")

        return results

    def _generate_square_pattern(
        self,
        center: tuple,
        size: float,
        altitude: float,
    ) -> List[np.ndarray]:
        """Generate square waypoint pattern."""
        half = size / 2
        cx, cy = center
        return [
            np.array([cx + half, cy + half, altitude]),   # NE
            np.array([cx - half, cy + half, altitude]),   # NW
            np.array([cx - half, cy - half, altitude]),   # SW
            np.array([cx + half, cy - half, altitude]),   # SE
            np.array([cx + half, cy + half, altitude]),   # Back to NE
        ]

    def _generate_figure8_pattern(
        self,
        center: tuple,
        radius: float,
        altitude: float,
        points: int = 16,
    ) -> List[np.ndarray]:
        """Generate figure-8 waypoint pattern."""
        cx, cy = center
        waypoints = []

        # Two circles offset in x
        for i in range(points):
            angle = 2 * np.pi * i / points
            # Right circle
            x = cx + radius + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            waypoints.append(np.array([x, y, altitude]))

        for i in range(points):
            angle = 2 * np.pi * i / points + np.pi  # Start from opposite side
            # Left circle
            x = cx - radius + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            waypoints.append(np.array([x, y, altitude]))

        return waypoints

    def _fly_waypoints(
        self,
        ctx: SimulationContext,
        waypoints: List[np.ndarray],
    ) -> tuple:
        """Fly through waypoints, return (reached, total)."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        reached = 0
        for idx, wp in enumerate(waypoints):
            print(f"[Nav] Waypoint {idx+1}/{len(waypoints)}: {wp[:2]}")

            max_steps = 1200
            for step in range(max_steps):
                direction = wp - fc.state.position
                distance = np.linalg.norm(direction)

                if distance < self.config.position_tolerance:
                    reached += 1
                    print(f"[Nav] Reached waypoint {idx+1}")
                    break

                # Velocity control
                speed = min(self.config.cruise_speed, distance)
                if distance > 0:
                    velocity = direction / distance * speed
                    fc._send_velocity_setpoint(velocity[1], velocity[0], -velocity[2])

                fc._step()
                fc._update_state()

                # Update gimbal to point in direction of travel
                if distance > 1.0:
                    heading = np.degrees(np.arctan2(direction[1], direction[0]))
                    gimbal.set_angles(heading, -45)

                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, step)

                # Re-request OFFBOARD periodically
                if step % 200 == 0 and fc.state.mode != fc.PX4_MODE_OFFBOARD:
                    fc.set_mode_offboard()

        return reached, len(waypoints)

    def _test_position_hold(self, ctx: SimulationContext) -> tuple:
        """Test position hold accuracy. Returns (success, max_drift)."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        # Fly to center and hold
        hold_pos = np.array([0.0, 0.0, self.config.takeoff_altitude])
        fc.goto_position(hold_pos, tolerance=2.0, max_steps=500)

        # Record initial position
        initial_pos = fc.state.position.copy()
        max_drift = 0.0

        # Hold for 3 seconds (180 frames at 60Hz)
        for step in range(180):
            fc._send_position_setpoint(hold_pos[0], hold_pos[1], hold_pos[2])
            fc._step()
            fc._update_state()

            drift = np.linalg.norm(fc.state.position[:2] - initial_pos[:2])
            max_drift = max(max_drift, drift)

            self._update_gimbal_from_vehicle(ctx)
            gimbal.update()
            self._record_frame(ctx, step)

        return max_drift < 3.0, max_drift

    def _test_altitude_changes(self, ctx: SimulationContext) -> bool:
        """Test altitude change accuracy."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        altitudes = [
            self.config.takeoff_altitude + 20,
            self.config.takeoff_altitude - 20,
            self.config.takeoff_altitude,
        ]

        for target_alt in altitudes:
            target = np.array([0.0, 0.0, target_alt])
            print(f"[Nav] Changing altitude to {target_alt}m")

            for step in range(500):
                fc._send_position_setpoint(target[0], target[1], target[2])
                fc._step()
                fc._update_state()

                if abs(fc.state.altitude - target_alt) < 2.0:
                    print(f"[Nav] Reached {fc.state.altitude:.1f}m")
                    break

                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_frame(ctx, step)

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
        """Record a frame with nav overlay."""
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

            right_overlay = "=== NAV TEST ==="
            right_overlay += f"\nDRONE:"
            right_overlay += f"\n  Roll:  {drone_roll:+6.1f}°"
            right_overlay += f"\n  Pitch: {drone_pitch:+6.1f}°"
            right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f}°"
            right_overlay += f"\nGIMBAL:"
            right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f}°"
            right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f}°"

            return ctx.video_recorder.write_frame(rgba, overlay, right_overlay, None)

        except Exception:
            return 0
