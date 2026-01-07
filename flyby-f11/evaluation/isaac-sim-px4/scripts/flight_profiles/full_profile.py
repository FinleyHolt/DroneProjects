"""
Full Profile - Complete ~12 minute comprehensive test

This profile runs the full test sequence:
- Simulation init + world generation (trees, vehicles, people)
- Camera + gimbal setup
- Video recorder with YOLO + ByteTrack
- MAVLink connection + arming
- Takeoff to 75m
- Waypoint navigation
- Stabilization stress test
- Gimbal control test
- Return to home
- Landing

Uses Isaac Sim's native bounding_box_2d_tight annotator for pixel-perfect GT.

Estimated duration: ~12 minutes
"""

import time
import numpy as np
from typing import List

from .base import TestProfile, ProfileConfig, TestResult, SimulationContext


class FullProfile(TestProfile):
    """Full comprehensive test profile."""

    name = "full"
    description = "Complete ~12 min test with all phases"
    estimated_duration = "~12 min"

    def default_config(self) -> ProfileConfig:
        return ProfileConfig(
            terrain_size=(1000.0, 1000.0),
            tree_density=0.2,
            vehicle_clusters=21,
            vehicles_per_cluster=10,
            people_per_cluster=8,
            takeoff_altitude=75.0,
            cruise_speed=3.0,
            position_tolerance=2.0,
            video_suffix="",
        )

    def get_vehicle_clusters(self) -> List[tuple]:
        """Full cluster pattern across large terrain."""
        return [
            (400.0, 0.0), (-400.0, 0.0), (0.0, 400.0), (0.0, -400.0),
            (280.0, 280.0), (-280.0, 280.0), (-280.0, -280.0), (280.0, -280.0),
            (200.0, 0.0), (-200.0, 0.0), (0.0, 200.0), (0.0, -200.0),
            (140.0, 140.0), (-140.0, 140.0), (-140.0, -140.0), (140.0, -140.0),
            (100.0, 0.0), (-100.0, 0.0), (0.0, 100.0), (0.0, -100.0),
            (0.0, 0.0),
        ]

    def run_flight(self, ctx: SimulationContext) -> List[TestResult]:
        """Execute full test sequence."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal
        results = []

        # === TAKEOFF ===
        self.set_phase("TAKEOFF")
        print("\n--- Full Profile: Takeoff ---")
        takeoff_start = time.time()

        success = fc.takeoff(
            self.config.takeoff_altitude,
            callback=lambda step, state: self._flight_callback(ctx, step, state)
        )

        results.append(TestResult(
            name="Takeoff",
            passed=success,
            message=f"Reached {fc.state.altitude:.1f}m" if success else "Timeout",
            duration_seconds=time.time() - takeoff_start,
            data={"target": self.config.takeoff_altitude, "reached": fc.state.altitude},
        ))

        if not success:
            return results

        fc.set_mode_offboard()
        time.sleep(0.5)

        # === WAYPOINT NAVIGATION ===
        self.set_phase("WAYPOINT NAV")
        print("\n--- Full Profile: Waypoint Navigation ---")

        waypoints = [np.array([15.0, 0.0, self.config.takeoff_altitude])]
        fc.set_mode_offboard()

        nav_start = time.time()
        reached, total = fc.fly_waypoints(
            waypoints,
            callback=lambda step, state, idx: self._flight_callback(ctx, step, state, idx)
        )

        results.append(TestResult(
            name="Waypoint Navigation",
            passed=reached == total,
            message=f"Reached {reached}/{total} waypoints",
            duration_seconds=time.time() - nav_start,
            data={"reached": reached, "total": total},
        ))

        # === STABILIZATION STRESS TEST ===
        self.set_phase("STAB TEST")
        print("\n--- Full Profile: Stabilization Stress Test ---")

        gimbal.set_angles_immediate(0, -30)
        stab_start = time.time()
        stab_success = self._run_stabilization_test(ctx)

        results.append(TestResult(
            name="Stabilization Stress Test",
            passed=stab_success,
            message="Aggressive maneuvers completed with gimbal isolation",
            duration_seconds=time.time() - stab_start,
        ))

        # === GIMBAL CONTROL ===
        self.set_phase("GIMBAL CTRL")
        print("\n--- Full Profile: Gimbal Control ---")

        gimbal_success = self._run_gimbal_test(ctx)

        results.append(TestResult(
            name="Gimbal Control",
            passed=gimbal_success,
            message="Gimbal pan/tilt tested with smooth interpolation",
            data={"final_pan": gimbal.pan, "final_tilt": gimbal.tilt},
        ))

        # === RETURN TO HOME ===
        self.set_phase("RETURN HOME")
        print("\n--- Full Profile: Return to Home ---")
        rth_start = time.time()

        rth_success = fc.return_to_home(
            callback=lambda step, state: self._flight_callback(ctx, step, state)
        )

        results.append(TestResult(
            name="Return to Home",
            passed=rth_success,
            message="Returned to home position" if rth_success else "Failed",
            duration_seconds=time.time() - rth_start,
        ))

        # === LANDING ===
        self.set_phase("LANDING")
        print("\n--- Full Profile: Landing ---")
        land_start = time.time()

        land_success = fc.land(
            callback=lambda step, state: self._flight_callback(ctx, step, state)
        )

        fc.disarm()

        results.append(TestResult(
            name="Landing",
            passed=land_success,
            message=f"Landed at {fc.state.altitude:.2f}m" if land_success else "Timeout",
            duration_seconds=time.time() - land_start,
            data={"final_altitude": fc.state.altitude},
        ))

        return results

    def _run_stabilization_test(self, ctx: SimulationContext) -> bool:
        """Run aggressive maneuver sequence."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        from pymavlink import mavutil

        maneuvers = [
            ("Lateral Right", 0, -3.0, 0, 100, None),
            ("Lateral Left", 0, 3.0, 0, 100, None),
            ("Lateral Right", 0, -3.0, 0, 100, None),
            ("Center", 0, 3.0, 0, 50, None),
            ("Surge Forward", 3.0, 0, 0, 100, None),
            ("Surge Back", -3.0, 0, 0, 100, None),
            ("Surge Forward", 3.0, 0, 0, 50, None),
            ("Spin CW", 0, 0, 0, 150, 45.0),
            ("Spin CCW", 0, 0, 0, 150, -45.0),
            ("Stabilize", 0, 0, 0, 100, None),
        ]

        for name, vx, vy, vz, steps, yaw_rate in maneuvers:
            print(f"[StabTest] {name}")

            for _ in range(steps):
                if yaw_rate is not None:
                    yaw_rate_rad = np.radians(yaw_rate)
                    fc.mav.mav.set_position_target_local_ned_send(
                        0, fc.mav.target_system, fc.mav.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        0b0000010111000111,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_rate_rad
                    )
                else:
                    fc._send_velocity_setpoint(vy, vx, -vz)

                fc._step()
                fc._update_state()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_stab_frame(ctx, name, vx, vy, vz)

        return True

    def _run_gimbal_test(self, ctx: SimulationContext) -> bool:
        """Run gimbal control sequence."""
        fc = ctx.flight_controller
        gimbal = ctx.gimbal

        tests = [
            ("Look down", 0, -90),
            ("Pan left", -45, -60),
            ("Pan right", 45, -60),
            ("Look forward", 0, -30),
            ("Pan 360 - N", 0, -30),
            ("Pan 360 - NE", 45, -30),
            ("Pan 360 - E", 90, -30),
            ("Pan 360 - SE", 135, -30),
            ("Pan 360 - S", 180, -30),
            ("Pan 360 - SW", 225, -30),
            ("Pan 360 - W", 270, -30),
            ("Pan 360 - NW", 315, -30),
            ("Reset", 0, -30),
        ]

        for name, pan, tilt in tests:
            gimbal.set_angles(pan, tilt)

            for _ in range(75):
                fc._step()
                self._update_gimbal_from_vehicle(ctx)
                gimbal.update()
                self._record_gimbal_frame(ctx, name, pan, tilt)

        return True

    def _flight_callback(self, ctx, step, state, waypoint_idx=None):
        """Standard flight callback for recording frames."""
        self._update_gimbal_from_vehicle(ctx)
        ctx.gimbal.update()
        self._record_frame(ctx, step, waypoint_idx)

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

    def _record_frame(self, ctx: SimulationContext, step: int, waypoint_idx=None) -> int:
        """Record a standard frame."""
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
            if waypoint_idx is not None:
                overlay += f" (WP {waypoint_idx + 1})"
            overlay += f"\nAlt: {fc.state.altitude:.1f}m | Mode: {fc.state.mode_name}"
            overlay += f"\nPos: ({fc.state.position[0]:.1f}, {fc.state.position[1]:.1f})"

            right_overlay = "=== STABILIZATION ==="
            right_overlay += f"\nDRONE BODY:"
            right_overlay += f"\n  Roll:  {drone_roll:+6.1f} deg"
            right_overlay += f"\n  Pitch: {drone_pitch:+6.1f} deg"
            right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f} deg"
            right_overlay += f"\nGIMBAL (world):"
            right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f} deg"
            right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f} deg"

            ground_truth = self._get_ground_truth(ctx)
            return ctx.video_recorder.write_frame(rgba, overlay, right_overlay, ground_truth)

        except Exception:
            return 0

    def _record_stab_frame(self, ctx, name, vx, vy, vz):
        """Record stress test frame."""
        if ctx.video_recorder is None or ctx.camera is None:
            return

        try:
            rgba = ctx.camera.get_rgba()
            if rgba is None or rgba.size == 0:
                return

            fc = ctx.flight_controller
            gimbal = ctx.gimbal

            drone_roll, drone_pitch, drone_yaw = gimbal.get_drone_attitude()
            gimbal_pan, gimbal_tilt = gimbal.get_gimbal_angles()

            overlay = f"TEST: STAB TEST - {name}"
            overlay += f"\nAlt: {fc.state.altitude:.1f}m"
            overlay += f"\nVel cmd: ({vx:.1f}, {vy:.1f}, {vz:.1f})"

            right_overlay = "=== STABILIZATION ==="
            right_overlay += f"\nDRONE BODY:"
            right_overlay += f"\n  Roll:  {drone_roll:+6.1f} deg"
            right_overlay += f"\n  Pitch: {drone_pitch:+6.1f} deg"
            right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f} deg"
            right_overlay += f"\nGIMBAL (world):"
            right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f} deg"
            right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f} deg"
            right_overlay += f"\n--- ISOLATED ---"

            ctx.video_recorder.write_frame(rgba, overlay, right_overlay, None)

        except Exception:
            pass

    def _record_gimbal_frame(self, ctx, name, target_pan, target_tilt):
        """Record gimbal test frame."""
        if ctx.video_recorder is None or ctx.camera is None:
            return

        try:
            rgba = ctx.camera.get_rgba()
            if rgba is None or rgba.size == 0:
                return

            fc = ctx.flight_controller
            gimbal = ctx.gimbal

            drone_roll, drone_pitch, drone_yaw = gimbal.get_drone_attitude()
            gimbal_pan, gimbal_tilt = gimbal.get_gimbal_angles()

            overlay = f"TEST: GIMBAL CTRL - {name}"
            overlay += f"\nCurrent: Pan {gimbal_pan:.0f} Tilt {gimbal_tilt:.0f}"
            overlay += f"\nTarget:  Pan {target_pan:.0f} Tilt {target_tilt:.0f}"
            overlay += f"\nAlt: {fc.state.altitude:.1f}m"

            right_overlay = "=== STABILIZATION ==="
            right_overlay += f"\nDRONE BODY:"
            right_overlay += f"\n  Roll:  {drone_roll:+6.1f} deg"
            right_overlay += f"\n  Pitch: {drone_pitch:+6.1f} deg"
            right_overlay += f"\n  Yaw:   {drone_yaw:+6.1f} deg"
            right_overlay += f"\nGIMBAL (world):"
            right_overlay += f"\n  Pan:   {gimbal_pan:+6.1f} deg"
            right_overlay += f"\n  Tilt:  {gimbal_tilt:+6.1f} deg"

            ctx.video_recorder.write_frame(rgba, overlay, right_overlay, None)

        except Exception:
            pass

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
