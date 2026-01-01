"""
Action bridge: Convert RL agent actions to PX4 MAVLink commands.

Coordinate Frame Handling:
- Isaac Sim uses Z-up (ENU-like): +X=East, +Y=North, +Z=Up
- PX4 MAVLink uses NED: +X=North, +Y=East, +Z=Down
- RL actions are in Isaac Sim frame (Z-up) by default
- This bridge converts to PX4 NED before sending

When z_up_input=True (default):
- Input vz positive = ascend (Z-up convention)
- Output vz positive = descend (NED convention)
- Conversion: negate vz for MAVLink

When z_up_input=False:
- Input already in NED, no conversion needed
"""
import numpy as np
from typing import Optional, Tuple


class PX4ActionBridge:
    """
    Bridges RL agent actions to PX4 velocity commands.

    Supports:
    - Velocity setpoints (vx, vy, vz, yaw_rate)
    - Position setpoints (x, y, z, yaw)
    - Safety limiting (max velocity, geofence)
    - Coordinate frame conversion (Z-up to NED)
    """

    def __init__(self, mavlink_connection=None, z_up_input: bool = True):
        """
        Initialize action bridge.

        Args:
            mavlink_connection: Optional pymavlink connection. Can be set later
                               via set_mavlink_connection().
            z_up_input: If True, input velocities use Z-up convention (Isaac Sim).
                       If False, input is already NED (no conversion needed).
        """
        self.mav = mavlink_connection
        self.z_up_input = z_up_input

        # F-11 safety limits
        self.max_horizontal_vel = 6.3  # m/s (F-11 cruise speed)
        self.max_vertical_vel = 3.0    # m/s
        self.max_yaw_rate = 1.0        # rad/s

    def set_mavlink_connection(self, mav):
        """Set MAVLink connection (called after PX4 connects)."""
        self.mav = mav

    def send_velocity_command(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float
    ) -> bool:
        """
        Send velocity setpoint to PX4.

        Args:
            vx: Forward velocity (m/s, body frame)
            vy: Right velocity (m/s, body frame)
            vz: Vertical velocity (m/s). Interpretation depends on z_up_input:
                - If z_up_input=True: positive = ascend (Z-up), will be negated for NED
                - If z_up_input=False: positive = descend (NED), sent as-is
            yaw_rate: Yaw rate (rad/s)

        Returns:
            True if command sent successfully
        """
        if self.mav is None:
            return False

        # Convert Z-up to NED if needed (negate vz)
        # Z-up: positive vz = ascend
        # NED:  positive vz = descend
        if self.z_up_input:
            vz_ned = -vz
        else:
            vz_ned = vz

        # Apply safety limits
        vx = np.clip(vx, -self.max_horizontal_vel, self.max_horizontal_vel)
        vy = np.clip(vy, -self.max_horizontal_vel, self.max_horizontal_vel)
        vz_ned = np.clip(vz_ned, -self.max_vertical_vel, self.max_vertical_vel)
        yaw_rate = np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

        try:
            from pymavlink import mavutil

            # Send SET_POSITION_TARGET_LOCAL_NED with velocity mask
            # type_mask: ignore position, use velocity
            # Bits: 0b0000011111000111
            #   - Bit 0-2: ignore position
            #   - Bit 6-8: ignore acceleration
            #   - Bit 9-10: ignore yaw, use yaw_rate
            type_mask = 0b0000011111000111

            self.mav.mav.set_position_target_local_ned_send(
                0,                      # time_boot_ms
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                type_mask,
                0, 0, 0,               # position (ignored)
                vx, vy, vz_ned,        # velocity (NED frame)
                0, 0, 0,               # acceleration (ignored)
                0,                      # yaw (ignored)
                yaw_rate                # yaw_rate
            )
            return True
        except Exception as e:
            print(f"MAVLink send error: {e}")
            return False

    def send_position_command(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float
    ) -> bool:
        """
        Send position setpoint to PX4.

        Args:
            x: North position (m, local frame)
            y: East position (m, local frame)
            z: Down position (m, NED frame, negative = up)
            yaw: Heading (rad)

        Returns:
            True if command sent successfully
        """
        if self.mav is None:
            return False

        try:
            from pymavlink import mavutil

            # type_mask: use position, ignore velocity/acceleration
            type_mask = 0b0000111111111000

            self.mav.mav.set_position_target_local_ned_send(
                0,                      # time_boot_ms
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x, y, z,               # position
                0, 0, 0,               # velocity (ignored)
                0, 0, 0,               # acceleration (ignored)
                yaw,                    # yaw
                0                       # yaw_rate (ignored)
            )
            return True
        except Exception as e:
            print(f"MAVLink send error: {e}")
            return False

    def action_to_velocity(self, action: np.ndarray) -> Tuple[float, float, float, float]:
        """
        Convert normalized action [-1, 1] to velocity command.

        Args:
            action: [vx, vy, vz, yaw_rate] in [-1, 1] range

        Returns:
            (vx, vy, vz, yaw_rate) in physical units (m/s, rad/s)
        """
        # Ensure action is numpy array
        action = np.asarray(action, dtype=np.float32)

        # Clip to [-1, 1] range for safety
        action = np.clip(action, -1.0, 1.0)

        vx = float(action[0] * self.max_horizontal_vel)
        vy = float(action[1] * self.max_horizontal_vel)
        vz = float(action[2] * self.max_vertical_vel)
        yaw_rate = float(action[3] * self.max_yaw_rate)

        return vx, vy, vz, yaw_rate

    def velocity_to_action(self, vx: float, vy: float, vz: float, yaw_rate: float) -> np.ndarray:
        """
        Convert velocity command to normalized action.

        Args:
            vx: Forward velocity (m/s)
            vy: Right velocity (m/s)
            vz: Vertical velocity (m/s)
            yaw_rate: Yaw rate (rad/s)

        Returns:
            Normalized action in [-1, 1] range
        """
        return np.array([
            vx / self.max_horizontal_vel,
            vy / self.max_horizontal_vel,
            vz / self.max_vertical_vel,
            yaw_rate / self.max_yaw_rate
        ], dtype=np.float32).clip(-1.0, 1.0)

    def execute_action(self, action: np.ndarray) -> bool:
        """
        Execute normalized action through MAVLink.

        Args:
            action: [vx, vy, vz, yaw_rate] in [-1, 1] range

        Returns:
            True if command sent successfully
        """
        vx, vy, vz, yaw_rate = self.action_to_velocity(action)
        return self.send_velocity_command(vx, vy, vz, yaw_rate)

    def hover(self) -> bool:
        """
        Command UAV to hover in place.

        Returns:
            True if command sent successfully
        """
        return self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
