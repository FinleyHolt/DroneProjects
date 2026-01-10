"""
PX4 SITL MAVLink Bridge for Isaac Sim.

Provides bidirectional communication between Isaac Sim drone simulation
and PX4 SITL autopilot via MAVLink UDP.

Protocol based on PX4 SITL Gazebo interface.
"""

import socket
import struct
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import numpy as np


@dataclass
class SensorState:
    """Sensor data to send to PX4."""
    # Timestamp (microseconds since start)
    timestamp_us: int = 0

    # IMU data (body frame, NED convention)
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))  # rad/s
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s^2

    # Magnetometer (body frame, Gauss)
    mag: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Barometer
    abs_pressure: float = 101325.0  # Pa
    diff_pressure: float = 0.0  # Pa
    pressure_alt: float = 0.0  # m

    # GPS (WGS84)
    gps_lat: float = 47.397742  # deg
    gps_lon: float = 8.545594  # deg
    gps_alt: float = 488.0  # m MSL
    gps_eph: float = 1.0  # m
    gps_epv: float = 1.5  # m
    gps_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s NED
    gps_fix_type: int = 3  # 3D fix

    # Ground truth (optional, for debugging)
    pos_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m
    vel_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s
    quat: np.ndarray = field(default_factory=lambda: np.array([1, 0, 0, 0]))  # w,x,y,z


@dataclass
class ActuatorCommands:
    """Actuator commands received from PX4."""
    timestamp_us: int = 0

    # Motor commands (normalized 0-1)
    motor: np.ndarray = field(default_factory=lambda: np.zeros(4))

    # Control surface commands (normalized -1 to 1) - not used for quadrotor
    control: np.ndarray = field(default_factory=lambda: np.zeros(8))

    # Arm state
    armed: bool = False

    # Mode
    mode: int = 0


class PX4Bridge:
    """
    MAVLink bridge for PX4 SITL communication.

    Uses the PX4 SITL lockstep protocol:
    1. Simulator sends sensor data
    2. PX4 processes and sends actuator commands
    3. Simulator advances physics
    4. Repeat

    Port mapping for multi-vehicle:
    - Instance 0: recv 9002, send 9003
    - Instance 1: recv 9012, send 9013
    - Instance N: recv 9002 + N*10, send 9003 + N*10
    """

    # HIL Sensor message structure (from PX4 mavlink_hil_sensor_t)
    # Total size: 64 bytes
    HIL_SENSOR_FORMAT = '<QfffffffffffffffffI'
    HIL_SENSOR_SIZE = struct.calcsize(HIL_SENSOR_FORMAT)

    # HIL GPS message structure
    HIL_GPS_FORMAT = '<QiiiiiiiHBBI'
    HIL_GPS_SIZE = struct.calcsize(HIL_GPS_FORMAT)

    # HIL Actuator Controls message structure
    HIL_ACTUATOR_FORMAT = '<Qffffffffffffffff'
    HIL_ACTUATOR_SIZE = struct.calcsize(HIL_ACTUATOR_FORMAT)

    # MAVLink message IDs
    MAVLINK_MSG_ID_HIL_SENSOR = 107
    MAVLINK_MSG_ID_HIL_GPS = 113
    MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS = 93

    def __init__(
        self,
        instance: int = 0,
        px4_host: str = "127.0.0.1",
        local_host: str = "0.0.0.0",
        lockstep: bool = True,
    ):
        """
        Initialize PX4 bridge.

        Args:
            instance: Vehicle instance number (for multi-vehicle)
            px4_host: PX4 SITL host address
            local_host: Local bind address
            lockstep: Enable lockstep synchronization
        """
        self.instance = instance
        self.px4_host = px4_host
        self.local_host = local_host
        self.lockstep = lockstep

        # Port calculation
        self.recv_port = 9002 + instance * 10  # Receive from PX4
        self.send_port = 9003 + instance * 10  # Send to PX4

        # Socket setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setblocking(False)

        # State
        self.connected = False
        self.running = False
        self.last_actuator = ActuatorCommands()
        self.sensor_state = SensorState()

        # Timing
        self.start_time = time.time()
        self.last_sensor_time = 0
        self.last_gps_time = 0

        # Sensor rates
        self.imu_rate = 250  # Hz
        self.gps_rate = 10  # Hz
        self.baro_rate = 50  # Hz

        # Reference position for local NED to GPS conversion
        self.ref_lat = 47.397742  # Zurich area
        self.ref_lon = 8.545594
        self.ref_alt = 488.0

        # Receive thread (for async mode)
        self._recv_thread = None
        self._recv_lock = threading.Lock()

    def connect(self) -> bool:
        """
        Bind socket and prepare for communication.

        Returns:
            True if successful
        """
        try:
            self.socket.bind((self.local_host, self.recv_port))
            self.connected = True
            self.start_time = time.time()
            print(f"PX4Bridge: Bound to {self.local_host}:{self.recv_port}, "
                  f"sending to {self.px4_host}:{self.send_port}")
            return True
        except OSError as e:
            print(f"PX4Bridge: Failed to bind: {e}")
            return False

    def disconnect(self) -> None:
        """Close connection."""
        self.running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        self.socket.close()
        self.connected = False

    def start_async_receive(self) -> None:
        """Start background thread for receiving actuator commands."""
        self.running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def _recv_loop(self) -> None:
        """Background receive loop."""
        while self.running:
            try:
                self._receive_actuators()
            except BlockingIOError:
                time.sleep(0.0001)  # 100us sleep if no data
            except Exception as e:
                print(f"PX4Bridge recv error: {e}")

    def send_sensors(self, state: SensorState) -> None:
        """
        Send sensor data to PX4.

        Args:
            state: Current sensor state
        """
        if not self.connected:
            return

        current_time = time.time() - self.start_time
        timestamp_us = int(current_time * 1e6)

        # Send IMU/magnetometer/barometer at sensor rate
        if current_time - self.last_sensor_time >= 1.0 / self.imu_rate:
            self._send_hil_sensor(timestamp_us, state)
            self.last_sensor_time = current_time

        # Send GPS at lower rate
        if current_time - self.last_gps_time >= 1.0 / self.gps_rate:
            self._send_hil_gps(timestamp_us, state)
            self.last_gps_time = current_time

    def _send_hil_sensor(self, timestamp_us: int, state: SensorState) -> None:
        """Send HIL_SENSOR message."""
        # Pack sensor data
        # Fields: time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro,
        #         xmag, ymag, zmag, abs_pressure, diff_pressure,
        #         pressure_alt, temperature, fields_updated
        msg = struct.pack(
            self.HIL_SENSOR_FORMAT,
            timestamp_us,
            state.accel[0], state.accel[1], state.accel[2],
            state.gyro[0], state.gyro[1], state.gyro[2],
            state.mag[0], state.mag[1], state.mag[2],
            state.abs_pressure,
            state.diff_pressure,
            state.pressure_alt,
            25.0,  # temperature (C)
            0x1FFF,  # All fields updated
        )

        # Send with minimal MAVLink header
        self._send_mavlink(self.MAVLINK_MSG_ID_HIL_SENSOR, msg)

    def _send_hil_gps(self, timestamp_us: int, state: SensorState) -> None:
        """Send HIL_GPS message."""
        # Convert local NED position to GPS coordinates
        lat, lon, alt = self._ned_to_gps(state.pos_ned)

        # Ground speed
        ground_speed = np.sqrt(state.vel_ned[0]**2 + state.vel_ned[1]**2)

        # Course over ground
        cog = np.arctan2(state.vel_ned[1], state.vel_ned[0])
        cog_deg = np.degrees(cog) % 360

        # Pack GPS data
        msg = struct.pack(
            self.HIL_GPS_FORMAT,
            timestamp_us,
            int(lat * 1e7),  # lat (degE7)
            int(lon * 1e7),  # lon (degE7)
            int(alt * 1000),  # alt (mm)
            int(state.gps_eph * 100),  # eph (cm)
            int(state.gps_epv * 100),  # epv (cm)
            int(ground_speed * 100),  # vel (cm/s)
            int(state.vel_ned[2] * 100),  # vd (cm/s)
            int(cog_deg * 100),  # cog (cdeg)
            state.gps_fix_type,
            12,  # satellites visible
            0,  # id
        )

        self._send_mavlink(self.MAVLINK_MSG_ID_HIL_GPS, msg)

    def _send_mavlink(self, msg_id: int, payload: bytes) -> None:
        """Send MAVLink v1 message."""
        # Simplified MAVLink v1 frame
        # Start | Len | Seq | Sys | Comp | Msg ID | Payload | CRC
        header = struct.pack(
            '<BBBBBB',
            0xFE,  # Start byte (MAVLink v1)
            len(payload),
            0,  # Sequence (not tracked)
            1,  # System ID
            1,  # Component ID
            msg_id,
        )

        # CRC (simplified - use X.25 CRC)
        crc = self._mavlink_crc(header[1:] + payload, msg_id)

        message = header + payload + struct.pack('<H', crc)

        try:
            self.socket.sendto(message, (self.px4_host, self.send_port))
        except OSError:
            pass  # Ignore send errors

    @staticmethod
    def _mavlink_crc(data: bytes, msg_id: int) -> int:
        """Calculate MAVLink CRC-16/MCRF4XX."""
        # CRC-CCITT with 0xFFFF initial value
        crc = 0xFFFF

        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp ^= (tmp << 4) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crc &= 0xFFFF

        # Add CRC extra byte (message-specific)
        # This is a simplified version - real implementation uses msg CRC extras
        crc_extra = msg_id & 0xFF
        tmp = crc_extra ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF

        return crc

    def receive_actuators(self, timeout: float = 0.001) -> Optional[ActuatorCommands]:
        """
        Receive actuator commands from PX4.

        Args:
            timeout: Receive timeout in seconds

        Returns:
            ActuatorCommands if received, None otherwise
        """
        if not self.connected:
            return None

        return self._receive_actuators()

    def _receive_actuators(self) -> Optional[ActuatorCommands]:
        """Internal receive implementation."""
        try:
            data, addr = self.socket.recvfrom(1024)
        except BlockingIOError:
            return None
        except OSError:
            return None

        if len(data) < 8:
            return None

        # Parse MAVLink header
        if data[0] != 0xFE:  # MAVLink v1 start
            return None

        payload_len = data[1]
        msg_id = data[5]

        if msg_id == self.MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            return self._parse_actuator_controls(data[6:6+payload_len])

        return None

    def _parse_actuator_controls(self, payload: bytes) -> ActuatorCommands:
        """Parse HIL_ACTUATOR_CONTROLS message."""
        if len(payload) < self.HIL_ACTUATOR_SIZE:
            return self.last_actuator

        # Unpack: time_usec, controls[16], mode, flags
        values = struct.unpack(self.HIL_ACTUATOR_FORMAT, payload[:self.HIL_ACTUATOR_SIZE])

        cmd = ActuatorCommands(
            timestamp_us=values[0],
            motor=np.array(values[1:5]),  # First 4 are motors
            control=np.array(values[1:9]),  # All 8 control channels
            armed=True,  # Assume armed if receiving commands
        )

        with self._recv_lock:
            self.last_actuator = cmd

        return cmd

    def get_last_actuators(self) -> ActuatorCommands:
        """Get most recent actuator commands (thread-safe)."""
        with self._recv_lock:
            return self.last_actuator

    def _ned_to_gps(self, ned: np.ndarray) -> Tuple[float, float, float]:
        """
        Convert local NED position to GPS coordinates.

        Args:
            ned: Position in NED frame [m]

        Returns:
            (latitude, longitude, altitude) in degrees/meters
        """
        # Earth radius approximation
        R = 6371000.0

        # Convert NED to lat/lon/alt
        lat = self.ref_lat + np.degrees(ned[0] / R)
        lon = self.ref_lon + np.degrees(ned[1] / (R * np.cos(np.radians(self.ref_lat))))
        alt = self.ref_alt - ned[2]  # NED Z is down

        return lat, lon, alt

    def set_reference_position(
        self,
        lat: float,
        lon: float,
        alt: float,
    ) -> None:
        """
        Set GPS reference position for NED conversion.

        Args:
            lat: Reference latitude (degrees)
            lon: Reference longitude (degrees)
            alt: Reference altitude MSL (meters)
        """
        self.ref_lat = lat
        self.ref_lon = lon
        self.ref_alt = alt


class IsaacSimPX4Interface:
    """
    High-level interface for connecting Isaac Sim drone to PX4.

    Handles:
    - Sensor data extraction from Isaac Sim
    - Actuator command application to articulation
    - Timing synchronization
    """

    def __init__(
        self,
        bridge: PX4Bridge,
        drone_prim_path: str,
        stage=None,
    ):
        """
        Initialize interface.

        Args:
            bridge: PX4Bridge instance
            drone_prim_path: USD path to drone root
            stage: USD stage (auto-detected if None)
        """
        self.bridge = bridge
        self.drone_path = drone_prim_path
        self.stage = stage

        # Sensor noise parameters
        self.gyro_noise_std = 0.009  # rad/s
        self.accel_noise_std = 0.017  # m/s^2
        self.mag_noise_std = 0.001  # Gauss

        # Reference magnetic field (approximately Zurich)
        self.mag_field_ned = np.array([0.21, 0.01, 0.43])  # Gauss

    def update(
        self,
        position_ned: np.ndarray,
        velocity_ned: np.ndarray,
        quaternion: np.ndarray,
        angular_velocity: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Update sensor data and get motor commands.

        Args:
            position_ned: Position in NED frame [m]
            velocity_ned: Velocity in NED frame [m/s]
            quaternion: Attitude quaternion [w, x, y, z]
            angular_velocity: Angular velocity body frame [rad/s]
            dt: Time step [s]

        Returns:
            Motor commands (normalized 0-1) [4]
        """
        # Build sensor state
        state = SensorState()
        state.timestamp_us = int(time.time() * 1e6)

        # IMU data (add noise for realism)
        state.gyro = angular_velocity + np.random.randn(3) * self.gyro_noise_std

        # Accelerometer includes gravity
        R = self._quaternion_to_rotation_matrix(quaternion)
        gravity_body = R.T @ np.array([0, 0, 9.81])
        state.accel = gravity_body + np.random.randn(3) * self.accel_noise_std

        # Magnetometer
        mag_body = R.T @ self.mag_field_ned
        state.mag = mag_body + np.random.randn(3) * self.mag_noise_std

        # Barometer
        state.pressure_alt = -position_ned[2]  # Convert NED Z to altitude
        state.abs_pressure = 101325.0 * np.exp(-state.pressure_alt / 8500.0)

        # GPS
        state.pos_ned = position_ned.copy()
        state.vel_ned = velocity_ned.copy()

        # Ground truth
        state.quat = quaternion.copy()

        # Send to PX4
        self.bridge.send_sensors(state)

        # Get actuator commands
        cmd = self.bridge.receive_actuators()
        if cmd is not None:
            return cmd.motor
        else:
            return self.bridge.get_last_actuators().motor

    @staticmethod
    def _quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to rotation matrix."""
        w, x, y, z = q
        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ])
