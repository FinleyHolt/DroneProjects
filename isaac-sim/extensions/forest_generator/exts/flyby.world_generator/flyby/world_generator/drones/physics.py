"""
Physics and thrust models for F-11 quadcopter simulation.

Provides:
- ThrustModel: Converts motor commands to forces/torques
- QuadrotorDynamics: Full 6-DOF dynamics simulation
"""

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np


@dataclass
class ThrustModelConfig:
    """Configuration for rotor thrust model."""
    # Thrust coefficient: F = k_thrust * omega^2
    # Tuned for ~2x thrust-to-weight ratio at max throttle
    # For 6.3kg drone: 4 rotors * 30.9N = 123.6N total thrust
    # At max omega ~1400 rad/s: k_thrust = 30.9 / 1400^2 = 1.58e-5
    thrust_coefficient: float = 1.58e-5  # N/(rad/s)^2

    # Torque coefficient: tau = k_torque * omega^2
    # Reaction torque from spinning prop
    torque_coefficient: float = 2.0e-7  # N*m/(rad/s)^2

    # Motor dynamics
    max_omega: float = 1400.0  # rad/s (max angular velocity)
    min_omega: float = 100.0   # rad/s (idle)
    motor_time_constant: float = 0.02  # s (motor response time)

    # Propeller inertia (affects spin-up dynamics)
    prop_inertia: float = 5e-4  # kg*m^2


class ThrustModel:
    """
    Rotor thrust and torque model.

    Converts normalized throttle commands (0-1) to thrust forces and
    reaction torques for each rotor.
    """

    def __init__(self, config: ThrustModelConfig = None):
        self.config = config or ThrustModelConfig()

        # Current rotor velocities (rad/s)
        self.omega = np.zeros(4)

    def reset(self) -> None:
        """Reset rotor velocities to idle."""
        self.omega = np.full(4, self.config.min_omega)

    def update(
        self,
        throttle_commands: np.ndarray,
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update rotor velocities and compute thrust/torque.

        Args:
            throttle_commands: Normalized throttle (0-1) for each rotor [4]
            dt: Time step in seconds

        Returns:
            thrust: Thrust force per rotor [4] in Newtons
            torque: Reaction torque per rotor [4] in N*m
        """
        # Clamp throttle commands
        throttle = np.clip(throttle_commands, 0.0, 1.0)

        # Target angular velocity
        omega_range = self.config.max_omega - self.config.min_omega
        omega_target = self.config.min_omega + throttle * omega_range

        # First-order motor dynamics
        tau = self.config.motor_time_constant
        alpha = dt / (tau + dt)
        self.omega = (1 - alpha) * self.omega + alpha * omega_target

        # Compute thrust: F = k * omega^2
        thrust = self.config.thrust_coefficient * self.omega ** 2

        # Compute reaction torque: tau = k * omega^2
        # Sign depends on spin direction (handled by caller)
        torque = self.config.torque_coefficient * self.omega ** 2

        return thrust, torque

    def get_hover_throttle(self, total_mass: float) -> float:
        """
        Compute throttle needed for hover.

        Args:
            total_mass: Total drone mass in kg

        Returns:
            Normalized throttle (0-1) for hover
        """
        # Required thrust per rotor
        g = 9.81
        thrust_per_rotor = (total_mass * g) / 4

        # omega = sqrt(F / k)
        omega_hover = np.sqrt(thrust_per_rotor / self.config.thrust_coefficient)

        # Convert to normalized throttle
        omega_range = self.config.max_omega - self.config.min_omega
        throttle = (omega_hover - self.config.min_omega) / omega_range

        return np.clip(throttle, 0.0, 1.0)


@dataclass
class QuadrotorState:
    """Full quadrotor state vector."""
    # Position (NED frame, meters)
    position: np.ndarray = None  # [x, y, z]

    # Velocity (NED frame, m/s)
    velocity: np.ndarray = None  # [vx, vy, vz]

    # Attitude quaternion (scalar-first: [w, x, y, z])
    quaternion: np.ndarray = None  # [qw, qx, qy, qz]

    # Angular velocity (body frame, rad/s)
    angular_velocity: np.ndarray = None  # [p, q, r]

    # Rotor velocities (rad/s)
    rotor_omega: np.ndarray = None  # [omega_0, omega_1, omega_2, omega_3]

    def __post_init__(self):
        if self.position is None:
            self.position = np.zeros(3)
        if self.velocity is None:
            self.velocity = np.zeros(3)
        if self.quaternion is None:
            self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        if self.angular_velocity is None:
            self.angular_velocity = np.zeros(3)
        if self.rotor_omega is None:
            self.rotor_omega = np.zeros(4)


class QuadrotorDynamics:
    """
    Full 6-DOF quadrotor dynamics simulation.

    Simulates rigid body dynamics with rotor thrust/torque inputs.
    Uses NED (North-East-Down) coordinate frame.
    """

    # Rotor positions relative to CoG (X-config, NED frame)
    # [x, y, z] in meters
    ROTOR_POSITIONS = np.array([
        [+0.195, -0.195, -0.04],  # FR
        [-0.195, +0.195, -0.04],  # BL
        [+0.195, +0.195, -0.04],  # FL
        [-0.195, -0.195, -0.04],  # BR
    ])

    # Rotor spin directions (1 = CW produces negative torque, -1 = CCW)
    ROTOR_DIRECTIONS = np.array([1, 1, -1, -1])

    def __init__(
        self,
        mass: float = 6.3,
        inertia: Tuple[float, float, float] = (0.115, 0.115, 0.175),
        thrust_model: ThrustModel = None,
    ):
        """
        Initialize quadrotor dynamics.

        Args:
            mass: Total mass in kg
            inertia: Diagonal inertia tensor [Ixx, Iyy, Izz] in kg*m^2
            thrust_model: ThrustModel instance (creates default if None)
        """
        self.mass = mass
        self.inertia = np.diag(inertia)
        self.inertia_inv = np.linalg.inv(self.inertia)

        self.thrust_model = thrust_model or ThrustModel()
        self.state = QuadrotorState()

        # Gravity in NED frame (down is positive Z)
        self.gravity = np.array([0, 0, 9.81])

    def reset(
        self,
        position: np.ndarray = None,
        velocity: np.ndarray = None,
        quaternion: np.ndarray = None,
    ) -> QuadrotorState:
        """
        Reset dynamics to initial state.

        Args:
            position: Initial position [x, y, z] in NED
            velocity: Initial velocity [vx, vy, vz]
            quaternion: Initial attitude [qw, qx, qy, qz]

        Returns:
            Initial state
        """
        self.state = QuadrotorState(
            position=position if position is not None else np.zeros(3),
            velocity=velocity if velocity is not None else np.zeros(3),
            quaternion=quaternion if quaternion is not None else np.array([1, 0, 0, 0]),
        )
        self.thrust_model.reset()
        return self.state

    def step(
        self,
        throttle_commands: np.ndarray,
        dt: float,
    ) -> QuadrotorState:
        """
        Advance simulation by one time step.

        Args:
            throttle_commands: Normalized throttle (0-1) for each rotor [4]
            dt: Time step in seconds

        Returns:
            Updated state
        """
        # Get thrust and torque from rotors
        thrust, reaction_torque = self.thrust_model.update(throttle_commands, dt)

        # Compute total thrust force (body frame, Z-up)
        total_thrust = np.array([0, 0, -np.sum(thrust)])  # Negative because NED

        # Compute body torques
        torque = self._compute_body_torque(thrust, reaction_torque)

        # Integrate dynamics
        self._integrate_dynamics(total_thrust, torque, dt)

        # Update rotor velocities in state
        self.state.rotor_omega = self.thrust_model.omega.copy()

        return self.state

    def _compute_body_torque(
        self,
        thrust: np.ndarray,
        reaction_torque: np.ndarray,
    ) -> np.ndarray:
        """
        Compute total body torque from rotor forces.

        Args:
            thrust: Thrust per rotor [4]
            reaction_torque: Reaction torque per rotor [4]

        Returns:
            Total torque [roll, pitch, yaw] in body frame
        """
        torque = np.zeros(3)

        for i in range(4):
            # Moment arm from CoG to rotor
            r = self.ROTOR_POSITIONS[i]

            # Thrust vector in body frame (negative Z in NED)
            f = np.array([0, 0, -thrust[i]])

            # Moment from thrust: tau = r x F
            torque += np.cross(r, f)

            # Add reaction torque (yaw)
            # CW rotation (direction=1) produces negative yaw torque
            torque[2] += -self.ROTOR_DIRECTIONS[i] * reaction_torque[i]

        return torque

    def _integrate_dynamics(
        self,
        thrust_body: np.ndarray,
        torque_body: np.ndarray,
        dt: float,
    ) -> None:
        """
        Integrate rigid body dynamics using RK4.

        Args:
            thrust_body: Total thrust in body frame
            torque_body: Total torque in body frame
            dt: Time step
        """
        # For simplicity, use semi-implicit Euler
        # (RK4 would be more accurate but slower)

        # Rotation matrix from body to world (NED)
        R = self._quaternion_to_rotation_matrix(self.state.quaternion)

        # Linear dynamics
        # F_world = R @ F_body + m * g
        force_world = R @ thrust_body + self.mass * self.gravity

        # Acceleration
        accel = force_world / self.mass

        # Update velocity and position
        self.state.velocity += accel * dt
        self.state.position += self.state.velocity * dt

        # Angular dynamics
        # tau = I @ omega_dot + omega x (I @ omega)
        omega = self.state.angular_velocity
        omega_dot = self.inertia_inv @ (
            torque_body - np.cross(omega, self.inertia @ omega)
        )

        # Update angular velocity
        self.state.angular_velocity += omega_dot * dt

        # Update quaternion
        self.state.quaternion = self._integrate_quaternion(
            self.state.quaternion, omega, dt
        )

    @staticmethod
    def _quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to rotation matrix."""
        w, x, y, z = q
        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ])

    @staticmethod
    def _integrate_quaternion(
        q: np.ndarray,
        omega: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Integrate quaternion with angular velocity."""
        # Quaternion derivative: q_dot = 0.5 * q * omega_quat
        omega_quat = np.array([0, omega[0], omega[1], omega[2]])

        # Quaternion multiplication
        w, x, y, z = q
        ow, ox, oy, oz = omega_quat

        q_dot = 0.5 * np.array([
            w*ow - x*ox - y*oy - z*oz,
            w*ox + x*ow + y*oz - z*oy,
            w*oy - x*oz + y*ow + z*ox,
            w*oz + x*oy - y*ox + z*ow,
        ])

        # Euler integration
        q_new = q + q_dot * dt

        # Normalize
        q_new = q_new / np.linalg.norm(q_new)

        return q_new


class MotorMixingMatrix:
    """
    Motor mixing matrix for converting control inputs to motor commands.

    Converts desired [thrust, roll, pitch, yaw] to individual motor throttles.
    """

    # X-configuration mixing matrix
    # Columns: [thrust, roll, pitch, yaw]
    # Rows: [motor_0, motor_1, motor_2, motor_3]
    # Motor order: FR(CW), BL(CW), FL(CCW), BR(CCW)
    MIXING_MATRIX = np.array([
        [1,  1, -1, -1],  # FR: + roll, - pitch, - yaw (CW)
        [1, -1,  1, -1],  # BL: - roll, + pitch, - yaw (CW)
        [1, -1, -1,  1],  # FL: - roll, - pitch, + yaw (CCW)
        [1,  1,  1,  1],  # BR: + roll, + pitch, + yaw (CCW)
    ])

    @classmethod
    def mix(
        cls,
        thrust: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> np.ndarray:
        """
        Mix control inputs to motor commands.

        Args:
            thrust: Normalized thrust (0-1)
            roll: Roll rate command (-1 to 1)
            pitch: Pitch rate command (-1 to 1)
            yaw: Yaw rate command (-1 to 1)

        Returns:
            Motor throttle commands [4] normalized 0-1
        """
        inputs = np.array([thrust, roll, pitch, yaw])
        outputs = cls.MIXING_MATRIX @ inputs

        # Normalize to 0-1 range
        outputs = np.clip(outputs, 0.0, 1.0)

        return outputs

    @classmethod
    def inverse_mix(cls, motor_commands: np.ndarray) -> Tuple[float, float, float, float]:
        """
        Inverse mixing: motor commands to control inputs.

        Args:
            motor_commands: Motor throttle commands [4]

        Returns:
            (thrust, roll, pitch, yaw) tuple
        """
        # Pseudo-inverse for overdetermined system
        mixing_inv = np.linalg.pinv(cls.MIXING_MATRIX)
        inputs = mixing_inv @ motor_commands

        return tuple(inputs)
