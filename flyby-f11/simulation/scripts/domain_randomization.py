#!/usr/bin/env python3
"""
Flyby F-11 Domain Randomization Module

Implements domain randomization techniques for sim-to-real transfer:
- Physics randomization (mass, drag, motor characteristics)
- Sensor noise injection (IMU, GPS, barometer)
- Visual randomization (lighting, textures - for future camera integration)
- Environment randomization (wind, initial conditions)

Based on 2025 research findings:
- "One Net to Rule Them All" (April 2025) - Optimal 10-30% randomization range
- "What Matters in Zero-Shot Sim-to-Real" (May 2025) - Selective randomization

Usage:
    from domain_randomization import DomainRandomization
    dr = DomainRandomization(config_path='configs/rl_training.yaml')
    dr.apply_randomization(env)

References:
    - https://arxiv.org/abs/2504.21586 (One Net to Rule Them All)
    - https://lilianweng.github.io/posts/2019-05-05-domain-randomization/
"""

import logging
import math
import random
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import numpy as np
import yaml

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RandomizationMode(Enum):
    """Randomization intensity modes."""
    NONE = "none"           # No randomization (debugging)
    CONSERVATIVE = "conservative"  # 10% variation
    MODERATE = "moderate"   # 20% variation
    AGGRESSIVE = "aggressive"  # 30% variation
    CUSTOM = "custom"       # User-defined ranges


@dataclass
class PhysicsParams:
    """Physical parameters subject to randomization."""
    mass_kg: float = 1.5
    inertia_xx: float = 0.029
    inertia_yy: float = 0.029
    inertia_zz: float = 0.055
    drag_coefficient_xy: float = 0.5
    drag_coefficient_z: float = 0.3
    motor_constant: float = 8.54858e-06
    motor_time_constant: float = 0.02
    propeller_diameter: float = 0.254
    arm_length: float = 0.23
    gravity: float = 9.81


@dataclass
class SensorNoiseParams:
    """Sensor noise parameters."""
    # IMU Accelerometer
    imu_accel_noise_std: float = 0.05  # m/s^2
    imu_accel_bias_std: float = 0.01   # m/s^2
    imu_accel_bias_correlation_time: float = 300.0  # seconds

    # IMU Gyroscope
    imu_gyro_noise_std: float = 0.005  # rad/s
    imu_gyro_bias_std: float = 0.001   # rad/s
    imu_gyro_bias_correlation_time: float = 300.0  # seconds

    # GPS
    gps_position_noise_std: float = 1.0  # meters
    gps_velocity_noise_std: float = 0.1  # m/s
    gps_update_rate: float = 5.0  # Hz

    # Barometer
    baro_noise_std: float = 0.5  # meters
    baro_drift_rate: float = 0.01  # m/s

    # Magnetometer
    mag_noise_std: float = 0.1  # Gauss
    mag_declination_error: float = 0.05  # radians


@dataclass
class EnvironmentParams:
    """Environmental parameters."""
    wind_velocity_x: float = 0.0  # m/s
    wind_velocity_y: float = 0.0  # m/s
    wind_velocity_z: float = 0.0  # m/s
    wind_gust_magnitude: float = 0.0  # m/s
    wind_gust_frequency: float = 0.1  # Hz
    air_density: float = 1.225  # kg/m^3
    temperature_c: float = 20.0


@dataclass
class VisualParams:
    """Visual randomization parameters (for camera-based learning)."""
    lighting_intensity: float = 1.0
    lighting_color_r: float = 1.0
    lighting_color_g: float = 1.0
    lighting_color_b: float = 1.0
    shadow_intensity: float = 0.5
    fog_density: float = 0.0
    camera_fov_deg: float = 90.0
    camera_exposure: float = 1.0


class DomainRandomization:
    """
    Domain randomization manager for UAV simulation.

    Applies configurable randomization to physics, sensors, environment,
    and visual parameters to improve sim-to-real transfer.
    """

    def __init__(
        self,
        config_path: Optional[str] = None,
        config_dict: Optional[Dict] = None,
        mode: RandomizationMode = RandomizationMode.MODERATE,
        seed: Optional[int] = None,
    ):
        """
        Initialize domain randomization.

        Args:
            config_path: Path to YAML configuration file
            config_dict: Configuration dictionary (overrides file)
            mode: Randomization intensity mode
            seed: Random seed for reproducibility
        """
        self.mode = mode
        self.config = self._load_config(config_path, config_dict)

        # Set random seed
        if seed is not None:
            np.random.seed(seed)
            random.seed(seed)

        # Default (nominal) parameters
        self.nominal_physics = PhysicsParams()
        self.nominal_sensors = SensorNoiseParams()
        self.nominal_environment = EnvironmentParams()
        self.nominal_visual = VisualParams()

        # Current randomized parameters
        self.current_physics = PhysicsParams()
        self.current_sensors = SensorNoiseParams()
        self.current_environment = EnvironmentParams()
        self.current_visual = VisualParams()

        # Randomization ranges (percentage variation)
        self._setup_ranges()

        # Sensor noise state (for correlated noise)
        self._sensor_noise_state = {}

        # Curriculum learning support
        self.curriculum_progress = 0.0  # 0.0 to 1.0

        logger.info(f"Domain randomization initialized (mode: {mode.value})")

    def _load_config(
        self,
        config_path: Optional[str],
        config_dict: Optional[Dict]
    ) -> Dict:
        """Load configuration from file or dictionary."""
        config = {}

        if config_path and Path(config_path).exists():
            with open(config_path, 'r') as f:
                full_config = yaml.safe_load(f)
                config = full_config.get('domain_randomization', {})
            logger.info(f"Loaded domain randomization config from {config_path}")

        if config_dict:
            config.update(config_dict)

        return config

    def _setup_ranges(self):
        """Set up randomization ranges based on mode."""
        # Get mode-specific multiplier
        mode_multipliers = {
            RandomizationMode.NONE: 0.0,
            RandomizationMode.CONSERVATIVE: 0.10,  # 10%
            RandomizationMode.MODERATE: 0.20,      # 20%
            RandomizationMode.AGGRESSIVE: 0.30,    # 30%
        }

        if self.mode == RandomizationMode.CUSTOM:
            # Use config-defined ranges
            self.ranges = self.config.get('ranges', {})
        else:
            mult = mode_multipliers.get(self.mode, 0.0)
            self.ranges = self._default_ranges(mult)

    def _default_ranges(self, mult: float) -> Dict:
        """Generate default ranges with given multiplier."""
        return {
            'physics': {
                'mass_kg': (1.0 - mult, 1.0 + mult),
                'inertia': (1.0 - mult, 1.0 + mult),
                'drag_coefficient': (1.0 - mult*2, 1.0 + mult*2),  # More variation
                'motor_constant': (1.0 - mult*0.5, 1.0 + mult*0.5),  # Less variation
                'motor_time_constant': (1.0 - mult, 1.0 + mult),
                'arm_length': (1.0 - mult*0.1, 1.0 + mult*0.1),  # Small variation
            },
            'sensors': {
                'imu_accel_noise': (0.5, 2.0 if mult > 0 else 1.0),
                'imu_accel_bias': (0.5, 2.0 if mult > 0 else 1.0),
                'imu_gyro_noise': (0.5, 2.0 if mult > 0 else 1.0),
                'imu_gyro_bias': (0.5, 2.0 if mult > 0 else 1.0),
                'gps_position_noise': (0.5, 3.0 if mult > 0 else 1.0),
                'gps_velocity_noise': (0.5, 2.0 if mult > 0 else 1.0),
                'baro_noise': (0.5, 2.0 if mult > 0 else 1.0),
            },
            'environment': {
                'wind_max': 5.0 * mult if mult > 0 else 0.0,  # Max wind speed
                'wind_gust_max': 3.0 * mult if mult > 0 else 0.0,
                'air_density': (1.0 - mult*0.1, 1.0 + mult*0.1),
            },
            'visual': {
                'lighting_intensity': (1.0 - mult, 1.0 + mult),
                'shadow_intensity': (0.2, 0.8) if mult > 0 else (0.5, 0.5),
                'fog_density': (0.0, 0.1 * mult) if mult > 0 else (0.0, 0.0),
                'camera_fov': (1.0 - mult*0.05, 1.0 + mult*0.05),
            },
        }

    def randomize_physics(self) -> PhysicsParams:
        """
        Randomize physical parameters.

        Returns:
            Randomized PhysicsParams
        """
        ranges = self.ranges.get('physics', {})

        # Apply curriculum scaling
        scale = self._get_curriculum_scale()

        self.current_physics = PhysicsParams(
            mass_kg=self.nominal_physics.mass_kg * self._sample_range(
                ranges.get('mass_kg', (1.0, 1.0)), scale
            ),
            inertia_xx=self.nominal_physics.inertia_xx * self._sample_range(
                ranges.get('inertia', (1.0, 1.0)), scale
            ),
            inertia_yy=self.nominal_physics.inertia_yy * self._sample_range(
                ranges.get('inertia', (1.0, 1.0)), scale
            ),
            inertia_zz=self.nominal_physics.inertia_zz * self._sample_range(
                ranges.get('inertia', (1.0, 1.0)), scale
            ),
            drag_coefficient_xy=self.nominal_physics.drag_coefficient_xy * self._sample_range(
                ranges.get('drag_coefficient', (1.0, 1.0)), scale
            ),
            drag_coefficient_z=self.nominal_physics.drag_coefficient_z * self._sample_range(
                ranges.get('drag_coefficient', (1.0, 1.0)), scale
            ),
            motor_constant=self.nominal_physics.motor_constant * self._sample_range(
                ranges.get('motor_constant', (1.0, 1.0)), scale
            ),
            motor_time_constant=self.nominal_physics.motor_time_constant * self._sample_range(
                ranges.get('motor_time_constant', (1.0, 1.0)), scale
            ),
            arm_length=self.nominal_physics.arm_length * self._sample_range(
                ranges.get('arm_length', (1.0, 1.0)), scale
            ),
        )

        return self.current_physics

    def randomize_sensors(self) -> SensorNoiseParams:
        """
        Randomize sensor noise parameters.

        Returns:
            Randomized SensorNoiseParams
        """
        ranges = self.ranges.get('sensors', {})
        scale = self._get_curriculum_scale()

        self.current_sensors = SensorNoiseParams(
            imu_accel_noise_std=self.nominal_sensors.imu_accel_noise_std * self._sample_range(
                ranges.get('imu_accel_noise', (1.0, 1.0)), scale
            ),
            imu_accel_bias_std=self.nominal_sensors.imu_accel_bias_std * self._sample_range(
                ranges.get('imu_accel_bias', (1.0, 1.0)), scale
            ),
            imu_gyro_noise_std=self.nominal_sensors.imu_gyro_noise_std * self._sample_range(
                ranges.get('imu_gyro_noise', (1.0, 1.0)), scale
            ),
            imu_gyro_bias_std=self.nominal_sensors.imu_gyro_bias_std * self._sample_range(
                ranges.get('imu_gyro_bias', (1.0, 1.0)), scale
            ),
            gps_position_noise_std=self.nominal_sensors.gps_position_noise_std * self._sample_range(
                ranges.get('gps_position_noise', (1.0, 1.0)), scale
            ),
            gps_velocity_noise_std=self.nominal_sensors.gps_velocity_noise_std * self._sample_range(
                ranges.get('gps_velocity_noise', (1.0, 1.0)), scale
            ),
            baro_noise_std=self.nominal_sensors.baro_noise_std * self._sample_range(
                ranges.get('baro_noise', (1.0, 1.0)), scale
            ),
        )

        # Reset sensor noise state
        self._reset_sensor_noise_state()

        return self.current_sensors

    def randomize_environment(self) -> EnvironmentParams:
        """
        Randomize environmental parameters (wind, air density).

        Returns:
            Randomized EnvironmentParams
        """
        ranges = self.ranges.get('environment', {})
        scale = self._get_curriculum_scale()

        # Wind direction and magnitude
        max_wind = ranges.get('wind_max', 0.0) * scale
        wind_direction = np.random.uniform(0, 2 * np.pi)
        wind_speed = np.random.uniform(0, max_wind)

        # Wind gust
        max_gust = ranges.get('wind_gust_max', 0.0) * scale

        self.current_environment = EnvironmentParams(
            wind_velocity_x=wind_speed * np.cos(wind_direction),
            wind_velocity_y=wind_speed * np.sin(wind_direction),
            wind_velocity_z=np.random.uniform(-max_wind * 0.2, max_wind * 0.2),
            wind_gust_magnitude=np.random.uniform(0, max_gust),
            wind_gust_frequency=np.random.uniform(0.05, 0.2),
            air_density=self.nominal_environment.air_density * self._sample_range(
                ranges.get('air_density', (1.0, 1.0)), scale
            ),
        )

        return self.current_environment

    def randomize_visual(self) -> VisualParams:
        """
        Randomize visual parameters (for camera-based learning).

        Returns:
            Randomized VisualParams
        """
        ranges = self.ranges.get('visual', {})
        scale = self._get_curriculum_scale()

        # Random lighting color (slight tint)
        base_color = 1.0
        color_variation = 0.1 * scale

        self.current_visual = VisualParams(
            lighting_intensity=self._sample_range(
                ranges.get('lighting_intensity', (1.0, 1.0)), scale
            ),
            lighting_color_r=base_color + np.random.uniform(-color_variation, color_variation),
            lighting_color_g=base_color + np.random.uniform(-color_variation, color_variation),
            lighting_color_b=base_color + np.random.uniform(-color_variation, color_variation),
            shadow_intensity=self._sample_range(
                ranges.get('shadow_intensity', (0.5, 0.5)), scale
            ),
            fog_density=self._sample_range(
                ranges.get('fog_density', (0.0, 0.0)), scale
            ),
            camera_fov_deg=self.nominal_visual.camera_fov_deg * self._sample_range(
                ranges.get('camera_fov', (1.0, 1.0)), scale
            ),
        )

        return self.current_visual

    def apply_randomization(self, env: Any) -> Dict[str, Any]:
        """
        Apply all randomizations to environment.

        Args:
            env: Environment instance (FlybyUAVEnv or similar)

        Returns:
            Dictionary of applied randomization parameters
        """
        applied = {}

        # Check which randomization types are enabled
        enabled = self.config.get('enabled', {
            'physics': True,
            'sensors': True,
            'environment': True,
            'visual': False,
        })

        if enabled.get('physics', True):
            physics = self.randomize_physics()
            applied['physics'] = self._physics_to_dict(physics)

        if enabled.get('sensors', True):
            sensors = self.randomize_sensors()
            applied['sensors'] = self._sensors_to_dict(sensors)

        if enabled.get('environment', True):
            environment = self.randomize_environment()
            applied['environment'] = self._environment_to_dict(environment)

        if enabled.get('visual', False):
            visual = self.randomize_visual()
            applied['visual'] = self._visual_to_dict(visual)

        # Log applied randomization
        if self.config.get('verbose', False):
            logger.info(f"Applied randomization: {applied}")

        return applied

    def add_sensor_noise(
        self,
        measurement: np.ndarray,
        sensor_type: str,
        dt: float = 0.01
    ) -> np.ndarray:
        """
        Add sensor noise to measurement.

        Args:
            measurement: Original measurement value
            sensor_type: Type of sensor ('imu_accel', 'imu_gyro', 'gps', 'baro')
            dt: Time step (for bias evolution)

        Returns:
            Noisy measurement
        """
        noisy = measurement.copy()

        if sensor_type == 'imu_accel':
            # Gaussian noise + bias
            noise = np.random.normal(
                0, self.current_sensors.imu_accel_noise_std, measurement.shape
            )
            bias = self._get_sensor_bias('imu_accel', measurement.shape, dt)
            noisy = measurement + noise + bias

        elif sensor_type == 'imu_gyro':
            noise = np.random.normal(
                0, self.current_sensors.imu_gyro_noise_std, measurement.shape
            )
            bias = self._get_sensor_bias('imu_gyro', measurement.shape, dt)
            noisy = measurement + noise + bias

        elif sensor_type == 'gps_position':
            noise = np.random.normal(
                0, self.current_sensors.gps_position_noise_std, measurement.shape
            )
            noisy = measurement + noise

        elif sensor_type == 'gps_velocity':
            noise = np.random.normal(
                0, self.current_sensors.gps_velocity_noise_std, measurement.shape
            )
            noisy = measurement + noise

        elif sensor_type == 'baro':
            noise = np.random.normal(0, self.current_sensors.baro_noise_std)
            drift = self._get_sensor_drift('baro', dt)
            noisy = measurement + noise + drift

        return noisy

    def get_wind_perturbation(self, time: float) -> np.ndarray:
        """
        Get wind perturbation at current time.

        Args:
            time: Current simulation time

        Returns:
            Wind velocity perturbation [vx, vy, vz]
        """
        env = self.current_environment

        # Base wind
        wind = np.array([
            env.wind_velocity_x,
            env.wind_velocity_y,
            env.wind_velocity_z
        ])

        # Add gust (sinusoidal variation)
        if env.wind_gust_magnitude > 0:
            gust_phase = np.random.uniform(0, 2 * np.pi)
            gust = env.wind_gust_magnitude * np.sin(
                2 * np.pi * env.wind_gust_frequency * time + gust_phase
            )
            # Apply gust in random direction
            gust_direction = np.random.uniform(0, 2 * np.pi)
            wind[0] += gust * np.cos(gust_direction)
            wind[1] += gust * np.sin(gust_direction)

        return wind

    def set_curriculum_progress(self, progress: float):
        """
        Set curriculum learning progress.

        Args:
            progress: Progress value from 0.0 (easy) to 1.0 (full difficulty)
        """
        self.curriculum_progress = np.clip(progress, 0.0, 1.0)
        logger.debug(f"Curriculum progress set to {self.curriculum_progress:.2f}")

    def _get_curriculum_scale(self) -> float:
        """Get current curriculum scaling factor."""
        if self.config.get('use_curriculum', False):
            # Linear scaling from 0.3 to 1.0
            return 0.3 + 0.7 * self.curriculum_progress
        return 1.0

    def _sample_range(
        self,
        range_tuple: Tuple[float, float],
        scale: float = 1.0
    ) -> float:
        """
        Sample uniformly from a range, scaled by curriculum.

        Args:
            range_tuple: (min, max) range
            scale: Curriculum scaling factor

        Returns:
            Sampled value
        """
        low, high = range_tuple

        # Apply curriculum scaling (reduce variation for early training)
        center = (low + high) / 2
        half_width = (high - low) / 2
        scaled_half_width = half_width * scale

        return np.random.uniform(
            center - scaled_half_width,
            center + scaled_half_width
        )

    def _get_sensor_bias(
        self,
        sensor_type: str,
        shape: Tuple,
        dt: float
    ) -> np.ndarray:
        """Get evolving sensor bias (random walk)."""
        key = f'{sensor_type}_bias'

        # Initialize if not exists
        if key not in self._sensor_noise_state:
            self._sensor_noise_state[key] = np.zeros(shape)

        # Get appropriate bias parameters
        if sensor_type == 'imu_accel':
            bias_std = self.current_sensors.imu_accel_bias_std
            tau = self.current_sensors.imu_accel_bias_correlation_time
        elif sensor_type == 'imu_gyro':
            bias_std = self.current_sensors.imu_gyro_bias_std
            tau = self.current_sensors.imu_gyro_bias_correlation_time
        else:
            return np.zeros(shape)

        # Evolve bias (first-order Gauss-Markov process)
        alpha = 1.0 - dt / tau
        noise = np.random.normal(0, bias_std * np.sqrt(dt / tau), shape)
        self._sensor_noise_state[key] = alpha * self._sensor_noise_state[key] + noise

        return self._sensor_noise_state[key]

    def _get_sensor_drift(self, sensor_type: str, dt: float) -> float:
        """Get cumulative sensor drift."""
        key = f'{sensor_type}_drift'

        if key not in self._sensor_noise_state:
            self._sensor_noise_state[key] = 0.0

        if sensor_type == 'baro':
            drift_rate = self.current_sensors.baro_drift_rate
        else:
            return 0.0

        # Random walk drift
        self._sensor_noise_state[key] += drift_rate * dt * np.random.choice([-1, 1])

        return self._sensor_noise_state[key]

    def _reset_sensor_noise_state(self):
        """Reset sensor noise state for new episode."""
        self._sensor_noise_state = {}

    def _physics_to_dict(self, params: PhysicsParams) -> Dict:
        """Convert PhysicsParams to dictionary."""
        return {
            'mass_kg': params.mass_kg,
            'inertia_xx': params.inertia_xx,
            'inertia_yy': params.inertia_yy,
            'inertia_zz': params.inertia_zz,
            'drag_xy': params.drag_coefficient_xy,
            'drag_z': params.drag_coefficient_z,
            'motor_constant': params.motor_constant,
            'motor_time_constant': params.motor_time_constant,
        }

    def _sensors_to_dict(self, params: SensorNoiseParams) -> Dict:
        """Convert SensorNoiseParams to dictionary."""
        return {
            'imu_accel_noise': params.imu_accel_noise_std,
            'imu_gyro_noise': params.imu_gyro_noise_std,
            'gps_position_noise': params.gps_position_noise_std,
            'baro_noise': params.baro_noise_std,
        }

    def _environment_to_dict(self, params: EnvironmentParams) -> Dict:
        """Convert EnvironmentParams to dictionary."""
        return {
            'wind_x': params.wind_velocity_x,
            'wind_y': params.wind_velocity_y,
            'wind_z': params.wind_velocity_z,
            'wind_gust': params.wind_gust_magnitude,
            'air_density': params.air_density,
        }

    def _visual_to_dict(self, params: VisualParams) -> Dict:
        """Convert VisualParams to dictionary."""
        return {
            'lighting': params.lighting_intensity,
            'shadow': params.shadow_intensity,
            'fog': params.fog_density,
            'camera_fov': params.camera_fov_deg,
        }

    def get_summary(self) -> str:
        """Get human-readable summary of current randomization."""
        return (
            f"Domain Randomization Summary (mode: {self.mode.value}):\n"
            f"  Curriculum progress: {self.curriculum_progress:.1%}\n"
            f"  Physics:\n"
            f"    Mass: {self.current_physics.mass_kg:.3f} kg "
            f"({100*(self.current_physics.mass_kg/self.nominal_physics.mass_kg - 1):+.1f}%)\n"
            f"    Drag XY: {self.current_physics.drag_coefficient_xy:.3f}\n"
            f"  Sensors:\n"
            f"    IMU accel noise: {self.current_sensors.imu_accel_noise_std:.4f} m/s^2\n"
            f"    GPS noise: {self.current_sensors.gps_position_noise_std:.2f} m\n"
            f"  Environment:\n"
            f"    Wind: ({self.current_environment.wind_velocity_x:.1f}, "
            f"{self.current_environment.wind_velocity_y:.1f}) m/s\n"
            f"    Gust: {self.current_environment.wind_gust_magnitude:.1f} m/s\n"
        )


class AdaptiveDomainRandomization(DomainRandomization):
    """
    Adaptive domain randomization that adjusts based on training performance.

    Implements automatic curriculum where randomization increases as the
    agent improves, based on success rate or reward metrics.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Performance tracking
        self.episode_rewards: List[float] = []
        self.episode_successes: List[bool] = []
        self.window_size = 100

        # Adaptive thresholds
        self.success_threshold_increase = 0.8  # Increase difficulty if success > 80%
        self.success_threshold_decrease = 0.3  # Decrease difficulty if success < 30%
        self.adaptation_rate = 0.05

    def update_performance(self, reward: float, success: bool):
        """
        Update performance tracking.

        Args:
            reward: Episode reward
            success: Whether episode was successful
        """
        self.episode_rewards.append(reward)
        self.episode_successes.append(success)

        # Keep window size
        if len(self.episode_rewards) > self.window_size:
            self.episode_rewards.pop(0)
            self.episode_successes.pop(0)

        # Adapt curriculum
        self._adapt_curriculum()

    def _adapt_curriculum(self):
        """Adapt curriculum based on recent performance."""
        if len(self.episode_successes) < self.window_size // 2:
            return  # Not enough data

        success_rate = sum(self.episode_successes) / len(self.episode_successes)

        if success_rate > self.success_threshold_increase:
            # Agent is doing well, increase difficulty
            new_progress = min(1.0, self.curriculum_progress + self.adaptation_rate)
            if new_progress > self.curriculum_progress:
                logger.info(
                    f"Increasing curriculum difficulty: {self.curriculum_progress:.2f} -> "
                    f"{new_progress:.2f} (success rate: {success_rate:.2%})"
                )
                self.curriculum_progress = new_progress

        elif success_rate < self.success_threshold_decrease:
            # Agent is struggling, decrease difficulty
            new_progress = max(0.0, self.curriculum_progress - self.adaptation_rate)
            if new_progress < self.curriculum_progress:
                logger.info(
                    f"Decreasing curriculum difficulty: {self.curriculum_progress:.2f} -> "
                    f"{new_progress:.2f} (success rate: {success_rate:.2%})"
                )
                self.curriculum_progress = new_progress


def create_domain_randomization(
    mode: str = 'moderate',
    config_path: Optional[str] = None,
    adaptive: bool = False,
    **kwargs
) -> Union[DomainRandomization, AdaptiveDomainRandomization]:
    """
    Factory function for creating domain randomization.

    Args:
        mode: Randomization mode ('none', 'conservative', 'moderate', 'aggressive')
        config_path: Path to configuration file
        adaptive: Whether to use adaptive curriculum
        **kwargs: Additional configuration

    Returns:
        DomainRandomization instance
    """
    mode_enum = RandomizationMode(mode.lower())

    if adaptive:
        return AdaptiveDomainRandomization(
            config_path=config_path,
            config_dict=kwargs,
            mode=mode_enum,
        )
    else:
        return DomainRandomization(
            config_path=config_path,
            config_dict=kwargs,
            mode=mode_enum,
        )


if __name__ == '__main__':
    # Test domain randomization
    print("Testing Domain Randomization Module...")

    # Create instance
    dr = DomainRandomization(mode=RandomizationMode.MODERATE)

    # Test physics randomization
    print("\n=== Physics Randomization ===")
    for i in range(3):
        physics = dr.randomize_physics()
        print(f"  Sample {i+1}: mass={physics.mass_kg:.3f} kg, "
              f"drag={physics.drag_coefficient_xy:.3f}")

    # Test sensor noise
    print("\n=== Sensor Noise ===")
    dr.randomize_sensors()
    measurement = np.array([0.0, 0.0, 9.81])
    for i in range(3):
        noisy = dr.add_sensor_noise(measurement, 'imu_accel', dt=0.01)
        print(f"  Sample {i+1}: {noisy}")

    # Test environment randomization
    print("\n=== Environment Randomization ===")
    for i in range(3):
        env = dr.randomize_environment()
        print(f"  Sample {i+1}: wind=({env.wind_velocity_x:.2f}, "
              f"{env.wind_velocity_y:.2f}) m/s")

    # Test curriculum
    print("\n=== Curriculum Learning ===")
    for progress in [0.0, 0.5, 1.0]:
        dr.set_curriculum_progress(progress)
        physics = dr.randomize_physics()
        print(f"  Progress {progress:.0%}: mass={physics.mass_kg:.3f} kg")

    # Print summary
    print("\n" + dr.get_summary())

    print("\nTest complete!")
