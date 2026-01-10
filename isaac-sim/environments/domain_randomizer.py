#!/usr/bin/env python3
"""
Domain Randomization System for Isaac Sim ISR Training

Implements comprehensive domain randomization for sim-to-real transfer to
Flyby F-11 hardware. Based on NVIDIA Isaac Sim Replicator patterns.

Randomization Categories:
1. Terrain/Environment - Coastal, urban, rural, desert, industrial
2. Weather/Lighting - Time of day, fog, visibility, shadows
3. Sensor Degradation - GPS denial, compass interference, camera noise
4. Dynamic Elements - Moving targets, pop-up threats, changing NFZs
5. Communications - Variable link quality, denied zones
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Callable
from enum import Enum, auto
import json


class TerrainType(Enum):
    """Terrain types for domain variation."""
    COASTAL = auto()      # Beach, ocean, cliffs
    URBAN = auto()        # City environment
    RURAL = auto()        # Farmland, sparse buildings
    DESERT = auto()       # Arid, minimal vegetation
    INDUSTRIAL = auto()   # Factories, warehouses


class LightingPreset(Enum):
    """Lighting presets matching Gazebo world configurations."""
    DAY = auto()          # Full daylight
    DAWN = auto()         # Orange tint, low angle
    DUSK = auto()         # Similar to dawn, opposite direction
    OVERCAST = auto()     # Flat, diffuse lighting
    NIGHT = auto()        # Low visibility (future: NVG training)


class WeatherCondition(Enum):
    """Weather conditions affecting visibility and flight."""
    CLEAR = auto()
    HAZY = auto()
    FOGGY = auto()
    DUSTY = auto()        # Desert conditions
    RAINY = auto()        # Affects sensors


@dataclass
class LightingConfig:
    """Lighting randomization configuration."""
    # Sun direction (normalized)
    direction: np.ndarray = field(default_factory=lambda: np.array([-0.5, 0.1, -0.9]))

    # Intensity multipliers
    diffuse_intensity: float = 0.8
    specular_intensity: float = 0.2
    ambient_intensity: float = 0.4

    # Color (RGBA)
    diffuse_color: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0, 1.0]))

    # Shadows
    cast_shadows: bool = True


@dataclass
class WeatherConfig:
    """Weather randomization configuration."""
    condition: WeatherCondition = WeatherCondition.CLEAR

    # Fog parameters
    fog_enabled: bool = False
    fog_start: float = 50.0
    fog_end: float = 300.0
    fog_density: float = 0.02
    fog_color: np.ndarray = field(default_factory=lambda: np.array([0.8, 0.8, 0.85, 1.0]))

    # Wind (affects UAV dynamics)
    wind_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    wind_turbulence: float = 0.0

    # Visibility
    visibility_range: float = 5000.0  # meters


@dataclass
class SensorNoiseConfig:
    """Sensor noise/degradation configuration."""
    # GPS
    gps_enabled: bool = True
    gps_hdop: float = 1.0  # Horizontal dilution of precision
    gps_noise_std: float = 0.5  # Position noise in meters

    # Camera
    camera_noise_std: float = 0.0  # Image noise
    camera_exposure_bias: float = 0.0  # Over/underexposure
    camera_blur_kernel: int = 0  # Motion blur

    # IMU
    imu_gyro_noise: float = 0.009  # rad/s (matching model.sdf)
    imu_accel_noise: float = 0.017  # m/s^2

    # Compass
    compass_declination_error: float = 0.0  # degrees
    compass_interference: float = 0.0  # Additional noise

    # VIO
    vio_available: bool = True
    vio_drift_rate: float = 0.0  # m/s drift


@dataclass
class ObstacleRandomization:
    """Obstacle placement randomization."""
    # Position noise
    position_noise_std: float = 2.0  # meters

    # Rotation noise
    rotation_noise_std: float = 0.1  # radians

    # Scale variation
    scale_min: float = 0.9
    scale_max: float = 1.1

    # Density variation
    density_multiplier: float = 1.0


@dataclass
class NFZRandomization:
    """No-fly zone randomization."""
    # Position offset
    position_noise_std: float = 5.0

    # Size variation
    radius_scale_min: float = 0.8
    radius_scale_max: float = 1.2

    # Activation timing for dynamic NFZs
    activation_time_range: Tuple[float, float] = (60.0, 300.0)


@dataclass
class TargetRandomization:
    """Target of Interest randomization."""
    # Position variation
    position_noise_std: float = 10.0

    # Visibility variation
    occlusion_probability: float = 0.1

    # Movement (for dynamic targets)
    movement_speed_range: Tuple[float, float] = (0.0, 2.0)


@dataclass
class CommsRandomization:
    """Communications zone randomization."""
    # Zone position noise
    position_noise_std: float = 20.0

    # Zone size variation
    radius_scale_range: Tuple[float, float] = (0.8, 1.2)

    # Link quality variation
    link_quality_noise_std: float = 10.0


@dataclass
class RandomizationConfig:
    """Complete domain randomization configuration."""
    # Random seed
    seed: Optional[int] = None

    # Terrain
    terrain_type: TerrainType = TerrainType.URBAN
    randomize_terrain: bool = False

    # Lighting
    lighting: LightingConfig = field(default_factory=LightingConfig)
    randomize_lighting: bool = True
    lighting_preset: LightingPreset = LightingPreset.DAY

    # Weather
    weather: WeatherConfig = field(default_factory=WeatherConfig)
    randomize_weather: bool = True

    # Sensors
    sensor_noise: SensorNoiseConfig = field(default_factory=SensorNoiseConfig)
    randomize_sensors: bool = True

    # Obstacles
    obstacle_config: ObstacleRandomization = field(default_factory=ObstacleRandomization)
    randomize_obstacles: bool = True

    # NFZs
    nfz_config: NFZRandomization = field(default_factory=NFZRandomization)
    randomize_nfz: bool = True

    # Targets
    target_config: TargetRandomization = field(default_factory=TargetRandomization)
    randomize_targets: bool = True

    # Communications
    comms_config: CommsRandomization = field(default_factory=CommsRandomization)
    randomize_comms: bool = True


class DomainRandomizer:
    """
    Domain randomization engine for Isaac Sim training environments.

    Provides procedural variation of:
    - Lighting and weather
    - Sensor characteristics
    - Obstacle placement
    - NFZ configuration
    - Target positions
    - Communications zones
    """

    # Lighting presets matching Gazebo world configs
    LIGHTING_PRESETS: Dict[LightingPreset, LightingConfig] = {
        LightingPreset.DAY: LightingConfig(
            direction=np.array([-0.5, 0.1, -0.9]),
            diffuse_intensity=0.8,
            diffuse_color=np.array([1.0, 1.0, 1.0, 1.0]),
            cast_shadows=True,
        ),
        LightingPreset.DAWN: LightingConfig(
            direction=np.array([0.8, 0.2, -0.3]),
            diffuse_intensity=0.6,
            diffuse_color=np.array([1.0, 0.6, 0.4, 1.0]),
            cast_shadows=True,
        ),
        LightingPreset.DUSK: LightingConfig(
            direction=np.array([-0.8, -0.2, -0.3]),
            diffuse_intensity=0.5,
            diffuse_color=np.array([1.0, 0.5, 0.3, 1.0]),
            cast_shadows=True,
        ),
        LightingPreset.OVERCAST: LightingConfig(
            direction=np.array([0.0, 0.0, -1.0]),
            diffuse_intensity=0.5,
            diffuse_color=np.array([0.9, 0.9, 0.95, 1.0]),
            cast_shadows=False,
        ),
        LightingPreset.NIGHT: LightingConfig(
            direction=np.array([0.0, 0.0, -1.0]),
            diffuse_intensity=0.1,
            diffuse_color=np.array([0.2, 0.2, 0.3, 1.0]),
            cast_shadows=False,
        ),
    }

    # Weather condition parameters
    WEATHER_PRESETS: Dict[WeatherCondition, WeatherConfig] = {
        WeatherCondition.CLEAR: WeatherConfig(
            condition=WeatherCondition.CLEAR,
            fog_enabled=False,
            visibility_range=10000.0,
        ),
        WeatherCondition.HAZY: WeatherConfig(
            condition=WeatherCondition.HAZY,
            fog_enabled=True,
            fog_start=200.0,
            fog_end=1000.0,
            fog_density=0.005,
            visibility_range=2000.0,
        ),
        WeatherCondition.FOGGY: WeatherConfig(
            condition=WeatherCondition.FOGGY,
            fog_enabled=True,
            fog_start=50.0,
            fog_end=300.0,
            fog_density=0.02,
            visibility_range=500.0,
        ),
        WeatherCondition.DUSTY: WeatherConfig(
            condition=WeatherCondition.DUSTY,
            fog_enabled=True,
            fog_start=100.0,
            fog_end=500.0,
            fog_density=0.015,
            fog_color=np.array([0.8, 0.7, 0.5, 1.0]),
            visibility_range=800.0,
        ),
    }

    def __init__(self, config: RandomizationConfig):
        self.config = config
        self.rng = np.random.default_rng(config.seed)

        # Current randomized parameters
        self.current_lighting: LightingConfig = config.lighting
        self.current_weather: WeatherConfig = config.weather
        self.current_sensor_config: SensorNoiseConfig = config.sensor_noise

        # Callbacks for applying randomization
        self._lighting_callback: Optional[Callable] = None
        self._weather_callback: Optional[Callable] = None
        self._sensor_callback: Optional[Callable] = None
        self._obstacle_callback: Optional[Callable] = None

    def set_seed(self, seed: int) -> None:
        """Set random seed for reproducible randomization."""
        self.rng = np.random.default_rng(seed)

    def register_lighting_callback(self, callback: Callable[[LightingConfig], None]) -> None:
        """Register callback for applying lighting changes."""
        self._lighting_callback = callback

    def register_weather_callback(self, callback: Callable[[WeatherConfig], None]) -> None:
        """Register callback for applying weather changes."""
        self._weather_callback = callback

    def register_sensor_callback(self, callback: Callable[[SensorNoiseConfig], None]) -> None:
        """Register callback for applying sensor noise."""
        self._sensor_callback = callback

    def register_obstacle_callback(self, callback: Callable[[List[Dict]], None]) -> None:
        """Register callback for obstacle placement updates."""
        self._obstacle_callback = callback

    def randomize(self, seed: Optional[int] = None) -> Dict[str, Any]:
        """
        Apply domain randomization to all enabled categories.

        Args:
            seed: Optional seed for this randomization pass

        Returns:
            Dictionary of applied randomization parameters
        """
        if seed is not None:
            self.set_seed(seed)

        applied = {}

        if self.config.randomize_lighting:
            applied['lighting'] = self._randomize_lighting()

        if self.config.randomize_weather:
            applied['weather'] = self._randomize_weather()

        if self.config.randomize_sensors:
            applied['sensors'] = self._randomize_sensors()

        return applied

    def _randomize_lighting(self) -> Dict[str, Any]:
        """Randomize lighting conditions."""
        # Select random preset as base
        presets = list(self.LIGHTING_PRESETS.keys())
        # Weight towards day for better training initially
        weights = [0.4, 0.15, 0.15, 0.2, 0.1]
        preset = self.rng.choice(presets, p=weights)

        # Get base configuration
        base_config = self.LIGHTING_PRESETS[preset]

        # Add variation
        direction_noise = self.rng.normal(0, 0.1, size=3)
        direction = base_config.direction + direction_noise
        direction = direction / np.linalg.norm(direction)

        intensity_scale = self.rng.uniform(0.8, 1.2)

        self.current_lighting = LightingConfig(
            direction=direction,
            diffuse_intensity=base_config.diffuse_intensity * intensity_scale,
            specular_intensity=base_config.specular_intensity * intensity_scale,
            ambient_intensity=base_config.ambient_intensity * intensity_scale,
            diffuse_color=base_config.diffuse_color,
            cast_shadows=base_config.cast_shadows,
        )

        # Apply via callback if registered
        if self._lighting_callback:
            self._lighting_callback(self.current_lighting)

        return {
            'preset': preset.name,
            'direction': direction.tolist(),
            'intensity_scale': intensity_scale,
        }

    def _randomize_weather(self) -> Dict[str, Any]:
        """Randomize weather conditions."""
        # Select random weather
        conditions = list(self.WEATHER_PRESETS.keys())
        # Weight towards clear/hazy for most training
        weights = [0.5, 0.25, 0.15, 0.1]
        condition = self.rng.choice(conditions, p=weights)

        base_config = self.WEATHER_PRESETS[condition]

        # Add wind variation
        wind_speed = self.rng.uniform(0, 10)  # m/s
        wind_direction = self.rng.uniform(0, 2 * np.pi)
        wind_velocity = np.array([
            wind_speed * np.cos(wind_direction),
            wind_speed * np.sin(wind_direction),
            0.0
        ])

        # Turbulence based on wind speed
        turbulence = wind_speed * self.rng.uniform(0.1, 0.3)

        self.current_weather = WeatherConfig(
            condition=condition,
            fog_enabled=base_config.fog_enabled,
            fog_start=base_config.fog_start * self.rng.uniform(0.8, 1.2),
            fog_end=base_config.fog_end * self.rng.uniform(0.8, 1.2),
            fog_density=base_config.fog_density * self.rng.uniform(0.8, 1.2),
            fog_color=base_config.fog_color,
            wind_velocity=wind_velocity,
            wind_turbulence=turbulence,
            visibility_range=base_config.visibility_range * self.rng.uniform(0.7, 1.3),
        )

        if self._weather_callback:
            self._weather_callback(self.current_weather)

        return {
            'condition': condition.name,
            'wind_speed': wind_speed,
            'wind_direction_deg': np.degrees(wind_direction),
            'visibility': self.current_weather.visibility_range,
        }

    def _randomize_sensors(self) -> Dict[str, Any]:
        """Randomize sensor characteristics."""
        cfg = self.config.sensor_noise

        # GPS status (sometimes denied for training)
        gps_denial_prob = 0.15  # 15% chance of GPS denied
        gps_enabled = self.rng.random() > gps_denial_prob

        if gps_enabled:
            # Vary HDOP
            gps_hdop = self.rng.uniform(1.0, 5.0)
            gps_noise = cfg.gps_noise_std * gps_hdop
        else:
            gps_hdop = float('inf')
            gps_noise = float('inf')

        # Camera noise based on lighting
        if self.current_lighting.diffuse_intensity < 0.3:
            # Low light = more noise
            camera_noise = self.rng.uniform(0.02, 0.08)
            camera_exposure = self.rng.uniform(-0.5, 0.5)
        else:
            camera_noise = self.rng.uniform(0.0, 0.02)
            camera_exposure = self.rng.uniform(-0.2, 0.2)

        # IMU noise (slight variation around nominal)
        imu_gyro = cfg.imu_gyro_noise * self.rng.uniform(0.8, 1.5)
        imu_accel = cfg.imu_accel_noise * self.rng.uniform(0.8, 1.5)

        # Compass interference (more likely in urban/industrial)
        if self.config.terrain_type in [TerrainType.URBAN, TerrainType.INDUSTRIAL]:
            compass_interference = self.rng.uniform(0, 5.0)
        else:
            compass_interference = self.rng.uniform(0, 1.0)

        # VIO availability (usually available unless features are sparse)
        vio_failure_prob = 0.05
        vio_available = self.rng.random() > vio_failure_prob

        self.current_sensor_config = SensorNoiseConfig(
            gps_enabled=gps_enabled,
            gps_hdop=gps_hdop,
            gps_noise_std=gps_noise,
            camera_noise_std=camera_noise,
            camera_exposure_bias=camera_exposure,
            imu_gyro_noise=imu_gyro,
            imu_accel_noise=imu_accel,
            compass_interference=compass_interference,
            vio_available=vio_available,
        )

        if self._sensor_callback:
            self._sensor_callback(self.current_sensor_config)

        return {
            'gps_enabled': gps_enabled,
            'gps_hdop': gps_hdop if gps_enabled else 'denied',
            'camera_noise': camera_noise,
            'vio_available': vio_available,
        }

    def randomize_obstacle_positions(
        self,
        base_positions: List[np.ndarray]
    ) -> List[np.ndarray]:
        """
        Apply randomization to obstacle positions.

        Args:
            base_positions: Original obstacle positions

        Returns:
            Randomized positions
        """
        cfg = self.config.obstacle_config
        randomized = []

        for pos in base_positions:
            noise = self.rng.normal(0, cfg.position_noise_std, size=3)
            noise[2] = 0  # Keep z unchanged
            randomized.append(pos + noise)

        return randomized

    def randomize_nfz_config(
        self,
        base_nfzs: List[Dict]
    ) -> List[Dict]:
        """
        Apply randomization to NFZ configurations.

        Args:
            base_nfzs: Original NFZ definitions

        Returns:
            Randomized NFZ configurations
        """
        cfg = self.config.nfz_config
        randomized = []

        for nfz in base_nfzs:
            new_nfz = nfz.copy()

            # Position noise
            pos_noise = self.rng.normal(0, cfg.position_noise_std, size=2)
            if 'center' in new_nfz:
                new_nfz['center'] = np.array(new_nfz['center'])
                new_nfz['center'][:2] += pos_noise

            # Radius variation
            radius_scale = self.rng.uniform(
                cfg.radius_scale_min,
                cfg.radius_scale_max
            )
            if 'radius' in new_nfz:
                new_nfz['radius'] *= radius_scale

            # Activation time for dynamic NFZs
            if new_nfz.get('is_dynamic', False):
                new_nfz['activation_time'] = self.rng.uniform(
                    *cfg.activation_time_range
                )

            randomized.append(new_nfz)

        return randomized

    def randomize_target_positions(
        self,
        base_targets: List[Dict]
    ) -> List[Dict]:
        """
        Apply randomization to target positions.

        Args:
            base_targets: Original target definitions

        Returns:
            Randomized target configurations
        """
        cfg = self.config.target_config
        randomized = []

        for target in base_targets:
            new_target = target.copy()

            # Position noise
            pos_noise = self.rng.normal(0, cfg.position_noise_std, size=3)
            if 'position' in new_target:
                new_target['position'] = np.array(new_target['position']) + pos_noise

            # Occlusion
            new_target['occluded'] = self.rng.random() < cfg.occlusion_probability

            # Movement for dynamic targets
            if target.get('is_dynamic', False):
                new_target['speed'] = self.rng.uniform(*cfg.movement_speed_range)

            randomized.append(new_target)

        return randomized

    def get_curriculum_config(self, difficulty: float) -> RandomizationConfig:
        """
        Get randomization config scaled by difficulty level.

        Args:
            difficulty: Value from 0 (easy) to 1 (hard)

        Returns:
            Scaled randomization configuration
        """
        config = RandomizationConfig(seed=self.config.seed)

        # Scale noise levels by difficulty
        config.sensor_noise.gps_noise_std *= (0.5 + difficulty)
        config.sensor_noise.camera_noise_std *= difficulty
        config.sensor_noise.imu_gyro_noise *= (0.8 + 0.4 * difficulty)

        # More weather variation at higher difficulty
        config.weather.wind_turbulence = 5.0 * difficulty

        # More obstacle variation
        config.obstacle_config.position_noise_std = 1.0 + 4.0 * difficulty

        # Enable more randomization at higher difficulty
        config.randomize_terrain = difficulty > 0.5
        config.randomize_weather = True
        config.randomize_sensors = True

        return config

    def save_config(self, path: str) -> None:
        """Save current randomization state to file."""
        state = {
            'lighting': {
                'direction': self.current_lighting.direction.tolist(),
                'diffuse_intensity': self.current_lighting.diffuse_intensity,
            },
            'weather': {
                'condition': self.current_weather.condition.name,
                'wind_velocity': self.current_weather.wind_velocity.tolist(),
            },
            'sensors': {
                'gps_enabled': self.current_sensor_config.gps_enabled,
                'gps_hdop': self.current_sensor_config.gps_hdop,
                'vio_available': self.current_sensor_config.vio_available,
            },
        }
        with open(path, 'w') as f:
            json.dump(state, f, indent=2)

    def load_config(self, path: str) -> None:
        """Load randomization state from file."""
        with open(path, 'r') as f:
            state = json.load(f)

        # Restore state
        if 'lighting' in state:
            self.current_lighting.direction = np.array(state['lighting']['direction'])
            self.current_lighting.diffuse_intensity = state['lighting']['diffuse_intensity']

        # Apply callbacks
        if self._lighting_callback:
            self._lighting_callback(self.current_lighting)
        if self._weather_callback:
            self._weather_callback(self.current_weather)
        if self._sensor_callback:
            self._sensor_callback(self.current_sensor_config)
