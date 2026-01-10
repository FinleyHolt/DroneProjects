"""
Atmosphere Controller - Weather and atmospheric effects.

Single responsibility: Manage fog, visibility, and weather presets for domain randomization.
"""

# Standard library
import random
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

# Isaac Sim / Omniverse
import carb


@dataclass
class WeatherConfig:
    """Weather/atmosphere configuration for domain randomization."""
    # Visibility range in meters (affects camera sensing)
    visibility_range: float = 10000.0  # Clear day default
    # Fog density (0.0 = none, 1.0 = maximum)
    fog_density: float = 0.0
    # Fog color tint (bluish for atmospheric haze)
    fog_color: Tuple[float, float, float] = (0.7, 0.75, 0.85)
    # Dome light intensity multiplier (reduced in haze)
    light_intensity_multiplier: float = 1.0
    # Sun intensity multiplier (reduced in haze)
    sun_intensity_multiplier: float = 1.0
    # Color saturation (reduced in haze for washed-out look)
    saturation: float = 1.0


# Predefined weather presets for domain randomization
WEATHER_PRESETS: Dict[str, WeatherConfig] = {
    "clear": WeatherConfig(
        visibility_range=10000.0,
        fog_density=0.0,
        fog_color=(0.7, 0.75, 0.85),
        light_intensity_multiplier=1.0,
        sun_intensity_multiplier=1.0,
        saturation=1.0,
    ),
    "light_haze": WeatherConfig(
        visibility_range=5000.0,
        fog_density=0.15,
        fog_color=(0.75, 0.8, 0.9),
        light_intensity_multiplier=0.9,
        sun_intensity_multiplier=0.85,
        saturation=0.9,
    ),
    "moderate_haze": WeatherConfig(
        visibility_range=2000.0,
        fog_density=0.35,
        fog_color=(0.8, 0.82, 0.88),
        light_intensity_multiplier=0.75,
        sun_intensity_multiplier=0.65,
        saturation=0.75,
    ),
    "heavy_haze": WeatherConfig(
        visibility_range=800.0,
        fog_density=0.6,
        fog_color=(0.85, 0.85, 0.88),
        light_intensity_multiplier=0.55,
        sun_intensity_multiplier=0.4,
        saturation=0.55,
    ),
}


class AtmosphereController:
    """
    Controls weather and atmospheric effects for realistic rendering.

    Responsibilities:
    - Weather preset management
    - RTX fog configuration
    - Visibility range control
    - Integration with LightingManager for atmospheric light adjustments
    """

    def __init__(
        self,
        stage,
        randomize: bool = False,
        weather: Optional[str] = None,
    ):
        """
        Initialize AtmosphereController.

        Args:
            stage: USD stage
            randomize: Whether to randomize weather
            weather: Specific weather preset to use
        """
        self.stage = stage
        self.randomize = randomize
        self.weather_setting = weather
        self._current_weather: WeatherConfig = WEATHER_PRESETS["clear"]
        self._weather_name: str = "clear"
        self._lighting_callback: Optional[Callable] = None

    def set_lighting_callback(self, callback: Callable) -> None:
        """
        Set callback to adjust lighting when weather changes.

        Args:
            callback: Function(light_multiplier, sun_multiplier, fog_density, fog_color)
        """
        self._lighting_callback = callback

    def setup(self, weather: str = None) -> None:
        """
        Setup atmosphere/weather effects.

        Presets: clear, light_haze, moderate_haze, heavy_haze

        Args:
            weather: Weather preset name (overrides instance setting)
        """
        # Determine weather preset
        if weather is not None:
            weather_name = weather
        elif self.weather_setting is not None:
            weather_name = self.weather_setting
        elif self.randomize:
            # Weighted random selection (more common conditions more likely)
            weights = [0.4, 0.3, 0.2, 0.1]  # clear more likely than heavy haze
            weather_options = ["clear", "light_haze", "moderate_haze", "heavy_haze"]
            weather_name = random.choices(weather_options, weights=weights)[0]
        else:
            weather_name = "clear"

        # Validate and get config
        if weather_name not in WEATHER_PRESETS:
            carb.log_warn(f"Unknown weather preset '{weather_name}', using 'clear'")
            weather_name = "clear"

        weather_config = WEATHER_PRESETS[weather_name]
        self._current_weather = weather_config
        self._weather_name = weather_name

        # Apply fog/atmosphere effects
        self._apply_fog(weather_config)

        # Notify lighting manager to adjust lights
        if self._lighting_callback:
            self._lighting_callback(
                weather_config.light_intensity_multiplier,
                weather_config.sun_intensity_multiplier,
                weather_config.fog_density,
                weather_config.fog_color,
            )

        if weather_name != "clear":
            carb.log_info(f"[AtmosphereController] Weather set to '{weather_name}' "
                          f"(visibility: {weather_config.visibility_range:.0f}m)")

    def _apply_fog(self, weather_config: WeatherConfig) -> None:
        """Apply fog/haze effects using Isaac Sim's RTX fog settings."""
        if weather_config.fog_density <= 0.0:
            # No fog needed, ensure any existing fog is cleared
            self._clear_fog()
            return

        try:
            import carb.settings
            settings = carb.settings.get_settings()
            fog_start, fog_end = 10.0, weather_config.visibility_range

            # Set fog parameters via render settings
            # These are RTX renderer settings for distance fog
            settings.set("/rtx/fog/enabled", True)
            settings.set("/rtx/fog/fogStartDistance", fog_start)
            settings.set("/rtx/fog/fogEndDistance", fog_end)
            settings.set("/rtx/fog/fogColorR", weather_config.fog_color[0])
            settings.set("/rtx/fog/fogColorG", weather_config.fog_color[1])
            settings.set("/rtx/fog/fogColorB", weather_config.fog_color[2])
            settings.set("/rtx/fog/fogDensity", weather_config.fog_density)

            # Height fog for more realistic atmospheric effect (denser near ground)
            settings.set("/rtx/fog/fogHeightDensity", weather_config.fog_density * 0.5)
            settings.set("/rtx/fog/fogHeightFalloff", 0.02)  # Gradual falloff with altitude

        except (ImportError, AttributeError) as e:
            # Fallback: use lighting-based atmosphere simulation only
            carb.log_info(f"[AtmosphereController] RTX fog settings unavailable ({e}), "
                          "using lighting-based atmosphere simulation")

    def _clear_fog(self) -> None:
        """Clear any existing fog settings."""
        try:
            import carb.settings
            settings = carb.settings.get_settings()
            settings.set("/rtx/fog/enabled", False)
        except (ImportError, AttributeError):
            pass

    @property
    def visibility_range(self) -> float:
        """Current visibility range in meters (10000 = clear, 800 = heavy haze)."""
        return self._current_weather.visibility_range

    @property
    def current_weather(self) -> str:
        """Current weather preset name."""
        return self._weather_name

    @property
    def weather_config(self) -> WeatherConfig:
        """Current weather configuration object."""
        return self._current_weather

    def clear(self) -> None:
        """Clear atmosphere effects."""
        self._clear_fog()
        self._current_weather = WEATHER_PRESETS["clear"]
        self._weather_name = "clear"

    @staticmethod
    def get_available_presets() -> list:
        """Return list of available weather preset names."""
        return list(WEATHER_PRESETS.keys())
