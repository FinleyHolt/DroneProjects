"""
Lighting Manager - Environment lighting and sky setup.

Single responsibility: Manage HDRI skybox, sun/distant lights, and time-of-day presets.
"""

# Standard library
import random
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# Third-party
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux

# Isaac Sim / Omniverse
import carb


@dataclass
class TimeOfDayPreset:
    """Lighting preset for a specific time of day."""
    name: str
    sun_intensity: float
    sun_elevation: Tuple[float, float]  # (min, max) elevation angle in degrees
    sun_color: Tuple[float, float, float]  # RGB color
    dome_intensity: float
    color_temperature: float  # Kelvin (warm ~3000K, neutral ~5500K, cool ~7000K)


# Time of day presets for realistic lighting variation
TIME_OF_DAY_PRESETS: Dict[str, TimeOfDayPreset] = {
    "dawn": TimeOfDayPreset(
        name="dawn",
        sun_intensity=1500.0,
        sun_elevation=(5.0, 15.0),  # Very low sun
        sun_color=(1.0, 0.65, 0.35),   # Deep warm orange
        dome_intensity=600.0,
        color_temperature=3000.0,       # Warm
    ),
    "morning": TimeOfDayPreset(
        name="morning",
        sun_intensity=2500.0,
        sun_elevation=(30.0, 50.0),   # Rising sun
        sun_color=(1.0, 0.9, 0.75),     # Slightly warm
        dome_intensity=800.0,
        color_temperature=5000.0,       # Neutral-warm
    ),
    "noon": TimeOfDayPreset(
        name="noon",
        sun_intensity=3500.0,
        sun_elevation=(70.0, 80.0),   # High overhead
        sun_color=(1.0, 0.98, 0.95),    # Slightly cool white
        dome_intensity=1000.0,
        color_temperature=6000.0,       # Slightly cool
    ),
    "afternoon": TimeOfDayPreset(
        name="afternoon",
        sun_intensity=3000.0,
        sun_elevation=(30.0, 50.0),   # Descending sun
        sun_color=(1.0, 0.92, 0.8),     # Warm golden
        dome_intensity=900.0,
        color_temperature=5200.0,       # Neutral-warm
    ),
    "dusk": TimeOfDayPreset(
        name="dusk",
        sun_intensity=1200.0,
        sun_elevation=(5.0, 15.0),   # Very low sun
        sun_color=(1.0, 0.5, 0.25),     # Deep orange-red
        dome_intensity=500.0,
        color_temperature=2500.0,       # Very warm
    ),
    "overcast": TimeOfDayPreset(
        name="overcast",
        sun_intensity=500.0,            # Very dim direct light
        sun_elevation=(40.0, 60.0),   # Mid elevation (doesn't matter much)
        sun_color=(0.9, 0.9, 0.95),     # Neutral gray-blue
        dome_intensity=1800.0,          # High diffuse light
        color_temperature=6500.0,       # Cool diffuse
    ),
}


class LightingManager:
    """
    Manages environment lighting including HDRI skybox and sun.

    Responsibilities:
    - HDRI dome light creation and configuration
    - Sun/distant light setup with proper orientation
    - Time-of-day preset application
    - Color temperature conversion
    - Lighting randomization for domain variation
    """

    DEFAULT_HDRI = "sky_2k.hdr"  # Clean sky HDRI for aerial training

    def __init__(
        self,
        stage,
        textures_path: str,
        randomize: bool = True,
        time_of_day: Optional[str] = None,
    ):
        """
        Initialize LightingManager.

        Args:
            stage: USD stage to create lights on
            textures_path: Path to textures directory
            randomize: Whether to randomize lighting
            time_of_day: Specific time of day preset to use
        """
        self.stage = stage
        self.textures_path = textures_path
        self.randomize = randomize
        self.time_of_day_setting = time_of_day
        self.skybox_light = None
        self._current_time_of_day: Optional[str] = None

    def setup(
        self,
        hdri_path: str = None,
        sun_intensity: float = None,
        sun_angle: Tuple[float, float] = (-45.0, 30.0),
    ) -> None:
        """
        Setup environment lighting with HDRI skybox and sun.

        Uses time-of-day presets if configured.

        Args:
            hdri_path: Path to HDRI file (uses default if None)
            sun_intensity: Sun intensity override
            sun_angle: (elevation, azimuth) in degrees
        """
        if hdri_path is None:
            hdri_path = f"{self.textures_path}/{self.DEFAULT_HDRI}"

        # Get time of day preset if applicable
        tod_preset = self._get_time_of_day_preset()
        self._current_time_of_day = tod_preset.name if tod_preset else None

        # Determine lighting parameters from preset or config
        if tod_preset:
            # Use preset values
            effective_sun_intensity = tod_preset.sun_intensity
            dome_intensity = tod_preset.dome_intensity
            sun_color = Gf.Vec3f(*tod_preset.sun_color)
            sun_elevation = random.uniform(*tod_preset.sun_elevation)
            sun_azimuth = random.uniform(-180, 180)  # Random azimuth for variety
            color_temp = tod_preset.color_temperature
        else:
            # Use config values (fallback)
            effective_sun_intensity = sun_intensity or 3000.0
            dome_intensity = 1000.0
            sun_color = Gf.Vec3f(1.0, 0.95, 0.85)
            sun_elevation = sun_angle[0]
            sun_azimuth = sun_angle[1]
            color_temp = 5500.0  # Neutral daylight

        # Create dome light for sky background
        dome_path = "/World/Lighting/DomeLight"
        dome_light = UsdLux.DomeLight.Define(self.stage, dome_path)
        dome_light.CreateIntensityAttr(dome_intensity)

        # Use clean sky HDRI for realistic background
        dome_light.CreateTextureFileAttr().Set(hdri_path)
        dome_light.CreateTextureFormatAttr().Set("latlong")

        # Apply color temperature tint to dome light for time-of-day ambiance
        dome_prim = dome_light.GetPrim()
        dome_prim.CreateAttribute("inputs:enableColorTemperature", Sdf.ValueTypeNames.Bool).Set(True)
        dome_prim.CreateAttribute("inputs:colorTemperature", Sdf.ValueTypeNames.Float).Set(color_temp)

        # Apply color tint based on temperature for additional warmth/coolness
        temp_tint = self._kelvin_to_rgb(color_temp)
        dome_light.CreateColorAttr(Gf.Vec3f(*temp_tint))

        # Disable shadow casting from dome light - only sun should cast shadows
        shadow_api = UsdLux.ShadowAPI.Apply(dome_prim)
        shadow_api.CreateShadowEnableAttr().Set(False)

        # Expose sky texture - important for RayTracedLighting mode
        dome_light.CreateSpecularAttr().Set(1.0)  # Full specular reflection of sky

        # Create distant light (sun) - THE ONLY PRIMARY LIGHT SOURCE
        sun_path = "/World/Lighting/SunLight"
        sun_light = UsdLux.DistantLight.Define(self.stage, sun_path)
        sun_light.CreateIntensityAttr(effective_sun_intensity)
        sun_light.CreateColorAttr(sun_color)
        sun_light.CreateAngleAttr(0.53)  # Sun angular diameter

        # Rotate sun based on elevation and azimuth
        sun_xform = UsdGeom.Xformable(sun_light.GetPrim())
        sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(sun_elevation, sun_azimuth, 0.0))

        # Apply additional randomization if enabled and not using a preset
        if self.randomize and not tod_preset:
            # Random sun angle variation
            angle_x = sun_elevation + random.uniform(-15, 15)
            angle_y = sun_azimuth + random.uniform(-30, 30)
            sun_xform.GetOrderedXformOps()[0].Set(Gf.Vec3f(angle_x, angle_y, 0.0))

            # Random intensity variation
            intensity_var = random.uniform(0.8, 1.2)
            sun_light.GetIntensityAttr().Set(effective_sun_intensity * intensity_var)

        self.skybox_light = dome_path

    def _get_time_of_day_preset(self) -> Optional[TimeOfDayPreset]:
        """
        Get the time of day preset to use for lighting.

        Returns:
            TimeOfDayPreset if a preset should be used, None otherwise
        """
        # If time_of_day is explicitly set, use that preset
        if self.time_of_day_setting is not None:
            if self.time_of_day_setting in TIME_OF_DAY_PRESETS:
                return TIME_OF_DAY_PRESETS[self.time_of_day_setting]
            else:
                carb.log_warn(f"Unknown time_of_day '{self.time_of_day_setting}', "
                              f"valid options: {list(TIME_OF_DAY_PRESETS.keys())}")
                return None

        # If randomize is enabled and no time_of_day specified, pick randomly
        if self.randomize:
            preset_name = random.choice(list(TIME_OF_DAY_PRESETS.keys()))
            return TIME_OF_DAY_PRESETS[preset_name]

        return None

    def _kelvin_to_rgb(self, kelvin: float) -> Tuple[float, float, float]:
        """Convert color temperature in Kelvin to RGB tint (1000K-10000K range)."""
        temp = max(1000.0, min(10000.0, kelvin)) / 100.0

        if temp <= 66:
            red = 1.0
            green = max(0.0, min(255.0, 99.4708025861 * np.log(max(1.0, temp)) - 161.1195681661)) / 255.0
        else:
            red = max(0.0, min(255.0, 329.698727446 * ((temp - 60) ** -0.1332047592))) / 255.0
            green = max(0.0, min(255.0, 288.1221695283 * ((temp - 60) ** -0.0755148492))) / 255.0

        if temp >= 66:
            blue = 1.0
        elif temp <= 19:
            blue = 0.0
        else:
            blue = max(0.0, min(255.0, 138.5177312231 * np.log(max(1.0, temp - 10)) - 305.0447927307)) / 255.0

        return (red, green, blue)

    def adjust_for_weather(
        self,
        light_multiplier: float,
        sun_multiplier: float,
        fog_density: float,
        fog_color: Tuple[float, float, float],
    ) -> None:
        """
        Adjust scene lights to simulate atmospheric scattering.

        Args:
            light_multiplier: Dome light intensity multiplier
            sun_multiplier: Sun intensity multiplier
            fog_density: Fog density for color tinting
            fog_color: Fog RGB color
        """
        dome_path = "/World/Lighting/DomeLight"
        dome_prim = self.stage.GetPrimAtPath(dome_path)
        if dome_prim.IsValid():
            dome_light = UsdLux.DomeLight(dome_prim)

            # Get current intensity and apply weather multiplier
            current_intensity = dome_light.GetIntensityAttr().Get()
            if current_intensity:
                new_intensity = current_intensity * light_multiplier
                dome_light.GetIntensityAttr().Set(new_intensity)

            # Apply slight color tint for haze (desaturated, bluish)
            if fog_density > 0:
                blend_factor = fog_density * 0.5  # Subtle effect
                tint = Gf.Vec3f(
                    1.0 - blend_factor * (1.0 - fog_color[0]),
                    1.0 - blend_factor * (1.0 - fog_color[1]),
                    1.0 - blend_factor * (1.0 - fog_color[2]),
                )
                dome_light.CreateColorAttr(tint)

        # Adjust sun light
        sun_path = "/World/Lighting/SunLight"
        sun_prim = self.stage.GetPrimAtPath(sun_path)
        if sun_prim.IsValid():
            sun_light = UsdLux.DistantLight(sun_prim)

            # Get current intensity and apply weather multiplier
            current_intensity = sun_light.GetIntensityAttr().Get()
            if current_intensity:
                new_sun = current_intensity * sun_multiplier
                sun_light.GetIntensityAttr().Set(new_sun)

            # Shift sun color toward warmer/hazier tones
            if fog_density > 0:
                haze_factor = fog_density * 0.3
                sun_color = Gf.Vec3f(
                    1.0,
                    0.95 - haze_factor * 0.1,
                    0.85 - haze_factor * 0.2,
                )
                sun_light.GetColorAttr().Set(sun_color)

    @property
    def current_time_of_day(self) -> Optional[str]:
        """Current time of day preset name, or None if using default lighting."""
        return self._current_time_of_day

    def clear(self) -> None:
        """Remove all lighting prims."""
        lighting_path = "/World/Lighting"
        prim = self.stage.GetPrimAtPath(lighting_path)
        if prim.IsValid():
            self.stage.RemovePrim(lighting_path)
        self.skybox_light = None
        self._current_time_of_day = None

    @staticmethod
    def get_available_presets() -> list:
        """Return list of available time-of-day preset names."""
        return list(TIME_OF_DAY_PRESETS.keys())
