"""
Flyby World Generator Extension

Procedural world generation for ISR training environments.
Generates photorealistic terrains with vegetation, vehicles, and people.

Architecture:
    WorldGenerator (orchestrator) composes:
    - TerrainGenerator: Heightfield mesh generation
    - MaterialManager: PBR material application
    - LightingManager: HDRI skybox and sun
    - AtmosphereController: Weather and fog effects
"""

# Main orchestrator and config
from .world_generator import WorldGenerator, WorldConfig

# Individual components (for advanced use cases)
from .terrain_generator import TerrainGenerator
from .material_manager import MaterialManager
from .lighting_manager import LightingManager, TIME_OF_DAY_PRESETS, TimeOfDayPreset
from .atmosphere_controller import AtmosphereController, WeatherConfig, WEATHER_PRESETS

__all__ = [
    # Primary API
    "WorldGenerator",
    "WorldConfig",
    # Components
    "TerrainGenerator",
    "MaterialManager",
    "LightingManager",
    "AtmosphereController",
    # Presets and configs
    "WeatherConfig",
    "WEATHER_PRESETS",
    "TimeOfDayPreset",
    "TIME_OF_DAY_PRESETS",
]
