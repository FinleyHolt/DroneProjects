"""
Flyby World Generator Extension

Procedural world generation for ISR training environments.
Generates photorealistic terrains with vegetation, vehicles, and people.
"""

from .world_generator import WorldGenerator, WorldConfig

__all__ = [
    "WorldGenerator",
    "WorldConfig",
]
