#!/usr/bin/env python3
"""
Gazebo World Variant Generator for Flyby F-11 RL Training

Generates procedurally randomized world variants for reinforcement learning
domain randomization. Supports configurable obstacle density, placement,
lighting conditions, and weather effects.

Features:
- Reproducible generation with seed control
- Multiple difficulty presets
- Obstacle type variety (poles, buildings, trees, walls)
- NFZ randomization within safe bounds
- Landing zone placement
- Lighting and weather variations
- Gazebo Harmonic SDF 1.9+ compatible output

Usage:
    python generate_world_variants.py --num-worlds 10 --seed 42
    python generate_world_variants.py --difficulty hard --output-dir ./generated
    python generate_world_variants.py --config custom_config.json

Author: Finley Holt
Version: 1.0.0
"""

import argparse
import json
import math
import os
import random
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple
import xml.etree.ElementTree as ET
from xml.dom import minidom


@dataclass
class Vector3:
    """3D vector for positions and sizes."""
    x: float
    y: float
    z: float

    def to_pose_string(self, roll: float = 0, pitch: float = 0, yaw: float = 0) -> str:
        return f"{self.x} {self.y} {self.z} {roll} {pitch} {yaw}"


@dataclass
class Obstacle:
    """Base obstacle configuration."""
    name: str
    position: Vector3
    obstacle_type: str  # 'cylinder', 'box', 'tree', 'wall'
    size: Vector3  # For box: xyz dimensions; for cylinder: x=radius, z=height
    rotation: float = 0.0
    color: Tuple[float, float, float] = (0.5, 0.5, 0.5)


@dataclass
class NoFlyZone:
    """No-fly zone configuration."""
    name: str
    position: Vector3
    zone_type: str  # 'cylinder', 'box'
    size: Vector3  # For cylinder: x=radius, z=height; for box: xyz
    priority: int = 1


@dataclass
class LandingZone:
    """Landing zone configuration."""
    name: str
    position: Vector3
    radius: float
    zone_type: str = "standard"  # 'primary', 'secondary', 'emergency'


@dataclass
class WorldConfig:
    """Complete world generation configuration."""
    name: str
    seed: int
    difficulty: str
    geofence_size: float
    max_altitude: float
    num_obstacles: int
    num_nfz: int
    num_landing_zones: int
    wind_enabled: bool
    wind_speed: Vector3
    fog_enabled: bool
    fog_density: float
    lighting_preset: str  # 'day', 'dusk', 'overcast', 'night'
    obstacles: List[Obstacle] = field(default_factory=list)
    no_fly_zones: List[NoFlyZone] = field(default_factory=list)
    landing_zones: List[LandingZone] = field(default_factory=list)


# Difficulty presets
DIFFICULTY_PRESETS = {
    "easy": {
        "num_obstacles": (5, 10),
        "num_nfz": (1, 2),
        "obstacle_density": 0.3,
        "max_obstacle_height": 15,
        "narrow_passages": False,
    },
    "medium": {
        "num_obstacles": (15, 25),
        "num_nfz": (2, 3),
        "obstacle_density": 0.5,
        "max_obstacle_height": 25,
        "narrow_passages": True,
    },
    "hard": {
        "num_obstacles": (30, 45),
        "num_nfz": (3, 5),
        "obstacle_density": 0.7,
        "max_obstacle_height": 35,
        "narrow_passages": True,
    },
    "extreme": {
        "num_obstacles": (50, 70),
        "num_nfz": (4, 6),
        "obstacle_density": 0.85,
        "max_obstacle_height": 45,
        "narrow_passages": True,
    },
}

# Lighting presets
LIGHTING_PRESETS = {
    "day": {
        "sun_direction": (-0.5, 0.1, -0.9),
        "sun_diffuse": (0.8, 0.8, 0.8),
        "sun_specular": (0.2, 0.2, 0.2),
        "ambient": (0.6, 0.6, 0.6),
        "background": (0.7, 0.8, 0.9),
    },
    "dusk": {
        "sun_direction": (-0.1, 0.3, -0.3),
        "sun_diffuse": (0.9, 0.6, 0.4),
        "sun_specular": (0.2, 0.1, 0.1),
        "ambient": (0.4, 0.35, 0.35),
        "background": (0.8, 0.5, 0.4),
    },
    "overcast": {
        "sun_direction": (-0.3, 0.0, -0.95),
        "sun_diffuse": (0.5, 0.5, 0.55),
        "sun_specular": (0.05, 0.05, 0.05),
        "ambient": (0.5, 0.5, 0.52),
        "background": (0.6, 0.6, 0.65),
    },
    "night": {
        "sun_direction": (0.0, 0.0, -1.0),
        "sun_diffuse": (0.1, 0.1, 0.15),
        "sun_specular": (0.0, 0.0, 0.0),
        "ambient": (0.15, 0.15, 0.2),
        "background": (0.05, 0.05, 0.1),
    },
}

# Obstacle color palettes
OBSTACLE_COLORS = {
    "building": [(0.45, 0.45, 0.5), (0.5, 0.5, 0.55), (0.55, 0.55, 0.55), (0.4, 0.4, 0.45)],
    "pole": [(0.65, 0.65, 0.65), (0.7, 0.7, 0.7), (0.75, 0.75, 0.75)],
    "tree_trunk": [(0.4, 0.25, 0.1), (0.38, 0.22, 0.08), (0.42, 0.27, 0.12)],
    "tree_canopy": [(0.15, 0.45, 0.15), (0.12, 0.4, 0.12), (0.18, 0.5, 0.18), (0.1, 0.38, 0.1)],
    "wall": [(0.5, 0.5, 0.45), (0.55, 0.55, 0.5), (0.48, 0.48, 0.44)],
    "industrial": [(0.5, 0.45, 0.4), (0.55, 0.5, 0.45), (0.6, 0.6, 0.65)],
}


class WorldGenerator:
    """Procedural Gazebo world generator for UAV RL training."""

    def __init__(
        self,
        seed: Optional[int] = None,
        geofence_size: float = 300.0,
        max_altitude: float = 100.0,
    ):
        self.seed = seed if seed is not None else random.randint(0, 2**32 - 1)
        self.rng = random.Random(self.seed)
        self.geofence_size = geofence_size
        self.max_altitude = max_altitude
        self.half_size = geofence_size / 2
        self.placed_obstacles: List[Tuple[Vector3, float]] = []  # (position, radius)

    def reset_seed(self, seed: int):
        """Reset the random number generator with a new seed."""
        self.seed = seed
        self.rng = random.Random(seed)
        self.placed_obstacles = []

    def _check_collision(
        self, pos: Vector3, radius: float, min_distance: float = 5.0
    ) -> bool:
        """Check if position collides with existing obstacles."""
        for existing_pos, existing_radius in self.placed_obstacles:
            dist = math.sqrt(
                (pos.x - existing_pos.x) ** 2 + (pos.y - existing_pos.y) ** 2
            )
            if dist < (radius + existing_radius + min_distance):
                return True
        return False

    def _find_valid_position(
        self,
        radius: float,
        zone_bounds: Optional[Tuple[float, float, float, float]] = None,
        max_attempts: int = 100,
        min_distance: float = 5.0,
        avoid_center: bool = True,
        center_exclusion_radius: float = 15.0,
    ) -> Optional[Vector3]:
        """Find a valid position that doesn't collide with existing obstacles."""
        if zone_bounds:
            min_x, max_x, min_y, max_y = zone_bounds
        else:
            margin = radius + 10
            min_x = -self.half_size + margin
            max_x = self.half_size - margin
            min_y = -self.half_size + margin
            max_y = self.half_size - margin

        for _ in range(max_attempts):
            x = self.rng.uniform(min_x, max_x)
            y = self.rng.uniform(min_y, max_y)
            pos = Vector3(x, y, 0)

            # Check center exclusion (for landing zone protection)
            if avoid_center:
                dist_from_center = math.sqrt(x**2 + y**2)
                if dist_from_center < center_exclusion_radius:
                    continue

            if not self._check_collision(pos, radius, min_distance):
                return pos

        return None

    def generate_obstacle(
        self,
        name: str,
        obstacle_type: str,
        zone_bounds: Optional[Tuple[float, float, float, float]] = None,
        max_height: float = 25.0,
    ) -> Optional[Obstacle]:
        """Generate a single obstacle of the specified type."""
        if obstacle_type == "pole":
            radius = self.rng.uniform(0.3, 0.6)
            height = self.rng.uniform(15, max_height)
            pos = self._find_valid_position(radius * 2, zone_bounds)
            if pos is None:
                return None
            pos.z = height / 2
            color = self.rng.choice(OBSTACLE_COLORS["pole"])
            self.placed_obstacles.append((Vector3(pos.x, pos.y, 0), radius * 2))
            return Obstacle(
                name=name,
                position=pos,
                obstacle_type="cylinder",
                size=Vector3(radius, radius, height),
                color=color,
            )

        elif obstacle_type == "building":
            width = self.rng.uniform(8, 20)
            depth = self.rng.uniform(6, 15)
            height = self.rng.uniform(8, max_height)
            radius = max(width, depth) / 2
            pos = self._find_valid_position(radius, zone_bounds, min_distance=8)
            if pos is None:
                return None
            pos.z = height / 2
            color = self.rng.choice(OBSTACLE_COLORS["building"])
            rotation = self.rng.uniform(0, math.pi / 4)
            self.placed_obstacles.append((Vector3(pos.x, pos.y, 0), radius))
            return Obstacle(
                name=name,
                position=pos,
                obstacle_type="box",
                size=Vector3(width, depth, height),
                rotation=rotation,
                color=color,
            )

        elif obstacle_type == "tree":
            trunk_radius = self.rng.uniform(0.5, 0.8)
            canopy_radius = self.rng.uniform(3, 6)
            trunk_height = self.rng.uniform(6, 12)
            pos = self._find_valid_position(canopy_radius, zone_bounds)
            if pos is None:
                return None
            # Tree position is at ground level
            self.placed_obstacles.append((Vector3(pos.x, pos.y, 0), canopy_radius))
            # Store extra data in size for tree rendering
            return Obstacle(
                name=name,
                position=pos,
                obstacle_type="tree",
                size=Vector3(trunk_radius, canopy_radius, trunk_height),
                color=self.rng.choice(OBSTACLE_COLORS["tree_canopy"]),
            )

        elif obstacle_type == "wall":
            length = self.rng.uniform(10, 25)
            thickness = self.rng.uniform(1, 2)
            height = self.rng.uniform(6, 12)
            radius = length / 2
            pos = self._find_valid_position(radius, zone_bounds, min_distance=6)
            if pos is None:
                return None
            pos.z = height / 2
            color = self.rng.choice(OBSTACLE_COLORS["wall"])
            rotation = self.rng.uniform(0, math.pi)
            self.placed_obstacles.append((Vector3(pos.x, pos.y, 0), radius))
            return Obstacle(
                name=name,
                position=pos,
                obstacle_type="box",
                size=Vector3(length, thickness, height),
                rotation=rotation,
                color=color,
            )

        elif obstacle_type == "tank":
            radius = self.rng.uniform(4, 8)
            height = self.rng.uniform(6, 12)
            pos = self._find_valid_position(radius, zone_bounds, min_distance=8)
            if pos is None:
                return None
            pos.z = height / 2
            color = self.rng.choice(OBSTACLE_COLORS["industrial"])
            self.placed_obstacles.append((Vector3(pos.x, pos.y, 0), radius))
            return Obstacle(
                name=name,
                position=pos,
                obstacle_type="cylinder",
                size=Vector3(radius, radius, height),
                color=color,
            )

        return None

    def generate_nfz(
        self,
        name: str,
        existing_nfz: List[NoFlyZone],
    ) -> Optional[NoFlyZone]:
        """Generate a no-fly zone that doesn't overlap with existing ones."""
        zone_type = self.rng.choice(["cylinder", "box"])

        if zone_type == "cylinder":
            radius = self.rng.uniform(15, 30)
            height = self.rng.uniform(30, 60)
            size = Vector3(radius, radius, height)
            collision_radius = radius
        else:
            width = self.rng.uniform(20, 40)
            depth = self.rng.uniform(15, 35)
            height = self.rng.uniform(30, 60)
            size = Vector3(width, depth, height)
            collision_radius = max(width, depth) / 2

        # Find valid position away from center and other NFZs
        pos = self._find_valid_position(
            collision_radius,
            min_distance=collision_radius + 20,
            center_exclusion_radius=40,
        )
        if pos is None:
            return None

        # Check distance from other NFZs
        for existing in existing_nfz:
            dist = math.sqrt(
                (pos.x - existing.position.x) ** 2 + (pos.y - existing.position.y) ** 2
            )
            existing_radius = existing.size.x if existing.zone_type == "cylinder" else max(existing.size.x, existing.size.y) / 2
            if dist < collision_radius + existing_radius + 30:
                return None

        pos.z = height / 2
        priority = self.rng.randint(1, 3)

        return NoFlyZone(
            name=name,
            position=pos,
            zone_type=zone_type,
            size=size,
            priority=priority,
        )

    def generate_landing_zone(
        self,
        name: str,
        zone_type: str,
        existing_lz: List[LandingZone],
    ) -> Optional[LandingZone]:
        """Generate a landing zone."""
        if zone_type == "primary":
            # Primary is always at origin
            return LandingZone(
                name=name,
                position=Vector3(0, 0, 0.02),
                radius=4.0,
                zone_type="primary",
            )

        radius = 3.0 if zone_type == "secondary" else 4.0

        # Find position away from center and obstacles
        pos = self._find_valid_position(
            radius,
            min_distance=10,
            avoid_center=True,
            center_exclusion_radius=50,
        )
        if pos is None:
            return None

        # Check distance from other landing zones
        for existing in existing_lz:
            dist = math.sqrt(
                (pos.x - existing.position.x) ** 2 + (pos.y - existing.position.y) ** 2
            )
            if dist < radius + existing.radius + 30:
                return None

        pos.z = 0.02
        return LandingZone(name=name, position=pos, radius=radius, zone_type=zone_type)

    def generate_world(
        self,
        name: str,
        difficulty: str = "medium",
        lighting: str = "day",
        wind_enabled: bool = True,
        fog_enabled: bool = False,
    ) -> WorldConfig:
        """Generate a complete world configuration."""
        preset = DIFFICULTY_PRESETS.get(difficulty, DIFFICULTY_PRESETS["medium"])

        num_obstacles = self.rng.randint(*preset["num_obstacles"])
        num_nfz = self.rng.randint(*preset["num_nfz"])
        max_height = preset["max_obstacle_height"]

        # Reset placed obstacles
        self.placed_obstacles = []

        # Generate obstacles with variety
        obstacle_types = ["pole", "building", "tree", "wall", "tank"]
        weights = [0.25, 0.25, 0.2, 0.15, 0.15]
        obstacles = []

        for i in range(num_obstacles):
            obs_type = self.rng.choices(obstacle_types, weights=weights)[0]
            obstacle = self.generate_obstacle(
                f"obstacle_{obs_type}_{i}",
                obs_type,
                max_height=max_height,
            )
            if obstacle:
                obstacles.append(obstacle)

        # Generate NFZs
        nfzs = []
        for i in range(num_nfz):
            nfz = self.generate_nfz(f"nfz_{i}", nfzs)
            if nfz:
                nfzs.append(nfz)

        # Generate landing zones
        landing_zones = []
        landing_zones.append(
            self.generate_landing_zone("lz_primary", "primary", landing_zones)
        )
        secondary = self.generate_landing_zone("lz_secondary", "secondary", landing_zones)
        if secondary:
            landing_zones.append(secondary)
        emergency = self.generate_landing_zone("lz_emergency", "emergency", landing_zones)
        if emergency:
            landing_zones.append(emergency)

        # Wind configuration
        wind_speed = Vector3(0, 0, 0)
        if wind_enabled:
            wind_speed = Vector3(
                self.rng.uniform(0, 5),
                self.rng.uniform(-2, 2),
                0,
            )

        fog_density = 0.0
        if fog_enabled:
            fog_density = self.rng.uniform(0.1, 0.4)

        return WorldConfig(
            name=name,
            seed=self.seed,
            difficulty=difficulty,
            geofence_size=self.geofence_size,
            max_altitude=self.max_altitude,
            num_obstacles=len(obstacles),
            num_nfz=len(nfzs),
            num_landing_zones=len(landing_zones),
            wind_enabled=wind_enabled,
            wind_speed=wind_speed,
            fog_enabled=fog_enabled,
            fog_density=fog_density,
            lighting_preset=lighting,
            obstacles=obstacles,
            no_fly_zones=nfzs,
            landing_zones=[lz for lz in landing_zones if lz is not None],
        )


class SDFWriter:
    """Converts WorldConfig to Gazebo SDF format."""

    def __init__(self, world_config: WorldConfig):
        self.config = world_config
        self.lighting = LIGHTING_PRESETS.get(
            world_config.lighting_preset, LIGHTING_PRESETS["day"]
        )

    def _create_element(self, tag: str, text: Optional[str] = None, **attrs) -> ET.Element:
        """Create an XML element with optional text and attributes."""
        elem = ET.Element(tag, attrs)
        if text is not None:
            elem.text = str(text)
        return elem

    def _add_subelement(
        self, parent: ET.Element, tag: str, text: Optional[str] = None, **attrs
    ) -> ET.Element:
        """Add a subelement to a parent element."""
        elem = ET.SubElement(parent, tag, **attrs)
        if text is not None:
            elem.text = str(text)
        return elem

    def _color_to_string(self, color: Tuple[float, float, float], alpha: float = 1.0) -> str:
        """Convert color tuple to SDF string format."""
        return f"{color[0]} {color[1]} {color[2]} {alpha}"

    def _generate_physics(self) -> ET.Element:
        """Generate physics configuration element."""
        physics = self._create_element("physics", name="training_physics", type="ode")
        self._add_subelement(physics, "max_step_size", "0.001")
        self._add_subelement(physics, "real_time_factor", "1.0")
        self._add_subelement(physics, "real_time_update_rate", "1000")

        ode = self._add_subelement(physics, "ode")
        solver = self._add_subelement(ode, "solver")
        self._add_subelement(solver, "type", "quick")
        self._add_subelement(solver, "iters", "50")
        self._add_subelement(solver, "sor", "1.3")

        constraints = self._add_subelement(ode, "constraints")
        self._add_subelement(constraints, "cfm", "0.0")
        self._add_subelement(constraints, "erp", "0.2")

        return physics

    def _generate_plugins(self) -> List[ET.Element]:
        """Generate system plugin elements."""
        plugins = [
            ("gz-sim-physics-system", "gz::sim::systems::Physics"),
            ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
            ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
            ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
            ("gz-sim-imu-system", "gz::sim::systems::Imu"),
            ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
            ("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer"),
            ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
            ("gz-sim-contact-system", "gz::sim::systems::Contact"),
        ]

        elements = []
        for filename, name in plugins:
            plugin = self._create_element("plugin", filename=filename, name=name)
            if "sensors" in filename:
                self._add_subelement(plugin, "render_engine", "ogre2")
            elements.append(plugin)

        # Wind effects plugin if enabled
        if self.config.wind_enabled:
            wind_plugin = self._create_element(
                "plugin",
                filename="gz-sim-wind-effects-system",
                name="gz::sim::systems::WindEffects",
            )
            self._add_subelement(wind_plugin, "force_approximation_scaling_factor", "1.0")
            elements.append(wind_plugin)

        return elements

    def _generate_environment(self) -> List[ET.Element]:
        """Generate atmosphere, wind, and spherical coordinates."""
        elements = []

        # Atmosphere
        atmosphere = self._create_element("atmosphere", type="adiabatic")
        self._add_subelement(atmosphere, "temperature", "288.15")
        self._add_subelement(atmosphere, "pressure", "101325")
        self._add_subelement(atmosphere, "mass_density", "1.225")
        elements.append(atmosphere)

        # Wind
        if self.config.wind_enabled:
            wind = self._create_element("wind")
            ws = self.config.wind_speed
            self._add_subelement(wind, "linear_velocity", f"{ws.x} {ws.y} {ws.z}")
            elements.append(wind)

        # Spherical coordinates (Camp Pendleton)
        coords = self._create_element("spherical_coordinates")
        self._add_subelement(coords, "surface_model", "EARTH_WGS84")
        self._add_subelement(coords, "world_frame_orientation", "ENU")
        self._add_subelement(coords, "latitude_deg", "33.3853")
        self._add_subelement(coords, "longitude_deg", "-117.5653")
        self._add_subelement(coords, "elevation", "100")
        self._add_subelement(coords, "heading_deg", "0")
        elements.append(coords)

        return elements

    def _generate_scene(self) -> ET.Element:
        """Generate scene configuration."""
        scene = self._create_element("scene")
        ambient = self.lighting["ambient"]
        background = self.lighting["background"]
        self._add_subelement(scene, "ambient", f"{ambient[0]} {ambient[1]} {ambient[2]} 1")
        self._add_subelement(scene, "background", f"{background[0]} {background[1]} {background[2]} 1")
        self._add_subelement(scene, "shadows", "true")
        self._add_subelement(scene, "grid", "false")
        return scene

    def _generate_lighting(self) -> List[ET.Element]:
        """Generate lighting elements."""
        elements = []

        # Main sun
        sun = self._create_element("light", type="directional", name="sun")
        self._add_subelement(sun, "cast_shadows", "true")
        self._add_subelement(sun, "pose", "0 0 100 0 0 0")

        diffuse = self.lighting["sun_diffuse"]
        specular = self.lighting["sun_specular"]
        direction = self.lighting["sun_direction"]

        self._add_subelement(sun, "diffuse", f"{diffuse[0]} {diffuse[1]} {diffuse[2]} 1")
        self._add_subelement(sun, "specular", f"{specular[0]} {specular[1]} {specular[2]} 1")

        attenuation = self._add_subelement(sun, "attenuation")
        self._add_subelement(attenuation, "range", "1000")
        self._add_subelement(attenuation, "constant", "0.9")
        self._add_subelement(attenuation, "linear", "0.01")
        self._add_subelement(attenuation, "quadratic", "0.001")

        self._add_subelement(sun, "direction", f"{direction[0]} {direction[1]} {direction[2]}")
        elements.append(sun)

        return elements

    def _generate_ground(self) -> ET.Element:
        """Generate ground plane model."""
        model = self._create_element("model", name="ground_plane")
        self._add_subelement(model, "static", "true")

        link = self._add_subelement(model, "link", name="link")

        # Collision
        collision = self._add_subelement(link, "collision", name="collision")
        geom = self._add_subelement(collision, "geometry")
        plane = self._add_subelement(geom, "plane")
        self._add_subelement(plane, "normal", "0 0 1")
        self._add_subelement(plane, "size", "500 500")

        surface = self._add_subelement(collision, "surface")
        friction = self._add_subelement(surface, "friction")
        ode = self._add_subelement(friction, "ode")
        self._add_subelement(ode, "mu", "100")
        self._add_subelement(ode, "mu2", "50")

        # Visual
        visual = self._add_subelement(link, "visual", name="visual")
        geom = self._add_subelement(visual, "geometry")
        plane = self._add_subelement(geom, "plane")
        self._add_subelement(plane, "normal", "0 0 1")
        self._add_subelement(plane, "size", "500 500")

        material = self._add_subelement(visual, "material")
        self._add_subelement(material, "ambient", "0.35 0.4 0.3 1")
        self._add_subelement(material, "diffuse", "0.35 0.4 0.3 1")

        return model

    def _generate_geofence(self) -> List[ET.Element]:
        """Generate geofence boundary markers."""
        elements = []
        half = self.config.geofence_size / 2
        size = self.config.geofence_size

        boundaries = [
            ("geofence_north", (0, half, 1), (size, 0.5, 2)),
            ("geofence_south", (0, -half, 1), (size, 0.5, 2)),
            ("geofence_east", (half, 0, 1), (0.5, size, 2)),
            ("geofence_west", (-half, 0, 1), (0.5, size, 2)),
        ]

        for name, pos, box_size in boundaries:
            model = self._create_element("model", name=name)
            self._add_subelement(model, "static", "true")
            self._add_subelement(model, "pose", f"{pos[0]} {pos[1]} {pos[2]} 0 0 0")

            link = self._add_subelement(model, "link", name="link")
            visual = self._add_subelement(link, "visual", name="visual")

            geom = self._add_subelement(visual, "geometry")
            box = self._add_subelement(geom, "box")
            self._add_subelement(box, "size", f"{box_size[0]} {box_size[1]} {box_size[2]}")

            material = self._add_subelement(visual, "material")
            self._add_subelement(material, "ambient", "0 0.8 0 0.6")
            self._add_subelement(material, "diffuse", "0 0.8 0 0.6")
            self._add_subelement(material, "emissive", "0 0.2 0 1")

            elements.append(model)

        return elements

    def _generate_obstacle_model(self, obstacle: Obstacle) -> ET.Element:
        """Generate model element for an obstacle."""
        model = self._create_element("model", name=obstacle.name)
        self._add_subelement(model, "static", "true")
        self._add_subelement(
            model,
            "pose",
            obstacle.position.to_pose_string(yaw=obstacle.rotation),
        )

        link = self._add_subelement(model, "link", name="link")

        if obstacle.obstacle_type == "tree":
            # Tree has trunk and canopy
            trunk_radius = obstacle.size.x
            canopy_radius = obstacle.size.y
            trunk_height = obstacle.size.z

            # Trunk visual
            trunk_visual = self._add_subelement(link, "visual", name="trunk")
            self._add_subelement(trunk_visual, "pose", f"0 0 {trunk_height/2} 0 0 0")
            geom = self._add_subelement(trunk_visual, "geometry")
            cyl = self._add_subelement(geom, "cylinder")
            self._add_subelement(cyl, "radius", str(trunk_radius))
            self._add_subelement(cyl, "length", str(trunk_height))
            material = self._add_subelement(trunk_visual, "material")
            trunk_color = random.choice(OBSTACLE_COLORS["tree_trunk"])
            self._add_subelement(material, "ambient", self._color_to_string(trunk_color))
            self._add_subelement(material, "diffuse", self._color_to_string(trunk_color))

            # Canopy visual
            canopy_visual = self._add_subelement(link, "visual", name="canopy")
            canopy_z = trunk_height + canopy_radius * 0.6
            self._add_subelement(canopy_visual, "pose", f"0 0 {canopy_z} 0 0 0")
            geom = self._add_subelement(canopy_visual, "geometry")
            sphere = self._add_subelement(geom, "sphere")
            self._add_subelement(sphere, "radius", str(canopy_radius))
            material = self._add_subelement(canopy_visual, "material")
            self._add_subelement(material, "ambient", self._color_to_string(obstacle.color))
            self._add_subelement(material, "diffuse", self._color_to_string(obstacle.color))

            # Collision (simplified cylinder)
            collision = self._add_subelement(link, "collision", name="collision")
            total_height = trunk_height + canopy_radius * 2
            self._add_subelement(collision, "pose", f"0 0 {total_height/2} 0 0 0")
            geom = self._add_subelement(collision, "geometry")
            cyl = self._add_subelement(geom, "cylinder")
            self._add_subelement(cyl, "radius", str(canopy_radius))
            self._add_subelement(cyl, "length", str(total_height))

        elif obstacle.obstacle_type == "cylinder":
            visual = self._add_subelement(link, "visual", name="visual")
            geom = self._add_subelement(visual, "geometry")
            cyl = self._add_subelement(geom, "cylinder")
            self._add_subelement(cyl, "radius", str(obstacle.size.x))
            self._add_subelement(cyl, "length", str(obstacle.size.z))
            material = self._add_subelement(visual, "material")
            self._add_subelement(material, "ambient", self._color_to_string(obstacle.color))
            self._add_subelement(material, "diffuse", self._color_to_string(obstacle.color))

            collision = self._add_subelement(link, "collision", name="collision")
            geom = self._add_subelement(collision, "geometry")
            cyl = self._add_subelement(geom, "cylinder")
            self._add_subelement(cyl, "radius", str(obstacle.size.x))
            self._add_subelement(cyl, "length", str(obstacle.size.z))

        elif obstacle.obstacle_type == "box":
            visual = self._add_subelement(link, "visual", name="visual")
            geom = self._add_subelement(visual, "geometry")
            box = self._add_subelement(geom, "box")
            self._add_subelement(box, "size", f"{obstacle.size.x} {obstacle.size.y} {obstacle.size.z}")
            material = self._add_subelement(visual, "material")
            self._add_subelement(material, "ambient", self._color_to_string(obstacle.color))
            self._add_subelement(material, "diffuse", self._color_to_string(obstacle.color))

            collision = self._add_subelement(link, "collision", name="collision")
            geom = self._add_subelement(collision, "geometry")
            box = self._add_subelement(geom, "box")
            self._add_subelement(box, "size", f"{obstacle.size.x} {obstacle.size.y} {obstacle.size.z}")

        return model

    def _generate_nfz_model(self, nfz: NoFlyZone) -> List[ET.Element]:
        """Generate NFZ volume and optional ground marker."""
        elements = []

        # NFZ volume (translucent)
        volume = self._create_element("model", name=f"{nfz.name}_volume")
        self._add_subelement(volume, "static", "true")
        self._add_subelement(volume, "pose", nfz.position.to_pose_string())

        link = self._add_subelement(volume, "link", name="link")
        visual = self._add_subelement(link, "visual", name="visual")
        geom = self._add_subelement(visual, "geometry")

        if nfz.zone_type == "cylinder":
            cyl = self._add_subelement(geom, "cylinder")
            self._add_subelement(cyl, "radius", str(nfz.size.x))
            self._add_subelement(cyl, "length", str(nfz.size.z))
        else:
            box = self._add_subelement(geom, "box")
            self._add_subelement(box, "size", f"{nfz.size.x} {nfz.size.y} {nfz.size.z}")

        material = self._add_subelement(visual, "material")
        alpha = 0.2 + (nfz.priority * 0.05)
        self._add_subelement(material, "ambient", f"1 0 0 {alpha}")
        self._add_subelement(material, "diffuse", f"1 0 0 {alpha}")

        elements.append(volume)

        return elements

    def _generate_landing_zone_model(self, lz: LandingZone) -> ET.Element:
        """Generate landing zone model."""
        model = self._create_element("model", name=lz.name)
        self._add_subelement(model, "static", "true")
        self._add_subelement(model, "pose", lz.position.to_pose_string())

        link = self._add_subelement(model, "link", name="link")

        # Pad
        visual = self._add_subelement(link, "visual", name="pad")
        geom = self._add_subelement(visual, "geometry")
        cyl = self._add_subelement(geom, "cylinder")
        self._add_subelement(cyl, "radius", str(lz.radius))
        self._add_subelement(cyl, "length", "0.04")

        material = self._add_subelement(visual, "material")
        if lz.zone_type == "primary":
            self._add_subelement(material, "ambient", "0.9 0.9 0.9 1")
            self._add_subelement(material, "diffuse", "0.9 0.9 0.9 1")
        elif lz.zone_type == "emergency":
            self._add_subelement(material, "ambient", "1 0.5 0 1")
            self._add_subelement(material, "diffuse", "1 0.5 0 1")
        else:
            self._add_subelement(material, "ambient", "0.85 0.85 0.85 1")
            self._add_subelement(material, "diffuse", "0.85 0.85 0.85 1")

        # Add H marker for primary
        if lz.zone_type == "primary":
            for name, size in [("H_v", "0.6 2.5 0.01"), ("H_h", "2.5 0.6 0.01")]:
                marker = self._add_subelement(link, "visual", name=name)
                self._add_subelement(marker, "pose", "0 0 0.02 0 0 0")
                geom = self._add_subelement(marker, "geometry")
                box = self._add_subelement(geom, "box")
                self._add_subelement(box, "size", size)
                mat = self._add_subelement(marker, "material")
                self._add_subelement(mat, "ambient", "0 0 0.8 1")
                self._add_subelement(mat, "diffuse", "0 0 0.8 1")

        return model

    def generate_sdf(self) -> str:
        """Generate complete SDF world file."""
        # Root SDF element
        sdf = ET.Element("sdf", version="1.9")

        # Add comment with generation info
        comment = ET.Comment(
            f"\n  Generated World: {self.config.name}\n"
            f"  Seed: {self.config.seed}\n"
            f"  Difficulty: {self.config.difficulty}\n"
            f"  Obstacles: {self.config.num_obstacles}\n"
            f"  NFZs: {self.config.num_nfz}\n"
            f"  Generated: {datetime.now().isoformat()}\n"
            f"  Generator: Flyby F-11 World Generator v1.0.0\n"
        )
        sdf.insert(0, comment)

        # World element
        world = self._add_subelement(sdf, "world", name=self.config.name)

        # Physics
        world.append(self._generate_physics())

        # Plugins
        for plugin in self._generate_plugins():
            world.append(plugin)

        # Environment
        for elem in self._generate_environment():
            world.append(elem)

        # Scene
        world.append(self._generate_scene())

        # Lighting
        for light in self._generate_lighting():
            world.append(light)

        # Ground
        world.append(self._generate_ground())

        # Geofence
        for fence in self._generate_geofence():
            world.append(fence)

        # NFZs
        for nfz in self.config.no_fly_zones:
            for elem in self._generate_nfz_model(nfz):
                world.append(elem)

        # Obstacles
        for obstacle in self.config.obstacles:
            world.append(self._generate_obstacle_model(obstacle))

        # Landing zones
        for lz in self.config.landing_zones:
            world.append(self._generate_landing_zone_model(lz))

        # UAV include
        include = self._add_subelement(world, "include")
        self._add_subelement(include, "uri", "model://iris_with_ardupilot")
        self._add_subelement(include, "name", "iris")
        self._add_subelement(include, "pose", "0 0 0.5 0 0 0")

        # Format and return
        rough_string = ET.tostring(sdf, encoding="unicode")
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")


def generate_worlds(
    num_worlds: int,
    output_dir: Path,
    base_seed: int,
    difficulty: str,
    lighting_variations: bool = True,
    wind_variations: bool = True,
) -> List[Path]:
    """Generate multiple world variants."""
    output_dir.mkdir(parents=True, exist_ok=True)
    generated_files = []

    lightings = ["day", "dusk", "overcast"] if lighting_variations else ["day"]

    for i in range(num_worlds):
        seed = base_seed + i
        generator = WorldGenerator(seed=seed)

        lighting = lightings[i % len(lightings)] if lighting_variations else "day"
        wind_enabled = (i % 3 != 0) if wind_variations else True

        world_config = generator.generate_world(
            name=f"training_variant_{i:03d}",
            difficulty=difficulty,
            lighting=lighting,
            wind_enabled=wind_enabled,
            fog_enabled=False,
        )

        writer = SDFWriter(world_config)
        sdf_content = writer.generate_sdf()

        # Write SDF file
        filename = f"variant_{i:03d}_seed{seed}_{difficulty}_{lighting}.sdf"
        filepath = output_dir / filename
        with open(filepath, "w") as f:
            f.write(sdf_content)

        generated_files.append(filepath)
        print(f"Generated: {filepath.name}")

        # Write config JSON
        config_filename = filepath.with_suffix(".json")
        config_data = {
            "name": world_config.name,
            "seed": world_config.seed,
            "difficulty": world_config.difficulty,
            "lighting": world_config.lighting_preset,
            "wind_enabled": world_config.wind_enabled,
            "wind_speed": {
                "x": world_config.wind_speed.x,
                "y": world_config.wind_speed.y,
                "z": world_config.wind_speed.z,
            },
            "num_obstacles": world_config.num_obstacles,
            "num_nfz": world_config.num_nfz,
            "num_landing_zones": world_config.num_landing_zones,
            "geofence_size": world_config.geofence_size,
            "max_altitude": world_config.max_altitude,
        }
        with open(config_filename, "w") as f:
            json.dump(config_data, f, indent=2)

    return generated_files


def main():
    parser = argparse.ArgumentParser(
        description="Generate Gazebo world variants for UAV RL training",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Generate 10 medium difficulty worlds:
    python generate_world_variants.py --num-worlds 10

  Generate hard difficulty worlds with specific seed:
    python generate_world_variants.py --difficulty hard --seed 12345

  Generate to custom directory:
    python generate_world_variants.py --output-dir ./my_worlds --num-worlds 5
        """,
    )

    parser.add_argument(
        "--num-worlds",
        type=int,
        default=10,
        help="Number of world variants to generate (default: 10)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Base random seed for reproducibility (default: 42)",
    )
    parser.add_argument(
        "--difficulty",
        choices=["easy", "medium", "hard", "extreme"],
        default="medium",
        help="Difficulty preset (default: medium)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).parent.parent / "worlds" / "generated",
        help="Output directory for generated worlds",
    )
    parser.add_argument(
        "--no-lighting-variation",
        action="store_true",
        help="Disable lighting variations (all worlds use 'day')",
    )
    parser.add_argument(
        "--no-wind-variation",
        action="store_true",
        help="Disable wind variations (all worlds have wind)",
    )
    parser.add_argument(
        "--geofence-size",
        type=float,
        default=300.0,
        help="Geofence size in meters (default: 300)",
    )

    args = parser.parse_args()

    print(f"Generating {args.num_worlds} world variants...")
    print(f"  Difficulty: {args.difficulty}")
    print(f"  Base seed: {args.seed}")
    print(f"  Output: {args.output_dir}")
    print()

    generated = generate_worlds(
        num_worlds=args.num_worlds,
        output_dir=args.output_dir,
        base_seed=args.seed,
        difficulty=args.difficulty,
        lighting_variations=not args.no_lighting_variation,
        wind_variations=not args.no_wind_variation,
    )

    print()
    print(f"Successfully generated {len(generated)} world files")
    print(f"Output directory: {args.output_dir}")


if __name__ == "__main__":
    main()
