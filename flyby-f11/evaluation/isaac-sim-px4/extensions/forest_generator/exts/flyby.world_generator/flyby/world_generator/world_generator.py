"""
World Generator - Main API for procedural environment generation.

Provides a unified interface for:
- Terrain generation with materials
- HDRI skybox/lighting
- Vegetation, vehicles, and people spawning
- Domain randomization for RL training
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import random
import numpy as np

from pxr import Gf, UsdGeom, UsdShade, Sdf, UsdPhysics, PhysxSchema, UsdLux

import omni.usd
import omni.replicator.core as rep
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.prims import XFormPrim

from .spawners.base_spawner import SpawnConfig, BaseSpawner
from .spawners.vegetation_spawner import VegetationSpawner
from .spawners.vehicle_spawner import VehicleSpawner
from .spawners.people_spawner import PeopleSpawner
from .spawners.drone_spawner import DroneSpawner


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


@dataclass
class WorldConfig:
    """Configuration for world generation."""
    # Terrain
    terrain_size: Tuple[float, float] = (200.0, 200.0)
    terrain_roughness: float = 3.0
    terrain_material: str = "forest_floor"

    # Lighting/Sky
    hdri_path: str = None
    sun_intensity: float = 3000.0
    sun_angle: Tuple[float, float] = (-45.0, 30.0)
    time_of_day: Optional[str] = None  # "dawn", "morning", "noon", "afternoon", "dusk", "overcast"

    # Weather/Atmosphere
    weather: Optional[str] = None  # "clear", "light_haze", "moderate_haze", "heavy_haze"

    # Vegetation (lower defaults for performance)
    tree_density: float = 1.5  # Trees per 100m² (reduced from 3.0)
    tree_proportions: Dict[str, float] = field(default_factory=lambda: {
        "Birch": 30, "Spruce": 40, "Pine": 30
    })
    undergrowth_density: float = 0.0  # Disabled by default for performance

    # Domain randomization
    randomize_lighting: bool = True
    randomize_weather: bool = False
    randomize_terrain: bool = True

    # Spawning
    seed: Optional[int] = None


class WorldGenerator:
    """
    Main class for procedural world generation.

    Provides:
    - Terrain generation with Perlin noise
    - Material application with PBR textures
    - HDRI skybox for realistic lighting
    - Vegetation, vehicle, and people spawning
    - Domain randomization for RL training
    """

    # Default texture paths (relative to models/textures/)
    DEFAULT_MATERIALS = {
        "forest_floor": {
            "diffuse": "forest_floor_diff_2k.jpg",
            "normal": "forest_floor_nor_gl_2k.exr",
            "roughness": "forest_floor_rough_2k.exr",
            "displacement": "forest_floor_disp_2k.png",
        },
        "grass": {
            "diffuse": "forest_floor_diff_2k.jpg",
        },
    }

    DEFAULT_HDRI = "sky_2k.hdr"  # Clean sky HDRI for aerial training

    def __init__(
        self,
        models_path: str,
        config: WorldConfig = None,
    ):
        """
        Initialize WorldGenerator.

        Args:
            models_path: Path to models directory
            config: World configuration
        """
        self.models_path = models_path
        self.textures_path = f"{models_path}/textures"
        self.config = config or WorldConfig()

        if self.config.seed is not None:
            random.seed(self.config.seed)
            np.random.seed(self.config.seed)

        # Get stage
        self.stage = get_current_stage()

        # Create world structure
        self._init_world_structure()

        # Initialize spawners with appropriate spacing for each type
        # Vegetation can be close together
        veg_spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
            min_spacing=1.0,
        )
        # Vehicles need more space to prevent physics collision issues
        # Tanks are ~6-8m long, so we need at least 8m spacing
        vehicle_spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
            min_spacing=8.0,
        )
        # People need moderate spacing
        people_spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
            min_spacing=1.5,
        )

        self.vegetation = VegetationSpawner(
            self.stage, models_path, veg_spawn_config
        )
        self.vehicles = VehicleSpawner(
            self.stage, models_path, vehicle_spawn_config
        )
        self.people = PeopleSpawner(
            self.stage, models_path, people_spawn_config
        )
        # Drones need space similar to vehicles
        drone_spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
            min_spacing=5.0,
        )
        self.drones = DroneSpawner(
            self.stage, models_path, drone_spawn_config
        )

        # Track generated elements
        self.terrain_prim = None
        self.skybox_light = None
        self._current_weather: WeatherConfig = WEATHER_PRESETS["clear"]
        self._weather_name: str = "clear"
        self._current_time_of_day: Optional[str] = None

    def _init_world_structure(self) -> None:
        """Create base prim structure."""
        paths = [
            "/World",
            "/World/Terrain",
            "/World/Vegetation",
            "/World/Vegetation/Trees",
            "/World/Vegetation/Bushes",
            "/World/Vehicles",
            "/World/People",
            "/World/Lighting",
        ]

        for path in paths:
            if not self.stage.GetPrimAtPath(path):
                prim_utils.create_prim(path, "Xform")

    def generate_terrain(
        self,
        size: Tuple[float, float] = None,
        roughness: float = None,
        apply_material: bool = True,
    ) -> str:
        """
        Generate procedural terrain mesh.

        Args:
            size: (width, length) in meters
            roughness: Height variation in meters
            apply_material: Whether to apply ground material

        Returns:
            Prim path of terrain
        """
        size = size or self.config.terrain_size
        roughness = roughness if roughness is not None else self.config.terrain_roughness

        try:
            import noise
        except ImportError:
            print("Warning: 'noise' module not available, creating flat terrain")
            return self._create_flat_terrain(size)

        # Terrain mesh parameters
        horizontal_scale = 0.5  # meters per vertex
        vertical_scale = 0.1
        num_rows = int(size[0] / horizontal_scale)
        num_cols = int(size[1] / horizontal_scale)

        # Generate Perlin noise heightfield
        seed = random.randint(0, 10000) if self.config.randomize_terrain else 42
        heightfield = np.zeros((num_rows, num_cols), dtype=np.float32)

        for i in range(num_rows):
            for j in range(num_cols):
                x = i / num_rows * 4
                y = j / num_cols * 4
                heightfield[i, j] = noise.pnoise2(
                    x, y, octaves=6, base=seed
                ) * roughness

        # Convert to mesh
        vertices, triangles = self._heightfield_to_mesh(
            heightfield, horizontal_scale, size
        )

        # Create terrain prim
        terrain_path = "/World/Terrain/Ground"
        terrain_mesh = self.stage.DefinePrim(terrain_path, "Mesh")

        terrain_mesh.GetAttribute("points").Set(vertices)
        terrain_mesh.GetAttribute("faceVertexIndices").Set(triangles.flatten())
        terrain_mesh.GetAttribute("faceVertexCounts").Set(
            np.array([3] * len(triangles), dtype=np.int32)
        )

        # Generate UVs for texturing
        uvs = self._generate_uvs(num_rows, num_cols, size)
        terrain_mesh.GetAttribute("primvars:st").Set(uvs)

        # Position at center
        terrain_xform = XFormPrim(
            prim_path=terrain_path,
            name="ground",
            position=np.array([-size[0]/2, -size[1]/2, 0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        # Add collision
        self._add_terrain_collision(terrain_path)

        # Apply material
        if apply_material:
            self.apply_ground_material(terrain_path, self.config.terrain_material)

        self.terrain_prim = terrain_path
        return terrain_path

    def _create_flat_terrain(self, size: Tuple[float, float]) -> str:
        """Create simple flat terrain (fallback)."""
        vertices = np.array([
            [-size[0]/2, -size[1]/2, 0],
            [size[0]/2, -size[1]/2, 0],
            [size[0]/2, size[1]/2, 0],
            [-size[0]/2, size[1]/2, 0],
        ], dtype=np.float32)

        triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)

        terrain_path = "/World/Terrain/Ground"
        terrain_mesh = self.stage.DefinePrim(terrain_path, "Mesh")
        terrain_mesh.GetAttribute("points").Set(vertices)
        terrain_mesh.GetAttribute("faceVertexIndices").Set(triangles.flatten())
        terrain_mesh.GetAttribute("faceVertexCounts").Set(np.array([3, 3]))

        self._add_terrain_collision(terrain_path)

        # Apply ground material (green grass color)
        self.apply_ground_material(terrain_path, self.config.terrain_material)

        self.terrain_prim = terrain_path
        return terrain_path

    def _heightfield_to_mesh(
        self,
        heightfield: np.ndarray,
        horizontal_scale: float,
        size: Tuple[float, float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Convert heightfield to mesh vertices and triangles."""
        num_rows, num_cols = heightfield.shape

        # Generate vertex grid
        x = np.linspace(0, size[0], num_rows)
        y = np.linspace(0, size[1], num_cols)
        yy, xx = np.meshgrid(y, x)

        vertices = np.zeros((num_rows * num_cols, 3), dtype=np.float32)
        vertices[:, 0] = xx.flatten()
        vertices[:, 1] = yy.flatten()
        vertices[:, 2] = heightfield.flatten()

        # Generate triangles
        triangles = []
        for i in range(num_rows - 1):
            for j in range(num_cols - 1):
                idx = i * num_cols + j
                triangles.append([idx, idx + num_cols + 1, idx + 1])
                triangles.append([idx, idx + num_cols, idx + num_cols + 1])

        return vertices, np.array(triangles, dtype=np.uint32)

    def _generate_uvs(
        self,
        num_rows: int,
        num_cols: int,
        size: Tuple[float, float],
    ) -> np.ndarray:
        """Generate UV coordinates for terrain texturing."""
        # Tile UVs based on terrain size (1 tile per 10 meters)
        tile_scale = 10.0

        u = np.linspace(0, size[0] / tile_scale, num_cols)
        v = np.linspace(0, size[1] / tile_scale, num_rows)
        uu, vv = np.meshgrid(u, v)

        uvs = np.zeros((num_rows * num_cols, 2), dtype=np.float32)
        uvs[:, 0] = uu.flatten()
        uvs[:, 1] = vv.flatten()

        return uvs

    def _add_terrain_collision(self, prim_path: str) -> None:
        """Add collision properties to terrain."""
        prim = self.stage.GetPrimAtPath(prim_path)

        # Apply collision API
        UsdPhysics.CollisionAPI.Apply(prim)

        # Apply mesh collision with "none" approximation for static triangle mesh
        # Valid values: "none" (triangle mesh), "convexHull", "convexDecomposition",
        # "boundingCube", "boundingSphere", "meshSimplification", "sdf"
        collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        collision_api.CreateApproximationAttr().Set("none")

        # PhysX collision settings
        physx_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx_api.GetContactOffsetAttr().Set(0.001)
        physx_api.GetRestOffsetAttr().Set(0.0)

    def apply_ground_material(
        self,
        prim_path: str,
        material_name: str,
    ) -> None:
        """
        Apply PBR material to terrain.

        Args:
            prim_path: Path to terrain prim
            material_name: Name of material from DEFAULT_MATERIALS
        """
        if material_name not in self.DEFAULT_MATERIALS:
            print(f"Warning: Material '{material_name}' not found, using default")
            material_name = "forest_floor"

        mat_config = self.DEFAULT_MATERIALS[material_name]

        # Create material
        mat_path = f"/World/Looks/{material_name}_mat"
        material = UsdShade.Material.Define(self.stage, mat_path)

        # Create PBR shader
        shader_path = f"{mat_path}/Shader"
        shader = UsdShade.Shader.Define(self.stage, shader_path)
        shader.CreateIdAttr("UsdPreviewSurface")

        # Create UV primvar reader
        uv_reader = UsdShade.Shader.Define(self.stage, f"{mat_path}/uvReader")
        uv_reader.CreateIdAttr("UsdPrimvarReader_float2")
        uv_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        uv_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        # Deep forest green ground color
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.08, 0.18, 0.05)  # Deep forest green
        )

        # Set roughness
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)

        # Set metallic
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        # Connect shader to material surface
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )

        # Bind material to terrain
        terrain_prim = self.stage.GetPrimAtPath(prim_path)
        UsdShade.MaterialBindingAPI(terrain_prim).Bind(material)

    def _get_time_of_day_preset(self) -> Optional[TimeOfDayPreset]:
        """
        Get the time of day preset to use for lighting.

        Returns:
            TimeOfDayPreset if a preset should be used, None otherwise
        """
        # If time_of_day is explicitly set, use that preset
        if self.config.time_of_day is not None:
            if self.config.time_of_day in TIME_OF_DAY_PRESETS:
                return TIME_OF_DAY_PRESETS[self.config.time_of_day]
            else:
                print(f"Warning: Unknown time_of_day '{self.config.time_of_day}', "
                      f"valid options: {list(TIME_OF_DAY_PRESETS.keys())}")
                return None

        # If randomize_lighting is enabled and no time_of_day specified, pick randomly
        if self.config.randomize_lighting:
            preset_name = random.choice(list(TIME_OF_DAY_PRESETS.keys()))
            return TIME_OF_DAY_PRESETS[preset_name]

        return None

    def _color_temperature_to_rgb(self, kelvin: float) -> Tuple[float, float, float]:
        """
        Convert color temperature in Kelvin to RGB tint.

        Uses approximation for the visible range (1000K - 10000K).

        Args:
            kelvin: Color temperature in Kelvin

        Returns:
            Tuple of (R, G, B) values in 0-1 range
        """
        # Clamp to valid range
        kelvin = max(1000.0, min(10000.0, kelvin))

        # Normalize to 0-1 range for calculation
        temp = kelvin / 100.0

        # Calculate red
        if temp <= 66:
            red = 1.0
        else:
            red = temp - 60
            red = 329.698727446 * (red ** -0.1332047592)
            red = max(0.0, min(255.0, red)) / 255.0

        # Calculate green
        if temp <= 66:
            green = temp
            green = 99.4708025861 * np.log(max(1.0, green)) - 161.1195681661
            green = max(0.0, min(255.0, green)) / 255.0
        else:
            green = temp - 60
            green = 288.1221695283 * (green ** -0.0755148492)
            green = max(0.0, min(255.0, green)) / 255.0

        # Calculate blue
        if temp >= 66:
            blue = 1.0
        elif temp <= 19:
            blue = 0.0
        else:
            blue = temp - 10
            blue = 138.5177312231 * np.log(max(1.0, blue)) - 305.0447927307
            blue = max(0.0, min(255.0, blue)) / 255.0

        return (red, green, blue)

    def setup_lighting(
        self,
        hdri_path: str = None,
        sun_intensity: float = None,
    ) -> None:
        """
        Setup environment lighting with HDRI skybox and sun.

        Supports time-of-day presets for realistic lighting variation:
        - dawn: Low sun, warm orange tint, intensity ~1500
        - morning: Rising sun, neutral, intensity ~2500
        - noon: High sun, slightly cool, intensity ~3500
        - afternoon: Descending sun, warm golden, intensity ~3000
        - dusk: Very low sun, deep orange/red, intensity ~1200
        - overcast: No direct sun, high diffuse dome light

        Args:
            hdri_path: Path to HDR image (uses default if None)
            sun_intensity: Intensity of directional sun light (overridden by time_of_day preset)
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
            effective_sun_intensity = sun_intensity or self.config.sun_intensity
            dome_intensity = 1000.0
            sun_color = Gf.Vec3f(1.0, 0.95, 0.85)
            sun_elevation = self.config.sun_angle[0]
            sun_azimuth = self.config.sun_angle[1]
            color_temp = 5500.0  # Neutral daylight

        # Create dome light for sky background
        # Provides visible sky texture AND ambient lighting for realistic outdoor scenes
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
        temp_tint = self._color_temperature_to_rgb(color_temp)
        dome_light.CreateColorAttr(Gf.Vec3f(*temp_tint))

        # Disable shadow casting from dome light - only sun should cast shadows
        from pxr import UsdLux as UsdLuxShadow
        shadow_api = UsdLuxShadow.ShadowAPI.Apply(dome_prim)
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
        # (presets already have built-in variation through random elevation/azimuth selection)
        if self.config.randomize_lighting and not tod_preset:
            # Random sun angle variation
            angle_x = sun_elevation + random.uniform(-15, 15)
            angle_y = sun_azimuth + random.uniform(-30, 30)
            sun_xform.GetOrderedXformOps()[0].Set(Gf.Vec3f(angle_x, angle_y, 0.0))

            # Random intensity variation
            intensity_var = random.uniform(0.8, 1.2)
            sun_light.GetIntensityAttr().Set(effective_sun_intensity * intensity_var)

        self.skybox_light = dome_path

        # Apply atmosphere effects after lighting setup
        self.setup_atmosphere()

    def setup_atmosphere(
        self,
        weather: str = None,
    ) -> None:
        """
        Setup atmosphere/weather effects for domain randomization.

        This method configures fog, visibility, and lighting adjustments
        to simulate various atmospheric conditions. These effects are
        critical for sim-to-real transfer in aerial ISR scenarios where
        visibility conditions significantly impact detection performance.

        Weather presets:
        - clear: 10km visibility, no fog, full light intensity
        - light_haze: 5km visibility, slight fog, 90% light
        - moderate_haze: 2km visibility, noticeable fog, 75% light
        - heavy_haze: 800m visibility, significant fog, 55% light

        Args:
            weather: Weather preset name ("clear", "light_haze",
                    "moderate_haze", "heavy_haze"). If None, uses config
                    or randomizes if randomize_weather is enabled.
        """
        # Determine weather preset
        if weather is not None:
            weather_name = weather
        elif self.config.weather is not None:
            weather_name = self.config.weather
        elif self.config.randomize_weather:
            # Weighted random selection (more common conditions more likely)
            weights = [0.4, 0.3, 0.2, 0.1]  # clear more likely than heavy haze
            weather_options = ["clear", "light_haze", "moderate_haze", "heavy_haze"]
            weather_name = random.choices(weather_options, weights=weights)[0]
        else:
            weather_name = "clear"

        # Validate and get config
        if weather_name not in WEATHER_PRESETS:
            print(f"Warning: Unknown weather preset '{weather_name}', using 'clear'")
            weather_name = "clear"

        weather_config = WEATHER_PRESETS[weather_name]
        self._current_weather = weather_config
        self._weather_name = weather_name

        # Apply fog/atmosphere effects
        self._apply_fog(weather_config)

        # Adjust existing lights based on weather
        self._adjust_lights_for_weather(weather_config)

        if weather_name != "clear":
            print(f"[WorldGenerator] Atmosphere set to '{weather_name}' "
                  f"(visibility: {weather_config.visibility_range:.0f}m)")

    def _apply_fog(self, weather_config: WeatherConfig) -> None:
        """
        Apply fog/haze effects to the scene.

        Uses Isaac Sim's render settings for fog when available,
        with fallback to visual approximations via lighting.

        Args:
            weather_config: Weather configuration to apply
        """
        if weather_config.fog_density <= 0.0:
            # No fog needed, ensure any existing fog is cleared
            self._clear_fog()
            return

        # Try to use Isaac Sim's built-in fog settings via render settings
        try:
            import carb.settings
            settings = carb.settings.get_settings()

            # Calculate fog distances based on visibility
            fog_start = 10.0  # Start fog close for gradual effect
            fog_end = weather_config.visibility_range

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
            print(f"[WorldGenerator] RTX fog settings unavailable ({e}), "
                  "using lighting-based atmosphere simulation")

    def _clear_fog(self) -> None:
        """Clear any existing fog settings."""
        try:
            import carb.settings
            settings = carb.settings.get_settings()
            settings.set("/rtx/fog/enabled", False)
        except (ImportError, AttributeError):
            pass

    def _adjust_lights_for_weather(self, weather_config: WeatherConfig) -> None:
        """
        Adjust scene lights to simulate atmospheric scattering.

        Reduces light intensity and shifts colors to simulate
        the visual effects of haze and reduced visibility.

        Args:
            weather_config: Weather configuration to apply
        """
        # Adjust dome light (skybox)
        dome_path = "/World/Lighting/DomeLight"
        dome_prim = self.stage.GetPrimAtPath(dome_path)
        if dome_prim.IsValid():
            dome_light = UsdLux.DomeLight(dome_prim)

            # Get current intensity and apply weather multiplier
            current_intensity = dome_light.GetIntensityAttr().Get()
            if current_intensity:
                # Apply weather multiplier
                new_intensity = current_intensity * weather_config.light_intensity_multiplier
                dome_light.GetIntensityAttr().Set(new_intensity)

            # Apply slight color tint for haze (desaturated, bluish)
            if weather_config.fog_density > 0:
                # Blend dome light color toward fog color
                blend_factor = weather_config.fog_density * 0.5  # Subtle effect
                tint = Gf.Vec3f(
                    1.0 - blend_factor * (1.0 - weather_config.fog_color[0]),
                    1.0 - blend_factor * (1.0 - weather_config.fog_color[1]),
                    1.0 - blend_factor * (1.0 - weather_config.fog_color[2]),
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
                new_sun = current_intensity * weather_config.sun_intensity_multiplier
                sun_light.GetIntensityAttr().Set(new_sun)

            # Shift sun color toward warmer/hazier tones
            if weather_config.fog_density > 0:
                # Haze causes warm color shift and reduced contrast
                haze_factor = weather_config.fog_density * 0.3
                sun_color = Gf.Vec3f(
                    1.0,
                    0.95 - haze_factor * 0.1,
                    0.85 - haze_factor * 0.2,
                )
                sun_light.GetColorAttr().Set(sun_color)

    @property
    def visibility_range(self) -> float:
        """
        Current visibility range in meters.

        This property is useful for RL environments to:
        - Adjust reward functions based on sensing conditions
        - Scale detection thresholds appropriately
        - Log environment conditions for analysis

        Returns:
            Visibility range in meters (10000 = clear, 800 = heavy haze)
        """
        return self._current_weather.visibility_range

    @property
    def current_weather(self) -> str:
        """
        Current weather condition name.

        Returns:
            Weather preset name ("clear", "light_haze", etc.)
        """
        return self._weather_name

    @property
    def weather_cfg(self) -> WeatherConfig:
        """
        Current weather configuration object.

        Returns:
            Full WeatherConfig with all atmosphere parameters
        """
        return self._current_weather

    @property
    def time_of_day(self) -> Optional[str]:
        """
        Current time of day setting.

        Returns:
            Time of day preset name if set, None otherwise
        """
        return self._current_time_of_day

    def generate_forest(
        self,
        density: float = None,
        proportions: Dict[str, float] = None,
        include_undergrowth: bool = True,
    ) -> Dict[str, List[str]]:
        """
        Generate complete forest with trees and undergrowth.

        Args:
            density: Trees per 100m²
            proportions: Tree type proportions
            include_undergrowth: Whether to add bushes

        Returns:
            Dict with lists of spawned tree and bush paths
        """
        density = density or self.config.tree_density
        proportions = proportions or self.config.tree_proportions

        result = {"trees": [], "bushes": []}

        # Spawn trees
        result["trees"] = self.vegetation.spawn_forest(density, proportions)

        # Spawn undergrowth
        if include_undergrowth:
            result["bushes"] = self.vegetation.spawn_undergrowth(
                self.config.undergrowth_density
            )

        return result

    def generate_full_environment(self) -> Dict[str, Any]:
        """
        Generate complete environment with terrain, lighting, and vegetation.

        Returns:
            Dict with all generated element paths
        """
        result = {
            "terrain": None,
            "lighting": None,
            "forest": None,
        }

        # Generate terrain
        result["terrain"] = self.generate_terrain()

        # Setup lighting
        self.setup_lighting()
        result["lighting"] = "/World/Lighting"

        # Generate forest
        result["forest"] = self.generate_forest()

        return result

    def randomize_environment(self, seed: int = None) -> None:
        """
        Randomize environment for new episode.

        Supports randomization of:
        - Terrain (if randomize_terrain=True)
        - Lighting/time of day (if randomize_lighting=True)
        - Weather/atmosphere (if randomize_weather=True)

        Args:
            seed: Random seed for reproducibility
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
            self.config.seed = seed

        # Clear existing spawned objects
        self.clear_spawned_objects()

        # Regenerate with new random parameters
        if self.config.randomize_terrain:
            self.generate_terrain()

        if self.config.randomize_lighting:
            # setup_lighting() calls setup_atmosphere() automatically
            self.setup_lighting()
        elif self.config.randomize_weather:
            # Only weather randomization (lighting stays the same)
            self.setup_atmosphere()

    def spawn_drone(
        self,
        variant: str = "isr_camera",
        position: Tuple[float, float, float] = None,
        heading: float = 0.0,
    ) -> str:
        """
        Spawn an F-11 drone.

        Args:
            variant: Drone variant ("base", "isr_camera", "lidar", "multispectral")
            position: (x, y, z) spawn position in meters. If None, uses (0, 0, 10)
            heading: Yaw angle in radians

        Returns:
            Prim path of spawned drone
        """
        if position is None:
            position = (0.0, 0.0, 10.0)
        return self.drones.spawn(variant, position, heading)

    def spawn_drone_formation(
        self,
        variant: str = "isr_camera",
        center: Tuple[float, float, float] = (0, 0, 15),
        count: int = 1,
        spacing: float = 5.0,
        formation: str = "line",
    ) -> List[str]:
        """
        Spawn multiple drones in formation.

        Args:
            variant: Drone variant to spawn
            center: Center position (x, y, z) in meters
            count: Number of drones (default 1)
            spacing: Distance between drones in meters
            formation: "line", "wedge", "box", or "circle"

        Returns:
            List of spawned drone prim paths
        """
        return self.drones.spawn_formation(variant, center, count, spacing, formation)

    def spawn_poi_clusters(
        self,
        num_clusters: int = 3,
        targets_per_cluster: Tuple[int, int] = (3, 8),
        cluster_radius: float = 30.0,
        include_vehicles: bool = True,
        include_people: bool = True,
    ) -> List[Dict[str, Any]]:
        """
        Spawn realistic POI clusters for ISR training.

        Each cluster contains a mix of vehicles and people grouped together,
        simulating real-world scenarios (encampments, convoys, activity sites).
        This creates realistic detection scenarios where targets are clustered
        rather than uniformly distributed.

        Args:
            num_clusters: Number of clusters to spawn
            targets_per_cluster: (min, max) targets per cluster
            cluster_radius: Maximum spread of targets from cluster center
            include_vehicles: Whether to include vehicles
            include_people: Whether to include people

        Returns:
            List of cluster dicts with 'id', 'center', 'targets', 'radius' keys
        """
        clusters = []
        half_size = min(self.config.terrain_size) / 2 * 0.8  # 80% of area for margin

        for i in range(num_clusters):
            # Find non-overlapping cluster center
            center_x, center_y = None, None
            for _ in range(50):  # Max attempts to find valid center
                candidate_x = random.uniform(-half_size, half_size)
                candidate_y = random.uniform(-half_size, half_size)

                # Check distance from existing clusters
                if clusters:
                    min_dist = min(
                        np.sqrt((candidate_x - c['center'][0])**2 +
                               (candidate_y - c['center'][1])**2)
                        for c in clusters
                    )
                    if min_dist > cluster_radius * 2.5:
                        center_x, center_y = candidate_x, candidate_y
                        break
                else:
                    center_x, center_y = candidate_x, candidate_y
                    break

            if center_x is None:
                # Fallback: use random position
                center_x = random.uniform(-half_size, half_size)
                center_y = random.uniform(-half_size, half_size)

            center = (center_x, center_y)
            num_targets = random.randint(*targets_per_cluster)
            cluster_targets = []

            # Decide composition (mix of vehicles and people)
            if include_vehicles and include_people:
                num_vehicles = random.randint(1, max(1, num_targets // 2))
                num_people = num_targets - num_vehicles
            elif include_vehicles:
                num_vehicles, num_people = num_targets, 0
            else:
                num_vehicles, num_people = 0, num_targets

            # Spawn vehicles at explicit cluster center
            if num_vehicles > 0:
                vehicle_types = ["sedan", "sedan2", "suv", "tank", "tank2"]
                available_types = [vt for vt in vehicle_types
                                  if vt in self.vehicles.vehicle_configs]
                if available_types:
                    vehicle_paths = self.vehicles.spawn_vehicle_group(
                        vehicle_types=available_types,
                        count=num_vehicles,
                        clustering=0.8,  # Tight clustering
                        center=center,
                    )
                    for vp in vehicle_paths:
                        cluster_targets.append({'type': 'vehicle', 'path': vp})

            # Spawn people in cluster
            if num_people > 0 and self.people.person_configs:
                person_types = list(self.people.person_configs.keys())
                people_paths = self.people.spawn_crowd(
                    person_types=person_types,
                    count=num_people,
                    center=center,
                    radius=cluster_radius * 0.5,
                )
                for pp in people_paths:
                    cluster_targets.append({'type': 'person', 'path': pp})

            clusters.append({
                'id': f'cluster_{i}',
                'center': center,
                'radius': cluster_radius,
                'targets': cluster_targets,
                'num_targets': len(cluster_targets),
            })

        return clusters

    def clear_spawned_objects(self) -> None:
        """Clear all spawned objects (vegetation, vehicles, people, drones)."""
        self.vegetation.clear_all()
        self.vehicles.clear_all()
        self.people.clear_all()
        self.drones.clear_all()
        # Clear position registry for overlap prevention
        BaseSpawner.clear_position_registry()

    def clear_all(self) -> None:
        """Clear entire generated world."""
        self.clear_spawned_objects()

        # Remove terrain
        if self.terrain_prim:
            prim = self.stage.GetPrimAtPath(self.terrain_prim)
            if prim.IsValid():
                self.stage.RemovePrim(self.terrain_prim)
            self.terrain_prim = None

        # Remove lighting
        if self.skybox_light:
            prim = self.stage.GetPrimAtPath("/World/Lighting")
            if prim.IsValid():
                self.stage.RemovePrim("/World/Lighting")
            self.skybox_light = None

    def get_stats(self) -> Dict[str, Any]:
        """Get statistics about generated world."""
        return {
            "trees": self.vegetation.tree_count,
            "bushes": self.vegetation.bush_count,
            "vehicles": self.vehicles.vehicle_count,
            "people": self.people.person_count,
            "drones": self.drones.drone_count,
            "time_of_day": self._current_time_of_day,
            "weather": self._weather_name,
        }

    def get_current_time_of_day(self) -> Optional[str]:
        """
        Get the current time of day preset name.

        Returns:
            Name of current time of day preset, or None if using default lighting
        """
        return self._current_time_of_day
