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

        # Initialize spawners
        spawn_config = SpawnConfig(
            area_size=self.config.terrain_size,
            seed=self.config.seed,
        )

        self.vegetation = VegetationSpawner(
            self.stage, models_path, spawn_config
        )
        self.vehicles = VehicleSpawner(
            self.stage, models_path, spawn_config
        )
        self.people = PeopleSpawner(
            self.stage, models_path, spawn_config
        )

        # Track generated elements
        self.terrain_prim = None
        self.skybox_light = None

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

    def setup_lighting(
        self,
        hdri_path: str = None,
        sun_intensity: float = None,
    ) -> None:
        """
        Setup environment lighting with HDRI skybox and sun.

        Args:
            hdri_path: Path to HDR image (uses default if None)
            sun_intensity: Intensity of directional sun light
        """
        if hdri_path is None:
            hdri_path = f"{self.textures_path}/{self.DEFAULT_HDRI}"

        sun_intensity = sun_intensity or self.config.sun_intensity

        # Create dome light for sky background ONLY (no significant lighting contribution)
        # This provides the visible sky texture but doesn't contribute to scene lighting
        dome_path = "/World/Lighting/DomeLight"
        dome_light = UsdLux.DomeLight.Define(self.stage, dome_path)
        dome_light.CreateIntensityAttr(50)  # Very low - just for sky background visibility

        # Use clean sky HDRI for realistic background
        dome_light.CreateTextureFileAttr().Set(hdri_path)
        dome_light.CreateTextureFormatAttr().Set("latlong")

        # Disable shadow casting and color temperature on dome light
        dome_prim = dome_light.GetPrim()
        dome_prim.CreateAttribute("inputs:enableColorTemperature", Sdf.ValueTypeNames.Bool).Set(False)

        # Create distant light (sun) - THE ONLY PRIMARY LIGHT SOURCE
        sun_path = "/World/Lighting/SunLight"
        sun_light = UsdLux.DistantLight.Define(self.stage, sun_path)
        sun_light.CreateIntensityAttr(sun_intensity)
        sun_light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85))
        sun_light.CreateAngleAttr(0.53)  # Sun angular diameter

        # Rotate sun
        sun_xform = UsdGeom.Xformable(sun_light.GetPrim())
        sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(*self.config.sun_angle, 0.0))

        # Randomize lighting if enabled
        if self.config.randomize_lighting:
            # Random sun angle variation
            angle_x = self.config.sun_angle[0] + random.uniform(-15, 15)
            angle_y = self.config.sun_angle[1] + random.uniform(-30, 30)
            sun_xform.GetOrderedXformOps()[0].Set(Gf.Vec3f(angle_x, angle_y, 0.0))

            # Random intensity variation
            intensity_var = random.uniform(0.8, 1.2)
            sun_light.GetIntensityAttr().Set(sun_intensity * intensity_var)

        self.skybox_light = dome_path

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
            self.setup_lighting()

    def clear_spawned_objects(self) -> None:
        """Clear all spawned objects (vegetation, vehicles, people)."""
        self.vegetation.clear_all()
        self.vehicles.clear_all()
        self.people.clear_all()
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

    def get_stats(self) -> Dict[str, int]:
        """Get statistics about generated world."""
        return {
            "trees": self.vegetation.tree_count,
            "bushes": self.vegetation.bush_count,
            "vehicles": self.vehicles.vehicle_count,
            "people": self.people.person_count,
        }
