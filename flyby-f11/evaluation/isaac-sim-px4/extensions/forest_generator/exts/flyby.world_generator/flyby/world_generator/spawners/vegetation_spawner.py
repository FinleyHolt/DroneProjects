"""
Vegetation spawner for trees, bushes, and other plants.

Uses simple procedural geometry (cones for trees) for performance
while maintaining realistic shapes for physics and occlusion.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional
import random

from pxr import Gf, UsdGeom, UsdShade, Sdf
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.core.utils.stage import add_reference_to_stage

from .base_spawner import BaseSpawner, SpawnConfig


@dataclass
class TreeConfig:
    """Configuration for tree spawning."""
    usd_path: str
    base_scale: float = 50.0
    scale_variation: tuple = (0.8, 1.2)
    height_multiplier: float = 1.0
    age_range: tuple = (0.5, 1.0)


@dataclass
class BushConfig:
    """Configuration for bush/undergrowth spawning."""
    usd_path: str
    base_scale: float = 1.0
    scale_variation: tuple = (0.4, 1.0)
    cluster_probability: float = 0.5
    cluster_size: int = 10
    cluster_spread: float = 0.4


class VegetationSpawner(BaseSpawner):
    """
    Spawner for procedural vegetation (trees, bushes, rocks).

    Handles:
    - Multiple tree species with different proportions
    - Undergrowth clustering
    - Terrain-following placement
    - Y-up to Z-up coordinate conversion
    """

    # Default model paths (relative to extension models directory)
    DEFAULT_TREES = {
        "Birch": TreeConfig(usd_path="Birch_obj/Birch.usd", height_multiplier=1.35),
        "Spruce": TreeConfig(usd_path="Spruce_obj/Spruce.usd", height_multiplier=1.25),
        "Pine": TreeConfig(usd_path="Pine_obj/Pine.usd", height_multiplier=1.35),
    }

    DEFAULT_BUSHES = {
        "Bush": BushConfig(usd_path="Bush_obj/Bush.usd"),
        "Blueberry": BushConfig(usd_path="Blueberry_obj/Blueberry.usd", cluster_probability=1.0),
    }

    def __init__(
        self,
        stage,
        models_path: str,
        config: SpawnConfig = None,
        tree_configs: Dict[str, TreeConfig] = None,
        bush_configs: Dict[str, BushConfig] = None,
    ):
        super().__init__(stage, config)
        self.models_path = models_path
        self.tree_configs = tree_configs or self._load_default_trees()
        self.bush_configs = bush_configs or self._load_default_bushes()
        self.tree_count = 0
        self.bush_count = 0

    def _load_default_trees(self) -> Dict[str, TreeConfig]:
        """Load default tree configs with full paths."""
        configs = {}
        for name, cfg in self.DEFAULT_TREES.items():
            configs[name] = TreeConfig(
                usd_path=f"{self.models_path}/{cfg.usd_path}",
                base_scale=cfg.base_scale,
                scale_variation=cfg.scale_variation,
                height_multiplier=cfg.height_multiplier,
                age_range=cfg.age_range,
            )
        return configs

    def _load_default_bushes(self) -> Dict[str, BushConfig]:
        """Load default bush configs with full paths."""
        configs = {}
        for name, cfg in self.DEFAULT_BUSHES.items():
            configs[name] = BushConfig(
                usd_path=f"{self.models_path}/{cfg.usd_path}",
                base_scale=cfg.base_scale,
                scale_variation=cfg.scale_variation,
                cluster_probability=cfg.cluster_probability,
                cluster_size=cfg.cluster_size,
                cluster_spread=cfg.cluster_spread,
            )
        return configs

    def _ensure_tree_material(self) -> str:
        """Create shared green material for trees."""
        mat_path = "/World/Looks/tree_green_mat"
        if not self.stage.GetPrimAtPath(mat_path):
            material = UsdShade.Material.Define(self.stage, mat_path)
            shader = UsdShade.Shader.Define(self.stage, f"{mat_path}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            # Dark forest green color
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(0.05, 0.15, 0.05)  # Dark forest green
            )
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            material.CreateSurfaceOutput().ConnectToSource(
                shader.ConnectableAPI(), "surface"
            )
        return mat_path

    def _ensure_trunk_material(self) -> str:
        """Create shared brown material for tree trunks."""
        mat_path = "/World/Looks/tree_trunk_mat"
        if not self.stage.GetPrimAtPath(mat_path):
            material = UsdShade.Material.Define(self.stage, mat_path)
            shader = UsdShade.Shader.Define(self.stage, f"{mat_path}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            # Dark brown bark color
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(0.15, 0.08, 0.03)  # Dark brown
            )
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            material.CreateSurfaceOutput().ConnectToSource(
                shader.ConnectableAPI(), "surface"
            )
        return mat_path

    def spawn_tree(
        self,
        tree_type: str,
        position: tuple = None,
        age: float = None,
    ) -> Optional[str]:
        """
        Spawn a simple procedural tree (cone + cylinder trunk).

        Args:
            tree_type: Type of tree (affects size variation)
            position: Optional (x, y) position, random if not provided
            age: Optional age factor (0.5-1.0), affects size

        Returns:
            Prim path of spawned tree, or None if no valid position available
        """
        # Tree approximate radius for overlap prevention (based on expected canopy size)
        tree_radius = 3.0  # ~3m radius for typical tree canopy

        # Get position with overlap prevention
        if position is None:
            result = self.get_random_position(object_radius=tree_radius)
            if result is None:
                return None  # No valid position, skip spawning
            x, y = result
        else:
            # Even with explicit position, find valid nearby spot to avoid overlap
            result = self.find_valid_position(position[0], position[1], tree_radius)
            if result is None:
                # No valid position found - skip spawning this tree
                return None
            x, y = result

        # Register position to prevent future overlaps
        self.register_position(x, y, tree_radius)

        # Calculate size with variation
        if age is None:
            age = random.uniform(0.5, 1.0)

        # Base dimensions for a ~12-18m tree (20% larger than before)
        base_height = random.uniform(9.6, 18.0) * age
        base_radius = base_height * 0.25  # Cone radius proportional to height
        trunk_height = base_height * 0.3
        trunk_radius = base_height * 0.05

        # Randomly choose tree shape: cone (conifer) or sphere (deciduous)
        tree_shape = random.choice(["cone", "cone", "sphere"])  # 2:1 ratio favoring cones

        # Noticeable color variation for each individual tree
        # Base dark green with random shifts toward yellow-green, blue-green, or darker
        hue_shift = random.uniform(-0.08, 0.08)  # Shift green toward yellow or blue
        brightness = random.uniform(0.7, 1.3)  # Overall brightness variation
        saturation = random.uniform(0.8, 1.2)  # Color intensity

        # Base color: dark forest green (0.05, 0.15, 0.05)
        base_r, base_g, base_b = 0.05, 0.15, 0.05

        foliage_color = Gf.Vec3f(
            max(0.02, min(0.15, (base_r + hue_shift * 0.5) * brightness * saturation)),
            max(0.08, min(0.25, (base_g + abs(hue_shift) * 0.3) * brightness)),
            max(0.02, min(0.12, (base_b - hue_shift * 0.3) * brightness * saturation))
        )

        # Get ground height
        z = self.raycast_to_ground(x, y)

        # Create tree parent xform using stage.DefinePrim (doesn't add default xform ops)
        tree_path = f'/World/Vegetation/Trees/Tree_{self.tree_count:04d}'
        tree_prim = self.stage.DefinePrim(tree_path, "Xform")

        tree_xform = UsdGeom.Xformable(tree_prim)
        tree_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, z))

        # Create trunk (cylinder)
        trunk_path = f"{tree_path}/trunk"
        trunk = UsdGeom.Cylinder.Define(self.stage, trunk_path)
        trunk.CreateRadiusAttr(trunk_radius)
        trunk.CreateHeightAttr(trunk_height)
        trunk.CreateAxisAttr("Z")

        trunk_xform = UsdGeom.Xformable(trunk.GetPrim())
        trunk_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0, 0, trunk_height / 2)
        )

        # Apply trunk material
        trunk_mat = UsdShade.Material(self.stage.GetPrimAtPath(self._ensure_trunk_material()))
        UsdShade.MaterialBindingAPI(trunk.GetPrim()).Bind(trunk_mat)

        # Create foliage based on tree shape
        foliage_path = f"{tree_path}/foliage"
        foliage_height = base_height - trunk_height

        if tree_shape == "sphere":
            # Deciduous tree - sphere/ellipsoid canopy
            foliage = UsdGeom.Sphere.Define(self.stage, foliage_path)
            sphere_radius = base_radius * 0.8
            foliage.CreateRadiusAttr(sphere_radius)

            foliage_xform = UsdGeom.Xformable(foliage.GetPrim())
            # Stretch vertically to make an ellipsoid
            foliage_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(0, 0, trunk_height + sphere_radius * 0.8)
            )
            foliage_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(1.0, 1.0, 1.2)  # Slightly taller than wide
            )
        else:
            # Conifer tree - cone canopy
            foliage = UsdGeom.Cone.Define(self.stage, foliage_path)
            foliage.CreateRadiusAttr(base_radius)
            foliage.CreateHeightAttr(foliage_height)
            foliage.CreateAxisAttr("Z")

            foliage_xform = UsdGeom.Xformable(foliage.GetPrim())
            foliage_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(0, 0, trunk_height + foliage_height / 2)
            )

        # Create unique material for this tree with color variation
        tree_mat_path = f"/World/Looks/tree_foliage_{self.tree_count:04d}"
        material = UsdShade.Material.Define(self.stage, tree_mat_path)
        shader = UsdShade.Shader.Define(self.stage, f"{tree_mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(foliage_color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        UsdShade.MaterialBindingAPI(foliage.GetPrim()).Bind(material)

        self.spawned_objects.append(tree_path)
        self.tree_count += 1

        return tree_path

    def spawn_forest(
        self,
        density: float,
        proportions: Dict[str, float] = None,
    ) -> List[str]:
        """
        Spawn a forest with specified density and tree proportions.

        Args:
            density: Trees per 100m² (e.g., 3 = 3 trees per 10x10m area)
            proportions: Dict of tree_type -> percentage (e.g., {"Birch": 30, "Spruce": 40, "Pine": 30})

        Returns:
            List of spawned tree prim paths
        """
        if proportions is None:
            proportions = {"Birch": 30, "Spruce": 40, "Pine": 30}

        # Calculate total trees
        area = self.config.area_size[0] * self.config.area_size[1]
        total_trees = int(density * area / 100.0)

        spawned = []

        for tree_type, percentage in proportions.items():
            count = int(total_trees * percentage / 100)
            for _ in range(count):
                path = self.spawn_tree(tree_type)
                if path is not None:
                    spawned.append(path)

        return spawned

    def spawn_bush(
        self,
        bush_type: str,
        position: tuple = None,
    ) -> Optional[str]:
        """Spawn a single bush. Returns None if no valid position available."""
        if bush_type not in self.bush_configs:
            raise ValueError(f"Unknown bush type: {bush_type}")

        cfg = self.bush_configs[bush_type]

        # Bush approximate radius for overlap prevention (~1m)
        bush_radius = 1.0 * cfg.base_scale

        if position is None:
            result = self.get_random_position(object_radius=bush_radius)
            if result is None:
                return None
            x, y = result
        else:
            result = self.find_valid_position(position[0], position[1], bush_radius)
            if result is None:
                return None
            x, y = result

        # Register position to prevent future overlaps
        self.register_position(x, y, bush_radius)

        scale = random.uniform(*cfg.scale_variation) * cfg.base_scale

        bush_path = f'/World/Vegetation/Bushes/Bush_{self.bush_count:04d}'
        prim_utils.create_prim(bush_path, "Xform")
        add_reference_to_stage(usd_path=cfg.usd_path, prim_path=bush_path)

        yaw = random.randrange(-180, 180)
        qw, qx, qy, qz = self.euler_to_quaternion(0, 0, yaw)

        z = self.raycast_to_ground(x, y)

        prim_utils.set_prim_property(bush_path, "xformOp:orient", Gf.Quatd(qw, qx, qy, qz))
        prim_utils.set_prim_property(bush_path, "xformOp:translate", Gf.Vec3d(x, y, z))
        prim_utils.set_prim_property(bush_path, "xformOp:scale", Gf.Vec3d(scale, scale, scale))

        self.spawned_objects.append(bush_path)
        self.bush_count += 1

        return bush_path

    def spawn_undergrowth(
        self,
        density: float,
        bush_types: List[str] = None,
    ) -> List[str]:
        """
        Spawn undergrowth (bushes, blueberries) with clustering.

        Args:
            density: Bushes per 100m²
            bush_types: List of bush types to spawn

        Returns:
            List of spawned bush prim paths
        """
        if bush_types is None:
            bush_types = list(self.bush_configs.keys())

        area = self.config.area_size[0] * self.config.area_size[1]
        total_bushes = int(density * area / 100.0)

        spawned = []

        for _ in range(total_bushes):
            bush_type = random.choice(bush_types)
            cfg = self.bush_configs[bush_type]

            # Check if this bush should spawn as a cluster
            if random.random() < cfg.cluster_probability:
                # Spawn cluster
                center_x, center_y = self.get_random_position()
                for _ in range(cfg.cluster_size):
                    offset_x = random.gauss(0, cfg.cluster_spread)
                    offset_y = random.gauss(0, cfg.cluster_spread)
                    path = self.spawn_bush(bush_type, (center_x + offset_x, center_y + offset_y))
                    spawned.append(path)
            else:
                path = self.spawn_bush(bush_type)
                spawned.append(path)

        return spawned
