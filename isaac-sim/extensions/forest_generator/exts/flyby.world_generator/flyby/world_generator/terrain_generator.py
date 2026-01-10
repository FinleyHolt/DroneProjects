"""
Terrain Generator - Procedural terrain mesh generation.

Single responsibility: Generate and manage terrain geometry including
heightfield generation, mesh creation, UV coordinates, and collision.
"""

# Standard library
import random
from typing import Optional, Tuple

# Third-party
import numpy as np
from pxr import Gf, PhysxSchema, Sdf, UsdPhysics, UsdShade

# Isaac Sim / Omniverse
import carb
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.core.prims import XFormPrim


class TerrainGenerator:
    """
    Generates procedural terrain meshes with Perlin noise.

    Responsibilities:
    - Heightfield generation using Perlin noise
    - Mesh creation with proper vertices and triangles
    - UV coordinate generation for texturing
    - Collision geometry setup
    - Physics material binding
    """

    def __init__(self, stage, randomize: bool = True, seed: Optional[int] = None):
        """
        Initialize TerrainGenerator.

        Args:
            stage: USD stage to create terrain on
            randomize: Whether to randomize terrain generation
            seed: Random seed for reproducibility
        """
        self.stage = stage
        self.randomize = randomize
        self.seed = seed
        self.terrain_prim = None

    def generate(
        self,
        size: Tuple[float, float] = (200.0, 200.0),
        roughness: float = 3.0,
    ) -> str:
        """
        Generate procedural terrain mesh with Perlin noise.

        Args:
            size: (width, length) in meters
            roughness: Height variation in meters

        Returns:
            Prim path of terrain
        """
        try:
            import noise
        except ImportError:
            carb.log_warn("'noise' module not available, creating flat terrain")
            return self._create_flat_terrain(size)

        # Terrain mesh parameters
        horizontal_scale = 0.5  # meters per vertex
        num_rows = int(size[0] / horizontal_scale)
        num_cols = int(size[1] / horizontal_scale)

        # Generate Perlin noise heightfield
        seed = random.randint(0, 10000) if self.randomize else 42
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
        XFormPrim(
            prim_path=terrain_path,
            name="ground",
            position=np.array([-size[0]/2, -size[1]/2, 0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        # Add collision
        self._add_collision(terrain_path)

        self.terrain_prim = terrain_path
        return terrain_path

    def _create_flat_terrain(self, size: Tuple[float, float]) -> str:
        """Create simple flat terrain (fallback when noise module unavailable)."""
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

        self._add_collision(terrain_path)

        self.terrain_prim = terrain_path
        return terrain_path

    def _heightfield_to_mesh(
        self,
        heightfield: np.ndarray,
        horizontal_scale: float,
        size: Tuple[float, float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Convert heightfield array to mesh vertices and triangles."""
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

    def _add_collision(self, prim_path: str) -> None:
        """Add collision properties to terrain with proper physics material."""
        prim = self.stage.GetPrimAtPath(prim_path)

        # Apply collision API
        UsdPhysics.CollisionAPI.Apply(prim)

        # Use convex decomposition for better performance than triangle mesh
        collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        collision_api.CreateApproximationAttr().Set("convexDecomposition")

        # Make terrain a static rigid body (kinematic, immovable)
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body_api.CreateKinematicEnabledAttr(True)

        # Set PhysX-specific collision properties
        physx_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx_api.GetContactOffsetAttr().Set(0.02)  # 2cm contact offset
        physx_api.GetRestOffsetAttr().Set(0.01)     # 1cm rest offset

        # Create and apply physics material for terrain
        self._create_physics_material(prim)

    def _create_physics_material(self, prim) -> None:
        """Create and bind a physics material to terrain for realistic friction."""
        mat_path = "/World/PhysicsMaterials/TerrainMaterial"

        # Only create if it doesn't exist
        if not self.stage.GetPrimAtPath(mat_path).IsValid():
            # Ensure parent path exists
            parent_path = "/World/PhysicsMaterials"
            if not self.stage.GetPrimAtPath(parent_path).IsValid():
                prim_utils.create_prim(parent_path, "Xform")

            # Create the physics material
            UsdShade.Material.Define(self.stage, mat_path)
            material_prim = self.stage.GetPrimAtPath(mat_path)

            # Apply physics material API
            physics_mat = UsdPhysics.MaterialAPI.Apply(material_prim)
            physics_mat.CreateStaticFrictionAttr(0.7)   # High friction for ground
            physics_mat.CreateDynamicFrictionAttr(0.5)
            physics_mat.CreateRestitutionAttr(0.1)      # Low bounce

        # Bind material to terrain
        material = UsdShade.Material.Get(self.stage, mat_path)
        UsdShade.MaterialBindingAPI(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants)

    def clear(self) -> None:
        """Remove generated terrain."""
        if self.terrain_prim:
            prim = self.stage.GetPrimAtPath(self.terrain_prim)
            if prim.IsValid():
                self.stage.RemovePrim(self.terrain_prim)
            self.terrain_prim = None
