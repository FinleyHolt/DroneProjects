"""
Material Manager - PBR material creation and management.

Single responsibility: Create and apply PBR materials to terrain and objects.
"""

# Standard library
from typing import Dict

# Third-party
from pxr import Gf, Sdf, UsdShade

# Isaac Sim / Omniverse
import carb


# Default texture paths (relative to models/textures/)
DEFAULT_MATERIALS: Dict[str, Dict[str, str]] = {
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


class MaterialManager:
    """
    Manages PBR material creation and application.

    Responsibilities:
    - Create PBR shader materials
    - Apply materials to terrain and objects
    - Manage UV reader setup
    - Handle texture file references
    """

    def __init__(self, stage, textures_path: str):
        """
        Initialize MaterialManager.

        Args:
            stage: USD stage to create materials on
            textures_path: Path to textures directory
        """
        self.stage = stage
        self.textures_path = textures_path
        self._materials: Dict[str, str] = DEFAULT_MATERIALS.copy()

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
        if material_name not in self._materials:
            carb.log_warn(f"Material '{material_name}' not found, using default")
            material_name = "forest_floor"

        mat_config = self._materials[material_name]

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

    def register_material(self, name: str, config: Dict[str, str]) -> None:
        """
        Register a custom material configuration.

        Args:
            name: Material name
            config: Dict with texture file paths (diffuse, normal, roughness, etc.)
        """
        self._materials[name] = config

    def get_available_materials(self) -> list:
        """Return list of available material names."""
        return list(self._materials.keys())
