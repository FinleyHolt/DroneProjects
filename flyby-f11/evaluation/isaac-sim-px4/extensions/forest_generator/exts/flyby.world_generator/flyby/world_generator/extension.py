"""
Flyby World Generator Extension

Omni Kit extension for procedural world generation.
Provides both UI controls and programmatic API for RL training.
"""

import omni.ext
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.ui_utils import (
    dropdown_builder,
    float_builder,
    int_builder,
    str_builder,
    cb_builder,
)
from pxr import UsdPhysics, Sdf, Gf

from isaacsim.core.utils.stage import get_current_stage

from .world_generator import WorldGenerator, WorldConfig


class FlybyWorldGeneratorExtension(omni.ext.IExt):
    """
    Extension providing UI and API for world generation.
    """

    def on_startup(self, ext_id):
        print("[Flyby WorldGenerator] Extension startup")

        self.ext_id = ext_id
        self.world_gen = None

        # Default models path (inside container)
        self.models_path = "/workspace/extensions/forest_generator/models"

        self._build_ui()

    def _build_ui(self):
        """Build the extension UI window."""
        self._window = ui.Window("Flyby World Generator", width=500, height=800)

        with self._window.frame:
            with ui.VStack(spacing=5):
                # Initialize physics scene
                self._init_physics_scene()

                # Header
                ui.Label("ISR Training Environment Generator", height=30)
                ui.Separator(height=10)

                # Terrain section
                with ui.CollapsableFrame("Terrain", height=0):
                    with ui.VStack(spacing=3):
                        self.terrain_size_x = int_builder(
                            label="Width (m)",
                            default_val=200,
                            tooltip="Terrain width in meters"
                        )
                        self.terrain_size_y = int_builder(
                            label="Length (m)",
                            default_val=200,
                            tooltip="Terrain length in meters"
                        )
                        self.terrain_roughness = float_builder(
                            label="Roughness (m)",
                            default_val=3.0,
                            tooltip="Maximum height variation"
                        )
                        self.terrain_material = dropdown_builder(
                            label="Material",
                            items=["forest_floor", "grass"],
                        )

                        with ui.HStack(height=40):
                            ui.Button(
                                "Generate Terrain",
                                clicked_fn=self._on_generate_terrain
                            )
                            ui.Button(
                                "Clear Terrain",
                                clicked_fn=self._on_clear_terrain
                            )

                # Lighting section
                with ui.CollapsableFrame("Lighting / Sky", height=0):
                    with ui.VStack(spacing=3):
                        self.hdri_path = str_builder(
                            label="HDRI Path",
                            tooltip="Path to HDR environment map",
                            use_folder_picker=True,
                            item_filter_fn=lambda item: item.is_folder or item.path.endswith('.hdr'),
                        )
                        self.sun_intensity = float_builder(
                            label="Sun Intensity",
                            default_val=3000.0,
                        )
                        self.randomize_lighting = cb_builder(
                            label="Randomize Lighting",
                        )

                        with ui.HStack(height=40):
                            ui.Button(
                                "Setup Lighting",
                                clicked_fn=self._on_setup_lighting
                            )
                            ui.Button(
                                "Clear Lighting",
                                clicked_fn=self._on_clear_lighting
                            )

                # Vegetation section
                with ui.CollapsableFrame("Vegetation", height=0):
                    with ui.VStack(spacing=3):
                        self.tree_density = int_builder(
                            label="Tree Density",
                            default_val=3,
                            tooltip="Trees per 100m²"
                        )

                        ui.Label("    Tree Proportions:", height=20)
                        self.birch_pct = float_builder(label="  Birch %", default_val=30.0)
                        self.spruce_pct = float_builder(label="  Spruce %", default_val=40.0)
                        self.pine_pct = float_builder(label="  Pine %", default_val=30.0)

                        self.undergrowth_density = int_builder(
                            label="Undergrowth Density",
                            default_val=5,
                            tooltip="Bushes per 100m²"
                        )
                        self.include_undergrowth = cb_builder(
                            label="Include Undergrowth",
                        )

                        with ui.HStack(height=40):
                            ui.Button(
                                "Generate Forest",
                                clicked_fn=self._on_generate_forest
                            )
                            ui.Button(
                                "Clear Forest",
                                clicked_fn=self._on_clear_forest
                            )

                # Vehicles section
                with ui.CollapsableFrame("Vehicles", height=0):
                    with ui.VStack(spacing=3):
                        self.vehicle_count = int_builder(
                            label="Vehicle Count",
                            default_val=5,
                        )
                        self.vehicle_clustering = float_builder(
                            label="Clustering",
                            default_val=0.3,
                            tooltip="0 = random, 1 = tight cluster"
                        )

                        ui.Label("    Note: Register vehicle USD models via API", height=20)

                        with ui.HStack(height=40):
                            ui.Button(
                                "Spawn Vehicles",
                                clicked_fn=self._on_spawn_vehicles
                            )
                            ui.Button(
                                "Clear Vehicles",
                                clicked_fn=self._on_clear_vehicles
                            )

                # People section
                with ui.CollapsableFrame("People", height=0):
                    with ui.VStack(spacing=3):
                        self.people_count = int_builder(
                            label="People Count",
                            default_val=10,
                        )
                        self.group_radius = float_builder(
                            label="Group Radius (m)",
                            default_val=15.0,
                        )

                        ui.Label("    Note: Register person USD models via API", height=20)

                        with ui.HStack(height=40):
                            ui.Button(
                                "Spawn People",
                                clicked_fn=self._on_spawn_people
                            )
                            ui.Button(
                                "Clear People",
                                clicked_fn=self._on_clear_people
                            )

                ui.Separator(height=10)

                # Quick actions
                with ui.HStack(height=50):
                    ui.Button(
                        "Generate Full Environment",
                        clicked_fn=self._on_generate_full,
                        style={"background_color": 0xFF228B22}
                    )

                with ui.HStack(height=50):
                    ui.Button(
                        "Clear All",
                        clicked_fn=self._on_clear_all,
                        style={"background_color": 0xFFB22222}
                    )

                # Status
                ui.Separator(height=10)
                self.status_label = ui.Label("Ready", height=30)

    def _init_physics_scene(self):
        """Initialize physics scene if needed."""
        stage = get_current_stage()
        physics_path = "/World/PhysicsScene"

        if not stage.GetPrimAtPath(physics_path):
            scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_path))
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)

    def _get_world_gen(self) -> WorldGenerator:
        """Get or create WorldGenerator instance."""
        if self.world_gen is None:
            config = WorldConfig(
                terrain_size=(
                    self.terrain_size_x.get_value_as_int(),
                    self.terrain_size_y.get_value_as_int()
                ),
                terrain_roughness=self.terrain_roughness.get_value_as_float(),
                tree_density=self.tree_density.get_value_as_int(),
                tree_proportions={
                    "Birch": self.birch_pct.get_value_as_float(),
                    "Spruce": self.spruce_pct.get_value_as_float(),
                    "Pine": self.pine_pct.get_value_as_float(),
                },
                randomize_lighting=self.randomize_lighting.get_value_as_bool(),
            )
            self.world_gen = WorldGenerator(self.models_path, config)

        return self.world_gen

    def _on_generate_terrain(self):
        """Generate terrain callback."""
        try:
            wg = self._get_world_gen()
            wg.generate_terrain()
            self.status_label.text = f"Terrain generated: {wg.terrain_prim}"
        except Exception as e:
            self.status_label.text = f"Error: {e}"

    def _on_clear_terrain(self):
        """Clear terrain callback."""
        if self.world_gen:
            stage = get_current_stage()
            if self.world_gen.terrain_prim:
                prim = stage.GetPrimAtPath(self.world_gen.terrain_prim)
                if prim.IsValid():
                    stage.RemovePrim(self.world_gen.terrain_prim)
                self.world_gen.terrain_prim = None
            self.status_label.text = "Terrain cleared"

    def _on_setup_lighting(self):
        """Setup lighting callback."""
        try:
            wg = self._get_world_gen()
            hdri = self.hdri_path.get_value_as_string()
            if hdri and hdri.strip():
                wg.setup_lighting(hdri_path=hdri)
            else:
                wg.setup_lighting()
            self.status_label.text = "Lighting setup complete"
        except Exception as e:
            self.status_label.text = f"Error: {e}"

    def _on_clear_lighting(self):
        """Clear lighting callback."""
        stage = get_current_stage()
        if stage.GetPrimAtPath("/World/Lighting"):
            stage.RemovePrim("/World/Lighting")
        if self.world_gen:
            self.world_gen.skybox_light = None
        self.status_label.text = "Lighting cleared"

    def _on_generate_forest(self):
        """Generate forest callback."""
        try:
            wg = self._get_world_gen()
            result = wg.generate_forest(
                include_undergrowth=self.include_undergrowth.get_value_as_bool()
            )
            tree_count = len(result["trees"])
            bush_count = len(result["bushes"])
            self.status_label.text = f"Forest: {tree_count} trees, {bush_count} bushes"
        except Exception as e:
            self.status_label.text = f"Error: {e}"

    def _on_clear_forest(self):
        """Clear forest callback."""
        if self.world_gen:
            self.world_gen.vegetation.clear_all()
            self.status_label.text = "Forest cleared"

    def _on_spawn_vehicles(self):
        """Spawn vehicles callback."""
        try:
            wg = self._get_world_gen()
            if not wg.vehicles.vehicle_configs:
                self.status_label.text = "No vehicles registered - use API to register"
                return

            count = self.vehicle_count.get_value_as_int()
            clustering = self.vehicle_clustering.get_value_as_float()
            types = list(wg.vehicles.vehicle_configs.keys())
            result = wg.vehicles.spawn_vehicle_group(types, count, clustering)
            self.status_label.text = f"Spawned {len(result)} vehicles"
        except Exception as e:
            self.status_label.text = f"Error: {e}"

    def _on_clear_vehicles(self):
        """Clear vehicles callback."""
        if self.world_gen:
            self.world_gen.vehicles.clear_all()
            self.status_label.text = "Vehicles cleared"

    def _on_spawn_people(self):
        """Spawn people callback."""
        try:
            wg = self._get_world_gen()
            if not wg.people.person_configs:
                self.status_label.text = "No people registered - use API to register"
                return

            count = self.people_count.get_value_as_int()
            radius = self.group_radius.get_value_as_float()
            types = list(wg.people.person_configs.keys())
            selected = [types[i % len(types)] for i in range(count)]
            result = wg.people.spawn_group(selected, radius=radius)
            self.status_label.text = f"Spawned {len(result)} people"
        except Exception as e:
            self.status_label.text = f"Error: {e}"

    def _on_clear_people(self):
        """Clear people callback."""
        if self.world_gen:
            self.world_gen.people.clear_all()
            self.status_label.text = "People cleared"

    def _on_generate_full(self):
        """Generate full environment callback."""
        try:
            wg = self._get_world_gen()
            result = wg.generate_full_environment()
            stats = wg.get_stats()
            self.status_label.text = (
                f"Environment: {stats['trees']} trees, "
                f"{stats['bushes']} bushes"
            )
        except Exception as e:
            self.status_label.text = f"Error: {e}"

    def _on_clear_all(self):
        """Clear all callback."""
        if self.world_gen:
            self.world_gen.clear_all()
            self.status_label.text = "All cleared"

    def on_shutdown(self):
        print("[Flyby WorldGenerator] Extension shutdown")
        self._window = None
        self.world_gen = None
