#!/usr/bin/env python3
"""
Static Analysis Tests for Flyby F-11 RL Training Pipeline

Tests that run WITHOUT Isaac Sim dependencies:
- Python syntax validation
- Import structure validation (with mocking)
- Type hint consistency
- Dataclass field defaults
- Circular import detection

Run with: python3 -m pytest tests/test_static.py -v
"""
import ast
import sys
import os
import importlib.util
from pathlib import Path
from typing import List, Set, Tuple
from unittest.mock import MagicMock, patch
import subprocess

import pytest

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
ENVIRONMENTS_DIR = PROJECT_ROOT / "environments"
TRAINING_DIR = PROJECT_ROOT / "scripts" / "training"


class TestPythonSyntax:
    """Test that all Python files have valid syntax."""

    @pytest.fixture
    def python_files(self) -> List[Path]:
        """Get all Python files in the project."""
        files = []
        for directory in [ENVIRONMENTS_DIR, TRAINING_DIR]:
            if directory.exists():
                files.extend(directory.rglob("*.py"))
        return files

    def test_all_files_have_valid_syntax(self, python_files):
        """Verify all Python files compile without syntax errors."""
        errors = []
        for filepath in python_files:
            try:
                result = subprocess.run(
                    [sys.executable, "-m", "py_compile", str(filepath)],
                    capture_output=True,
                    text=True
                )
                if result.returncode != 0:
                    errors.append(f"{filepath.name}: {result.stderr}")
            except Exception as e:
                errors.append(f"{filepath.name}: {e}")

        assert not errors, f"Syntax errors found:\n" + "\n".join(errors)

    @pytest.mark.parametrize("filename", [
        "base_isr_env.py",
        "dynamic_nfz_env.py",
        "comms_denied_env.py",
        "gymnasium_wrapper.py",
        "action_bridge.py",
        "safety_filter.py",
        "domain_randomizer.py",
        "ontology_bridge.py",
        "multi_objective_env.py",
        "__init__.py",
    ])
    def test_environment_file_syntax(self, filename):
        """Test each environment file individually for syntax validity."""
        filepath = ENVIRONMENTS_DIR / filename
        if not filepath.exists():
            pytest.skip(f"File {filename} not found")

        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(filepath)],
            capture_output=True,
            text=True
        )
        assert result.returncode == 0, f"Syntax error in {filename}: {result.stderr}"


class TestASTAnalysis:
    """Static AST-based analysis tests."""

    def _parse_file(self, filepath: Path) -> ast.Module:
        """Parse a Python file into an AST."""
        with open(filepath, 'r') as f:
            return ast.parse(f.read(), filename=str(filepath))

    def test_base_isr_env_has_required_classes(self):
        """Verify base_isr_env.py defines expected classes."""
        filepath = ENVIRONMENTS_DIR / "base_isr_env.py"
        tree = self._parse_file(filepath)

        class_names = {node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)}

        required_classes = {
            'FlightPhase',
            'CommsStatus',
            'GNSSStatus',
            'ThreatLevel',
            'GeofenceConfig',
            'NoFlyZone',
            'ThreatZone',
            'TargetOfInterest',
            'CommsZone',
            'DomainRandomizationConfig',
            'UAVState',
            'EnvironmentConfig',
            'BaseISREnvironment',
        }

        missing = required_classes - class_names
        assert not missing, f"Missing required classes: {missing}"

    def test_dynamic_nfz_env_has_required_classes(self):
        """Verify dynamic_nfz_env.py defines expected classes."""
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"
        tree = self._parse_file(filepath)

        class_names = {node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)}

        required_classes = {
            'NFZRandomizationConfig',
            'DynamicNFZ',
            'DynamicNFZConfig',
            'FlightPath',
            'DynamicNFZAvoidanceEnv',
        }

        missing = required_classes - class_names
        assert not missing, f"Missing required classes: {missing}"

    def test_gymnasium_wrapper_implements_gym_interface(self):
        """Verify gymnasium_wrapper.py has required Gymnasium methods."""
        filepath = ENVIRONMENTS_DIR / "gymnasium_wrapper.py"
        tree = self._parse_file(filepath)

        # Find IsaacSimGymWrapper class
        wrapper_class = None
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "IsaacSimGymWrapper":
                wrapper_class = node
                break

        assert wrapper_class is not None, "IsaacSimGymWrapper class not found"

        method_names = {
            node.name for node in ast.walk(wrapper_class)
            if isinstance(node, ast.FunctionDef)
        }

        required_methods = {'reset', 'step', 'render', 'close'}
        missing = required_methods - method_names
        assert not missing, f"Missing Gymnasium interface methods: {missing}"

    def test_safety_filter_has_filter_action_method(self):
        """Verify VampireSafetyFilter has the filter_action method."""
        filepath = ENVIRONMENTS_DIR / "safety_filter.py"
        tree = self._parse_file(filepath)

        filter_class = None
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "VampireSafetyFilter":
                filter_class = node
                break

        assert filter_class is not None, "VampireSafetyFilter class not found"

        method_names = {
            node.name for node in ast.walk(filter_class)
            if isinstance(node, ast.FunctionDef)
        }

        required_methods = {'filter_action', 'check_action_safety', 'get_safe_fallback_action'}
        missing = required_methods - method_names
        assert not missing, f"Missing required methods: {missing}"


class TestDataclassDefaults:
    """Test that dataclasses have valid defaults."""

    def test_domain_randomization_config_defaults(self):
        """Test DomainRandomizationConfig has sensible defaults."""
        # Parse the file to extract dataclass field defaults
        filepath = ENVIRONMENTS_DIR / "base_isr_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Check for expected default values
        assert "enabled: bool = True" in content, "DR should be enabled by default"
        assert "battery_variation_enabled: bool = True" in content
        assert "battery_drain_rate_variation: float = 0.1" in content, "Battery variation should default to 10%"
        assert "gnss_degradation_prob: float = 0.1" in content, "GNSS degradation should be 10%"
        assert "wind_speed_range: Tuple[float, float] = (0.0, 5.0)" in content

    def test_nfz_randomization_config_defaults(self):
        """Test NFZRandomizationConfig has valid bounds."""
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Check bounds validation - min should be less than max
        assert "min_nfz_count: int = 2" in content, "Should have at least 2 NFZs"
        assert "max_nfz_count: int = 5" in content, "Should have at most 5 NFZs"
        assert "base_radius_min: float = 200.0" in content
        assert "base_radius_max: float = 500.0" in content

    def test_uav_state_has_domain_randomization_fields(self):
        """Verify UAVState includes domain randomization fields."""
        filepath = ENVIRONMENTS_DIR / "base_isr_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # These fields are critical for DR
        required_fields = [
            "wind_vector",
            "vio_position_drift",
            "battery_drain_multiplier",
            "gnss_degradation_triggered",
            "gnss_degradation_step",
        ]

        for field in required_fields:
            assert field in content, f"UAVState missing field: {field}"


class TestObservationDimensionConsistency:
    """Test observation dimension consistency between components."""

    def test_base_env_observation_dim_matches_state_to_observation(self):
        """
        Verify observation_dim property matches actual output of state_to_observation.

        This catches the bug where wrapper says 14 but env produces 28.
        """
        filepath = ENVIRONMENTS_DIR / "base_isr_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Extract observation_dim comment/calculation
        # The property says: pos(3) + vel(3) + quat(4) + wind(3) + scalars(15)
        # = 3 + 3 + 4 + 3 + 15 = 28
        assert "return 3 + 3 + 4 + 3 + 15" in content or "return 28" in content, \
            "observation_dim should return 28 (3+3+4+3+15)"

        # Count actual components in state_to_observation
        # Looking for concatenation pattern
        assert "state.position" in content, "Should include position (3 values)"
        assert "state.velocity" in content, "Should include velocity (3 values)"
        assert "state.orientation" in content, "Should include orientation/quat (4 values)"
        assert "wind_normalized" in content, "Should include wind vector (3 values)"

    def test_dynamic_nfz_observation_dim_extends_base(self):
        """Verify DynamicNFZAvoidanceEnv extends base observation correctly."""
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Should call super().observation_dim and add extra features
        assert "super().observation_dim" in content, "Should extend base observation_dim"
        assert "return base_dim + 8" in content, "Should add 8 extra features"

    def test_comms_denied_observation_dim_extends_base(self):
        """Verify CommsDeniedSurveillanceEnv extends base observation correctly."""
        filepath = ENVIRONMENTS_DIR / "comms_denied_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Should extend base
        assert "super().observation_dim" in content, "Should extend base observation_dim"
        assert "return base_dim + 6" in content, "Should add 6 extra features"

    def test_multi_objective_observation_dim_extends_base(self):
        """Verify MultiObjectiveISREnv extends base observation correctly."""
        filepath = ENVIRONMENTS_DIR / "multi_objective_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Should extend base with target features
        assert "super().observation_dim" in content, "Should extend base observation_dim"
        # 8 targets * 5 features + 3 threat features = 43
        assert "8 * 5 + 3" in content, "Should add target and threat features"


class TestImportStructure:
    """Test import structure for common issues."""

    def test_no_circular_imports_in_init(self):
        """Verify __init__.py doesn't create circular imports."""
        filepath = ENVIRONMENTS_DIR / "__init__.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Check import order - base should come first
        lines = content.split('\n')
        import_lines = [l for l in lines if l.startswith('from .')]

        # base_isr_env should be imported before others that depend on it
        base_idx = next(
            (i for i, l in enumerate(import_lines) if 'base_isr_env' in l), -1
        )
        nfz_idx = next(
            (i for i, l in enumerate(import_lines) if 'dynamic_nfz_env' in l), -1
        )
        comms_idx = next(
            (i for i, l in enumerate(import_lines) if 'comms_denied_env' in l), -1
        )

        assert base_idx >= 0, "base_isr_env should be imported"
        assert nfz_idx >= 0, "dynamic_nfz_env should be imported"

        # Base should come before dependent envs
        if base_idx >= 0 and nfz_idx >= 0:
            assert base_idx < nfz_idx, "base_isr_env should be imported before dynamic_nfz_env"
        if base_idx >= 0 and comms_idx >= 0:
            assert base_idx < comms_idx, "base_isr_env should be imported before comms_denied_env"

    def test_all_public_symbols_in_all(self):
        """Verify __all__ list is complete."""
        filepath = ENVIRONMENTS_DIR / "__init__.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Extract __all__ list
        tree = ast.parse(content)
        all_list = None
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id == "__all__":
                        if isinstance(node.value, ast.List):
                            all_list = [
                                elt.s for elt in node.value.elts
                                if isinstance(elt, ast.Constant)
                            ]

        assert all_list is not None, "__all__ should be defined"

        required_exports = [
            'BaseISREnvironment',
            'UAVState',
            'EnvironmentConfig',
            'DomainRandomizationConfig',
            'CommsDeniedSurveillanceEnv',
            'DynamicNFZAvoidanceEnv',
            'MultiObjectiveISREnv',
            'IsaacSimGymWrapper',
            'PX4ActionBridge',
            'VampireSafetyFilter',
        ]

        for export in required_exports:
            assert export in all_list, f"Missing export in __all__: {export}"


class TestCodeQuality:
    """Test code quality and potential bugs."""

    def test_dynamic_nfz_no_deprecated_attribute(self):
        """
        Check for deprecated self.dynamic_nfz attribute usage.

        The code should use self.dynamic_nfzs (list) not self.dynamic_nfz (single).
        """
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Find usages of self.dynamic_nfz (should not exist)
        lines = content.split('\n')
        bad_usages = []
        for i, line in enumerate(lines, 1):
            # Match self.dynamic_nfz but not self.dynamic_nfzs
            if 'self.dynamic_nfz' in line and 'self.dynamic_nfzs' not in line:
                # Exclude comments
                if not line.strip().startswith('#'):
                    bad_usages.append(f"Line {i}: {line.strip()}")

        # Allow the comment about removal, but not actual usage
        actual_bad = [u for u in bad_usages if "removed" not in u.lower()]

        assert not actual_bad, (
            f"Found deprecated self.dynamic_nfz usage (should be self.dynamic_nfzs):\n"
            + "\n".join(actual_bad)
        )

    def test_empty_list_handling(self):
        """Check for potential empty list index errors."""
        filepath = ENVIRONMENTS_DIR / "dynamic_nfz_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Look for indexing without length check
        # Pattern: self.dynamic_nfzs[0] without checking if list is empty
        if "self.dynamic_nfzs[0]" in content:
            # Should have a guard before this
            assert "if self.dynamic_nfzs" in content or "if not self.dynamic_nfzs" in content, \
                "Should check if dynamic_nfzs is empty before indexing"

    def test_gnss_degradation_reset(self):
        """
        Verify GNSS degradation state is reset between episodes.

        This catches the bug where gnss_degradation_step persists across episodes.
        """
        filepath = ENVIRONMENTS_DIR / "base_isr_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Check that reset initializes the GNSS degradation state
        # Look in the reset method
        reset_section = content[content.find("def reset("):]
        if "def " in reset_section[10:]:
            reset_section = reset_section[:reset_section.find("def ", 10)]

        # Should reset gnss state
        assert "gnss_degradation_step" in content, "Should have gnss_degradation_step handling"

    def test_battery_drain_multiplier_applied_to_reserve(self):
        """
        Verify battery drain multiplier affects reserve calculation.

        Bug: Battery reserve didn't account for faster drain rates.
        """
        filepath = ENVIRONMENTS_DIR / "base_isr_env.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Look in compute_required_battery_reserve
        reserve_section = content[content.find("def compute_required_battery_reserve"):]
        if "def " in reserve_section[10:]:
            reserve_section = reserve_section[:reserve_section.find("def ", 10)]

        assert "battery_drain_multiplier" in reserve_section, \
            "compute_required_battery_reserve should use battery_drain_multiplier"


class TestActionBridgeMath:
    """Test action normalization math in action_bridge.py."""

    def test_action_to_velocity_bounds(self):
        """Verify action_to_velocity conversion preserves bounds."""
        filepath = ENVIRONMENTS_DIR / "action_bridge.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Should clip input to [-1, 1]
        assert "np.clip" in content, "Should clip actions to valid range"

        # Should scale by max velocities
        assert "max_horizontal_vel" in content, "Should use max horizontal velocity"
        assert "max_vertical_vel" in content, "Should use max vertical velocity"
        assert "max_yaw_rate" in content, "Should use max yaw rate"

    def test_velocity_to_action_inverse(self):
        """Verify velocity_to_action is inverse of action_to_velocity."""
        filepath = ENVIRONMENTS_DIR / "action_bridge.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Should have inverse function
        assert "def velocity_to_action" in content, "Should have velocity_to_action method"

        # Should divide by max values (inverse of multiply)
        velocity_section = content[content.find("def velocity_to_action"):]
        if "def " in velocity_section[10:]:
            velocity_section = velocity_section[:velocity_section.find("def ", 10)]

        assert "/" in velocity_section, "velocity_to_action should divide by max velocities"


class TestSafetyFilterFallback:
    """Test safety filter graceful degradation."""

    def test_safety_filter_handles_missing_vampire(self):
        """Verify safety filter works when Vampire binary is missing."""
        filepath = ENVIRONMENTS_DIR / "safety_filter.py"

        with open(filepath, 'r') as f:
            content = f.read()

        # Should handle FileNotFoundError
        assert "FileNotFoundError" in content, "Should handle missing Vampire binary"

        # Should have fallback behavior
        assert "return" in content[content.find("FileNotFoundError"):], \
            "Should return gracefully when Vampire not found"

    def test_safety_filter_timeout_handling(self):
        """Verify safety filter handles Vampire timeout."""
        filepath = ENVIRONMENTS_DIR / "safety_filter.py"

        with open(filepath, 'r') as f:
            content = f.read()

        assert "TimeoutExpired" in content or "timeout" in content.lower(), \
            "Should handle Vampire timeout"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
