#!/usr/bin/env python3
"""
Flyby F-11 Gazebo Model Validator

This script validates the structure and content of Gazebo SDF models for the
Flyby F-11 quadcopter variants. It checks for:
- Required files (model.sdf, model.config)
- Valid XML structure
- Required model components (links, joints, sensors)
- Reasonable mass and inertia values
- ArduPilot plugin configuration
- Sensor plugin configurations

Usage:
    python validate_models.py [--model MODEL_NAME] [--all]

Examples:
    python validate_models.py --all
    python validate_models.py --model f11_base
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class ValidationResult:
    """Stores validation results for a single model."""
    model_name: str
    passed: bool
    errors: list[str]
    warnings: list[str]
    info: list[str]


class ModelValidator:
    """Validates Gazebo SDF models for the F-11 quadcopter."""

    # Expected mass ranges for different model variants (kg)
    MASS_RANGES = {
        'f11_base': (4.0, 5.0),
        'f11_isr_camera': (5.8, 6.8),
        'f11_multispectral': (4.5, 5.5),
        'f11_lidar': (4.8, 5.8),
        'f11_comms_relay': (4.6, 5.5),
    }

    # Minimum expected components
    REQUIRED_LINKS = ['base_link', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3', 'imu_link']
    REQUIRED_JOINTS = ['rotor_0_joint', 'rotor_1_joint', 'rotor_2_joint', 'rotor_3_joint', 'imu_joint']

    def __init__(self, models_dir: Path):
        self.models_dir = models_dir

    def validate_model(self, model_name: str) -> ValidationResult:
        """Validate a single model."""
        result = ValidationResult(
            model_name=model_name,
            passed=True,
            errors=[],
            warnings=[],
            info=[]
        )

        model_path = self.models_dir / model_name

        # Check model directory exists
        if not model_path.is_dir():
            result.errors.append(f"Model directory not found: {model_path}")
            result.passed = False
            return result

        # Validate model.config
        self._validate_model_config(model_path, result)

        # Validate model.sdf
        self._validate_model_sdf(model_path, model_name, result)

        # Set overall pass/fail
        result.passed = len(result.errors) == 0

        return result

    def _validate_model_config(self, model_path: Path, result: ValidationResult) -> None:
        """Validate model.config file."""
        config_path = model_path / 'model.config'

        if not config_path.exists():
            result.errors.append("Missing model.config file")
            return

        try:
            tree = ET.parse(config_path)
            root = tree.getroot()

            # Check required elements
            name = root.find('name')
            if name is None or not name.text:
                result.errors.append("model.config: Missing <name> element")
            else:
                result.info.append(f"Model name: {name.text}")

            sdf = root.find('sdf')
            if sdf is None or not sdf.text:
                result.errors.append("model.config: Missing <sdf> element")
            elif sdf.text != 'model.sdf':
                result.warnings.append(f"model.config: SDF file is '{sdf.text}', expected 'model.sdf'")

            author = root.find('author')
            if author is None:
                result.warnings.append("model.config: Missing <author> element")

            description = root.find('description')
            if description is None or not description.text:
                result.warnings.append("model.config: Missing or empty <description>")

        except ET.ParseError as e:
            result.errors.append(f"model.config: XML parse error - {e}")

    def _validate_model_sdf(self, model_path: Path, model_name: str, result: ValidationResult) -> None:
        """Validate model.sdf file."""
        sdf_path = model_path / 'model.sdf'

        if not sdf_path.exists():
            result.errors.append("Missing model.sdf file")
            return

        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()

            # Check SDF version
            sdf_version = root.get('version')
            if sdf_version:
                result.info.append(f"SDF version: {sdf_version}")
                if sdf_version < '1.9':
                    result.warnings.append(f"SDF version {sdf_version} is older than recommended (1.9)")

            # Find model element
            model = root.find('model')
            if model is None:
                result.errors.append("model.sdf: Missing <model> element")
                return

            # Validate links
            self._validate_links(model, model_name, result)

            # Validate joints
            self._validate_joints(model, result)

            # Validate ArduPilot plugin
            self._validate_ardupilot_plugin(model, result)

            # Validate sensors
            self._validate_sensors(model, model_name, result)

        except ET.ParseError as e:
            result.errors.append(f"model.sdf: XML parse error - {e}")

    def _validate_links(self, model: ET.Element, model_name: str, result: ValidationResult) -> None:
        """Validate link elements."""
        links = model.findall('link')
        link_names = [link.get('name') for link in links]

        result.info.append(f"Found {len(links)} links")

        # Check required links
        for required_link in self.REQUIRED_LINKS:
            if required_link not in link_names:
                result.errors.append(f"Missing required link: {required_link}")

        # Validate base_link mass and inertia
        base_link = model.find("link[@name='base_link']")
        if base_link is not None:
            self._validate_inertial(base_link, model_name, result)

    def _validate_inertial(self, link: ET.Element, model_name: str, result: ValidationResult) -> None:
        """Validate inertial properties of a link."""
        inertial = link.find('inertial')
        if inertial is None:
            result.errors.append(f"Link '{link.get('name')}': Missing <inertial> element")
            return

        mass_elem = inertial.find('mass')
        if mass_elem is None or mass_elem.text is None:
            result.errors.append(f"Link '{link.get('name')}': Missing <mass> element")
            return

        try:
            mass = float(mass_elem.text)
            result.info.append(f"Total mass: {mass:.2f} kg")

            # Check mass range for known models
            if model_name in self.MASS_RANGES:
                min_mass, max_mass = self.MASS_RANGES[model_name]
                if mass < min_mass or mass > max_mass:
                    result.warnings.append(
                        f"Mass {mass:.2f} kg outside expected range [{min_mass}, {max_mass}] kg"
                    )
        except ValueError:
            result.errors.append(f"Link '{link.get('name')}': Invalid mass value '{mass_elem.text}'")
            return

        # Validate inertia tensor
        inertia = inertial.find('inertia')
        if inertia is None:
            result.errors.append(f"Link '{link.get('name')}': Missing <inertia> element")
            return

        # Check for required inertia components
        for component in ['ixx', 'iyy', 'izz']:
            elem = inertia.find(component)
            if elem is None or elem.text is None:
                result.errors.append(f"Link '{link.get('name')}': Missing <{component}> in inertia")
            else:
                try:
                    value = float(elem.text)
                    if value <= 0:
                        result.warnings.append(
                            f"Link '{link.get('name')}': {component} = {value} should be positive"
                        )
                except ValueError:
                    result.errors.append(
                        f"Link '{link.get('name')}': Invalid {component} value '{elem.text}'"
                    )

    def _validate_joints(self, model: ET.Element, result: ValidationResult) -> None:
        """Validate joint elements."""
        joints = model.findall('joint')
        joint_names = [joint.get('name') for joint in joints]

        result.info.append(f"Found {len(joints)} joints")

        # Check required joints
        for required_joint in self.REQUIRED_JOINTS:
            if required_joint not in joint_names:
                result.errors.append(f"Missing required joint: {required_joint}")

        # Validate rotor joints
        for joint in joints:
            joint_name = joint.get('name')
            if joint_name and 'rotor' in joint_name and 'joint' in joint_name:
                joint_type = joint.get('type')
                if joint_type != 'revolute':
                    result.warnings.append(
                        f"Joint '{joint_name}': type is '{joint_type}', expected 'revolute'"
                    )

    def _validate_ardupilot_plugin(self, model: ET.Element, result: ValidationResult) -> None:
        """Validate ArduPilot plugin configuration."""
        plugins = model.findall('plugin')
        ardupilot_plugin = None

        for plugin in plugins:
            if plugin.get('name') == 'ArduPilotPlugin':
                ardupilot_plugin = plugin
                break

        if ardupilot_plugin is None:
            result.errors.append("Missing ArduPilotPlugin")
            return

        result.info.append("ArduPilotPlugin found")

        # Check required plugin elements
        required_elements = ['fdm_addr', 'fdm_port_in', 'fdm_port_out', 'imuName']
        for elem_name in required_elements:
            elem = ardupilot_plugin.find(elem_name)
            if elem is None or not elem.text:
                result.errors.append(f"ArduPilotPlugin: Missing <{elem_name}>")

        # Check control channels
        controls = ardupilot_plugin.findall('control')
        if len(controls) < 4:
            result.errors.append(f"ArduPilotPlugin: Expected 4 control channels, found {len(controls)}")
        else:
            channels = set()
            for control in controls:
                channel = control.get('channel')
                if channel:
                    channels.add(int(channel))
            if channels != {0, 1, 2, 3}:
                result.warnings.append(
                    f"ArduPilotPlugin: Control channels are {sorted(channels)}, expected [0, 1, 2, 3]"
                )

    def _validate_sensors(self, model: ET.Element, model_name: str, result: ValidationResult) -> None:
        """Validate sensor configurations."""
        sensors = []
        for link in model.findall('link'):
            for sensor in link.findall('sensor'):
                sensors.append((link.get('name'), sensor))

        result.info.append(f"Found {len(sensors)} sensors")

        # Check for required IMU sensor
        imu_found = False
        for link_name, sensor in sensors:
            if sensor.get('type') == 'imu':
                imu_found = True
                break

        if not imu_found:
            result.errors.append("Missing IMU sensor (required for ArduPilot)")

        # Model-specific sensor checks
        if model_name == 'f11_isr_camera':
            camera_found = any(s.get('type') == 'camera' for _, s in sensors)
            if not camera_found:
                result.errors.append("f11_isr_camera: Missing camera sensor")

        elif model_name == 'f11_multispectral':
            camera_found = any(s.get('type') == 'camera' for _, s in sensors)
            if not camera_found:
                result.errors.append("f11_multispectral: Missing multispectral camera sensor")

        elif model_name == 'f11_lidar':
            lidar_found = any(s.get('type') in ['gpu_lidar', 'lidar'] for _, s in sensors)
            if not lidar_found:
                result.errors.append("f11_lidar: Missing LiDAR sensor")

    def validate_all(self) -> list[ValidationResult]:
        """Validate all F-11 model variants."""
        results = []
        expected_models = ['f11_base', 'f11_isr_camera', 'f11_multispectral', 'f11_lidar', 'f11_comms_relay']

        for model_name in expected_models:
            result = self.validate_model(model_name)
            results.append(result)

        return results


def print_result(result: ValidationResult) -> None:
    """Print validation result in a formatted way."""
    status = "PASS" if result.passed else "FAIL"
    status_color = "\033[92m" if result.passed else "\033[91m"
    reset_color = "\033[0m"

    print(f"\n{'=' * 60}")
    print(f"Model: {result.model_name}")
    print(f"Status: {status_color}{status}{reset_color}")
    print(f"{'=' * 60}")

    if result.info:
        print("\nInfo:")
        for info in result.info:
            print(f"  [INFO] {info}")

    if result.warnings:
        print("\nWarnings:")
        for warning in result.warnings:
            print(f"  \033[93m[WARN]\033[0m {warning}")

    if result.errors:
        print("\nErrors:")
        for error in result.errors:
            print(f"  \033[91m[ERROR]\033[0m {error}")


def main():
    parser = argparse.ArgumentParser(
        description='Validate Flyby F-11 Gazebo models',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--model',
        type=str,
        help='Name of specific model to validate (e.g., f11_base)'
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='Validate all F-11 model variants'
    )
    parser.add_argument(
        '--models-dir',
        type=str,
        default=None,
        help='Path to models directory (default: auto-detect)'
    )

    args = parser.parse_args()

    # Determine models directory
    if args.models_dir:
        models_dir = Path(args.models_dir)
    else:
        # Try to find models directory relative to script location
        script_dir = Path(__file__).parent.resolve()
        models_dir = script_dir.parent / 'models'
        if not models_dir.is_dir():
            # Try relative to current working directory
            models_dir = Path('simulation/models')
            if not models_dir.is_dir():
                print(f"Error: Could not find models directory. Use --models-dir to specify.")
                sys.exit(1)

    print(f"Models directory: {models_dir.resolve()}")

    validator = ModelValidator(models_dir)

    if args.all:
        results = validator.validate_all()
    elif args.model:
        results = [validator.validate_model(args.model)]
    else:
        print("Error: Specify --model MODEL_NAME or --all")
        parser.print_help()
        sys.exit(1)

    # Print results
    for result in results:
        print_result(result)

    # Print summary
    print(f"\n{'=' * 60}")
    print("SUMMARY")
    print(f"{'=' * 60}")
    passed = sum(1 for r in results if r.passed)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if passed == total:
        print("\n\033[92mAll models validated successfully!\033[0m")
        sys.exit(0)
    else:
        failed = [r.model_name for r in results if not r.passed]
        print(f"\n\033[91mFailed models: {', '.join(failed)}\033[0m")
        sys.exit(1)


if __name__ == '__main__':
    main()
