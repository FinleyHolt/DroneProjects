#!/usr/bin/env python3
"""
Domain Randomization Script for Flyby F-11 Training Worlds

Generates randomized world variants from a base configuration for
domain randomization training. This helps prevent RL overfitting by
creating diverse training environments with varied:
- Obstacle positions and orientations
- No-Fly Zone locations and sizes
- Spawn points
- Environmental conditions (lighting, textures)

Usage:
    python scripts/randomize_world.py --base-config simulation/worlds/world_configs/randomized_config.yaml \
        --output-dir simulation/worlds/generated \
        --num-variants 10 \
        --seed 42

Author: Finley Holt
"""

import argparse
import copy
import json
import random
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import xml.etree.ElementTree as ET

import yaml
import re


# Lighting presets for domain randomization
LIGHTING_PRESETS = {
    'day': {
        'diffuse': '0.8 0.8 0.8 1',
        'specular': '0.25 0.25 0.25 1',
        'direction': '-0.4 0.2 -0.9',
        'cast_shadows': 'true',
        'ambient': '0.4 0.4 0.4 1',
        'background': '0.7 0.75 0.85 1',
    },
    'dawn': {
        'diffuse': '1.0 0.6 0.4 1',
        'specular': '0.3 0.2 0.15 1',
        'direction': '0.8 0.2 -0.3',
        'cast_shadows': 'true',
        'ambient': '0.35 0.3 0.25 1',
        'background': '0.9 0.6 0.5 1',
    },
    'dusk': {
        'diffuse': '0.9 0.5 0.35 1',
        'specular': '0.25 0.15 0.1 1',
        'direction': '-0.8 0.2 -0.3',
        'cast_shadows': 'true',
        'ambient': '0.3 0.25 0.25 1',
        'background': '0.85 0.55 0.45 1',
    },
    'overcast': {
        'diffuse': '0.5 0.5 0.55 1',
        'specular': '0.1 0.1 0.1 1',
        'direction': '0 0 -1',
        'cast_shadows': 'false',
        'ambient': '0.45 0.45 0.48 1',
        'background': '0.6 0.62 0.65 1',
    },
    'morning': {
        'diffuse': '0.85 0.85 0.75 1',
        'specular': '0.3 0.3 0.25 1',
        'direction': '0.6 0.3 -0.7',
        'cast_shadows': 'true',
        'ambient': '0.38 0.38 0.35 1',
        'background': '0.65 0.72 0.85 1',
    },
}


def randomize_position(
    base_position: List[float],
    noise: float,
    bounds: Optional[Dict[str, List[float]]] = None
) -> List[float]:
    """
    Add random noise to a position while respecting bounds.

    Args:
        base_position: Original [x, y, z] or [x, y, z, roll, pitch, yaw]
        noise: Maximum displacement in any direction
        bounds: Optional dict with 'x_range', 'y_range', 'z_range'

    Returns:
        Randomized position
    """
    result = list(base_position)

    # Randomize x, y (and optionally z)
    for i in range(min(3, len(result))):
        result[i] += random.uniform(-noise, noise)

    # Apply bounds if provided
    if bounds:
        if 'x_range' in bounds:
            result[0] = max(bounds['x_range'][0], min(bounds['x_range'][1], result[0]))
        if 'y_range' in bounds:
            result[1] = max(bounds['y_range'][0], min(bounds['y_range'][1], result[1]))
        if 'z_range' in bounds and len(result) > 2:
            result[2] = max(bounds['z_range'][0], min(bounds['z_range'][1], result[2]))

    return result


def randomize_size(base_size: float, noise_fraction: float, min_val: float = 5.0) -> float:
    """
    Add random variation to a size value.

    Args:
        base_size: Original size
        noise_fraction: Fraction of original size to use as noise range
        min_val: Minimum allowed value

    Returns:
        Randomized size
    """
    variation = base_size * noise_fraction
    new_size = base_size + random.uniform(-variation, variation)
    return max(min_val, new_size)


def randomize_nfz(nfz: Dict, config: Dict) -> Dict:
    """
    Randomize a No-Fly Zone definition.

    Args:
        nfz: Original NFZ definition
        config: Randomization config with bounds

    Returns:
        Randomized NFZ
    """
    result = copy.deepcopy(nfz)

    if not nfz.get('randomizable', True):
        return result

    rand_config = nfz.get('randomization_bounds', {})
    position_noise = config.get('nfz_placement', {}).get('position_noise', 20)
    size_noise = config.get('nfz_placement', {}).get('size_noise', 0.3)

    if nfz.get('type') == 'cylinder':
        # Randomize center position
        result['center'] = randomize_position(
            nfz['center'],
            position_noise,
            rand_config
        )
        # Randomize radius
        if 'radius_range' in rand_config:
            result['radius'] = random.uniform(*rand_config['radius_range'])
        else:
            result['radius'] = randomize_size(nfz['radius'], size_noise, 10)

    elif nfz.get('type') == 'box':
        # Randomize box position (shift min/max together)
        offset = [random.uniform(-position_noise, position_noise) for _ in range(3)]

        result['min'] = [nfz['min'][i] + offset[i] for i in range(3)]
        result['max'] = [nfz['max'][i] + offset[i] for i in range(3)]

        # Randomize size
        if 'size_range' in rand_config:
            size_factor = random.uniform(0.7, 1.3)
            center = [(result['min'][i] + result['max'][i]) / 2 for i in range(3)]
            half_size = [(result['max'][i] - result['min'][i]) / 2 * size_factor for i in range(3)]
            result['min'] = [center[i] - half_size[i] for i in range(3)]
            result['max'] = [center[i] + half_size[i] for i in range(3)]

    return result


def randomize_obstacle(obstacle: Dict, config: Dict) -> Dict:
    """
    Randomize an obstacle definition including position and rotation.

    Args:
        obstacle: Original obstacle definition
        config: Randomization config

    Returns:
        Randomized obstacle
    """
    result = copy.deepcopy(obstacle)

    if not obstacle.get('randomizable', True):
        return result

    position_noise = config.get('obstacle_placement', {}).get('position_noise', 20)
    rotation_noise = config.get('obstacle_placement', {}).get('rotation_noise', 0.5)

    if 'pose' in obstacle:
        pose = list(obstacle['pose'])
        # Ensure pose has 6 elements [x, y, z, roll, pitch, yaw]
        while len(pose) < 6:
            pose.append(0.0)
        # Randomize x, y, keep z
        pose[0] += random.uniform(-position_noise, position_noise)
        pose[1] += random.uniform(-position_noise, position_noise)
        # Randomize yaw (index 5) with rotation_noise
        pose[5] += random.uniform(-rotation_noise, rotation_noise)
        result['pose'] = pose

    if 'position' in obstacle:
        pos = list(obstacle['position'])
        pos[0] += random.uniform(-position_noise, position_noise)
        pos[1] += random.uniform(-position_noise, position_noise)
        result['position'] = pos

    return result


def randomize_spawn_points(spawn_points: List[Dict], config: Dict) -> List[Dict]:
    """
    Randomize spawn point positions.

    Args:
        spawn_points: List of spawn point definitions
        config: Randomization config

    Returns:
        Randomized spawn points
    """
    result = []
    position_noise = config.get('spawn_variation', {}).get('position_noise', 5)

    for sp in spawn_points:
        new_sp = copy.deepcopy(sp)
        if sp.get('randomizable', sp.get('name') != 'default'):
            new_sp['pose'] = randomize_position(sp['pose'], position_noise)
        result.append(new_sp)

    return result


def randomize_landing_zones(landing_zones: List[Dict], config: Dict) -> List[Dict]:
    """
    Randomize landing zone positions.

    Args:
        landing_zones: List of landing zone definitions
        config: Randomization config

    Returns:
        Randomized landing zones
    """
    result = []
    position_noise = config.get('spawn_variation', {}).get('position_noise', 5)

    for lz in landing_zones:
        new_lz = copy.deepcopy(lz)
        if lz.get('randomizable', lz.get('type') != 'primary'):
            new_lz['position'] = randomize_position(lz['position'], position_noise)
        result.append(new_lz)

    return result


def select_random_environment(config: Dict) -> Dict:
    """
    Select random environmental conditions.

    Args:
        config: Randomization config with options

    Returns:
        Selected environment settings
    """
    env = {}

    lighting_config = config.get('lighting', {})
    if lighting_config.get('enabled', True):
        options = lighting_config.get('options', ['day', 'dusk', 'overcast'])
        env['lighting'] = random.choice(options)

    ground_config = config.get('ground_texture', {})
    if ground_config.get('enabled', True):
        options = ground_config.get('options', ['grass', 'asphalt', 'dirt'])
        env['ground_texture'] = random.choice(options)

    return env


def randomize_config(base_config: Dict, seed: int) -> Dict:
    """
    Create a randomized variant of a world configuration.

    Args:
        base_config: Original world configuration
        seed: Random seed for reproducibility

    Returns:
        Randomized configuration
    """
    random.seed(seed)

    config = copy.deepcopy(base_config)
    rand_config = base_config.get('randomization', {})

    if not rand_config.get('enabled', True):
        return config

    # Randomize NFZs
    if rand_config.get('nfz_placement', {}).get('enabled', True):
        config['no_fly_zones'] = [
            randomize_nfz(nfz, rand_config)
            for nfz in base_config.get('no_fly_zones', [])
        ]

    # Randomize obstacles
    if rand_config.get('obstacle_placement', {}).get('enabled', True):
        obstacles = config.get('obstacles', {})
        for category in obstacles:
            if isinstance(obstacles[category], list):
                obstacles[category] = [
                    randomize_obstacle(obs, rand_config)
                    for obs in obstacles[category]
                ]
        config['obstacles'] = obstacles

    # Randomize spawn points
    if rand_config.get('spawn_variation', {}).get('enabled', True):
        config['spawn_points'] = randomize_spawn_points(
            base_config.get('spawn_points', []),
            rand_config
        )

    # Randomize landing zones
    if rand_config.get('spawn_variation', {}).get('enabled', True):
        config['landing_zones'] = randomize_landing_zones(
            base_config.get('landing_zones', []),
            rand_config
        )

    # Select random environment
    env_settings = select_random_environment(rand_config)
    if 'environment' in config:
        config['environment'].update(env_settings)
    else:
        config['environment'] = env_settings

    return config


def update_pose_element(pose_elem: ET.Element, new_pose: List[float]) -> None:
    """Update a pose element with new position values."""
    pose_str = ' '.join(str(v) for v in new_pose)
    pose_elem.text = pose_str


def find_model_by_name(world: ET.Element, name: str) -> Optional[ET.Element]:
    """Find a model element by name in the world."""
    for model in world.findall('.//model'):
        if model.get('name') == name:
            return model
    for include in world.findall('.//include'):
        name_elem = include.find('name')
        if name_elem is not None and name_elem.text == name:
            return include
    return None


def update_lighting(world: ET.Element, lighting_preset: str) -> None:
    """Update the lighting in the world based on preset."""
    if lighting_preset not in LIGHTING_PRESETS:
        return

    preset = LIGHTING_PRESETS[lighting_preset]

    # Find the main sun light
    for light in world.findall('.//light'):
        if light.get('name') == 'sun':
            # Update diffuse
            diffuse = light.find('diffuse')
            if diffuse is not None:
                diffuse.text = preset['diffuse']

            # Update specular
            specular = light.find('specular')
            if specular is not None:
                specular.text = preset['specular']

            # Update direction
            direction = light.find('direction')
            if direction is not None:
                direction.text = preset['direction']

            # Update cast_shadows
            cast_shadows = light.find('cast_shadows')
            if cast_shadows is not None:
                cast_shadows.text = preset['cast_shadows']

    # Update scene ambient and background
    scene = world.find('.//scene')
    if scene is not None:
        ambient = scene.find('ambient')
        if ambient is not None:
            ambient.text = preset['ambient']

        background = scene.find('background')
        if background is not None:
            background.text = preset['background']


def generate_sdf_from_config(config: Dict, template_sdf: str, seed: int) -> str:
    """
    Generate a modified SDF file based on randomized config.

    This creates a variant SDF by modifying obstacle and NFZ positions,
    lighting settings, and other randomizable elements in the template SDF file.

    Args:
        config: Randomized world configuration
        template_sdf: Path to template SDF file
        seed: Random seed used for this variant

    Returns:
        Modified SDF content as string
    """
    try:
        tree = ET.parse(template_sdf)
        root = tree.getroot()

        # Find the world element
        world = root.find('.//world')
        if world is None:
            print(f"Warning: No <world> element found in {template_sdf}")
            return ET.tostring(root, encoding='unicode')

        # Update world name to include variant info
        world_name = world.get('name', 'flyby_world')
        world.set('name', f"{world_name}_seed{seed}")

        # Update lighting based on environment settings
        env = config.get('environment', {})
        lighting = env.get('lighting', 'day')
        update_lighting(world, lighting)

        # Update obstacle positions from config
        obstacles = config.get('obstacles', {})
        for category, obs_list in obstacles.items():
            if isinstance(obs_list, list):
                for obs in obs_list:
                    obs_name = obs.get('name', '')
                    if not obs_name:
                        continue

                    # Find model in SDF
                    model = find_model_by_name(world, obs_name)
                    if model is None:
                        continue

                    # Update pose if present in config
                    if 'pose' in obs:
                        pose_elem = model.find('pose')
                        if pose_elem is not None:
                            update_pose_element(pose_elem, obs['pose'])
                        else:
                            # Create pose element
                            pose_elem = ET.SubElement(model, 'pose')
                            update_pose_element(pose_elem, obs['pose'])

                    # Also update corresponding collision model
                    collision_model = find_model_by_name(world, f"{obs_name}_collision")
                    if collision_model is not None and 'pose' in obs:
                        collision_pose = model.find('pose')
                        if collision_pose is not None:
                            # Collision needs to match visual position
                            update_pose_element(collision_pose, obs['pose'])

        # Update NFZ positions from config
        nfz_list = config.get('no_fly_zones', [])
        for nfz in nfz_list:
            nfz_name = f"nfz_{nfz.get('name', '')}"

            model = find_model_by_name(world, nfz_name)
            if model is None:
                continue

            if nfz.get('type') == 'cylinder' and 'center' in nfz:
                center = nfz['center']
                height = nfz.get('height', 40)
                pose = [center[0], center[1], height / 2, 0, 0, 0]
                pose_elem = model.find('pose')
                if pose_elem is not None:
                    update_pose_element(pose_elem, pose)

        # Update spawn point positions
        spawn_points = config.get('spawn_points', [])
        for sp in spawn_points:
            if sp.get('name') == 'default':
                # Find F-11 drone include
                for include in world.findall('.//include'):
                    name_elem = include.find('name')
                    if name_elem is not None and 'f11' in name_elem.text.lower():
                        pose_elem = include.find('pose')
                        if pose_elem is not None and 'pose' in sp:
                            update_pose_element(pose_elem, sp['pose'])
                        break

        # Add generation comment
        comment = ET.Comment(f" Generated variant: seed={seed}, lighting={lighting} ")
        root.insert(0, comment)

        return ET.tostring(root, encoding='unicode', xml_declaration=True)

    except Exception as e:
        print(f"Warning: Could not parse SDF template: {e}")
        import traceback
        traceback.print_exc()
        return ""


def generate_variants_for_world(
    base_config_path: Path,
    output_dir: Path,
    num_variants: int,
    base_seed: int,
    generate_sdf: bool = False,
    template_sdf: Optional[Path] = None,
    verbose: bool = False
) -> List[str]:
    """
    Generate randomized variants for a single world.

    Returns list of generated variant names.
    """
    with open(base_config_path, 'r') as f:
        base_config = yaml.safe_load(f)

    world_name = base_config.get('world', {}).get('name', 'unknown')

    # Create world-specific output directory
    world_output_dir = output_dir / world_name
    world_output_dir.mkdir(parents=True, exist_ok=True)

    # Find template SDF if generating SDF files
    sdf_template = None
    if generate_sdf:
        if template_sdf:
            sdf_template = template_sdf
        else:
            sdf_file = base_config.get('world', {}).get('sdf_file', '')
            sdf_template = base_config_path.parent.parent / sdf_file

        if sdf_template and not sdf_template.exists():
            if verbose:
                print(f"  Warning: Template SDF not found: {sdf_template}")
            sdf_template = None

    variants = []
    for i in range(num_variants):
        variant_seed = base_seed + i

        # Generate randomized config
        variant_config = randomize_config(base_config, variant_seed)

        # Determine environment settings for filename
        env = variant_config.get('environment', {})
        lighting = env.get('lighting', 'day')
        world_type = env.get('type', 'mixed')

        # Generate output filename
        variant_name = f"{world_name}_v{i:03d}_seed{variant_seed}_{lighting}"
        variants.append(variant_name)

        # Save YAML config
        yaml_path = world_output_dir / f"{variant_name}.yaml"
        with open(yaml_path, 'w') as f:
            yaml.dump(variant_config, f, default_flow_style=False, sort_keys=False)

        # Save JSON config
        json_path = world_output_dir / f"{variant_name}.json"
        with open(json_path, 'w') as f:
            json.dump(variant_config, f, indent=2)

        # Generate SDF if requested
        if sdf_template and sdf_template.exists():
            sdf_content = generate_sdf_from_config(variant_config, str(sdf_template), variant_seed)
            if sdf_content:
                sdf_path = world_output_dir / f"{variant_name}.sdf"
                with open(sdf_path, 'w') as f:
                    f.write(sdf_content)

        if verbose:
            print(f"    {variant_name}: NFZ={len(variant_config.get('no_fly_zones', []))}, "
                  f"LZ={len(variant_config.get('landing_zones', []))}, lighting={lighting}")

    return variants


def main():
    parser = argparse.ArgumentParser(
        description='Generate randomized world variants for domain randomization training.'
    )
    parser.add_argument(
        '--base-config',
        type=str,
        help='Path to a single base world configuration YAML file'
    )
    parser.add_argument(
        '--config-dir',
        type=str,
        help='Path to directory containing all world configs (randomizes all worlds)'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        required=True,
        help='Directory to write generated variants'
    )
    parser.add_argument(
        '--num-variants',
        type=int,
        default=10,
        help='Number of variants to generate per world (default: 10)'
    )
    parser.add_argument(
        '--seed',
        type=int,
        default=42,
        help='Base random seed (default: 42)'
    )
    parser.add_argument(
        '--generate-sdf',
        action='store_true',
        help='Also generate modified SDF files (requires template)'
    )
    parser.add_argument(
        '--template-sdf',
        type=str,
        help='Path to template SDF file for --generate-sdf (single world mode)'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Print detailed information about generated variants'
    )

    args = parser.parse_args()

    # Validate arguments
    if not args.base_config and not args.config_dir:
        print("Error: Must specify either --base-config or --config-dir")
        sys.exit(1)

    if args.base_config and args.config_dir:
        print("Error: Cannot specify both --base-config and --config-dir")
        sys.exit(1)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    all_variants = {}

    if args.config_dir:
        # Batch mode: randomize all worlds
        config_dir = Path(args.config_dir)
        if not config_dir.exists():
            print(f"Error: Config directory not found: {config_dir}")
            sys.exit(1)

        config_files = list(config_dir.glob("*_config.yaml"))
        if not config_files:
            print(f"Error: No config files found in {config_dir}")
            sys.exit(1)

        print(f"Batch randomization mode: {len(config_files)} worlds")
        print(f"Generating {args.num_variants} variants per world")
        print(f"Total variants: {len(config_files) * args.num_variants}")
        print(f"Output directory: {output_dir}")
        print()

        for config_path in sorted(config_files):
            world_name = config_path.stem.replace("_config", "")
            print(f"Processing: {world_name}")

            variants = generate_variants_for_world(
                base_config_path=config_path,
                output_dir=output_dir,
                num_variants=args.num_variants,
                base_seed=args.seed,
                generate_sdf=args.generate_sdf,
                verbose=args.verbose
            )
            all_variants[world_name] = variants

            if not args.verbose:
                print(f"  Generated {len(variants)} variants")

    else:
        # Single world mode
        base_config_path = Path(args.base_config)
        if not base_config_path.exists():
            print(f"Error: Base config not found: {base_config_path}")
            sys.exit(1)

        with open(base_config_path, 'r') as f:
            base_config = yaml.safe_load(f)

        world_name = base_config.get('world', {}).get('name', 'unknown')

        print(f"Single world mode: {world_name}")
        print(f"Generating {args.num_variants} variants")
        print(f"Output directory: {output_dir}")

        template_sdf = Path(args.template_sdf) if args.template_sdf else None

        variants = generate_variants_for_world(
            base_config_path=base_config_path,
            output_dir=output_dir,
            num_variants=args.num_variants,
            base_seed=args.seed,
            generate_sdf=args.generate_sdf,
            template_sdf=template_sdf,
            verbose=args.verbose
        )
        all_variants[world_name] = variants

    # Write master manifest
    total_variants = sum(len(v) for v in all_variants.values())
    manifest = {
        'mode': 'batch' if args.config_dir else 'single',
        'source': str(args.config_dir or args.base_config),
        'num_variants_per_world': args.num_variants,
        'base_seed': args.seed,
        'total_variants': total_variants,
        'worlds': {
            world: {
                'count': len(variants),
                'variants': variants
            }
            for world, variants in all_variants.items()
        }
    }

    manifest_path = output_dir / 'manifest.json'
    with open(manifest_path, 'w') as f:
        json.dump(manifest, f, indent=2)

    print()
    print(f"Generation complete!")
    print(f"  Worlds processed: {len(all_variants)}")
    print(f"  Total variants: {total_variants}")
    print(f"  Manifest: {manifest_path}")


if __name__ == '__main__':
    main()
