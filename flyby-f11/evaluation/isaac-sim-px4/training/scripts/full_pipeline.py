#!/usr/bin/env python3
"""
Full Training → Deployment Pipeline for ISR Policies

This script orchestrates the complete workflow:
1. Train SearchPolicy (PPO, 2M steps)
2. Train DwellPolicy (SAC, 1.5M steps)
3. Export to ONNX
4. Validate exports
5. Convert to TensorRT (on Jetson)
6. Benchmark inference latency

Usage:
    # Full pipeline (training on RTX 5090, deployment on Jetson)
    python full_pipeline.py --train --export

    # Export and validate only (skip training)
    python full_pipeline.py --export --search-checkpoint outputs/search_policy/final_model.zip

    # Quick test with reduced steps
    python full_pipeline.py --train --export --quick-test
"""

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Full ISR Policy Training and Deployment Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Pipeline stages
    parser.add_argument("--train", action="store_true",
                        help="Run training for both policies")
    parser.add_argument("--export", action="store_true",
                        help="Export trained models to ONNX")
    parser.add_argument("--benchmark", action="store_true",
                        help="Benchmark ONNX inference latency")

    # Checkpoints (for export without training)
    parser.add_argument("--search-checkpoint", type=str, default=None,
                        help="Path to SearchPolicy checkpoint (skip training)")
    parser.add_argument("--dwell-checkpoint", type=str, default=None,
                        help="Path to DwellPolicy checkpoint (skip training)")

    # Training options
    parser.add_argument("--num-envs", type=int, default=1024,
                        help="Number of parallel environments")
    parser.add_argument("--device", type=str, default="cuda:0",
                        help="Training device")
    parser.add_argument("--quick-test", action="store_true",
                        help="Quick test with reduced steps (10k each)")

    # Output
    parser.add_argument("--output-dir", type=str, default="./outputs/pipeline",
                        help="Output directory")

    return parser.parse_args()


def run_command(cmd: list, description: str) -> bool:
    """Run a command and return success status."""
    print(f"\n{'='*60}")
    print(f"  {description}")
    print(f"{'='*60}")
    print(f"  Command: {' '.join(cmd)}")
    print()

    result = subprocess.run(cmd)
    success = result.returncode == 0

    if success:
        print(f"\n  [SUCCESS] {description}")
    else:
        print(f"\n  [FAILED] {description} (exit code: {result.returncode})")

    return success


def train_search_policy(args, output_dir: Path) -> Path:
    """Train SearchPolicy using PPO."""
    steps = 10_000 if args.quick_test else 2_000_000
    checkpoint_dir = output_dir / "search_policy"

    cmd = [
        sys.executable, "-m", "training.scripts.train_search_policy",
        "--num-envs", str(args.num_envs),
        "--total-steps", str(steps),
        "--device", args.device,
        "--output-dir", str(checkpoint_dir),
    ]

    if run_command(cmd, "Training SearchPolicy (PPO)"):
        # Find the latest checkpoint
        checkpoints = list(checkpoint_dir.rglob("final_model.zip"))
        if checkpoints:
            return checkpoints[0]

    return None


def train_dwell_policy(args, output_dir: Path) -> Path:
    """Train DwellPolicy using SAC."""
    steps = 10_000 if args.quick_test else 1_500_000
    checkpoint_dir = output_dir / "dwell_policy"

    cmd = [
        sys.executable, "-m", "training.scripts.train_dwell_policy",
        "--num-envs", str(args.num_envs),
        "--total-steps", str(steps),
        "--device", args.device,
        "--output-dir", str(checkpoint_dir),
    ]

    if run_command(cmd, "Training DwellPolicy (SAC)"):
        # Find the latest checkpoint
        checkpoints = list(checkpoint_dir.rglob("final_model.zip"))
        if checkpoints:
            return checkpoints[0]

    return None


def export_to_onnx(checkpoint_path: Path, output_path: Path, obs_dim: int, action_dim: int) -> bool:
    """Export a checkpoint to ONNX format."""
    cmd = [
        sys.executable, "-m", "training.deployment.onnx_export",
        "--checkpoint", str(checkpoint_path),
        "--output", str(output_path),
        "--obs_dim", str(obs_dim),
        "--action_dim", str(action_dim),
        "--validate",
    ]

    return run_command(cmd, f"Exporting {checkpoint_path.name} to ONNX")


def benchmark_onnx(onnx_path: Path, obs_dim: int) -> bool:
    """Benchmark ONNX inference latency."""
    cmd = [
        sys.executable, "-m", "training.deployment.onnx_export",
        "--checkpoint", "dummy",  # Not used for benchmark
        "--output", "dummy.onnx",  # Not used
        "--benchmark",
        "--obs_dim", str(obs_dim),
    ]

    # Actually run the tensorrt_policy benchmark
    cmd = [
        sys.executable, "-m", "training.deployment.tensorrt_policy",
        str(onnx_path),
        "--iterations", "1000",
    ]

    return run_command(cmd, f"Benchmarking {onnx_path.name}")


def print_tensorrt_instructions(onnx_paths: list):
    """Print instructions for TensorRT conversion on Jetson."""
    print("\n" + "="*60)
    print("  TensorRT Conversion Instructions (Run on Jetson)")
    print("="*60)
    print("""
    After transferring ONNX files to Jetson Orin NX, run:

    # Convert SearchPolicy to TensorRT FP16
    trtexec --onnx=search_policy.onnx \\
            --saveEngine=search_policy_fp16.engine \\
            --fp16 \\
            --workspace=256

    # Convert DwellPolicy to TensorRT FP16
    trtexec --onnx=dwell_policy.onnx \\
            --saveEngine=dwell_policy_fp16.engine \\
            --fp16 \\
            --workspace=256

    # Benchmark TensorRT inference
    trtexec --loadEngine=search_policy_fp16.engine \\
            --iterations=1000 \\
            --avgRuns=100

    Expected performance on Jetson Orin NX:
    - SearchPolicy: ~1-2ms inference (FP16)
    - DwellPolicy: ~1-2ms inference (FP16)
    - Target: <20ms for 50Hz control loop
    """)

    print("  ONNX files to transfer:")
    for path in onnx_paths:
        print(f"    - {path}")


def print_ros2_deployment_instructions(output_dir: Path):
    """Print instructions for ROS2 deployment."""
    print("\n" + "="*60)
    print("  ROS2 Deployment Instructions")
    print("="*60)
    print(f"""
    After TensorRT conversion, copy .engine files to:
      ros2_ws/src/rl_inference/models/

    Update inference_params.yaml:
      search_policy_engine: "$(find rl_inference)/models/search_policy_fp16.engine"
      dwell_policy_engine: "$(find rl_inference)/models/dwell_policy_fp16.engine"

    Build and launch:
      cd ~/flyby-f11/ros2_ws
      colcon build --packages-select rl_inference
      source install/setup.bash
      ros2 launch rl_inference rl_inference.launch.py

    Full system launch:
      ros2 launch flyby_f11_bringup full_autonomy.launch.py
    """)


def main():
    args = parse_args()

    if not (args.train or args.export or args.benchmark):
        print("Error: Specify at least one stage: --train, --export, or --benchmark")
        return 1

    # Setup output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir) / timestamp
    output_dir.mkdir(parents=True, exist_ok=True)

    print("="*60)
    print("  ISR Policy Training & Deployment Pipeline")
    print("="*60)
    print(f"  Output directory: {output_dir}")
    print(f"  Stages: train={args.train}, export={args.export}, benchmark={args.benchmark}")
    if args.quick_test:
        print(f"  Mode: QUICK TEST (reduced steps)")
    print("="*60)

    # Track checkpoints
    search_checkpoint = Path(args.search_checkpoint) if args.search_checkpoint else None
    dwell_checkpoint = Path(args.dwell_checkpoint) if args.dwell_checkpoint else None

    # Stage 1: Training
    if args.train:
        print("\n" + "#"*60)
        print("  STAGE 1: TRAINING")
        print("#"*60)

        # Train SearchPolicy
        search_checkpoint = train_search_policy(args, output_dir)
        if not search_checkpoint:
            print("ERROR: SearchPolicy training failed")
            return 1

        # Train DwellPolicy
        dwell_checkpoint = train_dwell_policy(args, output_dir)
        if not dwell_checkpoint:
            print("ERROR: DwellPolicy training failed")
            return 1

        print("\n  Training complete!")
        print(f"    SearchPolicy: {search_checkpoint}")
        print(f"    DwellPolicy: {dwell_checkpoint}")

    # Stage 2: Export to ONNX
    onnx_paths = []
    if args.export:
        print("\n" + "#"*60)
        print("  STAGE 2: ONNX EXPORT")
        print("#"*60)

        if not search_checkpoint or not dwell_checkpoint:
            print("ERROR: Checkpoints required for export")
            print("  Use --search-checkpoint and --dwell-checkpoint, or run with --train")
            return 1

        # Export SearchPolicy (40-dim obs, 4-dim action)
        search_onnx = output_dir / "models" / "search_policy.onnx"
        search_onnx.parent.mkdir(parents=True, exist_ok=True)
        if export_to_onnx(search_checkpoint, search_onnx, obs_dim=40, action_dim=4):
            onnx_paths.append(search_onnx)

        # Export DwellPolicy (25-dim obs, 6-dim action)
        dwell_onnx = output_dir / "models" / "dwell_policy.onnx"
        if export_to_onnx(dwell_checkpoint, dwell_onnx, obs_dim=25, action_dim=6):
            onnx_paths.append(dwell_onnx)

        if len(onnx_paths) < 2:
            print("WARNING: Some exports failed")

    # Stage 3: Benchmark
    if args.benchmark and onnx_paths:
        print("\n" + "#"*60)
        print("  STAGE 3: BENCHMARK")
        print("#"*60)

        for onnx_path in onnx_paths:
            obs_dim = 40 if "search" in onnx_path.name else 25
            benchmark_onnx(onnx_path, obs_dim)

    # Print deployment instructions
    if onnx_paths:
        print_tensorrt_instructions(onnx_paths)
        print_ros2_deployment_instructions(output_dir)

    # Summary
    print("\n" + "="*60)
    print("  PIPELINE COMPLETE")
    print("="*60)
    print(f"  Output: {output_dir}")
    if search_checkpoint:
        print(f"  SearchPolicy checkpoint: {search_checkpoint}")
    if dwell_checkpoint:
        print(f"  DwellPolicy checkpoint: {dwell_checkpoint}")
    if onnx_paths:
        print(f"  ONNX models: {[str(p) for p in onnx_paths]}")
    print("="*60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
