"""
ONNX export utilities for ISR policies.

Exports trained SB3 policies to ONNX format for deployment on Jetson Orin NX.
Supports both full policy export and actor-only export for deterministic inference.

Usage:
    python -m training.deployment.onnx_export \
        --checkpoint checkpoints/area_coverage_final.zip \
        --output models/area_coverage.onnx

Author: Finley Holt
"""

import argparse
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch
import torch.nn as nn


class PolicyWrapper(nn.Module):
    """
    Wrapper to extract deterministic action from SB3 policy.

    SB3 policies output (action_mean, action_std) for stochastic policies.
    For deployment, we only need the deterministic action (mean).
    """

    def __init__(self, policy: nn.Module, obs_dim: int, action_dim: int):
        """
        Initialize the policy wrapper.

        Args:
            policy: SB3 policy module
            obs_dim: Observation dimension
            action_dim: Action dimension
        """
        super().__init__()
        self.policy = policy
        self.obs_dim = obs_dim
        self.action_dim = action_dim

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        """
        Forward pass returning deterministic action.

        Args:
            observation: Input observation tensor of shape (batch, obs_dim)

        Returns:
            Action tensor of shape (batch, action_dim)
        """
        # Get features from policy
        features = self.policy.mlp_extractor.forward_actor(
            self.policy.extract_features(observation, self.policy.features_extractor)
        )

        # Get action from action network
        action_mean = self.policy.action_net(features)

        # Apply tanh squashing (SB3 uses squashed gaussian)
        action = torch.tanh(action_mean)

        return action


def export_to_onnx(
    checkpoint_path: str,
    output_path: str,
    obs_dim: int = 119,
    action_dim: int = 6,
    opset_version: int = 17,
    dynamic_batch: bool = True,
    simplify: bool = True,
    verbose: bool = True,
) -> str:
    """
    Export SB3 PPO policy to ONNX format.

    Args:
        checkpoint_path: Path to SB3 model checkpoint (.zip)
        output_path: Path for output ONNX file
        obs_dim: Observation dimension (default: 119 for ISR env)
        action_dim: Action dimension (default: 6 for ISR env)
        opset_version: ONNX opset version
        dynamic_batch: Allow dynamic batch size
        simplify: Simplify ONNX graph (requires onnx-simplifier)
        verbose: Print export details

    Returns:
        Path to exported ONNX file
    """
    from stable_baselines3 import PPO

    if verbose:
        print(f"[ONNX] Loading checkpoint: {checkpoint_path}")

    # Load model
    model = PPO.load(checkpoint_path, device="cpu")
    policy = model.policy

    # Get observation and action dimensions from model if available
    if hasattr(model, "observation_space"):
        obs_dim = model.observation_space.shape[0]
    if hasattr(model, "action_space"):
        action_dim = model.action_space.shape[0]

    if verbose:
        print(f"[ONNX] Observation dim: {obs_dim}")
        print(f"[ONNX] Action dim: {action_dim}")

    # Wrap policy for deterministic action output
    wrapped_policy = PolicyWrapper(policy, obs_dim, action_dim)
    wrapped_policy.eval()

    # Create dummy input
    dummy_input = torch.zeros((1, obs_dim), dtype=torch.float32)

    # Setup dynamic axes
    dynamic_axes = None
    if dynamic_batch:
        dynamic_axes = {
            "observation": {0: "batch_size"},
            "action": {0: "batch_size"},
        }

    # Export to ONNX
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if verbose:
        print(f"[ONNX] Exporting to: {output_path}")

    torch.onnx.export(
        wrapped_policy,
        dummy_input,
        str(output_path),
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes=dynamic_axes,
        opset_version=opset_version,
        do_constant_folding=True,
        export_params=True,
    )

    # Simplify if requested
    if simplify:
        try:
            import onnx
            from onnxsim import simplify as onnx_simplify

            if verbose:
                print("[ONNX] Simplifying model...")

            model_onnx = onnx.load(str(output_path))
            model_simp, check = onnx_simplify(model_onnx)

            if check:
                onnx.save(model_simp, str(output_path))
                if verbose:
                    print("[ONNX] Model simplified successfully")
            else:
                if verbose:
                    print("[WARN] Simplification check failed, using original")

        except ImportError:
            if verbose:
                print("[WARN] onnx-simplifier not installed, skipping simplification")

    if verbose:
        # Print model size
        file_size = output_path.stat().st_size / 1024
        print(f"[ONNX] Export complete! Size: {file_size:.1f} KB")

    return str(output_path)


def validate_onnx_export(
    checkpoint_path: str,
    onnx_path: str,
    num_samples: int = 100,
    tolerance: float = 1e-5,
    verbose: bool = True,
) -> Tuple[bool, float]:
    """
    Validate ONNX export by comparing outputs with PyTorch model.

    Args:
        checkpoint_path: Path to original SB3 checkpoint
        onnx_path: Path to exported ONNX model
        num_samples: Number of random inputs to test
        tolerance: Maximum allowed difference
        verbose: Print validation details

    Returns:
        Tuple of (is_valid, max_difference)
    """
    import onnxruntime as ort
    from stable_baselines3 import PPO

    if verbose:
        print(f"[VALIDATE] Comparing PyTorch and ONNX outputs...")

    # Load PyTorch model
    model = PPO.load(checkpoint_path, device="cpu")
    obs_dim = model.observation_space.shape[0]

    # Wrap for deterministic action
    wrapped_policy = PolicyWrapper(model.policy, obs_dim, model.action_space.shape[0])
    wrapped_policy.eval()

    # Load ONNX model
    ort_session = ort.InferenceSession(onnx_path)

    max_diff = 0.0
    all_valid = True

    for i in range(num_samples):
        # Random observation
        obs = np.random.randn(1, obs_dim).astype(np.float32)

        # PyTorch inference
        with torch.no_grad():
            torch_action = wrapped_policy(torch.from_numpy(obs)).numpy()

        # ONNX inference
        onnx_action = ort_session.run(None, {"observation": obs})[0]

        # Compare
        diff = np.abs(torch_action - onnx_action).max()
        max_diff = max(max_diff, diff)

        if diff > tolerance:
            all_valid = False
            if verbose:
                print(f"[VALIDATE] Sample {i}: diff={diff:.2e} (FAIL)")

    if verbose:
        status = "PASS" if all_valid else "FAIL"
        print(f"[VALIDATE] Max difference: {max_diff:.2e}")
        print(f"[VALIDATE] Result: {status}")

    return all_valid, max_diff


def benchmark_onnx_inference(
    onnx_path: str,
    obs_dim: int = 119,
    num_iterations: int = 1000,
    warmup: int = 100,
    verbose: bool = True,
) -> dict:
    """
    Benchmark ONNX inference latency.

    Args:
        onnx_path: Path to ONNX model
        obs_dim: Observation dimension
        num_iterations: Number of inference iterations
        warmup: Warmup iterations before timing
        verbose: Print benchmark results

    Returns:
        Dict with latency statistics
    """
    import time
    import onnxruntime as ort

    if verbose:
        print(f"[BENCHMARK] Testing inference latency...")

    # Load model
    sess_options = ort.SessionOptions()
    sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

    session = ort.InferenceSession(onnx_path, sess_options)

    # Warmup
    dummy_input = np.random.randn(1, obs_dim).astype(np.float32)
    for _ in range(warmup):
        session.run(None, {"observation": dummy_input})

    # Benchmark
    latencies = []
    for _ in range(num_iterations):
        obs = np.random.randn(1, obs_dim).astype(np.float32)

        start = time.perf_counter()
        session.run(None, {"observation": obs})
        end = time.perf_counter()

        latencies.append((end - start) * 1000)  # Convert to ms

    results = {
        "mean_ms": np.mean(latencies),
        "std_ms": np.std(latencies),
        "min_ms": np.min(latencies),
        "max_ms": np.max(latencies),
        "p50_ms": np.percentile(latencies, 50),
        "p95_ms": np.percentile(latencies, 95),
        "p99_ms": np.percentile(latencies, 99),
        "max_hz": 1000 / np.max(latencies),
    }

    if verbose:
        print(f"[BENCHMARK] Results ({num_iterations} iterations):")
        print(f"  Mean: {results['mean_ms']:.3f} ms")
        print(f"  Std:  {results['std_ms']:.3f} ms")
        print(f"  Min:  {results['min_ms']:.3f} ms")
        print(f"  Max:  {results['max_ms']:.3f} ms")
        print(f"  P95:  {results['p95_ms']:.3f} ms")
        print(f"  P99:  {results['p99_ms']:.3f} ms")
        print(f"  Max sustainable Hz: {results['max_hz']:.0f}")

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Export SB3 policy to ONNX",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--checkpoint",
        type=str,
        required=True,
        help="Path to SB3 model checkpoint (.zip)",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output path for ONNX file",
    )
    parser.add_argument(
        "--obs_dim",
        type=int,
        default=119,
        help="Observation dimension",
    )
    parser.add_argument(
        "--action_dim",
        type=int,
        default=6,
        help="Action dimension",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=17,
        help="ONNX opset version",
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Validate export against PyTorch",
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Benchmark inference latency",
    )
    parser.add_argument(
        "--no-simplify",
        action="store_true",
        help="Skip ONNX simplification",
    )

    args = parser.parse_args()

    # Export
    output_path = export_to_onnx(
        checkpoint_path=args.checkpoint,
        output_path=args.output,
        obs_dim=args.obs_dim,
        action_dim=args.action_dim,
        opset_version=args.opset,
        simplify=not args.no_simplify,
        verbose=True,
    )

    # Validate
    if args.validate:
        print()
        is_valid, max_diff = validate_onnx_export(
            checkpoint_path=args.checkpoint,
            onnx_path=output_path,
            verbose=True,
        )
        if not is_valid:
            print("[ERROR] Validation failed!")
            return 1

    # Benchmark
    if args.benchmark:
        print()
        benchmark_onnx_inference(
            onnx_path=output_path,
            obs_dim=args.obs_dim,
            verbose=True,
        )

    return 0


if __name__ == "__main__":
    exit(main())
