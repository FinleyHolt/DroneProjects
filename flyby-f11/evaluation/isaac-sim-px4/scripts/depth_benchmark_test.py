#!/usr/bin/env python3
"""
Fast offline depth estimation benchmark.

Runs on pre-captured benchmark dataset without requiring Isaac Sim.
Enables rapid iteration on depth correction algorithms (<30 seconds).

Dataset structure:
    benchmark/
    ├── metadata.json           # Dataset info, camera params
    └── frames/
        ├── 000000/
        │   ├── rgb.png         # RGB input (640x480)
        │   ├── depth_gt.npy    # Ground truth depth (float32, meters)
        │   ├── model_output.npy # Raw DA V2 output (float32, normalized)
        │   └── rangefinder.json # {"range": 45.2, "valid": true, "step": 0}
        ├── 000001/
        ...

Usage:
    # Compare methods
    python depth_benchmark_test.py --dataset ./output/benchmark --method affine

    # Run ablation study
    python depth_benchmark_test.py --dataset ./output/benchmark --ablation

    # Verbose output with per-frame metrics
    python depth_benchmark_test.py --dataset ./output/benchmark --verbose

Author: Finley Holt
"""

import argparse
import json
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional, Tuple
import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# Import correctors
from scale_corrector import ScaleCorrector
from affine_depth_corrector import (
    AffineDepthCorrector,
    HybridAffineCorrector,
    KalmanAffineFilter,
)


@dataclass
class FrameMetrics:
    """Metrics for a single frame."""
    frame_id: int
    rmse: float
    abs_rel: float
    sq_rel: float
    delta1: float
    delta2: float
    delta3: float
    scale: float
    shift: float
    confidence: float


@dataclass
class BenchmarkResults:
    """Aggregate benchmark results."""
    method: str
    num_frames: int
    elapsed_seconds: float

    # Depth metrics (averaged)
    rmse_mean: float
    rmse_std: float
    abs_rel_mean: float
    abs_rel_std: float
    delta1_mean: float
    delta2_mean: float
    delta3_mean: float

    # Parameter stability
    scale_mean: float
    scale_std: float
    shift_mean: float
    shift_std: float
    confidence_mean: float

    # Distance-specific metrics
    rmse_near: float   # 0-20m
    rmse_mid: float    # 20-50m
    rmse_far: float    # 50-100m


def compute_metrics(
    pred: np.ndarray,
    gt: np.ndarray,
    valid_mask: Optional[np.ndarray] = None,
    min_depth: float = 0.5,
    max_depth: float = 200.0,
) -> Dict[str, float]:
    """
    Compute depth estimation metrics.

    Args:
        pred: Predicted depth (H, W) in meters
        gt: Ground truth depth (H, W) in meters
        valid_mask: Optional mask for valid pixels
        min_depth: Minimum valid depth
        max_depth: Maximum valid depth

    Returns:
        Dictionary of metrics
    """
    if valid_mask is None:
        valid_mask = (gt > min_depth) & (gt < max_depth) & (pred > min_depth)

    if valid_mask.sum() == 0:
        return {
            "rmse": float('nan'),
            "abs_rel": float('nan'),
            "sq_rel": float('nan'),
            "delta1": 0.0,
            "delta2": 0.0,
            "delta3": 0.0,
        }

    pred_v = pred[valid_mask]
    gt_v = gt[valid_mask]

    # RMSE
    rmse = float(np.sqrt(np.mean((pred_v - gt_v) ** 2)))

    # Absolute relative error
    abs_rel = float(np.mean(np.abs(pred_v - gt_v) / gt_v))

    # Squared relative error
    sq_rel = float(np.mean(((pred_v - gt_v) ** 2) / gt_v))

    # Delta accuracy thresholds
    ratio = np.maximum(pred_v / gt_v, gt_v / pred_v)
    delta1 = float((ratio < 1.25).mean())
    delta2 = float((ratio < 1.25 ** 2).mean())
    delta3 = float((ratio < 1.25 ** 3).mean())

    return {
        "rmse": rmse,
        "abs_rel": abs_rel,
        "sq_rel": sq_rel,
        "delta1": delta1,
        "delta2": delta2,
        "delta3": delta3,
    }


def compute_distance_metrics(
    pred: np.ndarray,
    gt: np.ndarray,
    ranges: List[Tuple[float, float]] = [(0, 20), (20, 50), (50, 100)],
) -> Dict[str, float]:
    """
    Compute RMSE at different distance ranges.

    Args:
        pred: Predicted depth
        gt: Ground truth depth
        ranges: List of (min, max) distance ranges

    Returns:
        Dictionary with RMSE for each range
    """
    results = {}
    for min_d, max_d in ranges:
        mask = (gt >= min_d) & (gt < max_d) & (pred > 0.5)
        if mask.sum() > 0:
            rmse = float(np.sqrt(np.mean((pred[mask] - gt[mask]) ** 2)))
        else:
            rmse = float('nan')
        results[f"rmse_{min_d}_{max_d}"] = rmse
    return results


def create_corrector(method: str):
    """Create depth corrector by method name."""
    if method == "scale_only":
        return ScaleCorrector(alpha=0.3)
    elif method == "affine":
        return AffineDepthCorrector(window_size=20, alpha=0.3, min_samples=5)
    elif method == "affine_kalman":
        return HybridAffineCorrector(window_size=20, min_samples=5, use_kalman=True)
    elif method == "affine_robust":
        return HybridAffineCorrector(
            window_size=20, min_samples=5, use_kalman=True, use_robust_sampling=True
        )
    else:
        raise ValueError(f"Unknown method: {method}")


def run_benchmark(
    dataset_path: Path,
    method: str = "affine",
    max_frames: int = 100,
    verbose: bool = False,
) -> BenchmarkResults:
    """
    Run benchmark on dataset with specified method.

    Args:
        dataset_path: Path to benchmark dataset
        method: Correction method to use
        max_frames: Maximum frames to process
        verbose: Print per-frame metrics

    Returns:
        BenchmarkResults with aggregate metrics
    """
    frames_dir = dataset_path / "frames"
    if not frames_dir.exists():
        raise FileNotFoundError(f"Frames directory not found: {frames_dir}")

    frame_dirs = sorted(frames_dir.iterdir())[:max_frames]
    if len(frame_dirs) == 0:
        raise ValueError(f"No frames found in {frames_dir}")

    print(f"\nRunning benchmark: method={method}, frames={len(frame_dirs)}")

    # Create corrector
    corrector = create_corrector(method)

    all_metrics: List[FrameMetrics] = []
    start_time = time.perf_counter()

    for i, frame_dir in enumerate(frame_dirs):
        # Load frame data
        gt_path = frame_dir / "depth_gt.npy"
        model_path = frame_dir / "model_output.npy"
        rf_path = frame_dir / "rangefinder.json"

        if not all(p.exists() for p in [gt_path, model_path, rf_path]):
            if verbose:
                print(f"  Skipping {frame_dir.name}: missing files")
            continue

        gt_depth = np.load(gt_path)
        model_output = np.load(model_path)
        with open(rf_path) as f:
            rf_data = json.load(f)

        rangefinder = rf_data["range"]
        rf_valid = rf_data.get("valid", True)

        # Apply correction based on method
        if method == "scale_only":
            # Old method: invert disparity then scale
            if model_output.max() > 1.0:
                model_output = model_output / 255.0
            relative_depth = 1.0 / (model_output + 0.01)
            corrector.update(relative_depth, rangefinder, rf_valid)
            pred_depth = corrector.apply_scale(relative_depth)
            scale = corrector.scale_factor
            shift = 0.0
            confidence = corrector.confidence
        else:
            # New affine methods: direct model output
            if model_output.max() > 1.0:
                model_output = model_output / 255.0
            estimate = corrector.update(model_output, rangefinder, rf_valid)
            pred_depth = corrector.apply(model_output)
            scale = estimate.scale
            shift = estimate.shift
            confidence = estimate.confidence

        # Compute metrics
        valid_mask = (gt_depth > 0.5) & (gt_depth < 200) & (pred_depth > 0.5)
        metrics = compute_metrics(pred_depth, gt_depth, valid_mask)

        frame_metrics = FrameMetrics(
            frame_id=i,
            rmse=metrics["rmse"],
            abs_rel=metrics["abs_rel"],
            sq_rel=metrics["sq_rel"],
            delta1=metrics["delta1"],
            delta2=metrics["delta2"],
            delta3=metrics["delta3"],
            scale=scale,
            shift=shift,
            confidence=confidence,
        )
        all_metrics.append(frame_metrics)

        if verbose:
            print(
                f"  Frame {i:04d}: RMSE={metrics['rmse']:.2f}m, "
                f"δ1={metrics['delta1']*100:.1f}%, "
                f"scale={scale:.2f}, shift={shift:.2f}"
            )

    elapsed = time.perf_counter() - start_time

    # Aggregate results
    rmse_vals = [m.rmse for m in all_metrics if not np.isnan(m.rmse)]
    abs_rel_vals = [m.abs_rel for m in all_metrics if not np.isnan(m.abs_rel)]
    delta1_vals = [m.delta1 for m in all_metrics]
    delta2_vals = [m.delta2 for m in all_metrics]
    delta3_vals = [m.delta3 for m in all_metrics]
    scale_vals = [m.scale for m in all_metrics]
    shift_vals = [m.shift for m in all_metrics]
    conf_vals = [m.confidence for m in all_metrics]

    # Compute distance-specific metrics on last frame (representative)
    if len(frame_dirs) > 0:
        last_frame = frame_dirs[-1]
        gt_depth = np.load(last_frame / "depth_gt.npy")
        model_output = np.load(last_frame / "model_output.npy")
        if model_output.max() > 1.0:
            model_output = model_output / 255.0
        if method == "scale_only":
            relative_depth = 1.0 / (model_output + 0.01)
            pred_depth = corrector.apply_scale(relative_depth)
        else:
            pred_depth = corrector.apply(model_output)
        dist_metrics = compute_distance_metrics(pred_depth, gt_depth)
    else:
        dist_metrics = {"rmse_0_20": float('nan'), "rmse_20_50": float('nan'), "rmse_50_100": float('nan')}

    return BenchmarkResults(
        method=method,
        num_frames=len(all_metrics),
        elapsed_seconds=elapsed,
        rmse_mean=float(np.mean(rmse_vals)) if rmse_vals else float('nan'),
        rmse_std=float(np.std(rmse_vals)) if rmse_vals else float('nan'),
        abs_rel_mean=float(np.mean(abs_rel_vals)) if abs_rel_vals else float('nan'),
        abs_rel_std=float(np.std(abs_rel_vals)) if abs_rel_vals else float('nan'),
        delta1_mean=float(np.mean(delta1_vals)),
        delta2_mean=float(np.mean(delta2_vals)),
        delta3_mean=float(np.mean(delta3_vals)),
        scale_mean=float(np.mean(scale_vals)),
        scale_std=float(np.std(scale_vals)),
        shift_mean=float(np.mean(shift_vals)),
        shift_std=float(np.std(shift_vals)),
        confidence_mean=float(np.mean(conf_vals)),
        rmse_near=dist_metrics.get("rmse_0_20", float('nan')),
        rmse_mid=dist_metrics.get("rmse_20_50", float('nan')),
        rmse_far=dist_metrics.get("rmse_50_100", float('nan')),
    )


def run_ablation(dataset_path: Path, max_frames: int = 100) -> List[BenchmarkResults]:
    """
    Run ablation study comparing all methods.

    Args:
        dataset_path: Path to benchmark dataset
        max_frames: Maximum frames to process

    Returns:
        List of BenchmarkResults for each method
    """
    methods = ["scale_only", "affine", "affine_kalman", "affine_robust"]
    results = []

    for method in methods:
        try:
            result = run_benchmark(dataset_path, method, max_frames)
            results.append(result)
        except Exception as e:
            print(f"  Error with {method}: {e}")

    return results


def print_results(results: BenchmarkResults):
    """Print formatted benchmark results."""
    print(f"\n{'='*60}")
    print(f"Method: {results.method}")
    print(f"{'='*60}")
    print(f"Frames: {results.num_frames}, Time: {results.elapsed_seconds:.2f}s")
    print(f"\nDepth Metrics:")
    print(f"  RMSE:    {results.rmse_mean:.3f} ± {results.rmse_std:.3f} m")
    print(f"  AbsRel:  {results.abs_rel_mean:.3f} ± {results.abs_rel_std:.3f}")
    print(f"  δ < 1.25:    {results.delta1_mean*100:.1f}%")
    print(f"  δ < 1.25²:   {results.delta2_mean*100:.1f}%")
    print(f"  δ < 1.25³:   {results.delta3_mean*100:.1f}%")
    print(f"\nDistance-Specific RMSE:")
    print(f"  Near (0-20m):   {results.rmse_near:.3f} m")
    print(f"  Mid (20-50m):   {results.rmse_mid:.3f} m")
    print(f"  Far (50-100m):  {results.rmse_far:.3f} m")
    print(f"\nParameter Stability:")
    print(f"  Scale: {results.scale_mean:.3f} ± {results.scale_std:.3f}")
    print(f"  Shift: {results.shift_mean:.3f} ± {results.shift_std:.3f}")
    print(f"  Confidence: {results.confidence_mean:.2f}")


def print_comparison_table(results: List[BenchmarkResults]):
    """Print comparison table of all methods."""
    print(f"\n{'='*80}")
    print("ABLATION STUDY COMPARISON")
    print(f"{'='*80}")
    print(f"{'Method':<20} {'RMSE':<12} {'AbsRel':<10} {'δ1':<8} {'Scale σ':<10} {'Shift σ':<10}")
    print(f"{'-'*80}")

    for r in results:
        print(
            f"{r.method:<20} "
            f"{r.rmse_mean:>6.3f}±{r.rmse_std:<4.2f} "
            f"{r.abs_rel_mean:>6.3f}    "
            f"{r.delta1_mean*100:>5.1f}%  "
            f"{r.scale_std:>8.4f}  "
            f"{r.shift_std:>8.4f}"
        )

    print(f"{'='*80}")

    # Highlight best method
    if results:
        best_rmse = min(results, key=lambda r: r.rmse_mean if not np.isnan(r.rmse_mean) else float('inf'))
        best_delta1 = max(results, key=lambda r: r.delta1_mean)
        print(f"\nBest RMSE: {best_rmse.method} ({best_rmse.rmse_mean:.3f}m)")
        print(f"Best δ1:   {best_delta1.method} ({best_delta1.delta1_mean*100:.1f}%)")


def main():
    parser = argparse.ArgumentParser(
        description="Fast offline depth estimation benchmark"
    )
    parser.add_argument(
        "--dataset",
        type=str,
        required=True,
        help="Path to benchmark dataset directory",
    )
    parser.add_argument(
        "--method",
        type=str,
        default="affine",
        choices=["scale_only", "affine", "affine_kalman", "affine_robust"],
        help="Depth correction method to test",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=100,
        help="Maximum frames to process",
    )
    parser.add_argument(
        "--ablation",
        action="store_true",
        help="Run ablation study comparing all methods",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print per-frame metrics",
    )
    parser.add_argument(
        "--output",
        type=str,
        help="Save results to JSON file",
    )

    args = parser.parse_args()
    dataset_path = Path(args.dataset)

    if not dataset_path.exists():
        print(f"Error: Dataset not found: {dataset_path}")
        print("\nTo create a benchmark dataset, run depth_avoidance_test.py with")
        print("--save-benchmark flag (after implementing the capture code).")
        return 1

    if args.ablation:
        results = run_ablation(dataset_path, args.max_frames)
        print_comparison_table(results)

        if args.output:
            output_data = [asdict(r) for r in results]
            with open(args.output, "w") as f:
                json.dump(output_data, f, indent=2)
            print(f"\nResults saved to: {args.output}")
    else:
        results = run_benchmark(
            dataset_path, args.method, args.max_frames, args.verbose
        )
        print_results(results)

        if args.output:
            with open(args.output, "w") as f:
                json.dump(asdict(results), f, indent=2)
            print(f"\nResults saved to: {args.output}")

    return 0


if __name__ == "__main__":
    exit(main())
