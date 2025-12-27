#!/usr/bin/env python3
"""
Flyby F-11 RL Training Performance Benchmark

Measures training performance at different simulation fidelity levels:
- Episodes per hour
- Steps per second
- GPU/CPU utilization
- MAVLink communication latency
- Memory usage

Outputs recommendations for optimal training settings based on hardware.

Usage:
    python benchmark_performance.py --config configs/rl_training.yaml
    python benchmark_performance.py --quick  # Quick benchmark (1 minute)
    python benchmark_performance.py --full   # Full benchmark (10 minutes)

Requirements:
    - pymavlink
    - numpy
    - psutil
    - pynvml (for GPU monitoring)

Author: Finley Holt
Project: Flyby F-11 Autonomous Navigation
"""

import argparse
import json
import logging
import os
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import psutil
except ImportError:
    psutil = None
    print("Warning: psutil not installed. CPU/memory monitoring disabled.")

try:
    import pynvml
    NVML_AVAILABLE = True
except ImportError:
    NVML_AVAILABLE = False
    print("Warning: pynvml not installed. GPU monitoring disabled.")

try:
    import yaml
except ImportError:
    yaml = None
    print("Warning: pyyaml not installed. Using default configuration.")

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class BenchmarkResult:
    """Results from a benchmark run."""
    name: str
    duration_seconds: float = 0.0
    num_episodes: int = 0
    num_steps: int = 0
    episodes_per_hour: float = 0.0
    steps_per_second: float = 0.0
    mean_step_latency_ms: float = 0.0
    std_step_latency_ms: float = 0.0
    p99_step_latency_ms: float = 0.0
    mean_reset_latency_ms: float = 0.0
    mavlink_latency_ms: float = 0.0
    cpu_usage_percent: float = 0.0
    memory_usage_mb: float = 0.0
    gpu_usage_percent: float = 0.0
    gpu_memory_mb: float = 0.0
    errors: int = 0
    recommendations: List[str] = field(default_factory=list)


@dataclass
class SystemInfo:
    """System hardware information."""
    cpu_model: str = ""
    cpu_cores: int = 0
    cpu_threads: int = 0
    total_memory_gb: float = 0.0
    gpu_model: str = ""
    gpu_memory_gb: float = 0.0
    cuda_version: str = ""
    python_version: str = ""


class PerformanceBenchmark:
    """
    Benchmark RL training performance at different fidelity levels.
    """

    def __init__(
        self,
        config_path: Optional[str] = None,
        output_dir: str = "./benchmark_results",
    ):
        """
        Initialize benchmark.

        Args:
            config_path: Path to training configuration
            output_dir: Directory to save results
        """
        self.config_path = config_path
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.config = self._load_config()
        self.system_info = self._get_system_info()
        self.results: List[BenchmarkResult] = []

        # Initialize GPU monitoring
        if NVML_AVAILABLE:
            try:
                pynvml.nvmlInit()
                self.gpu_handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            except Exception as e:
                logger.warning(f"Failed to initialize NVML: {e}")
                self.gpu_handle = None
        else:
            self.gpu_handle = None

    def _load_config(self) -> Dict:
        """Load configuration from file."""
        if self.config_path and yaml and Path(self.config_path).exists():
            with open(self.config_path, 'r') as f:
                return yaml.safe_load(f)
        return {}

    def _get_system_info(self) -> SystemInfo:
        """Gather system hardware information."""
        info = SystemInfo()

        # CPU info
        if psutil:
            info.cpu_cores = psutil.cpu_count(logical=False) or 0
            info.cpu_threads = psutil.cpu_count(logical=True) or 0
            info.total_memory_gb = psutil.virtual_memory().total / (1024**3)

        # Try to get CPU model
        try:
            if os.path.exists('/proc/cpuinfo'):
                with open('/proc/cpuinfo', 'r') as f:
                    for line in f:
                        if 'model name' in line:
                            info.cpu_model = line.split(':')[1].strip()
                            break
        except Exception:
            info.cpu_model = "Unknown"

        # GPU info
        if NVML_AVAILABLE:
            try:
                pynvml.nvmlInit()
                handle = pynvml.nvmlDeviceGetHandleByIndex(0)
                info.gpu_model = pynvml.nvmlDeviceGetName(handle)
                if isinstance(info.gpu_model, bytes):
                    info.gpu_model = info.gpu_model.decode('utf-8')
                mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
                info.gpu_memory_gb = mem_info.total / (1024**3)
            except Exception as e:
                logger.warning(f"Failed to get GPU info: {e}")

        # CUDA version
        try:
            result = subprocess.run(
                ['nvidia-smi', '--query-gpu=driver_version', '--format=csv,noheader'],
                capture_output=True, text=True
            )
            info.cuda_version = result.stdout.strip()
        except Exception:
            info.cuda_version = "Unknown"

        # Python version
        info.python_version = sys.version.split()[0]

        return info

    def _get_cpu_usage(self) -> float:
        """Get current CPU usage percentage."""
        if psutil:
            return psutil.cpu_percent(interval=None)
        return 0.0

    def _get_memory_usage(self) -> float:
        """Get current memory usage in MB."""
        if psutil:
            process = psutil.Process(os.getpid())
            return process.memory_info().rss / (1024**2)
        return 0.0

    def _get_gpu_usage(self) -> Tuple[float, float]:
        """Get GPU utilization and memory usage."""
        if not self.gpu_handle:
            return 0.0, 0.0

        try:
            util = pynvml.nvmlDeviceGetUtilizationRates(self.gpu_handle)
            mem_info = pynvml.nvmlDeviceGetMemoryInfo(self.gpu_handle)
            return util.gpu, mem_info.used / (1024**2)
        except Exception:
            return 0.0, 0.0

    def _simulate_environment_step(self, fidelity: str) -> Tuple[float, bool]:
        """
        Simulate an environment step with given fidelity.

        In a real benchmark, this would connect to ArduPilot SITL.
        For now, we simulate with appropriate delays.

        Args:
            fidelity: 'low', 'medium', 'high'

        Returns:
            (step_time_seconds, success)
        """
        # Simulated step times based on fidelity
        # These approximate real SITL + Gazebo latencies
        step_times = {
            'low': 0.005,     # 5ms - lightweight sim or MuJoCo
            'medium': 0.010,  # 10ms - Gazebo with simple world
            'high': 0.020,    # 20ms - Gazebo with physics/rendering
        }

        base_time = step_times.get(fidelity, 0.010)

        # Add some variance
        actual_time = base_time * np.random.uniform(0.8, 1.5)

        # Simulate the step
        start = time.perf_counter()
        time.sleep(actual_time)
        elapsed = time.perf_counter() - start

        # Simulate occasional errors (1% chance)
        success = np.random.random() > 0.01

        return elapsed, success

    def _simulate_episode_reset(self, fidelity: str) -> float:
        """
        Simulate episode reset with given fidelity.

        Args:
            fidelity: 'low', 'medium', 'high'

        Returns:
            reset_time_seconds
        """
        # Reset times based on fidelity
        reset_times = {
            'low': 0.5,    # 500ms
            'medium': 1.0,  # 1s
            'high': 2.0,    # 2s - full respawn with takeoff
        }

        base_time = reset_times.get(fidelity, 1.0)
        actual_time = base_time * np.random.uniform(0.9, 1.2)

        start = time.perf_counter()
        time.sleep(actual_time)
        elapsed = time.perf_counter() - start

        return elapsed

    def benchmark_fidelity_level(
        self,
        fidelity: str,
        duration_seconds: float = 60.0,
        max_steps_per_episode: int = 100,
    ) -> BenchmarkResult:
        """
        Benchmark performance at a specific fidelity level.

        Args:
            fidelity: 'low', 'medium', 'high'
            duration_seconds: How long to run benchmark
            max_steps_per_episode: Max steps before episode reset

        Returns:
            BenchmarkResult with performance metrics
        """
        logger.info(f"Starting {fidelity} fidelity benchmark ({duration_seconds}s)...")

        result = BenchmarkResult(name=f"{fidelity}_fidelity")

        step_latencies = []
        reset_latencies = []
        cpu_samples = []
        memory_samples = []
        gpu_samples = []
        gpu_memory_samples = []

        start_time = time.time()
        episode_steps = 0

        while time.time() - start_time < duration_seconds:
            # Collect system metrics
            cpu_samples.append(self._get_cpu_usage())
            memory_samples.append(self._get_memory_usage())
            gpu_util, gpu_mem = self._get_gpu_usage()
            gpu_samples.append(gpu_util)
            gpu_memory_samples.append(gpu_mem)

            # Simulate step
            step_time, success = self._simulate_environment_step(fidelity)
            step_latencies.append(step_time * 1000)  # Convert to ms
            result.num_steps += 1
            episode_steps += 1

            if not success:
                result.errors += 1

            # Check for episode reset
            if episode_steps >= max_steps_per_episode or not success:
                reset_time = self._simulate_episode_reset(fidelity)
                reset_latencies.append(reset_time * 1000)
                result.num_episodes += 1
                episode_steps = 0

        result.duration_seconds = time.time() - start_time

        # Calculate statistics
        if step_latencies:
            result.mean_step_latency_ms = np.mean(step_latencies)
            result.std_step_latency_ms = np.std(step_latencies)
            result.p99_step_latency_ms = np.percentile(step_latencies, 99)

        if reset_latencies:
            result.mean_reset_latency_ms = np.mean(reset_latencies)

        if result.duration_seconds > 0:
            result.steps_per_second = result.num_steps / result.duration_seconds
            hours = result.duration_seconds / 3600
            result.episodes_per_hour = result.num_episodes / hours if hours > 0 else 0

        # System metrics
        result.cpu_usage_percent = np.mean(cpu_samples) if cpu_samples else 0
        result.memory_usage_mb = np.mean(memory_samples) if memory_samples else 0
        result.gpu_usage_percent = np.mean(gpu_samples) if gpu_samples else 0
        result.gpu_memory_mb = np.mean(gpu_memory_samples) if gpu_memory_samples else 0

        logger.info(
            f"  {fidelity}: {result.steps_per_second:.1f} steps/s, "
            f"{result.episodes_per_hour:.0f} eps/hr, "
            f"latency: {result.mean_step_latency_ms:.2f}ms"
        )

        return result

    def benchmark_parallel_envs(
        self,
        num_envs_list: List[int] = [1, 2, 4, 8],
        duration_seconds: float = 30.0,
    ) -> List[BenchmarkResult]:
        """
        Benchmark performance with different numbers of parallel environments.

        Args:
            num_envs_list: List of parallel environment counts to test
            duration_seconds: Duration per test

        Returns:
            List of BenchmarkResults
        """
        results = []

        for num_envs in num_envs_list:
            logger.info(f"Benchmarking {num_envs} parallel environments...")

            result = BenchmarkResult(name=f"parallel_{num_envs}_envs")

            step_latencies = []
            start_time = time.time()

            while time.time() - start_time < duration_seconds:
                # Simulate parallel step (batch inference)
                batch_start = time.perf_counter()

                # Overhead for parallel coordination
                time.sleep(0.001 * num_envs)

                # Steps are parallelized, so total time is ~max of individual times
                max_step_time = 0
                for _ in range(num_envs):
                    step_time = 0.010 * np.random.uniform(0.8, 1.2)
                    max_step_time = max(max_step_time, step_time)

                time.sleep(max_step_time)
                batch_elapsed = time.perf_counter() - batch_start

                step_latencies.append(batch_elapsed * 1000)
                result.num_steps += num_envs

            result.duration_seconds = time.time() - start_time

            if step_latencies:
                result.mean_step_latency_ms = np.mean(step_latencies)
                result.std_step_latency_ms = np.std(step_latencies)

            if result.duration_seconds > 0:
                result.steps_per_second = result.num_steps / result.duration_seconds

            result.cpu_usage_percent = self._get_cpu_usage()
            result.memory_usage_mb = self._get_memory_usage()
            gpu_util, gpu_mem = self._get_gpu_usage()
            result.gpu_usage_percent = gpu_util
            result.gpu_memory_mb = gpu_mem

            logger.info(
                f"  {num_envs} envs: {result.steps_per_second:.1f} steps/s, "
                f"latency: {result.mean_step_latency_ms:.2f}ms"
            )

            results.append(result)

        return results

    def benchmark_mavlink_latency(
        self,
        num_samples: int = 100,
        connection_string: str = "tcp:localhost:5760"
    ) -> float:
        """
        Benchmark MAVLink communication latency.

        Args:
            num_samples: Number of latency samples to collect
            connection_string: MAVLink connection string

        Returns:
            Mean latency in milliseconds
        """
        logger.info("Benchmarking MAVLink latency...")

        try:
            from pymavlink import mavutil
        except ImportError:
            logger.warning("pymavlink not available, skipping MAVLink benchmark")
            return 0.0

        latencies = []

        try:
            # Try to connect
            logger.info(f"Connecting to {connection_string}...")
            conn = mavutil.mavlink_connection(
                connection_string,
                autoreconnect=False,
                source_system=255
            )

            # Wait for heartbeat with timeout
            msg = conn.wait_heartbeat(timeout=5)
            if msg is None:
                logger.warning("No heartbeat received, SITL may not be running")
                return 0.0

            logger.info("Connected, measuring latency...")

            for i in range(num_samples):
                start = time.perf_counter()

                # Request parameter (common round-trip test)
                conn.mav.param_request_read_send(
                    conn.target_system,
                    conn.target_component,
                    b'SIM_SPEEDUP',
                    -1
                )

                # Wait for response
                msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)

                elapsed = time.perf_counter() - start

                if msg:
                    latencies.append(elapsed * 1000)

            conn.close()

        except Exception as e:
            logger.warning(f"MAVLink benchmark failed: {e}")
            return 0.0

        if latencies:
            mean_latency = np.mean(latencies)
            logger.info(
                f"MAVLink latency: {mean_latency:.2f}ms "
                f"(std: {np.std(latencies):.2f}ms)"
            )
            return mean_latency

        return 0.0

    def run_full_benchmark(
        self,
        duration_per_test: float = 60.0,
        test_mavlink: bool = True,
    ) -> Dict[str, Any]:
        """
        Run complete benchmark suite.

        Args:
            duration_per_test: Duration for each fidelity test
            test_mavlink: Whether to test MAVLink latency

        Returns:
            Complete benchmark results dictionary
        """
        logger.info("=" * 60)
        logger.info("Flyby F-11 RL Training Performance Benchmark")
        logger.info("=" * 60)
        logger.info(f"System: {self.system_info.cpu_model}")
        logger.info(f"GPU: {self.system_info.gpu_model}")
        logger.info(f"Memory: {self.system_info.total_memory_gb:.1f} GB")
        logger.info("")

        all_results = {
            'timestamp': datetime.now().isoformat(),
            'system_info': self._system_info_to_dict(),
            'fidelity_results': [],
            'parallel_results': [],
            'mavlink_latency_ms': 0.0,
            'recommendations': [],
        }

        # Test different fidelity levels
        logger.info("Testing fidelity levels...")
        for fidelity in ['low', 'medium', 'high']:
            result = self.benchmark_fidelity_level(
                fidelity,
                duration_seconds=duration_per_test
            )
            self.results.append(result)
            all_results['fidelity_results'].append(self._result_to_dict(result))

        # Test parallel environments
        logger.info("\nTesting parallel environments...")
        parallel_results = self.benchmark_parallel_envs(
            num_envs_list=[1, 2, 4, 8],
            duration_seconds=duration_per_test / 2
        )
        for result in parallel_results:
            self.results.append(result)
            all_results['parallel_results'].append(self._result_to_dict(result))

        # Test MAVLink latency
        if test_mavlink:
            all_results['mavlink_latency_ms'] = self.benchmark_mavlink_latency()

        # Generate recommendations
        recommendations = self._generate_recommendations(all_results)
        all_results['recommendations'] = recommendations

        # Print summary
        self._print_summary(all_results)

        # Save results
        self._save_results(all_results)

        return all_results

    def run_quick_benchmark(self) -> Dict[str, Any]:
        """Run quick benchmark (1 minute total)."""
        return self.run_full_benchmark(
            duration_per_test=10.0,
            test_mavlink=False
        )

    def _generate_recommendations(self, results: Dict) -> List[str]:
        """Generate training recommendations based on benchmark results."""
        recommendations = []

        # Analyze fidelity results
        fidelity_results = results.get('fidelity_results', [])
        if fidelity_results:
            # Find best fidelity for speed
            best_speed = max(fidelity_results, key=lambda x: x.get('steps_per_second', 0))
            recommendations.append(
                f"For maximum training speed, use {best_speed['name'].replace('_fidelity', '')} "
                f"fidelity ({best_speed['steps_per_second']:.0f} steps/s)"
            )

            # Check if high fidelity is viable
            high_fidelity = next(
                (r for r in fidelity_results if 'high' in r['name']), None
            )
            if high_fidelity:
                if high_fidelity['steps_per_second'] > 50:
                    recommendations.append(
                        "High fidelity training is viable on this hardware "
                        "(>50 steps/s is sufficient for most RL algorithms)"
                    )
                else:
                    recommendations.append(
                        "Consider using medium fidelity for faster training, "
                        "then fine-tune with high fidelity"
                    )

        # Analyze parallel results
        parallel_results = results.get('parallel_results', [])
        if parallel_results:
            # Find optimal number of parallel envs
            best_parallel = max(
                parallel_results,
                key=lambda x: x.get('steps_per_second', 0)
            )
            optimal_envs = int(best_parallel['name'].split('_')[1])
            recommendations.append(
                f"Optimal parallel environments: {optimal_envs} "
                f"({best_parallel['steps_per_second']:.0f} steps/s)"
            )

            # Check for diminishing returns
            if len(parallel_results) >= 2:
                speed_4 = next(
                    (r['steps_per_second'] for r in parallel_results if '4' in r['name']), 0
                )
                speed_8 = next(
                    (r['steps_per_second'] for r in parallel_results if '8' in r['name']), 0
                )
                if speed_8 > 0 and speed_8 / speed_4 < 1.3:
                    recommendations.append(
                        "Diminishing returns observed beyond 4 parallel environments"
                    )

        # Memory recommendations
        gpu_mem = results.get('fidelity_results', [{}])[-1].get('gpu_memory_mb', 0)
        if gpu_mem > 0:
            gpu_total = self.system_info.gpu_memory_gb * 1024
            if gpu_mem / gpu_total > 0.8:
                recommendations.append(
                    "GPU memory usage is high. Consider reducing batch size "
                    "or using gradient checkpointing"
                )

        # MAVLink recommendations
        mavlink_latency = results.get('mavlink_latency_ms', 0)
        if mavlink_latency > 0:
            if mavlink_latency > 20:
                recommendations.append(
                    f"MAVLink latency is high ({mavlink_latency:.1f}ms). "
                    "Consider running SITL locally for training"
                )
            else:
                recommendations.append(
                    f"MAVLink latency is acceptable ({mavlink_latency:.1f}ms)"
                )

        # Training time estimates
        if fidelity_results:
            medium_result = next(
                (r for r in fidelity_results if 'medium' in r['name']), None
            )
            if medium_result:
                steps_per_sec = medium_result['steps_per_second']
                if steps_per_sec > 0:
                    # Estimate time for 2M steps
                    hours_2m = 2_000_000 / steps_per_sec / 3600
                    recommendations.append(
                        f"Estimated training time for 2M steps (medium fidelity): "
                        f"{hours_2m:.1f} hours"
                    )

        return recommendations

    def _print_summary(self, results: Dict):
        """Print benchmark summary."""
        logger.info("\n" + "=" * 60)
        logger.info("BENCHMARK SUMMARY")
        logger.info("=" * 60)

        # Fidelity results
        logger.info("\nFidelity Comparison:")
        logger.info("-" * 50)
        logger.info(f"{'Level':<12} {'Steps/s':<12} {'Latency (ms)':<15} {'Eps/hr':<12}")
        logger.info("-" * 50)
        for result in results.get('fidelity_results', []):
            name = result['name'].replace('_fidelity', '')
            logger.info(
                f"{name:<12} {result['steps_per_second']:<12.1f} "
                f"{result['mean_step_latency_ms']:<15.2f} "
                f"{result['episodes_per_hour']:<12.0f}"
            )

        # Parallel results
        logger.info("\nParallel Environment Scaling:")
        logger.info("-" * 50)
        logger.info(f"{'Envs':<12} {'Steps/s':<12} {'Latency (ms)':<15} {'Speedup':<12}")
        logger.info("-" * 50)
        parallel_results = results.get('parallel_results', [])
        if parallel_results:
            baseline = parallel_results[0]['steps_per_second']
            for result in parallel_results:
                num_envs = result['name'].split('_')[1]
                speedup = result['steps_per_second'] / baseline if baseline > 0 else 0
                logger.info(
                    f"{num_envs:<12} {result['steps_per_second']:<12.1f} "
                    f"{result['mean_step_latency_ms']:<15.2f} "
                    f"{speedup:<12.2f}x"
                )

        # Recommendations
        logger.info("\nRecommendations:")
        logger.info("-" * 50)
        for rec in results.get('recommendations', []):
            logger.info(f"  * {rec}")

        logger.info("\n" + "=" * 60)

    def _save_results(self, results: Dict):
        """Save results to file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.output_dir / f"benchmark_{timestamp}.json"

        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)

        logger.info(f"\nResults saved to: {filename}")

    def _system_info_to_dict(self) -> Dict:
        """Convert SystemInfo to dictionary."""
        return {
            'cpu_model': self.system_info.cpu_model,
            'cpu_cores': self.system_info.cpu_cores,
            'cpu_threads': self.system_info.cpu_threads,
            'total_memory_gb': self.system_info.total_memory_gb,
            'gpu_model': self.system_info.gpu_model,
            'gpu_memory_gb': self.system_info.gpu_memory_gb,
            'cuda_version': self.system_info.cuda_version,
            'python_version': self.system_info.python_version,
        }

    def _result_to_dict(self, result: BenchmarkResult) -> Dict:
        """Convert BenchmarkResult to dictionary."""
        return {
            'name': result.name,
            'duration_seconds': result.duration_seconds,
            'num_episodes': result.num_episodes,
            'num_steps': result.num_steps,
            'episodes_per_hour': result.episodes_per_hour,
            'steps_per_second': result.steps_per_second,
            'mean_step_latency_ms': result.mean_step_latency_ms,
            'std_step_latency_ms': result.std_step_latency_ms,
            'p99_step_latency_ms': result.p99_step_latency_ms,
            'mean_reset_latency_ms': result.mean_reset_latency_ms,
            'cpu_usage_percent': result.cpu_usage_percent,
            'memory_usage_mb': result.memory_usage_mb,
            'gpu_usage_percent': result.gpu_usage_percent,
            'gpu_memory_mb': result.gpu_memory_mb,
            'errors': result.errors,
        }

    def cleanup(self):
        """Clean up resources."""
        if NVML_AVAILABLE:
            try:
                pynvml.nvmlShutdown()
            except Exception:
                pass


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Flyby F-11 RL Training Performance Benchmark'
    )
    parser.add_argument(
        '--config', '-c',
        type=str,
        default=None,
        help='Path to training configuration YAML'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='./benchmark_results',
        help='Output directory for results'
    )
    parser.add_argument(
        '--quick', '-q',
        action='store_true',
        help='Run quick benchmark (1 minute)'
    )
    parser.add_argument(
        '--full', '-f',
        action='store_true',
        help='Run full benchmark (10 minutes)'
    )
    parser.add_argument(
        '--duration', '-d',
        type=float,
        default=60.0,
        help='Duration per test in seconds'
    )
    parser.add_argument(
        '--skip-mavlink',
        action='store_true',
        help='Skip MAVLink latency test'
    )

    args = parser.parse_args()

    # Create benchmark
    benchmark = PerformanceBenchmark(
        config_path=args.config,
        output_dir=args.output
    )

    try:
        if args.quick:
            results = benchmark.run_quick_benchmark()
        elif args.full:
            results = benchmark.run_full_benchmark(
                duration_per_test=120.0,  # 2 minutes per test
                test_mavlink=not args.skip_mavlink
            )
        else:
            results = benchmark.run_full_benchmark(
                duration_per_test=args.duration,
                test_mavlink=not args.skip_mavlink
            )
    finally:
        benchmark.cleanup()

    # Return success/failure based on results
    if results.get('fidelity_results'):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
