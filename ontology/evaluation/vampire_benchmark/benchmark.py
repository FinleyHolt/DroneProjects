#!/usr/bin/env python3
"""
Vampire Theorem Prover Benchmark Suite
Measures query latency for UAV domain ontology reasoning

Runs each query multiple times and collects comprehensive timing statistics:
- cold_start_ms: First run timing (before any caching)
- mean_ms, median_ms: Central tendency measures
- min_ms, max_ms: Range of timings
- p95_ms, p99_ms: Tail latency percentiles
- success_rate: Percentage of successful runs
- memory_usage_mb: Peak memory usage (from Vampire output)

Categories with requirements:
- safety_*.tptp: Must be <10ms (p95) - real-time flight safety
- operational_*.tptp: Must be <100ms (p95) - mission operations
- planning_*.tptp: Must be <1000ms (p95) - pre-flight planning
"""

import json
import subprocess
import time
import statistics
import os
import sys
import re
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
import platform


def parse_vampire_output(output: str) -> Tuple[str, Optional[float], Optional[float]]:
    """
    Parse Vampire output to extract status, time elapsed, and memory usage.

    Returns:
        (status, time_elapsed_s, memory_mb)
    """
    # Determine status from SZS output
    if "SZS status Theorem" in output:
        status = "THEOREM"
    elif "SZS status CounterSatisfiable" in output:
        status = "COUNTER_SATISFIABLE"
    elif "SZS status Satisfiable" in output:
        status = "SATISFIABLE"
    elif "SZS status Timeout" in output:
        status = "TIMEOUT"
    elif "SZS status GaveUp" in output:
        status = "GAVE_UP"
    elif "SZS status ResourceOut" in output:
        status = "RESOURCE_OUT"
    else:
        status = "UNKNOWN"

    # Parse time elapsed from output
    time_elapsed = None
    time_match = re.search(r"Time elapsed:\s*([\d.]+)\s*s", output)
    if time_match:
        time_elapsed = float(time_match.group(1))

    # Parse memory usage from output
    memory_mb = None
    memory_match = re.search(r"Peak memory usage:\s*(\d+)\s*MB", output)
    if memory_match:
        memory_mb = float(memory_match.group(1))

    return status, time_elapsed, memory_mb


def run_vampire_query(query_file: str, timeout: int = 30) -> Dict[str, Any]:
    """
    Run a single Vampire query and return timing/result info.

    Uses wall-clock time for consistent measurement across all conditions.
    """
    start_time = time.perf_counter()

    try:
        result = subprocess.run(
            ["vampire", "--input_syntax", "tptp", "--time_limit", str(timeout), query_file],
            capture_output=True,
            text=True,
            timeout=timeout + 5
        )
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000

        # Combine stdout and stderr for parsing
        output = result.stdout + result.stderr
        status, vampire_time, memory_mb = parse_vampire_output(output)

        return {
            "elapsed_ms": elapsed_ms,
            "vampire_time_ms": vampire_time * 1000 if vampire_time else None,
            "status": status,
            "returncode": result.returncode,
            "memory_mb": memory_mb
        }

    except subprocess.TimeoutExpired:
        return {
            "elapsed_ms": timeout * 1000,
            "vampire_time_ms": None,
            "status": "TIMEOUT",
            "returncode": -1,
            "memory_mb": None
        }
    except FileNotFoundError:
        return {
            "elapsed_ms": 0,
            "vampire_time_ms": None,
            "status": "ERROR",
            "error": "vampire executable not found",
            "returncode": -1,
            "memory_mb": None
        }
    except Exception as e:
        return {
            "elapsed_ms": 0,
            "vampire_time_ms": None,
            "status": "ERROR",
            "error": str(e),
            "returncode": -1,
            "memory_mb": None
        }


def calculate_percentile(sorted_values: List[float], percentile: float) -> float:
    """Calculate the given percentile from a sorted list of values."""
    if not sorted_values:
        return 0.0
    idx = int(len(sorted_values) * percentile / 100)
    idx = min(idx, len(sorted_values) - 1)
    return sorted_values[idx]


def benchmark_query(query_file: str, iterations: int = 100) -> Dict[str, Any]:
    """
    Benchmark a single query with multiple iterations.

    Collects comprehensive timing statistics including cold start,
    percentiles, and success rates.
    """
    query_name = Path(query_file).stem
    print(f"  Benchmarking {query_name}... ", end="", flush=True)

    # Cold start (first run - may have higher latency due to disk/cache)
    cold_result = run_vampire_query(query_file)
    cold_start_ms = cold_result["elapsed_ms"]
    cold_status = cold_result["status"]

    # Warm runs
    times = []
    vampire_times = []
    memory_values = []
    statuses = {
        "THEOREM": 0,
        "COUNTER_SATISFIABLE": 0,
        "SATISFIABLE": 0,
        "TIMEOUT": 0,
        "ERROR": 0,
        "UNKNOWN": 0,
        "GAVE_UP": 0,
        "RESOURCE_OUT": 0
    }

    for i in range(iterations):
        result = run_vampire_query(query_file)

        # Count status
        status = result["status"]
        statuses[status] = statuses.get(status, 0) + 1

        # Only include successful runs in timing statistics
        if status in ["THEOREM", "COUNTER_SATISFIABLE", "SATISFIABLE"]:
            times.append(result["elapsed_ms"])
            if result.get("vampire_time_ms") is not None:
                vampire_times.append(result["vampire_time_ms"])

        # Track memory usage
        if result.get("memory_mb") is not None:
            memory_values.append(result["memory_mb"])

    # Calculate statistics
    if times:
        times_sorted = sorted(times)

        stats = {
            "name": query_name,
            "file": str(query_file),
            "iterations": iterations,
            "successful_runs": len(times),
            "cold_start_ms": round(cold_start_ms, 3),
            "cold_start_status": cold_status,
            "mean_ms": round(statistics.mean(times), 3),
            "median_ms": round(statistics.median(times), 3),
            "min_ms": round(min(times), 3),
            "max_ms": round(max(times), 3),
            "stdev_ms": round(statistics.stdev(times), 3) if len(times) > 1 else 0.0,
            "p95_ms": round(calculate_percentile(times_sorted, 95), 3),
            "p99_ms": round(calculate_percentile(times_sorted, 99), 3),
            "success_rate": round(len(times) / iterations * 100, 2),
            "statuses": {k: v for k, v in statuses.items() if v > 0},
        }

        # Add Vampire's internal timing if available
        if vampire_times:
            stats["vampire_mean_ms"] = round(statistics.mean(vampire_times), 3)
            stats["vampire_median_ms"] = round(statistics.median(vampire_times), 3)

        # Add memory stats if available
        if memory_values:
            stats["memory_mean_mb"] = round(statistics.mean(memory_values), 1)
            stats["memory_max_mb"] = round(max(memory_values), 1)

        print(f"OK (mean: {stats['mean_ms']:.1f}ms, p95: {stats['p95_ms']:.1f}ms)")
    else:
        stats = {
            "name": query_name,
            "file": str(query_file),
            "iterations": iterations,
            "successful_runs": 0,
            "cold_start_ms": round(cold_start_ms, 3),
            "cold_start_status": cold_status,
            "error": "All iterations failed",
            "statuses": {k: v for k, v in statuses.items() if v > 0},
            "success_rate": 0.0
        }
        print("FAILED (all iterations failed)")

    return stats


def get_vampire_version() -> str:
    """Get Vampire version string."""
    try:
        result = subprocess.run(
            ["vampire", "--version"],
            capture_output=True,
            text=True,
            timeout=5
        )
        version_line = (result.stdout + result.stderr).strip().split('\n')[0]
        return version_line or "unknown"
    except Exception:
        return "unknown"


def get_system_info() -> Dict[str, str]:
    """Collect system information for reproducibility."""
    return {
        "platform": platform.system(),
        "platform_release": platform.release(),
        "architecture": platform.machine(),
        "processor": platform.processor() or "unknown",
        "python_version": platform.python_version(),
        "hostname": platform.node()
    }


def run_all_benchmarks(queries_dir: str, iterations: int = 100) -> Dict[str, Any]:
    """
    Run benchmarks for all query categories.

    Categories:
    - safety: Real-time safety checks, must be <10ms (p95)
    - operational: Mission operations, must be <100ms (p95)
    - planning: Pre-flight planning, must be <1000ms (p95)
    """
    queries_path = Path(queries_dir)

    results = {
        "metadata": {
            "reasoner": "Vampire",
            "version": get_vampire_version(),
            "system": get_system_info(),
            "timestamp": datetime.now().isoformat(),
            "iterations": iterations,
            "queries_directory": str(queries_path.absolute())
        },
        "categories": {
            "safety": {"queries": [], "requirement_ms": 10},
            "operational": {"queries": [], "requirement_ms": 100},
            "planning": {"queries": [], "requirement_ms": 1000}
        },
        "summary": {}
    }

    # Category definitions with requirements
    categories = {
        "safety": {"pattern": "safety_*.tptp", "requirement_ms": 10},
        "operational": {"pattern": "operational_*.tptp", "requirement_ms": 100},
        "planning": {"pattern": "planning_*.tptp", "requirement_ms": 1000}
    }

    total_queries = 0
    total_passed = 0
    all_results = []

    for category, config in categories.items():
        print(f"\n{'='*60}")
        print(f"Category: {category.upper()} (requirement: <{config['requirement_ms']}ms p95)")
        print(f"{'='*60}")

        query_files = sorted(queries_path.glob(config["pattern"]))
        category_passed = 0

        for query_file in query_files:
            stats = benchmark_query(str(query_file), iterations)
            stats["requirement_ms"] = config["requirement_ms"]
            stats["category"] = category

            # Check if meets requirement
            if "p95_ms" in stats:
                stats["meets_requirement"] = stats["p95_ms"] < config["requirement_ms"]
                if stats["meets_requirement"]:
                    category_passed += 1
                    total_passed += 1
            else:
                stats["meets_requirement"] = False

            results["categories"][category]["queries"].append(stats)
            all_results.append(stats)
            total_queries += 1

        # Category summary
        results["categories"][category]["total"] = len(query_files)
        results["categories"][category]["passed"] = category_passed

    # Calculate comprehensive summary statistics
    def get_p95_values(category: str) -> List[float]:
        return [s["p95_ms"] for s in results["categories"][category]["queries"] if "p95_ms" in s]

    def get_mean_values(category: str) -> List[float]:
        return [s["mean_ms"] for s in results["categories"][category]["queries"] if "mean_ms" in s]

    safety_p95 = get_p95_values("safety")
    operational_p95 = get_p95_values("operational")
    planning_p95 = get_p95_values("planning")

    results["summary"] = {
        "total_queries": total_queries,
        "queries_meeting_requirements": total_passed,
        "overall_pass_rate": round(total_passed / total_queries * 100, 1) if total_queries > 0 else 0,

        "safety": {
            "count": len(safety_p95),
            "passed": results["categories"]["safety"]["passed"],
            "pass_rate": round(results["categories"]["safety"]["passed"] / len(safety_p95) * 100, 1) if safety_p95 else 0,
            "requirement_ms": 10,
            "avg_p95_ms": round(statistics.mean(safety_p95), 3) if safety_p95 else None,
            "max_p95_ms": round(max(safety_p95), 3) if safety_p95 else None,
            "avg_mean_ms": round(statistics.mean(get_mean_values("safety")), 3) if get_mean_values("safety") else None,
            "meets_requirement": max(safety_p95) < 10 if safety_p95 else False
        },
        "operational": {
            "count": len(operational_p95),
            "passed": results["categories"]["operational"]["passed"],
            "pass_rate": round(results["categories"]["operational"]["passed"] / len(operational_p95) * 100, 1) if operational_p95 else 0,
            "requirement_ms": 100,
            "avg_p95_ms": round(statistics.mean(operational_p95), 3) if operational_p95 else None,
            "max_p95_ms": round(max(operational_p95), 3) if operational_p95 else None,
            "avg_mean_ms": round(statistics.mean(get_mean_values("operational")), 3) if get_mean_values("operational") else None,
            "meets_requirement": max(operational_p95) < 100 if operational_p95 else False
        },
        "planning": {
            "count": len(planning_p95),
            "passed": results["categories"]["planning"]["passed"],
            "pass_rate": round(results["categories"]["planning"]["passed"] / len(planning_p95) * 100, 1) if planning_p95 else 0,
            "requirement_ms": 1000,
            "avg_p95_ms": round(statistics.mean(planning_p95), 3) if planning_p95 else None,
            "max_p95_ms": round(max(planning_p95), 3) if planning_p95 else None,
            "avg_mean_ms": round(statistics.mean(get_mean_values("planning")), 3) if get_mean_values("planning") else None,
            "meets_requirement": max(planning_p95) < 1000 if planning_p95 else False
        }
    }

    return results


def print_summary(results: Dict[str, Any]) -> None:
    """Print a formatted summary of benchmark results."""
    summary = results["summary"]

    print("\n" + "=" * 60)
    print("BENCHMARK SUMMARY")
    print("=" * 60)

    print(f"\nReasoner: {results['metadata']['reasoner']} {results['metadata']['version'].split()[1] if len(results['metadata']['version'].split()) > 1 else ''}")
    print(f"Timestamp: {results['metadata']['timestamp']}")
    print(f"Iterations per query: {results['metadata']['iterations']}")

    print(f"\nOverall Results:")
    print(f"  Total queries: {summary['total_queries']}")
    print(f"  Meeting requirements: {summary['queries_meeting_requirements']} ({summary['overall_pass_rate']}%)")

    for category in ["safety", "operational", "planning"]:
        cat_summary = summary[category]
        status = "PASS" if cat_summary["meets_requirement"] else "FAIL"
        status_color = "" if status == "PASS" else ""

        print(f"\n{category.upper()} Queries (requirement: <{cat_summary['requirement_ms']}ms p95):")
        print(f"  Status: {status}")
        print(f"  Queries: {cat_summary['passed']}/{cat_summary['count']} passed ({cat_summary['pass_rate']}%)")
        if cat_summary['avg_mean_ms'] is not None:
            print(f"  Average mean: {cat_summary['avg_mean_ms']}ms")
        if cat_summary['avg_p95_ms'] is not None:
            print(f"  Average p95: {cat_summary['avg_p95_ms']}ms")
        if cat_summary['max_p95_ms'] is not None:
            print(f"  Worst p95: {cat_summary['max_p95_ms']}ms")


def main():
    """Main entry point for the benchmark suite."""
    # Determine paths
    script_dir = Path(__file__).parent.resolve()
    queries_dir = script_dir.parent / "benchmark_queries"
    results_file = script_dir / "results.json"

    # Parse command line arguments
    iterations = 100
    if len(sys.argv) > 1:
        try:
            iterations = int(sys.argv[1])
        except ValueError:
            print(f"ERROR: Invalid iterations argument: {sys.argv[1]}")
            sys.exit(1)

    print("=" * 60)
    print("Vampire Theorem Prover Benchmark Suite")
    print("=" * 60)
    print(f"Queries directory: {queries_dir}")
    print(f"Iterations per query: {iterations}")
    print(f"Results file: {results_file}")

    # Verify queries directory exists
    if not queries_dir.exists():
        print(f"ERROR: Queries directory not found: {queries_dir}")
        sys.exit(1)

    # Verify Vampire is available
    vampire_version = get_vampire_version()
    if vampire_version == "unknown":
        print("ERROR: Vampire executable not found in PATH")
        sys.exit(1)
    print(f"Vampire version: {vampire_version}")

    # Run benchmarks
    results = run_all_benchmarks(str(queries_dir), iterations)

    # Save results to JSON
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)

    # Print summary
    print_summary(results)

    print(f"\nResults saved to: {results_file}")

    # Exit with appropriate code based on pass rate
    if results["summary"]["overall_pass_rate"] < 100:
        print("\nWARNING: Not all queries meet latency requirements")
        sys.exit(1 if results["summary"]["overall_pass_rate"] < 50 else 0)
    else:
        print("\nSUCCESS: All queries meet latency requirements")
        sys.exit(0)


if __name__ == "__main__":
    main()
