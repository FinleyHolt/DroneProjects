#!/usr/bin/env python3
"""
Process raw timing data from container benchmark into final JSON results.

Reads raw_timings.csv and generates results.json with comprehensive statistics.
"""

import json
import csv
import statistics
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any
from collections import defaultdict
import platform


def calculate_percentile(sorted_values: List[float], percentile: float) -> float:
    """Calculate the given percentile from a sorted list of values."""
    if not sorted_values:
        return 0.0
    idx = int(len(sorted_values) * percentile / 100)
    idx = min(idx, len(sorted_values) - 1)
    return sorted_values[idx]


def get_category(query_name: str) -> str:
    """Determine category from query name."""
    if query_name.startswith("safety_"):
        return "safety"
    elif query_name.startswith("operational_"):
        return "operational"
    elif query_name.startswith("planning_"):
        return "planning"
    return "unknown"


def get_requirement(category: str) -> int:
    """Get latency requirement in ms for category."""
    requirements = {
        "safety": 10,
        "operational": 100,
        "planning": 1000
    }
    return requirements.get(category, 1000)


def process_timings(csv_file: str) -> Dict[str, Any]:
    """Process raw timings CSV into structured results."""
    query_data = defaultdict(lambda: {
        "times": [],
        "statuses": defaultdict(int),
        "memory": []
    })

    # Read CSV
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            query = row['query']
            elapsed = float(row['elapsed_ms'])
            status = row['status']
            memory = row.get('memory_mb', '')

            query_data[query]["statuses"][status] += 1

            # Only include successful runs in timing stats
            if status in ["THEOREM", "COUNTER_SATISFIABLE", "SATISFIABLE"]:
                query_data[query]["times"].append(elapsed)

            if memory:
                query_data[query]["memory"].append(float(memory))

    # Build results structure
    results = {
        "metadata": {
            "reasoner": "Vampire",
            "version": "5.0.0",
            "system": {
                "platform": platform.system(),
                "architecture": platform.machine(),
                "python_version": platform.python_version()
            },
            "timestamp": datetime.now().isoformat(),
            "iterations": 100,
            "csv_source": str(csv_file)
        },
        "categories": {
            "safety": {"queries": [], "requirement_ms": 10},
            "operational": {"queries": [], "requirement_ms": 100},
            "planning": {"queries": [], "requirement_ms": 1000}
        },
        "summary": {}
    }

    total_queries = 0
    total_passed = 0

    # Process each query
    for query_name in sorted(query_data.keys()):
        data = query_data[query_name]
        category = get_category(query_name)
        requirement = get_requirement(category)

        times = data["times"]
        iterations = sum(data["statuses"].values())

        if times:
            times_sorted = sorted(times)

            stats = {
                "name": query_name,
                "category": category,
                "iterations": iterations,
                "successful_runs": len(times),
                "cold_start_ms": round(times[0], 3),  # First run as cold start
                "mean_ms": round(statistics.mean(times), 3),
                "median_ms": round(statistics.median(times), 3),
                "min_ms": round(min(times), 3),
                "max_ms": round(max(times), 3),
                "stdev_ms": round(statistics.stdev(times), 3) if len(times) > 1 else 0.0,
                "p95_ms": round(calculate_percentile(times_sorted, 95), 3),
                "p99_ms": round(calculate_percentile(times_sorted, 99), 3),
                "success_rate": round(len(times) / iterations * 100, 2),
                "statuses": dict(data["statuses"]),
                "requirement_ms": requirement,
            }

            # Check if meets requirement
            stats["meets_requirement"] = stats["p95_ms"] < requirement
            if stats["meets_requirement"]:
                total_passed += 1

            # Add memory stats if available
            if data["memory"]:
                stats["memory_mean_mb"] = round(statistics.mean(data["memory"]), 1)
                stats["memory_max_mb"] = round(max(data["memory"]), 1)
        else:
            stats = {
                "name": query_name,
                "category": category,
                "iterations": iterations,
                "successful_runs": 0,
                "error": "All iterations failed",
                "statuses": dict(data["statuses"]),
                "success_rate": 0.0,
                "requirement_ms": requirement,
                "meets_requirement": False
            }

        if category in results["categories"]:
            results["categories"][category]["queries"].append(stats)
        total_queries += 1

    # Update category totals
    for category in ["safety", "operational", "planning"]:
        queries = results["categories"][category]["queries"]
        passed = sum(1 for q in queries if q.get("meets_requirement", False))
        results["categories"][category]["total"] = len(queries)
        results["categories"][category]["passed"] = passed

    # Calculate summary statistics
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

    print(f"\nReasoner: Vampire {results['metadata']['version']}")
    print(f"Timestamp: {results['metadata']['timestamp']}")

    print(f"\nOverall Results:")
    print(f"  Total queries: {summary['total_queries']}")
    print(f"  Meeting requirements: {summary['queries_meeting_requirements']} ({summary['overall_pass_rate']}%)")

    for category in ["safety", "operational", "planning"]:
        cat_summary = summary[category]
        status = "PASS" if cat_summary["meets_requirement"] else "FAIL"

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
    script_dir = Path(__file__).parent.resolve()
    csv_file = script_dir / "raw_timings.csv"
    results_file = script_dir / "results.json"

    if not csv_file.exists():
        print(f"ERROR: Raw timings file not found: {csv_file}")
        print("Run the container benchmark first.")
        return 1

    print(f"Processing: {csv_file}")

    results = process_timings(str(csv_file))

    # Save results
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)

    print_summary(results)
    print(f"\nResults saved to: {results_file}")

    return 0 if results["summary"]["overall_pass_rate"] >= 50 else 1


if __name__ == "__main__":
    exit(main())
