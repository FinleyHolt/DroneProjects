#!/usr/bin/env python3
"""
Statistical analysis for Vampire benchmark results.
Calculates confidence intervals, coefficient of variation, IQR, and outlier detection.
"""

import json
import math
import csv
from collections import defaultdict
from typing import Dict, List, Any

def calculate_statistics(values: List[float]) -> Dict[str, Any]:
    """Calculate comprehensive statistics for a list of timing values."""
    n = len(values)
    if n == 0:
        return {}

    sorted_vals = sorted(values)

    # Basic statistics
    mean = sum(values) / n
    variance = sum((x - mean) ** 2 for x in values) / (n - 1) if n > 1 else 0
    stdev = math.sqrt(variance)

    # Standard error
    std_error = stdev / math.sqrt(n) if n > 0 else 0

    # 95% confidence interval (using t-distribution approximation)
    # For n=100, t_0.975 ~ 1.984 (close to 1.96 for z-distribution)
    if n >= 30:
        # Use z-approximation for large samples
        z_975 = 1.96
        ci_lower = mean - z_975 * std_error
        ci_upper = mean + z_975 * std_error
    else:
        # t-distribution critical values for small samples
        t_values = {
            10: 2.262, 15: 2.131, 20: 2.093, 25: 2.064, 30: 2.042
        }
        t_crit = t_values.get(n, 2.0)
        ci_lower = mean - t_crit * std_error
        ci_upper = mean + t_crit * std_error

    # Coefficient of variation (CV) as percentage
    cv_percent = (stdev / mean * 100) if mean != 0 else 0

    # Quartiles and IQR
    def percentile(data: List[float], p: float) -> float:
        """Calculate percentile using linear interpolation."""
        k = (len(data) - 1) * p / 100
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return data[int(k)]
        return data[int(f)] * (c - k) + data[int(c)] * (k - f)

    q1 = percentile(sorted_vals, 25)
    median = percentile(sorted_vals, 50)
    q3 = percentile(sorted_vals, 75)
    iqr = q3 - q1

    # Outlier detection (1.5 * IQR rule)
    lower_fence = q1 - 1.5 * iqr
    upper_fence = q3 + 1.5 * iqr
    outliers = [v for v in values if v < lower_fence or v > upper_fence]
    outlier_count = len(outliers)

    # Additional percentiles
    p95 = percentile(sorted_vals, 95)
    p99 = percentile(sorted_vals, 99)

    return {
        "n": n,
        "mean_ms": round(mean, 3),
        "median_ms": round(median, 3),
        "min_ms": round(min(values), 3),
        "max_ms": round(max(values), 3),
        "stdev_ms": round(stdev, 3),
        "p95_ms": round(p95, 3),
        "p99_ms": round(p99, 3),
        "standard_error_ms": round(std_error, 3),
        "confidence_interval_95": {
            "lower_ms": round(ci_lower, 3),
            "upper_ms": round(ci_upper, 3),
            "margin_of_error_ms": round(z_975 * std_error if n >= 30 else 2.0 * std_error, 3)
        },
        "coefficient_of_variation_percent": round(cv_percent, 2),
        "interquartile_range": {
            "q1_ms": round(q1, 3),
            "q3_ms": round(q3, 3),
            "iqr_ms": round(iqr, 3)
        },
        "outliers": {
            "lower_fence_ms": round(lower_fence, 3),
            "upper_fence_ms": round(upper_fence, 3),
            "count": outlier_count,
            "values_ms": [round(v, 3) for v in sorted(outliers)]
        }
    }


def load_raw_timings(csv_path: str) -> Dict[str, List[float]]:
    """Load raw timing data from CSV, grouped by query name."""
    query_timings = defaultdict(list)

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            query_name = row['query']
            elapsed_ms = float(row['elapsed_ms'])
            query_timings[query_name].append(elapsed_ms)

    return dict(query_timings)


def load_existing_results(json_path: str) -> Dict:
    """Load existing results JSON."""
    with open(json_path, 'r') as f:
        return json.load(f)


def enhance_results(original_results: Dict, query_timings: Dict[str, List[float]]) -> Dict:
    """Enhance results with additional statistical analysis."""
    enhanced = {
        "metadata": original_results["metadata"].copy(),
        "statistical_analysis": {
            "method": "frequentist",
            "confidence_level": 0.95,
            "outlier_method": "IQR_1.5x",
            "notes": [
                "95% CI uses z-distribution (n=100 is sufficiently large)",
                "CV < 10% indicates low variability; > 20% indicates high variability",
                "Outliers detected using Tukey's fences (1.5 * IQR from quartiles)"
            ]
        },
        "categories": {},
        "summary": original_results["summary"].copy()
    }

    # Process each category
    for cat_name, cat_data in original_results["categories"].items():
        enhanced_queries = []

        for query in cat_data["queries"]:
            query_name = query["name"]
            timings = query_timings.get(query_name, [])

            if not timings:
                print(f"Warning: No timing data found for {query_name}")
                continue

            # Calculate enhanced statistics
            stats = calculate_statistics(timings)

            enhanced_query = {
                "name": query_name,
                "category": query["category"],
                "iterations": query["iterations"],
                "successful_runs": query["successful_runs"],
                "cold_start_ms": query["cold_start_ms"],
                "statuses": query["statuses"],
                "requirement_ms": query["requirement_ms"],
                "meets_requirement": query["meets_requirement"],
                "memory_mean_mb": query["memory_mean_mb"],
                "memory_max_mb": query["memory_max_mb"],
                "statistics": stats
            }

            enhanced_queries.append(enhanced_query)

        enhanced["categories"][cat_name] = {
            "queries": enhanced_queries,
            "requirement_ms": cat_data["requirement_ms"],
            "total": cat_data["total"],
            "passed": cat_data["passed"]
        }

    # Add category-level statistical summary
    for cat_name, cat_data in enhanced["categories"].items():
        cat_stats = []
        for q in cat_data["queries"]:
            cat_stats.append({
                "mean": q["statistics"]["mean_ms"],
                "cv": q["statistics"]["coefficient_of_variation_percent"],
                "outliers": q["statistics"]["outliers"]["count"]
            })

        if cat_stats:
            avg_cv = sum(s["cv"] for s in cat_stats) / len(cat_stats)
            total_outliers = sum(s["outliers"] for s in cat_stats)
            enhanced["categories"][cat_name]["category_statistics"] = {
                "average_cv_percent": round(avg_cv, 2),
                "total_outliers": total_outliers,
                "measurement_stability": "excellent" if avg_cv < 10 else ("good" if avg_cv < 15 else ("moderate" if avg_cv < 20 else "high_variability"))
            }

    # Update summary with statistical overview
    all_cvs = []
    all_outliers = 0
    for cat_data in enhanced["categories"].values():
        for q in cat_data["queries"]:
            all_cvs.append(q["statistics"]["coefficient_of_variation_percent"])
            all_outliers += q["statistics"]["outliers"]["count"]

    enhanced["summary"]["statistical_overview"] = {
        "average_cv_percent": round(sum(all_cvs) / len(all_cvs), 2) if all_cvs else 0,
        "cv_range": {
            "min_percent": round(min(all_cvs), 2) if all_cvs else 0,
            "max_percent": round(max(all_cvs), 2) if all_cvs else 0
        },
        "total_outliers_detected": all_outliers,
        "overall_measurement_stability": "excellent" if (sum(all_cvs)/len(all_cvs) if all_cvs else 0) < 10 else "good"
    }

    return enhanced


def main():
    import os
    from pathlib import Path

    base_dir = Path(__file__).parent
    csv_path = os.path.join(base_dir, "raw_timings.csv")
    json_path = os.path.join(base_dir, "results.json")
    output_path = os.path.join(base_dir, "results_enhanced.json")

    print("Loading raw timing data...")
    query_timings = load_raw_timings(csv_path)
    print(f"  Loaded {len(query_timings)} queries")

    print("Loading existing results...")
    original_results = load_existing_results(json_path)

    print("Calculating enhanced statistics...")
    enhanced_results = enhance_results(original_results, query_timings)

    print(f"Writing enhanced results to {output_path}...")
    with open(output_path, 'w') as f:
        json.dump(enhanced_results, f, indent=2)

    print("\n" + "=" * 70)
    print("STATISTICAL ANALYSIS SUMMARY")
    print("=" * 70)

    for cat_name in ["safety", "operational", "planning"]:
        cat_data = enhanced_results["categories"].get(cat_name, {})
        cat_stats = cat_data.get("category_statistics", {})

        print(f"\n{cat_name.upper()} QUERIES (Requirement: {cat_data.get('requirement_ms', 'N/A')}ms)")
        print("-" * 70)

        for q in cat_data.get("queries", []):
            stats = q["statistics"]
            ci = stats["confidence_interval_95"]
            outliers = stats["outliers"]

            print(f"\n  {q['name']}")
            print(f"    Mean: {stats['mean_ms']:.3f}ms (95% CI: [{ci['lower_ms']:.3f}, {ci['upper_ms']:.3f}])")
            print(f"    Std Error: {stats['standard_error_ms']:.3f}ms | CV: {stats['coefficient_of_variation_percent']:.1f}%")
            print(f"    IQR: [{stats['interquartile_range']['q1_ms']:.3f}, {stats['interquartile_range']['q3_ms']:.3f}] = {stats['interquartile_range']['iqr_ms']:.3f}ms")
            print(f"    Outliers: {outliers['count']} (fences: {outliers['lower_fence_ms']:.1f} - {outliers['upper_fence_ms']:.1f}ms)")
            if outliers['count'] > 0:
                print(f"    Outlier values: {outliers['values_ms']}")

        print(f"\n  Category Summary:")
        print(f"    Average CV: {cat_stats.get('average_cv_percent', 0):.1f}%")
        print(f"    Measurement Stability: {cat_stats.get('measurement_stability', 'N/A')}")
        print(f"    Total Outliers: {cat_stats.get('total_outliers', 0)}")

    print("\n" + "=" * 70)
    overview = enhanced_results["summary"]["statistical_overview"]
    print(f"OVERALL MEASUREMENT QUALITY")
    print("-" * 70)
    print(f"  Average CV: {overview['average_cv_percent']:.1f}%")
    print(f"  CV Range: [{overview['cv_range']['min_percent']:.1f}%, {overview['cv_range']['max_percent']:.1f}%]")
    print(f"  Total Outliers: {overview['total_outliers_detected']}")
    print(f"  Stability Assessment: {overview['overall_measurement_stability']}")
    print("=" * 70)

    print(f"\nEnhanced results saved to: {output_path}")


if __name__ == "__main__":
    main()
