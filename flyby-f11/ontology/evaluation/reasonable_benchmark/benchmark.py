#!/usr/bin/env python3
"""
Reasonable OWL 2 RL Reasoner Benchmark Suite
Measures query latency for UAV domain ontology reasoning

Reasonable is a Rust-based OWL 2 RL reasoner claimed to be 7x faster than
Allegro GraphDB for certain workloads.

This benchmark tests:
1. Ontology loading time
2. Materialization (forward chaining) time
3. Query response latency
4. Memory usage
"""

import json
import subprocess
import time
import statistics
import os
import sys
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
import platform
import resource

# Try to import reasonable Python bindings (if available)
try:
    import reasonable
    HAS_REASONABLE_BINDINGS = True
except ImportError:
    HAS_REASONABLE_BINDINGS = False

# Fallback to rdflib + owlrl for OWL 2 RL reasoning
try:
    from rdflib import Graph, Namespace, RDF, RDFS, OWL
    from rdflib.namespace import XSD
    import owlrl
    HAS_RDFLIB = True
except ImportError:
    HAS_RDFLIB = False

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


def get_memory_usage_mb() -> float:
    """Get current process memory usage in MB."""
    if HAS_PSUTIL:
        process = psutil.Process(os.getpid())
        return process.memory_info().rss / (1024 * 1024)
    else:
        # Fallback to resource module
        usage = resource.getrusage(resource.RUSAGE_SELF)
        return usage.ru_maxrss / 1024  # Convert KB to MB


def get_peak_memory_mb() -> float:
    """Get peak memory usage in MB."""
    if HAS_PSUTIL:
        process = psutil.Process(os.getpid())
        return process.memory_info().peak_wset / (1024 * 1024) if hasattr(
            process.memory_info(), 'peak_wset') else get_memory_usage_mb()
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024


class ReasonableRustBenchmark:
    """Benchmark using native Reasonable Rust binary."""

    def __init__(self, ontology_path: str):
        self.ontology_path = ontology_path
        self.binary_path = self._find_reasonable_binary()

    def _find_reasonable_binary(self) -> Optional[str]:
        """Find the reasonable binary in PATH or common locations."""
        # Check if in PATH
        result = subprocess.run(["which", "reasonable"], capture_output=True, text=True)
        if result.returncode == 0:
            return result.stdout.strip()

        # Check common locations
        common_paths = [
            "/usr/local/bin/reasonable",
            "/usr/bin/reasonable",
            str(Path.home() / ".cargo/bin/reasonable"),
        ]
        for path in common_paths:
            if Path(path).exists():
                return path

        return None

    def is_available(self) -> bool:
        """Check if Reasonable binary is available."""
        return self.binary_path is not None

    def run_reasoning(self, timeout: int = 60) -> Dict[str, Any]:
        """Run reasoning using Reasonable CLI."""
        if not self.is_available():
            return {"error": "Reasonable binary not found"}

        start_time = time.perf_counter()
        start_memory = get_memory_usage_mb()

        try:
            # Reasonable CLI usage (adjust based on actual CLI interface)
            result = subprocess.run(
                [self.binary_path, self.ontology_path],
                capture_output=True,
                text=True,
                timeout=timeout
            )

            end_time = time.perf_counter()
            elapsed_ms = (end_time - start_time) * 1000
            end_memory = get_memory_usage_mb()

            return {
                "elapsed_ms": elapsed_ms,
                "memory_delta_mb": end_memory - start_memory,
                "success": result.returncode == 0,
                "stdout": result.stdout[:1000] if result.stdout else "",
                "stderr": result.stderr[:1000] if result.stderr else "",
            }

        except subprocess.TimeoutExpired:
            return {
                "elapsed_ms": timeout * 1000,
                "error": "TIMEOUT",
                "success": False
            }
        except Exception as e:
            return {
                "elapsed_ms": 0,
                "error": str(e),
                "success": False
            }


class ReasonablePythonBenchmark:
    """Benchmark using Reasonable Python bindings."""

    def __init__(self, ontology_path: str):
        self.ontology_path = ontology_path
        self.reasoner = None

    def is_available(self) -> bool:
        """Check if Python bindings are available."""
        return HAS_REASONABLE_BINDINGS

    def load_ontology(self) -> Tuple[bool, float, str]:
        """Load ontology and return (success, time_ms, error)."""
        if not self.is_available():
            return False, 0, "Reasonable Python bindings not installed"

        start_time = time.perf_counter()
        try:
            self.reasoner = reasonable.PyReasoner()
            self.reasoner.load_file(self.ontology_path)
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return True, elapsed_ms, ""
        except Exception as e:
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return False, elapsed_ms, str(e)

    def run_materialization(self) -> Tuple[bool, float, int, str]:
        """Run materialization and return (success, time_ms, triple_count, error)."""
        if self.reasoner is None:
            return False, 0, 0, "Ontology not loaded"

        start_time = time.perf_counter()
        try:
            # Materialize inferred triples
            self.reasoner.reason()
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            triple_count = len(list(self.reasoner.triples()))
            return True, elapsed_ms, triple_count, ""
        except Exception as e:
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return False, elapsed_ms, 0, str(e)


class OWLRLFallbackBenchmark:
    """Fallback benchmark using rdflib + owlrl for OWL 2 RL reasoning."""

    def __init__(self, ontology_path: str):
        self.ontology_path = ontology_path
        self.graph = None

    def is_available(self) -> bool:
        """Check if rdflib + owlrl are available."""
        return HAS_RDFLIB

    def load_ontology(self) -> Tuple[bool, float, str]:
        """Load ontology and return (success, time_ms, error)."""
        if not self.is_available():
            return False, 0, "rdflib/owlrl not installed"

        start_time = time.perf_counter()
        try:
            self.graph = Graph()
            # Try different formats
            for fmt in ['xml', 'turtle', 'n3', 'nt']:
                try:
                    self.graph.parse(self.ontology_path, format=fmt)
                    break
                except Exception:
                    continue

            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return True, elapsed_ms, ""
        except Exception as e:
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return False, elapsed_ms, str(e)

    def run_materialization(self) -> Tuple[bool, float, int, str]:
        """Run OWL 2 RL materialization and return (success, time_ms, triple_count, error)."""
        if self.graph is None:
            return False, 0, 0, "Ontology not loaded"

        start_time = time.perf_counter()
        initial_count = len(self.graph)

        try:
            # Apply OWL 2 RL rules
            owlrl.DeductiveClosure(owlrl.OWLRL_Semantics).expand(self.graph)
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            final_count = len(self.graph)
            inferred = final_count - initial_count
            return True, elapsed_ms, final_count, ""
        except Exception as e:
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return False, elapsed_ms, 0, str(e)

    def query_sparql(self, query: str) -> Tuple[bool, float, int, str]:
        """Run SPARQL query and return (success, time_ms, result_count, error)."""
        if self.graph is None:
            return False, 0, 0, "Ontology not loaded"

        start_time = time.perf_counter()
        try:
            results = list(self.graph.query(query))
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return True, elapsed_ms, len(results), ""
        except Exception as e:
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return False, elapsed_ms, 0, str(e)


# Sample SPARQL queries for benchmarking UAV domain
BENCHMARK_QUERIES = {
    "safety": [
        {
            "name": "geofence_violations",
            "description": "Find all UAVs with geofence violations",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav WHERE {
                    ?uav a uav:UAV .
                    ?uav uav:hasGeofenceViolation true .
                }
            """
        },
        {
            "name": "nfz_violations",
            "description": "Find UAVs in no-fly zones",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav ?nfz WHERE {
                    ?uav a uav:UAV .
                    ?uav uav:isWithin ?nfz .
                    ?nfz a uav:NoFlyZone .
                }
            """
        },
        {
            "name": "low_battery",
            "description": "Find UAVs with low battery",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav WHERE {
                    ?uav a uav:UAV .
                    ?uav uav:batteryBelowReserve true .
                }
            """
        },
        {
            "name": "collision_risk",
            "description": "Find UAVs with imminent collision",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav ?obstacle WHERE {
                    ?uav a uav:UAV .
                    ?uav uav:collisionImminent ?obstacle .
                }
            """
        },
        {
            "name": "sensor_degraded",
            "description": "Find UAVs with degraded sensors",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav ?sensor WHERE {
                    ?uav a uav:UAV .
                    ?uav uav:hasSensor ?sensor .
                    ?sensor uav:sensorFailed true .
                    ?sensor a uav:CriticalSensor .
                }
            """
        },
        {
            "name": "safe_state_check",
            "description": "Find UAVs in safe state",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav WHERE {
                    ?uav a uav:UAV .
                    ?uav uav:inSafeState true .
                }
            """
        }
    ],
    "operational": [
        {
            "name": "hover_capable",
            "description": "Find UAVs that can hover",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav WHERE {
                    ?uav a uav:Multirotor .
                }
            """
        },
        {
            "name": "valid_localization",
            "description": "Find UAVs with valid localization",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav WHERE {
                    ?uav a uav:UAV .
                    ?uav uav:hasValidLocalization true .
                }
            """
        },
        {
            "name": "ndaa_compliant",
            "description": "Find NDAA-compliant UAVs",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav WHERE {
                    ?uav a uav:UAV .
                    ?uav uav:ndaaCompliant true .
                }
            """
        },
        {
            "name": "weather_ok",
            "description": "Find UAVs without weather violations",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav WHERE {
                    ?uav a uav:UAV .
                    FILTER NOT EXISTS { ?uav uav:weatherConstraintViolation true }
                }
            """
        },
        {
            "name": "terrain_traversable",
            "description": "Find traversable paths",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?path WHERE {
                    ?path a uav:FlightPath .
                    ?path uav:traversable true .
                }
            """
        }
    ],
    "planning": [
        {
            "name": "mission_feasibility",
            "description": "Check if mission is feasible",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?mission WHERE {
                    ?mission a uav:Mission .
                    ?mission uav:isValid true .
                }
            """
        },
        {
            "name": "regulatory_compliance",
            "description": "Check regulatory compliance",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?mission WHERE {
                    ?mission a uav:Mission .
                    ?mission uav:regulatoryCompliant true .
                }
            """
        },
        {
            "name": "capability_match",
            "description": "Match UAV capabilities to mission requirements",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?uav ?mission WHERE {
                    ?mission a uav:Mission .
                    ?mission uav:requiredCapability ?cap .
                    ?uav a uav:UAV .
                    ?uav uav:hasCapability ?cap .
                }
            """
        },
        {
            "name": "path_safety",
            "description": "Verify path safety",
            "sparql": """
                PREFIX uav: <http://example.org/uav#>
                SELECT ?path WHERE {
                    ?path a uav:FlightPath .
                    FILTER NOT EXISTS {
                        ?path uav:intersects ?nfz .
                        ?nfz a uav:NoFlyZone .
                    }
                }
            """
        }
    ]
}


def benchmark_query(benchmark: OWLRLFallbackBenchmark, query_info: Dict,
                    iterations: int = 100) -> Dict[str, Any]:
    """Benchmark a single SPARQL query."""
    query_name = query_info["name"]
    sparql = query_info["sparql"]

    print(f"  Benchmarking {query_name}... ", end="", flush=True)

    # Cold start
    cold_success, cold_ms, cold_count, cold_error = benchmark.query_sparql(sparql)

    if not cold_success:
        print(f"FAILED ({cold_error})")
        return {
            "name": query_name,
            "description": query_info["description"],
            "error": cold_error,
            "success_rate": 0
        }

    # Warm runs
    times = []
    successes = 0
    result_counts = []

    for _ in range(iterations):
        success, elapsed_ms, count, error = benchmark.query_sparql(sparql)
        if success:
            times.append(elapsed_ms)
            result_counts.append(count)
            successes += 1

    if times:
        times_sorted = sorted(times)
        p95_idx = int(len(times) * 0.95)
        p99_idx = int(len(times) * 0.99)

        stats = {
            "name": query_name,
            "description": query_info["description"],
            "iterations": iterations,
            "cold_start_ms": round(cold_ms, 3),
            "mean_ms": round(statistics.mean(times), 3),
            "median_ms": round(statistics.median(times), 3),
            "min_ms": round(min(times), 3),
            "max_ms": round(max(times), 3),
            "stdev_ms": round(statistics.stdev(times), 3) if len(times) > 1 else 0,
            "p95_ms": round(times_sorted[min(p95_idx, len(times)-1)], 3),
            "p99_ms": round(times_sorted[min(p99_idx, len(times)-1)], 3),
            "success_rate": round(successes / iterations * 100, 1),
            "avg_result_count": round(statistics.mean(result_counts), 1) if result_counts else 0
        }
        print(f"OK (mean: {stats['mean_ms']:.1f}ms, p95: {stats['p95_ms']:.1f}ms)")
    else:
        stats = {
            "name": query_name,
            "description": query_info["description"],
            "error": "All iterations failed",
            "success_rate": 0
        }
        print("FAILED (all iterations)")

    return stats


def run_benchmarks(ontology_path: str, iterations: int = 100) -> Dict[str, Any]:
    """Run all benchmarks."""

    results = {
        "reasoner": "Reasonable (OWL 2 RL)",
        "backend": "unknown",
        "version": "unknown",
        "platform": platform.machine(),
        "python_version": platform.python_version(),
        "timestamp": datetime.now().isoformat(),
        "iterations": iterations,
        "ontology_path": str(ontology_path),
        "ontology_stats": {},
        "materialization": {},
        "categories": {
            "safety": [],
            "operational": [],
            "planning": []
        },
        "summary": {}
    }

    # Determine which backend to use
    if HAS_REASONABLE_BINDINGS:
        print("Using: Reasonable Python bindings (native Rust)")
        results["backend"] = "reasonable-python"
        benchmark = ReasonablePythonBenchmark(ontology_path)
    elif HAS_RDFLIB:
        print("Using: rdflib + owlrl (Python fallback)")
        results["backend"] = "owlrl-python"
        benchmark = OWLRLFallbackBenchmark(ontology_path)
    else:
        print("ERROR: No OWL reasoning backend available")
        results["error"] = "No reasoning backend available"
        return results

    # Check ontology exists
    if not Path(ontology_path).exists():
        print(f"WARNING: Ontology not found at {ontology_path}")
        print("Running with placeholder results...")
        results["error"] = f"Ontology not found: {ontology_path}"
        return results

    # Load ontology
    print(f"\nLoading ontology: {ontology_path}")
    start_memory = get_memory_usage_mb()
    success, load_ms, error = benchmark.load_ontology()

    if not success:
        print(f"ERROR: Failed to load ontology: {error}")
        results["error"] = f"Failed to load ontology: {error}"
        return results

    results["ontology_stats"]["load_time_ms"] = round(load_ms, 3)
    print(f"  Load time: {load_ms:.1f}ms")

    # Run materialization (forward chaining)
    print("\nRunning OWL 2 RL materialization...")
    success, mat_ms, triple_count, error = benchmark.run_materialization()

    if success:
        results["materialization"] = {
            "time_ms": round(mat_ms, 3),
            "triple_count": triple_count,
            "success": True
        }
        end_memory = get_memory_usage_mb()
        results["materialization"]["memory_mb"] = round(end_memory - start_memory, 2)
        print(f"  Materialization time: {mat_ms:.1f}ms")
        print(f"  Total triples: {triple_count}")
        print(f"  Memory used: {end_memory - start_memory:.1f}MB")
    else:
        results["materialization"] = {
            "error": error,
            "success": False
        }
        print(f"  Materialization failed: {error}")

    # Run query benchmarks
    total_queries = 0
    total_passed = 0

    # Category requirements (same as Vampire benchmark)
    requirements = {
        "safety": 10,      # <10ms p95
        "operational": 100,  # <100ms p95
        "planning": 1000    # <1000ms p95
    }

    for category, queries in BENCHMARK_QUERIES.items():
        print(f"\n{'='*60}")
        print(f"Category: {category.upper()} (requirement: <{requirements[category]}ms)")
        print(f"{'='*60}")

        for query_info in queries:
            stats = benchmark_query(benchmark, query_info, iterations)
            stats["requirement_ms"] = requirements[category]

            if "p95_ms" in stats:
                stats["meets_requirement"] = stats["p95_ms"] < requirements[category]
                if stats["meets_requirement"]:
                    total_passed += 1
            else:
                stats["meets_requirement"] = False

            results["categories"][category].append(stats)
            total_queries += 1

    # Calculate summary
    all_p95 = {cat: [s["p95_ms"] for s in stats if "p95_ms" in s]
               for cat, stats in results["categories"].items()}

    results["summary"] = {
        "total_queries": total_queries,
        "queries_meeting_requirements": total_passed,
        "pass_rate": round(total_passed / total_queries * 100, 1) if total_queries > 0 else 0,
        "peak_memory_mb": round(get_peak_memory_mb(), 2),
    }

    for cat in ["safety", "operational", "planning"]:
        if all_p95[cat]:
            results["summary"][f"{cat}_avg_p95_ms"] = round(statistics.mean(all_p95[cat]), 3)
            results["summary"][f"{cat}_max_p95_ms"] = round(max(all_p95[cat]), 3)
        else:
            results["summary"][f"{cat}_avg_p95_ms"] = None
            results["summary"][f"{cat}_max_p95_ms"] = None

    return results


def main():
    script_dir = Path(__file__).parent

    # Default ontology path (will be created by owl_export phase)
    ontology_path = script_dir.parent / "owl_export" / "uav_domain.owl"
    results_file = script_dir / "results.json"

    # Allow override via command line
    if len(sys.argv) > 1:
        ontology_path = Path(sys.argv[1])

    iterations = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    print("="*60)
    print("Reasonable OWL 2 RL Reasoner Benchmark Suite")
    print("="*60)
    print(f"Ontology: {ontology_path}")
    print(f"Iterations per query: {iterations}")
    print(f"Results file: {results_file}")
    print()
    print("Backend availability:")
    print(f"  Reasonable Python bindings: {'YES' if HAS_REASONABLE_BINDINGS else 'NO'}")
    print(f"  rdflib + owlrl fallback:    {'YES' if HAS_RDFLIB else 'NO'}")
    print(f"  psutil (memory tracking):   {'YES' if HAS_PSUTIL else 'NO'}")

    results = run_benchmarks(str(ontology_path), iterations)

    # Save results
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)

    # Print summary
    print("\n" + "="*60)
    print("BENCHMARK SUMMARY")
    print("="*60)

    if "error" in results:
        print(f"ERROR: {results['error']}")
    else:
        summary = results["summary"]
        print(f"Backend: {results['backend']}")
        print(f"Total queries: {summary['total_queries']}")
        print(f"Meeting requirements: {summary['queries_meeting_requirements']} ({summary['pass_rate']}%)")
        print()
        print(f"Safety queries (req <10ms):")
        print(f"  Avg p95: {summary.get('safety_avg_p95_ms')}ms, Max p95: {summary.get('safety_max_p95_ms')}ms")
        print(f"Operational queries (req <100ms):")
        print(f"  Avg p95: {summary.get('operational_avg_p95_ms')}ms, Max p95: {summary.get('operational_max_p95_ms')}ms")
        print(f"Planning queries (req <1000ms):")
        print(f"  Avg p95: {summary.get('planning_avg_p95_ms')}ms, Max p95: {summary.get('planning_max_p95_ms')}ms")
        print()
        print(f"Peak memory: {summary.get('peak_memory_mb')}MB")

    print()
    print(f"Results saved to: {results_file}")

    # Return exit code based on pass rate
    if "error" in results or results.get("summary", {}).get("pass_rate", 0) < 50:
        sys.exit(1)


if __name__ == "__main__":
    main()
