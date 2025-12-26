#!/bin/bash
# ==========================================================================
# UAV Ontology Prolog Benchmark Runner
# ==========================================================================
#
# Runs the Prolog benchmark harness and outputs results to JSON.
#
# Usage:
#   ./run_benchmark.sh                    # Run benchmark, output to stdout
#   ./run_benchmark.sh --output FILE      # Run benchmark, save to file
#   ./run_benchmark.sh --test             # Run quick test of rules
#   ./run_benchmark.sh --help             # Show usage
#
# Container usage:
#   podman build -t uav-prolog-benchmark -f Containerfile .
#   podman run --rm uav-prolog-benchmark
#   podman run --rm -v $(pwd)/results:/results uav-prolog-benchmark \
#       ./run_benchmark.sh --output /results/benchmark.json
#
# Author: Finley Holt
# Date: 2024-12-25
# ==========================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_FILE=""
MODE="benchmark"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --output|-o)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --test|-t)
            MODE="test"
            shift
            ;;
        --help|-h)
            echo "UAV Ontology Prolog Benchmark Runner"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --output, -o FILE   Save JSON results to FILE"
            echo "  --test, -t          Run quick test of rules instead of benchmark"
            echo "  --help, -h          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                           Run benchmark, output to stdout"
            echo "  $0 --output results.json    Save results to file"
            echo "  $0 --test                    Test that rules load correctly"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

cd "$SCRIPT_DIR"

# Check for SWI-Prolog
if ! command -v swipl &> /dev/null; then
    echo "Error: swipl (SWI-Prolog) not found in PATH" >&2
    echo "Install SWI-Prolog or run this script in the container" >&2
    exit 1
fi

# Report Prolog version
echo "SWI-Prolog version: $(swipl --version)" >&2

if [[ "$MODE" == "test" ]]; then
    echo "Running rule tests..." >&2
    swipl -g test_rules -t halt benchmark.pl
elif [[ -n "$OUTPUT_FILE" ]]; then
    echo "Running benchmark, saving to $OUTPUT_FILE..." >&2
    swipl -g run_all_benchmarks -t halt benchmark.pl > "$OUTPUT_FILE"
    echo "Benchmark complete. Results saved to $OUTPUT_FILE" >&2
else
    echo "Running benchmark..." >&2
    swipl -g run_all_benchmarks -t halt benchmark.pl
fi
