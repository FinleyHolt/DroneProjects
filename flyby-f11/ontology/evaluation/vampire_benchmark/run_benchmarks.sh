#!/bin/bash
#------------------------------------------------------------------------------
# Vampire Benchmark Runner
# Runs the benchmark script inside the flyby-f11-planning container
# then processes results on the host with Python
#
# Usage:
#   ./run_benchmarks.sh [iterations]
#
# Examples:
#   ./run_benchmarks.sh        # Run with default 100 iterations
#   ./run_benchmarks.sh 50     # Run with 50 iterations
#   ./run_benchmarks.sh 10     # Quick test with 10 iterations
#------------------------------------------------------------------------------

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
EVAL_DIR="$(dirname "$SCRIPT_DIR")"
ITERATIONS="${1:-100}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}Vampire Theorem Prover Benchmark Suite${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""
echo -e "Container: ${GREEN}flyby-f11-planning:latest${NC}"
echo -e "Iterations: ${GREEN}${ITERATIONS}${NC}"
echo ""

# Check if container image exists
if ! podman image exists flyby-f11-planning:latest 2>/dev/null; then
    echo -e "${RED}ERROR: Container image 'flyby-f11-planning:latest' not found${NC}"
    echo "Please build the container first."
    exit 1
fi

# Verify benchmark queries exist
if [ ! -d "${EVAL_DIR}/benchmark_queries" ]; then
    echo -e "${RED}ERROR: Benchmark queries directory not found${NC}"
    echo "Expected: ${EVAL_DIR}/benchmark_queries"
    exit 1
fi

echo "Mounting directories:"
echo "  - benchmark_queries -> /workspace/benchmark_queries"
echo "  - vampire_benchmark -> /workspace/vampire_benchmark"
echo ""
echo -e "${YELLOW}Starting benchmark (this may take several minutes)...${NC}"
echo ""

# Make container script executable
chmod +x "${SCRIPT_DIR}/benchmark_container.sh"

# Run the benchmark inside the container
podman run --rm \
    -v "${EVAL_DIR}/benchmark_queries:/workspace/benchmark_queries:z" \
    -v "${SCRIPT_DIR}:/workspace/vampire_benchmark:z" \
    -w /workspace/benchmark_queries \
    flyby-f11-planning:latest \
    bash /workspace/vampire_benchmark/benchmark_container.sh "${ITERATIONS}"

CONTAINER_EXIT=$?

if [ $CONTAINER_EXIT -ne 0 ]; then
    echo -e "${RED}ERROR: Container benchmark failed with exit code ${CONTAINER_EXIT}${NC}"
    exit $CONTAINER_EXIT
fi

echo ""
echo -e "${BLUE}Processing results...${NC}"
echo ""

# Process results with Python on the host
chmod +x "${SCRIPT_DIR}/process_results.py"
python3 "${SCRIPT_DIR}/process_results.py"

EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}============================================================${NC}"
    echo -e "${GREEN}Benchmark completed successfully${NC}"
    echo -e "${GREEN}============================================================${NC}"
else
    echo -e "${YELLOW}============================================================${NC}"
    echo -e "${YELLOW}Benchmark completed with warnings (exit code: ${EXIT_CODE})${NC}"
    echo -e "${YELLOW}============================================================${NC}"
fi

echo ""
echo -e "Results saved to: ${GREEN}${SCRIPT_DIR}/results.json${NC}"

exit $EXIT_CODE
