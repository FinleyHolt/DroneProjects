#!/bin/bash
#------------------------------------------------------------------------------
# Reasonable OWL 2 RL Reasoner Benchmark Runner
#
# Runs all benchmark queries and collects timing statistics
# Compatible with both native Reasonable binary and Python fallback
#------------------------------------------------------------------------------

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ONTOLOGY_DIR="${SCRIPT_DIR}/../owl_export"
ONTOLOGY_FILE="${ONTOLOGY_DIR}/uav_domain.owl"
RESULTS_FILE="${SCRIPT_DIR}/results.json"
ITERATIONS="${1:-100}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}Reasonable OWL 2 RL Reasoner Benchmark Suite${NC}"
echo -e "${BLUE}============================================================${NC}"
echo "Iterations per query: ${ITERATIONS}"
echo "Ontology file: ${ONTOLOGY_FILE}"
echo "Results file: ${RESULTS_FILE}"
echo ""

#------------------------------------------------------------------------------
# Check for required tools
#------------------------------------------------------------------------------

echo "Checking environment..."

# Check for Reasonable binary
if command -v reasonable &> /dev/null; then
    REASONABLE_VERSION=$(reasonable --version 2>&1 || echo "unknown")
    echo -e "  ${GREEN}✓${NC} Reasonable binary found: ${REASONABLE_VERSION}"
    HAS_REASONABLE=1
else
    echo -e "  ${YELLOW}!${NC} Reasonable binary not found"
    HAS_REASONABLE=0
fi

# Check for Python
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version)
    echo -e "  ${GREEN}✓${NC} Python3 found: ${PYTHON_VERSION}"
else
    echo -e "  ${RED}✗${NC} Python3 not found"
    exit 1
fi

# Check for Python dependencies
echo ""
echo "Checking Python dependencies..."
python3 -c "import rdflib; print(f'  rdflib: {rdflib.__version__}')" 2>/dev/null || echo -e "  ${YELLOW}!${NC} rdflib not installed"
python3 -c "import owlrl; print('  owlrl: available')" 2>/dev/null || echo -e "  ${YELLOW}!${NC} owlrl not installed"
python3 -c "import psutil; print(f'  psutil: {psutil.__version__}')" 2>/dev/null || echo -e "  ${YELLOW}!${NC} psutil not installed"
python3 -c "import reasonable; print('  reasonable: available')" 2>/dev/null || echo -e "  ${YELLOW}!${NC} reasonable Python bindings not installed"

#------------------------------------------------------------------------------
# Check for ontology file
#------------------------------------------------------------------------------

echo ""
echo "Checking ontology..."
if [ -f "${ONTOLOGY_FILE}" ]; then
    ONTOLOGY_SIZE=$(du -h "${ONTOLOGY_FILE}" | cut -f1)
    echo -e "  ${GREEN}✓${NC} Ontology found: ${ONTOLOGY_FILE} (${ONTOLOGY_SIZE})"
else
    echo -e "  ${YELLOW}!${NC} Ontology not found at: ${ONTOLOGY_FILE}"
    echo "     (Will be created by owl_export phase)"
    echo "     Running with placeholder/test mode..."
fi

#------------------------------------------------------------------------------
# Run benchmark
#------------------------------------------------------------------------------

echo ""
echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}Running Benchmarks${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""

# Change to script directory for relative imports
cd "${SCRIPT_DIR}"

# Run Python benchmark script
python3 benchmark.py "${ONTOLOGY_FILE}" "${ITERATIONS}"

BENCHMARK_EXIT_CODE=$?

#------------------------------------------------------------------------------
# Summary
#------------------------------------------------------------------------------

echo ""
echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}Benchmark Complete${NC}"
echo -e "${BLUE}============================================================${NC}"

if [ -f "${RESULTS_FILE}" ]; then
    echo -e "Results saved to: ${GREEN}${RESULTS_FILE}${NC}"

    # Extract key metrics from JSON
    if command -v jq &> /dev/null; then
        echo ""
        echo "Quick Summary:"
        jq -r '.summary | "  Pass Rate: \(.pass_rate)%\n  Safety Avg p95: \(.safety_avg_p95_ms)ms\n  Operational Avg p95: \(.operational_avg_p95_ms)ms\n  Planning Avg p95: \(.planning_avg_p95_ms)ms"' "${RESULTS_FILE}" 2>/dev/null || true
    fi
else
    echo -e "${RED}Warning: Results file not created${NC}"
fi

exit ${BENCHMARK_EXIT_CODE}
