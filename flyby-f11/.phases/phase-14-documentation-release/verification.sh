#!/bin/bash
# Phase 14: Documentation and Release - Verification Script
set -e

echo "========================================"
echo "Phase 14: Documentation and Release"
echo "Verification Script"
echo "========================================"
echo ""

PHASE_DIR="$(dirname "$0")"
PROJECT_ROOT="$(cd "$PHASE_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

PASS=0
FAIL=0

check() {
    local description="$1"
    local command="$2"

    echo -n "Checking: $description... "
    if eval "$command" > /dev/null 2>&1; then
        echo "PASS"
        ((PASS++))
    else
        echo "FAIL"
        ((FAIL++))
    fi
}

echo "=== Documentation Checks ==="
check "architecture docs exist" "test -d docs/architecture"
check "API docs exist" "test -d docs/api"
check "installation guide exists" "test -f docs/INSTALLATION.md"

echo ""
echo "=== Research Paper Checks ==="
check "paper directory exists" "test -d paper"
check "main.tex exists" "test -f paper/main.tex"
check "figures directory exists" "test -d paper/figures"

echo ""
echo "=== Repository Checks ==="
check "README.md updated" "test -f README.md && test -s README.md"
check "CONTRIBUTING.md exists" "test -f CONTRIBUTING.md"
check "LICENSE exists" "test -f LICENSE"

echo ""
echo "=== Tutorial Checks ==="
check "tutorials directory exists" "test -d tutorials"

echo ""
echo "=== MCTSSA Deliverables Checks ==="
check "MCTSSA deliverables exist" "test -d deliverables/mctssa"

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 14 verification PASSED"
echo ""
echo "🎉 PROJECT COMPLETE! 🎉"
exit 0
