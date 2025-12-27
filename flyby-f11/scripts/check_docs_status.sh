#!/bin/bash
# Flyby F-11 Documentation Status Checker
# Verifies documentation completeness across all domains

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DOCS_DIR="$PROJECT_ROOT/docs"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================"
echo "Flyby F-11 Documentation Status"
echo -e "========================================${NC}"
echo ""

# Check function
check_domain() {
    local domain="$1"
    local dir="$DOCS_DIR/$domain"
    local has_manifest=false
    local has_readme=false
    local file_count=0

    if [ ! -d "$dir" ]; then
        echo -e "${RED}✗ $domain - Directory not found${NC}"
        return 1
    fi

    # Check for manifest
    if [ -f "$dir/MANIFEST.md" ] || [ -f "$dir/manifest.txt" ]; then
        has_manifest=true
    fi

    # Check for README
    if [ -f "$dir/README.md" ]; then
        has_readme=true
    fi

    # Count documentation files
    file_count=$(find "$dir" -name "*.md" -o -name "*.txt" -o -name "*.pdf" 2>/dev/null | wc -l)

    # Status output
    if [ "$has_readme" = true ] && [ "$has_manifest" = true ]; then
        echo -e "${GREEN}✓ $domain${NC}"
        echo "  - README: present"
        echo "  - Manifest: present"
        echo "  - Files: $file_count"
    elif [ "$has_readme" = true ]; then
        echo -e "${YELLOW}○ $domain${NC}"
        echo "  - README: present"
        echo "  - Manifest: missing"
        echo "  - Files: $file_count"
    else
        echo -e "${RED}✗ $domain${NC}"
        echo "  - README: missing"
        echo "  - Files: $file_count"
    fi
    echo ""
}

# Check each documentation domain
echo -e "${BLUE}Documentation Domains:${NC}"
echo ""

check_domain "ontology"
check_domain "vision"
check_domain "flight_control"
check_domain "standards"
check_domain "tools"
check_domain "ros2"
check_domain "rl"

# Summary
echo -e "${BLUE}========================================"
echo "Summary"
echo -e "========================================${NC}"

total_md=$(find "$DOCS_DIR" -name "*.md" 2>/dev/null | wc -l)
total_pdf=$(find "$DOCS_DIR" -name "*.pdf" 2>/dev/null | wc -l)

echo "Total Markdown files: $total_md"
echo "Total PDF files: $total_pdf"
echo ""

# Check for key root-level documents
echo -e "${BLUE}Root Documents:${NC}"
for doc in "README.md" "DOCUMENTATION_INDEX.md" "DOCUMENTATION_SUMMARY.md" "DOWNLOAD_SUMMARY.md" "DEVELOPMENT_WORKFLOW.md"; do
    if [ -f "$DOCS_DIR/$doc" ]; then
        echo -e "  ${GREEN}✓${NC} $doc"
    else
        echo -e "  ${RED}✗${NC} $doc (missing)"
    fi
done
echo ""

echo -e "${GREEN}Documentation status check complete${NC}"
