#!/bin/bash

# Verification Script for Vision Documentation Setup
# Checks that all documentation files and directories are in place
# Author: Finley Holt
# Date: 2025-12-25

set -e

DOCS_DIR="/home/finley/Github/DroneProjects/flyby-f11/docs/vision"

echo "=================================="
echo "Vision Documentation Setup Verification"
echo "=================================="
echo ""

# Check if we're in the right directory
if [ ! -d "$DOCS_DIR" ]; then
    echo "ERROR: Vision docs directory not found at $DOCS_DIR"
    exit 1
fi

cd "$DOCS_DIR"

# Check required files
echo "[1/3] Checking documentation files..."

REQUIRED_FILES=(
    "INDEX.md"
    "GETTING_STARTED.md"
    "README.md"
    "MODEL_CONFIGURATIONS.md"
    "PHASE2_CHECKLIST.md"
    "download_vision_docs.sh"
)

MISSING_FILES=0

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file (MISSING)"
        MISSING_FILES=$((MISSING_FILES + 1))
    fi
done

echo ""

# Check required directories
echo "[2/3] Checking directory structure..."

REQUIRED_DIRS=(
    "ultralytics"
    "tensorrt"
    "realsense"
    "jetson-inference"
    "vlm"
    "repos"
)

MISSING_DIRS=0

for dir in "${REQUIRED_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        echo "  ✓ $dir/"
    else
        echo "  ✗ $dir/ (MISSING)"
        MISSING_DIRS=$((MISSING_DIRS + 1))
    fi
done

echo ""

# Check download script permissions
echo "[3/3] Checking download script permissions..."

if [ -x "download_vision_docs.sh" ]; then
    echo "  ✓ download_vision_docs.sh is executable"
else
    echo "  ✗ download_vision_docs.sh is NOT executable"
    echo "    Run: chmod +x download_vision_docs.sh"
fi

echo ""

# Summary
echo "=================================="
echo "Verification Summary"
echo "=================================="

if [ $MISSING_FILES -eq 0 ] && [ $MISSING_DIRS -eq 0 ]; then
    echo "✓ All documentation files present"
    echo "✓ All directories created"
    echo ""
    echo "Status: READY FOR PHASE 2"
    echo ""
    echo "Next steps:"
    echo "  1. Read GETTING_STARTED.md"
    echo "  2. Run ./download_vision_docs.sh"
    echo "  3. Follow PHASE2_CHECKLIST.md"
    echo ""
else
    echo "✗ Missing $MISSING_FILES file(s) and $MISSING_DIRS director(ies)"
    echo ""
    echo "Status: INCOMPLETE SETUP"
    echo ""
    echo "Please re-run setup or create missing items manually."
    echo ""
fi

# Show file sizes
echo "=================================="
echo "Documentation Statistics"
echo "=================================="
echo ""

echo "File sizes:"
ls -lh *.md *.sh 2>/dev/null | awk '{print "  " $5 "\t" $9}'

echo ""

# Count total lines
TOTAL_LINES=0
for file in *.md *.sh; do
    if [ -f "$file" ]; then
        LINES=$(wc -l < "$file")
        TOTAL_LINES=$((TOTAL_LINES + LINES))
        echo "  $LINES lines\t$file"
    fi
done

echo ""
echo "Total documentation: $TOTAL_LINES lines"

echo ""

# Check if repos have been downloaded
echo "=================================="
echo "Repository Status"
echo "=================================="
echo ""

REPOS_DOWNLOADED=0
TOTAL_REPOS=4

cd repos/

EXPECTED_REPOS=(
    "ultralytics"
    "realsense-ros"
    "realsense_t265_shelfino"
    "jetson-inference"
)

for repo in "${EXPECTED_REPOS[@]}"; do
    if [ -d "$repo" ]; then
        echo "  ✓ $repo (downloaded)"
        REPOS_DOWNLOADED=$((REPOS_DOWNLOADED + 1))
    else
        echo "  ✗ $repo (not downloaded yet)"
    fi
done

cd ..

echo ""

if [ $REPOS_DOWNLOADED -eq 0 ]; then
    echo "Status: Repositories NOT downloaded yet"
    echo "Run: ./download_vision_docs.sh"
elif [ $REPOS_DOWNLOADED -eq $TOTAL_REPOS ]; then
    echo "Status: All repositories downloaded ✓"
    echo "Check manifest.txt for details"
else
    echo "Status: Partial download ($REPOS_DOWNLOADED/$TOTAL_REPOS repositories)"
    echo "Re-run: ./download_vision_docs.sh"
fi

echo ""
echo "=================================="
echo "Verification Complete"
echo "=================================="
