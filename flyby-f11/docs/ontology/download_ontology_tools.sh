#!/bin/bash
# Download script for ontology and reasoning tools
# Phase 1 - Weeks 1-6 Priority Tools
# Run this script to populate /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/

set -e  # Exit on error

ONTOLOGY_DIR="/home/finley/Github/DroneProjects/flyby-f11/docs/ontology"
cd "$ONTOLOGY_DIR"

echo "========================================="
echo "Downloading Ontology & Reasoning Tools"
echo "========================================="
echo ""

# Create manifest file
MANIFEST="$ONTOLOGY_DIR/manifest.txt"
echo "Ontology and Reasoning Tools Download Manifest" > "$MANIFEST"
echo "Generated: $(date)" >> "$MANIFEST"
echo "=========================================" >> "$MANIFEST"
echo "" >> "$MANIFEST"

# Function to log downloads
log_download() {
    echo "$1" | tee -a "$MANIFEST"
}

# 1. SUMO Ontology
echo "[1/5] Cloning SUMO Ontology..."
if [ -d "sumo" ]; then
    echo "  SUMO already exists, skipping..."
    log_download "[SKIP] SUMO Ontology - already exists at sumo/"
else
    git clone --depth 1 https://github.com/ontologyportal/sumo.git sumo
    log_download "[OK] SUMO Ontology - cloned to sumo/"
    log_download "     Key files: Merge.kif, Mid-level-ontology.kif, Spatial.kif"
    log_download "     Source: https://github.com/ontologyportal/sumo"
fi
echo ""

# 2. Vampire Theorem Prover
echo "[2/5] Cloning Vampire Theorem Prover..."
if [ -d "vampire" ]; then
    echo "  Vampire already exists, skipping..."
    log_download "[SKIP] Vampire Theorem Prover - already exists at vampire/"
else
    git clone --depth 1 https://github.com/vprover/vampire.git vampire
    log_download "[OK] Vampire Theorem Prover - cloned to vampire/"
    log_download "     Documentation: vampire/README.md"
    log_download "     Source: https://github.com/vprover/vampire"
fi
echo ""

# 3. Clingo (Potassco)
echo "[3/5] Cloning Clingo (Potassco ASP Solver)..."
if [ -d "clingo" ]; then
    echo "  Clingo already exists, skipping..."
    log_download "[SKIP] Clingo (Potassco) - already exists at clingo/"
else
    git clone --depth 1 https://github.com/potassco/clingo.git clingo
    log_download "[OK] Clingo (Potassco) - cloned to clingo/"
    log_download "     Documentation: clingo/README.md"
    log_download "     Source: https://github.com/potassco/clingo"
fi
echo ""

# 4. E-Prover
echo "[4/5] Cloning E-Prover..."
if [ -d "eprover" ]; then
    echo "  E-Prover already exists, skipping..."
    log_download "[SKIP] E-Prover - already exists at eprover/"
else
    git clone --depth 1 https://github.com/eprover/eprover.git eprover
    log_download "[OK] E-Prover - cloned to eprover/"
    log_download "     Documentation: eprover/README.md, eprover/DOC/"
    log_download "     Source: https://github.com/eprover/eprover"
fi
echo ""

# 5. SWI-Prolog Documentation
echo "[5/5] Downloading SWI-Prolog Documentation..."
mkdir -p swi-prolog-docs

# Download SWI-Prolog manual HTML (main sections)
log_download ""
log_download "[INFO] SWI-Prolog Documentation:"

# Manual overview
if [ ! -f "swi-prolog-docs/manual.html" ]; then
    wget -q -O swi-prolog-docs/manual.html "https://www.swi-prolog.org/pldoc/doc_for?object=manual" && \
        log_download "[OK] SWI-Prolog Manual Overview - swi-prolog-docs/manual.html" || \
        log_download "[FAIL] Could not download SWI-Prolog manual overview"
fi

# Foreign Language Interface (C/FFI)
if [ ! -f "swi-prolog-docs/foreign.html" ]; then
    wget -q -O swi-prolog-docs/foreign.html "https://www.swi-prolog.org/pldoc/man?section=foreign" && \
        log_download "[OK] SWI-Prolog FFI Guide - swi-prolog-docs/foreign.html" || \
        log_download "[FAIL] Could not download SWI-Prolog FFI guide"
fi

# Embedding SWI-Prolog
if [ ! -f "swi-prolog-docs/embedded.html" ]; then
    wget -q -O swi-prolog-docs/embedded.html "https://www.swi-prolog.org/pldoc/man?section=embedded" && \
        log_download "[OK] SWI-Prolog Embedding Guide - swi-prolog-docs/embedded.html" || \
        log_download "[FAIL] Could not download SWI-Prolog embedding guide"
fi

# C API Reference
if [ ! -f "swi-prolog-docs/c-api.html" ]; then
    wget -q -O swi-prolog-docs/c-api.html "https://www.swi-prolog.org/pldoc/man?section=foreigninclude" && \
        log_download "[OK] SWI-Prolog C API Reference - swi-prolog-docs/c-api.html" || \
        log_download "[FAIL] Could not download SWI-Prolog C API reference"
fi

log_download "     Source: https://www.swi-prolog.org/pldoc/doc_for?object=manual"

echo ""
echo "========================================="
echo "Download Complete!"
echo "========================================="
echo ""
echo "Summary written to: $MANIFEST"
echo ""
echo "Directory structure:"
tree -L 2 "$ONTOLOGY_DIR" || ls -lR "$ONTOLOGY_DIR"
