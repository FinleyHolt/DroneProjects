#!/bin/bash
# =============================================================================
# ELK Reasoner Benchmark Runner
# =============================================================================
#
# This script runs the ELK reasoner benchmark on the UAV domain ontology.
# Designed to run inside the elk-benchmark container.
#
# Usage:
#   ./run_benchmark.sh [ontology-path] [output-path]
#
# Defaults:
#   ontology-path: /workspace/owl_export/uav_domain.owl
#   output-path:   /workspace/elk_benchmark/results.json
#
# =============================================================================

set -e

# Default paths
ONTOLOGY_PATH="${1:-/workspace/owl_export/uav_domain.owl}"
OUTPUT_PATH="${2:-/workspace/elk_benchmark/results.json}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=============================================="
echo "ELK Reasoner Benchmark"
echo "=============================================="
echo ""

# Check if ontology exists
if [ ! -f "$ONTOLOGY_PATH" ]; then
    echo -e "${RED}Error: Ontology not found at $ONTOLOGY_PATH${NC}"
    echo ""
    echo "Expected directory structure:"
    echo "  /workspace/"
    echo "    owl_export/"
    echo "      uav_domain.owl"
    echo "    elk_benchmark/"
    echo "      results.json (output)"
    echo ""
    echo "Mount the evaluation directory to /workspace:"
    echo "  podman run --rm -v \$(pwd):/workspace:Z elk-benchmark"
    exit 1
fi

echo "Ontology: $ONTOLOGY_PATH"
echo "Output:   $OUTPUT_PATH"
echo ""

# Ensure output directory exists
mkdir -p "$(dirname "$OUTPUT_PATH")"

# Check Java version
echo -e "${YELLOW}Java Version:${NC}"
java -version 2>&1 | head -1
echo ""

# Run the benchmark
echo -e "${YELLOW}Running ELK Benchmark...${NC}"
echo ""

# Check if compiled class exists
if [ -f "/opt/elk/ElkBenchmark.class" ]; then
    # Run compiled version
    cd /opt/elk
    java -cp "${CLASSPATH}:/opt/elk" ElkBenchmark "$ONTOLOGY_PATH" "$OUTPUT_PATH"
else
    # Compile and run
    echo "Compiling ElkBenchmark.java..."
    cd /opt/elk/src

    # Compile
    if ! javac -cp "${CLASSPATH}" -d /opt/elk ElkBenchmark.java 2>/dev/null; then
        echo -e "${YELLOW}Note: Compilation may show warnings for missing ELK classes.${NC}"
        echo -e "${YELLOW}Falling back to simple OWLAPI-only benchmark...${NC}"

        # Create a simpler fallback benchmark
        cat > /tmp/SimpleBenchmark.java << 'JAVA_CODE'
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.*;
import java.io.*;
import java.util.*;
import java.time.Instant;

public class SimpleBenchmark {
    public static void main(String[] args) throws Exception {
        if (args.length < 2) {
            System.err.println("Usage: java SimpleBenchmark <ontology> <output>");
            System.exit(1);
        }

        String ontologyPath = args[0];
        String outputPath = args[1];

        // Load ontology
        long loadStart = System.nanoTime();
        OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
        OWLOntology ontology = manager.loadOntologyFromOntologyDocument(new File(ontologyPath));
        long loadTime = System.nanoTime() - loadStart;

        // Write results
        PrintWriter w = new PrintWriter(new FileWriter(outputPath));
        w.println("{");
        w.println("  \"reasoner\": \"OWLAPI (ELK not available)\",");
        w.println("  \"timestamp\": \"" + Instant.now().toString() + "\",");
        w.println("  \"ontology_file\": \"" + ontologyPath + "\",");
        w.println("  \"load_time_ms\": " + (loadTime / 1000000.0) + ",");
        w.println("  \"ontology_metrics\": {");
        w.println("    \"class_count\": " + ontology.getClassesInSignature().size() + ",");
        w.println("    \"object_property_count\": " + ontology.getObjectPropertiesInSignature().size() + ",");
        w.println("    \"data_property_count\": " + ontology.getDataPropertiesInSignature().size() + ",");
        w.println("    \"individual_count\": " + ontology.getIndividualsInSignature().size() + ",");
        w.println("    \"axiom_count\": " + ontology.getAxiomCount());
        w.println("  },");
        w.println("  \"note\": \"ELK reasoner not available in classpath. Install elk-owlapi JAR.\"");
        w.println("}");
        w.close();

        System.out.println("Basic ontology metrics written to: " + outputPath);
    }
}
JAVA_CODE

        javac -cp "${CLASSPATH}" -d /tmp /tmp/SimpleBenchmark.java
        java -cp "${CLASSPATH}:/tmp" SimpleBenchmark "$ONTOLOGY_PATH" "$OUTPUT_PATH"
        exit 0
    fi

    cd /opt/elk
    java -cp "${CLASSPATH}:/opt/elk" ElkBenchmark "$ONTOLOGY_PATH" "$OUTPUT_PATH"
fi

# Check if output was created
if [ -f "$OUTPUT_PATH" ]; then
    echo ""
    echo -e "${GREEN}Benchmark complete!${NC}"
    echo ""
    echo "Results summary:"
    echo "----------------"

    # Extract key metrics using jq if available
    if command -v jq &> /dev/null; then
        echo "Load time:           $(jq -r '.load_time_ms' "$OUTPUT_PATH") ms"
        echo "Classification time: $(jq -r '.classification_time_ms // "N/A"' "$OUTPUT_PATH") ms"
        echo "Is consistent:       $(jq -r '.is_consistent // "N/A"' "$OUTPUT_PATH")"
        echo ""
        echo "Ontology metrics:"
        echo "  Classes:           $(jq -r '.ontology_metrics.class_count' "$OUTPUT_PATH")"
        echo "  Object properties: $(jq -r '.ontology_metrics.object_property_count' "$OUTPUT_PATH")"
        echo "  Data properties:   $(jq -r '.ontology_metrics.data_property_count' "$OUTPUT_PATH")"
        echo "  Individuals:       $(jq -r '.ontology_metrics.individual_count' "$OUTPUT_PATH")"
        echo "  Axioms:            $(jq -r '.ontology_metrics.axiom_count' "$OUTPUT_PATH")"
    else
        echo "(Install jq for formatted output)"
        head -30 "$OUTPUT_PATH"
    fi

    echo ""
    echo "Full results: $OUTPUT_PATH"
else
    echo -e "${RED}Error: Benchmark failed to produce output${NC}"
    exit 1
fi
