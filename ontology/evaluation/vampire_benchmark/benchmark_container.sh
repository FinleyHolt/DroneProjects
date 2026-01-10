#!/bin/bash
#------------------------------------------------------------------------------
# Vampire Benchmark Script (Container Version)
# Runs inside the flyby-f11-planning container
# Outputs timing data in a format parseable by the host Python script
#------------------------------------------------------------------------------

set -e

QUERIES_DIR="/workspace/benchmark_queries"
OUTPUT_FILE="/workspace/vampire_benchmark/raw_timings.csv"
ITERATIONS="${1:-100}"

# CSV header
echo "query,iteration,elapsed_ms,status,memory_mb" > "$OUTPUT_FILE"

# Function to get time in nanoseconds (portable)
get_time_ns() {
    date +%s%N 2>/dev/null || date +%s000000000
}

run_single_query() {
    local query_file="$1"
    local query_name="$2"
    local iteration="$3"

    # Get start time in nanoseconds
    local start_ns=$(get_time_ns)

    local output
    output=$(vampire --input_syntax tptp --time_limit 30 "$query_file" 2>&1) || true

    # Get end time and calculate elapsed
    local end_ns=$(get_time_ns)

    # Calculate elapsed in milliseconds using awk
    local elapsed_ms=$(awk -v start="$start_ns" -v end="$end_ns" 'BEGIN { printf "%.3f", (end - start) / 1000000 }')

    # Parse status
    local status="UNKNOWN"
    if echo "$output" | grep -q "SZS status Theorem"; then
        status="THEOREM"
    elif echo "$output" | grep -q "SZS status CounterSatisfiable"; then
        status="COUNTER_SATISFIABLE"
    elif echo "$output" | grep -q "SZS status Satisfiable"; then
        status="SATISFIABLE"
    elif echo "$output" | grep -q "SZS status Timeout"; then
        status="TIMEOUT"
    elif echo "$output" | grep -q "SZS status GaveUp"; then
        status="GAVE_UP"
    fi

    # Parse memory
    local memory_mb=""
    local memory_line=$(echo "$output" | grep -o "Peak memory usage: [0-9]* MB" | head -1)
    if [ -n "$memory_line" ]; then
        memory_mb=$(echo "$memory_line" | grep -o "[0-9]*")
    fi

    echo "${query_name},${iteration},${elapsed_ms},${status},${memory_mb}" >> "$OUTPUT_FILE"
}

echo "Vampire Benchmark - Container Script"
echo "====================================="
echo "Iterations: $ITERATIONS"
echo ""

# Get Vampire version
vampire --version | head -1
echo ""

# Process all queries
for category in safety operational planning; do
    echo "Processing ${category} queries..."
    for query_file in "${QUERIES_DIR}/${category}_"*.tptp; do
        if [ -f "$query_file" ]; then
            query_name=$(basename "$query_file" .tptp)
            echo -n "  ${query_name}: "

            for ((i=1; i<=ITERATIONS; i++)); do
                run_single_query "$query_file" "$query_name" "$i"
                # Progress indicator
                if [ $((i % 10)) -eq 0 ]; then
                    echo -n "."
                fi
            done
            echo " done"
        fi
    done
done

echo ""
echo "Raw timings saved to: $OUTPUT_FILE"
