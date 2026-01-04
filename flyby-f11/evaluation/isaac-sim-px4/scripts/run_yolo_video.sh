#!/bin/bash
# Run YOLO video recording in Isaac Sim
# Usage: ./run_yolo_video.sh [--duration 60] [--headless]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "YOLO Video Recording"
echo "=============================================="
echo ""

# Default values
DURATION=60
HEADLESS=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --headless)
            HEADLESS="--headless"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--duration SECONDS] [--headless]"
            exit 1
            ;;
    esac
done

echo "Duration: ${DURATION}s"
echo "Mode: ${HEADLESS:-GUI}"
echo ""

# Create output directory
OUTPUT_DIR="$WORKSPACE_DIR/output/yolo_video"
mkdir -p "$OUTPUT_DIR"
echo "Output directory: $OUTPUT_DIR"
echo ""

# Run the recording script
echo "Starting recording..."
/isaac-sim/python.sh "$SCRIPT_DIR/record_yolo_video.py" --duration "$DURATION" $HEADLESS

echo ""
echo "=============================================="
echo "Recording complete!"
echo "Check $OUTPUT_DIR for videos"
echo "=============================================="
