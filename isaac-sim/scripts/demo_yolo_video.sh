#!/bin/bash
# =============================================================================
# YOLO Computer Vision Demo
# =============================================================================
#
# Demonstrates the full YOLO perception pipeline:
# 1. Launch Isaac Sim with procedural world and PX4 drone
# 2. Record ISR camera footage during autonomous flight
# 3. Process video with YOLOv11 + ByteTrack detection overlays
#
# Everything runs inside the Isaac Sim container (no external dependencies).
#
# Output:
#   - Raw video from drone camera
#   - Annotated video with YOLO bounding boxes and track IDs
#   - Detection statistics
#
# Usage:
#   ./scripts/demo_yolo_video.sh              # Full demo with GUI
#   ./scripts/demo_yolo_video.sh --headless   # Headless mode (faster)
#   ./scripts/demo_yolo_video.sh --skip-sim   # Skip simulation, process existing frames
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

# Container image
IMAGE_NAME="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

# Parse arguments
HEADLESS=""
SKIP_SIM=false

for arg in "$@"; do
    case $arg in
        --headless)
            HEADLESS="--headless"
            ;;
        --skip-sim)
            SKIP_SIM=true
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --headless     Run simulation without GUI (faster)"
            echo "  --skip-sim     Skip simulation, process existing frames"
            echo "  --help         Show this help message"
            exit 0
            ;;
    esac
done

echo "============================================================================="
echo "YOLO COMPUTER VISION DEMO"
echo "============================================================================="
echo "Project: Flyby F-11 ISR Drone"
echo "Container: $IMAGE_NAME"
echo "Date: $(date)"
echo "============================================================================="
echo ""

# Create output directory
mkdir -p "$PROJECT_DIR/output/yolo_demo"

# =============================================================================
# STEP 1: Run Isaac Sim to capture drone camera footage
# =============================================================================
if [ "$SKIP_SIM" = false ]; then
    echo "[Step 1/3] Running Isaac Sim to capture drone camera footage..."
    echo "           Mode: ${HEADLESS:-GUI}"
    echo ""

    # Run phase 3d which records ISR video with bounding boxes
    ./scripts/run_phase_test.sh 3d $HEADLESS

    echo ""
    echo "[Step 1/3] Simulation complete, frames captured"
else
    echo "[Step 1/3] Skipping simulation (--skip-sim)"
fi

# Check for captured frames
FRAMES_DIR="$PROJECT_DIR/output/phase3c_video/frames"
if [ ! -d "$FRAMES_DIR" ] || [ -z "$(ls -A $FRAMES_DIR 2>/dev/null)" ]; then
    echo ""
    echo "ERROR: No frames found in $FRAMES_DIR"
    echo "       Run without --skip-sim to capture frames first"
    exit 1
fi

FRAME_COUNT=$(ls "$FRAMES_DIR"/*.png 2>/dev/null | wc -l)
echo "           Found $FRAME_COUNT frames"

echo ""
echo "============================================================================="

# =============================================================================
# STEP 2: Create raw video from frames
# =============================================================================
echo "[Step 2/3] Creating raw video from $FRAME_COUNT frames..."

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RAW_VIDEO="$PROJECT_DIR/output/yolo_demo/isr_flight_raw_${TIMESTAMP}.mp4"

ffmpeg -y -framerate 10 \
    -i "$FRAMES_DIR/frame_%05d.png" \
    -c:v libx264 -pix_fmt yuv420p \
    -preset fast \
    "$RAW_VIDEO" 2>/dev/null

if [ -f "$RAW_VIDEO" ]; then
    RAW_SIZE=$(du -h "$RAW_VIDEO" | cut -f1)
    echo "           Created: $RAW_VIDEO ($RAW_SIZE)"
else
    echo "ERROR: Failed to create raw video"
    exit 1
fi

echo ""
echo "============================================================================="

# =============================================================================
# STEP 3: Process video with YOLO detection overlays (inside container)
# =============================================================================
echo "[Step 3/3] Processing video with YOLOv11 + ByteTrack (in container)..."

YOLO_VIDEO="$PROJECT_DIR/output/yolo_demo/isr_flight_yolo_${TIMESTAMP}.mp4"

# Setup X11 for container
xhost +local: 2>/dev/null || echo "[WARN] xhost command failed"

# Find Xauthority file
XAUTH_FILE=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
if [ -z "$XAUTH_FILE" ]; then
    XAUTH_FILE="$HOME/.Xauthority"
fi

# Run YOLO processing inside container
podman run --rm \
    --device nvidia.com/gpu=all \
    --network host \
    --ipc host \
    --security-opt label=disable \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -v "$PROJECT_DIR:/workspace:Z" \
    --user root \
    "$IMAGE_NAME" \
    /isaac-sim/python.sh /workspace/scripts/process_video_with_yolo.py \
        /workspace/output/yolo_demo/isr_flight_raw_${TIMESTAMP}.mp4 \
        /workspace/output/yolo_demo/isr_flight_yolo_${TIMESTAMP}.mp4 \
        --conf 0.25 \
        --track

if [ -f "$YOLO_VIDEO" ]; then
    YOLO_SIZE=$(du -h "$YOLO_VIDEO" | cut -f1)
    echo ""
    echo "           Created: $YOLO_VIDEO ($YOLO_SIZE)"
else
    echo "WARNING: YOLO video not created"
fi

echo ""
echo "============================================================================="
echo "DEMO COMPLETE"
echo "============================================================================="
echo ""
echo "Output files:"
echo "  Raw video:      $RAW_VIDEO"
if [ -f "$YOLO_VIDEO" ]; then
    echo "  YOLO annotated: $YOLO_VIDEO"
fi
echo "  Frames:         $FRAMES_DIR/"
echo ""
echo "To view the annotated video:"
echo "  mpv $YOLO_VIDEO"
echo "  # or"
echo "  vlc $YOLO_VIDEO"
echo ""
echo "============================================================================="
