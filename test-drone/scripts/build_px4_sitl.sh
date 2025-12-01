#!/usr/bin/env bash
set -euo pipefail

# Simple helper to build PX4 SITL inside the container
# Usage (inside container):
#   chmod +x /opt/build_px4_sitl.sh
#   /opt/build_px4_sitl.sh
#
# Optional: override jobs using PX4_BUILD_JOBS env var:
#   PX4_BUILD_JOBS=8 /opt/build_px4_sitl.sh

PX4_DIR=/opt/PX4-Autopilot

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4 directory not found at $PX4_DIR"
    exit 1
fi

cd "$PX4_DIR"

# Determine parallel jobs
JOBS="${PX4_BUILD_JOBS:-$(nproc)}"

echo "=== Building PX4 SITL (gz_x500) with ${JOBS} jobs in ${PX4_DIR} ==="
echo

# Build without DONT_RUN to ensure proper configuration
# Note: This may briefly launch and exit PX4, which is expected
make px4_sitl gz_x500 -j"${JOBS}" || true

# If that fails, try with DONT_RUN and then configure
if [ ! -f build/px4_sitl_default/bin/px4 ]; then
    echo "Initial build failed, trying with DONT_RUN..."
    DONT_RUN=1 VERBOSE=1 make px4_sitl gz_x500 -j"${JOBS}"
fi

# Symlink custom world files from /workspace/simulation/worlds to PX4's worlds directory
echo
echo "=== Symlinking custom worlds ==="
CUSTOM_WORLDS_DIR="/workspace/simulation/worlds"
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"

if [ -d "$CUSTOM_WORLDS_DIR" ]; then
    for world_file in "$CUSTOM_WORLDS_DIR"/*.sdf; do
        if [ -f "$world_file" ]; then
            world_name=$(basename "$world_file")
            target_link="$PX4_WORLDS_DIR/$world_name"

            # Remove existing symlink or file if it exists
            if [ -e "$target_link" ] || [ -L "$target_link" ]; then
                rm -f "$target_link"
            fi

            # Create symlink
            ln -s "$world_file" "$target_link"
            echo "  Linked: $world_name -> $world_file"
        fi
    done
else
    echo "  Warning: Custom worlds directory not found at $CUSTOM_WORLDS_DIR"
fi

# Symlink custom model files from /workspace/simulation/models to PX4's models directory
echo
echo "=== Symlinking custom models ==="
CUSTOM_MODELS_DIR="/workspace/simulation/models"
PX4_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"

if [ -d "$CUSTOM_MODELS_DIR" ]; then
    for model_dir in "$CUSTOM_MODELS_DIR"/*; do
        if [ -d "$model_dir" ]; then
            model_name=$(basename "$model_dir")
            target_link="$PX4_MODELS_DIR/$model_name"

            # Remove existing symlink or directory if it exists
            if [ -e "$target_link" ] || [ -L "$target_link" ]; then
                rm -rf "$target_link"
            fi

            # Create symlink
            ln -s "$model_dir" "$target_link"
            echo "  Linked: $model_name -> $model_dir"
        fi
    done
else
    echo "  Warning: Custom models directory not found at $CUSTOM_MODELS_DIR"
fi

# Copy custom airframe files from /workspace/config/px4 to PX4's airframes directory
echo
echo "=== Installing custom airframe files ==="
CUSTOM_AIRFRAMES_DIR="/workspace/config/px4"
PX4_AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"

if [ -d "$CUSTOM_AIRFRAMES_DIR" ]; then
    # Find all files matching airframe pattern (4-digit number followed by _gz_)
    for airframe_file in "$CUSTOM_AIRFRAMES_DIR"/[0-9][0-9][0-9][0-9]_gz_*; do
        if [ -f "$airframe_file" ]; then
            airframe_name=$(basename "$airframe_file")
            target_file="$PX4_AIRFRAMES_DIR/$airframe_name"

            # Copy airframe file (overwrite if exists)
            cp "$airframe_file" "$target_file"
            chmod +x "$target_file"
            echo "  Installed: $airframe_name"
        fi
    done
else
    echo "  Warning: Custom airframes directory not found at $CUSTOM_AIRFRAMES_DIR"
fi

echo
echo "=== PX4 SITL build complete ==="
echo "You can now run:"
echo "  cd /opt/PX4-Autopilot"
echo "  make px4_sitl gz_test_drone_x500  # Custom drone with T265+D455"
echo "  make px4_sitl gz_x500              # Default x500"
echo
echo "Available custom worlds:"
ls -1 "$PX4_WORLDS_DIR"/*.sdf 2>/dev/null | xargs -n1 basename | sed 's/.sdf$//' | sed 's/^/  - /'
