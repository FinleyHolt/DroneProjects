# Custom Gazebo Model Setup for test-drone

This document explains how PX4's Gazebo integration works and how to spawn custom drone models in simulation.

## Problem Summary

When running `make px4_sitl gz_x500`, PX4 was spawning the default `x500` model instead of our custom `test_drone_x500` model (which includes T265 and D455 cameras).

Even setting `PX4_SIM_MODEL=gz_test_drone_x500` didn't work because the build system wasn't creating that target.

## Root Cause

PX4's Gazebo integration requires TWO things for a custom model to work:

1. **Model directory**: `Tools/simulation/gz/models/<model_name>/model.sdf`
2. **Airframe file**: `ROMFS/px4fmu_common/init.d-posix/airframes/<ID>_gz_<model_name>`

The CMake system (in `src/modules/simulation/gz_bridge/CMakeLists.txt`) automatically:
- Scans for all models in `Tools/simulation/gz/models/`
- Looks for matching airframe files with pattern `*_gz_<model_name>`
- Creates make targets like `gz_<model_name>` ONLY if both exist
- Each target sets `PX4_SIM_MODEL=gz_<model_name>` when launched

**We had the model but NOT the airframe file**, so the target was never created.

## How PX4 Model Spawning Works

### Step 1: Make Target (Optional but Convenient)

When you run `make px4_sitl gz_x500`, it:
1. Builds `px4_sitl_default` configuration
2. Runs the `gz_x500` custom CMake target
3. The target sets environment variable `PX4_SIM_MODEL=gz_x500`
4. Launches PX4 SITL with that environment

### Step 2: PX4 Init Scripts

PX4's startup script (`ROMFS/px4fmu_common/init.d-posix/rcS`) reads `PX4_SIM_MODEL`:
1. Strips the `gz_` prefix: `gz_x500` → `x500`
2. Searches for matching airframe file: looks for `*_x500` in airframes directory
3. Sources the airframe file (e.g., `4001_gz_x500`)
4. The airframe file can override `PX4_SIM_MODEL` with its own default

### Step 3: Gazebo Model Spawning

PX4's `gz_bridge` module (in `px4-rc.simulator`) uses `PX4_SIM_MODEL`:
1. Strips `gz_` prefix again: `gz_test_drone_x500` → `test_drone_x500`
2. Looks for model in `GZ_SIM_RESOURCE_PATH` (includes `Tools/simulation/gz/models/`)
3. Spawns `test_drone_x500/model.sdf`

## Solution: Custom Airframe File

We created `/home/finley/Github/LLMDrone/test-drone/config/px4/4007_gz_test_drone_x500`:

```bash
#!/bin/sh
#
# @name Test Drone X500 (Gazebo)
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=test_drone_x500}  # KEY LINE: Sets model name

# ... motor configuration, sensor settings, etc.
```

The key is `PX4_SIM_MODEL=${PX4_SIM_MODEL:=test_drone_x500}` which tells PX4 to spawn our custom model.

## Implementation Details

### 1. Airframe File Location

**Source**: `/home/finley/Github/LLMDrone/test-drone/config/px4/4007_gz_test_drone_x500`

**Must be copied to**: `/opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_test_drone_x500`

### 2. Automated Installation

Our build scripts now automatically install custom airframes:

**`scripts/build_px4_sitl.sh`** (runs during initial build):
- Copies all `[0-9][0-9][0-9][0-9]_gz_*` files from `config/px4/` to PX4's airframes directory
- Makes them executable
- Triggers full rebuild so CMake creates the new targets

**`scripts/start_simulation.sh`** (runs each time):
- Checks if airframe files need updating
- Copies if missing or source is newer
- No rebuild needed (just runtime installation)

### 3. Model Directory

**Source**: `/home/finley/Github/LLMDrone/test-drone/simulation/models/test_drone_x500/`

**Symlinked to**: `/opt/PX4-Autopilot/Tools/simulation/gz/models/test_drone_x500`

The build script creates this symlink automatically.

## Usage After Fix

### Rebuild Required (One Time)

After adding the airframe file, you MUST rebuild PX4 so CMake creates the new target:

```bash
# Inside container
cd /opt/PX4-Autopilot
make clean
make px4_sitl_default
```

This is automated in `build_px4_sitl.sh`.

### Launch Commands

Now you can use the make target:

```bash
# Default world
make px4_sitl gz_test_drone_x500

# Custom world
PX4_GZ_WORLD=test_drone make px4_sitl gz_test_drone_x500
```

Or use environment variables directly:

```bash
# Equivalent to make target
PX4_SIM_MODEL=gz_test_drone_x500 make px4_sitl

# With custom world
PX4_SIM_MODEL=gz_test_drone_x500 PX4_GZ_WORLD=test_drone make px4_sitl
```

### What Changed

**Before (BROKEN)**:
```bash
# This command existed: make px4_sitl gz_x500
# But this didn't: make px4_sitl gz_test_drone_x500 (target never created by CMake)
# So we tried: PX4_SIM_MODEL=gz_test_drone_x500 make px4_sitl gz_x500
# Result: gz_x500 target overrode PX4_SIM_MODEL, spawned default x500
```

**After (FIXED)**:
```bash
# CMake now sees airframe file 4007_gz_test_drone_x500
# Creates new target: gz_test_drone_x500
# Target properly sets PX4_SIM_MODEL=gz_test_drone_x500
# Spawns correct model: test_drone_x500 with T265 and D455
```

## Verification

After rebuilding, verify the target exists:

```bash
cd /opt/PX4-Autopilot/build/px4_sitl_default
ninja -t targets | grep test_drone
# Should show: gz_test_drone_x500
```

Check airframe is installed:

```bash
ls /opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/ | grep test_drone
# Should show: 4007_gz_test_drone_x500
```

Check model is linked:

```bash
ls -la /opt/PX4-Autopilot/Tools/simulation/gz/models/ | grep test_drone
# Should show: test_drone_x500 -> /workspace/simulation/models/test_drone_x500
```

## Creating Additional Custom Models

To add more custom models (e.g., `test_drone_x500_lidar`):

1. **Create model directory**: `simulation/models/test_drone_x500_lidar/model.sdf`
2. **Create airframe file**: `config/px4/4008_gz_test_drone_x500_lidar`
   - Set `PX4_SIM_MODEL=${PX4_SIM_MODEL:=test_drone_x500_lidar}`
3. **Rebuild PX4**: Run `build_px4_sitl.sh` or manually clean + rebuild
4. **Use new target**: `make px4_sitl gz_test_drone_x500_lidar`

The build scripts will automatically install the airframe file and symlink the model.

## Troubleshooting

### Model still spawning default x500

**Check 1**: Airframe file installed?
```bash
ls /opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_test_drone_x500
```

**Check 2**: PX4 rebuilt after adding airframe?
```bash
# Must rebuild for CMake to create new targets
cd /opt/PX4-Autopilot && make clean && make px4_sitl_default
```

**Check 3**: Target created by CMake?
```bash
cd /opt/PX4-Autopilot/build/px4_sitl_default
ninja -t targets | grep test_drone
```

### "Model not found" errors in Gazebo

**Check 1**: Model symlinked?
```bash
ls -la /opt/PX4-Autopilot/Tools/simulation/gz/models/test_drone_x500
```

**Check 2**: Model SDF valid?
```bash
# Test parsing
gz sdf -p /workspace/simulation/models/test_drone_x500/model.sdf
```

### "Barometer missing" warnings

This means wrong model is spawning. The default `x500_base` model doesn't include all sensors, but our `test_drone_x500` does. Follow troubleshooting steps above to ensure correct model is loaded.

## References

- **PX4 Gazebo Integration**: `src/modules/simulation/gz_bridge/CMakeLists.txt` (target creation logic)
- **Init Script**: `ROMFS/px4fmu_common/init.d-posix/rcS` (airframe selection)
- **Simulator Script**: `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator` (model spawning)
- **Default Airframe Example**: `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`
