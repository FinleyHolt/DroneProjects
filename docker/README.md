# Docker Containers

This directory contains Dockerfiles for isolated hardware and software dependencies.

## Container Organization

Each container isolates specific hardware dependencies that may have conflicting requirements:

### `Dockerfile.d455` - Intel RealSense D455
- **Purpose**: Depth camera for perception pipeline
- **SDK**: librealsense2 (current version)
- **Hardware**: Intel RealSense D455 depth camera
- **Run**: `bash scripts/run_realsense_container.sh`

### `Dockerfile.t265` - Intel RealSense T265
- **Purpose**: Visual-inertial odometry (VIO) for localization
- **SDK**: librealsense2 v2.50.0 (last version supporting T265, built from source)
- **Hardware**: Intel RealSense T265 tracking camera (deprecated/EOL)
- **Run**: `bash scripts/run_t265_container.sh`
- **Note**: Requires librealsense 2.50.0, incompatible with D455's current SDK
- **Build time**: 5-10 minutes (compiles from source)

## Why Separate Containers?

The T265 tracking camera was deprecated by Intel and requires an older version of librealsense2 that is incompatible with the current SDK used by the D455. Running them in separate containers allows both cameras to function simultaneously without dependency conflicts.

## Building Containers

From repository root:
```bash
# D455 depth camera
sudo docker build -f docker/Dockerfile.d455 -t realsense-d455 .

# T265 tracking camera (builds from source, takes 5-10 min)
sudo docker build -f docker/Dockerfile.t265 -t realsense-t265 .
```

## Running Containers

Use the helper scripts in `scripts/`:
```bash
bash scripts/run_realsense_container.sh    # D455
bash scripts/run_t265_container.sh         # T265
```

## Inter-Container Communication

When both cameras need to communicate with the ROS 2 autonomy stack:
- Mount shared volumes for data exchange
- Use host network mode for ROS 2 DDS discovery
- Consider docker-compose for orchestration
