# Intel RealSense Camera Docker Setup

## Overview

Multiple RealSense cameras require separate containers due to conflicting librealsense2 versions:
- **D455** (depth camera): Current librealsense2 SDK
- **T265** (tracking camera): Legacy librealsense2 SDK (deprecated hardware)

See `docker/README.md` for container organization details.

## Quick Start - D455

Build and run the D455 depth camera container:

```bash
bash scripts/run_realsense_container.sh
```

The container will automatically enumerate connected RealSense devices and drop you into a bash shell.

## Quick Start - T265

Build and run the T265 tracking camera container (builds from source, takes 5-10 min):

```bash
bash scripts/run_t265_container.sh
```

**Note**: T265 uses librealsense v2.50.0 (last version supporting T265 before deprecation). The build takes longer because it compiles from source.

## Manual Usage

### Build the D455 container:
```bash
docker build -f docker/Dockerfile.d455 -t realsense-d455 .
```

### Run with device access:
```bash
docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    --device-cgroup-rule='c 81:* rmw' \
    --device-cgroup-rule='c 189:* rmw' \
    realsense-d455
```

### Build the T265 container:
```bash
docker build -f docker/Dockerfile.t265 -t realsense-t265 .
```

### Run T265 with device access:
```bash
docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    --device-cgroup-rule='c 81:* rmw' \
    --device-cgroup-rule='c 189:* rmw' \
    realsense-t265
```

## Testing the Camera

Inside the container, first enable X11 on your host:
```bash
# On host machine (outside container)
xhost +local:docker
```

Then inside the container:

```bash
# List connected RealSense devices
rs-enumerate-devices

# Live capture with visualization (RECOMMENDED)
rs-capture

# View RGB stream info
rs-color

# View depth stream info
rs-depth

# Full GUI viewer (requires X11 forwarding)
realsense-viewer
```

**Recommended**: `rs-capture` provides the best quick visualization of camera streams.

## Device Permissions

The container needs:
- `--privileged` flag for full USB access
- `/dev` volume mount for device nodes
- Device cgroup rules for USB video (81) and USB devices (189)

## Next Steps

For ROS 2 integration with your autonomy stack, you'll need to extend this with:
- ROS 2 Humble installation
- `realsense-ros` package
- Proper network configuration for ROS 2 topics
