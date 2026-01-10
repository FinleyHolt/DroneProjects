# Podman Compose - Development Workflow

This directory contains the Podman Compose configuration for **development** on your laptop/desktop. This is the primary way to iterate on code, run tests, and validate changes before deploying to the Jetson.

## Quick Reference

```bash
# Start all services
podman-compose -f containers/compose/podman-compose.yml up

# Start in background
podman-compose -f containers/compose/podman-compose.yml up -d

# Rebuild and start
podman-compose -f containers/compose/podman-compose.yml up --build

# Stop all
podman-compose -f containers/compose/podman-compose.yml down

# Stop and remove volumes (full reset)
podman-compose -f containers/compose/podman-compose.yml down -v

# Enter ROS 2 container
podman exec -it flyby-f11-ros2-dev bash

# View logs
podman-compose -f containers/compose/podman-compose.yml logs -f
```

## Working Directory Shortcut

To avoid typing the full path every time, create an alias or use a symlink:

```bash
# Option 1: Create alias (add to ~/.bashrc)
alias flyby-compose='podman-compose -f /home/finley/Github/DroneProjects/flyby-f11/containers/compose/podman-compose.yml'

# Option 2: Run from project root with relative path
cd /home/finley/Github/DroneProjects/flyby-f11
podman-compose -f containers/compose/podman-compose.yml up
```

## Services

The compose file defines five services:

### 1. planning (profile: planning)
- **Purpose**: Mission planning and verification
- **Auto-start**: No (use `--profile planning` to start)
- **Use**: Generate and verify mission plans offline

```bash
# Start planning service
podman-compose -f containers/compose/podman-compose.yml --profile planning up planning
```

### 2. execution
- **Purpose**: Prolog runtime for constraint checking
- **Auto-start**: Yes
- **Use**: Always running in production, available for testing

### 3. ros2
- **Purpose**: ROS 2 development environment
- **Auto-start**: Yes
- **Use**: Primary development container - enter and run ROS 2 commands

```bash
# Enter container
podman exec -it flyby-f11-ros2-dev bash

# Inside container - build workspace
cd /workspace
colcon build --symlink-install
source install/setup.bash

# Run nodes
ros2 launch flyby_f11_bringup simulation.launch.py
```

### 4. vision
- **Purpose**: GPU vision pipeline development
- **Auto-start**: Yes
- **Use**: Test vision models, depth processing, object detection

### 5. sitl (profile: simulation)
- **Purpose**: ArduPilot SITL simulation
- **Auto-start**: No (use `--profile simulation` to start)
- **Use**: Test flight controller integration

```bash
# Start SITL
podman-compose -f containers/compose/podman-compose.yml --profile simulation up sitl
```

## Development Workflow

### Typical Iteration Cycle

1. **Start Services**
   ```bash
   cd /home/finley/Github/DroneProjects/flyby-f11
   podman-compose -f containers/compose/podman-compose.yml up -d
   ```

2. **Enter ROS 2 Container**
   ```bash
   podman exec -it flyby-f11-ros2-dev bash
   ```

3. **Build and Test** (inside container)
   ```bash
   cd /workspace
   colcon build --symlink-install --packages-select <package-name>
   source install/setup.bash
   ros2 run <package> <node>
   ```

4. **Edit Code** (on host, changes reflect in container via volume mount)
   - Edit files in `ros2_ws/src/`
   - Rebuild inside container

5. **Test Changes**
   ```bash
   # Inside container
   colcon test --packages-select <package-name>
   colcon test-result --verbose
   ```

6. **Rebuild Container** (if dependencies change)
   ```bash
   # On host
   podman-compose -f containers/compose/podman-compose.yml up --build ros2
   ```

### Running Specific Services

```bash
# Only execution + ROS 2 (no vision)
podman-compose -f containers/compose/podman-compose.yml up execution ros2

# Only SITL simulation
podman-compose -f containers/compose/podman-compose.yml --profile simulation up sitl

# Planning mode only
podman-compose -f containers/compose/podman-compose.yml --profile planning up planning
```

## GPU Access

The compose file is configured for NVIDIA GPU passthrough:

```yaml
devices:
  - nvidia.com/gpu=all
```

Verify GPU access:
```bash
podman exec -it flyby-f11-vision-dev nvidia-smi
```

If GPU not detected:
```bash
# Regenerate CDI config
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Restart containers
podman-compose -f containers/compose/podman-compose.yml down
podman-compose -f containers/compose/podman-compose.yml up
```

## Volume Mounts

All services mount their respective directories with `:z` suffix for SELinux compatibility:

- `./ontology:/workspace:z` (planning, execution)
- `./ros2_ws:/workspace:z` (ros2, vision)
- `./simulation:/simulation:z` (sitl)

**Note**: The `:z` suffix automatically relabels files for container access. This is required on Fedora/RHEL-based systems with SELinux.

## Network Configuration

The compose file uses:
- **flyby-net bridge network** for planning/execution communication
- **host networking** for ROS 2, vision, and SITL (required for ROS 2 DDS discovery)

## Troubleshooting

### Containers Won't Start

```bash
# Check logs
podman-compose -f containers/compose/podman-compose.yml logs <service-name>

# Verify images built
podman images | grep flyby-f11

# Rebuild from scratch
podman-compose -f containers/compose/podman-compose.yml build --no-cache
```

### ROS 2 Nodes Can't Discover Each Other

```bash
# Ensure ROS_DOMAIN_ID is consistent (set to 42 in compose file)
podman exec -it flyby-f11-ros2-dev bash -c 'echo $ROS_DOMAIN_ID'

# Ensure host networking is enabled (already configured)
```

### Permission Denied on Volumes

```bash
# Fix ownership
chown -R $USER:$USER ./ros2_ws ./ontology ./simulation

# If using SELinux, ensure :z suffix on volume mounts (already configured)
```

### GPU Not Available

```bash
# Check CDI config
ls /etc/cdi/nvidia.yaml

# Regenerate
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Test
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

## Differences from Production (Quadlet)

| Feature | Compose (Development) | Quadlet (Production) |
|---------|----------------------|----------------------|
| **Container names** | `*-dev` suffix | No suffix |
| **Auto-restart** | No | Yes |
| **Keep running** | `stdin_open: true, tty: true` | Not needed |
| **Resource limits** | None (use full host) | Limited (Memory, CPU) |
| **Logging** | Container logs | journald |

## Next Steps

Once development is complete:
1. Build production images (without `-dev` suffix)
2. Test production images locally using Quadlet
3. Transfer images to Jetson
4. Deploy using systemd services

See [../quadlet/README.md](../quadlet/README.md) for production deployment.
