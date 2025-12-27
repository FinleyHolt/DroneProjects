# Flyby F-11 Quick Start Guide

## TL;DR

**Development** (your laptop/desktop):
```bash
podman-compose -f containers/compose/podman-compose.yml up --build    # Start everything
podman exec -it flyby-f11-ros2-dev bash  # Enter container
```

**Production** (Jetson on drone):
```bash
systemctl --user start flyby-f11-ros2.service   # Start autonomy
systemctl --user status flyby-f11-ros2.service  # Check status
journalctl --user -u flyby-f11-ros2.service -f  # View logs
```

---

## Development Workflow (Podman Compose)

### First-Time Setup

```bash
# 1. Install Podman and GPU support
sudo apt update
sudo apt install podman nvidia-container-toolkit

# 2. Configure GPU passthrough
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# 3. Verify GPU access
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

### Daily Development

```bash
cd /path/to/flyby-f11

# Start all services
podman-compose -f containers/compose/podman-compose.yml up

# OR start in background
podman-compose -f containers/compose/podman-compose.yml up -d

# Enter ROS 2 container for development
podman exec -it flyby-f11-ros2-dev bash

# Inside container - build and test
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch flyby_f11_bringup simulation.launch.py

# In another terminal - view logs
podman-compose -f containers/compose/podman-compose.yml logs -f

# Stop everything
podman-compose -f containers/compose/podman-compose.yml down
```

### Running Specific Services

```bash
# Only execution mode + ROS 2 (no vision)
podman-compose -f containers/compose/podman-compose.yml up execution ros2

# Run SITL simulation
podman-compose -f containers/compose/podman-compose.yml --profile simulation up sitl

# Run mission planning (heavyweight reasoning)
podman-compose -f containers/compose/podman-compose.yml --profile planning up planning
```

### Rebuilding After Changes

```bash
# Rebuild all containers
podman-compose -f containers/compose/podman-compose.yml build

# Rebuild and restart specific service
podman-compose -f containers/compose/podman-compose.yml up --build ros2

# Force rebuild (no cache)
podman-compose -f containers/compose/podman-compose.yml build --no-cache
```

---

## Production Deployment (Quadlet on Jetson)

### First-Time Deployment

#### 1. Build Images on Dev Machine

```bash
cd /path/to/flyby-f11

# Build all production images
podman build -f ontology/Containerfile.execution -t flyby-f11-execution .
podman build -f ros2_ws/Containerfile.ros2 -t flyby-f11-ros2 .
podman build -f ros2_ws/Containerfile.vision -t flyby-f11-vision .
```

#### 2. Save and Transfer to Jetson

```bash
# Save all images to single archive
podman save -o flyby-f11-production.tar \
  flyby-f11-execution \
  flyby-f11-ros2 \
  flyby-f11-vision

# Transfer to Jetson (adjust IP)
scp flyby-f11-production.tar jetson@192.168.1.100:/tmp/

# Also transfer Quadlet configs
scp containers/quadlet/*.container jetson@192.168.1.100:/tmp/
```

#### 3. Install on Jetson

```bash
# SSH to Jetson
ssh jetson@192.168.1.100

# Load images
podman load -i /tmp/flyby-f11-production.tar

# Install Quadlet files
mkdir -p ~/.config/containers/systemd
cp /tmp/*.container ~/.config/containers/systemd/

# Reload systemd
systemctl --user daemon-reload

# Verify services discovered
systemctl --user list-unit-files | grep flyby

# Enable auto-start on boot
systemctl --user enable flyby-f11-execution.service
systemctl --user enable flyby-f11-ros2.service
systemctl --user enable flyby-f11-vision.service

# Enable lingering (services start before login)
loginctl enable-linger $USER
```

#### 4. Start Services

```bash
# Start all enabled services
systemctl --user start flyby-f11-execution.service
systemctl --user start flyby-f11-ros2.service
systemctl --user start flyby-f11-vision.service

# OR start just ROS 2 (dependencies start automatically)
systemctl --user start flyby-f11-ros2.service
```

### Updating After Code Changes

```bash
# On dev machine - rebuild and save
podman build -f ros2_ws/Containerfile.ros2 -t flyby-f11-ros2 .
podman save -o flyby-f11-ros2-update.tar flyby-f11-ros2
scp flyby-f11-ros2-update.tar jetson@192.168.1.100:/tmp/

# On Jetson - load new image and restart
ssh jetson@192.168.1.100
podman load -i /tmp/flyby-f11-ros2-update.tar
systemctl --user restart flyby-f11-ros2.service
```

### Managing Services on Jetson

```bash
# Check status of all services
systemctl --user status 'flyby-f11-*'

# View logs (follow mode)
journalctl --user -u flyby-f11-ros2.service -f

# View logs (last 100 lines)
journalctl --user -u flyby-f11-ros2.service -n 100

# View logs since boot
journalctl --user -u flyby-f11-ros2.service -b

# Restart service
systemctl --user restart flyby-f11-ros2.service

# Stop service
systemctl --user stop flyby-f11-ros2.service

# Disable auto-start
systemctl --user disable flyby-f11-ros2.service
```

### Running Mission Planning (On-Demand)

```bash
# On Jetson - planning doesn't auto-start
systemctl --user start flyby-f11-planning.service

# Monitor planning
journalctl --user -u flyby-f11-planning.service -f

# Check if planning completed
systemctl --user status flyby-f11-planning.service

# View results
cat ~/flyby-f11/ontology/planning_mode/verified_mission_plan.json
```

---

## Common Tasks

### Check GPU Access

```bash
# In any container
nvidia-smi

# Verify CUDA
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

### Interactive Container Shell

```bash
# Development (podman-compose)
podman exec -it flyby-f11-ros2-dev bash

# Production (systemd service)
# First, find container ID
podman ps | grep flyby-f11-ros2

# Then exec into it
podman exec -it <container-id> bash
```

### View ROS 2 Topics/Nodes

```bash
# Enter container
podman exec -it flyby-f11-ros2-dev bash

# Source workspace
source /workspace/install/setup.bash

# List topics
ros2 topic list

# List nodes
ros2 node list

# Echo topic
ros2 topic echo /camera/image_raw
```

### Clean Up Containers/Volumes

```bash
# Development - stop and remove volumes
podman-compose -f containers/compose/podman-compose.yml down -v

# Remove unused containers
podman container prune

# Remove unused images
podman image prune -a

# Remove unused volumes
podman volume prune

# Nuclear option - remove everything
podman system prune -a --volumes
```

---

## Troubleshooting

### GPU Not Detected

```bash
# Regenerate CDI config
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Test GPU access
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

### Container Won't Start

```bash
# Development - check logs
podman-compose -f containers/compose/podman-compose.yml logs ros2

# Production - check systemd logs
journalctl --user -u flyby-f11-ros2.service -xe

# Check if image exists
podman images | grep flyby-f11
```

### ROS 2 Nodes Can't Discover Each Other

```bash
# Check ROS_DOMAIN_ID is consistent
echo $ROS_DOMAIN_ID  # Should be 42

# Use host networking
# Already configured in podman-compose.yml and Quadlet files
```

### Permission Denied on Volumes

```bash
# Fix ownership
chown -R $USER:$USER ./ros2_ws
```

### Out of Disk Space

```bash
# Check usage
podman system df

# Clean up
podman system prune -a --volumes
```

---

## File Locations

### Development Machine

- **Source code**: `./` (this repository root)
- **Container configs**: `containers/compose/podman-compose.yml`, `ontology/Containerfile.*`, `ros2_ws/Containerfile.*`
- **Quadlet templates**: `containers/quadlet/*.container`

### Jetson (Production)

- **Source code**: Clone to any location (e.g., `~/flyby-f11/`)
- **Quadlet configs**: `~/.config/containers/systemd/*.container`
- **Systemd unit files**: Auto-generated in `~/.config/systemd/user/`
- **Logs**: `journalctl --user -u <service>`

---

## Next Steps

### For Development

1. Read [CONTAINERIZATION_STRATEGY.md](CONTAINERIZATION_STRATEGY.md) for detailed container architecture
2. Read [IMPLEMENTATION_ROADMAP.qmd](IMPLEMENTATION_ROADMAP.qmd) for development tasks
3. Start with `podman-compose up` and explore the environment

### For Production Deployment

1. Read [containers/quadlet/README.md](containers/quadlet/README.md) for detailed Quadlet configuration
2. Test Quadlet locally before deploying to Jetson
3. Set up monitoring and alerting on Jetson

### For Understanding the System

1. Read [APPROACH.qmd](APPROACH.qmd) for overall architecture
2. Read [ONTOLOGY_FOUNDATION.qmd](ONTOLOGY_FOUNDATION.qmd) for reasoning system design
3. Explore literature review in [literature_review/SYNTHESIS.qmd](literature_review/SYNTHESIS.qmd)

---

## Quick Reference Cards

### Podman Compose Commands

| Command | Purpose |
|---------|---------|
| `podman-compose -f containers/compose/podman-compose.yml up` | Start all services |
| `podman-compose -f containers/compose/podman-compose.yml up -d` | Start in background |
| `podman-compose -f containers/compose/podman-compose.yml up --build` | Rebuild and start |
| `podman-compose -f containers/compose/podman-compose.yml down` | Stop all services |
| `podman-compose -f containers/compose/podman-compose.yml down -v` | Stop and remove volumes |
| `podman-compose -f containers/compose/podman-compose.yml logs -f` | Follow logs |
| `podman-compose -f containers/compose/podman-compose.yml ps` | List running services |
| `podman-compose -f containers/compose/podman-compose.yml restart <service>` | Restart specific service |

### Systemd (Quadlet) Commands

| Command | Purpose |
|---------|---------|
| `systemctl --user start <service>` | Start service |
| `systemctl --user stop <service>` | Stop service |
| `systemctl --user restart <service>` | Restart service |
| `systemctl --user status <service>` | Check status |
| `systemctl --user enable <service>` | Enable auto-start |
| `systemctl --user disable <service>` | Disable auto-start |
| `journalctl --user -u <service> -f` | Follow logs |
| `systemctl --user daemon-reload` | Reload configs |

### Service Names (Quadlet)

- `flyby-f11-planning.service` - Mission planning (on-demand)
- `flyby-f11-execution.service` - Prolog runtime (always on)
- `flyby-f11-ros2.service` - ROS 2 stack (always on)
- `flyby-f11-vision.service` - Vision pipeline (always on)
- `flyby-f11-sitl.service` - SITL simulation (development only)
