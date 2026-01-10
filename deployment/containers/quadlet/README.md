# Quadlet Configuration for Systemd-Managed Containers

This directory contains **Quadlet** configuration files for deploying Flyby F-11 containers as systemd services on the Jetson Orin NX.

## What is Quadlet?

Quadlet is Podman's native integration with systemd. It converts `.container` files into systemd unit files automatically, providing:

- **Systemd lifecycle management** (start on boot, auto-restart, logging)
- **Native systemd commands** (`systemctl start/stop/status`)
- **Better for production** deployments on embedded systems
- **Rootless-first design** with no daemon required

## Directory Contents

### Production Services (Auto-start on boot)

1. **`flyby-f11-execution.container`** - Runtime Prolog reasoning (always running)
2. **`flyby-f11-ros2.container`** - ROS 2 autonomy stack (always running)
3. **`flyby-f11-vision.container`** - GPU vision pipeline (always running)

### On-Demand Services

4. **`flyby-f11-planning.container`** - Heavyweight mission planning (manual trigger)
5. **`flyby-f11-sitl.container`** - ArduPilot SITL simulation (development only)

## Installation on Jetson

### 1. Build Container Images

First, build all container images on your development machine:

```bash
cd /home/finley/Github/DroneProjects/flyby-f11

# Build all images
podman build -f ontology/Containerfile.planning -t flyby-f11-planning .
podman build -f ontology/Containerfile.execution -t flyby-f11-execution .
podman build -f ros2_ws/Containerfile.ros2 -t flyby-f11-ros2 .
podman build -f ros2_ws/Containerfile.vision -t flyby-f11-vision .
podman build -f simulation/Containerfile.sitl -t flyby-f11-sitl .
```

### 2. Transfer Images to Jetson

Save images and transfer to Jetson:

```bash
# Save images
podman save -o flyby-f11-images.tar \
  flyby-f11-planning \
  flyby-f11-execution \
  flyby-f11-ros2 \
  flyby-f11-vision

# Transfer to Jetson
scp flyby-f11-images.tar jetson@<jetson-ip>:/tmp/

# On Jetson - load images
ssh jetson@<jetson-ip>
podman load -i /tmp/flyby-f11-images.tar
```

### 3. Install Quadlet Files

Copy `.container` files to systemd user directory:

```bash
# On Jetson
mkdir -p ~/.config/containers/systemd

# Copy all Quadlet files
cd /home/jetson/flyby-f11
cp quadlet/*.container ~/.config/containers/systemd/

# Reload systemd to pick up new units
systemctl --user daemon-reload
```

### 4. Verify Installation

Check that systemd recognizes the new services:

```bash
# List all flyby services
systemctl --user list-unit-files | grep flyby

# Should show:
# flyby-f11-execution.service    disabled
# flyby-f11-planning.service     disabled
# flyby-f11-ros2.service         disabled
# flyby-f11-vision.service       disabled
# flyby-f11-sitl.service         disabled
```

## Usage

### Start Services Manually

```bash
# Start execution mode (lightweight Prolog runtime)
systemctl --user start flyby-f11-execution.service

# Start ROS 2 stack (depends on execution mode)
systemctl --user start flyby-f11-ros2.service

# Start vision pipeline (depends on ROS 2)
systemctl --user start flyby-f11-vision.service
```

### Enable Auto-Start on Boot

```bash
# Enable mission-critical services
systemctl --user enable flyby-f11-execution.service
systemctl --user enable flyby-f11-ros2.service
systemctl --user enable flyby-f11-vision.service

# Verify enabled
systemctl --user is-enabled flyby-f11-ros2.service
# Output: enabled
```

### Check Service Status

```bash
# Check status of all services
systemctl --user status 'flyby-f11-*'

# Check individual service
systemctl --user status flyby-f11-ros2.service
```

### View Logs

```bash
# Follow logs for ROS 2 service
journalctl --user -u flyby-f11-ros2.service -f

# View last 100 lines
journalctl --user -u flyby-f11-ros2.service -n 100

# View logs since boot
journalctl --user -u flyby-f11-ros2.service -b
```

### Restart Services

```bash
# Restart after code update
systemctl --user restart flyby-f11-ros2.service

# Restart all services
systemctl --user restart 'flyby-f11-*'
```

### Stop Services

```bash
# Stop individual service
systemctl --user stop flyby-f11-vision.service

# Stop all services
systemctl --user stop 'flyby-f11-*'
```

### Disable Auto-Start

```bash
# Disable auto-start
systemctl --user disable flyby-f11-ros2.service
```

## Running Planning Mode (On-Demand)

Planning mode is not auto-started. Trigger manually when mission planning is needed:

```bash
# Start planning (heavyweight reasoning)
systemctl --user start flyby-f11-planning.service

# Monitor planning progress
journalctl --user -u flyby-f11-planning.service -f

# Planning completes and exits - check status
systemctl --user status flyby-f11-planning.service

# View planning results
cat ~/flyby-f11/ontology/planning_mode/verified_mission_plan.json
```

## Service Dependencies

The services have dependency ordering:

```
flyby-f11-planning.service (on-demand)
    ↓
flyby-f11-execution.service (always running)
    ↓
flyby-f11-ros2.service (depends on execution)
    ↓
flyby-f11-vision.service (depends on ROS 2)
```

Starting `flyby-f11-vision` will automatically start its dependencies.

## Customization

### Adjust Resource Limits

Edit `.container` files to change memory/CPU limits:

```ini
[Container]
Memory=8G          # Change memory limit
MemorySwap=10G     # Change swap limit
CPUQuota=75%       # Limit CPU usage
```

After editing, reload:

```bash
systemctl --user daemon-reload
systemctl --user restart flyby-f11-ros2.service
```

### Change Auto-Restart Behavior

```ini
[Service]
Restart=always      # Options: always, on-failure, no
RestartSec=10s      # Wait before restart
```

### Add Environment Variables

```ini
[Container]
Environment=NEW_VAR=value
Environment=ANOTHER_VAR=another_value
```

### Mount Additional Volumes

```ini
[Container]
Volume=%h/additional/path:/container/path:z
```

## Troubleshooting

### Service Won't Start

```bash
# Check detailed status
systemctl --user status flyby-f11-ros2.service

# View full logs
journalctl --user -u flyby-f11-ros2.service -xe

# Check if image exists
podman images | grep flyby-f11

# Manually run container to debug
podman run --rm -it flyby-f11-ros2 bash
```

### GPU Not Available

```bash
# Verify CDI configuration exists
ls /etc/cdi/nvidia.yaml

# Regenerate if missing
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Restart service
systemctl --user restart flyby-f11-vision.service
```

### Service Fails After Boot

```bash
# Check if network is ready
systemctl status network-online.target

# Enable lingering (services start before login)
loginctl enable-linger $USER
```

### High Resource Usage

```bash
# Check resource consumption
systemctl --user show flyby-f11-ros2.service | grep Memory
systemctl --user show flyby-f11-ros2.service | grep CPU

# Monitor in real-time
systemctl --user status flyby-f11-ros2.service
```

## Comparison with Podman Compose

| Feature | Quadlet (Production) | Podman Compose (Dev) |
|---------|---------------------|----------------------|
| **Use Case** | Jetson deployment | Development iteration |
| **Auto-start on boot** | ✅ Yes | ❌ No |
| **Systemd integration** | ✅ Native | ❌ No |
| **Restart policies** | ✅ Full systemd | ⚠️ Limited |
| **Logging** | ✅ Journald | ⚠️ Container logs |
| **Quick rebuild** | ❌ Manual | ✅ `up --build` |
| **Multi-container** | ⚠️ Via dependencies | ✅ Easy |
| **Resource limits** | ✅ Systemd managed | ⚠️ Container-level |

**Recommendation**: Use Podman Compose for development, Quadlet for Jetson deployment.

## Advanced: Monitoring and Alerting

### Email Alerts on Failure

Create a systemd drop-in for email notifications:

```bash
mkdir -p ~/.config/systemd/user/flyby-f11-ros2.service.d/
cat > ~/.config/systemd/user/flyby-f11-ros2.service.d/email-alert.conf <<EOF
[Unit]
OnFailure=email-alert@%n.service
EOF

systemctl --user daemon-reload
```

### Watchdog for Health Monitoring

Add to `.container` file:

```ini
[Service]
WatchdogSec=30
```

Requires application to send watchdog pings via `sd_notify()`.

### Automatic Cleanup

Clean up old container logs:

```bash
# Add to crontab
0 0 * * * journalctl --user --vacuum-time=7d
```

## Further Reading

- [Podman Quadlet Documentation](https://docs.podman.io/en/latest/markdown/podman-systemd.unit.5.html)
- [Systemd Service Management](https://www.freedesktop.org/software/systemd/man/systemctl.html)
- [Podman Rootless Containers](https://github.com/containers/podman/blob/main/docs/tutorials/rootless_tutorial.md)
