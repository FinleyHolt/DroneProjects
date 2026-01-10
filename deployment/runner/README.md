# GitHub Actions Self-Hosted Runner

This directory contains the setup script for the DroneProjects self-hosted runner.

## Why Self-Hosted?

GitHub-hosted runners don't have:
- NVIDIA GPUs for Isaac Sim
- Access to local container images
- Sufficient resources for simulation

## Prerequisites

1. **NVIDIA GPU** with drivers installed
2. **Podman** with GPU passthrough configured
3. **GitHub CLI** (`gh`) authenticated
4. **Isaac Sim container** built locally:
   ```bash
   cd isaac-sim
   podman-compose build
   ```

## Quick Setup

```bash
# Run the setup script
./deployment/runner/setup-runner.sh

# Start the runner (foreground)
cd /home/finley/Github/DroneProjects/.runner
./run.sh

# Or install as a service (recommended)
cd /home/finley/Github/DroneProjects/.runner
sudo ./svc.sh install
sudo ./svc.sh start
```

## Runner Labels

The runner is configured with these labels:
- `self-hosted` - Standard self-hosted label
- `linux` - OS
- `x64` - Architecture
- `gpu` - Has NVIDIA GPU
- `isaac-sim` - Has Isaac Sim container

Workflows target these labels:
```yaml
runs-on: [self-hosted, gpu, isaac-sim]
```

## Test Pipeline

| Stage | What it tests | Container |
|-------|---------------|-----------|
| Preflight | GPU + container exist | isaac-sim-px4 |
| Unit | Mocked tests | isaac-sim-px4 |
| Integration | Real Isaac Sim | isaac-sim-px4 |
| E2E Smoke | Quick sim runs | isaac-sim-px4 |
| E2E Full | Long sim runs (manual) | isaac-sim-px4 |

## Troubleshooting

### Runner won't start
```bash
# Check service status
sudo ./svc.sh status

# View logs
journalctl -u actions.runner.FinleyHolt-DroneProjects.droneprojects-$(hostname)
```

### GPU not accessible in container
```bash
# Verify CDI config exists
ls /etc/cdi/nvidia.yaml

# Test GPU access
podman run --rm --device nvidia.com/gpu=all nvidia/cuda:12.0-base nvidia-smi
```

### Container not found
```bash
# Rebuild Isaac Sim container
cd isaac-sim
podman-compose build
```

## Updating the Runner

```bash
cd ~/actions-runner
./svc.sh stop
# Re-run setup script (will download new version)
./deployment/runner/setup-runner.sh
./svc.sh start
```
