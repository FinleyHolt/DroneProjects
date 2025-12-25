# Tools Documentation Manifest

## Overview

This directory contains reference documentation for development tools and frameworks used in the flyby-f11 project.

**Location**: `/home/finley/Github/DroneProjects/flyby-f11/docs/tools/`

**Purpose**: Provide offline reference material for NVIDIA JetPack, Docker containerization, and Quarto documentation rendering.

## Contents

### 1. NVIDIA JetPack and Docker Reference

**File**: `JETPACK_DOCKER_REFERENCE.md`

**Description**: Comprehensive guide for JetPack 6.1 SDK and Docker-based development on NVIDIA Jetson platforms:
- JetPack 6.1 components and features
- Linux for Tegra (L4T) 36.4 configuration
- CUDA 12.6, TensorRT 10.3, and AI frameworks
- Docker with NVIDIA runtime setup
- dusty-nv/jetson-containers integration
- Performance optimization and power management

**Key Topics**:
- **JetPack Components**:
  - CUDA Toolkit 12.6
  - TensorRT 10.3.0 for model optimization
  - cuDNN 9.3.0 for deep learning
  - PyTorch 2.5.0, TensorFlow 2.17.0
  - VPI 3.2 (Vision Programming Interface)
  - DeepStream SDK 7.1

- **Jetson Platforms**:
  - Jetson Orin Nano Super 8GB (project-drone: 67 TOPS)
  - Jetson Orin NX 16GB (flyby-f11: 50 TOPS)
  - Configuration and optimization for both

- **Docker Integration**:
  - NVIDIA Container Runtime setup
  - Base images from NGC catalog
  - Docker Compose for multi-service deployments

- **dusty-nv/jetson-containers**:
  - Pre-built containers for PyTorch, TensorFlow, ROS 2
  - Build system for custom images
  - Examples for robotics applications

**External Resources**:
- **JetPack Documentation**: https://developer.nvidia.com/embedded/jetpack
- **L4T 36.4 Docs**: https://docs.nvidia.com/jetson/archives/r36.4/
- **dusty-nv/jetson-containers GitHub**: https://github.com/dusty-nv/jetson-containers
  - CRITICAL REFERENCE for containerized Jetson development
  - Pre-built images for common frameworks
  - Build scripts and automation
  - ROS 2 + PyTorch combined containers
- **NVIDIA NGC Catalog**: https://catalog.ngc.nvidia.com/containers
- **Jetson Forums**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/

**Tools Covered**:
- **Monitoring**: jetson-stats (jtop), tegrastats
- **Power Management**: nvpmodel, jetson_clocks
- **Model Optimization**: TensorRT, torch2trt
- **Vision Processing**: VPI, OpenCV with CUDA
- **Multimedia**: GStreamer with hardware acceleration

**Docker Compose Example**:
```yaml
services:
  autonomy:
    image: dustynv/ros:humble-pytorch-r36.4.0
    runtime: nvidia
    volumes:
      - ./ros2_ws:/workspace/ros2_ws
```

**Use in flyby-f11**:
- Containerized ROS 2 development
- AI model deployment with TensorRT
- Hardware-accelerated vision processing
- Multi-service orchestration with Docker Compose
- Target platform: Jetson Orin NX 16GB

---

### 2. Quarto Documentation Reference

**File**: `QUARTO_REFERENCE.md`

**Description**: Complete reference for Quarto scientific publishing system, as used for flyby-f11 project documentation:
- Quarto installation and setup
- PDF rendering with LaTeX
- YAML frontmatter configuration
- Project-specific documentation template
- Code execution and dynamic content

**Key Topics**:
- **PDF Output**:
  - LaTeX integration (TinyTeX recommended)
  - Custom headers with fancyhdr
  - Geometry and font configuration
  - Table of contents and section numbering

- **flyby-f11 Standard Template**:
  ```yaml
  ---
  title: "Document Title"
  author: "Finley Holt"
  date: today
  format:
    pdf:
      documentclass: article
      geometry:
        - margin=1in
      fontsize: 11pt
      number-sections: true
      toc: true
      include-in-header:
        text: |
          \usepackage{fancyhdr}
          \pagestyle{fancy}
          \fancyhf{}
          \fancyhead[L]{\textit{Short Title}}
          \fancyhead[R]{\thepage}
          \renewcommand{\headrulewidth}{0.4pt}
  ---
  ```

- **Advanced Features**:
  - Code execution (Python, R, Julia)
  - Cross-references for figures, tables, equations
  - Citations and bibliographies
  - Callout blocks and tabbed content
  - Mathematical equations with LaTeX

**Commands**:
```bash
# Render document to PDF
quarto render document.qmd

# Live preview with auto-reload
quarto preview document.qmd

# Render all project documents
quarto render .

# Install TinyTeX for LaTeX
quarto install tinytex
```

**External Resources**:
- **Official Documentation**: https://quarto.org/docs/guide/
- **PDF Format Guide**: https://quarto.org/docs/output-formats/pdf-basics.html
- **Reference**: https://quarto.org/docs/reference/
- **Gallery**: https://quarto.org/docs/gallery/
- **GitHub**: https://github.com/quarto-dev/quarto-cli

**Use in flyby-f11**:
- All technical documentation (`.qmd` files)
- Consistent formatting across project docs
- Examples: `APPROACH.qmd`, `ONTOLOGY_FOUNDATION.qmd`, `SYSTEM_CONSTRAINTS.qmd`
- Literature review papers in `/literature_review/`

---

## Related Documentation

### In Other Directories

- **Standards**: `/home/finley/Github/DroneProjects/flyby-f11/docs/standards/`
  - IEEE 1872.2 autonomous robotics ontology
  - TPTP formal logic specifications

- **ROS 2**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/`
  - ROS 2 Humble integration
  - Package development guides

- **Vision**: `/home/finley/Github/DroneProjects/flyby-f11/docs/vision/`
  - Computer vision pipelines
  - Model deployment on Jetson

- **Flight Control**: `/home/finley/Github/DroneProjects/flyby-f11/docs/flight_control/`
  - PX4 configuration
  - MAVSDK integration

## Download Instructions

### Cloning Referenced Repositories

```bash
# Navigate to tools directory
cd /home/finley/Github/DroneProjects/flyby-f11/docs/tools/

# Clone dusty-nv/jetson-containers (CRITICAL REFERENCE)
git clone --depth 1 https://github.com/dusty-nv/jetson-containers.git

# This provides:
# - Pre-built container recipes for PyTorch, TensorFlow, ROS 2
# - Build scripts for custom images
# - Documentation and examples
# - Integration patterns for Jetson development
```

### Installing Tools

#### JetPack (on Jetson Hardware)

```bash
# Using NVIDIA SDK Manager (GUI method)
# Download from: https://developer.nvidia.com/sdk-manager

# Or using command-line flash tools
# Download L4T from: https://developer.nvidia.com/embedded/linux-tegra

# Post-installation: Install jetson-stats
sudo pip3 install -U jetson-stats
```

#### Quarto (on Development Machine)

```bash
# Download latest release
wget https://github.com/quarto-dev/quarto-cli/releases/download/v1.5.40/quarto-1.5.40-linux-amd64.deb
sudo dpkg -i quarto-1.5.40-linux-amd64.deb

# Install TinyTeX for PDF rendering
quarto install tinytex

# Verify installation
quarto --version
quarto check
```

## Usage Guidelines

### JetPack Development Workflow

1. **Start with dusty-nv Containers**: Use pre-built base images
2. **Layer Project Code**: Mount ROS 2 workspace as volume
3. **Optimize Models**: Convert to TensorRT for inference
4. **Monitor Resources**: Use jtop to track GPU/CPU/memory
5. **Configure Power**: Select nvpmodel based on mission requirements

### Quarto Documentation Workflow

1. **Create `.qmd` File**: Use standard template from reference
2. **Write Content**: Markdown with optional code execution
3. **Preview**: `quarto preview document.qmd` for live updates
4. **Render PDF**: `quarto render document.qmd`
5. **Version Control**: Commit `.qmd` files (PDFs optional)

## Performance and Optimization

### Jetson Orin NX (flyby-f11 Target)

**Specifications**:
- GPU: Ampere architecture, 1,024 CUDA cores, 32 Tensor cores
- Compute: 50 TOPS (INT8)
- Memory: 16GB LPDDR5
- Power: 10W / 15W / 25W modes

**Optimization Tips**:
1. Use TensorRT for model inference (2-10x speedup)
2. Leverage VPI for hardware-accelerated CV operations
3. Configure appropriate power mode for mission duration
4. Use GStreamer for hardware video encoding/decoding
5. Monitor thermal throttling with tegrastats

### Docker Best Practices

1. **Base Images**: Start from dustynv/ros:humble-pytorch-r36.4.0
2. **Volume Mounts**: Use volumes for code (avoid rebuilds)
3. **Network Mode**: Use `network_mode: host` for ROS 2 discovery
4. **Device Access**: Mount `/dev` for cameras and sensors
5. **Resource Limits**: Set memory limits to prevent OOM

## Troubleshooting

### JetPack Issues

**Problem**: Docker can't access GPU
```bash
# Solution: Verify NVIDIA runtime
docker run --rm --runtime nvidia nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi
sudo systemctl restart docker
```

**Problem**: Camera not detected
```bash
# Solution: Check permissions
sudo usermod -aG video $USER
ls -l /dev/video*
```

### Quarto Issues

**Problem**: LaTeX errors during PDF rendering
```bash
# Solution: Install missing packages
quarto install tinytex
tlmgr install <package-name>

# Or use system LaTeX
sudo apt install texlive-full
```

**Problem**: Fonts not found
```yaml
# Solution: Specify available fonts
format:
  pdf:
    mainfont: "DejaVu Serif"
```

## Integration Examples

### Docker Compose for flyby-f11

```yaml
version: "3.9"

services:
  ros2_autonomy:
    image: dustynv/ros:humble-pytorch-r36.4.0
    runtime: nvidia
    network_mode: host
    privileged: true
    volumes:
      - ./ros2_ws:/workspace/ros2_ws
      - /dev:/dev
    environment:
      - ROS_DOMAIN_ID=42
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        cd /workspace/ros2_ws &&
        colcon build --symlink-install &&
        source install/setup.bash &&
        ros2 launch flyby_f11_bringup simulation.launch.py
      "
```

### Quarto Project Configuration

`_quarto.yml` in `/home/finley/Github/DroneProjects/flyby-f11/`:

```yaml
project:
  type: default
  output-dir: _output

format:
  pdf:
    documentclass: article
    fontsize: 11pt
    geometry:
      - margin=1in
    toc: true
    number-sections: true
    include-in-header:
      text: |
        \usepackage{fancyhdr}
        \pagestyle{fancy}
```

## Maintenance

### Updating References

```bash
# Update jetson-containers repository
cd /home/finley/Github/DroneProjects/flyby-f11/docs/tools/jetson-containers
git pull

# Check for JetPack updates
# Visit: https://developer.nvidia.com/embedded/jetpack

# Update Quarto
# Check: https://github.com/quarto-dev/quarto-cli/releases
```

### Version Information

- **JetPack**: 6.1 (L4T 36.4)
- **CUDA**: 12.6
- **TensorRT**: 10.3.0
- **ROS 2**: Humble (matches Ubuntu 22.04)
- **Quarto**: 1.5.40+
- **Documentation Created**: 2025-12-25

## Hardware Platforms

### project-drone (Development Platform)
- **Compute**: Jetson Orin Nano Super 8GB (67 TOPS)
- **Status**: Available, active development
- **Use**: Algorithm development, shared component testing

### flyby-f11 (Target Platform)
- **Compute**: Jetson Orin NX 16GB (50 TOPS)
- **Status**: Hardware access via MCTSSA
- **Use**: Production deployment, mission-specific autonomy

**Note**: flyby-f11 has 2x memory (16GB vs 8GB) but slightly less compute (50 vs 67 TOPS). Optimize models on project-drone, deploy to flyby-f11 with TensorRT.

## Contact and Support

For questions about using these tools in the flyby-f11 project:
- Project Lead: Finley Holt
- Repository: `/home/finley/Github/DroneProjects/flyby-f11/`

For tool-specific support:
- **JetPack**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/
- **dusty-nv/jetson-containers**: https://github.com/dusty-nv/jetson-containers/discussions
- **Quarto**: https://github.com/quarto-dev/quarto-cli/discussions

## License and Attribution

- **NVIDIA JetPack**: Proprietary (free for Jetson platforms)
- **dusty-nv/jetson-containers**: MIT License
- **Quarto**: GPL v2
- **Reference Documentation**: Created for flyby-f11 project, educational use

---

**Last Updated**: 2025-12-25
**Maintainer**: Finley Holt
