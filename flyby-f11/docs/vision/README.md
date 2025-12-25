# Vision and Perception Documentation

**Platform**: flyby-f11 (NVIDIA Jetson Orin NX 16GB)
**Phase**: 2 (Weeks 7-10) - Vision and Perception Integration
**Last Updated**: 2025-12-25

## Overview

This directory contains critical documentation for vision and perception tools that will be integrated into the flyby-f11 autonomous navigation system. All tools are optimized for NVIDIA Jetson Orin NX deployment.

## Quick Start

```bash
# Download all documentation and repositories
cd /home/finley/Github/DroneProjects/flyby-f11/docs/vision
chmod +x download_vision_docs.sh
./download_vision_docs.sh

# View manifest
cat manifest.txt
```

## Hardware Platform

**NVIDIA Jetson Orin NX 16GB Specifications:**
- **Compute**: 50 TOPS (AI performance)
- **Memory**: 16GB LPDDR5 (2x more than Orin Nano Super)
- **GPU**: 1024 CUDA cores, 32 Tensor cores (Ampere architecture)
- **CPU**: 8-core ARM Cortex-A78AE
- **Power**: 10W - 25W configurable TDP

**Advantages for Vision Tasks:**
- Higher memory enables larger models and concurrent vision pipelines
- TensorRT acceleration for real-time object detection (30+ FPS)
- INT8 quantization support for 3-4x inference speedup
- Sufficient compute for multiple vision tasks simultaneously

## Priority Tools (Phase 2)

### 1. YOLO11 (Ultralytics) - Object Detection

**Purpose**: Real-time object detection for obstacle avoidance and situational awareness

**Key Features:**
- Latest YOLO architecture with improved accuracy and speed
- Native TensorRT export for Jetson optimization
- Multiple model sizes: YOLOv11n (nano), YOLOv11s (small), YOLOv11m (medium)
- DeepStream SDK integration for video analytics

**Documentation:**
- `ultralytics/jetson-deployment-guide.html` - Complete Jetson deployment guide
- `ultralytics/tensorrt-integration.html` - TensorRT export and optimization
- `ultralytics/deepstream-guide.html` - DeepStream SDK integration
- `ultralytics/yolo11-docs.html` - Model architecture and training
- `repos/ultralytics/` - Full source code and examples

**Expected Performance on Jetson Orin NX:**
- YOLOv11n: 80-100 FPS (TensorRT FP16)
- YOLOv11s: 50-70 FPS (TensorRT FP16)
- YOLOv11m: 30-40 FPS (TensorRT FP16)

**Deployment Strategy:**
1. Export model to TensorRT format with FP16 precision
2. Use YOLOv11n for real-time applications (navigation, obstacle avoidance)
3. Use YOLOv11s for higher accuracy tasks (object identification)
4. Integrate with ROS 2 vision_msgs for standardized detection output

---

### 2. TensorRT - Inference Optimization

**Purpose**: Optimize vision models for maximum performance on Jetson

**Key Features:**
- INT8 quantization (3-4x speedup with minimal accuracy loss)
- FP16 precision (2x speedup, negligible accuracy loss)
- Dynamic batch sizes and input shapes
- Layer fusion and kernel auto-tuning

**Documentation:**
- `tensorrt/tensorrt-quick-start.html` - Getting started with TensorRT
- `tensorrt/tensorrt-developer-guide.html` - Advanced optimization techniques
- `tensorrt/jetson-optimization.html` - Jetson-specific best practices

**Optimization Pipeline:**
1. Train model in PyTorch/TensorFlow
2. Export to ONNX format
3. Convert ONNX to TensorRT engine with calibration
4. Deploy optimized engine in ROS 2 inference node

**Recommended Settings for Jetson Orin NX:**
- Precision: FP16 (best speed/accuracy tradeoff)
- Workspace size: 4GB (sufficient for most models)
- Max batch size: 1-4 (real-time applications)
- Builder optimization level: 5 (maximum optimization)

---

### 3. Intel RealSense - Sensor Integration

**Purpose**: Visual odometry, depth perception, and RGB imaging

**Sensors:**
- **T265 Tracking Camera**: 6DOF visual-inertial odometry (VIO)
- **D455 Depth Camera**: Stereo depth + RGB imaging

**Documentation:**
- `realsense/realsense-sdk-overview.html` - SDK setup and API
- `realsense/t265-tracking-camera.html` - T265 specifications and usage
- `realsense/d455-depth-camera.html` - D455 specifications and features
- `repos/realsense-ros/` - Official ROS 2 wrapper
- `repos/realsense_t265_shelfino/` - T265 ROS 2 wrapper with VIO

**ROS 2 Integration:**
```bash
# Install RealSense ROS 2 package
cd flyby-f11/ros2_ws/src
ln -s /home/finley/Github/DroneProjects/flyby-f11/docs/vision/repos/realsense-ros realsense2_camera

# Build workspace
cd ../
colcon build --packages-select realsense2_camera

# Launch cameras
ros2 launch realsense2_camera rs_launch.py
```

**Key Topics:**
- `/camera/color/image_raw` - RGB image (640x480 @ 30 FPS)
- `/camera/depth/image_rect_raw` - Depth image (640x480 @ 30 FPS)
- `/camera/odom/sample` - T265 visual odometry (200 Hz)
- `/camera/imu` - T265 IMU data (200 Hz)

---

### 4. Semantic Segmentation - Scene Understanding

**Purpose**: Pixel-wise scene classification for terrain analysis and obstacle detection

**Tools:**
- **jetson-inference**: NVIDIA's inference library with pretrained models
- **STEPP**: Lightweight semantic segmentation for edge deployment

**Documentation:**
- `jetson-inference/` - Complete jetson-inference documentation
- `jetson-inference/stepp-paper.pdf` - STEPP research paper

**Pretrained Models (jetson-inference):**
- FCN-ResNet18 (Cityscapes): 21 classes, 30 FPS @ 512x256
- SegNet (Cityscapes): 19 classes, 25 FPS @ 640x360
- DeepLabV3+ (ADE20K): 150 classes, 15 FPS @ 512x512

**Use Cases:**
- Terrain classification (ground, obstacles, vegetation)
- Navigable area detection
- Landing zone identification
- Indoor/outdoor scene understanding

**Deployment:**
```bash
# Clone jetson-inference
cd flyby-f11/docs/vision/repos
git clone --depth 1 https://github.com/dusty-nv/jetson-inference

# Run semantic segmentation
cd jetson-inference/build
./segnet --network=fcn-resnet18-cityscapes /dev/video0
```

---

### 5. Vision-Language Models (VLM) - Mission-Intent Interpretation

**Purpose**: Natural language understanding for mission commands and situational awareness

**Tools:**
- **VILA**: Visual Language Intelligence Assistant (NVIDIA)
- **NanoVLM**: Lightweight VLM for edge deployment
- **Jetson Platform Services**: NVIDIA's VLM inference framework

**Documentation:**
- `vlm/jetson-platform-services-vlm.html` - Platform services overview
- `vlm/vila-deployment.html` - VILA deployment guide
- `vlm/jetson-generative-ai-lab.html` - Jetson AI Lab VLM examples
- `vlm/nanovlm-docs.html` - NanoVLM implementation

**Capabilities:**
- Visual question answering (VQA)
- Image captioning and scene description
- Mission intent parsing from natural language
- Situational awareness and anomaly detection

**Example Use Cases:**
- "Fly to the red building and land on the roof"
- "Avoid the area with people"
- "Follow the road until you see a blue car"

**Model Sizes for Jetson Orin NX:**
- VILA-3B: 3 billion parameters, ~6GB VRAM, 2-3 tokens/sec
- VILA-7B: 7 billion parameters, ~14GB VRAM, 1-2 tokens/sec (tight fit)
- NanoVLM: Optimized lightweight model, <2GB VRAM, 5-10 tokens/sec

**Deployment Strategy:**
1. Start with NanoVLM for real-time applications
2. Use VILA-3B for higher accuracy mission planning (offline)
3. Quantize models to INT8 for faster inference
4. Integrate with ROS 2 for mission command processing

---

## Integration Roadmap

### Week 7: Object Detection (YOLO11)
1. Download and set up Ultralytics repository
2. Export YOLOv11n to TensorRT (FP16)
3. Benchmark performance on Jetson Orin NX
4. Create ROS 2 object detection node
5. Integrate with perception pipeline

### Week 8: Sensor Fusion (RealSense)
1. Set up RealSense SDK and ROS 2 wrapper
2. Calibrate T265 and D455 cameras
3. Fuse visual odometry with PX4 EKF
4. Create depth-based obstacle avoidance
5. Test in simulation and on hardware

### Week 9: Semantic Segmentation
1. Set up jetson-inference library
2. Benchmark segmentation models (FCN-ResNet18, SegNet)
3. Create ROS 2 segmentation node
4. Implement terrain classification
5. Integrate with navigation stack

### Week 10: VLM Integration
1. Set up NVIDIA Jetson Platform Services
2. Deploy NanoVLM or VILA-3B
3. Create mission intent parser
4. Integrate with behavior tree system
5. Test natural language mission commands

---

## Performance Benchmarks

**Target Performance (Jetson Orin NX 16GB):**

| Task | Model | Input Size | FPS | Latency | VRAM |
|------|-------|------------|-----|---------|------|
| Object Detection | YOLOv11n | 640x640 | 80-100 | 10-12ms | 500MB |
| Object Detection | YOLOv11s | 640x640 | 50-70 | 14-20ms | 800MB |
| Object Detection | YOLOv11m | 640x640 | 30-40 | 25-33ms | 1.2GB |
| Semantic Segmentation | FCN-ResNet18 | 512x256 | 30 | 33ms | 600MB |
| Semantic Segmentation | SegNet | 640x360 | 25 | 40ms | 700MB |
| VLM | NanoVLM | 224x224 | 5-10 | 100-200ms | 2GB |
| VLM | VILA-3B | 336x336 | 2-3 | 333-500ms | 6GB |

**Concurrent Pipeline (Example):**
- YOLOv11n (object detection): 500MB
- FCN-ResNet18 (segmentation): 600MB
- RealSense processing: 500MB
- ROS 2 overhead: 500MB
- System overhead: 2GB
- **Total**: ~4.1GB / 16GB available (comfortable margin)

---

## Directory Structure

```
/home/finley/Github/DroneProjects/flyby-f11/docs/vision/
├── README.md                     # This file
├── download_vision_docs.sh       # Download script
├── manifest.txt                  # Documentation manifest
│
├── ultralytics/                  # YOLO11 documentation
│   ├── jetson-deployment-guide.html
│   ├── tensorrt-integration.html
│   ├── deepstream-guide.html
│   ├── yolo11-docs.html
│   └── export-guide.html
│
├── tensorrt/                     # TensorRT optimization
│   ├── tensorrt-quick-start.html
│   ├── tensorrt-developer-guide.html
│   └── jetson-optimization.html
│
├── realsense/                    # Intel RealSense cameras
│   ├── realsense-sdk-overview.html
│   ├── t265-tracking-camera.html
│   ├── d455-depth-camera.html
│   └── realsense-ros-readme.html
│
├── jetson-inference/             # Semantic segmentation
│   ├── stepp-paper.pdf
│   └── [jetson-inference docs/]
│
├── vlm/                          # Vision-Language Models
│   ├── jetson-platform-services-vlm.html
│   ├── vila-deployment.html
│   ├── jetson-generative-ai-lab.html
│   └── nanovlm-docs.html
│
└── repos/                        # Cloned repositories
    ├── ultralytics/
    ├── realsense-ros/
    ├── realsense_t265_shelfino/
    └── jetson-inference/
```

---

## Development Workflow

### 1. Model Development (Simulation)
```bash
# Develop and test models in simulation
cd project-drone/
docker compose up

# Train/fine-tune models on desktop GPU
# Export to TensorRT format
# Benchmark in simulation
```

### 2. Model Optimization (Jetson)
```bash
# Deploy to flyby-f11 hardware
cd flyby-f11/
docker compose up

# Convert models to TensorRT
# Benchmark on Jetson Orin NX
# Profile memory and compute usage
```

### 3. ROS 2 Integration
```bash
# Create perception pipeline nodes
cd flyby-f11/ros2_ws/src
# Add vision nodes to perception_pipeline package
# Build and test with real sensors
```

### 4. System Integration
```bash
# Integrate with behavior trees and autonomy stack
# Test full navigation pipeline
# Validate in simulation and hardware
```

---

## Troubleshooting

### TensorRT Conversion Issues
- **Problem**: Out of memory during engine build
- **Solution**: Reduce workspace size or use smaller model variant

### RealSense Camera Not Detected
- **Problem**: Camera not showing in `rs-enumerate-devices`
- **Solution**: Check USB connection, update udev rules, verify firmware

### Low Frame Rate
- **Problem**: Vision pipeline running below target FPS
- **Solution**: Use smaller model, enable INT8 quantization, reduce input size

### ROS 2 Topic Latency
- **Problem**: High latency between camera and perception nodes
- **Solution**: Use zero-copy transport, increase DDS buffer sizes, optimize node execution

---

## Additional Resources

**Official Documentation:**
- [Ultralytics YOLO Docs](https://docs.ultralytics.com/)
- [NVIDIA TensorRT Developer Guide](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/)
- [Intel RealSense SDK](https://dev.intelrealsense.com/)
- [Jetson AI Lab](https://www.jetson-ai-lab.com/)

**Community Resources:**
- [Jetson Forums](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/)
- [RealSense GitHub Discussions](https://github.com/IntelRealSense/librealsense/discussions)
- [Ultralytics Discord](https://discord.gg/ultralytics)

**Research Papers:**
- STEPP: Semantic Segmentation for Edge Deployment
- YOLO11: Latest Architecture Improvements
- VILA: Visual Language Intelligence Assistant

---

## Next Steps

1. **Run Download Script**: Execute `download_vision_docs.sh` to fetch all documentation
2. **Review Documentation**: Read through key guides for each tool
3. **Set Up Development Environment**: Install dependencies and test tools
4. **Benchmark Models**: Test performance on Jetson Orin NX hardware
5. **Begin Integration**: Start with YOLO11 object detection in Week 7

---

**Maintainer**: Finley Holt
**Last Updated**: 2025-12-25
**Version**: 1.0
