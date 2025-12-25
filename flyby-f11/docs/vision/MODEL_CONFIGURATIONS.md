# Model Configurations and Optimization Guide

**Platform**: NVIDIA Jetson Orin NX 16GB (50 TOPS)
**Target Framework**: TensorRT, ROS 2 Humble
**Last Updated**: 2025-12-25

## Quick Reference: Recommended Models

| Use Case | Model | Input Size | Precision | FPS | VRAM | Priority |
|----------|-------|------------|-----------|-----|------|----------|
| Real-time obstacle detection | YOLOv11n | 640x640 | FP16 | 80-100 | 500MB | HIGH |
| Accurate object detection | YOLOv11s | 640x640 | FP16 | 50-70 | 800MB | MEDIUM |
| Terrain classification | FCN-ResNet18 | 512x256 | FP16 | 30 | 600MB | HIGH |
| Scene understanding | SegNet | 640x360 | FP16 | 25 | 700MB | MEDIUM |
| Mission intent parsing | NanoVLM | 224x224 | FP16 | 5-10 | 2GB | MEDIUM |
| Advanced VLM tasks | VILA-3B | 336x336 | FP16 | 2-3 | 6GB | LOW |

---

## 1. YOLO11 Object Detection

### Model Variants

#### YOLOv11n (Nano) - **RECOMMENDED FOR REAL-TIME**
```yaml
Model: yolo11n.pt
Parameters: 2.6M
Input: 640x640
Output: 80 COCO classes
Precision: FP16 (TensorRT)
Expected FPS: 80-100
Latency: 10-12ms
VRAM: ~500MB
Use Case: Real-time obstacle detection, navigation
```

**Export to TensorRT:**
```bash
# Install Ultralytics
pip install ultralytics

# Export model
from ultralytics import YOLO
model = YOLO('yolo11n.pt')
model.export(format='engine', device=0, half=True, workspace=4)
# Output: yolo11n.engine
```

**ROS 2 Integration:**
```python
# Create inference node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from ultralytics import YOLO

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection')
        self.model = YOLO('yolo11n.engine')  # Load TensorRT engine
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.pub = self.create_publisher(Detection2DArray, '/detections', 10)

    def callback(self, msg):
        # Run inference
        results = self.model(msg, verbose=False)
        # Publish detections
        self.pub.publish(self.convert_to_detection_msg(results))
```

---

#### YOLOv11s (Small) - Balanced Accuracy/Speed
```yaml
Model: yolo11s.pt
Parameters: 9.4M
Input: 640x640
Output: 80 COCO classes
Precision: FP16 (TensorRT)
Expected FPS: 50-70
Latency: 14-20ms
VRAM: ~800MB
Use Case: Higher accuracy object identification
```

---

#### YOLOv11m (Medium) - High Accuracy
```yaml
Model: yolo11m.pt
Parameters: 20.1M
Input: 640x640
Output: 80 COCO classes
Precision: FP16 (TensorRT)
Expected FPS: 30-40
Latency: 25-33ms
VRAM: ~1.2GB
Use Case: Offline analysis, mission planning
```

---

### TensorRT Optimization Settings

```python
# Export with optimal settings for Jetson Orin NX
model.export(
    format='engine',           # TensorRT engine
    device=0,                  # GPU device
    half=True,                 # FP16 precision
    workspace=4,               # 4GB workspace
    int8=False,                # Disable INT8 (minimal accuracy gain)
    simplify=True,             # ONNX simplification
    dynamic=False,             # Static shapes (faster)
    batch=1,                   # Batch size 1 for real-time
    imgsz=640                  # Input size 640x640
)
```

**Performance Tuning:**
```python
# For maximum FPS (lower accuracy)
imgsz=416  # Smaller input -> 120+ FPS with YOLOv11n

# For maximum accuracy (lower FPS)
imgsz=1280  # Larger input -> ~20 FPS with YOLOv11n
```

---

## 2. Semantic Segmentation

### FCN-ResNet18 (Cityscapes) - **RECOMMENDED**

```yaml
Model: FCN-ResNet18-Cityscapes
Classes: 21 (road, sidewalk, building, wall, fence, pole, traffic light, etc.)
Input: 512x256
Precision: FP16
Expected FPS: 30
Latency: 33ms
VRAM: ~600MB
Use Case: Urban navigation, terrain classification
```

**Class Labels (Cityscapes):**
```
0: road, 1: sidewalk, 2: building, 3: wall, 4: fence,
5: pole, 6: traffic light, 7: traffic sign, 8: vegetation,
9: terrain, 10: sky, 11: person, 12: rider, 13: car,
14: truck, 15: bus, 16: train, 17: motorcycle, 18: bicycle,
19: void, 20: ego-vehicle
```

**jetson-inference Usage:**
```bash
# Download pretrained model
cd jetson-inference/tools
./download-models.sh

# Run segmentation
./segnet --network=fcn-resnet18-cityscapes \
         --visualize=overlay \
         /dev/video0
```

**ROS 2 Integration:**
```bash
# Use jetson-inference ROS 2 wrapper
sudo apt install ros-humble-vision-msgs
cd flyby-f11/ros2_ws/src
git clone https://github.com/dusty-nv/ros_deep_learning.git
cd ../
colcon build --packages-select ros_deep_learning

# Launch segmentation node
ros2 launch ros_deep_learning video_source.ros2.launch \
    input:=/camera/color/image_raw \
    output:=/segmentation/overlay
```

---

### SegNet (Cityscapes)

```yaml
Model: SegNet-Cityscapes
Classes: 19
Input: 640x360
Precision: FP16
Expected FPS: 25
Latency: 40ms
VRAM: ~700MB
Use Case: Alternative segmentation model
```

---

### Custom Segmentation Model (STEPP)

**STEPP (Semantic Segmentation for Edge Deployment):**
- Lightweight architecture optimized for edge devices
- Reduced parameters while maintaining accuracy
- See `jetson-inference/stepp-paper.pdf` for details

**Training Custom Model:**
```python
# PyTorch training example
import torch
import torchvision

model = torchvision.models.segmentation.fcn_resnet50(pretrained=True)
# Fine-tune on custom dataset
# Export to ONNX
torch.onnx.export(model, dummy_input, "custom_segmentation.onnx")

# Convert ONNX to TensorRT
import tensorrt as trt
# ... TensorRT conversion code
```

---

## 3. Vision-Language Models (VLM)

### NanoVLM - **RECOMMENDED FOR REAL-TIME**

```yaml
Model: NanoVLM (optimized for Jetson)
Parameters: ~1-2B
Input: 224x224 (image) + text
Precision: FP16
Expected FPS: 5-10 tokens/sec
Latency: 100-200ms per generation
VRAM: ~2GB
Use Case: Real-time mission intent parsing
```

**Installation:**
```bash
# Clone NanoLLM repository (includes NanoVLM)
cd flyby-f11/docs/vision/repos
git clone https://github.com/dusty-nv/NanoLLM.git
cd NanoLLM

# Install dependencies
./install.sh

# Download model
./download.sh nanovlm
```

**Usage Example:**
```python
from nanovlm import NanoVLM

# Load model
vlm = NanoVLM('nanovlm')

# Visual question answering
image = load_image('/camera/color/image_raw')
question = "What obstacles do you see?"
answer = vlm.generate(image, question)
# Output: "I see a person walking on the sidewalk and a car parked on the street."

# Mission intent parsing
mission_command = "Fly to the red building and land on the roof"
parsed_intent = vlm.parse_mission(image, mission_command)
# Output: {"target": "red building", "action": "land", "location": "roof"}
```

---

### VILA-3B - Higher Accuracy

```yaml
Model: VILA-3B
Parameters: 3B
Input: 336x336 (image) + text
Precision: FP16
Expected FPS: 2-3 tokens/sec
Latency: 333-500ms per generation
VRAM: ~6GB
Use Case: Complex mission planning (offline)
```

**Installation:**
```bash
# Install NVIDIA Jetson Platform Services
# Follow guide in vlm/jetson-platform-services-vlm.html

# Or use Jetson Generative AI Lab
cd flyby-f11/docs/vision/repos
git clone https://github.com/NVIDIA-AI-IOT/jetson-generative-ai-playground.git
cd jetson-generative-ai-playground
./install.sh vila
```

---

### Model Selection Guide

**Use NanoVLM when:**
- Real-time mission command processing required
- Simple visual question answering (VQA)
- Low latency critical (<200ms)
- Running concurrent vision pipelines

**Use VILA-3B when:**
- Complex scene understanding needed
- Offline mission planning
- High accuracy critical
- Single-task inference (no concurrent pipelines)

---

## 4. Sensor-Specific Configurations

### Intel RealSense T265 (Visual Odometry)

```yaml
Camera: Intel RealSense T265
Resolution: 848x800 (fisheye, stereo)
Frame Rate: 30 FPS (visual), 200 Hz (IMU)
Output: 6DOF pose (position + orientation)
Latency: 5-10ms
Topics:
  - /camera/odom/sample (nav_msgs/Odometry, 200 Hz)
  - /camera/imu (sensor_msgs/Imu, 200 Hz)
  - /camera/fisheye1/image_raw (848x800, 30 FPS)
  - /camera/fisheye2/image_raw (848x800, 30 FPS)
```

**ROS 2 Launch Configuration:**
```yaml
# t265_launch.yaml
camera:
  enable_fisheye1: true
  enable_fisheye2: true
  enable_pose: true
  enable_imu: true
  fisheye_width: 848
  fisheye_height: 800
  fisheye_fps: 30
  gyro_fps: 200
  accel_fps: 200
  unite_imu_method: 'linear_interpolation'
  publish_odom_tf: true
```

---

### Intel RealSense D455 (Depth Camera)

```yaml
Camera: Intel RealSense D455
RGB Resolution: 1920x1080
Depth Resolution: 1280x720
Frame Rate: 30 FPS
Range: 0.6m - 20m (optimal: 0.6m - 6m)
FOV: 87° × 58° (RGB), 87° × 58° (depth)
Topics:
  - /camera/color/image_raw (1920x1080 or 640x480, 30 FPS)
  - /camera/depth/image_rect_raw (1280x720 or 640x480, 30 FPS)
  - /camera/aligned_depth_to_color/image_raw (aligned depth)
  - /camera/pointcloud (sensor_msgs/PointCloud2)
```

**ROS 2 Launch Configuration:**
```yaml
# d455_launch.yaml
camera:
  enable_color: true
  enable_depth: true
  color_width: 640
  color_height: 480
  color_fps: 30
  depth_width: 640
  depth_height: 480
  depth_fps: 30
  align_depth: true
  pointcloud.enable: true
  decimation_filter.enable: true
  spatial_filter.enable: true
  temporal_filter.enable: true
```

**Post-Processing Filters:**
```yaml
# Optimize depth quality
decimation_filter:
  filter_magnitude: 2          # Reduce resolution by 2x

spatial_filter:
  smooth_alpha: 0.5           # Edge-preserving smoothing
  smooth_delta: 20
  magnitude: 2
  hole_fill: 0

temporal_filter:
  smooth_alpha: 0.4           # Temporal smoothing
  smooth_delta: 20
  persistence_control: 3
```

---

## 5. Multi-Model Pipeline Configurations

### Pipeline 1: Real-Time Navigation (High Priority)

```yaml
Purpose: Real-time obstacle avoidance and navigation
Total VRAM: ~2.1GB / 16GB (13% utilization)
Total Latency: ~45-55ms

Components:
  1. YOLOv11n (Object Detection):
     - Input: 640x640 from D455 RGB
     - Output: Bounding boxes for obstacles
     - FPS: 80-100
     - Latency: 10-12ms
     - VRAM: 500MB

  2. FCN-ResNet18 (Terrain Classification):
     - Input: 512x256 from D455 RGB (downsampled)
     - Output: Navigable terrain mask
     - FPS: 30
     - Latency: 33ms
     - VRAM: 600MB

  3. RealSense Processing:
     - T265 visual odometry: 200 Hz
     - D455 depth: 30 FPS
     - VRAM: 500MB

  4. ROS 2 + System Overhead: 500MB
```

**ROS 2 Launch File:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RealSense cameras
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='t265',
            parameters=[{'camera_name': 't265', 'enable_pose': True}]
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d455',
            parameters=[{'camera_name': 'd455', 'enable_color': True, 'enable_depth': True}]
        ),

        # Object detection
        Node(
            package='perception_pipeline',
            executable='yolo_detection_node',
            name='yolo_detection',
            parameters=[{'model': 'yolo11n.engine', 'conf_threshold': 0.5}]
        ),

        # Terrain segmentation
        Node(
            package='perception_pipeline',
            executable='terrain_segmentation_node',
            name='terrain_segmentation',
            parameters=[{'model': 'fcn-resnet18-cityscapes'}]
        ),

        # Obstacle avoidance
        Node(
            package='autonomy_core',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance'
        )
    ])
```

---

### Pipeline 2: Mission Planning (Medium Priority)

```yaml
Purpose: Offline mission planning with VLM
Total VRAM: ~7.3GB / 16GB (46% utilization)

Components:
  1. YOLOv11s (Accurate Object Detection):
     - VRAM: 800MB
     - FPS: 50-70
     - Use Case: Scene analysis for mission planning

  2. SegNet (Scene Understanding):
     - VRAM: 700MB
     - FPS: 25
     - Use Case: Terrain and environment classification

  3. NanoVLM (Mission Intent):
     - VRAM: 2GB
     - Tokens/sec: 5-10
     - Use Case: Parse mission commands

  4. RealSense + ROS 2: 1GB

  5. Map Building: 2.8GB
```

---

### Pipeline 3: Maximum Accuracy (Low Priority, Offline Only)

```yaml
Purpose: Post-mission analysis and advanced planning
Total VRAM: ~9.7GB / 16GB (61% utilization)

Components:
  1. YOLOv11m (Maximum Detection Accuracy):
     - VRAM: 1.2GB
     - FPS: 30-40

  2. DeepLabV3+ (Detailed Segmentation):
     - VRAM: 1.5GB
     - FPS: 15

  3. VILA-3B (Advanced VLM):
     - VRAM: 6GB
     - Tokens/sec: 2-3

  4. ROS 2 + System: 1GB
```

---

## 6. TensorRT Conversion Commands

### YOLO Models

```bash
# YOLOv11n (Nano)
python export_tensorrt.py \
    --model yolo11n.pt \
    --precision fp16 \
    --workspace 4 \
    --batch 1 \
    --device 0

# YOLOv11s (Small)
python export_tensorrt.py \
    --model yolo11s.pt \
    --precision fp16 \
    --workspace 4 \
    --batch 1 \
    --device 0

# YOLOv11m (Medium)
python export_tensorrt.py \
    --model yolo11m.pt \
    --precision fp16 \
    --workspace 4 \
    --batch 1 \
    --device 0
```

### Segmentation Models

```bash
# Export PyTorch model to ONNX
python -m torch.onnx.export \
    model.pth \
    custom_segmentation.onnx \
    --input-shape 1 3 512 256

# Convert ONNX to TensorRT
trtexec \
    --onnx=custom_segmentation.onnx \
    --saveEngine=custom_segmentation.engine \
    --fp16 \
    --workspace=4096
```

---

## 7. Benchmarking Commands

### GPU Utilization Monitoring

```bash
# Install jetson-stats
sudo pip install jetson-stats

# Monitor GPU, CPU, memory in real-time
jtop
```

### Model Benchmarking

```bash
# Benchmark YOLOv11n
python benchmark.py \
    --model yolo11n.engine \
    --input-size 640 640 \
    --iterations 100

# Expected output:
# Average FPS: 85.3
# Average Latency: 11.7ms
# Peak Memory: 512MB
```

### ROS 2 Performance Analysis

```bash
# Analyze topic latency
ros2 topic hz /camera/color/image_raw
ros2 topic hz /detections

# Profile node execution
ros2 run performance_test perf_test \
    --topic /detections \
    --msg-count 1000
```

---

## 8. Troubleshooting Common Issues

### Issue: Out of Memory (OOM)

**Symptoms:**
- TensorRT engine fails to build
- Segmentation fault during inference
- System freeze/crash

**Solutions:**
```bash
# 1. Reduce workspace size
--workspace 2  # Instead of 4GB

# 2. Use smaller model
yolo11n.pt  # Instead of yolo11m.pt

# 3. Reduce input size
--imgsz 416  # Instead of 640

# 4. Close other applications
sudo systemctl stop docker  # If not needed

# 5. Increase swap space
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

---

### Issue: Low FPS / High Latency

**Symptoms:**
- Actual FPS < 50% of expected
- Latency > 2x expected

**Solutions:**
```bash
# 1. Verify GPU is being used
jtop  # Check GPU utilization > 90%

# 2. Enable max performance mode
sudo nvpmodel -m 0  # Max performance
sudo jetson_clocks  # Max clocks

# 3. Check thermal throttling
jtop  # Monitor temperature < 80°C

# 4. Optimize TensorRT engine
--fp16  # Use FP16 instead of FP32
--workspace 4  # Larger workspace = more optimization

# 5. Use DLA (Deep Learning Accelerator) if available
--useDLACore=0  # Use DLA core 0
```

---

### Issue: Camera Not Detected

**Symptoms:**
- RealSense camera not showing in `rs-enumerate-devices`
- No image topics published

**Solutions:**
```bash
# 1. Update udev rules
cd /etc/udev/rules.d/
sudo wget https://github.com/IntelRealSense/librealsense/raw/master/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 2. Check USB connection
lsusb | grep Intel  # Should show RealSense devices

# 3. Update firmware
rs-fw-update -l  # List devices
rs-fw-update -f latest.bin  # Update firmware

# 4. Verify permissions
sudo chmod 666 /dev/video*
```

---

## 9. Optimization Checklist

### Pre-Deployment Checklist

- [ ] Models exported to TensorRT with FP16 precision
- [ ] Benchmarked on Jetson Orin NX hardware (not simulation)
- [ ] Memory usage < 80% of available VRAM
- [ ] FPS meets real-time requirements (≥30 FPS)
- [ ] Latency meets control loop requirements (<50ms)
- [ ] ROS 2 topics validated with `ros2 topic hz`
- [ ] Thermal performance verified (temp < 80°C sustained)
- [ ] Power mode set to maximum performance (`nvpmodel -m 0`)
- [ ] Integration tested with full autonomy stack
- [ ] Failsafe mechanisms implemented for model failures

---

## 10. Next Steps

1. **Download all documentation**: Run `download_vision_docs.sh`
2. **Review model configurations**: Choose models for your use case
3. **Set up Jetson environment**: Install TensorRT, CUDA, ROS 2
4. **Benchmark models**: Test on real hardware
5. **Integrate with ROS 2**: Create perception pipeline nodes
6. **Test full system**: Validate with autonomy stack

---

**Maintainer**: Finley Holt
**Last Updated**: 2025-12-25
**Version**: 1.0
