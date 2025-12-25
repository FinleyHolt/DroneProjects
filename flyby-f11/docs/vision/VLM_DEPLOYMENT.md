# Vision-Language Model (VLM) Deployment Guide

**Platform**: NVIDIA Jetson Orin NX 16GB (50 TOPS)
**Models**: NanoVLM (1-2B), VILA-3B, VILA-7B
**Use Case**: Mission intent parsing, visual question answering
**Last Updated**: 2025-12-25

---

## Table of Contents

1. [VLM Overview](#vlm-overview)
2. [Model Selection](#model-selection)
3. [NanoVLM Deployment](#nanolvlm-deployment)
4. [VILA Deployment](#vila-deployment)
5. [ROS 2 Integration](#ros-2-integration)
6. [Memory Management](#memory-management)
7. [Performance Optimization](#performance-optimization)
8. [Use Cases and Examples](#use-cases-and-examples)

---

## VLM Overview

### What are Vision-Language Models?

Vision-Language Models (VLMs) combine computer vision and natural language processing to:
- Answer questions about images
- Generate captions for scenes
- Parse mission commands with visual context
- Perform visual reasoning and grounding

**Example Use Case for UAV:**
```
Image: Aerial view of urban scene
Question: "Where should I land to avoid obstacles?"
VLM Response: "Land on the flat rooftop of the gray building on the left.
              It has no obstructions and is away from the trees."
```

### Jetson-Optimized VLMs

| Model | Parameters | VRAM | Tokens/sec | Latency | Use Case |
|-------|------------|------|------------|---------|----------|
| NanoVLM | 1-2B | ~2GB | 8-12 | 100-150ms | Real-time mission parsing |
| VILA-3B | 3B | ~6GB | 2-4 | 300-500ms | Complex scene understanding |
| VILA-7B | 7B | ~14GB | 1-2 | 800-1200ms | Offline analysis (maxes out memory) |

**Recommendation**: Use **NanoVLM** for real-time operations. Reserve VILA-3B for offline mission planning.

---

## Model Selection

### NanoVLM - Recommended for Real-Time

**Pros:**
- Optimized specifically for Jetson devices
- Low memory footprint (2GB)
- Fast inference (8-12 tokens/sec)
- Good accuracy for simple VQA tasks

**Cons:**
- Limited reasoning capability compared to larger models
- May struggle with complex multi-step reasoning

**Best For:**
- Real-time mission command parsing
- Quick visual question answering
- Running alongside object detection and segmentation
- Battery-powered operations

---

### VILA-3B - Balanced Performance

**Pros:**
- Better reasoning than NanoVLM
- Still fits in memory budget (~6GB)
- Good scene understanding
- Moderate inference speed

**Cons:**
- Slower than NanoVLM (2-4 tok/sec)
- Leaves less memory for concurrent vision tasks

**Best For:**
- Offline mission planning
- Complex scene analysis
- When not running heavy object detection

---

### VILA-7B - Maximum Accuracy (Not Recommended)

**Pros:**
- Best reasoning and accuracy
- Handles complex queries

**Cons:**
- Uses almost all available memory (14GB/16GB)
- Very slow (1-2 tok/sec)
- Not practical for concurrent vision tasks

**Best For:**
- Post-mission analysis on desktop GPU
- Not recommended for Jetson deployment

---

## NanoVLM Deployment

### Installation

```bash
# Clone NanoLLM repository (includes NanoVLM)
cd /home/finley/Github/DroneProjects/flyby-f11/docs/vision/repos
git clone --depth 1 https://github.com/dusty-nv/NanoLLM.git
cd NanoLLM

# Install dependencies
pip3 install -r requirements.txt

# Build NanoLLM (includes TensorRT optimizations)
bash install.sh

# Download NanoVLM model
python3 -m nano_llm.download --model nano_vlm
# Model will be saved to ~/.cache/nano_llm/models/nano_vlm
```

### Verify Installation

```bash
# Test VLM with sample image
python3 << EOF
from nano_llm import NanoVLM

# Load model
vlm = NanoVLM('nano_vlm')

# Test inference
from PIL import Image
img = Image.open('test_image.jpg')
response = vlm.generate(img, "What do you see in this image?")
print(response)
EOF
```

### Basic Usage

```python
from nano_llm import NanoVLM
from PIL import Image

# Initialize model
vlm = NanoVLM(
    model='nano_vlm',
    quantization='fp16',  # FP16 for best speed/accuracy tradeoff
    max_context_len=512,   # Maximum tokens in context
    max_new_tokens=128     # Maximum tokens to generate
)

# Load image
image = Image.open('/path/to/image.jpg')

# Visual question answering
question = "What obstacles do you see in this image?"
answer = vlm.generate(image, question)
print(f"Answer: {answer}")

# Mission command parsing
mission_cmd = "Fly to the red building and land on the roof"
parsed = vlm.generate(image, f"Parse this command: {mission_cmd}")
print(f"Parsed: {parsed}")
```

### Advanced Configuration

```python
vlm = NanoVLM(
    model='nano_vlm',
    quantization='fp16',

    # Generation parameters
    max_new_tokens=128,
    temperature=0.7,        # Randomness (0=deterministic, 1=creative)
    top_p=0.9,              # Nucleus sampling
    repetition_penalty=1.1, # Penalize repetition

    # Performance
    num_beams=1,            # Beam search (1=greedy, >1=beam search)
    do_sample=True          # Enable sampling
)
```

---

## VILA Deployment

### Installation via Jetson Generative AI Lab

```bash
# Clone Jetson AI Lab
cd /home/finley/Github/DroneProjects/flyby-f11/docs/vision/repos
git clone --depth 1 https://github.com/dusty-nv/jetson-containers.git
cd jetson-containers

# Pull VILA container (includes pre-optimized models)
sudo docker pull dustynv/vila:r36.2.0

# Run container
sudo docker run -it --runtime nvidia --network host \
    -v /home/finley/Github/DroneProjects/flyby-f11:/workspace \
    dustynv/vila:r36.2.0
```

### Download and Optimize VILA Models

```bash
# Inside Docker container or native environment
pip3 install huggingface_hub transformers

# Download VILA-3B
python3 << EOF
from huggingface_hub import snapshot_download

model_path = snapshot_download(
    repo_id="Efficient-Large-Model/VILA1.5-3b",
    cache_dir="/home/finley/.cache/huggingface"
)
print(f"Model downloaded to: {model_path}")
EOF

# Export to TensorRT-LLM (for optimal performance)
python3 /opt/NanoLLM/nano_llm/vision/vila/export.py \
    --model Efficient-Large-Model/VILA1.5-3b \
    --output /workspace/models/vila-3b-trt \
    --quantization fp16
```

### Basic VILA Usage

```python
from nano_llm import VisionLanguageModel
from PIL import Image

# Load VILA-3B
vlm = VisionLanguageModel(
    model='Efficient-Large-Model/VILA1.5-3b',
    quantization='fp16'
)

# Load image
image = Image.open('aerial_view.jpg')

# Complex reasoning
prompt = """
You are an autonomous UAV navigation assistant. Analyze this aerial image and:
1. Identify safe landing zones
2. Detect potential obstacles
3. Assess terrain suitability
4. Recommend optimal landing location with reasoning
"""

response = vlm.generate(image, prompt, max_new_tokens=256)
print(response)
```

---

## ROS 2 Integration

### Create VLM ROS 2 Node

Create `/home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src/perception_pipeline/perception_pipeline/vlm_node.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from nano_llm import NanoVLM
from PIL import Image as PILImage
import numpy as np

class VLMNode(Node):
    def __init__(self):
        super().__init__('vlm_node')

        # Parameters
        self.declare_parameter('model', 'nano_vlm')
        self.declare_parameter('quantization', 'fp16')
        model_name = self.get_parameter('model').value
        quant = self.get_parameter('quantization').value

        # Initialize VLM
        self.get_logger().info(f'Loading VLM model: {model_name}')
        self.vlm = NanoVLM(model=model_name, quantization=quant)
        self.get_logger().info('VLM model loaded successfully')

        # ROS 2 interface
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.query_sub = self.create_subscription(
            String,
            '/vlm/query',
            self.query_callback,
            10
        )
        self.response_pub = self.create_publisher(String, '/vlm/response', 10)

        # State
        self.latest_image = None

    def image_callback(self, msg):
        """Store latest image for VLM queries."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.latest_image = PILImage.fromarray(cv_image)

    def query_callback(self, msg):
        """Process VLM query."""
        if self.latest_image is None:
            self.get_logger().warning('No image available for VLM query')
            return

        query = msg.data
        self.get_logger().info(f'Processing query: {query}')

        # Run VLM inference
        response = self.vlm.generate(self.latest_image, query)

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)
        self.get_logger().info(f'VLM response: {response}')

def main():
    rclpy.init()
    node = VLMNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Launch File

Create `/home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src/perception_pipeline/launch/vlm.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_pipeline',
            executable='vlm_node',
            name='vlm_node',
            parameters=[{
                'model': 'nano_vlm',
                'quantization': 'fp16'
            }],
            output='screen'
        )
    ])
```

### Usage Example

```bash
# Terminal 1: Launch VLM node
ros2 launch perception_pipeline vlm.launch.py

# Terminal 2: Send query
ros2 topic pub --once /vlm/query std_msgs/String \
    "{data: 'What obstacles are visible in the current view?'}"

# Terminal 3: Listen for response
ros2 topic echo /vlm/response
```

---

## Memory Management

### Memory Budget Planning

**Total Available**: 16GB

**Typical Allocation:**
```
ROS 2 + System:          1.0 GB
T265 + D455:             0.5 GB
YOLO11n (TensorRT):      0.5 GB
FCN-ResNet18:            0.6 GB
NanoVLM:                 2.0 GB
PX4 Interface:           0.1 GB
Prolog Runtime:          0.1 GB
Map Building:            2.0 GB
-----------------------------------
Total Used:              6.8 GB
Available:               9.2 GB (buffer for spikes)
```

### Monitor Memory Usage

```bash
# Real-time monitoring with jtop
jtop

# Check VLM memory usage
nvidia-smi

# ROS 2 node memory profiling
ros2 run performance_test perf_test \
    --node /vlm_node \
    --duration 60
```

### Reduce Memory Footprint

1. **Use INT8 quantization (if accuracy permits):**
```python
vlm = NanoVLM(model='nano_vlm', quantization='int8')
# Memory: ~2GB → ~1GB
# Speed: Faster
# Accuracy: Slightly reduced
```

2. **Reduce context length:**
```python
vlm = NanoVLM(
    model='nano_vlm',
    max_context_len=256,   # Reduced from 512
    max_new_tokens=64      # Reduced from 128
)
```

3. **Lazy loading (load VLM only when needed):**
```python
class VLMNode(Node):
    def __init__(self):
        self.vlm = None  # Don't load immediately

    def query_callback(self, msg):
        if self.vlm is None:
            self.vlm = NanoVLM('nano_vlm')  # Load on first query
        # Process query...
```

---

## Performance Optimization

### Batching (if processing multiple images)

```python
# Process multiple images in batch
images = [img1, img2, img3]
queries = ["Query 1", "Query 2", "Query 3"]

# Batch inference (faster than sequential)
responses = vlm.generate_batch(images, queries)
```

### Caching Frequent Queries

```python
from functools import lru_cache

class CachedVLM:
    def __init__(self, model='nano_vlm'):
        self.vlm = NanoVLM(model)

    @lru_cache(maxsize=128)
    def cached_generate(self, image_hash, query):
        """Cache responses for identical image+query pairs."""
        return self.vlm.generate(image, query)

    def generate(self, image, query):
        # Hash image for caching
        import hashlib
        img_bytes = image.tobytes()
        img_hash = hashlib.md5(img_bytes).hexdigest()

        return self.cached_generate(img_hash, query)
```

### TensorRT-LLM Optimization (Advanced)

For maximum performance, convert to TensorRT-LLM:

```bash
# Install TensorRT-LLM
pip3 install tensorrt_llm

# Export NanoVLM to TensorRT-LLM
python3 /opt/NanoLLM/nano_llm/export_trt_llm.py \
    --model nano_vlm \
    --output /workspace/models/nano_vlm_trt \
    --quantization fp16 \
    --max_batch_size 1
```

Expected speedup: 1.5-2x faster than standard inference.

---

## Use Cases and Examples

### 1. Mission Command Parsing

**Scenario**: UAV receives natural language mission command

```python
from nano_llm import NanoVLM
from PIL import Image

vlm = NanoVLM('nano_vlm')
image = Image.open('current_view.jpg')

# Mission command
command = "Fly to the tallest building and inspect its roof for damage"

# Parse with visual context
prompt = f"""
Parse this mission command with visual context:
Command: "{command}"

Provide structured output:
- Target: <description>
- Action: <action to perform>
- Parameters: <any specific parameters>
"""

response = vlm.generate(image, prompt)
print(response)

# Example output:
# Target: The tallest building (gray high-rise on the right)
# Action: Navigate to roof, perform inspection
# Parameters: Look for cracks, missing tiles, structural damage
```

### 2. Dynamic Obstacle Avoidance

**Scenario**: Identify safe flight paths

```python
image = Image.open('forward_view.jpg')

query = """
Identify obstacles in this image that a drone should avoid.
For each obstacle, provide:
1. Description
2. Approximate position (left/center/right, near/far)
3. Risk level (high/medium/low)
"""

response = vlm.generate(image, query)

# Example output:
# 1. Tree branches - Center, Near - HIGH RISK
# 2. Power lines - Right, Far - MEDIUM RISK
# 3. Bird - Left, Near - LOW RISK (moving)
```

### 3. Landing Zone Selection

**Scenario**: Find safe landing area

```python
image = Image.open('aerial_view.jpg')

query = """
Analyze this aerial view and recommend the safest landing zone for a 50cm diameter drone.
Consider:
- Flat surfaces
- Obstacle clearance
- Terrain type
Provide top 3 options with reasoning.
"""

response = vlm.generate(image, query)

# Example output:
# 1. BEST: Concrete parking lot (center-right)
#    - Completely flat
#    - No overhead obstacles
#    - Sufficient clearance (5m radius)
#
# 2. GOOD: Grass field (left)
#    - Mostly flat, slight slope
#    - Clear of trees
#    - Soft landing surface
#
# 3. ACCEPTABLE: Building rooftop (top-center)
#    - Flat surface
#    - HVAC units on edges (avoid)
#    - Limited space (2m x 3m)
```

### 4. Semantic Grounding for Object Detection

**Scenario**: Enhance YOLO detections with semantic understanding

```python
from ultralytics import YOLO
yolo = YOLO('yolo11n.engine')
vlm = NanoVLM('nano_vlm')

# Get YOLO detections
image = Image.open('scene.jpg')
detections = yolo(image)

# Enhance with VLM semantic understanding
prompt = f"""
YOLO detected the following objects:
{[det.class_name for det in detections]}

Provide semantic context:
1. What is the overall scene? (urban, rural, indoor, etc.)
2. What are the objects doing? (static, moving, interacting)
3. Any safety concerns for a drone?
"""

semantic_context = vlm.generate(image, prompt)
print(semantic_context)
```

### 5. Communications-Denied Operations

**Scenario**: Parse mission updates from visual signals

```python
image = Image.open('ground_signal.jpg')

query = """
This image shows a ground operator using visual signals.
Decode the signal and determine mission intent:
- Hand gestures
- Flag colors/patterns
- Ground markings
"""

response = vlm.generate(image, query)

# Example output:
# Signal detected: Operator pointing northeast with right arm extended
# Flag: Orange (caution/slow approach)
# Ground marking: "LZ2" painted in white (Landing Zone 2)
# Interpreted intent: Proceed to Landing Zone 2, approach with caution
```

---

## Troubleshooting

### Issue: OOM (Out of Memory)

**Symptoms:**
```
RuntimeError: CUDA out of memory
```

**Solutions:**

1. **Use smaller model:**
```python
vlm = NanoVLM('nano_vlm')  # Instead of VILA-3B
```

2. **Reduce context length:**
```python
vlm = NanoVLM('nano_vlm', max_context_len=256, max_new_tokens=64)
```

3. **Unload other models:**
```python
# Temporarily unload YOLO or segmentation
del yolo_model
torch.cuda.empty_cache()
```

---

### Issue: Slow Inference

**Symptoms:**
- Tokens/sec < 50% of expected

**Solutions:**

1. **Enable max performance:**
```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

2. **Use TensorRT-LLM:**
Export to TensorRT-LLM for 2x speedup (see optimization section).

3. **Reduce output length:**
```python
vlm.generate(image, query, max_new_tokens=32)  # Instead of 128
```

---

### Issue: Poor Accuracy

**Symptoms:**
- VLM provides incorrect or nonsensical answers

**Solutions:**

1. **Improve prompt engineering:**
```python
# Bad prompt:
"What is this?"

# Good prompt:
"Describe the scene in this aerial image. Focus on obstacles, terrain type,
and safe landing zones. Provide specific details about object locations."
```

2. **Use VILA-3B for complex reasoning:**
```python
vlm = VisionLanguageModel('Efficient-Large-Model/VILA1.5-3b')
```

3. **Adjust generation parameters:**
```python
vlm.generate(
    image,
    query,
    temperature=0.3,      # More deterministic (0.1-0.5 for factual responses)
    top_p=0.85,
    repetition_penalty=1.2
)
```

---

## Quick Reference

### Install NanoVLM
```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/vision/repos/NanoLLM
bash install.sh
python3 -m nano_llm.download --model nano_vlm
```

### Basic Inference
```python
from nano_llm import NanoVLM
vlm = NanoVLM('nano_vlm')
response = vlm.generate(image, "What do you see?")
```

### ROS 2 Integration
```bash
ros2 launch perception_pipeline vlm.launch.py
ros2 topic pub --once /vlm/query std_msgs/String "{data: 'Your question'}"
ros2 topic echo /vlm/response
```

### Monitor Memory
```bash
jtop  # GPU memory usage
nvidia-smi  # Detailed GPU stats
```

---

**Maintainer**: Finley Holt
**Last Updated**: 2025-12-25
**Version**: 1.0
