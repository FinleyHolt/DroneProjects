# TensorRT Optimization Guide for Jetson Orin NX

**Platform**: NVIDIA Jetson Orin NX 16GB (50 TOPS)
**TensorRT Version**: 8.5+
**Target Precision**: FP16
**Last Updated**: 2025-12-25

---

## Table of Contents

1. [TensorRT Overview](#tensorrt-overview)
2. [Installation and Setup](#installation-and-setup)
3. [YOLO11 TensorRT Conversion](#yolo11-tensorrt-conversion)
4. [Semantic Segmentation Conversion](#semantic-segmentation-conversion)
5. [Optimization Techniques](#optimization-techniques)
6. [Performance Benchmarking](#performance-benchmarking)
7. [Troubleshooting](#troubleshooting)
8. [Advanced Topics](#advanced-topics)

---

## TensorRT Overview

### What is TensorRT?

NVIDIA TensorRT is a high-performance deep learning inference library that optimizes trained neural networks for production deployment. It provides:

- **Layer & Tensor Fusion**: Combines layers to reduce memory bandwidth
- **Kernel Auto-Tuning**: Selects optimal kernels for your GPU
- **Precision Calibration**: FP16 and INT8 quantization
- **Dynamic Tensor Memory**: Efficient memory allocation
- **Multi-Stream Execution**: Parallel inference streams

### Performance Gains on Jetson Orin NX

| Model | PyTorch FP32 | TensorRT FP16 | Speedup |
|-------|--------------|---------------|---------|
| YOLOv11n | 25 FPS | 85 FPS | 3.4x |
| YOLOv11s | 12 FPS | 60 FPS | 5.0x |
| FCN-ResNet18 | 15 FPS | 30 FPS | 2.0x |
| NanoVLM | 2 tok/s | 8 tok/s | 4.0x |

---

## Installation and Setup

### Prerequisites

```bash
# Verify CUDA installation
nvcc --version
# Expected: CUDA 11.4+ for JetPack 5.x

# Verify TensorRT installation
dpkg -l | grep TensorRT
# Expected: libnvinfer8, python3-libnvinfer

# Check Python bindings
python3 -c "import tensorrt; print(tensorrt.__version__)"
# Expected: 8.5.x or 8.6.x
```

### Install Required Tools

```bash
# Install ONNX tools
pip3 install onnx onnxruntime onnx-graphsurgeon

# Install Ultralytics (includes export utilities)
pip3 install ultralytics

# Install TensorRT utilities
sudo apt-get install tensorrt python3-libnvinfer-dev

# Install PyCUDA for advanced usage
pip3 install pycuda
```

### Verify Installation

```bash
# Test TensorRT Python API
python3 << EOF
import tensorrt as trt
print(f"TensorRT Version: {trt.__version__}")
print(f"Builder available: {trt.Builder is not None}")
EOF

# Test trtexec command-line tool
/usr/src/tensorrt/bin/trtexec --help
```

---

## YOLO11 TensorRT Conversion

### Method 1: Ultralytics Built-in Export (Recommended)

This is the simplest method and handles the entire ONNX → TensorRT pipeline automatically.

```python
from ultralytics import YOLO

# Load pretrained model
model = YOLO('yolo11n.pt')

# Export to TensorRT
model.export(
    format='engine',           # TensorRT engine format
    device=0,                  # GPU device ID
    half=True,                 # FP16 precision
    workspace=4,               # 4GB workspace for optimization
    simplify=True,             # ONNX graph simplification
    dynamic=False,             # Static input shapes (faster)
    batch=1,                   # Batch size
    imgsz=640                  # Input image size
)

# Output: yolo11n.engine
```

**Command-Line Alternative:**
```bash
yolo export model=yolo11n.pt format=engine device=0 half=True workspace=4
```

### Method 2: Manual ONNX → TensorRT Conversion

For advanced users who need fine-grained control:

#### Step 1: Export to ONNX

```python
from ultralytics import YOLO

model = YOLO('yolo11n.pt')
model.export(
    format='onnx',
    opset=12,                  # ONNX opset version
    simplify=True,             # Simplify graph
    dynamic=False,             # Static shapes
    imgsz=640
)
# Output: yolo11n.onnx
```

#### Step 2: Optimize ONNX Graph (Optional)

```python
import onnx
from onnxsim import simplify

# Load ONNX model
model = onnx.load('yolo11n.onnx')

# Simplify graph
model_simp, check = simplify(model)
assert check, "Simplified ONNX model could not be validated"

# Save optimized model
onnx.save(model_simp, 'yolo11n_optimized.onnx')
```

#### Step 3: Convert ONNX to TensorRT

```bash
# Using trtexec command-line tool
/usr/src/tensorrt/bin/trtexec \
    --onnx=yolo11n_optimized.onnx \
    --saveEngine=yolo11n.engine \
    --fp16 \
    --workspace=4096 \
    --verbose \
    --timingCacheFile=timing.cache
```

**Optimization Flags Explained:**
- `--fp16`: Enable FP16 precision (2x memory reduction, 3-5x speedup)
- `--workspace=4096`: Allocate 4GB for optimization (larger = more optimizations)
- `--verbose`: Print detailed build information
- `--timingCacheFile`: Cache kernel timings for faster rebuilds

#### Step 4: Python API Conversion (Advanced)

```python
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def build_engine(onnx_file_path, engine_file_path):
    """Build TensorRT engine from ONNX file."""

    # Create builder and network
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, TRT_LOGGER)

    # Parse ONNX model
    with open(onnx_file_path, 'rb') as model:
        if not parser.parse(model.read()):
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return None

    # Configure builder
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 4 << 30)  # 4GB

    # Enable FP16 mode
    if builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
        print("FP16 mode enabled")

    # Build engine
    print("Building TensorRT engine (this may take several minutes)...")
    serialized_engine = builder.build_serialized_network(network, config)

    # Save engine to file
    with open(engine_file_path, 'wb') as f:
        f.write(serialized_engine)

    print(f"Engine saved to {engine_file_path}")
    return serialized_engine

# Build engine
build_engine('yolo11n.onnx', 'yolo11n.engine')
```

---

### Model Variants and Build Times

| Model | ONNX Size | Engine Size | Build Time | FP16 FPS |
|-------|-----------|-------------|------------|----------|
| YOLOv11n | 5.4 MB | 11 MB | 2-3 min | 85 |
| YOLOv11s | 18 MB | 36 MB | 4-5 min | 60 |
| YOLOv11m | 40 MB | 78 MB | 8-10 min | 35 |
| YOLOv11l | 52 MB | 102 MB | 12-15 min | 22 |

**Note**: Build times on Jetson Orin NX. First build is slower; subsequent rebuilds use cached timings.

---

## Semantic Segmentation Conversion

### FCN-ResNet18 (Cityscapes)

#### Option 1: Using jetson-inference (Pre-optimized)

The `jetson-inference` library provides pre-optimized TensorRT engines:

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/vision/repos/jetson-inference

# Download models
cd tools
./download-models.sh

# Select: FCN-ResNet18-Cityscapes
# Models are automatically TensorRT-optimized during first run
```

#### Option 2: Custom PyTorch Model Export

If you have a custom segmentation model:

```python
import torch
import torchvision

# Load pretrained model
model = torchvision.models.segmentation.fcn_resnet18(pretrained=True)
model.eval()

# Export to ONNX
dummy_input = torch.randn(1, 3, 512, 256).cuda()
torch.onnx.export(
    model,
    dummy_input,
    'fcn_resnet18.onnx',
    opset_version=12,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={
        'input': {0: 'batch_size'},
        'output': {0: 'batch_size'}
    }
)

# Convert to TensorRT
import subprocess
subprocess.run([
    '/usr/src/tensorrt/bin/trtexec',
    '--onnx=fcn_resnet18.onnx',
    '--saveEngine=fcn_resnet18.engine',
    '--fp16',
    '--workspace=4096',
    '--minShapes=input:1x3x512x256',
    '--optShapes=input:1x3x512x256',
    '--maxShapes=input:1x3x512x256'
])
```

### DeepLabV3+ Export

```python
import torch
from torchvision.models.segmentation import deeplabv3_resnet50

# Load model
model = deeplabv3_resnet50(pretrained=True)
model.eval()

# Export to ONNX
dummy_input = torch.randn(1, 3, 512, 512).cuda()
torch.onnx.export(
    model,
    dummy_input,
    'deeplabv3.onnx',
    opset_version=12,
    input_names=['input'],
    output_names=['output']
)

# Convert to TensorRT with explicit shapes
subprocess.run([
    '/usr/src/tensorrt/bin/trtexec',
    '--onnx=deeplabv3.onnx',
    '--saveEngine=deeplabv3.engine',
    '--fp16',
    '--workspace=6144',  # 6GB for larger model
    '--minShapes=input:1x3x512x512',
    '--optShapes=input:1x3x512x512',
    '--maxShapes=input:1x3x512x512'
])
```

---

## Optimization Techniques

### 1. Precision Calibration

#### FP16 Precision (Recommended)

**Pros:**
- 2x memory reduction
- 3-5x inference speedup
- Minimal accuracy loss (<1%)
- Native support on Jetson Orin NX

**Cons:**
- Slight accuracy degradation (typically <0.5% mAP)

```bash
# Enable FP16
trtexec --onnx=model.onnx --saveEngine=model.engine --fp16
```

#### INT8 Precision (Advanced)

**Pros:**
- 4x memory reduction
- 5-10x inference speedup (theoretical)

**Cons:**
- Requires calibration dataset
- Potential accuracy loss (1-3%)
- More complex setup

```python
import tensorrt as trt
import numpy as np

class Int8Calibrator(trt.IInt8EntropyCalibrator2):
    def __init__(self, calibration_images, cache_file):
        trt.IInt8EntropyCalibrator2.__init__(self)
        self.cache_file = cache_file
        self.images = calibration_images
        self.batch_size = 1
        self.current_index = 0

        # Allocate device memory
        self.device_input = cuda.mem_alloc(self.images[0].nbytes)

    def get_batch_size(self):
        return self.batch_size

    def get_batch(self, names):
        if self.current_index >= len(self.images):
            return None

        # Copy image to device
        cuda.memcpy_htod(self.device_input, self.images[self.current_index])
        self.current_index += 1
        return [int(self.device_input)]

    def read_calibration_cache(self):
        if os.path.exists(self.cache_file):
            with open(self.cache_file, 'rb') as f:
                return f.read()

    def write_calibration_cache(self, cache):
        with open(self.cache_file, 'wb') as f:
            f.write(cache)

# Use calibrator during engine build
config.set_flag(trt.BuilderFlag.INT8)
config.int8_calibrator = Int8Calibrator(calibration_images, 'calibration.cache')
```

**Note**: INT8 is typically not recommended for Jetson unless memory is extremely constrained. FP16 provides the best accuracy/performance tradeoff.

---

### 2. Layer Fusion and Kernel Optimization

TensorRT automatically fuses layers during engine build:

**Common Fusions:**
- Conv + BatchNorm + ReLU → Single fused kernel
- Conv + Bias + ReLU → Single kernel
- Pointwise convolutions → Optimized GEMM

**Enable Aggressive Optimizations:**
```bash
trtexec \
    --onnx=model.onnx \
    --saveEngine=model.engine \
    --fp16 \
    --workspace=8192 \  # Larger workspace = more fusion opportunities
    --builderOptimizationLevel=5 \  # Max optimization level
    --timingCacheFile=timing.cache
```

---

### 3. Dynamic vs. Static Shapes

#### Static Shapes (Recommended for UAV)

**Pros:**
- Maximum performance
- Simplified inference
- Better kernel selection

**Cons:**
- Fixed input size
- Requires rebuild for different sizes

```bash
# Static shape optimization
trtexec \
    --onnx=model.onnx \
    --saveEngine=model.engine \
    --fp16 \
    --shapes=input:1x3x640x640  # Fixed shape
```

#### Dynamic Shapes

**Pros:**
- Single engine for multiple input sizes
- Flexible deployment

**Cons:**
- Slower inference (5-15%)
- More complex memory management

```bash
# Dynamic shape optimization
trtexec \
    --onnx=model.onnx \
    --saveEngine=model.engine \
    --fp16 \
    --minShapes=input:1x3x320x320 \
    --optShapes=input:1x3x640x640 \
    --maxShapes=input:1x3x1280x1280
```

**Recommendation**: Use static shapes (640x640) for UAV real-time inference.

---

### 4. Multi-Stream Inference

For processing multiple camera feeds concurrently:

```python
import tensorrt as trt
import pycuda.driver as cuda

class TRTInference:
    def __init__(self, engine_path, num_streams=2):
        # Load engine
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(trt.Logger()).deserialize_cuda_engine(f.read())

        # Create execution contexts (one per stream)
        self.contexts = [self.engine.create_execution_context() for _ in range(num_streams)]

        # Create CUDA streams
        self.streams = [cuda.Stream() for _ in range(num_streams)]

    def infer_async(self, stream_idx, input_data):
        """Asynchronous inference on specific stream."""
        context = self.contexts[stream_idx]
        stream = self.streams[stream_idx]

        # Copy input to device (async)
        cuda.memcpy_htod_async(self.d_input, input_data, stream)

        # Execute inference (async)
        context.execute_async_v2(bindings=self.bindings, stream_handle=stream.handle)

        # Copy output from device (async)
        cuda.memcpy_dtoh_async(self.h_output, self.d_output, stream)

        return stream

# Usage
inference = TRTInference('yolo11n.engine', num_streams=2)

# Process two camera feeds concurrently
stream0 = inference.infer_async(0, camera0_image)
stream1 = inference.infer_async(1, camera1_image)

# Synchronize
stream0.synchronize()
stream1.synchronize()
```

---

## Performance Benchmarking

### Using trtexec

```bash
# Benchmark YOLOv11n
/usr/src/tensorrt/bin/trtexec \
    --loadEngine=yolo11n.engine \
    --iterations=100 \
    --warmUp=500 \
    --duration=10 \
    --useSpinWait \
    --noDataTransfers

# Sample output:
# Average Latency: 11.7 ms
# Throughput: 85.5 FPS
# GPU Utilization: 92%
```

### Custom Benchmarking Script

```python
import time
import numpy as np
from ultralytics import YOLO

def benchmark_model(model_path, num_iterations=100):
    """Benchmark TensorRT model."""

    # Load model
    model = YOLO(model_path)

    # Dummy input
    dummy_img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

    # Warm-up
    for _ in range(10):
        _ = model(dummy_img, verbose=False)

    # Benchmark
    latencies = []
    for _ in range(num_iterations):
        start = time.time()
        results = model(dummy_img, verbose=False)
        latency = (time.time() - start) * 1000  # ms
        latencies.append(latency)

    # Results
    avg_latency = np.mean(latencies)
    std_latency = np.std(latencies)
    fps = 1000 / avg_latency

    print(f"Average Latency: {avg_latency:.2f} ms ± {std_latency:.2f} ms")
    print(f"FPS: {fps:.2f}")
    print(f"Min Latency: {np.min(latencies):.2f} ms")
    print(f"Max Latency: {np.max(latencies):.2f} ms")

# Run benchmark
benchmark_model('yolo11n.engine')
```

### Monitor GPU Metrics

```bash
# Install jetson-stats
sudo pip3 install jetson-stats

# Monitor in real-time
jtop

# Command-line monitoring
tegrastats --interval 500

# Expected metrics for optimal performance:
# - GPU Utilization: >90%
# - Memory Usage: <80% of 16GB
# - Temperature: <75°C
# - Power: 15-25W (Orin NX)
```

---

## Troubleshooting

### Issue: Engine Build Fails with OOM

**Error:**
```
[TRT] Error: Out of memory
```

**Solutions:**

1. **Reduce workspace size:**
```bash
--workspace=2048  # Use 2GB instead of 4GB
```

2. **Use swap space:**
```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

3. **Build on more powerful machine:**
```bash
# Build on desktop GPU, deploy to Jetson
trtexec --onnx=model.onnx --saveEngine=model.engine --fp16 --workspace=8192
# Copy model.engine to Jetson
```

---

### Issue: Inference Slower Than Expected

**Symptoms:**
- FPS < 50% of expected
- High latency variance

**Solutions:**

1. **Enable max performance mode:**
```bash
sudo nvpmodel -m 0  # Max performance profile
sudo jetson_clocks   # Lock clocks to maximum
```

2. **Verify FP16 is enabled:**
```python
import tensorrt as trt

# Load engine and inspect
with open('model.engine', 'rb') as f:
    engine = trt.Runtime(trt.Logger()).deserialize_cuda_engine(f.read())

# Check if FP16 layers exist
inspector = engine.create_engine_inspector()
print(inspector.get_engine_information(trt.LayerInformationFormat.JSON))
```

3. **Check thermal throttling:**
```bash
jtop  # Monitor temperature
# If temp > 80°C, add cooling or reduce workload
```

---

### Issue: Accuracy Degradation After TensorRT Conversion

**Symptoms:**
- mAP drops >2% after FP16 conversion
- Incorrect detections

**Solutions:**

1. **Validate ONNX export:**
```python
import onnx
import onnxruntime as ort

# Load ONNX model
onnx_model = onnx.load('model.onnx')
onnx.checker.check_model(onnx_model)

# Run inference with ONNX Runtime
session = ort.InferenceSession('model.onnx')
# Compare outputs with PyTorch
```

2. **Use calibration for INT8 (if applicable):**
See INT8 calibration section above.

3. **Increase workspace size:**
```bash
--workspace=8192  # Larger workspace allows better optimizations
```

---

## Advanced Topics

### 1. Custom Plugins

For unsupported operations:

```cpp
// Custom TensorRT plugin example
class MyCustomPlugin : public nvinfer1::IPluginV2DynamicExt {
public:
    // Implement required methods
    int32_t enqueue(
        const nvinfer1::PluginTensorDesc* inputDesc,
        const nvinfer1::PluginTensorDesc* outputDesc,
        const void* const* inputs,
        void* const* outputs,
        void* workspace,
        cudaStream_t stream) override {

        // Custom CUDA kernel invocation
        myCustomKernel<<<blocks, threads, 0, stream>>>(inputs, outputs);
        return 0;
    }
};
```

### 2. Engine Serialization and Deployment

```python
# Serialize engine to file
with open('model.engine', 'wb') as f:
    f.write(serialized_engine)

# Deploy: Load engine on target device
with open('model.engine', 'rb') as f:
    engine = trt.Runtime(trt.Logger()).deserialize_cuda_engine(f.read())
```

### 3. Profiling and Layer-Level Analysis

```bash
# Enable profiling
trtexec \
    --loadEngine=model.engine \
    --dumpProfile \
    --dumpLayerInfo \
    --exportProfile=profile.json

# Analyze profile
cat profile.json | jq '.layers[] | select(.time > 5)'  # Layers taking >5ms
```

---

## Quick Reference Commands

```bash
# Export YOLO to TensorRT (Python)
yolo export model=yolo11n.pt format=engine device=0 half=True

# Export YOLO to TensorRT (CLI)
trtexec --onnx=yolo11n.onnx --saveEngine=yolo11n.engine --fp16 --workspace=4096

# Benchmark engine
trtexec --loadEngine=yolo11n.engine --iterations=100

# Monitor GPU
jtop

# Enable max performance
sudo nvpmodel -m 0 && sudo jetson_clocks
```

---

**Maintainer**: Finley Holt
**Last Updated**: 2025-12-25
**Version**: 1.0
