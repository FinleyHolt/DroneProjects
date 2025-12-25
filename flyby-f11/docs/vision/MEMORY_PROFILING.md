# Memory Profiling and Optimization Guide

**Platform**: NVIDIA Jetson Orin NX 16GB (50 TOPS)
**Total Memory**: 16GB unified memory (shared CPU/GPU)
**Target Budget**: Perception pipeline + autonomy stack < 14GB (leaving 2GB buffer)
**Last Updated**: 2025-12-25

---

## Table of Contents

1. [Memory Architecture](#memory-architecture)
2. [Monitoring Tools](#monitoring-tools)
3. [Memory Budget Planning](#memory-budget-planning)
4. [Profiling Techniques](#profiling-techniques)
5. [Optimization Strategies](#optimization-strategies)
6. [Common Memory Issues](#common-memory-issues)

---

## Memory Architecture

### Jetson Orin NX Memory Characteristics

**Unified Memory Architecture:**
- 16GB LPDDR5 (204.8 GB/s bandwidth)
- Shared between CPU and GPU
- No discrete GPU memory pool
- Dynamic allocation based on workload

**Memory Pools:**
```
Total: 16GB
├─ Kernel/System: ~500MB (reserved)
├─ Desktop/GUI: ~300MB (if running)
├─ Available: ~15.2GB
    ├─ CPU allocations
    ├─ GPU allocations (CUDA)
    └─ Shared buffers
```

**Key Differences from Desktop GPUs:**
- **Pros**: No CPU↔GPU memory transfers needed
- **Cons**: Limited total memory (16GB vs 24GB+ on desktop)
- **Implication**: Must carefully manage concurrent models

---

## Monitoring Tools

### 1. jtop (Recommended for Real-Time)

**Installation:**
```bash
sudo pip3 install jetson-stats
sudo systemctl restart jtop.service
```

**Usage:**
```bash
# Interactive dashboard
jtop

# Key metrics to watch:
# - RAM: Total system memory usage
# - GPU: GPU utilization %
# - VRAM: GPU memory usage (MB/GB)
# - CPU: CPU usage per core
# - Temp: Temperature (should stay < 80°C)
# - Power: Power consumption (W)
```

**Screenshot of jtop:**
```
┌─────────────────────────────────────────────────────┐
│ RAM: 8.2GB / 15.4GB (53%)                          │
│ GPU: 92% @ 1300MHz                                 │
│ VRAM: 6.8GB / 15.4GB (44%)                         │
│ CPU: [████████░░] 80% @ 2.0GHz                     │
│ Temp: GPU 72°C, CPU 68°C                           │
│ Power: 18.5W / 25W                                 │
└─────────────────────────────────────────────────────┘
```

---

### 2. nvidia-smi (GPU-Specific)

**Usage:**
```bash
# Current snapshot
nvidia-smi

# Continuous monitoring (update every 1 second)
watch -n 1 nvidia-smi

# Save log to file
nvidia-smi --query-gpu=timestamp,memory.used,memory.free,utilization.gpu \
           --format=csv -l 1 > gpu_log.csv
```

**Sample Output:**
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.54.03    Driver Version: 535.54.03    CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  Orin NX            On   | 00000000:00:00.0 Off |                  N/A |
| N/A   72C    P0    18W /  25W |   6823MiB / 15759MiB |     92%      Default |
+-------------------------------+----------------------+----------------------+
```

---

### 3. tegrastats (System-Level)

**Usage:**
```bash
# Real-time stats
tegrastats

# Log to file with interval (ms)
tegrastats --interval 500 --logfile stats.log

# Stop logging
pkill tegrastats
```

**Sample Output:**
```
RAM 8234/15759MB (lfb 128x4MB) SWAP 0/7879MB (cached 0MB) CPU [80%@2035,75%@2035,70%@2035,85%@2035,90%@2035,65%@2035]
EMC_FREQ 0%@2133 GR3D_FREQ 92%@1300 VIC_FREQ 729 APE 150 MTS fg 0% bg 0% AO@47C GPU@72C PMIC@50C AUX@46C CPU@68C thermal@48C VDD_IN 18500/18500 VDD_CPU_GPU_CV 6750/6750 VDD_SOC 3125/3125
```

---

### 4. ROS 2 Memory Profiling

**Using ros2 CLI:**
```bash
# Node memory usage
ros2 node info /vlm_node

# Topic bandwidth (affects buffer memory)
ros2 topic bw /camera/color/image_raw
```

**Custom Memory Monitor Node:**

Create `memory_monitor_node.py`:
```python
import rclpy
from rclpy.node import Node
import psutil
import subprocess

class MemoryMonitor(Node):
    def __init__(self):
        super().__init__('memory_monitor')
        self.timer = self.create_timer(1.0, self.monitor_callback)

    def monitor_callback(self):
        # CPU/RAM usage
        ram_percent = psutil.virtual_memory().percent
        ram_used_gb = psutil.virtual_memory().used / (1024**3)

        # GPU usage (via nvidia-smi)
        try:
            result = subprocess.run(
                ['nvidia-smi', '--query-gpu=memory.used,memory.total',
                 '--format=csv,noheader,nounits'],
                capture_output=True, text=True
            )
            gpu_mem = result.stdout.strip().split(',')
            gpu_used = int(gpu_mem[0])
            gpu_total = int(gpu_mem[1])
            gpu_percent = (gpu_used / gpu_total) * 100
        except:
            gpu_used = 0
            gpu_percent = 0

        self.get_logger().info(
            f"RAM: {ram_used_gb:.1f}GB ({ram_percent:.1f}%) | "
            f"GPU: {gpu_used}MB ({gpu_percent:.1f}%)"
        )

        # Alert if memory exceeds threshold
        if gpu_percent > 80:
            self.get_logger().warning(f"GPU memory critically high: {gpu_percent:.1f}%")

def main():
    rclpy.init()
    node = MemoryMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Launch:**
```bash
ros2 run perception_pipeline memory_monitor_node
```

---

### 5. PyTorch Memory Profiling

**For PyTorch-based models:**

```python
import torch

# Enable memory tracking
torch.cuda.memory._record_memory_history(enabled=True)

# Run inference
model = load_model()
output = model(input_tensor)

# Print memory summary
print(torch.cuda.memory_summary())

# Detailed allocation history
snapshot = torch.cuda.memory._snapshot()
for entry in snapshot:
    print(f"{entry['name']}: {entry['size'] / 1024**2:.2f} MB")

# Reset peak memory stats
torch.cuda.reset_peak_memory_stats()
```

---

## Memory Budget Planning

### Current System Allocation (Example)

| Component | Memory Usage | Notes |
|-----------|--------------|-------|
| **System/Kernel** | 500 MB | OS, drivers |
| **ROS 2 Core** | 200 MB | ROS daemon, nodes |
| **PX4 Interface** | 100 MB | MAVSDK, MAVLink |
| **T265 Driver** | 150 MB | Visual odometry |
| **D455 Driver** | 350 MB | RGB, depth, filters |
| **YOLO11n (TRT)** | 500 MB | Object detection |
| **FCN-ResNet18** | 600 MB | Segmentation |
| **NanoVLM** | 2000 MB | Vision-language model |
| **Prolog Runtime** | 100 MB | Symbolic reasoning |
| **Map Building** | 2000 MB | OctoMap/grid map |
| **ROS Buffers** | 500 MB | Topic queues |
| **Misc/Overhead** | 1000 MB | Buffer for spikes |
| **TOTAL** | **8.0 GB** | **50% utilization** |
| **Available** | **8.0 GB** | **Safe buffer** |

**Verdict**: Healthy configuration with room for growth.

---

### Tight Budget Configuration (Maximum Models)

| Component | Memory Usage | Notes |
|-----------|--------------|-------|
| System/ROS/PX4 | 800 MB | Baseline |
| T265 + D455 | 500 MB | Sensors |
| YOLO11s (TRT) | 800 MB | Higher accuracy |
| SegNet | 700 MB | Segmentation |
| VILA-3B | 6000 MB | Larger VLM |
| Prolog Runtime | 100 MB | Reasoning |
| Map Building | 3000 MB | Larger maps |
| Buffers/Overhead | 1100 MB | Minimal buffer |
| **TOTAL** | **13.0 GB** | **81% utilization** |
| **Available** | **3.0 GB** | **Tight but viable** |

**Verdict**: Possible but risky. Monitor closely for OOM events.

---

### Maximum Safe Allocation

**Rule of Thumb**: Keep total usage < 85% (13.6GB) to avoid OOM crashes.

**Recommended Buffer**: 2GB minimum for:
- Peak memory spikes during model loading
- ROS message queues during bursts
- Temporary allocations during processing

---

## Profiling Techniques

### 1. Baseline Memory Footprint

**Measure system idle state:**
```bash
# Reboot Jetson
sudo reboot

# Wait for boot, then measure
jtop  # Note "RAM" and "VRAM" usage

# Typical baseline: 800MB - 1.5GB
```

---

### 2. Per-Model Memory Profiling

**Measure individual model footprints:**

```python
import torch
import numpy as np
from ultralytics import YOLO

def measure_model_memory(model_path):
    """Measure memory footprint of a model."""

    # Clear GPU memory
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()

    # Record baseline
    baseline = torch.cuda.memory_allocated()
    print(f"Baseline GPU memory: {baseline / 1024**2:.1f} MB")

    # Load model
    model = YOLO(model_path)

    # Record after load
    after_load = torch.cuda.memory_allocated()
    model_size = (after_load - baseline) / 1024**2
    print(f"Model memory: {model_size:.1f} MB")

    # Run inference to measure peak
    dummy_input = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
    _ = model(dummy_input)

    # Peak memory
    peak = torch.cuda.max_memory_allocated()
    peak_mb = peak / 1024**2
    print(f"Peak GPU memory: {peak_mb:.1f} MB")

    return {
        'model_size': model_size,
        'peak_memory': peak_mb
    }

# Test YOLO models
results = {}
for model in ['yolo11n.engine', 'yolo11s.engine', 'yolo11m.engine']:
    print(f"\n--- Testing {model} ---")
    results[model] = measure_model_memory(model)
    torch.cuda.empty_cache()  # Clean up

# Summary
print("\n=== Summary ===")
for model, stats in results.items():
    print(f"{model}: {stats['peak_memory']:.1f} MB peak")
```

**Expected Results:**
```
yolo11n.engine: 512 MB peak
yolo11s.engine: 823 MB peak
yolo11m.engine: 1245 MB peak
```

---

### 3. Full Pipeline Profiling

**Measure complete perception pipeline:**

```python
import torch
import time
from ultralytics import YOLO
from nano_llm import NanoVLM

def profile_full_pipeline():
    """Profile full perception pipeline."""

    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()

    print("Loading models...")

    # Load YOLO
    start = time.time()
    yolo = YOLO('yolo11n.engine')
    yolo_time = time.time() - start
    yolo_mem = torch.cuda.memory_allocated() / 1024**2
    print(f"YOLO loaded: {yolo_time:.1f}s, {yolo_mem:.1f} MB")

    # Load VLM
    start = time.time()
    vlm = NanoVLM('nano_vlm')
    vlm_time = time.time() - start
    total_mem = torch.cuda.memory_allocated() / 1024**2
    vlm_mem = total_mem - yolo_mem
    print(f"VLM loaded: {vlm_time:.1f}s, {vlm_mem:.1f} MB")

    # Run inference
    dummy_img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

    start = time.time()
    yolo_results = yolo(dummy_img)
    yolo_infer_time = (time.time() - start) * 1000  # ms

    start = time.time()
    vlm_response = vlm.generate(dummy_img, "What do you see?")
    vlm_infer_time = (time.time() - start) * 1000  # ms

    peak_mem = torch.cuda.max_memory_allocated() / 1024**2

    print("\n=== Performance Summary ===")
    print(f"Total memory: {peak_mem:.1f} MB")
    print(f"YOLO inference: {yolo_infer_time:.1f} ms")
    print(f"VLM inference: {vlm_infer_time:.1f} ms")

profile_full_pipeline()
```

---

### 4. Runtime Memory Tracking

**Log memory usage during live operation:**

```python
import threading
import time
import torch

class MemoryTracker:
    def __init__(self, interval=1.0):
        self.interval = interval
        self.running = False
        self.thread = None
        self.log = []

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._track)
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def _track(self):
        while self.running:
            mem_used = torch.cuda.memory_allocated() / 1024**2
            mem_reserved = torch.cuda.memory_reserved() / 1024**2
            timestamp = time.time()

            self.log.append({
                'time': timestamp,
                'used': mem_used,
                'reserved': mem_reserved
            })

            time.sleep(self.interval)

    def plot(self):
        import matplotlib.pyplot as plt
        times = [entry['time'] - self.log[0]['time'] for entry in self.log]
        used = [entry['used'] for entry in self.log]
        reserved = [entry['reserved'] for entry in self.log]

        plt.figure(figsize=(10, 5))
        plt.plot(times, used, label='Used Memory')
        plt.plot(times, reserved, label='Reserved Memory')
        plt.xlabel('Time (s)')
        plt.ylabel('Memory (MB)')
        plt.title('GPU Memory Usage Over Time')
        plt.legend()
        plt.grid()
        plt.savefig('memory_usage.png')
        print("Plot saved to memory_usage.png")

# Usage
tracker = MemoryTracker(interval=0.5)
tracker.start()

# Run your perception pipeline...
# ... (autonomous operation)

tracker.stop()
tracker.plot()
```

---

## Optimization Strategies

### 1. Model Quantization

**FP16 → INT8 Conversion:**

```python
from ultralytics import YOLO

# Export to INT8 (requires calibration dataset)
model = YOLO('yolo11n.pt')
model.export(
    format='engine',
    device=0,
    int8=True,  # Enable INT8
    data='coco128.yaml'  # Calibration dataset
)

# Memory reduction: ~50% (500MB → 250MB)
# Speed improvement: 10-20%
# Accuracy loss: ~1-2% mAP
```

**When to Use INT8:**
- Memory is critically constrained
- Running many concurrent models
- Acceptable to lose 1-2% accuracy

**When to Avoid INT8:**
- FP16 already fits in budget
- Accuracy is critical
- Calibration dataset unavailable

---

### 2. Model Pruning

**Reduce model parameters:**

```python
import torch
import torch.nn.utils.prune as prune

def prune_model(model, amount=0.3):
    """Prune model weights by specified amount."""

    for name, module in model.named_modules():
        if isinstance(module, torch.nn.Conv2d):
            prune.l1_unstructured(module, name='weight', amount=amount)
            prune.remove(module, 'weight')  # Make pruning permanent

    return model

# Prune 30% of weights
pruned_model = prune_model(model, amount=0.3)

# Memory reduction: ~30%
# Speed improvement: 10-20%
# Accuracy loss: ~2-5% (retrain to recover)
```

---

### 3. Lazy Loading

**Load models only when needed:**

```python
class LazyVLM:
    def __init__(self):
        self._vlm = None

    def generate(self, image, query):
        # Load VLM on first query
        if self._vlm is None:
            print("Loading VLM...")
            self._vlm = NanoVLM('nano_vlm')

        return self._vlm.generate(image, query)

    def unload(self):
        """Free memory when VLM not needed."""
        if self._vlm is not None:
            del self._vlm
            torch.cuda.empty_cache()
            self._vlm = None

# Usage
vlm = LazyVLM()

# Only loads when first used
response = vlm.generate(img, "What's this?")

# Free memory when done
vlm.unload()
```

---

### 4. Reduce Input Resolution

**Trade resolution for memory:**

```python
# Original: 1920x1080 RGB
# Memory: ~6MB per frame
# Buffer (10 frames): 60MB

# Downscaled: 640x480 RGB
# Memory: ~1MB per frame
# Buffer (10 frames): 10MB
# Savings: 50MB (83% reduction)

import cv2

def downsample_image(image, target_size=(640, 480)):
    """Downsample image to reduce memory."""
    return cv2.resize(image, target_size, interpolation=cv2.INTER_LINEAR)
```

---

### 5. Memory Pool Pre-Allocation

**Avoid fragmentation with pre-allocated pools:**

```python
import torch

# Pre-allocate memory pool at startup
def preallocate_memory_pool(size_mb=1024):
    """Pre-allocate GPU memory to avoid fragmentation."""
    pool_size = size_mb * 1024 * 1024  # Convert to bytes
    dummy_tensor = torch.empty(pool_size // 4, dtype=torch.float32, device='cuda')
    del dummy_tensor
    torch.cuda.empty_cache()

# Run at startup
preallocate_memory_pool(size_mb=2048)  # Reserve 2GB pool
```

---

## Common Memory Issues

### Issue 1: CUDA Out of Memory (OOM)

**Error:**
```
RuntimeError: CUDA out of memory. Tried to allocate 1024.00 MiB
(GPU 0; 15.37 GiB total capacity; 14.12 GiB already allocated)
```

**Solutions:**

1. **Identify memory hog:**
```bash
# Monitor while reproducing error
jtop  # Watch which component spikes
```

2. **Reduce batch size:**
```python
# Before: batch_size = 4
# After:
batch_size = 1
```

3. **Clear cache regularly:**
```python
import torch
import gc

# After inference
torch.cuda.empty_cache()
gc.collect()
```

4. **Use gradient checkpointing (training only):**
```python
model.gradient_checkpointing_enable()
```

---

### Issue 2: Memory Leak

**Symptoms:**
- Memory usage grows over time
- Eventually crashes after hours

**Diagnosis:**
```python
import tracemalloc

tracemalloc.start()

# Run your code...

snapshot = tracemalloc.take_snapshot()
top_stats = snapshot.statistics('lineno')

for stat in top_stats[:10]:
    print(stat)
```

**Common Causes:**
- Unclosed file handles
- Accumulating ROS messages
- TensorFlow/PyTorch graph accumulation

**Solutions:**
```python
# Close file handles
with open('file.txt', 'r') as f:
    data = f.read()
# File auto-closed

# Clear ROS message queues
subscriber.reset()  # Clear old messages

# PyTorch: Detach tensors
output = model(input).detach()  # Remove from computation graph
```

---

### Issue 3: Swap Thrashing

**Symptoms:**
- System becomes very slow
- High disk I/O

**Diagnosis:**
```bash
# Check swap usage
free -h

# Expected:
#               total        used        free
# Mem:           15Gi       8.2Gi       7.1Gi
# Swap:         7.9Gi          0B       7.9Gi  # Should be 0 or low
```

**Solutions:**

1. **Reduce memory usage** (see optimization strategies)

2. **Disable swap** (if performance critical):
```bash
sudo swapoff -a
```

3. **Increase swap** (if OOM is issue):
```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

---

## Quick Reference

### Monitor GPU Memory
```bash
# Real-time dashboard
jtop

# Command-line snapshot
nvidia-smi

# System stats
tegrastats
```

### Clear GPU Cache
```python
import torch
torch.cuda.empty_cache()
```

### Measure Model Memory
```python
torch.cuda.reset_peak_memory_stats()
# Run inference
peak = torch.cuda.max_memory_allocated() / 1024**2  # MB
```

### Check Available Memory
```bash
# Total available
free -h

# GPU available
nvidia-smi --query-gpu=memory.free --format=csv
```

---

**Maintainer**: Finley Holt
**Last Updated**: 2025-12-25
**Version**: 1.0
