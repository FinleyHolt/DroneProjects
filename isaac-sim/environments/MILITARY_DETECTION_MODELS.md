# Military Object Detection Model Research

This document summarizes research and recommendations for military-oriented object detection
suitable for the Flyby F-11 ISR training environment.

## Target Requirements

**Target Classes:**
- Personnel (dismounts, soldiers)
- Light vehicles (trucks, technicals)
- Armored vehicles (tanks, APCs, IFVs)
- Aircraft (helicopters, UAVs)
- Weapons emplacements

**Performance Constraints:**
- Platform: Jetson Orin NX 16GB (50 TOPS)
- Minimum FPS: 10+ fps
- Input resolution: 640x480 minimum

## Recommended Approaches

### 1. Fine-tuned YOLOv8/YOLOv11 (Recommended)

**Best for:** Production deployment with validated performance

Fine-tuning a YOLOv8 or YOLOv11 model on military-specific datasets provides the best
balance of accuracy, speed, and reliability.

**Available Datasets:**

1. **YOLO Military Dataset (Roboflow)**
   - 3,285 images with pre-trained models
   - Classes: Tank, Vehicle, Helicopter, Person, Airplane
   - License: CC BY 4.0
   - Source: https://universe.roboflow.com/rl4pcd/yolo-military-s48o9

2. **Military Object Detection Dataset**
   - 2,636 images
   - Classes: Military vehicle, Soldier, Tank
   - License: CC BY 4.0
   - Source: https://universe.roboflow.com/yolo-datasets-ymdve/military-object-detection-uxkcn

3. **ADOMVI Dataset**
   - Custom dataset for Automated Detection of Military Vehicles
   - Classes: AFV, APC, MEV, LAV
   - Includes YOLOv8 training utilities
   - Source: https://github.com/jonasrenault/adomvi

4. **xView Dataset**
   - 1M+ objects, 60 classes including military vehicles
   - 0.3m resolution satellite imagery
   - Source: https://docs.ultralytics.com/datasets/detect/xview/

**Expected Performance (YOLOv8n on Jetson Orin NX):**
- FPS: 30-60 fps
- mAP@0.5: 70-85% (depending on dataset quality)
- Memory: ~2GB VRAM

**Training Recipe:**
```python
from ultralytics import YOLO

# Start with pretrained COCO weights
model = YOLO('yolov8n.pt')

# Fine-tune on military dataset
model.train(
    data='military_dataset.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,
)

# Export for Jetson deployment
model.export(format='engine', device=0)  # TensorRT
```

### 2. YOLO-World Zero-Shot (Development/Prototyping)

**Best for:** Rapid prototyping without dataset collection

YOLO-World provides open-vocabulary detection using natural language prompts,
enabling zero-shot detection of military objects without fine-tuning.

**Key Features:**
- 35.4 AP on LVIS at 52 FPS (V100)
- Customizable class prompts
- No retraining required for new classes

**Example Military Prompts:**
```python
from ultralytics import YOLOWorld

model = YOLOWorld('yolov8l-world.pt')

# Set military-oriented classes
military_classes = [
    "soldier", "person with rifle", "military personnel",
    "tank", "armored vehicle", "APC", "IFV",
    "military truck", "humvee", "technical vehicle",
    "helicopter", "military aircraft",
    "weapons emplacement", "fortification"
]
model.set_classes(military_classes)

# Run inference
results = model.predict(image)
```

**Limitations:**
- Lower accuracy than fine-tuned models (~10-20% lower mAP)
- May confuse similar classes (tank vs APC)
- Requires careful prompt engineering

Source: https://docs.ultralytics.com/models/yolo-world/

### 3. Synthetic Data Fine-tuning (Future Work)

**Best for:** Domain-specific adaptation using Isaac Sim

Isaac Sim can generate unlimited synthetic training data with perfect annotations,
enabling domain-specific fine-tuning for the exact deployment environment.

**Approach:**
1. Use Isaac Sim's Replicator to generate varied scenes
2. Export images with ground truth annotations
3. Fine-tune YOLOv8 on synthetic + real data
4. Validate on real-world military imagery

**Benefits:**
- Perfect bounding boxes and labels
- Unlimited data generation
- Controllable domain randomization
- No manual annotation required

**Implementation Plan:**
```python
# In Isaac Sim
import omni.replicator.core as rep

# Create synthetic military scene dataset
with rep.new_layer():
    camera = rep.create.camera(position=(0, 0, 100))

    # Spawn military assets with randomization
    tanks = rep.create.from_usd(military_assets, count=10)

    with rep.trigger.on_frame(num_frames=10000):
        with tanks:
            rep.modify.pose(
                position=rep.distribution.uniform((-100, -100, 0), (100, 100, 0)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
            )

        # Write annotations
        rep.WriterRegistry.get("BasicWriter")(
            output_dir="military_synthetic_data",
            rgb=True,
            bounding_box_2d_tight=True,
        )
```

## Model Comparison

| Model | FPS (Orin NX) | mAP@0.5 | Training Required | Recommended Use |
|-------|---------------|---------|-------------------|-----------------|
| YOLOv8n (COCO) | 60+ | Low | No | Baseline/testing |
| YOLOv8n (fine-tuned) | 50-60 | 75-85% | Yes | Production |
| YOLOv8s (fine-tuned) | 35-45 | 80-90% | Yes | High accuracy |
| YOLO-World-S | 40-50 | 60-70% | No | Prototyping |
| YOLO-World-L | 20-30 | 70-80% | No | Development |

## Recommendation

**For Functions Check & Development:**
Use YOLO-World with military prompts for rapid iteration without dataset collection.
This enables quick validation of the perception pipeline.

**For Training & Deployment:**
Fine-tune YOLOv8n on the YOLO Military Dataset from Roboflow, augmented with
synthetic data from Isaac Sim. This provides the best accuracy/speed tradeoff
for the Jetson Orin NX platform.

**Implementation Priority:**
1. Integrate YOLO-World for immediate testing (1-2 days)
2. Download and format Roboflow military datasets (1 day)
3. Fine-tune YOLOv8n on combined datasets (2-3 days)
4. Export TensorRT engine for Jetson deployment (1 day)
5. (Future) Add synthetic data generation pipeline

## References

- [YOLO Military Dataset](https://universe.roboflow.com/rl4pcd/yolo-military-s48o9)
- [YOLO-World Documentation](https://docs.ultralytics.com/models/yolo-world/)
- [xView Dataset](https://docs.ultralytics.com/datasets/detect/xview/)
- [DOTA Dataset](https://captain-whu.github.io/DOTA/)
- [ADOMVI GitHub](https://github.com/jonasrenault/adomvi)
- [Military Vehicles Tracking](https://github.com/Lin-Sinorodin/Military_Vehicles_Tracking)
