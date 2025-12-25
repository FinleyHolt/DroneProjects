# Vision Documentation Index

**Location**: `/home/finley/Github/DroneProjects/flyby-f11/docs/vision/`
**Created**: 2025-12-25
**Status**: Ready for Phase 2 (Weeks 7-10)

## Quick Links

### Start Here
1. **[GETTING_STARTED.md](GETTING_STARTED.md)** - Read this first!
   - What was created and why
   - How to download documentation and repositories
   - Next steps and timeline

2. **[README.md](README.md)** - Main documentation
   - Overview of all vision tools
   - Hardware specifications
   - Integration roadmap
   - Performance benchmarks

### Implementation Guides
3. **[PHASE2_CHECKLIST.md](PHASE2_CHECKLIST.md)** - Week-by-week tasks
   - Complete checklist for Phase 2 (Weeks 7-10)
   - Setup, installation, testing, validation
   - Success criteria for each week

4. **[MODEL_CONFIGURATIONS.md](MODEL_CONFIGURATIONS.md)** - Technical reference
   - Model selection and optimization
   - TensorRT export commands
   - ROS 2 integration examples
   - Multi-model pipelines

### Download Script
5. **[download_vision_docs.sh](download_vision_docs.sh)** - Run this to download everything!
   - Clones 4 repositories (shallow)
   - Downloads 20+ documentation pages
   - Creates manifest.txt
   - Estimated time: 15-30 minutes

## Documentation Size

| File | Size | Lines | Purpose |
|------|------|-------|---------|
| GETTING_STARTED.md | ~12KB | ~450 | Overview and next steps |
| README.md | ~13KB | ~400 | Main documentation |
| MODEL_CONFIGURATIONS.md | ~17KB | ~900 | Technical reference |
| PHASE2_CHECKLIST.md | ~23KB | ~1,100 | Implementation checklist |
| download_vision_docs.sh | ~15KB | ~620 | Download script |
| INDEX.md | ~2KB | ~100 | This file |
| **Total** | **~82KB** | **~3,570** | Complete documentation |

## Directory Structure

```
/home/finley/Github/DroneProjects/flyby-f11/docs/vision/
│
├── INDEX.md                      # This file (start here)
├── GETTING_STARTED.md           # Quick start guide
├── README.md                     # Main documentation
├── MODEL_CONFIGURATIONS.md       # Technical reference
├── PHASE2_CHECKLIST.md          # Week-by-week tasks
├── download_vision_docs.sh      # Download script (run this!)
│
├── ultralytics/                 # YOLO11 documentation (populated by script)
├── tensorrt/                    # TensorRT docs (populated by script)
├── realsense/                   # RealSense docs (populated by script)
├── jetson-inference/            # Segmentation docs (populated by script)
├── vlm/                         # VLM docs (populated by script)
└── repos/                       # Cloned repositories (populated by script)
    ├── ultralytics/
    ├── realsense-ros/
    ├── realsense_t265_shelfino/
    └── jetson-inference/
```

## Quick Start (3 Steps)

### Step 1: Download Everything
```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/vision
./download_vision_docs.sh
```

### Step 2: Review Documentation
```bash
# Read the getting started guide
less GETTING_STARTED.md

# Review main documentation
less README.md

# Check what was downloaded
cat manifest.txt
```

### Step 3: Begin Phase 2
```bash
# Follow the checklist
less PHASE2_CHECKLIST.md

# Start with Week 7 (Object Detection)
# Install Ultralytics, export YOLO11 to TensorRT, benchmark on Jetson
```

## Tools Covered

### 1. YOLO11 (Ultralytics)
- **Purpose**: Real-time object detection
- **Models**: YOLOv11n, YOLOv11s, YOLOv11m
- **Performance**: 30-100 FPS on Jetson Orin NX
- **Documentation**: `ultralytics/` + `repos/ultralytics/`

### 2. TensorRT
- **Purpose**: Inference optimization
- **Optimizations**: FP16, INT8 quantization
- **Speedup**: 2-4x faster inference
- **Documentation**: `tensorrt/`

### 3. Intel RealSense
- **Purpose**: Visual odometry, depth sensing
- **Sensors**: T265 (VIO), D455 (depth + RGB)
- **Performance**: 200 Hz odometry, 30 FPS depth
- **Documentation**: `realsense/` + `repos/realsense-ros/`

### 4. Semantic Segmentation
- **Purpose**: Terrain classification
- **Models**: FCN-ResNet18, SegNet
- **Performance**: 25-30 FPS
- **Documentation**: `jetson-inference/` + `repos/jetson-inference/`

### 5. Vision-Language Models (VLM)
- **Purpose**: Mission intent parsing
- **Models**: NanoVLM, VILA-3B
- **Performance**: 2-10 tokens/sec
- **Documentation**: `vlm/`

## Phase 2 Timeline (Weeks 7-10)

### Week 7: Object Detection (YOLO11)
- Install Ultralytics
- Export YOLOv11n to TensorRT
- Create ROS 2 detection nodes
- Benchmark: 80+ FPS target

### Week 8: Sensor Fusion (RealSense)
- Install RealSense SDK and ROS 2 wrappers
- Integrate T265 visual odometry with PX4
- Implement D455 depth-based obstacle avoidance
- Benchmark: 200 Hz odometry, 30 FPS depth

### Week 9: Semantic Segmentation
- Install jetson-inference
- Benchmark FCN-ResNet18 and SegNet
- Create terrain classification nodes
- Implement landing zone detection

### Week 10: VLM Integration
- Install NanoVLM
- Create mission intent parser
- Integrate with behavior trees
- Test natural language commands

## Performance Targets

### Hardware: Jetson Orin NX 16GB
- Compute: 50 TOPS
- Memory: 16GB LPDDR5
- GPU: 1024 CUDA cores, 32 Tensor cores

### Target Benchmarks

| Task | Model | FPS | Latency | VRAM |
|------|-------|-----|---------|------|
| Object Detection | YOLOv11n | 80-100 | 10-12ms | 500MB |
| Object Detection | YOLOv11s | 50-70 | 14-20ms | 800MB |
| Terrain Segmentation | FCN-ResNet18 | 30 | 33ms | 600MB |
| Mission Parsing | NanoVLM | 5-10 tok/s | 100-200ms | 2GB |

### Multi-Model Pipeline (Real-time Navigation)
- **Models**: YOLOv11n + FCN-ResNet18 + RealSense
- **Total VRAM**: ~2.1GB / 16GB (13% utilization)
- **FPS**: 30 (camera-limited)
- **Latency**: 45-55ms end-to-end
- **Temperature**: <80°C sustained

## Common Questions

### Q: Where do I start?
**A**: Read `GETTING_STARTED.md`, then run `download_vision_docs.sh`

### Q: How long will Phase 2 take?
**A**: 4 weeks (Weeks 7-10), estimated 7-10 hours per week

### Q: What if I don't have Jetson hardware yet?
**A**: You can still:
- Read documentation and plan architecture
- Develop and test on project-drone platform
- Export models to TensorRT on desktop GPU
- Test in simulation

### Q: Which model should I use for object detection?
**A**: Start with YOLOv11n (nano):
- Best FPS (80-100)
- Lowest VRAM (500MB)
- Good enough accuracy for most tasks
- Can upgrade to YOLOv11s if needed

### Q: Will all models fit in 16GB memory?
**A**: Yes, comfortably:
- Recommended pipeline: ~2.1GB (13% utilization)
- Full pipeline with VLM: ~4-5GB (25-30% utilization)
- Maximum configuration: ~9.7GB (61% utilization)

### Q: What if I encounter issues?
**A**: Check these resources:
- Troubleshooting section in MODEL_CONFIGURATIONS.md
- Known issues in PHASE2_CHECKLIST.md
- Official documentation (links in README.md)
- Jetson forums: https://forums.developer.nvidia.com/

## Success Criteria

### Documentation Setup (Now)
- [x] All documentation files created
- [x] Download script ready to execute
- [x] Directory structure in place

### Phase 2 Completion (Weeks 7-10)
- [ ] Object detection: YOLOv11n at 80+ FPS
- [ ] Visual odometry: T265 at 200 Hz, fused with PX4 EKF
- [ ] Segmentation: FCN-ResNet18 at 30 FPS
- [ ] VLM: Mission parsing <500ms latency
- [ ] Full pipeline: <5GB VRAM, 30 FPS, <80°C

### Integration (Post-Phase 2)
- [ ] Perception pipeline integrated with behavior trees
- [ ] Tested in simulation (Gazebo)
- [ ] Tested on hardware (flyby-f11)
- [ ] Documentation updated with lessons learned

## Next Actions (In Order)

1. **Read GETTING_STARTED.md** (5 minutes)
2. **Run download_vision_docs.sh** (15-30 minutes)
3. **Review README.md** (30 minutes)
4. **Read MODEL_CONFIGURATIONS.md** (1 hour)
5. **Begin PHASE2_CHECKLIST.md Week 7** (next session)

## File Permissions

All files are readable and the download script is executable:
```bash
# Verify permissions
ls -lh /home/finley/Github/DroneProjects/flyby-f11/docs/vision/

# Download script should be executable (-rwx--x--x)
# Markdown files should be readable (-rw-------)
```

## Estimated Download Size

When you run `download_vision_docs.sh`, expect:
- **Repositories**: ~150MB (4 repos, shallow clones)
- **Documentation**: ~5-10MB (20+ HTML pages)
- **Research Papers**: ~2-5MB (PDFs)
- **Total**: ~150-200MB

**Disk Space Required**: 300MB (safe margin)

## Contact and Support

**Project**: flyby-f11 Autonomous Navigation
**Developer**: Finley Holt
**Documentation Version**: 1.0
**Last Updated**: 2025-12-25

For questions or issues:
1. Check troubleshooting sections in documentation
2. Review official tool documentation
3. Search Jetson forums for hardware-specific issues
4. Consult ROS 2 documentation for integration questions

---

**Ready to start? Begin with GETTING_STARTED.md or run the download script!**

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/vision
./download_vision_docs.sh
```
