# Phase 6 - Validation & Deployment

**Status**: Not Started
**Dependencies**: Phase 5 (adaptive replanning validated in simulation)

## Objective
Bridge the simulation-to-reality gap through Hardware-in-the-Loop (HIL) testing, edge deployment on NVIDIA Jetson, containerization, and live-flight validation. Prepare system for operational deployment in communications-denied environments.

## Rationale
Phases 1-5 delivered a reasoning-enabled autonomy stack validated entirely in Gazebo simulation. Phase 6 addresses critical remaining challenges:
- **Simulation-reality gap**: Gazebo physics and sensors are idealized; real hardware has noise, delays, and failures
- **Edge compute constraints**: Development on workstation GPU ≠ Jetson Orin edge deployment
- **Operational safety**: Live flight requires safety cases, failure mode analysis, and regulatory considerations
- **System integration**: Containerized deployment for field operations and reproducible builds

Without Phase 6, the system is a research prototype, not an operational capability.

## Goals
1. **Hardware-in-the-Loop (HIL) Testing**: Validate stack on real PX4 autopilot before live flight
2. **Edge Deployment**: Port entire stack to NVIDIA Jetson Orin with resource constraints
3. **Perception Model Fine-Tuning**: Address sim-to-real gap in object detection
4. **Containerization**: Docker-based deployment for reproducible builds and field operations
5. **Data Management**: Implement logging, telemetry storage, and post-mission analysis pipeline
6. **Live-Flight Testing**: Execute Missions 1-2 on physical drone in controlled environment
7. **Safety Case Documentation**: Formal safety analysis and failure mode mitigation

## Success Criteria
- [ ] HIL tests pass with real PX4 autopilot (100 simulated missions, >95% success)
- [ ] Stack runs on Jetson Orin within resource budgets (CPU <80%, GPU mem <24GB, RAM <28GB)
- [ ] Object detection performance validated on real drone imagery (recall >70%, precision >60%)
- [ ] Containerized deployment works across development and edge platforms
- [ ] 10 live test flights completed safely (no crashes, no geofence violations)
- [ ] Mission 1 executed successfully on physical drone (≥2 of 3 vehicles detected)
- [ ] Safety case document completed and reviewed
- [ ] System ready for field demonstration

## Tasks

### 6.1 Hardware-in-the-Loop (HIL) Testing

#### HIL Setup
- [ ] Procure PX4-compatible autopilot board:
  - Pixhawk 6C/6X or Holybro Pixhawk equivalent
  - Verified compatibility with PX4 v1.14.0 (from Phase 1)
  - Bench setup (no motors initially—test FMU logic only)

- [ ] Configure HIL connection:
  - Connect autopilot to development machine via USB/serial
  - Configure PX4 for HIL mode (HITL simulation)
  - Bridge MAVLink between PX4 hardware and Gazebo simulator
  - Verify telemetry flow: PX4 → MAVSDK → ROS 2

- [ ] Document HIL architecture in `docs/hil_setup.md`:
  - Wiring diagram
  - PX4 parameter changes for HIL mode
  - Known limitations (no motor feedback, idealized IMU)

#### HIL Validation
- [ ] Run Phase 3 manual mission script (Mission 1) in HIL:
  - Use real autopilot hardware for flight controller logic
  - Gazebo still provides physics and sensors (hybrid setup)
  - Verify no behavior regressions vs pure SITL

- [ ] Execute 100 automated HIL mission runs:
  - Mission 1 (urban vehicle search) × 50 runs
  - Mission 2 (open area recon) × 50 runs
  - Track success rate, failure modes, edge cases
  - **Success threshold**: >95% completion rate, no crashes

- [ ] Stress test HIL with Phase 5 curveball scenarios:
  - Inject failures (GPS loss, sensor dropout, low battery)
  - Validate replanning works with real autopilot latency
  - Measure any timing differences vs SITL

- [ ] Document HIL findings:
  - Performance differences from SITL (latency, jitter)
  - Autopilot-specific quirks or tuning needs
  - Recommended PX4 parameter adjustments for live flight

**Deliverable**: Validated system with real PX4 autopilot, >95% HIL mission success rate

### 6.2 Edge Deployment on Jetson Orin

#### Jetson Orin Setup
- [ ] Procure NVIDIA Jetson Orin NX/AGX:
  - Orin NX 16GB (minimum) or Orin AGX 32GB (preferred)
  - JetPack 6.0+ with CUDA 12.x support
  - microSD card (128GB) or NVMe SSD (256GB) for storage

- [ ] Install base system:
  - Flash JetPack OS (Ubuntu 22.04 for Jetson)
  - Install ROS 2 Humble (ARM64 build)
  - Install CUDA, cuDNN, TensorRT (via JetPack)
  - Clone LLMDrone repository

- [ ] Validate Phase 1-style setup on Jetson:
  - Run `make setup` adapted for Jetson (no PX4 build—remote autopilot)
  - Install dependencies: MAVSDK, OpenCV, YOLO runtime
  - **Skip Gazebo on Jetson** (runs on dev machine, Jetson connects remotely)

#### Resource Optimization
- [ ] Profile Phase 3 autonomy stack on Jetson:
  - Run without LLM first—measure CPU, GPU, RAM baseline
  - Identify bottlenecks (likely YOLO inference)
  - Document resource consumption in `docs/jetson_performance.md`

- [ ] Optimize YOLO inference:
  - Convert YOLO model to TensorRT engine (FP16 or INT8 precision)
  - Benchmark latency: target <50ms per frame (Phase 3 requirement)
  - If TensorRT exceeds GPU memory → reduce input resolution or use lighter model (YOLOv8n)

- [ ] Deploy LLM on Jetson:
  - Use 4-bit quantized model (GPTQ/AWQ format)
  - Options:
    - **llama.cpp**: CPU/GPU hybrid inference, memory-efficient
    - **TensorRT-LLM**: NVIDIA-optimized, complex setup
    - **Ollama**: Easy deployment, may lack optimization
  - Benchmark latency on Jetson: target <2s (Phase 4 requirement)
  - If latency exceeds target → escalate (use lighter model, offload to companion CPU)

- [ ] Validate resource budgets:
  - CPU <80% sustained (autonomy stack + LLM)
  - GPU memory <24GB (YOLO TensorRT + LLM)
  - RAM <28GB total (leave 4GB OS headroom)
  - Power consumption <15W sustained (Orin NX thermal limits)
  - If budgets violated → profile, optimize, or upgrade to Orin AGX

#### End-to-End Jetson Test
- [ ] Run Phase 3 Mission 1 script from Jetson:
  - Jetson runs autonomy stack (ROS 2 nodes, LLM)
  - Connect to PX4 autopilot via MAVLink (serial/WiFi)
  - Gazebo runs on dev machine (remote connection)
  - Verify mission completes successfully

- [ ] Run Phase 5 replanning scenarios on Jetson:
  - Validate replanning latency meets requirements
  - Check thermal throttling under sustained load
  - Monitor battery if running on drone power

**Deliverable**: Autonomy stack running on Jetson Orin within resource budgets

### 6.3 Sim-to-Real Perception Transfer

#### Real-World Data Collection
- [ ] Collect drone imagery for fine-tuning:
  - If available, fly manual missions with camera recording
  - Or use vehicle detection datasets (CARPK, Stanford Drone Dataset)
  - Annotate 500-1000 images with vehicle bounding boxes
  - Split: 80% train, 10% val, 10% test

- [ ] Measure sim-to-real gap:
  - Run Gazebo-trained YOLO on real images
  - Calculate recall, precision (expect 20-40% drop from simulation)
  - Document gap in `docs/perception_performance.md`

#### Model Fine-Tuning
- [ ] Fine-tune YOLO on real drone imagery:
  - Use transfer learning from COCO-pretrained model
  - Train for 50-100 epochs with real data
  - Validate on held-out test set
  - Target: recall >70%, precision >60% on real imagery

- [ ] Domain randomization (if fine-tuning data limited):
  - Add Gazebo lighting variations (sun angle, clouds)
  - Texture randomization on vehicles and buildings
  - Sensor noise injection (blur, exposure variation)
  - Retrain perception model on augmented simulation data

- [ ] Deploy fine-tuned model:
  - Convert to TensorRT (FP16) for Jetson
  - Replace Gazebo-trained model in autonomy stack
  - Re-run HIL tests to validate no regressions

**Deliverable**: Perception model validated on real imagery, >70% recall

### 6.4 Containerization & Deployment

#### Docker Containerization
- [ ] Create multi-stage Dockerfile:
  - Stage 1: Build ROS 2 workspace (llmdrone packages)
  - Stage 2: Install LLM runtime (llama.cpp or TensorRT-LLM)
  - Stage 3: Production image with minimal dependencies
  - Base image: `nvcr.io/nvidia/l4t-base` (Jetson-compatible)

- [ ] Create docker-compose.yml:
  - Service: `llmdrone_autonomy` (ROS 2 nodes)
  - Service: `llmdrone_llm` (LLM inference server)
  - Service: `llmdrone_mavsdk` (MAVLink bridge)
  - Shared volumes: config, logs, model weights
  - Network: bridge mode for inter-service communication

- [ ] Test containerized deployment:
  - Build containers on dev machine and Jetson
  - Run Mission 1 in HIL using containers
  - Verify no performance regression vs native install
  - Document deployment in `docs/docker_deployment.md`

#### Configuration Management
- [ ] Externalize all configurations:
  - ROS 2 parameters in `config/ros_params.yaml`
  - LLM settings in `llm/configs/llm_config.yaml`
  - Mission definitions in `Planning/missions/*.json`
  - PX4 parameters in `px4-config/llm_drone_v1/params.params`
  - Mount as Docker volumes (no rebuild for config changes)

- [ ] Create deployment profiles:
  - `dev`: Development with debugging symbols, verbose logging
  - `hil`: Hardware-in-the-loop testing
  - `flight`: Production flight with minimal logging
  - Select via environment variable: `DEPLOY_PROFILE=flight`

**Deliverable**: Containerized autonomy stack deployable on Jetson via docker-compose

### 6.5 Data Management & Telemetry

#### Logging Strategy
- [ ] Implement structured logging:
  - ROS 2 bag recording: `/drone/state`, `/drone/gps`, `/mission/status`
  - LLM decision logs: JSON files with prompts, outputs, latency
  - Perception logs: Detections with confidence scores, images (sampled)
  - Store in `/data/logs/<mission_id>/`

- [ ] Add log rotation and storage management:
  - Max log size: 10GB per mission
  - Auto-delete logs older than 30 days (configurable)
  - Compress completed mission logs (gzip)

- [ ] Post-mission analysis tools:
  - Script: `scripts/analyze_mission.py <mission_id>`
  - Generates report: flight path, detections, battery usage, LLM decisions
  - Plots: altitude profile, detection heatmap, replanning events
  - Export mission summary CSV for spreadsheet analysis

#### Telemetry Transmission
- [ ] Design telemetry downlink (optional, for field ops):
  - If WiFi/LTE available: stream `/mission/status` to ground station
  - Use MAVLink STATUSTEXT messages for critical events
  - Log everything locally (telemetry may fail in comms-denied)

**Deliverable**: Logging and telemetry pipeline for post-mission analysis

### 6.6 Live-Flight Testing

**WARNING**: Live flight is inherently dangerous. Proceed only with:
- Appropriate pilot training and certification (FAA Part 107 or equivalent)
- Designated flight zone (away from people, structures, airports)
- Safety observer and manual RC override available at all times
- Emergency stop procedure documented and rehearsed

#### Pre-Flight Checklist
- [ ] Create `docs/flight_safety_checklist.md`:
  - Airframe inspection (props, motors, battery connections)
  - Geofence configured and validated (test with manual flight first)
  - Kill switch tested (RC failsafe triggers RTL)
  - Battery fully charged (>90%)
  - Weather conditions acceptable (wind <10 mph, clear visibility)
  - Safety observer briefed

#### Test Flight Progression
- [ ] **Flight 1-3: Manual RC validation**:
  - Fly drone manually to validate flight characteristics
  - Test RC failsafe (turn off transmitter → RTL)
  - Validate geofence (approach boundary → RTL triggered)
  - Measure baseline battery consumption (flight time at hover)

- [ ] **Flight 4-6: Basic autonomy (no LLM)**:
  - Autonomous takeoff → waypoint → land (Phase 3 tactical primitives)
  - Run from Jetson with LLM disabled (scripted mission)
  - Validate MAVSDK bridge, path following accuracy
  - Safety pilot ready for RC takeover

- [ ] **Flight 7-9: Full autonomy with simple missions**:
  - Execute Mission 1 from natural language (Phase 4 LLM)
  - "Survey this area" with small 50m × 50m zone
  - No replanning scenarios yet—straight-line mission
  - Log all LLM decisions for post-flight review

- [ ] **Flight 10: Mission 1 full execution**:
  - Full urban vehicle search mission
  - Physical vehicles placed in field (e.g., parked cars)
  - Execute from natural language: "Survey designated area and identify vehicles"
  - Measure detection accuracy (compare to ground truth)
  - Success if ≥2 of 3 vehicles detected and geotagged

#### Failure Mode Testing (Optional, if time/budget allows)
- [ ] Flight 11: GPS degradation test:
  - Fly to waypoint, manually disable GPS mid-flight (via GCS command)
  - Validate fallback behavior (hold position or RTL)

- [ ] Flight 12: Low battery scenario:
  - Start flight with 40% battery
  - Validate battery-constrained replanning triggers RTL appropriately

**Deliverable**: 10 live flights completed safely, Mission 1 executed successfully on physical drone

### 6.7 Safety Case & Documentation

#### Failure Mode & Effects Analysis (FMEA)
- [ ] Create `docs/safety_case_fmea.md`:
  - Identify all failure modes:
    - GPS loss, IMU failure, motor failure
    - LLM crash or infinite loop
    - Perception failure (false positives/negatives)
    - Battery estimation error
    - Geofence breach
  - For each failure:
    - Likelihood (rare, occasional, frequent)
    - Severity (minor, major, catastrophic)
    - Mitigation (failsafe behavior, detection mechanism)
    - Residual risk assessment

- [ ] Example FMEA entry:
  ```
  Failure: GPS loss mid-flight
  Likelihood: Occasional (jamming, multipath)
  Severity: Major (position uncertainty, navigation failure)
  Detection: GPS fix quality < threshold for 5s
  Mitigation: Trigger replanning → hold position → RTL on last known coords
  Residual Risk: Medium (IMU drift may cause landing offset)
  ```

#### Regulatory Compliance
- [ ] Document regulatory considerations in `docs/regulatory_compliance.md`:
  - FAA Part 107 (US): Requires visual line-of-sight, no autonomous BVLOS
  - Military operations: Coordinate with range control, airspace authority
  - Risk assessment required for operations near people/structures
  - **Note**: Current system is research prototype; operational deployment requires certification

- [ ] Define operational limits:
  - Maximum altitude: 50m AGL (configurable)
  - Geofence: required for all missions
  - Battery reserve: 20% minimum for RTL
  - Wind limits: <15 mph sustained
  - Communication loss: auto-RTL after 30s
  - Manual override: RC transmitter required and tested pre-flight

#### Human-in-the-Loop Decision Points
- [ ] Document HITL requirements in `docs/human_in_the_loop.md`:
  - **Mission approval**: Operator reviews LLM-generated plan before execution
  - **Critical decisions**: If armed operations, human approves target engagement
  - **Anomaly escalation**: On replanning failure, system requests human guidance
  - **Emergency abort**: Operator can trigger RTL at any time via RC or GCS

**Deliverable**: Safety case document, FMEA, regulatory compliance assessment

## Deliverables Checklist
- [ ] HIL test results: >95% success rate over 100 missions
- [ ] Jetson deployment: Stack running within resource budgets
- [ ] Perception model: Fine-tuned for real imagery, >70% recall
- [ ] Docker containers: Reproducible deployment on dev and edge platforms
- [ ] Logging pipeline: Post-mission analysis tools functional
- [ ] 10 live test flights: All completed safely, Mission 1 successful
- [ ] Safety case: FMEA, regulatory compliance, HITL procedures documented
- [ ] Publication-ready results: Performance metrics, lessons learned

## Known Risks and Mitigation

### Risk: Jetson Orin insufficient for LLM + autonomy stack
**Impact**: Critical (system won't run on target hardware)
**Mitigation**: Benchmark early in 6.2; use lighter LLM (7B quantized); offload LLM to companion compute if necessary; cloud fallback for non-field operations

### Risk: Sim-to-real gap too large (perception fails on real imagery)
**Impact**: High (mission failures in live flight)
**Mitigation**: Fine-tune on real data (6.3); domain randomization in simulation; accept lower performance thresholds; plan multiple flights for data collection

### Risk: Live flight crashes damage hardware or injure people
**Impact**: Critical (safety, legal, program termination)
**Mitigation**: Strict safety protocols (6.6), RC override mandatory, safety observer, start with manual flights, geofence tested, insurance/liability coverage

### Risk: PX4 autopilot tuning differs for physical drone
**Impact**: Medium (poor flight performance, oscillations)
**Mitigation**: HIL testing identifies tuning needs early; use autotune feature; iterative tuning during manual flights before autonomy

### Risk: Containerization adds latency or resource overhead
**Impact**: Medium
**Mitigation**: Benchmark container vs native; use host networking mode; minimize image size; profile and optimize

### Risk: Data storage fills up mid-mission
**Impact**: Low
**Mitigation**: Pre-flight storage check in safety checklist; log rotation; configurable log verbosity (reduce in production)

## Phase Exit Criteria
Before declaring system operationally ready:

**Hardware Validation**:
1. ✓ HIL tests: >95% success rate over 100 automated missions
2. ✓ Jetson deployment: CPU <80%, GPU mem <24GB, RAM <28GB, latency requirements met
3. ✓ Live flights: 10 flights completed, zero crashes, zero geofence violations

**Performance Validation**:
1. ✓ Mission 1 on physical drone: ≥2 of 3 vehicles detected and correctly geotagged
2. ✓ Perception model: recall >70%, precision >60% on real drone imagery
3. ✓ LLM inference: <2s initial planning, <1s replanning on Jetson
4. ✓ End-to-end latency: perception → LLM → action execution <5s total

**Safety & Compliance**:
1. ✓ FMEA completed with all critical failure modes addressed
2. ✓ Safety checklist validated in all 10 test flights
3. ✓ Geofence tested and never violated
4. ✓ RC override tested and functional in all flights
5. ✓ Post-flight log analysis shows no undetected anomalies

**Documentation & Reproducibility**:
1. ✓ Docker deployment tested on 2+ Jetson devices
2. ✓ Setup documentation enables independent reproduction
3. ✓ Performance metrics documented for publication
4. ✓ Lessons learned captured for future work

## Next Steps (Post-Phase 6)
Once Phase 6 is complete:
- **Research publication**: Submit findings to ICRA, IROS, or RSS
- **Field demonstration**: Coordinate with sponsor for operational scenario demo
- **Technology transfer**: Package system for handoff to operational unit
- **Future work**:
  - Multi-drone coordination using LLM reasoning
  - Adversarial robustness testing (active GPS jamming)
  - Integration with tactical network (MANET, mesh comms)
  - Swarm intelligence with distributed LLM inference

## Notes and Findings
_Use this section to document HIL insights, Jetson optimization strategies, live-flight lessons, and recommendations for operational deployment._

---

**Phase Started**: [Date]
**Phase Completed**: [Date]

**Hardware Configuration**:
- Jetson model and specs
- PX4 autopilot version and board
- Drone frame and sensor suite
- LLM model deployed

**Performance Achieved**:
- HIL success rate
- Live flight success rate
- Detection accuracy on real imagery
- Resource utilization on Jetson

**Critical Findings**:
- Sim-to-real gap magnitude
- PX4 tuning adjustments needed
- Failure modes encountered
- Operational limitations discovered

**Recommendations for Operational Use**:
- Mission envelope (weather, environment constraints)
- Maintenance and calibration procedures
- Operator training requirements
- Risk mitigation for field deployment
