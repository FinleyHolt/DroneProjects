# Phase 12: Project-Drone Hardware Validation

## Overview

Deploy and validate the ontology-constrained RL system on the project-drone platform (Jetson Orin Nano Super 8GB). This phase performs real-world flight testing in a controlled environment before deploying to the Flyby-F11.

## Human Description

Project-drone serves as the development and validation platform:

- **Hardware**: Jetson Orin Nano Super 8GB (67 TOPS), T265 + D455 sensors
- **Purpose**: Indoor/outdoor testing with accessible hardware
- **Goal**: Validate system behavior before Flyby-F11 deployment

This phase covers:
1. Hardware bringup and sensor integration
2. System deployment on Jetson
3. Indoor flight testing (controlled environment)
4. Performance profiling and optimization
5. Safety validation (collision-free operation)

## AI Agent Instructions

### Prerequisites
- Phase 11 completed (simulation benchmarks passing)
- Project-drone hardware available
- Safe testing environment identified
- Understanding of hardware deployment
- Flight testing safety protocols

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Hardware Bringup**
   - Flash Jetson Orin Nano Super
   - Install base system (JetPack)
   - Deploy container images (ARM builds)
   - Validate GPU and sensor access

2. **Sensor Integration**
   - T265 visual odometry verification
   - D455 depth camera verification
   - PX4 telemetry connection
   - Sensor fusion validation

3. **System Deployment**
   - Deploy execution mode container
   - Load trained RL models
   - Configure ROS 2 network
   - Validate Prolog runtime performance

4. **Indoor Flight Testing**
   - Location: Controlled indoor environment
   - Basic waypoint navigation
   - Obstacle avoidance (static objects)
   - GPS-denied navigation (T265 only)

5. **Performance Profiling**
   - CPU/GPU/memory utilization
   - Reasoning latency measurements
   - Control loop frequency validation
   - Thermal management

6. **Iterative Refinement**
   - Identify failure modes
   - Tune parameters for real hardware
   - Optimize perception pipeline
   - Document sim-to-real gaps

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] Hardware bringup complete
- [ ] All sensors verified working
- [ ] System deploys and runs on Jetson
- [ ] 10+ successful indoor flights
- [ ] 5+ obstacle avoidance tests passed
- [ ] GPS-denied navigation > 5 minutes
- [ ] Zero collisions in testing
- [ ] Resource utilization < 70%

### Verification

Run automated verification:
```bash
bash .phases/phase-12-project-drone-validation/verification.sh
```

### Time Estimate
20-30 hours (includes hardware setup and multiple flight test sessions)

### Common Pitfalls

- **Thermal throttling**: Jetson may throttle under load
- **Sensor calibration**: T265/D455 require proper mounting
- **Network latency**: ROS 2 over network introduces delay
- **Power issues**: Battery sag affects flight dynamics
- **Sim-to-real gap**: Models may need real-world tuning
- **Safety margins**: Use wider margins for initial flights

### References

- [Jetson Orin Nano Developer Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
- [T265 Documentation](https://www.intelrealsense.com/tracking-camera-t265/)
- [D455 Documentation](https://www.intelrealsense.com/depth-camera-d455/)
- [project-drone Documentation](../../project-drone/README.md)

### Dependencies
See `dependencies.json` - requires simulation benchmarks passing.

### Next Phase
After completion, proceed to Phase 13: Flyby-F11 Integration
