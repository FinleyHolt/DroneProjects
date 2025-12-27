# RL Simulation Requirements for Flyby F-11

## Overview

This document captures research findings on reinforcement learning best practices for UAV simulation, domain randomization techniques, episode management strategies, and performance considerations specific to the Flyby F-11 platform.

## Research Summary

### 1. Environment Design Best Practices

#### Multi-Goal and Long-Horizon Challenges
Recent research from ICLR 2025 (FlyCraft/VVC-Gym) highlights that UAV RL tasks are characterized by:
- **Multi-goal nature**: Additional goal space intensifies spatial complexity of exploration
- **Long-horizon problems**: Extended interaction sequences exacerbate temporal complexity
- **Exploration difficulty**: Common RL algorithms (PPO, SAC) struggle without curriculum learning

**Recommendation**: Implement curriculum learning with progressive difficulty adjustment.

#### Environment Architecture (Gazebo Best Practices)
Based on established Gazebo-RL patterns, use a three-tier environment architecture:

1. **GazeboEnv (Base Layer)**
   - Handles simulation communication
   - Manages resets, pausing, and cleanup
   - Platform-agnostic implementation

2. **RobotEnv (Middle Layer)**
   - Inherits from GazeboEnv
   - Robot-specific functions (movement, sensor reading)
   - MAVLink/pymavlink integration

3. **TaskEnv (Top Layer)**
   - Inherits from RobotEnv
   - Task-specific rewards, actions, termination conditions
   - Specific to trajectory optimization, behavior selection, or mission planning

#### Key UAV RL Environments (Reference Implementations)
- **FlyCraft (ICLR 2025)**: Goal-conditioned fixed-wing UAV control
- **gym-pybullet-drones**: Gymnasium + stable-baselines3 2.0 compatible quadcopter env
- **PyFlyt**: Multi-UAV simulation for RL research
- **Air Learning**: Deep RL benchmark with domain randomization support

### 2. Domain Randomization Techniques

#### Physics Randomization
Based on recent 2025 research on sim-to-real transfer:

| Parameter | Default Range | Aggressive Range | Notes |
|-----------|--------------|------------------|-------|
| Mass | +/- 10% | +/- 30% | Critical for real-world transfer |
| Inertia | +/- 10% | +/- 20% | Affects rotational dynamics |
| Motor constants | +/- 5% | +/- 15% | Key for thrust modeling |
| Drag coefficients | +/- 20% | +/- 40% | Wind effects approximation |
| Ground friction | 0.3-0.8 | 0.1-1.0 | Landing behavior |
| Propeller efficiency | 0.85-0.95 | 0.70-1.0 | Thrust accuracy |

#### Sensor Noise Randomization
| Sensor | Noise Type | Range |
|--------|-----------|-------|
| IMU accelerometer | Gaussian | sigma: 0.01-0.1 m/s^2 |
| IMU gyroscope | Gaussian + bias | sigma: 0.001-0.01 rad/s |
| GPS position | Gaussian | sigma: 0.5-3.0 m |
| Barometer | Gaussian + drift | sigma: 0.1-1.0 m |
| Depth camera | Multiplicative | 0.95-1.05 scale |

#### Visual Randomization (If Using Vision)
- Lighting intensity: 0.5-2.0x baseline
- Texture variations: Random procedural textures
- Camera intrinsics: +/- 5% focal length variation
- Motion blur: Random based on velocity

#### Critical Finding (April 2025)
From "One Net to Rule Them All: Domain Randomization in Quadcopter Racing":
- **No randomization fails sim-to-real transfer**
- Increasing randomization improves robustness but reduces peak speed
- Optimal range varies by platform (10-30% tested on 3-inch and 5-inch quadcopters)

#### Key Factors for Zero-Shot Transfer (May 2025)
From "What Matters in Learning A Zero-Shot Sim-to-Real RL":
1. Integrate velocity and rotation matrix into actor's input
2. Add time vector to critic's input
3. Apply action difference regularization as smoothness reward
4. Use system identification with selective randomization
5. Utilize large batch sizes (reduces variance)

### 3. Episode Management Strategies

#### Efficient Episode Reset
```
Episode Reset Pipeline:
1. Stop actuators / disarm
2. Reset physics simulation time
3. Teleport vehicle to spawn position
4. Reset vehicle state (velocity, acceleration)
5. Apply randomization (if enabled)
6. Re-arm and initialize sensors
7. Wait for sensor settling (50-100ms)
8. Return initial observation
```

**Critical Issue**: Default Gazebo resets can break odometry topics. Solution: Use "WORLD" reset instead of "SIMULATION" reset.

#### Episode Termination Conditions
| Condition | Type | Handling |
|-----------|------|----------|
| Goal reached | Success | terminated=True, truncated=False |
| Collision | Failure | terminated=True, large negative reward |
| Out of bounds | Failure | terminated=True, negative reward |
| Battery depleted | Failure | terminated=True |
| Timeout | Truncated | terminated=False, truncated=True |
| Safety constraint | Failure | terminated=True |

#### Spawn Position Strategies
1. **Fixed spawn**: Consistent for debugging
2. **Random spawn**: Select from predefined safe positions
3. **Curriculum spawn**: Start easy, progressively harder positions
4. **Adversarial spawn**: Spawn in challenging positions after basic proficiency

### 4. Simulation Fidelity vs Training Speed

#### Multi-Fidelity Training Pipeline
Based on 2025 robotics research, use a staged approach:

**Stage 1: Core Simulation (Low Fidelity)**
- Purpose: Extensive exploration, policy initialization
- Physics: Simplified dynamics, faster than real-time
- Goal: Learn basic behaviors
- Duration: ~70% of total training

**Stage 2: High-Fidelity Simulation**
- Purpose: Policy refinement, precise control
- Physics: Full dynamics, contact modeling
- Goal: Learn nuanced behaviors
- Duration: ~25% of total training

**Stage 3: Real-World Fine-Tuning**
- Purpose: Final adaptation
- Duration: ~5% of total training (if applicable)

#### Speed vs Fidelity Benchmarks
| Simulator | Fidelity | Speed (steps/sec) | GPU Required |
|-----------|----------|-------------------|--------------|
| Gazebo Harmonic | High | 100-500 | Optional |
| PyBullet | Medium | 1,000-5,000 | No |
| MuJoCo | High | 2,000-10,000 | No |
| Isaac Sim | Very High | 10,000-100,000 | Yes (RTX) |
| Custom lightweight | Low | 50,000+ | No |

**Note**: With RTX 5090 available, Gazebo with GPU acceleration is viable for moderate-scale training.

#### ArduPilot SITL Considerations
- SITL speedup factor: 1-10x real-time (set via `SITL_SPEEDUP`)
- MAVLink latency: ~5-10ms per message round-trip
- Recommended: Run multiple SITL instances in parallel (4-8 instances)

### 5. Training Metrics and Logging

#### Essential Metrics to Track

**Episode-Level Metrics**
- Episode return (cumulative reward)
- Episode length
- Success rate (per N episodes)
- Collision rate
- Constraint violation rate

**Step-Level Metrics**
- Instantaneous reward components
- Position/velocity tracking error
- Action magnitude and smoothness
- Sensor readings (for debugging)

**Training Progress Metrics**
- Policy loss (actor loss)
- Value loss (critic loss)
- KL divergence (for PPO)
- Entropy (exploration indicator)
- Learning rate schedule

**System Metrics**
- Episodes per hour
- Steps per second
- GPU utilization
- Memory usage
- SITL instance health

#### Logging Framework Recommendations
1. **TensorBoard**: Primary logging for training curves
2. **Weights & Biases (WandB)**: Cloud-based experiment tracking, hyperparameter sweeps
3. **Custom CSV**: Raw data export for analysis

### 6. Performance Optimization Strategies

#### Vectorized Environments
- Use `gymnasium.vector.AsyncVectorEnv` for parallel data collection
- Each environment connects to separate ArduPilot SITL instance
- Recommended: 4-8 parallel environments per GPU

#### Headless Simulation
```bash
# Run Gazebo headless (no rendering)
HEADLESS=true podman-compose up

# ArduPilot SITL without MAVProxy GUI
sim_vehicle.py -v ArduCopter --no-mavproxy -S 10  # 10x speedup
```

#### Batched Inference
- Batch policy forward passes across parallel environments
- Use GPU for neural network inference
- Target: <2ms per batch inference

#### Memory Management
- Replay buffer: Consider prioritized experience replay (PER)
- Clear unnecessary tensors between updates
- Monitor for memory leaks in long training runs

### 7. ArduPilot SITL + pymavlink Integration

#### Connection Architecture
```
+-------------+     MAVLink TCP      +----------------+
| ArduPilot   | <----------------->  | pymavlink      |
| SITL        |     (port 5760)      | Connection     |
+-------------+                      +----------------+
                                            |
                                            v
                                     +--------------+
                                     | Gymnasium    |
                                     | Environment  |
                                     +--------------+
```

#### Key pymavlink Operations
```python
# Connect to SITL
mavutil.mavlink_connection('tcp:localhost:5760')

# Send velocity command
connection.mav.set_position_target_local_ned_send(
    time_boot_ms, target_system, target_component,
    coordinate_frame, type_mask,
    x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
)

# Receive state
msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
```

#### GSoC 2025 Initiative
ArduPilot has proposed an "AP-SITL reinforcement learning script concept" for Google Summer of Code 2025, indicating growing official support for RL integration.

## Implementation Recommendations for Flyby F-11

### Immediate Actions
1. Implement Gymnasium wrapper using existing `episode_manager.py` as foundation
2. Add pymavlink direct connection (bypass ROS 2 for lower latency in training)
3. Create YAML-configurable domain randomization
4. Set up TensorBoard logging from day one

### Training Configuration Defaults
```yaml
# Recommended starting configuration
simulation:
  sitl_speedup: 5  # 5x real-time
  headless: true
  num_parallel_envs: 4

training:
  algorithm: TD3  # Start with trajectory optimizer
  batch_size: 256
  learning_rate: 3e-4
  buffer_size: 1_000_000
  total_timesteps: 2_000_000

domain_randomization:
  mass_range: [0.9, 1.1]  # Start conservative
  drag_range: [0.8, 1.2]
  sensor_noise: 0.01
```

### Safety Considerations
- Always enforce geofence at simulation level
- Implement action clipping before sending to SITL
- Monitor for unstable training (diverging losses)
- Regular model checkpointing

## References

### Papers and Research
- [FlyCraft/VVC-Gym (ICLR 2025)](https://github.com/GongXudong/fly-craft) - Goal-conditioned UAV RL environment
- [One Net to Rule Them All (April 2025)](https://arxiv.org/abs/2504.21586) - Domain randomization in quadcopter racing
- [What Matters in Zero-Shot Sim-to-Real (May 2025)](https://nicsefc.ee.tsinghua.edu.cn/nics_file/pdf/2bcdb470-f63c-49fd-8d1f-69be970ce82d.pdf) - Key factors for sim-to-real transfer
- [Sim-to-Real Transfer for Variable-Pitch MAV (April 2025)](https://arxiv.org/abs/2504.07694) - RL for complex MAV maneuvers
- [DeepSim Toolkit](https://arxiv.org/abs/2205.08034) - RL environment build toolkit for ROS and Gazebo

### Tools and Frameworks
- [Gymnasium Documentation](https://gymnasium.farama.org/) - Official Gymnasium API
- [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) - Quadcopter RL environments
- [ArduPilot GSoC 2025 Ideas](https://ardupilot.org/dev/docs/gsoc-ideas-list.html) - Includes RL integration proposal
- [pymavlink](https://github.com/ArduPilot/pymavlink) - Python MAVLink interface

### Domain Randomization References
- [Domain Randomization for Sim2Real Transfer](https://lilianweng.github.io/posts/2019-05-05-domain-randomization/) - Comprehensive overview
- [Understanding Domain Randomization](https://arxiv.org/abs/2110.03239) - Theoretical analysis

### Logging and Monitoring
- [Weights & Biases (WandB)](https://wandb.ai/site/experiment-tracking/) - Experiment tracking
- [TensorBoard Integration](https://docs.wandb.ai/guides/integrations/tensorboard/) - TensorBoard with WandB

---

**Document Version**: 1.0.0
**Last Updated**: 2025-12-27
**Author**: Finley Holt
**Project**: Flyby F-11 Autonomous Navigation
