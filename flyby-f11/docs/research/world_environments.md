# Gazebo World Environments for UAV RL Training

**Research Date**: December 2025
**Author**: Finley Holt
**Purpose**: Document best practices and resources for Gazebo Harmonic UAV training environments

## Executive Summary

This document synthesizes research on Gazebo simulation environments for UAV reinforcement learning training. Key findings indicate that procedural generation, domain randomization, and performance optimization are critical for effective RL training. The recommendations prioritize simulation throughput while maintaining sufficient visual and physical fidelity for sim-to-real transfer.

---

## 1. Best Existing Gazebo Harmonic World Repositories (2025)

### 1.1 DARPA SubT Challenge Worlds

The DARPA Subterranean Challenge produced the most comprehensive open-source Gazebo environments for autonomous robot training.

**Repository**: [osrf/subt](https://github.com/osrf/subt)
**Highlights**:
- 200+ new models freely available at SubT Tech Repo
- Three environment types: Urban, Cave, Man-made tunnels
- High-fidelity photogrammetry and laser-scanned assets
- Multi-robot support (58 robots including 37 aerial)
- Battle-tested in competition with 7,781 hours of simulation

**Relevant Resources**:
- [AirLab SubT UAV Code Release](https://theairlab.org/research/2022/05/02/subt_code/) - UAV-specific environments including room.world, hawkins_qualification.world, filmmakers2.world
- [subt_hello_world](https://github.com/osrf/subt_hello_world) - Quick-start tutorials

**Applicability to Flyby F-11**: SubT environments are excellent for GPS-denied/indoor navigation training but may be too complex for initial RL training. Recommend using simpler custom worlds first, then progressive curriculum learning with SubT environments.

### 1.2 PX4/ArduPilot Community Worlds

**PX4 SITL Gazebo Repository**: [PX4/PX4-SITL_gazebo-classic](https://github.com/PX4/PX4-SITL_gazebo-classic)
**Documentation**: [PX4 Gazebo Worlds](https://docs.px4.io/v1.12/en/simulation/gazebo_worlds)

**Available Worlds**:
- `empty.world` - Flat plane for basic testing
- `warehouse.world` - Indoor logistics environment
- `uuv_hippocampus.world` - Underwater simulation
- `typhoon_h480.world` - Video streaming enabled
- `iris_irlock.world` - Precision landing with IR beacon

**ArduPilot Models**: [ArduPilot Gazebo Plugin](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- Iris quadcopter and Zephyr delta-wing included
- Garden and Harmonic support confirmed

### 1.3 Research Community Worlds

**MuSHR Racing**: University of Washington's indoor racing environment
**TartanAir**: CMU AirLab's diverse aerial environments
**Fuel Models Database**: Gazebo's official model repository with 500+ assets

---

## 2. Procedural World Generation Tools

### 2.1 PCG Gazebo (Bosch Research) - ARCHIVED

**Repository**: [boschresearch/pcg_gazebo](https://github.com/boschresearch/pcg_gazebo) *(Archived January 2025)*
**Documentation**: [PCG Gazebo Examples](https://boschresearch.github.io/pcg_gazebo/examples/)

**Key Features**:
- Python-based world generation
- Model placement policies with constraint satisfaction
- Collision-aware obstacle placement
- SDF/URDF generation from Python objects
- Reproducible generation with seeds

**Limitations**:
- **Archived**: Repository was archived by Bosch Research on January 20, 2025
- ROS 1 focused (rospy dependency for runtime features)
- No native ROS 2 support
- Still installable via pip but no longer actively maintained
- Works best for static world generation (our use case)

**Example Usage**:
```python
from pcg_gazebo.generators import WorldGenerator

world_gen = WorldGenerator()
world_gen.add_engine(
    engine_name='random_pose',
    models=['cylinder', 'box'],
    max_num={'cylinder': 10, 'box': 5},
    model_picker='random',
    no_collision=True,
    min_distance=2.0
)
world_gen.run_engines()
world_gen.export_world('/path/to/output.world')
```

### 2.2 Blender Procedural Datasets

**Documentation**: [Gazebo Blender Procedural Datasets](https://gazebosim.org/api/gazebo/6/blender_procedural_datasets.html)

**Use Case**: Generate nearly unlimited 3D model variety for domain randomization. Recommended for creating diverse obstacle textures and shapes.

### 2.3 Custom Python SDF Generation

For Gazebo Harmonic (SDF 1.9+), direct Python generation of SDF XML is often simpler than external tools. Our `generate_world_variants.py` script implements this approach.

---

## 3. Challenging Obstacle Courses for RL

### 3.1 Design Principles

Based on literature review of RL navigation training:

1. **Progressive Difficulty**: Start with sparse obstacles, increase density
2. **Variety in Shape**: Mix cylinders, boxes, spheres, L-shapes
3. **Height Variation**: Critical for 3D UAV navigation
4. **Narrow Passages**: Force precise control
5. **Dead Ends**: Test backtracking and replanning
6. **Dynamic Elements**: Moving obstacles for advanced training

### 3.2 Recommended Obstacle Types

| Type | Purpose | Difficulty |
|------|---------|------------|
| Thin poles | Precision avoidance | Medium |
| Wide buildings | Path planning | Easy |
| Narrow gaps | Fine control | Hard |
| Overhangs | 3D awareness | Hard |
| L-shaped walls | Corner navigation | Medium |
| Tree canopies | Altitude decisions | Medium |
| Moving platforms | Reactive control | Very Hard |

### 3.3 Training Curriculum

1. **Phase 1**: Empty world with geofence only
2. **Phase 2**: Sparse static obstacles (5-10)
3. **Phase 3**: Dense static obstacles (20-30)
4. **Phase 4**: NFZ integration
5. **Phase 5**: Dynamic obstacles (moving)
6. **Phase 6**: Weather effects (wind, visibility)

---

## 4. RL-Specific World Generators

### 4.1 DeepSim Toolkit

**Paper**: [DeepSim: A RL Environment Build Toolkit for ROS and Gazebo](https://arxiv.org/abs/2205.08034)

**Features**:
- Domain randomization built-in
- Collision detection helpers
- Behavior control abstractions
- Spawner utilities for episode reset

### 4.2 YamaS Simulator

**Research**: [Advancing Behavior Generation in Mobile Robotics](https://arxiv.org/html/2405.16818)

**Features**:
- Unity3D + ROS/ROS2 integration
- Procedural generation with NLP
- VR support for teleoperation
- Multi-agent scenarios

### 4.3 OpenAI Gym + ROS + Gazebo

**Tutorials**: [Reinforcement Learning with ROS and Gazebo](https://github.com/vmayoral/basic_reinforcement_learning/blob/master/tutorial7/README.md)

Standard pattern for Gym-compatible RL environments using Gazebo as physics backend.

---

## 5. Visual Fidelity vs Performance Trade-offs

### 5.1 Key Research Findings

From [Robot Learning From Randomized Simulations: A Review](https://www.frontiersin.org/articles/10.3389/frobt.2022.799893/full):

> "High-fidelity renderings lead to marked performance improvements - error reductions of an order of magnitude when increasing rendering quality from level 1 to 8."

However, from [Domain Randomization for Sim2Real Transfer](https://lilianweng.github.io/posts/2019-05-05-domain-randomization/):

> "Low-fidelity simulation can sometimes be advantageous. Robust visual-legged navigation policies can be learned with low-fidelity kinematic-only simulation."

### 5.2 Recommendations for Flyby F-11

| Setting | Training | Evaluation | Rationale |
|---------|----------|------------|-----------|
| Resolution | 640x480 | 1280x720 | Speed vs accuracy |
| Shadows | Off | On | Major perf impact |
| Physics Hz | 1000 | 1000 | Stable dynamics |
| Real-time factor | 2-4x | 1x | Training throughput |
| Render engine | ogre2 | ogre2 | Harmonic default |

### 5.3 Domain Randomization Strategy

Critical parameters to randomize:
1. **Lighting**: Direction, intensity, color temperature
2. **Textures**: Ground, obstacles, skybox
3. **Physics**: Wind, turbulence, sensor noise
4. **Geometry**: Obstacle positions, sizes (within bounds)
5. **Camera**: Noise, exposure, color jitter

---

## 6. DARPA SubT and Community Assets

### 6.1 SubT Model Library

**Location**: [SubT Tech Repo](https://subtchallenge.world/openrobotics/fuel/models)

**Recommended Models**:
- Cave tiles (procedural cave generation)
- Urban tunnel sections
- Debris and rubble
- Barrels and containers
- Communication nodes
- Artifact markers

### 6.2 ArduPilot Community Resources

**Forum**: [ArduPilot Discourse - Simulation](https://discuss.ardupilot.org/c/simulation/72)

**Key Threads**:
- Multi-UAV simulation in Gazebo Harmonic
- Custom motor control plugins
- Sensor integration guides

### 6.3 Fuel Model Database

**URL**: [app.gazebosim.org](https://app.gazebosim.org/fuel/models)

**Useful Categories**:
- Vehicles (reference models)
- Buildings (urban training)
- Nature (trees, rocks)
- Industrial (warehouses, equipment)

---

## 7. Weather and Environmental Effects

### 7.1 Wind Simulation

Gazebo Harmonic supports global wind models configurable per-location.

```xml
<plugin filename="gz-sim-wind-effects-system"
        name="gz::sim::systems::WindEffects">
  <horizontal>
    <magnitude><time_function>LINEAR</time_function>
    <direction><time_function>SINE</time_function>
  </horizontal>
</plugin>
```

### 7.2 Fog and Particle Effects

**Documentation**: [Gazebo Particle Emitter](https://gazebosim.org/api/sim/8/particle_emitter.html)

Particle emitters create fog effects that interact with sensors:
- RGB cameras: Visual fog rendering
- RGBD cameras: Noisy depth readings from scattering
- LiDAR: Range affected by particle scatter
- Thermal: Particles not detected

```xml
<particle_emitter name="fog_emitter" type="box">
  <emitting>true</emitting>
  <size>50 50 10</size>
  <particle_size>1 1 1</particle_size>
  <lifetime>10</lifetime>
  <rate>50</rate>
  <particle_scatter_ratio>0.5</particle_scatter_ratio>
</particle_emitter>
```

### 7.3 Lighting Conditions

**Skybox Customization**: SDF supports custom skybox textures for different times of day.

**Directional Light**: Adjustable sun position for shadow variations.

---

## 8. Recommendations for Flyby F-11 Project

### 8.1 Immediate Actions

1. **Use our custom world generator** (`generate_world_variants.py`) for initial training
2. **Implement domain randomization** for obstacle placement and lighting
3. **Optimize physics settings** for 2-4x real-time training
4. **Disable shadows** during training, enable for evaluation

### 8.2 Medium-term Improvements

1. **Integrate SubT cave tiles** for GPS-denied training scenarios
2. **Add wind effects plugin** for disturbance rejection training
3. **Implement curriculum learning** with progressive difficulty
4. **Create urban environment** for ISR mission simulation

### 8.3 Long-term Goals

1. **Photorealistic rendering** for sim-to-real validation
2. **Multi-UAV swarm training** environments
3. **Hardware-in-the-loop** integration with real sensor data
4. **Digital twin** of actual deployment environments

---

## 9. References

### Primary Sources

1. [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted/)
2. [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
3. [PX4 Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
4. [DARPA SubT Repository](https://github.com/osrf/subt)
5. [PCG Gazebo](https://github.com/boschresearch/pcg_gazebo)

### Research Papers

1. [Domain Randomization for Transferring DNNs from Simulation to Real World](https://arxiv.org/abs/1703.06907)
2. [Robot Learning From Randomized Simulations: A Review](https://www.frontiersin.org/articles/10.3389/frobt.2022.799893/full)
3. [DeepSim: A RL Environment Build Toolkit](https://arxiv.org/abs/2205.08034)
4. [A Subterranean Virtual Cave World for Gazebo](https://arxiv.org/abs/2004.08452)

### Community Resources

1. [Gazebo Fuel Models](https://app.gazebosim.org/fuel/models)
2. [ArduPilot Discourse](https://discuss.ardupilot.org/)
3. [Open Robotics Discourse](https://discourse.openrobotics.org/)
4. [Karelics: Using Gazebo for RL](https://karelics.fi/using-gazebo-for-reinforcement-learning/)

---

*Document generated as part of Flyby F-11 Phase 11: Simulation Benchmark development.*
