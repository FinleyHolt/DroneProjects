# ArduPilot Gazebo Plugin - Advanced Features Research

**Author**: Finley Holt
**Date**: 2025-12-27
**Platform**: Flyby F-11 / Gazebo Harmonic

## Executive Summary

This document provides a comprehensive analysis of the ArduPilot Gazebo plugin's advanced features, sensor modeling capabilities, environmental effect simulation, and performance optimization strategies for multi-drone simulations in Gazebo Harmonic (LTS).

---

## 1. ArduPilot Gazebo Plugin Overview

### 1.1 Supported Gazebo Versions

The official ArduPilot Gazebo plugin supports:
- Gazebo Garden
- **Gazebo Harmonic (LTS)** - Recommended for production use
- Gazebo Ionic
- Gazebo Jetty (LTS)

**Note**: Gazebo Classic reached end-of-life in January 2025. Migration to modern Gazebo is strongly recommended.

### 1.2 Key Features

| Feature | Description |
|---------|-------------|
| JSON Interface | Flexible data exchange between SITL and Gazebo using JSON protocol |
| Lock-step Simulation | True simulation lockstepping for deterministic behavior |
| 32-Channel Support | Support for up to 32 servo channels |
| Debugging Support | GDB integration - can pause Gazebo time for debugging |
| ogre2 Rendering | Improved 3D rendering with ogre2 engine |

### 1.3 Plugin Architecture

The ArduPilotPlugin operates as a bidirectional bridge:

```
ArduPilot SITL <---> UDP/JSON <---> ArduPilotPlugin <---> Gazebo Physics
```

Communication uses custom JSON over UDP sockets for high-frequency data exchange, replacing traditional MAVLink with a format optimized for simulation.

### 1.4 Plugin Configuration

```xml
<plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
  <!-- Network Configuration -->
  <fdm_addr>127.0.0.1</fdm_addr>
  <fdm_port_in>9002</fdm_port_in>
  <fdm_port_out>9003</fdm_port_out>
  <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>

  <!-- Simulation Settings -->
  <lock_step>1</lock_step>
  <have_32_channels>0</have_32_channels>

  <!-- Coordinate Transforms -->
  <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
  <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>

  <!-- Sensor Reference -->
  <imuName>base_link::imu_sensor</imuName>
</plugin>
```

---

## 2. Sensor Modeling in Gazebo Harmonic

### 2.1 World-Level Sensor Plugins

Gazebo Harmonic requires explicit sensor system plugins at the world level:

```xml
<world name="training_world">
  <!-- Core Systems -->
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

  <!-- Sensor Systems -->
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
  <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
  <plugin filename="gz-sim-altimeter-system" name="gz::sim::systems::Altimeter"/>
  <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
</world>
```

### 2.2 Sensor Types and Configuration

#### 2.2.1 IMU Sensor

The Inertial Measurement Unit provides orientation (quaternion), angular velocity, and linear acceleration.

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>250</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.017</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.017</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.017</stddev></noise></z>
    </linear_acceleration>
  </imu>
</sensor>
```

**Noise Parameters** (typical values for MEMS IMU):
- Angular velocity: stddev 0.005-0.015 rad/s
- Linear acceleration: stddev 0.01-0.03 m/s^2

#### 2.2.2 NavSat (GPS) Sensor

GPS provides latitude, longitude, and altitude with configurable noise.

```xml
<sensor name="navsat_sensor" type="navsat">
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <navsat>
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.5</stddev>
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.0</stddev>
        </noise>
      </vertical>
    </position_sensing>
    <velocity_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
        </noise>
      </vertical>
    </velocity_sensing>
  </navsat>
</sensor>
```

**Typical GPS Accuracy**:
- Consumer GPS: 2-5m horizontal, 5-10m vertical
- RTK GPS: 0.01-0.02m horizontal

#### 2.2.3 Magnetometer Sensor

Magnetometer measures Earth's magnetic field for heading estimation.

```xml
<sensor name="magnetometer_sensor" type="magnetometer">
  <always_on>1</always_on>
  <update_rate>50</update_rate>
  <magnetometer>
    <x><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></x>
    <y><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></y>
    <z><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></z>
  </magnetometer>
</sensor>
```

**Note**: Magnetometer declination is location-dependent. For San Diego area, declination is approximately 11.5 degrees East.

#### 2.2.4 Air Pressure (Barometer) Sensor

Barometric pressure sensor for altitude estimation.

```xml
<sensor name="air_pressure_sensor" type="air_pressure">
  <always_on>1</always_on>
  <update_rate>20</update_rate>
  <air_pressure>
    <reference_altitude>0</reference_altitude>
    <pressure>
      <noise type="gaussian">
        <mean>0</mean>
        <stddev>10</stddev>
      </noise>
    </pressure>
  </air_pressure>
</sensor>
```

**Pressure-Altitude Relationship**:
- ~8.3 Pa per meter at sea level
- Typical barometer noise: 10-50 Pa (1-6m equivalent)

#### 2.2.5 Altimeter Sensor

Direct altitude measurement with reference altitude offset.

```xml
<sensor name="altimeter_sensor" type="altimeter">
  <always_on>1</always_on>
  <update_rate>20</update_rate>
  <altimeter>
    <vertical_position>
      <noise type="gaussian">
        <mean>0</mean>
        <stddev>0.1</stddev>
      </noise>
    </vertical_position>
    <vertical_velocity>
      <noise type="gaussian">
        <mean>0</mean>
        <stddev>0.1</stddev>
      </noise>
    </vertical_velocity>
  </altimeter>
</sensor>
```

#### 2.2.6 Rangefinder (Lidar) Sensor

Downward-facing rangefinder for precise altitude estimation.

```xml
<sensor name="rangefinder" type="gpu_lidar">
  <pose>0 0 -0.05 0 1.5708 0</pose>  <!-- Pointing down -->
  <always_on>1</always_on>
  <update_rate>20</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>40.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0</mean>
      <stddev>0.02</stddev>
    </noise>
  </ray>
</sensor>
```

### 2.3 Noise Model Types

Gazebo supports three noise model types:

| Type | Description |
|------|-------------|
| `none` | No noise added |
| `gaussian` | Independent Gaussian noise per measurement |
| `gaussian_quantized` | Gaussian + quantization (ADC effects) |

**Gaussian Parameters**:
- `mean`: Mean of distribution (typically 0 for zero-mean noise)
- `stddev`: Standard deviation
- `bias_mean`: Mean of bias distribution
- `bias_stddev`: Standard deviation of bias

---

## 3. Environmental Effects

### 3.1 Wind Simulation

Wind can be simulated using Gazebo plugins that apply forces to the vehicle.

#### 3.1.1 Constant Wind

```xml
<plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
  <force_approximation_scaling_factor>1.0</force_approximation_scaling_factor>
  <horizontal>
    <magnitude>
      <time_for_rise>10</time_for_rise>
      <sin>
        <amplitude_percent>0.05</amplitude_percent>
        <period>60</period>
      </sin>
      <noise type="gaussian">
        <mean>0</mean>
        <stddev>0.1</stddev>
      </noise>
    </magnitude>
    <direction>
      <time_for_rise>30</time_for_rise>
      <sin>
        <amplitude>5</amplitude>
        <period>20</period>
      </sin>
      <noise type="gaussian">
        <mean>0</mean>
        <stddev>0.5</stddev>
      </noise>
    </direction>
  </horizontal>
  <vertical>
    <noise type="gaussian">
      <mean>0</mean>
      <stddev>0.03</stddev>
    </noise>
  </vertical>
</plugin>
```

#### 3.1.2 Turbulence Modeling

For more realistic turbulence:
- Use Gaussian noise with time-varying magnitude
- Apply CFD-generated wind fields for urban environments
- Model gusts as impulse disturbances

### 3.2 ArduPilot SITL Wind Parameters

ArduPilot SITL provides built-in wind simulation:

```
SIM_WIND_SPD       # Wind speed (m/s)
SIM_WIND_DIR       # Wind direction (degrees)
SIM_WIND_TURB      # Turbulence intensity (0-1)
SIM_WIND_T         # Wind type (0=constant, 1=gust)
```

---

## 4. PID Tuning in Simulation

### 4.1 ArduPilot Control Architecture

```
Rate PID <-- Attitude PID <-- Position PID <-- Navigation
```

Key parameters for multicopter:
- `ATC_RAT_*_P/I/D`: Rate controller gains
- `ATC_ANG_*_P`: Attitude controller gains
- `PSC_*`: Position controller gains

### 4.2 Tuning Methodology

1. **Start with rate controllers** (innermost loop)
2. **Tune attitude controllers** (outer loop)
3. **Tune position controllers** (outermost loop)

### 4.3 Performance Metrics

| Metric | Description | Target |
|--------|-------------|--------|
| Overshoot | Maximum deviation beyond setpoint | < 10% |
| Settling Time | Time to reach steady state | < 2s |
| Steady-State Error | Final offset from setpoint | < 1% |
| Rise Time | Time to reach setpoint | < 1s |

### 4.4 Simulation Speed Control

```
SIM_SPEEDUP=1.0   # Real-time
SIM_SPEEDUP=5.0   # 5x faster for rapid testing
SIM_SPEEDUP=0.1   # Slow-motion for debugging
```

---

## 5. Multi-Drone Performance Optimization

### 5.1 Port Configuration

Each drone requires unique ports:

| Drone | fdm_port_in | fdm_port_out | SITL Instance |
|-------|-------------|--------------|---------------|
| 1 | 9002 | 9003 | -I0 |
| 2 | 9012 | 9013 | -I1 |
| 3 | 9022 | 9023 | -I2 |
| N | 9002+(N-1)*10 | 9003+(N-1)*10 | -I(N-1) |

### 5.2 Performance Recommendations

1. **Reduce sensor update rates** for non-critical sensors
2. **Use GPU-accelerated rendering** (ogre2)
3. **Limit physics step size** to 0.001s
4. **Disable visualization** for headless training
5. **Use separate process threads** per drone

### 5.3 Typical Multi-Drone Limits

| Hardware | Max Drones (Real-time) |
|----------|----------------------|
| Consumer laptop | 2-3 |
| Workstation (8-core) | 5-8 |
| Server (16-core + GPU) | 10-20 |

---

## 6. GSoC 2025 Developments

Recent developments from Google Summer of Code 2025:
- **BLDC Motor Model Plugin**: Physics-based motor modeling for improved fidelity
- **Enhanced rotor aerodynamics**: Better thrust/torque modeling
- **Thermal effects**: Motor heating simulation

---

## 7. References

- [ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) - Official ArduPilot Gazebo plugin
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/) - Official Gazebo docs
- [SDFormat Specification](http://sdformat.org/spec) - SDF element reference
- [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/sitl-with-gazebo.html) - SITL setup guide
- [Gazebo Sensors Library](https://gazebosim.org/libs/sensors/) - Sensor API reference
- [MOGI-ROS Gazebo Sensors Tutorial](https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors) - Sensor examples

---

## 8. Appendix: Sensor Profile Recommendations

### 8.1 High-Fidelity Profile
For validation and testing:
- Realistic sensor noise based on datasheet specifications
- Environmental effects enabled
- Full physics simulation

### 8.2 Low-Fidelity Profile
For debugging and rapid iteration:
- Minimal or no noise
- Simplified physics
- Faster-than-real-time execution

### 8.3 Adversarial Profile
For robust training:
- Exaggerated noise (2-5x typical values)
- Sensor dropouts and failures
- Strong wind and turbulence
