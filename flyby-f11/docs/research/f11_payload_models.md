# Flyby F-11 Payload Models Research

## Document Purpose

This document consolidates research on the Flyby F-11 quadcopter specifications and payload configurations, providing the technical foundation for creating accurate Gazebo simulation models.

## 1. Flyby F-11 Platform Specifications

### 1.1 Physical Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Payload Capacity | 3 kg (6.6 lbs) | Up to 6 lbs usable payload |
| Mounting Hardpoints | 12 | Distributed across three mounting areas |
| Mounting Locations | Top and bottom | Quick-release mounts |
| Flight Endurance | 50-54 minutes | Varies by configuration |
| Battery System | Dual hot-swappable | Enables rapid relaunching |

### 1.2 Compute Platform

| Component | Specification |
|-----------|---------------|
| GPU Module | NVIDIA Jetson Orin NX |
| Processing Power | 50-157 TOPS (variant dependent) |
| Memory | 16GB |
| Storage | 500GB or 1TB NVMe SSD |

### 1.3 Connectivity

- USB 3.0 ports
- Ethernet
- Flight telemetry UART
- Various power outputs (12V, 5V)
- Dual-band RTK GPS

### 1.4 Compliance

- NDAA-compliant supply chain
- F-11H: Compliant with 2020 NDAA Sec 848
- F-11D/F-11S: Compliant with 2023 NDAA Sec 817 and 2024 American Security Drone Act

## 2. Estimated Physical Parameters for Simulation

Based on the 3kg payload capacity and typical quadrotor designs in this class:

### 2.1 Base Frame (Estimated)

| Parameter | Estimated Value | Basis |
|-----------|-----------------|-------|
| Frame Mass (without payload) | 4.5 kg | Typical for 3kg payload capacity |
| Arm Span (motor-to-motor) | 550 mm | Standard for this class |
| Body Dimensions | 300 x 300 x 150 mm | Estimated from similar platforms |
| Propeller Diameter | 15-17 inches (380-430 mm) | For 3kg payload lift capacity |

### 2.2 Moment of Inertia Estimates

Using the standard formulas for a symmetric quadrotor and estimated dimensions:

For a quadrotor with mass M, arm length L, and body approximated as a box:

```
Ixx = (1/12) * M * (height^2 + depth^2) + M_arms * L^2
Iyy = (1/12) * M * (width^2 + depth^2) + M_arms * L^2
Izz = (1/12) * M * (width^2 + height^2) + 4 * M_motor * L^2
```

**Estimated Values for F-11 Base (4.5 kg):**

| Axis | Inertia (kg*m^2) |
|------|------------------|
| Ixx | 0.085 |
| Iyy | 0.085 |
| Izz | 0.145 |

## 3. Payload Configurations

### 3.1 ISR Camera Payload

**Representative Hardware: Sony A7R with Gremsy T3V3 Gimbal**

| Component | Mass | Dimensions |
|-----------|------|------------|
| Sony A7R Camera Body | 407 g | 127 x 94 x 48 mm |
| Gremsy T3V3 Gimbal | 1,200 g (2.65 lbs) | 237 x 184 x 288 mm |
| **Total Payload Mass** | **~1.8 kg** | - |

**Capabilities:**
- Full-frame 36 MP sensor
- 30x optical zoom capability (with appropriate lens)
- 3-axis stabilization
- Pan: ±345°, Tilt: ±120°, Roll: ±45°

### 3.2 Thermal/FLIR Payload

**Representative Hardware: FLIR Hadron 640R+ or Boson+**

| Component | Mass | Dimensions |
|-----------|------|------------|
| FLIR Hadron 640R+ | 56 g | Compact dual-sensor |
| Gremsy Mio Gimbal | 180 g | 133 x 101 x 94 mm |
| **Total Payload Mass** | **~250 g** | - |

**Alternative (Heavier Thermal):**

| Component | Mass | Dimensions |
|-----------|------|------------|
| FLIR Vue TV128 (640x512) | ~400 g | - |
| Gremsy S1V3 Gimbal | ~600 g | - |
| **Total Payload Mass** | **~1.0 kg** | - |

### 3.3 Multispectral Payload

**Representative Hardware: MicaSense RedEdge-P**

| Parameter | Value |
|-----------|-------|
| Mass | 300 g (10.6 oz) |
| Dimensions | 82 x 62 x 54 mm |
| Bands | Blue, Green, Red, Red Edge, NIR, Panchromatic |
| Resolution | 2 cm GSD at 60 m (pan-sharpened) |
| Power | 7.0 - 25.2 V DC |

**Total Payload with Mount:**

| Component | Mass |
|-----------|------|
| RedEdge-P Camera | 300 g |
| DLS 2 Light Sensor | 50 g |
| Mounting Hardware | 100 g |
| **Total** | **~450 g** |

### 3.4 LiDAR Payload

**Representative Hardware: Ouster OS1-64**

| Parameter | Value |
|-----------|-------|
| Mass | 482 g |
| Diameter | ~85 mm |
| Height | ~73 mm (with thermal cap) |
| Channels | 32/64/128 options |
| Range | 90 m @ 10% reflectivity, 200 m max |
| FOV | 360° horizontal, 45° vertical |
| Points/Second | Up to 5.2 million |

**Total Payload with Mount:**

| Component | Mass |
|-----------|------|
| Ouster OS1-64 | 482 g |
| Mounting Hardware | 150 g |
| Processing Interface | 100 g |
| **Total** | **~750 g** |

**Note:** The Flyby AEGIS variant specifically integrates the Ouster OS-1 for 360-degree obstacle avoidance.

### 3.5 Communications Relay Payload

**Representative Hardware: SDR-based Relay**

| Component | Mass |
|-----------|------|
| SDR Transceiver | 145-200 g |
| Antennas (2x) | 100 g |
| Power Management | 150 g |
| **Total** | **~450-500 g** |

**Capabilities:**
- Frequency range: 900 MHz - 5.8 GHz
- Mesh networking capability
- MIMO support
- Up to 250 km LOS (with amplifiers)

## 4. Gazebo Modeling Approach

### 4.1 Model Structure

Each F-11 variant follows this SDF structure:

```
f11_<variant>/
├── model.config       # Model metadata
├── model.sdf          # Model definition
└── meshes/            # Optional visual meshes
    └── *.dae or *.stl
```

### 4.2 SDF Components

#### Base Frame
- `base_link` with mass/inertia properties
- 4 rotor links with motor dynamics
- IMU sensor (required for ArduPilot)

#### Payload Integration
- Payload link attached via fixed joint
- Payload-specific sensors (camera, LiDAR, etc.)
- Updated total mass and recalculated inertia

### 4.3 ArduPilot Plugin Configuration

```xml
<plugin filename="ArduPilotPlugin" name="ArduPilotPlugin">
  <fdm_addr>127.0.0.1</fdm_addr>
  <fdm_port_in>9002</fdm_port_in>
  <fdm_port_out>9003</fdm_port_out>
  <modelXYZToAirplaneXForwardZDown>0 0 0 3.141593 0 0</modelXYZToAirplaneXForwardZDown>
  <gazeboXYZToNED>0 0 0 3.141593 0 0</gazeboXYZToNED>
  <imuName>imu_link::imu_sensor</imuName>
  <!-- Motor controls for each rotor -->
</plugin>
```

### 4.4 Sensor Plugins (Gazebo Harmonic)

**Camera Sensor:**
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>1920</width>
      <height>1080</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>500</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
</sensor>
```

**LiDAR Sensor:**
```xml
<sensor name="lidar" type="gpu_lidar">
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>64</samples>
        <resolution>1</resolution>
        <min_angle>-0.3927</min_angle>
        <max_angle>0.3927</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.3</min>
      <max>200</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
</sensor>
```

## 5. Mass and Inertia Calculations

### 5.1 Combined System Calculations

For each payload variant, the total mass is:

```
M_total = M_base + M_payload
```

The center of gravity shifts based on payload position:

```
CoG_z = (M_base * z_base + M_payload * z_payload) / M_total
```

Inertia is recalculated using parallel axis theorem:

```
I_total = I_base + I_payload + M_payload * d^2
```

where `d` is the distance from payload CoG to vehicle CoG.

### 5.2 Summary of Variant Masses

| Variant | Base Mass | Payload Mass | Total Mass |
|---------|-----------|--------------|------------|
| f11_base | 4.5 kg | 0 kg | 4.5 kg |
| f11_isr_camera | 4.5 kg | 1.8 kg | 6.3 kg |
| f11_multispectral | 4.5 kg | 0.45 kg | 4.95 kg |
| f11_lidar | 4.5 kg | 0.75 kg | 5.25 kg |
| f11_comms_relay | 4.5 kg | 0.5 kg | 5.0 kg |

## 6. References

### Official Flyby Resources
- [Flyby Robotics Official Site](https://www.flybyrobotics.com/)
- [Flyby F-11 Spec Sheet (PDF)](https://www.flybyrobotics.com/documents/Flyby_F-11.pdf)
- [Flyby Developer Documentation](https://docs.flybydev.com/quickstart/)

### Third-Party Coverage
- [Unmanned Systems Technology - Flyby F-11](https://www.unmannedsystemstechnology.com/company/flyby-robotics/flyby-f-11-drone/)
- [BAVOVNA - Flyby F-11 Drone](https://bavovna.ai/uav/flyby-f-11-drone/)

### Sensor Datasheets
- [Ouster OS1 LiDAR](https://ouster.com/products/hardware/os1-lidar-sensor)
- [MicaSense RedEdge-P](https://wingtra.com/mapping-drone-wingtraone/drone-sensors/micasense-rededge-p/)
- [FLIR Hadron 640R+](https://oem.flir.com/)
- [Gremsy Gimbals](https://gremsy.com/)

### Gazebo Resources
- [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [Gazebo Harmonic Sensors](https://gazebosim.org/docs/harmonic/sensors/)
- [SDFormat Specification](http://sdformat.org/spec)

### Academic References
- Meyer, J., et al. "Comprehensive Simulation of Quadrotor UAVs Using ROS and Gazebo." SIMPAR 2012.
- Wilselby.com "ROS Integration - Quadrotor Model Dynamics & Sensor Simulation"

---

*Document created: December 2025*
*Author: Finley Holt*
