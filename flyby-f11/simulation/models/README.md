# Flyby F-11 Gazebo Simulation Models

This directory contains Gazebo Harmonic simulation models for the Flyby F-11 quadcopter platform with various payload configurations.

## Overview

The Flyby F-11 is an NDAA-compliant quadcopter with a 3 kg payload capacity, designed for versatile mission profiles. These simulation models enable testing of autonomous behaviors, sensor integration, and flight dynamics before deploying to real hardware.

## Model Variants

| Model | Description | Total Mass | Payload |
|-------|-------------|------------|---------|
| `f11_base` | Baseline platform (no payload) | 4.5 kg | None |
| `f11_isr_camera` | ISR camera with gimbal | 6.3 kg | Sony A7R + Gremsy T3V3 |
| `f11_multispectral` | Agricultural imaging | 4.95 kg | MicaSense RedEdge-P |
| `f11_lidar` | 3D mapping and obstacle avoidance | 5.25 kg | Ouster OS1-64 |
| `f11_comms_relay` | Communications relay | 5.0 kg | SDR + dual antennas |

## Model Structure

Each model directory contains:

```
f11_<variant>/
├── model.config    # Model metadata (name, author, description)
└── model.sdf       # Simulation Description Format model definition
```

## Quick Start

### Spawning a Model in Gazebo Harmonic

1. Set environment variables:
   ```bash
   export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/flyby-f11/simulation/models
   ```

2. Launch Gazebo with a world:
   ```bash
   gz sim -r /path/to/flyby-f11/simulation/worlds/runway.sdf
   ```

3. Spawn a model:
   ```bash
   gz service -s /world/runway/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf_filename: "f11_base/model.sdf", name: "f11_base", pose: {position: {z: 0.3}}'
   ```

### Using with ArduPilot SITL

1. Start ArduPilot SITL:
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
   ```

2. The model will connect to SITL via the ArduPilot plugin on ports 9002/9003.

3. For multiple drones, modify `fdm_port_in` and `fdm_port_out` in each model's SDF.

## Model Details

### f11_base

The baseline F-11 platform without any mission payload. Use this for:
- Basic flight testing
- Controller development
- Baseline dynamics validation

**Key Parameters:**
- Mass: 4.5 kg
- Arm span: 550 mm (motor-to-motor)
- Propeller diameter: 380 mm

### f11_isr_camera

ISR (Intelligence, Surveillance, Reconnaissance) payload with stabilized high-resolution camera.

**Payload Components:**
- Sony A7R camera (36 MP full-frame, 407g)
- Gremsy T3V3 3-axis gimbal (1,200g)
- Pan: +/-345 degrees
- Tilt: +/-120 degrees
- Roll: +/-45 degrees

**Simulation Sensors:**
- 1920x1080 RGB camera at 30 Hz
- IMU, GPS, NavSat

### f11_multispectral

Agricultural and environmental monitoring payload with multispectral imaging.

**Payload Components:**
- MicaSense RedEdge-P (300g)
- 6 spectral bands: Blue, Green, Red, Red Edge, NIR, Panchromatic
- DLS 2 downwelling light sensor (50g)

**Simulation Sensors:**
- 1280x960 camera at 1 Hz (mapping rate)
- IMU, GPS, NavSat

### f11_lidar

3D mapping and precision navigation payload based on the Flyby AEGIS configuration.

**Payload Components:**
- Ouster OS1-64 LiDAR (482g)
- 64 channels
- 360-degree horizontal FOV
- 45-degree vertical FOV
- Range: 90m @ 10% reflectivity, 200m max

**Simulation Sensors:**
- GPU-accelerated LiDAR (1024 horizontal x 64 vertical samples)
- 10 Hz update rate
- IMU, GPS, NavSat

### f11_comms_relay

Communications relay payload for extending network coverage.

**Payload Components:**
- SDR transceiver (200g)
- Dual omnidirectional antennas (100g)
- Power management module (150g)

**Simulation Features:**
- Visual antenna representation
- IMU, GPS, NavSat

## Technical Specifications

### Common Features (All Models)

- **ArduPilot Integration:** Pre-configured ArduPilotPlugin for SITL connection
- **IMU Sensor:** 250 Hz update rate with realistic noise model
- **GPS Sensor:** NavSat sensor at 10 Hz
- **Rotor Dynamics:** 4 motors with velocity control and damping

### Physical Properties

| Property | Value |
|----------|-------|
| Arm Span | 550 mm |
| Propeller Diameter | 380 mm |
| Body Size | 300 x 300 x 120 mm |
| Base Frame Mass | 4.5 kg |

### Inertia Values

Inertia tensors are calculated for each variant based on:
- Frame geometry approximation
- Payload mass and position
- Parallel axis theorem for offset payloads

## Validation

Run the model validator to check all models:

```bash
cd /path/to/flyby-f11/simulation
python scripts/validate_models.py --all
```

Or validate a specific model:

```bash
python scripts/validate_models.py --model f11_base
```

The validator checks:
- File structure (model.config, model.sdf)
- XML validity
- Required links and joints
- Mass and inertia reasonableness
- ArduPilot plugin configuration
- Sensor configurations

## Customization

### Adding a New Payload Variant

1. Copy an existing model directory:
   ```bash
   cp -r f11_base f11_custom_payload
   ```

2. Edit `model.config` with new name and description

3. Modify `model.sdf`:
   - Update `<model name="...">`
   - Adjust `base_link` mass for total weight
   - Recalculate inertia values
   - Add payload links and joints
   - Add sensor elements as needed

4. Validate:
   ```bash
   python scripts/validate_models.py --model f11_custom_payload
   ```

### Multi-UAV Simulation

For simulating multiple F-11 drones:

1. Each instance needs unique ArduPilot ports:
   - Drone 1: ports 9002/9003
   - Drone 2: ports 9012/9013
   - Drone 3: ports 9022/9023

2. Modify `fdm_port_in` and `fdm_port_out` in the model SDF

3. Start multiple SITL instances with corresponding `-I` parameter

## References

- [Flyby Robotics Official](https://www.flybyrobotics.com/)
- [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [SDFormat Specification](http://sdformat.org/)

## License

These models are developed for the DroneProjects portfolio. See the root repository LICENSE for details.

---

*Models created: December 2025*
*Author: Finley Holt*
