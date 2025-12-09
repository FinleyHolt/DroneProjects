# Test Drone X500 Model

Gazebo Harmonic model for the test-drone platform. Based on PX4's x500 quadcopter with simulated Intel RealSense sensors matching real hardware.

## Sensors

### Intel RealSense T265 (Visual Odometry)
- **Location**: Front of drone (0.15m forward from base_link)
- **Purpose**: 6DOF pose tracking (position + orientation)
- **Update Rate**: 200 Hz odometry
- **Cameras**: Dual fisheye cameras (848x800 @ 30Hz)
- **Gazebo Topics**:
  - `/t265/odom` - Odometry (nav_msgs/Odometry)
  - `/t265/fisheye1/image_raw` - Left fisheye image
  - `/t265/fisheye2/image_raw` - Right fisheye image

### Intel RealSense D455 (Depth Camera)
- **Location**: Front of drone, below T265 (0.15m forward, 0.05m down from base_link)
- **Purpose**: RGB-D imaging for obstacle detection and perception
- **Update Rate**: 30 Hz
- **Resolution**: 640x480 (RGB and depth)
- **Depth Range**: 0.3m to 10m
- **Gazebo Topics**:
  - `/d455/color/image_raw` - RGB image (sensor_msgs/Image)
  - `/d455/depth` - Depth image (sensor_msgs/Image, R_FLOAT32)
  - `/d455/infra1/image_raw` - Left IR image
  - `/d455/infra2/image_raw` - Right IR image

## Using the Model

### In PX4 Gazebo SITL

```bash
# From PX4-Autopilot directory
PX4_GZ_MODEL_PATH=/workspace/simulation/models make px4_sitl gz_test_drone_x500
```

### With ros_gz Bridge

ROS 2 topics are automatically bridged from Gazebo topics:

```bash
# List available topics
ros2 topic list

# View depth camera feed
ros2 run rqt_image_view rqt_image_view /d455/color/image_raw

# Echo odometry
ros2 topic echo /t265/odom
```

### Custom Worlds

Add to your world SDF file:

```xml
<include>
  <uri>model://test_drone_x500</uri>
  <name>test_drone</name>
  <pose>0 0 0.5 0 0 0</pose>
</include>
```

## Sensor Specifications

### T265 Physical Specs
- **Mass**: 55g
- **Dimensions**: 24mm x 108mm x 13mm
- **FOV**: 163° ± 5° (fisheye cameras)
- **Tracking**: Visual-inertial SLAM

### D455 Physical Specs
- **Mass**: 95g
- **Dimensions**: 26mm x 124mm x 29mm
- **Depth FOV**: 86° x 57°
- **RGB FOV**: 86° x 57°
- **Baseline**: 50mm (stereo cameras)

## ROS 2 Integration

The `ros_gz` bridge automatically publishes Gazebo sensor data to ROS 2 topics. Use these in your perception pipeline:

```python
# Example: Subscribe to depth camera in ROS 2 node
import rclpy
from sensor_msgs.msg import Image

def depth_callback(msg):
    # Process depth image
    pass

subscription = node.create_subscription(
    Image,
    '/d455/depth',
    depth_callback,
    10
)
```

## Notes

- **T265 Simulation**: Uses ground-truth odometry plugin for perfect pose tracking. Real T265 has drift and noise.
- **D455 Depth**: Gazebo's depth_camera sensor provides perfect depth. Real D455 has noise, holes, and lighting sensitivity.
- **Performance**: All sensors at full rate may impact simulation performance. Adjust update rates if needed.
- **PX4 Integration**: Odometry data can be fed to PX4 via MAVLink VISION_POSITION_ESTIMATE messages.

## Future Enhancements

- Add sensor noise models (Gaussian noise on depth, odometry drift)
- Implement T265 VSLAM simulation (feature tracking)
- Add IMU sensor for T265
- Model D455 depth accuracy degradation with distance
