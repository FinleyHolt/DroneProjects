# Intel RealSense Setup and Configuration Guide

**Platform**: NVIDIA Jetson Orin NX 16GB
**Cameras**: Intel RealSense T265 (VIO), D455 (Depth)
**ROS Version**: ROS 2 Humble
**Last Updated**: 2025-12-25

---

## Table of Contents

1. [Hardware Overview](#hardware-overview)
2. [Installation](#installation)
3. [ROS 2 Wrapper Setup](#ros-2-wrapper-setup)
4. [T265 Configuration](#t265-configuration)
5. [D455 Configuration](#d455-configuration)
6. [Calibration](#calibration)
7. [Troubleshooting](#troubleshooting)
8. [Performance Optimization](#performance-optimization)

---

## Hardware Overview

### Intel RealSense T265 (Tracking Camera)

**Purpose**: Visual-inertial odometry (VIO) for precise position tracking

**Specifications:**
- Dual fisheye cameras (848x800 @ 30 FPS)
- Built-in IMU (gyro + accelerometer @ 200 Hz)
- V-SLAM processor (onboard pose calculation)
- USB 2.0/3.0 interface
- Low power consumption (~1.5W)

**Use Cases:**
- GPS-denied navigation
- Indoor flight stabilization
- Odometry fusion with PX4

**Output:**
- 6DOF pose (position + orientation) @ 200 Hz
- Fisheye images @ 30 FPS
- IMU data @ 200 Hz

---

### Intel RealSense D455 (Depth Camera)

**Purpose**: Obstacle detection and 3D scene reconstruction

**Specifications:**
- RGB camera (1920x1080 @ 30 FPS)
- Stereo depth cameras (1280x720 @ 90 FPS)
- Active IR projector (for low-texture scenes)
- Depth range: 0.6m - 20m (optimal: 0.6m - 6m)
- USB 3.0 interface
- Global shutter (depth cameras)

**Use Cases:**
- Obstacle avoidance
- 3D mapping and SLAM
- Object detection with depth
- Terrain analysis

**Output:**
- RGB images @ 30 FPS
- Depth images @ 30 FPS
- Aligned RGB + depth
- Point clouds

---

## Installation

### Install librealsense2

```bash
# Add Intel RealSense repository
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"

# Update package list
sudo apt update

# Install librealsense2
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# Verify installation
realsense-viewer
```

### Install RealSense SDK from Source (Alternative)

If prebuilt packages are unavailable for ARM:

```bash
# Install dependencies
sudo apt install git cmake build-essential libssl-dev libusb-1.0-0-dev \
                 pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev \
                 libglu1-mesa-dev

# Clone repository
cd ~/
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Build and install
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
make -j$(nproc)
sudo make install

# Update library paths
sudo ldconfig
```

### Verify Installation

```bash
# List connected RealSense devices
rs-enumerate-devices

# Expected output for T265:
# Device 0: Intel RealSense T265 (Serial: XXXXXXXXXX)

# Expected output for D455:
# Device 1: Intel RealSense D455 (Serial: XXXXXXXXXX)

# Launch viewer
realsense-viewer
```

### Configure USB Permissions

```bash
# Download and install udev rules
cd /etc/udev/rules.d/
sudo wget https://github.com/IntelRealSense/librealsense/raw/master/config/99-realsense-libusb.rules

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Verify permissions
ls -l /dev/video* | grep video
# All devices should be accessible by current user
```

---

## ROS 2 Wrapper Setup

### Install from Binary (Recommended)

```bash
# Install ROS 2 RealSense wrapper
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description

# Verify installation
ros2 pkg list | grep realsense
# Expected: realsense2_camera, realsense2_description
```

### Install from Source (for latest features)

```bash
# Navigate to ROS 2 workspace
cd /home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src

# Clone RealSense ROS wrapper
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development

# Install dependencies
cd ..
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --packages-select realsense2_camera realsense2_description

# Source workspace
source install/setup.bash
```

### Test ROS 2 Wrapper

```bash
# Launch T265
ros2 launch realsense2_camera rs_launch.py \
    device_type:=t265 \
    enable_pose:=true \
    enable_fisheye1:=true \
    enable_fisheye2:=true

# Verify topics (in new terminal)
ros2 topic list | grep camera
# Expected topics:
# /camera/odom/sample
# /camera/accel/sample
# /camera/gyro/sample
# /camera/fisheye1/image_raw
# /camera/fisheye2/image_raw
```

---

## T265 Configuration

### Launch File Configuration

Create `/home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src/flyby_f11_sensors/launch/t265.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='t265',
            namespace='t265',
            parameters=[{
                'device_type': 't265',
                'serial_no': '',  # Leave empty to auto-detect
                'enable_pose': True,
                'enable_fisheye1': True,
                'enable_fisheye2': True,
                'fisheye_width': 848,
                'fisheye_height': 800,
                'fisheye_fps': 30,
                'enable_imu': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'unite_imu_method': 'linear_interpolation',
                'publish_odom_tf': True,
                'odom_frame_id': 't265_odom_frame',
                'base_frame_id': 't265_link',
                'pose_frame_id': 't265_pose_frame',
            }],
            output='screen'
        )
    ])
```

### Parameter Descriptions

| Parameter | Value | Description |
|-----------|-------|-------------|
| `device_type` | t265 | Camera model |
| `enable_pose` | true | Publish 6DOF pose |
| `enable_fisheye1` | true | Publish left fisheye image |
| `enable_fisheye2` | true | Publish right fisheye image |
| `fisheye_fps` | 30 | Fisheye frame rate |
| `enable_imu` | true | Publish IMU data |
| `gyro_fps` | 200 | Gyroscope sample rate |
| `accel_fps` | 200 | Accelerometer sample rate |
| `unite_imu_method` | linear_interpolation | Sync accel/gyro |
| `publish_odom_tf` | true | Publish TF transform |

### Topic Reference

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/t265/odom/sample` | `nav_msgs/Odometry` | 200 Hz | 6DOF pose |
| `/t265/accel/sample` | `sensor_msgs/Imu` | 200 Hz | Accelerometer |
| `/t265/gyro/sample` | `sensor_msgs/Imu` | 200 Hz | Gyroscope |
| `/t265/fisheye1/image_raw` | `sensor_msgs/Image` | 30 FPS | Left fisheye |
| `/t265/fisheye2/image_raw` | `sensor_msgs/Image` | 30 FPS | Right fisheye |
| `/t265/fisheye1/camera_info` | `sensor_msgs/CameraInfo` | 30 FPS | Calibration |

### Coordinate Frame Convention

T265 uses **NED convention** (North-East-Down):
- X: Forward
- Y: Right
- Z: Down

**Convert to ROS convention (ENU - East-North-Up):**

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations

class T265FrameConverter(Node):
    def __init__(self):
        super().__init__('t265_frame_converter')
        self.sub = self.create_subscription(
            Odometry,
            '/t265/odom/sample',
            self.odom_callback,
            10
        )
        self.pub = self.create_publisher(Odometry, '/odom', 10)

    def odom_callback(self, msg):
        """Convert T265 NED to ROS ENU."""
        # NED to ENU: X_enu = Y_ned, Y_enu = X_ned, Z_enu = -Z_ned
        converted = Odometry()
        converted.header = msg.header
        converted.header.frame_id = 'odom'
        converted.child_frame_id = 'base_link'

        # Position
        converted.pose.pose.position.x = msg.pose.pose.position.y
        converted.pose.pose.position.y = msg.pose.pose.position.x
        converted.pose.pose.position.z = -msg.pose.pose.position.z

        # Orientation (quaternion rotation)
        q_ned = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        # Apply 90° rotation around Z to convert NED → ENU
        q_rot = tf_transformations.quaternion_from_euler(0, 0, 1.5708)  # 90° in rad
        q_enu = tf_transformations.quaternion_multiply(q_rot, q_ned)

        converted.pose.pose.orientation.x = q_enu[0]
        converted.pose.pose.orientation.y = q_enu[1]
        converted.pose.pose.orientation.z = q_enu[2]
        converted.pose.pose.orientation.w = q_enu[3]

        # Velocity (linear and angular)
        converted.twist.twist.linear.x = msg.twist.twist.linear.y
        converted.twist.twist.linear.y = msg.twist.twist.linear.x
        converted.twist.twist.linear.z = -msg.twist.twist.linear.z

        self.pub.publish(converted)

def main():
    rclpy.init()
    node = T265FrameConverter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## D455 Configuration

### Launch File Configuration

Create `/home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src/flyby_f11_sensors/launch/d455.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d455',
            namespace='d455',
            parameters=[{
                'device_type': 'd455',
                'serial_no': '',  # Auto-detect

                # RGB camera
                'enable_color': True,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,

                # Depth camera
                'enable_depth': True,
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30,

                # Alignment
                'align_depth.enable': True,

                # Point cloud
                'pointcloud.enable': True,

                # Post-processing filters
                'decimation_filter.enable': True,
                'spatial_filter.enable': True,
                'temporal_filter.enable': True,
                'hole_filling_filter.enable': True,

                # Filter parameters
                'decimation_filter.filter_magnitude': 2,
                'spatial_filter.smooth_alpha': 0.5,
                'spatial_filter.smooth_delta': 20,
                'temporal_filter.smooth_alpha': 0.4,
                'temporal_filter.smooth_delta': 20,
            }],
            output='screen'
        )
    ])
```

### Parameter Reference

| Parameter | Value | Description |
|-----------|-------|-------------|
| `enable_color` | true | RGB camera |
| `color_width` | 640 | RGB width (1920 for high-res) |
| `color_height` | 480 | RGB height (1080 for high-res) |
| `enable_depth` | true | Depth camera |
| `depth_width` | 640 | Depth width |
| `depth_height` | 480 | Depth height |
| `align_depth.enable` | true | Align depth to RGB |
| `pointcloud.enable` | true | Publish point cloud |

### Post-Processing Filters

#### Decimation Filter
Reduces depth resolution for faster processing:
```yaml
decimation_filter.enable: true
decimation_filter.filter_magnitude: 2  # Reduce by 2x (640x480 → 320x240)
```

#### Spatial Filter
Edge-preserving smoothing to reduce noise:
```yaml
spatial_filter.enable: true
spatial_filter.smooth_alpha: 0.5    # Smoothing strength (0-1)
spatial_filter.smooth_delta: 20     # Edge threshold
spatial_filter.magnitude: 2         # Filter iterations
```

#### Temporal Filter
Reduces noise using multiple frames:
```yaml
temporal_filter.enable: true
temporal_filter.smooth_alpha: 0.4   # Temporal smoothing (0-1)
temporal_filter.smooth_delta: 20    # Value threshold
temporal_filter.persistence_control: 3  # History size
```

#### Hole Filling Filter
Fills missing depth pixels:
```yaml
hole_filling_filter.enable: true
hole_filling_filter.mode: 1  # 0=none, 1=fill from left, 2=farthest from around
```

### Topic Reference

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/d455/color/image_raw` | `sensor_msgs/Image` | 30 FPS | RGB image |
| `/d455/depth/image_rect_raw` | `sensor_msgs/Image` | 30 FPS | Depth image |
| `/d455/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 30 FPS | Aligned depth |
| `/d455/depth/color/points` | `sensor_msgs/PointCloud2` | 30 FPS | Point cloud |
| `/d455/color/camera_info` | `sensor_msgs/CameraInfo` | 30 FPS | Camera calibration |

---

## Calibration

### T265 Calibration

T265 is **factory-calibrated** and does not require user calibration. Calibration data is stored on-device.

To verify calibration:
```bash
# View intrinsics
rs-enumerate-devices -c

# Expected output:
# Fisheye 1 Intrinsics: fx=285.36, fy=285.50, ppx=419.95, ppy=403.89
# Fisheye 2 Intrinsics: fx=285.27, fy=285.43, ppx=422.14, ppy=401.76
```

### D455 Calibration

D455 is also **factory-calibrated**, but you can verify alignment:

```bash
# Launch D455 with alignment enabled
ros2 launch realsense2_camera rs_launch.py \
    device_type:=d455 \
    align_depth.enable:=true

# Visualize in RViz
rviz2

# Add displays:
# 1. Image: /d455/color/image_raw
# 2. Image: /d455/aligned_depth_to_color/image_raw
# 3. PointCloud2: /d455/depth/color/points
```

### Manual Calibration (if needed)

If alignment is poor:

1. **Download Intel RealSense Dynamic Calibration Tool:**
```bash
sudo apt install intel-realsense-dct
```

2. **Run calibration:**
```bash
intel-realsense-dct
# Follow on-screen instructions
# Point camera at textured surface 1-2m away
# Move camera slowly in different orientations
```

3. **Save calibration to device:**
Calibration is automatically written to camera EEPROM.

---

## Troubleshooting

### Issue: Camera Not Detected

**Symptoms:**
```bash
rs-enumerate-devices
# No device detected
```

**Solutions:**

1. **Check USB connection:**
```bash
lsusb | grep Intel
# Expected: Bus XXX Device XXX: ID 8086:XXXX Intel Corp. RealSense XXXX
```

2. **Verify udev rules:**
```bash
ls /etc/udev/rules.d/ | grep realsense
# Expected: 99-realsense-libusb.rules

# Reload if missing
sudo udevadm control --reload-rules && sudo udevadm trigger
```

3. **Try different USB port:**
- T265: USB 2.0 or 3.0
- D455: USB 3.0 required for full resolution

4. **Update firmware:**
```bash
rs-fw-update -l  # List devices
rs-fw-update -f firmware.bin  # Update if needed
```

---

### Issue: Poor Tracking Performance (T265)

**Symptoms:**
- Pose drift
- Tracking lost frequently

**Solutions:**

1. **Improve lighting:**
- T265 requires sufficient ambient light
- Avoid direct sunlight (causes glare)

2. **Add visual features:**
- Avoid textureless environments (blank walls)
- Add posters, patterns, or fiducial markers

3. **Reduce motion speed:**
- T265 has max velocity limit (~3 m/s)
- Avoid rapid rotations

4. **Check IMU data:**
```bash
ros2 topic echo /t265/accel/sample
ros2 topic echo /t265/gyro/sample
# Verify data is publishing at 200 Hz
```

---

### Issue: Noisy Depth Data (D455)

**Symptoms:**
- Speckled depth image
- Unstable point cloud

**Solutions:**

1. **Enable post-processing filters:**
```yaml
spatial_filter.enable: true
temporal_filter.enable: true
```

2. **Adjust filter strength:**
```yaml
spatial_filter.smooth_alpha: 0.7  # Increase for more smoothing
temporal_filter.smooth_alpha: 0.6
```

3. **Improve IR illumination:**
- Enable emitter:
```yaml
emitter_enabled: 1  # 0=off, 1=on, 2=auto
```

4. **Reduce range:**
- Depth quality degrades beyond 6m
- Focus on 0.6m - 6m range

---

### Issue: High CPU/GPU Usage

**Symptoms:**
- System lag
- Dropped frames

**Solutions:**

1. **Reduce resolution:**
```yaml
color_width: 640   # Instead of 1920
color_height: 480  # Instead of 1080
depth_width: 640   # Instead of 1280
depth_height: 480  # Instead of 720
```

2. **Disable unused streams:**
```yaml
enable_infra1: false  # Disable IR cameras if not needed
pointcloud.enable: false  # Disable point cloud if not needed
```

3. **Use decimation filter:**
```yaml
decimation_filter.enable: true
decimation_filter.filter_magnitude: 2  # Reduce depth resolution by 2x
```

---

## Performance Optimization

### Multi-Camera Setup (T265 + D455)

Launch both cameras concurrently:

```python
# dual_camera.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # T265 for odometry
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='t265',
            namespace='t265',
            parameters=[{
                'device_type': 't265',
                'enable_pose': True,
                'enable_fisheye1': False,  # Disable images to save bandwidth
                'enable_fisheye2': False,
                'enable_imu': True,
            }]
        ),

        # D455 for perception
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d455',
            namespace='d455',
            parameters=[{
                'device_type': 'd455',
                'enable_color': True,
                'color_width': 640,
                'color_height': 480,
                'enable_depth': True,
                'depth_width': 640,
                'depth_height': 480,
                'align_depth.enable': True,
                'pointcloud.enable': False,  # Disable to save CPU
                'spatial_filter.enable': True,
                'temporal_filter.enable': True,
            }]
        ),
    ])
```

### Bandwidth Optimization

For USB 3.0 bandwidth constraints:

1. **Reduce frame rates:**
```yaml
color_fps: 15  # Instead of 30
depth_fps: 15
```

2. **Use MJPEG compression (for color):**
```yaml
rgb_camera.profile: 640,480,15  # Width, height, FPS
```

3. **Disable unnecessary streams:**
- Disable fisheye images if only using T265 pose
- Disable IR cameras if not needed

---

## Quick Reference

### Launch T265
```bash
ros2 launch realsense2_camera rs_launch.py device_type:=t265 enable_pose:=true
```

### Launch D455
```bash
ros2 launch realsense2_camera rs_launch.py device_type:=d455 align_depth.enable:=true
```

### View Topics
```bash
ros2 topic list | grep camera
ros2 topic hz /d455/color/image_raw
ros2 topic echo /t265/odom/sample
```

### Monitor Performance
```bash
# Camera info
rs-enumerate-devices

# Frame rates
ros2 topic hz /d455/color/image_raw
ros2 topic hz /t265/odom/sample

# System resources
jtop
```

---

**Maintainer**: Finley Holt
**Last Updated**: 2025-12-25
**Version**: 1.0
