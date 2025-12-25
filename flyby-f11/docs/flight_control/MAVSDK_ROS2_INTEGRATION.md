# MAVSDK + ROS 2 Integration Guide for flyby-f11

This guide covers integrating MAVSDK with ROS 2 for autonomous flight control on the flyby-f11 platform.

## Architecture Options

### Option 1: ROS 2 MAVSDK Wrapper (Recommended)
Use existing ROS 2 wrapper around MAVSDK:
- Repository: https://github.com/slaghuis/drone_mavsdk
- Provides ROS 2 topics, services, actions for MAVSDK
- Easy integration with existing ROS 2 autonomy stack

### Option 2: Custom ROS 2 + MAVSDK Node
Write custom ROS 2 node using MAVSDK library:
- More control over implementation
- Tailored to specific mission needs
- Requires more development time

### Option 3: Direct MAVLink with ROS 2
Use mavros2 or custom MAVLink node:
- No MAVSDK dependency
- Lower-level MAVLink control
- More complex to implement

**Recommendation for flyby-f11**: Start with Option 1 (drone_mavsdk wrapper), customize as needed.

## drone_mavsdk Architecture

### Repository Structure
```
drone_mavsdk/
├── drone_base/              # Base MAVSDK node
├── drone_base_interfaces/   # ROS 2 message/service definitions
├── drone_telemetry/         # Telemetry publisher node
├── drone_action/            # Action client node (arm, takeoff, land)
├── drone_offboard/          # Offboard control node
└── drone_bringup/           # Launch files
```

### Key Components

#### drone_base
Core MAVSDK connection node:
- Connects to ArduPilot via UDP/serial
- Publishes system status
- Provides base for other nodes

#### drone_telemetry
Publishes telemetry data as ROS 2 topics:
- `/drone/telemetry/position` (geometry_msgs/PoseStamped)
- `/drone/telemetry/velocity` (geometry_msgs/TwistStamped)
- `/drone/telemetry/attitude` (geometry_msgs/QuaternionStamped)
- `/drone/telemetry/battery` (sensor_msgs/BatteryState)
- `/drone/telemetry/gps` (sensor_msgs/NavSatFix)

#### drone_action
ROS 2 action server for high-level commands:
- Arm/Disarm
- Takeoff
- Land
- Return to Launch

#### drone_offboard
Offboard control via ROS 2 topics:
- Subscribe to `/drone/offboard/setpoint_position` (geometry_msgs/PoseStamped)
- Subscribe to `/drone/offboard/setpoint_velocity` (geometry_msgs/TwistStamped)
- Sends setpoints to MAVSDK at 10Hz+

## Installation

### Clone and Build

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src
git clone https://github.com/slaghuis/drone_mavsdk.git

cd /home/finley/Github/DroneProjects/flyby-f11/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_base_interfaces drone_base drone_telemetry drone_action drone_offboard
source install/setup.bash
```

### Dependencies

```bash
# MAVSDK C++ library
sudo apt install libmavsdk-dev

# Or build from source:
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
cmake -Bbuild -H.
cmake --build build -j4
sudo cmake --build build --target install
```

## Configuration

### Launch File Example

Create `flyby-f11/ros2_ws/src/flyby_f11_bringup/launch/mavsdk_bridge.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Connection type argument
        DeclareLaunchArgument(
            'connection_url',
            default_value='udp://:14550',
            description='MAVSDK connection URL'
        ),

        # drone_base node - core MAVSDK connection
        Node(
            package='drone_base',
            executable='drone_base_node',
            name='drone_base',
            parameters=[{
                'connection_url': LaunchConfiguration('connection_url'),
                'system_id': 1,
                'component_id': 191
            }],
            output='screen'
        ),

        # drone_telemetry node - publish telemetry to ROS 2
        Node(
            package='drone_telemetry',
            executable='drone_telemetry_node',
            name='drone_telemetry',
            output='screen'
        ),

        # drone_action node - arm, takeoff, land actions
        Node(
            package='drone_action',
            executable='drone_action_node',
            name='drone_action',
            output='screen'
        ),

        # drone_offboard node - offboard position/velocity control
        Node(
            package='drone_offboard',
            executable='drone_offboard_node',
            name='drone_offboard',
            parameters=[{
                'control_rate_hz': 10.0
            }],
            output='screen'
        ),
    ])
```

### Connection URLs

**SITL (UDP)**:
```
udp://:14550
```

**Serial (Companion Computer)**:
```
serial:///dev/ttyTHS0:921600
```

**TCP**:
```
tcp://192.168.1.100:5760
```

## Usage Examples

### Launch MAVSDK Bridge

```bash
# With SITL (UDP)
ros2 launch flyby_f11_bringup mavsdk_bridge.launch.py

# With serial connection
ros2 launch flyby_f11_bringup mavsdk_bridge.launch.py connection_url:=serial:///dev/ttyTHS0:921600
```

### Command-Line Testing

```bash
# Check telemetry
ros2 topic echo /drone/telemetry/position
ros2 topic echo /drone/telemetry/battery

# Arm
ros2 action send_goal /drone/action/arm drone_base_interfaces/action/Arm "{}"

# Takeoff to 5m
ros2 action send_goal /drone/action/takeoff drone_base_interfaces/action/Takeoff "{altitude: 5.0}"

# Send position setpoint (5m north, 0m east, -5m down (5m altitude), 0° yaw)
ros2 topic pub /drone/offboard/setpoint_position geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 0.0, z: 5.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

# Land
ros2 action send_goal /drone/action/land drone_base_interfaces/action/Land "{}"
```

## Integration with Autonomy Stack

### Example: Waypoint Navigator Node

Create custom node that uses MAVSDK bridge:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class WaypointNavigator : public rclcpp::Node
{
public:
    WaypointNavigator() : Node("waypoint_navigator")
    {
        // Subscribe to current position
        position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/telemetry/position", 10,
            std::bind(&WaypointNavigator::position_callback, this, std::placeholders::_1));

        // Publish position setpoints to offboard controller
        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/drone/offboard/setpoint_position", 10);

        // Subscribe to mission waypoints
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/mission/path", 10,
            std::bind(&WaypointNavigator::path_callback, this, std::placeholders::_1));

        // Control loop timer (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WaypointNavigator::control_loop, this));
    }

private:
    void position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        waypoints_ = msg->poses;
        current_waypoint_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "Received %zu waypoints", waypoints_.size());
    }

    void control_loop()
    {
        if (waypoints_.empty() || current_waypoint_idx_ >= waypoints_.size()) {
            return;  // No waypoints to follow
        }

        // Get current waypoint
        auto target = waypoints_[current_waypoint_idx_];

        // Check if reached waypoint (within 0.5m)
        double distance = calculate_distance(current_position_, target);
        if (distance < 0.5) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_idx_);
            current_waypoint_idx_++;
            return;
        }

        // Publish setpoint
        geometry_msgs::msg::PoseStamped setpoint;
        setpoint.header.stamp = this->now();
        setpoint.header.frame_id = "map";
        setpoint.pose = target.pose;
        setpoint_pub_->publish(setpoint);
    }

    double calculate_distance(const geometry_msgs::msg::PoseStamped& a,
                             const geometry_msgs::msg::PoseStamped& b)
    {
        double dx = a.pose.position.x - b.pose.position.x;
        double dy = a.pose.position.y - b.pose.position.y;
        double dz = a.pose.position.z - b.pose.position.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped current_position_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_waypoint_idx_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavigator>());
    rclcpp::shutdown();
    return 0;
}
```

### Package Structure

```
flyby_f11_mission/
├── include/flyby_f11_mission/
│   └── waypoint_navigator.hpp
├── src/
│   └── waypoint_navigator.cpp
├── launch/
│   └── mission.launch.py
├── config/
│   └── mission_params.yaml
├── CMakeLists.txt
└── package.xml
```

## Frame Transformations

### ROS 2 ENU ↔ ArduPilot NED

ArduPilot uses NED (North-East-Down), ROS 2 typically uses ENU (East-North-Up).

**If using drone_mavsdk**: Check if wrapper handles frame conversion internally.

**If manual conversion needed**:

```cpp
// ROS 2 ENU to ArduPilot NED
geometry_msgs::msg::PoseStamped enu_to_ned(const geometry_msgs::msg::PoseStamped& enu)
{
    geometry_msgs::msg::PoseStamped ned;
    ned.header = enu.header;
    ned.pose.position.x = enu.pose.position.y;  // East → North
    ned.pose.position.y = enu.pose.position.x;  // North → East
    ned.pose.position.z = -enu.pose.position.z; // Up → Down (sign flip)

    // Quaternion rotation for frame transformation
    // Rotate 90° around Z axis, then flip Z
    tf2::Quaternion q_enu(enu.pose.orientation.x, enu.pose.orientation.y,
                          enu.pose.orientation.z, enu.pose.orientation.w);
    tf2::Quaternion q_rot;
    q_rot.setRPY(M_PI, 0, M_PI/2);
    tf2::Quaternion q_ned = q_rot * q_enu;
    ned.pose.orientation.x = q_ned.x();
    ned.pose.orientation.y = q_ned.y();
    ned.pose.orientation.z = q_ned.z();
    ned.pose.orientation.w = q_ned.w();

    return ned;
}
```

## Custom MAVSDK Node (Alternative)

If drone_mavsdk doesn't meet needs, create custom node:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MavsdkBridge : public rclcpp::Node
{
public:
    MavsdkBridge() : Node("mavsdk_bridge")
    {
        // Declare parameters
        this->declare_parameter("connection_url", "udp://:14550");
        std::string connection_url = this->get_parameter("connection_url").as_string();

        // Connect to drone
        mavsdk::ConnectionResult result = mavsdk_.add_any_connection(connection_url);
        if (result != mavsdk::ConnectionResult::Success) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed: %s",
                        mavsdk::connection_result_str(result));
            return;
        }

        // Wait for system
        auto system = mavsdk_.first_autopilot(3.0);
        if (!system) {
            RCLCPP_ERROR(this->get_logger(), "No autopilot found");
            return;
        }

        // Initialize plugins
        action_ = std::make_shared<mavsdk::Action>(system.value());
        offboard_ = std::make_shared<mavsdk::Offboard>(system.value());
        telemetry_ = std::make_shared<mavsdk::Telemetry>(system.value());

        // ROS 2 publishers
        position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/drone/position", 10);

        // ROS 2 subscribers
        setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/setpoint", 10,
            std::bind(&MavsdkBridge::setpoint_callback, this, std::placeholders::_1));

        // Telemetry callback
        telemetry_->subscribe_position([this](mavsdk::Telemetry::Position position) {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            msg.pose.position.x = position.latitude_deg;
            msg.pose.position.y = position.longitude_deg;
            msg.pose.position.z = position.relative_altitude_m;
            position_pub_->publish(msg);
        });

        RCLCPP_INFO(this->get_logger(), "MAVSDK bridge initialized");
    }

private:
    void setpoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        mavsdk::Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = msg->pose.position.x;
        setpoint.east_m = msg->pose.position.y;
        setpoint.down_m = -msg->pose.position.z;  // NED down is negative altitude
        setpoint.yaw_deg = 0.0f;  // Extract from quaternion if needed

        offboard_->set_position_ned(setpoint);
    }

    mavsdk::Mavsdk mavsdk_;
    std::shared_ptr<mavsdk::Action> action_;
    std::shared_ptr<mavsdk::Offboard> offboard_;
    std::shared_ptr<mavsdk::Telemetry> telemetry_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavsdkBridge>());
    rclcpp::shutdown();
    return 0;
}
```

## Testing Workflow

### 1. Start ArduPilot SITL

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map
```

### 2. Launch ROS 2 MAVSDK Bridge

```bash
source /home/finley/Github/DroneProjects/flyby-f11/ros2_ws/install/setup.bash
ros2 launch flyby_f11_bringup mavsdk_bridge.launch.py
```

### 3. Verify Telemetry

```bash
ros2 topic list
ros2 topic echo /drone/telemetry/position
```

### 4. Test Arm/Takeoff

```bash
ros2 action send_goal /drone/action/arm drone_base_interfaces/action/Arm "{}"
ros2 action send_goal /drone/action/takeoff drone_base_interfaces/action/Takeoff "{altitude: 5.0}"
```

### 5. Test Offboard Control

```bash
ros2 topic pub /drone/offboard/setpoint_position geometry_msgs/msg/PoseStamped "{
  pose: {position: {x: 5.0, y: 0.0, z: 5.0}}
}" --rate 10
```

## Troubleshooting

### No telemetry received
- Check MAVSDK connection: `ros2 node info /drone_base`
- Verify SITL is running and outputting MAVLink
- Check connection URL matches SITL output port

### Offboard mode not engaging
- Ensure setpoints are being published at >2Hz
- Check that drone is armed
- Verify GPS lock (or EKF configured for indoor)

### Frame mismatch issues
- Verify coordinate frame transformations (ENU ↔ NED)
- Check altitude sign (NED down is positive!)

### MAVSDK library not found
```bash
# Install from apt
sudo apt install libmavsdk-dev

# Or build from source
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
cmake -Bbuild -H.
cmake --build build -j4
sudo cmake --build build --target install
```

## Performance Considerations

### Update Rates
- **Telemetry**: 10-50Hz (configurable in telemetry node)
- **Setpoints**: 10-20Hz minimum (faster is better for smooth control)
- **Actions**: Event-based (arm, takeoff, land)

### Jetson Orin NX Optimization
- Use hardware acceleration for vision processing
- Offload heavy computation to GPU
- Keep control loops lightweight (<10ms cycle time)
- Monitor CPU/GPU usage with `jtop`

## Next Steps

1. Clone and build drone_mavsdk wrapper
2. Test basic telemetry and control in SITL
3. Integrate with flyby_f11_mission package
4. Add behavior tree integration
5. Test with real hardware on flyby-f11 platform

## References

- drone_mavsdk: https://github.com/slaghuis/drone_mavsdk
- MAVSDK: https://mavsdk.mavlink.io/
- ArduPilot ROS 2: https://ardupilot.org/dev/docs/ros2.html
- ROS 2 Humble: https://docs.ros.org/en/humble/
