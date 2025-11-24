# Home Test Drone - Implementation TODO

This tracks what's been scaffolded vs. what needs implementation.

## ✅ Completed (Scaffolding)

- [x] Directory structure created
- [x] Docker container with full stack (ROS 2, MAVSDK, RTAB-Map, BehaviorTree.CPP)
- [x] ROS 2 package structure (5 packages)
- [x] PX4 parameter configuration for vision-based flight
- [x] Sensor calibration templates (T265, D455)
- [x] Sample mission files (GPS waypoints, indoor SLAM)
- [x] Sample behavior tree (explore_and_map.xml)
- [x] Documentation (README, QUICKSTART, PX4 config guide)

## 🚧 In Progress / Next Steps

### Immediate (Phase 1: GPS Waypoints)

- [ ] **Build container on Jetson**
  ```bash
  cd home-test-drone/docker
  sudo docker-compose build
  ```

- [ ] **Test camera detection**
  - Verify T265 detected with librealsense v2.50.0
  - Verify D455 detected
  - Test both cameras simultaneously

- [ ] **Implement MAVSDK bridge node** (`hometestdrone_control`)
  - Create `mavsdk_offboard_node.cpp`
  - Connect to Pixhawk via serial/USB
  - Publish vehicle state to ROS topics
  - Subscribe to setpoint commands
  - Implement GPS waypoint following

- [ ] **Create simple GPS waypoint mission launcher**
  - `hometestdrone_bringup/launch/gps_mission.launch.py`
  - Launch MAVSDK bridge
  - Load waypoint file
  - Execute mission

- [ ] **Outdoor flight testing**
  - Validate takeoff/land
  - Test waypoint navigation
  - Verify telemetry logging

### Phase 2: Indoor VIO Flight

- [ ] **Implement T265 → PX4 bridge** (`hometestdrone_control`)
  - Subscribe to T265 odometry
  - Transform ROS (FLU) → PX4 (NED) coordinates
  - Publish VISION_POSITION_ESTIMATE to PX4 via MAVLink
  - Monitor tracking confidence

- [ ] **Create indoor VIO launcher**
  - `hometestdrone_bringup/launch/indoor_vio.launch.py`
  - Launch T265 camera node
  - Launch vision → PX4 bridge
  - Launch MAVSDK offboard controller

- [ ] **Indoor position hold testing**
  - Verify EKF2 fuses vision position
  - Test drift over time
  - Validate position hold performance

- [ ] **Manual waypoint navigation indoors**
  - Send simple waypoints via offboard
  - Verify drone navigates correctly without GPS

### Phase 3: Autonomous SLAM Flight

- [ ] **Configure RTAB-Map for T265 + D455**
  - Create `hometestdrone_slam/config/rtabmap.yaml`
  - Set up visual-inertial SLAM mode
  - Configure T265 for odometry, D455 for RGB-D
  - Tune loop closure parameters

- [ ] **Create SLAM launcher**
  - `hometestdrone_slam/launch/rtabmap.launch.py`
  - Launch both cameras
  - Launch RTAB-Map node with config
  - Publish map and pose

- [ ] **Implement behavior tree nodes** (`hometestdrone_behaviors`)
  - `CheckBatteryLevel` - monitor battery voltage
  - `CheckTrackingQuality` - verify T265 confidence
  - `CheckSLAMReady` - verify RTAB-Map initialized
  - `FindNearestFrontier` - frontier-based exploration
  - `PlanPath` - path planning around obstacles
  - `FollowPath` - trajectory tracking
  - `ReturnToLaunch` - navigate back to start

- [ ] **Implement behavior tree executor**
  - `hometestdrone_behaviors/src/bt_executor_node.cpp`
  - Load behavior tree XML
  - Register custom action nodes
  - Execute behavior tree with ROS integration

- [ ] **Obstacle detection** (`hometestdrone_perception`)
  - Subscribe to D455 depth image
  - Detect obstacles in flight path
  - Publish obstacle markers
  - Integrate with path planner

- [ ] **Create full autonomy launcher**
  - `hometestdrone_bringup/launch/indoor_slam.launch.py`
  - Launch all sensors
  - Launch SLAM
  - Launch perception
  - Launch behavior tree executor
  - Launch offboard controller

- [ ] **Single room exploration testing**
  - Start with small area (2m x 2m)
  - Verify map building
  - Test obstacle avoidance
  - Validate return-to-home

### Phase 4: Advanced Behaviors

- [ ] **Multi-room exploration**
  - Expand exploration radius
  - Handle doorways and corridors
  - Improve loop closure for larger spaces

- [ ] **Object detection integration**
  - Add vision model (YOLO, etc.)
  - Track objects of interest
  - Report findings to mission log

- [ ] **Dynamic replanning**
  - Handle blocked paths
  - Update exploration strategy based on discoveries
  - Integrate with behavior tree

- [ ] **Mission validation suite**
  - Automated tests for common scenarios
  - Performance metrics (coverage, time, accuracy)
  - Failure recovery testing

## 📝 Code Examples Needed

Priority implementations to get you started:

### 1. MAVSDK Bridge Node (Start Here)
```cpp
// hometestdrone_control/src/mavsdk_offboard_node.cpp
// Connect to PX4, arm, takeoff, follow waypoints
// See: https://mavsdk.mavlink.io/main/en/cpp/guide/
```

### 2. T265 to PX4 Vision Bridge
```cpp
// hometestdrone_control/src/t265_vision_bridge.cpp
// Subscribe: /t265/odom/sample
// Publish: VISION_POSITION_ESTIMATE to PX4
```

### 3. Basic Behavior Tree Node
```cpp
// hometestdrone_behaviors/src/check_battery_level.cpp
// Example custom BT action node
```

## 🎯 Current Focus

**You said you're working on behavior trees now** - here's what you need:

1. **Implement `bt_executor_node.cpp`** in `hometestdrone_behaviors/src/`
   - Use BehaviorTree.CPP library
   - Load `explore_and_map.xml`
   - Register custom action nodes
   - Execute tree in ROS 2 node

2. **Start with stub implementations** of behavior tree actions
   - Return SUCCESS for now
   - Add ROS logging to show execution flow
   - Gradually add real functionality

3. **Create test launcher** to run behavior tree standalone
   - `ros2 launch hometestdrone_behaviors test_bt.launch.py`
   - No drone needed initially
   - Verify tree executes correctly

## 📚 Useful References

- **BehaviorTree.CPP**: https://www.behaviortree.dev/
- **MAVSDK C++ Examples**: https://mavsdk.mavlink.io/main/en/cpp/examples/
- **RTAB-Map ROS 2**: https://github.com/introlab/rtabmap_ros/tree/humble-devel
- **RealSense ROS 2**: https://github.com/IntelRealSense/realsense-ros

## Questions?

If stuck, check:
1. Main project `CLAUDE.md` for development conventions
2. `QUICKSTART.md` for setup steps
3. Package-level READMEs for specific details

Want me to implement any of these? Let me know which part to start with!
