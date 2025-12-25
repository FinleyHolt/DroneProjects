---
title: "Flyby-F11 Development Workflow Guide"
author: "Finley Holt"
date: 2025-12-25
format:
  pdf:
    documentclass: article
    geometry:
      - margin=1in
    fontsize: 11pt
    number-sections: true
    toc: true
    include-in-header:
      text: |
        \usepackage{fancyhdr}
        \pagestyle{fancy}
        \fancyhf{}
        \fancyhead[L]{\textit{Development Workflow Guide}}
        \fancyhead[R]{\thepage}
        \renewcommand{\headrulewidth}{0.4pt}
---

# Development Workflow Guide

This guide covers the complete development workflow for the flyby-f11 ontology-constrained reinforcement learning autonomy system, from environment setup through simulation testing.

## Development Environment Setup

### Prerequisites

**Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)

**Hardware Requirements**:
- CPU: 8+ cores recommended
- RAM: 16GB minimum, 32GB recommended
- GPU: NVIDIA GPU with 8GB+ VRAM (for vision model training)
- Storage: 100GB free space

**NVIDIA GPU Setup** (if applicable):
```bash
# Check NVIDIA driver
nvidia-smi

# If not installed, install NVIDIA drivers
sudo apt update
sudo apt install nvidia-driver-535

# Install CUDA toolkit (for local training/testing)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install cuda-toolkit-12-2

# Add to ~/.bashrc
export PATH=/usr/local/cuda-12.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATH
```

### Core Development Tools

**Install ROS 2 Humble**:
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools python3-colcon-common-extensions

# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Install ArduPilot SITL**:
```bash
# Clone ArduPilot repository
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-4.5  # Stable release
git submodule update --init --recursive

# Install prerequisites
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload environment
. ~/.profile

# Build ArduCopter SITL
./waf configure --board sitl
./waf copter

# Test SITL
cd ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
# Press Ctrl+C to exit after verifying it runs
```

**Install Gazebo Classic (Gazebo 11)**:
```bash
# ROS 2 Humble uses Gazebo 11 by default
sudo apt install gazebo libgazebo11 libgazebo-dev

# Install ArduPilot Gazebo plugin
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Add to ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:$GAZEBO_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc

# Test Gazebo
gazebo --verbose
# Press Ctrl+C to exit after verifying it runs
```

**Install MAVSDK**:
```bash
# Install MAVSDK C++ library
sudo apt install libmavsdk-dev

# For Python development (optional)
pip3 install mavsdk
```

**Install Python Development Tools**:
```bash
# System Python packages
sudo apt install python3-pip python3-venv

# Create virtual environment for development
cd ~/Github/DroneProjects/flyby-f11
python3 -m venv venv
source venv/bin/activate

# Install development dependencies
pip install --upgrade pip
pip install ruff black mypy pytest pytest-cov
pip install numpy scipy matplotlib
pip install stable-baselines3[extra] gymnasium
pip install tensorboard wandb  # For training monitoring
pip install opencv-python pillow
```

**Install Ontology Tools**:
```bash
# SWI-Prolog (runtime reasoning)
sudo apt-add-repository ppa:swi-prolog/stable
sudo apt update
sudo apt install swi-prolog

# PySwip (Python-Prolog interface)
pip install pyswip

# Vampire theorem prover (planning phase)
cd ~/flyby-f11/docs/ontology/repos
# Follow ontology/DOWNLOAD_INSTRUCTIONS.md for Vampire setup

# Clingo (answer set programming)
sudo apt install gringo

# E-Prover (alternative FOL reasoner)
cd ~/flyby-f11/docs/ontology/repos
# Follow ontology/DOWNLOAD_INSTRUCTIONS.md for E-Prover setup
```

### Development Repository Setup

**Clone Repository**:
```bash
cd ~/Github
git clone <repository-url> DroneProjects
cd DroneProjects/flyby-f11
```

**Initialize Git LFS** (for large assets):
```bash
git lfs install
git lfs pull
```

**Install Pre-commit Hooks**:
```bash
pip install pre-commit
pre-commit install

# Test hooks
pre-commit run --all-files
```

## Building the ROS 2 Workspace

### Initial Build

**Navigate to workspace**:
```bash
cd ~/Github/DroneProjects/flyby-f11/ros2_ws
```

**Install ROS 2 dependencies**:
```bash
# Install rosdep if not already installed
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y
```

**Build workspace**:
```bash
# Clean build (first time or after major changes)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Incremental build (during development)
colcon build --symlink-install --packages-select <package_name>

# Build with debugging symbols
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**Source workspace**:
```bash
source install/setup.bash

# Add to ~/.bashrc for convenience (be careful with overlay precedence)
echo "source ~/Github/DroneProjects/flyby-f11/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Package Organization

**flyby-f11 ROS 2 packages** (located in `ros2_ws/src/`):

```
src/
├── flyby_f11_bringup/          # Launch configurations
├── flyby_f11_sensors/          # Sensor drivers (platform-specific)
├── flyby_f11_mission/          # Mission-specific logic
├── ontology_interface/         # Prolog reasoner, grounding nodes
├── perception_grounding/       # Vision → symbolic abstraction
├── ontology_rl/                # RL agents with ontology constraints
├── phase_transition_manager/   # Planning ↔ execution mode switching
└── agents_interface/           # Custom message/service definitions
```

**Shared packages** (symlinked from project-drone when ready):
```
src/
├── autonomy_core/              # Core autonomy (platform-agnostic)
├── behavior_trees/             # BehaviorTree.CPP logic
├── perception_pipeline/        # Vision models (YOLO, segmentation)
└── ardupilot_interface/        # MAVLink/MQTT bridge
```

### Common Build Issues

**Missing dependencies**:
```bash
# Re-run rosdep
rosdep install --from-paths src --ignore-src -r -y

# Check for missing system packages
apt-cache search <package-name>
```

**Python import errors**:
```bash
# Ensure workspace is sourced
source install/setup.bash

# Check PYTHONPATH
echo $PYTHONPATH

# Verify package installation
ros2 pkg list | grep flyby
```

**Symbol not found / linking errors**:
```bash
# Clean rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Running ArduPilot SITL Simulation

### Basic SITL Launch

**Start ArduPilot SITL** (Terminal 1):
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14550
```

**Parameters**:
- `-v ArduCopter`: Vehicle type (quadcopter)
- `--console`: MAVProxy command-line interface
- `--map`: Visual map display
- `--out=udp:127.0.0.1:14550`: MAVLink output for MAVSDK/ROS 2

**MAVProxy commands** (in SITL console):
```bash
# Arm and takeoff
mode GUIDED
arm throttle
takeoff 10  # 10 meters

# Set parameters
param set ARMING_CHECK 16384  # Skip pre-arm checks (testing only)
param show SERIAL*  # Show all serial parameters

# Save parameters
param save

# Manual flight
mode STABILIZE
rc 3 1500  # Throttle mid

# Return to launch
mode RTL
```

### SITL with Gazebo Visualization

**Start Gazebo** (Terminal 1):
```bash
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

**Start ArduPilot SITL** (Terminal 2):
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

**Verify connection**:
- Gazebo should show quadcopter model
- SITL console should show "Received HEARTBEAT from Gazebo"
- MAVProxy map should show vehicle position

### Custom Worlds and Models

**Create custom Gazebo world**:
```bash
cd ~/Github/DroneProjects/flyby-f11/simulation/worlds
# Copy and modify existing world
cp ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world custom_mission.world

# Edit world file
nano custom_mission.world
```

**Add custom models**:
```bash
export GAZEBO_MODEL_PATH=~/Github/DroneProjects/flyby-f11/simulation/models:$GAZEBO_MODEL_PATH
```

**Launch with custom world**:
```bash
gazebo --verbose ~/Github/DroneProjects/flyby-f11/simulation/worlds/custom_mission.world
```

### SITL Configuration Files

**Load custom parameters**:
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --add-param-file=~/Github/DroneProjects/flyby-f11/config/ardupilot_params.parm
```

**Save current parameters**:
```bash
# In MAVProxy console
param save ~/Github/DroneProjects/flyby-f11/config/ardupilot_params_backup.parm
```

## ROS 2 Development Workflow

### Creating New Packages

**C++ package**:
```bash
cd ~/Github/DroneProjects/flyby-f11/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs <package_name>
```

**Python package**:
```bash
cd ~/Github/DroneProjects/flyby-f11/ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs <package_name>
```

**Package with custom messages**:
```bash
cd ~/Github/DroneProjects/flyby-f11/ros2_ws/src
ros2 pkg create --build-type ament_cmake <package_name>
cd <package_name>
mkdir msg srv action
# Create .msg, .srv, .action files
# Edit CMakeLists.txt and package.xml accordingly
```

### Launch Files

**Create launch file**:
```bash
cd ~/Github/DroneProjects/flyby-f11/ros2_ws/src/flyby_f11_bringup/launch
nano simulation.launch.py
```

**Example launch file** (`simulation.launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # ArduPilot SITL
        ExecuteProcess(
            cmd=['sim_vehicle.py', '-v', 'ArduCopter', '-f', 'gazebo-iris',
                 '--console', '--map', '--out=udp:127.0.0.1:14550'],
            cwd='~/ardupilot/ArduCopter',
            output='screen'
        ),

        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 '~/Github/DroneProjects/flyby-f11/simulation/worlds/test_world.world'],
            output='screen'
        ),

        # ArduPilot interface node
        Node(
            package='ardupilot_interface',
            executable='ardupilot_bridge',
            name='ardupilot_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Perception grounding nodes
        Node(
            package='perception_grounding',
            executable='object_grounding_node',
            name='object_grounding',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Ontology interface (Prolog reasoner)
        Node(
            package='ontology_interface',
            executable='prolog_reasoner_node',
            name='prolog_reasoner',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'rules_file': '~/Github/DroneProjects/flyby-f11/ontology/execution_mode/compiled_rules.pl'}
            ],
            output='screen'
        ),
    ])
```

**Launch simulation**:
```bash
cd ~/Github/DroneProjects/flyby-f11/ros2_ws
source install/setup.bash
ros2 launch flyby_f11_bringup simulation.launch.py
```

### ROS 2 Development Commands

**List running nodes**:
```bash
ros2 node list
```

**Inspect node**:
```bash
ros2 node info /prolog_reasoner
```

**List topics**:
```bash
ros2 topic list
ros2 topic echo /ontology/grounded_objects
ros2 topic hz /camera/image_raw
```

**Call service**:
```bash
ros2 service list
ros2 service call /ontology/query_safety ontology_interface/srv/QuerySafety "{action: 'move_forward'}"
```

**Trigger action**:
```bash
ros2 action list
ros2 action send_goal /navigate_to_waypoint ontology_rl/action/NavigateToWaypoint "{target_position: {x: 10.0, y: 5.0, z: -2.0}}"
```

**Record bag for debugging**:
```bash
ros2 bag record -a  # Record all topics
ros2 bag record /camera/image_raw /telemetry/pose /ontology/grounded_objects
```

**Replay bag**:
```bash
ros2 bag play <bag_directory>
```

## Debugging Tools and Techniques

### ROS 2 Debugging

**Enable debug logging**:
```bash
# Set log level for specific node
ros2 run --prefix 'env RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"' \
    ontology_interface prolog_reasoner_node --ros-args --log-level DEBUG
```

**rqt tools**:
```bash
# Node graph visualization
rqt_graph

# Topic monitor
rqt_topic

# Service caller
rqt_service_caller

# All-in-one rqt
rqt
```

**tf2 debugging** (for coordinate transforms):
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo transform
ros2 run tf2_ros tf2_echo base_link camera_link

# Monitor TF
ros2 topic echo /tf
```

### ArduPilot Debugging

**MAVLink inspector**:
```bash
# In MAVProxy console
set moddebug 3  # Verbose debugging
link list       # Show MAVLink connections
status          # Show system status
```

**Parameter debugging**:
```bash
# Check specific parameter
param show SERIAL2_PROTOCOL

# Search parameters
param show SERIAL*

# Compare against defaults
param diff
```

**Log analysis**:
```bash
# Logs stored in ~/ardupilot/ArduCopter/logs/
cd ~/ardupilot/ArduCopter/logs

# Convert to CSV
mavlogdump.py --format csv <logfile.bin> > log.csv

# Plot data
mavgraph.py <logfile.bin> "ATT.Roll" "ATT.Pitch"
```

### Vision Model Debugging

**Test YOLO detection**:
```bash
# Run standalone inference
cd ~/Github/DroneProjects/flyby-f11
source venv/bin/activate
python3 scripts/test_yolo.py --image test_images/sample.jpg
```

**Monitor GPU usage**:
```bash
# NVIDIA GPU
watch -n 1 nvidia-smi

# TensorRT profiling
trtexec --loadEngine=yolo11n.engine --verbose
```

**Visualize detections in ROS 2**:
```bash
# Image transport
ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_repub

# RViz visualization
rviz2
# Add Image display, topic: /perception/detection_image
```

### Prolog Debugging

**Test Prolog rules standalone**:
```bash
swipl
?- ['/home/finley/Github/DroneProjects/flyby-f11/ontology/execution_mode/compiled_rules.pl'].
?- canExecute(moveForward).
?- trace.  % Enable trace mode
?- mustAvoid(X).
```

**PySwip debugging**:
```python
from pyswip import Prolog

prolog = Prolog()
prolog.consult("/path/to/compiled_rules.pl")

# Enable trace
prolog.query("trace")

# Query with debugging
for solution in prolog.query("canExecute(moveForward)"):
    print(solution)
```

**Log Prolog queries in ROS 2**:
```cpp
// In prolog_reasoner_node.cpp
RCLCPP_DEBUG(this->get_logger(), "Prolog query: %s", query.c_str());
RCLCPP_DEBUG(this->get_logger(), "Prolog result: %s", result ? "true" : "false");
```

### Performance Profiling

**CPU profiling** (perf):
```bash
# Record performance data
perf record -g ros2 run ontology_interface prolog_reasoner_node

# View report
perf report
```

**Memory profiling** (valgrind):
```bash
valgrind --leak-check=full --show-leak-kinds=all \
    ros2 run ontology_interface prolog_reasoner_node
```

**ROS 2 performance analysis**:
```bash
# Install ros2_tracing
sudo apt install ros-humble-tracing-tools

# Trace session
ros2 trace start -s prolog_trace
ros2 run ontology_interface prolog_reasoner_node
ros2 trace stop prolog_trace

# Analyze with babeltrace
babeltrace ~/.ros/tracing/prolog_trace
```

## Code Style and Conventions

### Python Style

**Tool**: `ruff` (fast Python linter) + `black` (formatter)

**Configuration** (`.ruff.toml`):
```toml
line-length = 100
target-version = "py310"

[lint]
select = ["E", "F", "W", "I", "N", "UP", "B", "A", "C4", "T20"]
ignore = ["E501"]  # Line too long (handled by black)

[format]
quote-style = "double"
indent-style = "space"
```

**Run linting**:
```bash
# Check files
ruff check src/

# Auto-fix issues
ruff check --fix src/

# Format code
black src/
```

**Naming conventions**:
- Modules: `snake_case.py`
- Classes: `PascalCase`
- Functions/methods: `snake_case()`
- Constants: `UPPER_SNAKE_CASE`
- Private: `_leading_underscore()`

**Docstrings** (Google style):
```python
def ground_object_to_prolog(detection: Detection) -> str:
    """Convert YOLO detection to Prolog fact.

    Args:
        detection: YOLO detection object with class, bbox, confidence.

    Returns:
        Prolog fact string, e.g., "objectType(obj_123, 'person')."

    Raises:
        ValueError: If detection class is unknown.
    """
    pass
```

### C++ Style

**Tool**: `clang-format` (ROS 2 default) + `ament_uncrustify`

**Configuration** (`.clang-format`):
```yaml
BasedOnStyle: Google
IndentWidth: 2
ColumnLimit: 100
```

**Run formatting**:
```bash
# Format single file
clang-format -i src/ontology_interface/src/prolog_reasoner_node.cpp

# Format package
find src/ontology_interface -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i

# ROS 2 uncrustify check
ament_uncrustify --reformat src/ontology_interface/
```

**Naming conventions**:
- Classes: `PascalCase`
- Functions: `snake_case()`
- Member variables: `snake_case_`
- Constants: `kPascalCase`
- Namespaces: `snake_case`

**Header guards**:
```cpp
#ifndef ONTOLOGY_INTERFACE__PROLOG_REASONER_NODE_HPP_
#define ONTOLOGY_INTERFACE__PROLOG_REASONER_NODE_HPP_

// Content

#endif  // ONTOLOGY_INTERFACE__PROLOG_REASONER_NODE_HPP_
```

### ROS 2 Message/Service Style

**Message files** (`.msg`):
```
# ObjectGrounding.msg
std_msgs/Header header
string object_id
string object_type
geometry_msgs/Point position
float32 confidence
```

**Service files** (`.srv`):
```
# QuerySafety.srv
string action_name
geometry_msgs/Pose target_pose
---
bool is_safe
string[] violated_constraints
string explanation
```

**Action files** (`.action`):
```
# NavigateToWaypoint.action
geometry_msgs/Point target_position
---
bool success
string result_message
---
geometry_msgs/Pose current_pose
float32 distance_remaining
```

**Naming**:
- Message types: `PascalCase.msg`
- Field names: `snake_case`

### Git Commit Style

**Format**: Imperative mood, 50-char summary, detailed body

**Good examples**:
```
Add Prolog reasoner node for ontology queries

Implements SWI-Prolog integration via PySwip for runtime
reasoning. Supports canExecute, mustAvoid, and safety
constraint queries. Includes unit tests and benchmarks.

Closes #42
```

```
Fix coordinate frame transform in spatial grounding

NED to ENU conversion was missing Z-axis flip. Added
tf2 transform listener to properly handle camera to
base_link transformations.
```

**Bad examples**:
```
Fixed stuff
```

```
WIP prolog node
```

**Commit message prefixes** (optional):
- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation
- `test:` Tests
- `refactor:` Code refactoring
- `perf:` Performance improvement

### Documentation

**Code comments**:
- Use docstrings for public APIs
- Inline comments for complex logic only
- Prefer self-documenting code over comments

**Quarto documentation** (`.qmd` files):
- All technical docs use Quarto format
- Include YAML frontmatter (see CLAUDE.md)
- Use `quarto render <file.qmd>` to generate PDFs

**README files**:
- Every package has README.md
- Include: purpose, dependencies, usage, examples

## Continuous Integration (Future)

**GitHub Actions** (planned):
```yaml
# .github/workflows/ci.yml
name: ROS 2 CI

on: [push, pull_request]

jobs:
  build_and_test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble
      - name: Build workspace
        run: |
          cd ros2_ws
          colcon build --symlink-install
      - name: Run tests
        run: |
          cd ros2_ws
          colcon test
          colcon test-result --verbose
```

## Recommended Workflow

### Daily Development Cycle

1. **Morning setup**:
```bash
cd ~/Github/DroneProjects/flyby-f11/ros2_ws
source install/setup.bash
git pull origin main
colcon build --symlink-install
```

2. **Feature development**:
```bash
# Create feature branch
git checkout -b feat/spatial-grounding-node

# Implement feature
nano src/perception_grounding/src/spatial_grounding_node.cpp

# Build and test
colcon build --packages-select perception_grounding
ros2 run perception_grounding spatial_grounding_node

# Run tests
colcon test --packages-select perception_grounding
colcon test-result --verbose
```

3. **Code review preparation**:
```bash
# Format code
ruff check --fix .
black .
find src -name '*.cpp' | xargs clang-format -i

# Run full test suite
colcon test
colcon test-result

# Commit
git add .
git commit -m "Add spatial grounding node for n-ary relations"
git push origin feat/spatial-grounding-node
```

4. **Simulation testing**:
```bash
# Terminal 1: Launch simulation
ros2 launch flyby_f11_bringup simulation.launch.py

# Terminal 2: Monitor topics
ros2 topic echo /ontology/spatial_relations

# Terminal 3: Record data
ros2 bag record -a -o test_spatial_grounding
```

### Troubleshooting Checklist

**Build fails**:
- [ ] Check dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- [ ] Clean build: `rm -rf build/ install/ log/`
- [ ] Check compiler version: `gcc --version` (should be 11+)
- [ ] Verify ROS 2 sourced: `echo $ROS_DISTRO` (should be "humble")

**Runtime errors**:
- [ ] Source workspace: `source install/setup.bash`
- [ ] Check node running: `ros2 node list`
- [ ] Inspect logs: `ros2 run --prefix 'env RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"'`
- [ ] Verify parameters: `ros2 param list`

**SITL connection issues**:
- [ ] Check MAVLink port: `netstat -an | grep 14550`
- [ ] Verify SITL running: `ps aux | grep sim_vehicle`
- [ ] Test connection: `ros2 topic echo /ardupilot/telemetry/pose`

**Performance issues**:
- [ ] Profile CPU: `perf record -g <command>`
- [ ] Check memory: `htop` or `/proc/<pid>/status`
- [ ] Monitor ROS 2: `ros2 topic hz <topic>` (should be >10Hz for critical topics)

## Additional Resources

**ROS 2 Documentation**:
- Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- API Reference: https://docs.ros.org/en/humble/p/rclcpp/
- Best Practices: https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html

**ArduPilot Documentation**:
- Copter: https://ardupilot.org/copter/
- SITL: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- MAVLink: https://mavlink.io/en/

**Ontology Tools**:
- SWI-Prolog Manual: https://www.swi-prolog.org/pldoc/doc_for?object=manual
- PySwip: https://github.com/yuce/pyswip
- SUMO: https://www.ontologyportal.org/

**Project-Specific**:
- Architecture: `APPROACH.qmd`
- System Constraints: `SYSTEM_CONSTRAINTS.qmd`
- Ontology Foundation: `ONTOLOGY_FOUNDATION.qmd`
- Documentation Index: `docs/DOCUMENTATION_INDEX.md`

---

**Last Updated**: 2025-12-25
**Status**: Active development guide
