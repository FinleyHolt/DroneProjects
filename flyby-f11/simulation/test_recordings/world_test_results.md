# Flyby F-11 Training World Test Results

**Test Date:** 2025-12-27
**Test Method:** Headless Gazebo Harmonic in container
**Container:** localhost/flyby-f11-sim:latest

## Summary

| World | Status | Models | NFZ Zones | Landing Zones | Camera |
|-------|--------|--------|-----------|---------------|--------|
| urban_training | PASS | 32 | 2 | 4 | Active |
| rural_training | PASS | 31 | 2 | 4 | Active |
| industrial_training | PASS | 36 | 2 | 4 | Active |
| coastal_training | PASS | 30 | 2 | 4 | Active |
| randomized_training | PASS | 22 | 2 | 4 | Active |

## Detailed Results

### 1. urban_training.sdf
**World Name:** `flyby_urban`
**Theme:** Dense urban environment - Phoenix, AZ area
**Area:** 300m x 300m

**Models Loaded (32):**
- Buildings: A1 (50m), A2 (30m), A3 (15m), B1 towers (40m), B2 (25m), C1 (45m), C3 (20m)
- Small buildings: 4 structures (10-14m)
- Special: Hospital (35m), Government building (30m)
- Vehicles: 4 cars, 1 truck
- NFZ zones: Hospital helipad (cylinder), Government building (box)
- Landing zones: Primary (ground), Rooftop A1, Rooftop B2, Emergency (SW corner)
- Drone: f11_isr spawned at origin

**Observations:**
- Grid layout creates street canyons for navigation challenges
- Helipad marker visible on hospital roof
- Building heights vary from 10m to 50m (good vertical diversity)
- Geofence boundary markers present on all 4 sides

### 2. rural_training.sdf
**World Name:** `flyby_rural`
**Theme:** Forest/natural terrain - Northern California
**Area:** 400m x 400m

**Models Loaded (31):**
- Trees: 8 individual trees with trunk/canopy (heights 8-14m)
- Terrain: 2 hills (hill_ne, hill_sw), 1 ridge
- Rocks: 2 rock formations, 2 boulders
- Structures: 1 cabin (private property)
- NFZ zones: Wildlife nesting site (cylinder), Private property (box)
- Landing zones: Primary (clearing), Meadow (NE), Hilltop, Emergency (SW)
- Drone: f11_isr spawned at origin

**Observations:**
- Good natural obstacle diversity with trees and terrain
- Tree canopies at varying heights create vertical obstacles
- Hills provide elevation changes (6m, 8m, 5m)
- Larger operational area (400m vs 300m urban)

### 3. industrial_training.sdf
**World Name:** `flyby_industrial`
**Theme:** Port/warehouse environment - Long Beach area
**Area:** 350m x 350m

**Models Loaded (36):**
- Warehouses: 3 main buildings (12-20m height)
- Shipping containers: 9 stacked containers (2-3 high)
- Equipment: Gantry crane (30m), 2 forklifts, pallets
- Storage: 2 cylindrical tanks, chemical storage building
- Office building (20m)
- NFZ zones: Chemical storage area (cylinder), Active crane zone (box)
- Landing zones: Primary, Warehouse rooftop, Distribution rooftop, Emergency
- Drone: f11_isr spawned at origin

**Observations:**
- Container stacks create multi-level obstacles
- Crane structure (30m) is tallest obstacle
- Industrial clutter (forklifts, pallets) at ground level
- Hazard sign model present near chemical storage

### 4. coastal_training.sdf
**World Name:** `flyby_coastal`
**Theme:** Maritime/harbor environment - San Diego area
**Area:** 400m x 400m (50% water, 50% land)

**Models Loaded (30):**
- Water: water_surface, beach_slope
- Docks: Main pier, dock_east, dock_west, 3 pier supports
- Boats: 2 fishing boats, 1 sailboat, 1 cargo vessel, 1 kayak
- Buildings: Harbor master (10m), boat house (7m), lighthouse (25m)
- Beach: 2 beach huts
- NFZ zones: Shipping lane (large box), Restricted harbor (cylinder)
- Landing zones: Primary (beach), Pier, Floating platform, Emergency
- Drone: f11_isr spawned on beach (y=50)

**Observations:**
- Water surface provides unique visual contrast
- Floating landing platform over water
- Lighthouse is prominent vertical obstacle
- Cargo vessel in shipping lane is large obstacle

### 5. randomized_training.sdf
**World Name:** `flyby_randomized`
**Theme:** Mixed obstacle template for domain randomization
**Area:** 350m x 350m

**Models Loaded (22):**
- Obstacles: 3 buildings, 1 tower, 1 tank/silo, 1 pole, 1 tree, 1 low structure
- NFZ zones: Zone 1 (cylinder), Zone 2 (box), plus ground facilities
- Landing zones: Primary, Secondary, Rooftop, Emergency
- Drone: f11_isr spawned at origin

**Observations:**
- Designed as base template for procedural generation
- Fewer models but higher variability potential
- All obstacles marked as "randomizable" in comments
- Compatible with scripts/randomize_world.py

## Technical Findings

### Camera System
All worlds correctly expose ISR camera topics:
- `/world/{world_name}/model/f11_isr/link/camera_link/sensor/isr_camera/image`
- `/world/{world_name}/model/f11_isr/link/camera_link/sensor/isr_camera/camera_info`

Camera specifications (from model.sdf):
- Resolution: 1920x1080
- Format: R8G8B8
- FOV: 60 degrees (1.047 rad)
- Update rate: 30 Hz
- Noise: Gaussian (stddev=0.007)

### Gimbal System
All worlds load gimbal control topics:
- `/f11_isr/gimbal/yaw` (continuous rotation)
- `/f11_isr/gimbal/pitch` (±120 degrees)
- `/f11_isr/gimbal/roll` (±45 degrees)

### Known Issues
1. **ArduPilot Plugin Warning:** `Failed to load system plugin [ArduPilotPlugin]`
   - This is expected in Gazebo-only mode without SITL
   - Plugin loads correctly when running with full entrypoint

2. **Container World Files:** New training worlds must be mounted via volume
   - Container has only training_world.sdf and rl_training_v2.sdf baked in
   - Use: `-v ./simulation/worlds:/simulation/worlds:z`

## Recommendations

### A. Issues to Fix
1. **None critical** - All worlds load and render correctly
2. **Minor:** Consider rebuilding container to include all 5 training worlds

### B. Ready for RL Training Integration
The worlds are ready for use with the following validated features:
- Diverse obstacle types and heights
- NFZ boundaries correctly visualized (red transparent volumes)
- Landing zones visible (white/yellow/orange circles)
- F-11 drone with all sensors spawns correctly
- Geofence boundaries marked (green lines)

### C. Suggested Enhancements (Optional)
1. Add dynamic obstacles (moving vehicles, boats)
2. Add wind effects via Gazebo atmosphere plugin
3. Add time-of-day variations (already supported in randomize_world.py)
4. Add weather effects (rain, fog) for perception challenges

## Test Command
```bash
podman run --rm \
    --entrypoint /bin/bash \
    -v ./simulation/worlds:/simulation/worlds:z \
    localhost/flyby-f11-sim:latest \
    -c "source /opt/ros/humble/setup.bash && \
        Xvfb :99 -screen 0 1920x1080x24 & \
        export DISPLAY=:99 && \
        gz sim -s /simulation/worlds/{WORLD_NAME}.sdf"
```
