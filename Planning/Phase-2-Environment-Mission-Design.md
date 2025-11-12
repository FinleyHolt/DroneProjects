# Phase 2 - Environment & Mission Design

**Status**: Not Started
**Dependencies**: Phase 1 (working PX4 SITL)

## Objective
Create the operational context for our drone: custom drone configuration, realistic Gazebo simulation environments, and define 2-3 concrete example missions that will drive the development of action primitives in Phase 3.

## Rationale
Before we can build LLM capabilities, we need to know:
- What kind of drone are we simulating (sensors, flight characteristics)?
- What environments will it operate in (urban, outdoor, obstacles)?
- What specific tasks do we want it to accomplish?
- **What coordinate systems will we use** (WGS84 GPS, local ENU, Gazebo world frame)?

This phase establishes the **requirements** that will inform what actions, perception models, and navigation capabilities we need to build.

**Critical Decision**: Define coordinate system standards early to prevent Phase 3 integration hell.

## Goals
1. **Coordinate System Definition**: Establish standards for GPS (WGS84), local planning (ENU), and Gazebo frames
2. **Custom Drone Configuration**: Create a drone variant with realistic sensor suite (camera, GPS, etc.)
3. **Simulation Environments**: Build 2-3 Gazebo worlds that represent operational scenarios
4. **Mission Definitions**: Document 2-3 example missions with success criteria and interfaces
5. **Manual Mission Validation**: Fly each mission manually with quantitative performance metrics

## Success Criteria
- [ ] Custom drone config in `px4-config/llm_drone_v1/` with parameters and mixer
- [ ] 2-3 Gazebo worlds in `sim/worlds/` representing different scenarios
- [ ] Mission definitions documented with objectives, constraints, and success criteria
- [ ] Each mission manually flown at least once in simulation to prove viability
- [ ] Git LFS configured for large Gazebo assets (models, textures)

## Tasks

### 2.0 Coordinate System Standards

**Critical Foundation**: Define coordinate system conventions to prevent integration issues.

- [ ] Document coordinate system hierarchy in `docs/coordinate_systems.md`:
  - **WGS84 GPS** (lat/lon/alt): Mission boundaries, waypoints, detection geotags
  - **Local ENU** (East-North-Up meters): Path planning, obstacle avoidance, local navigation
  - **Gazebo World Frame**: Simulation-specific, maps to local ENU origin
  - **PX4 Body Frame**: Vehicle-relative (Forward-Right-Down)

- [ ] Define conversion interfaces:
  - WGS84 → Local ENU: Use home position as origin (set in PX4 params)
  - Local ENU → Gazebo: Direct mapping with documented origin
  - GPS geotag format: JSON `{"lat": float, "lon": float, "alt_msl": float, "alt_agl": float}`

- [ ] Establish validation requirements:
  - All mission boundaries must be WGS84 polygons (GeoJSON format)
  - ROS 2 interfaces use `sensor_msgs/NavSatFix` for GPS, `geometry_msgs/PoseStamped` for local ENU
  - Phase 3 MAVSDK bridge must publish both frames

- [ ] Set home position conventions:
  - Gazebo world origin (0,0,0) maps to documented GPS coordinates
  - Example: Urban world origin = (38.1234, -76.5678, 0m MSL)
  - Document in each world's README

**Deliverable**: Coordinate system standards document preventing Phase 3 coordinate hell

### 2.1 Custom Drone Configuration

#### Create Drone Variant
- [ ] Create `px4-config/llm_drone_v1/` directory
- [ ] Define drone characteristics:
  - Frame type: Quadrotor X configuration
  - Sensor suite: Camera, GPS, IMU, barometer (standard SITL sensors)
  - Flight characteristics: Stable, moderate speed (good for autonomous nav)
- [ ] Create `params.params` with tuned parameters:
  - **Start with stock PX4 quadrotor parameters** (document baseline)
  - Conservative gains for stable autonomous flight (note: will require tuning in Phase 3)
  - Geofencing parameters (altitude limits, return-to-home)
  - Mission mode settings
  - **GPS home position** matching Gazebo world origin
- [ ] Document in `px4-config/llm_drone_v1/README.md`:
  - Rationale for parameter choices
  - Sensor specifications and mounting
  - Expected performance characteristics
  - **Parameter tuning roadmap** (defer detailed tuning to Phase 3 when autonomous flight tested)

#### Gazebo Model Integration
- [ ] Verify which PX4 SITL model to base on (likely `iris` quadrotor)
- [ ] Note any custom Gazebo model requirements for Phase 3 (camera plugins, etc.)
- [ ] Test drone spawns correctly with `make sim`

**Deliverable**: Custom drone config that spawns in Gazebo with documented characteristics

### 2.2 Simulation Environment Creation

#### Environment 1: Urban Search Area
**Purpose**: Vehicle detection in urban environment with roads and buildings

- [ ] Create `sim/worlds/urban_search.world`
- [ ] Design features:
  - Grid of roads (simple street layout)
  - 5-10 simple buildings (boxes with textures)
  - 3-5 vehicle models placed on roads (targets for detection)
  - Defined "Named Areas of Interest" (NAI regions) with **WGS84 GPS boundaries**
  - Appropriate lighting and terrain
  - **Documented GPS origin** (e.g., 38.1234°N, -76.5678°W)
- [ ] Set up Git LFS for large assets:
  - **Only track binary model files**: `git lfs track "*.dae"`, `git lfs track "*.stl"`, `git lfs track "*.png"`
  - **Do NOT track .world files** (they are text XML, <1MB, bloats LFS quota unnecessarily)
  - Track model meshes and textures only
- [ ] Test world loads in Gazebo with custom drone
- [ ] Document world layout and key coordinates in `sim/worlds/urban_search_README.md`:
  - GPS origin and coordinate mapping
  - NAI 3 boundary polygon (WGS84 GeoJSON)
  - Vehicle locations (GPS coords)
  - Recommended flight altitudes for visibility

#### Environment 2: Outdoor Reconnaissance Area
**Purpose**: Area coverage and waypoint navigation in open terrain

- [ ] Create `sim/worlds/outdoor_recon.world`
- [ ] Design features:
  - Open terrain with gentle elevation changes
  - Scattered vegetation/obstacles
  - Multiple potential landing zones
  - Defined survey area boundaries
  - Reference landmarks
- [ ] Test world performance (frame rate, physics stability)
- [ ] Document mission-relevant coordinates

#### Environment 3 (Optional): Obstacle Course
**Purpose**: Dynamic replanning and obstacle avoidance

- [ ] Create `sim/worlds/obstacle_course.world` if time permits
- [ ] Design features:
  - Narrow passages
  - Dynamic obstacles (if Gazebo plugins available)
  - Multiple path options
- [ ] This is a stretch goal for Phase 5 testing

**Deliverable**: 2-3 Gazebo worlds with documented features and coordinates

### 2.3 Mission Definition

#### Mission 1: Urban Vehicle Search
**Objective**: "Survey NAI 3 and identify vehicle movement"

- [ ] Define mission in `Planning/missions/01-urban-vehicle-search.md`:
  - **Objective**: Locate and count vehicles in defined search area
  - **Environment**: `urban_search.world`
  - **Start Position**: GPS coordinates (WGS84) and Gazebo local coords
  - **Search Area**: NAI 3 boundary (WGS84 GeoJSON polygon)
  - **Success Criteria** (quantitative):
    - Detection recall ≥90% (detect ≥3 of 3 vehicles placed in world)
    - Detection precision ≥80% (false positive rate <20%)
    - GPS geotag error <5m (compare detection coords to ground truth)
    - Return to home within 2m of launch position
    - Mission time <10 minutes
    - Battery remaining >20%
  - **Constraints**:
    - Altitude limits (30-50m AGL)
    - Geofence boundaries (WGS84 polygon)
    - Battery reserve for RTH (20%)
    - Maximum flight time: 15 minutes

- [ ] List required capabilities with **interface specifications**:
  - Waypoint navigation → ROS 2 service `/control/goto_waypoint(NavSatFix) → success`
  - Area coverage pattern → service `/control/plan_coverage(Polygon) → Path`
  - Object detection → topic `/perception/detections` publishing `DetectedObject[]`
  - GPS geolocation → `DetectedObject` includes `NavSatFix gps_location`
  - Return-to-home → service `/control/rtl() → success`

  **Note**: These interface specs drive Phase 3 ROS 2 package design.

#### Mission 2: Open Area Reconnaissance
**Objective**: "Conduct reconnaissance of sector AB-1234 and report any anomalies"

- [ ] Define mission in `Planning/missions/02-open-area-recon.md`:
  - **Objective**: Survey defined area and detect anomalies
  - **Environment**: `outdoor_recon.world`
  - **Search Area**: Sector coordinates
  - **Success Criteria**:
    - Complete coverage of area (>90%)
    - Anomaly detection (non-terrain objects)
    - Photographic evidence with GPS tags
  - **Constraints**:
    - Weather limits (wind handled by simulation)
    - Time on station
    - Communication range (simulated)

- [ ] List required capabilities:
  - Path planning for area coverage
  - Anomaly detection (basic change detection)
  - Adaptive altitude adjustment
  - Data logging

#### Mission 3 (Stretch): Dynamic Replanning
**Objective**: "Navigate to waypoint DELTA and return, adapt to obstacles"

- [ ] Define mission in `Planning/missions/03-adaptive-navigation.md`:
  - **Objective**: Reach waypoint with obstacle avoidance
  - **Environment**: `obstacle_course.world` (if created)
  - **Success Criteria**:
    - Reach target waypoint
    - Avoid all obstacles (no collisions)
    - Replan route when blocked
  - **Constraints**:
    - No predefined path
    - Must adapt in real-time

- [ ] This is a Phase 5 validation mission

**Deliverable**: 2-3 mission definition documents with objectives, constraints, and required capabilities

### 2.4 Manual Mission Validation

For each mission defined:

#### Mission 1 Manual Flight
- [ ] Spawn drone in `urban_search.world`
- [ ] Manually command waypoint navigation using PX4 console or QGroundControl
- [ ] Fly coverage pattern over NAI 3
- [ ] **Quantitative validation**:
  - Record flight time (target <10 min)
  - Measure position tracking error (log GPS vs commanded waypoints, calculate RMSE)
  - Verify all vehicle locations are reachable (altitude provides >60° FOV coverage)
  - Log battery consumption rate (%/minute)
  - Measure RTH landing accuracy (distance from home)
- [ ] Document results in `logs/phase2-manual-flight/mission1_results.csv`:
  - Flight time, position RMSE, battery drain, landing error
  - Compare against success criteria thresholds
- [ ] Take screenshots and screen recording of successful execution

#### Mission 2 Manual Flight
- [ ] Spawn drone in `outdoor_recon.world`
- [ ] Manually fly area coverage pattern
- [ ] Verify terrain is navigable
- [ ] Document coverage pattern effectiveness
- [ ] Record lessons learned

#### General Validation
- [ ] Confirm simulation is stable for mission duration
- [ ] Verify GPS accuracy and world coordinates
- [ ] Test return-to-home from various positions
- [ ] Document any Gazebo performance issues
- [ ] Identify parameters that need tuning

**Deliverable**: Flight logs and screenshots proving missions are manually achievable

## Deliverables Checklist
- [ ] Custom drone config: `px4-config/llm_drone_v1/`
- [ ] Gazebo worlds: `sim/worlds/urban_search.world`, `sim/worlds/outdoor_recon.world`
- [ ] Git LFS configured for large simulation assets
- [ ] Mission definitions: `Planning/missions/*.md` (2-3 missions)
- [ ] Manual flight validation logs/screenshots
- [ ] Updated README documenting how to launch each world

## Known Risks and Mitigation

### Risk: Gazebo performance issues with complex worlds
**Impact**: Medium
**Mitigation**: Start simple, add detail incrementally; test on target hardware early

### Risk: Mission requirements too complex for Phase 3 implementation
**Impact**: High
**Mitigation**: Validate required capabilities exist/are feasible during manual flights; adjust mission scope if needed; prioritize Mission 1

### Risk: Coordinate system confusion between GPS, local ENU, and Gazebo frames
**Impact**: Critical (causes Phase 3 integration failures)
**Mitigation**: Task 2.0 establishes standards early; document conversions; validate in manual flights

### Risk: Missions define capabilities Phase 3 can't deliver (e.g., real-time object tracking)
**Impact**: High
**Mitigation**: Mission definitions now include interface specs; review with Phase 3 architecture before locking

### Risk: PX4 tuning takes longer than expected
**Impact**: Low
**Mitigation**: Start with stock parameters, tune as needed; not critical for Phase 2 exit

## Phase Exit Criteria
Before moving to Phase 3, verify:
1. ✓ Coordinate system standards documented (`docs/coordinate_systems.md`)
2. ✓ Custom drone spawns correctly in all environments with documented GPS home position
3. ✓ At least 2 missions defined with quantitative success criteria and interface specifications
4. ✓ Each mission manually flown at least once with recorded performance metrics
5. ✓ Performance metrics meet success thresholds (position RMSE, detection visibility, etc.)
6. ✓ Required capabilities list validated against Phase 3 feasibility
7. ✓ Simulation is stable and performant (>30 FPS, no crashes)
8. ✓ Git LFS configured for binary assets only (not .world files)

## Next Phase
Once Phase 2 is complete, proceed to [Phase 3 - Action Vocabulary](Phase-3-Action-Vocabulary.md) to build the fundamental capabilities (navigation, perception, actions) required to execute these missions autonomously.

The mission definitions from this phase become the **requirements** that drive Phase 3 development.

## Notes and Findings
_Use this section to document design decisions, lessons from manual flights, and insights about mission requirements._

---

**Phase Started**: [Date]
**Phase Completed**: [Date]

**Key Decisions**:
- [Mission priorities]
- [World design choices]
- [Drone configuration rationale]

**Lessons from Manual Flights**:
- [What worked]
- [What needs improvement]
- [Unexpected challenges]
