# Phase 6g Objectives

## Primary Objective

Validate the complete training pipeline end-to-end by running it visually and observing all components working together.

## Specific Objectives

### 1. Visual Confirmation of World Generation
- Procedural terrain renders correctly
- Trees, vehicles, people are visible in scene
- Lighting and atmosphere appropriate

### 2. Drone Flight Validation
- F-11 spawns at correct position
- PX4 SITL connects (heartbeat)
- Drone arms and takes off
- Velocity commands execute correctly
- Drone responds to RL actions

### 3. Perception Pipeline Validation
- Ground truth detector receives world object positions
- Frustum culling identifies visible objects
- 516-dimensional observations are produced
- Observations are valid (no NaN/Inf, reasonable ranges)
- Detection counts match visible objects

### 4. Ontology Integration Validation
- OntologyBehaviorController monitors state
- RTL triggers at battery threshold (25% normal, 30% comms-denied)
- NFZ avoidance activates when approaching no-fly zones
- Geofence recovery triggers on boundary breach
- Emergency land triggers at critical battery (15%)
- Ontology preempts RL when safety-critical

### 5. RL Training Loop Validation
- Gymnasium wrapper produces valid observations
- Actions are normalized [-1, 1] and scaled correctly
- Rewards are computed per step
- Episodes terminate on done conditions
- Episodes truncate at max steps
- Environment resets correctly between episodes
- SAC agent can learn (buffer fills, updates happen)

### 6. Data Flow Validation
- World objects → Ground truth detector
- Detections → Perception encoder → 516-dim observation
- Observation → RL agent → Action
- Action → Safety filter → Safe action
- Safe action → PX4 → Drone movement
- Drone state → Ontology controller → Behavior decisions

### 7. Logging Validation
- All detections logged with timestamps
- Ontology decisions logged
- Rewards logged per step
- Episode summaries captured
- Logs are valid JSON/JSONL

## Non-Objectives (Out of Scope)

- Real YOLO inference (use GT mode for speed)
- CV accuracy metrics (separate validation)
- Policy performance benchmarks (just verify training runs)
- Multi-drone scenarios
- ROS 2 integration (direct Isaac Sim only)

## Acceptance Criteria

| Criterion | Measurement |
|-----------|-------------|
| World loads | Terrain and objects visible in GUI |
| Drone flies | Position changes > 10m from spawn |
| Perception works | 516-dim observations, <1% NaN |
| Ontology works | At least 1 RTL trigger in 5 episodes |
| Training runs | 5 episodes complete without crash |
| Logs valid | All JSONL files parse correctly |

## Definition of Done

Phase 6g is complete when:

1. `visual_training_demo.py` script exists and runs
2. Running the script opens Isaac Sim with procedural world
3. Drone spawns, arms, and flies
4. Console output shows detections, rewards, ontology status
5. 5 episodes complete without crash
6. Log files are written and contain valid data
7. At least one ontology behavior (RTL) is triggered during the run
