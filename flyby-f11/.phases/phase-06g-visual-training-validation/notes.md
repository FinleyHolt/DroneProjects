# Phase 6g Implementation Notes

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Isaac Sim GUI                                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐     │
│  │ Procedural World│  │   F-11 Drone    │  │    Viewport     │     │
│  │ - Terrain       │  │ - Pegasus       │  │    (visual)     │     │
│  │ - Trees         │  │ - PX4 SITL      │  │                 │     │
│  │ - Vehicles      │  │ - Sensors       │  │                 │     │
│  │ - People        │  │                 │  │                 │     │
│  └────────┬────────┘  └────────┬────────┘  └─────────────────┘     │
│           │                    │                                     │
│           ▼                    ▼                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    World Object Registry                      │   │
│  │   (positions of all spawned objects for GT perception)       │   │
│  └──────────────────────────┬──────────────────────────────────┘   │
└─────────────────────────────┼───────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Perception Pipeline                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐     │
│  │ GroundTruth     │  │ Perception      │  │ Temporal        │     │
│  │ Detector        │──▶ Encoder         │──▶ Tracker         │     │
│  │ (frustum math)  │  │ (516-dim)       │  │ (velocity est)  │     │
│  └─────────────────┘  └─────────────────┘  └────────┬────────┘     │
└─────────────────────────────────────────────────────┼───────────────┘
                                                      │
                                                      ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Ontology Layer                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐     │
│  │ Behavior        │  │ Vampire Safety  │  │ TPTP Generator  │     │
│  │ Controller      │◀─│ Filter          │◀─│ (facts)         │     │
│  │ (RTL, NFZ, etc) │  │ (action shield) │  │                 │     │
│  └────────┬────────┘  └─────────────────┘  └─────────────────┘     │
└───────────┼─────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         RL Layer                                     │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐     │
│  │ Gymnasium       │  │ SAC Agent       │  │ Replay Buffer   │     │
│  │ Wrapper         │──▶ (policy)        │──▶                 │     │
│  │ (obs/action)    │  │                 │  │                 │     │
│  └────────┬────────┘  └────────┬────────┘  └─────────────────┘     │
└───────────┼────────────────────┼────────────────────────────────────┘
            │                    │
            │                    ▼
            │           ┌─────────────────┐
            │           │ Training Logger │
            │           │ (JSONL files)   │
            │           └─────────────────┘
            │
            ▼
    ┌───────────────┐
    │ PX4 Action    │
    │ (velocity cmd)│
    └───────────────┘
```

## Key Implementation Details

### World Object Registry

The procedural world generator maintains a registry of all spawned objects:

```python
# From WorldGenerator
spawned_objects = {
    'trees': [(id, position, class_id), ...],
    'vehicles': [(id, position, class_id), ...],
    'people': [(id, position, class_id), ...],
}
```

This is converted to `WorldObject` instances for the GT detector:

```python
world_objects = create_world_objects_from_sim(spawned_objects)
detector.update_world_objects(world_objects)
```

### Ground Truth Detection Flow

1. Camera pose computed from drone position/orientation
2. Frustum constructed from camera FOV
3. AABB intersection test for each world object
4. Visible objects projected to image coordinates
5. Detection noise added (to match real YOLO behavior)
6. Detections sorted by priority (person > vehicle > building)

### Ontology Preemption

The `OntologyBehaviorController` checks state each step:

```python
behavior_command = ontology_controller.update(uav_state)

if behavior_command and behavior_command.preempts_rl:
    # Ontology takes over
    action = behavior_executor.execute(behavior_command, state)
else:
    # RL has control
    action = agent.predict(observation)
```

### Logging Format

**episodes.jsonl**:
```json
{"episode": 1, "steps": 247, "total_reward": 12.4, "termination": "rtl_complete", "timestamp": "..."}
```

**detections.jsonl**:
```json
{"step": 50, "episode": 1, "detections": [{"class": "person", "confidence": 0.95, "distance": 12.3}, ...]}
```

**ontology.jsonl**:
```json
{"step": 200, "episode": 1, "event": "RTL_TRIGGERED", "battery": 25.0, "distance_to_home": 145.0}
```

## Performance Considerations

- **GT mode target**: <1ms per observation (1000+ Hz theoretical)
- **Training loop target**: ~60 Hz with rendering
- **Memory**: World object registry scales with object count
- **Logs**: JSONL format for streaming writes (no memory accumulation)

## Known Limitations

1. **No real CV** - Uses GT mode, not actual YOLO inference
2. **Single drone** - Multi-agent not implemented
3. **No ROS 2** - Direct Isaac Sim integration only
4. **Simplified physics** - Uses Pegasus defaults, not full F-11 dynamics model

## Debugging Tips

1. **Drone doesn't arm**: Check PX4 SITL logs, ensure MAVLink connected
2. **No detections**: Verify world objects are spawned, check frustum FOV
3. **Ontology not triggering**: Check battery drain rate, lower threshold for testing
4. **Training crashes**: Check for NaN in observations, verify action bounds
