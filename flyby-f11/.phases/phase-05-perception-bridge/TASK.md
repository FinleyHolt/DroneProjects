# Phase 5: Perception-to-Reasoning Bridge

## Overview

Build the symbolic abstraction layer that translates sub-symbolic vision model outputs (bounding boxes, segmentation masks, depth maps) into symbolic Prolog facts for ontological reasoning. This bridges the gap between perception and reasoning.

## Human Description

Vision models output sub-symbolic representations that ontological reasoners cannot directly process. This phase creates ROS 2 "grounding nodes" that:

1. Take perception outputs (YOLO detections, segmentation, depth)
2. Convert them to ontology-aligned Prolog facts
3. Compute spatial relations (distance, between, northOf)
4. Detect temporal events (enters zone, loitering)
5. Assert facts into the Prolog knowledge base

This is the critical bridge that enables the ontology to reason about what the drone perceives in real-time.

## AI Agent Instructions

### Prerequisites
- Phase 4 completed (execution runtime with Prolog bridge)
- Understanding of ROS 2 message types
- Familiarity with computer vision outputs
- Knowledge of spatial reasoning concepts

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Create Perception Grounding Package**
   - Location: `ros2_ws/src/perception_grounding/`
   - ROS 2 package with Python nodes
   - Define custom message types for grounded facts
   - Set up launch file for all grounding nodes

2. **Implement ObjectGroundingNode**
   - Subscribe: YOLO detection messages
   - Process: Extract class, confidence, bounding box
   - Output: Prolog facts like `objectType(obj_123, person)`
   - Publish: GroundedObject messages to Prolog bridge

3. **Implement TerrainGroundingNode**
   - Subscribe: Segmentation mask messages
   - Process: Identify terrain types (water, urban, vegetation)
   - Output: Prolog facts like `terrainType(region_5, water)`
   - Include traversability assessment

4. **Implement SpatialRelationGroundingNode**
   - Subscribe: Depth images, object positions
   - Compute: Distance, between, northOf, clearance
   - Output: N-ary relations like `distance(drone, obj_123, 5.2)`
   - Handle coordinate frame transformations

5. **Implement EventDetectionNode**
   - Subscribe: Object tracking history
   - Detect: Zone entry/exit, loitering, approach
   - Output: Temporal facts like `enters(obj_123, no_fly_zone)`
   - Maintain temporal state machine

6. **Create Prolog Fact Publishers**
   - Integrate with prolog_bridge from Phase 4
   - Batch fact assertions for efficiency
   - Handle fact retraction (object leaves scene)
   - Implement fact timestamping

7. **Develop Integration Tests**
   - Test with simulated perception data
   - Verify Prolog facts are correctly asserted
   - Test spatial relation computations
   - Validate event detection logic

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `perception_grounding` package builds and installs
- [ ] ObjectGroundingNode converts YOLO detections to facts
- [ ] TerrainGroundingNode processes segmentation masks
- [ ] SpatialRelationGroundingNode computes spatial relations
- [ ] EventDetectionNode detects temporal events
- [ ] Facts correctly asserted into Prolog KB
- [ ] Integration tests pass with simulated data
- [ ] Latency from perception to fact < 50ms

### Verification

Run automated verification:
```bash
bash .phases/phase-05-perception-bridge/verification.sh
```

### Time Estimate
10-14 hours (includes all grounding nodes and integration testing)

### Common Pitfalls

- **Coordinate frames**: Ensure consistent TF frames across nodes
- **Race conditions**: Fact assertion/retraction timing
- **Object ID tracking**: Maintain consistent IDs across frames
- **Memory growth**: Retract old facts to prevent KB bloat
- **Type mismatches**: YOLO classes vs. ontology concepts
- **Quantization errors**: Depth discretization for distance

### References

- [ROS 2 Message Design](https://design.ros2.org/articles/interface_definition.html)
- [YOLO Detection Output Format](https://docs.ultralytics.com/modes/predict/)
- [TF2 Coordinate Frames](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [flyby-f11 APPROACH.qmd - Perception-to-Reasoning Bridge](../APPROACH.qmd)

### Dependencies
See `dependencies.json` - requires Phase 4 completion.

### Next Phase
After completion, proceed to Phase 6: Phase Transition Manager
