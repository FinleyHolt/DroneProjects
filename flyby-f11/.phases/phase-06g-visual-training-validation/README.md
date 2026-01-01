# Phase 6g: Visual Training Validation

## Overview

End-to-end visual validation of the complete training pipeline. Run the full stack with GUI to verify all components are connected and working together.

**Goal**: Watch the drone fly in the procedural world while training runs, with all perception, ontology, and RL components active.

## What This Phase Validates

1. **Procedural World Generation** - Terrain, trees, vehicles, people spawn correctly
2. **Drone + PX4 Integration** - F-11 spawns, arms, flies via Pegasus/PX4 SITL
3. **Ground Truth Perception** - Frustum-based detection produces valid 516-dim observations
4. **Ontology Behavior Controller** - Safety behaviors trigger (RTL, NFZ avoidance, emergency land)
5. **RL Training Loop** - Episodes run, rewards computed, resets work
6. **Full Data Flow** - Sim → Perception → Ontology → RL → Action → Sim

## Components Exercised

| Component | Mode | Purpose |
|-----------|------|---------|
| WorldGenerator | Full | Procedural terrain, vegetation, objects |
| Pegasus/PX4 | SITL | Flight control, MAVLink |
| DualModePerception | GT mode | Fast frustum-based detection |
| OntologyBehaviorController | Active | Safety behavior preemption |
| VampireSafetyFilter | Active | Action shielding |
| Gymnasium Wrapper | Active | SB3-compatible interface |
| SAC Agent | Training | RL policy learning |

## Usage

```bash
# Launch visual training demo (5 episodes)
/isaac-sim/python.sh /workspace/scripts/visual_training_demo.py --episodes 5

# Longer run for more thorough validation
/isaac-sim/python.sh /workspace/scripts/visual_training_demo.py --episodes 20 --log-dir /workspace/logs

# Headless mode for CI (still validates everything, just no GUI)
/isaac-sim/python.sh /workspace/scripts/visual_training_demo.py --headless --episodes 3
```

## What You'll See

1. **Isaac Sim GUI opens** with procedurally generated world
2. **Drone spawns** at origin, arms via PX4
3. **Training begins** - drone flies executing random/learned actions
4. **Console shows**:
   - Detection counts per step
   - Ontology status (geofence, NFZ, battery)
   - Reward signals
   - Episode boundaries
5. **Episodes reset** when done/truncated
6. **Logs written** to specified directory

## Console Output Example

```
======================================================================
 VISUAL TRAINING DEMO - Phase 6g Validation
======================================================================
World: 200x200m terrain, 847 trees, 12 vehicles, 8 people
Drone: F-11 ISR @ (0, 0, 0.5)
Perception: GT mode (516-dim)
Training: SAC, 5 episodes

[Episode 1] Starting...
  Step 50: detections=3 (person:1, vehicle:2) | reward=0.12 | battery=98%
  Step 100: detections=5 | reward=0.08 | ontology=OK
  Step 150: detections=2 | reward=-0.5 | NFZ_WARNING (dist=15m)
  Step 200: RTL triggered (battery=25%) | ontology preempting
  Step 247: Episode done (RTL complete) | total_reward=12.4

[Episode 2] Starting...
...

======================================================================
 VALIDATION COMPLETE
======================================================================
Episodes: 5 | Total steps: 1,247 | Avg reward: 10.2
Perception: 1,247 observations | avg_time=0.8ms | 4,821 detections
Ontology: 3 RTL triggers | 1 NFZ avoidance | 0 emergency lands
Safety: 47 interventions | 0 hard violations

Logs saved to: /workspace/logs/training_20260101_170000/
```

## Log Files

```
logs/training_<timestamp>/
├── episodes.jsonl       # Episode summaries
├── detections.jsonl     # All detections with timestamps
├── ontology.jsonl       # Ontology decisions and violations
├── rewards.jsonl        # Per-step rewards
└── perception_stats.json # Perception pipeline statistics
```

## Success Criteria

- [ ] GUI opens with procedural world (terrain, trees, objects visible)
- [ ] Drone spawns and arms (PX4 heartbeat received)
- [ ] Drone flies (position changes, velocity non-zero)
- [ ] Perception produces observations (516-dim, non-zero, no NaN)
- [ ] Ontology triggers behaviors when appropriate (RTL at low battery)
- [ ] Episodes complete and reset correctly
- [ ] No crashes during multi-episode run
- [ ] Logs capture full data flow

## Dependencies

- Phase 6e: Isaac Sim + PX4 + Pegasus integration
- Phase 6f: Dual-mode perception pipeline
- Ontology: Planning mode axioms for safety behaviors

## Files

```
scripts/
└── visual_training_demo.py    # Main entry point

logging/
└── training_logger.py         # Structured logging utilities
```
