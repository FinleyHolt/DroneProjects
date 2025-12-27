# Flyby F-11 Development Phases

This directory contains machine-readable task definitions for the Flyby F-11 development workflow. The structure is optimized for **AI agent task execution**, enabling automated sequential implementation with verification.

## Architectural Decision: Single-Reasoner (Vampire Only)

**Phase 3 Evaluation Outcome (Completed 2024-12-25)**

After comprehensive benchmarking, we determined that a **single-reasoner architecture using Vampire** is optimal:

| Finding | Impact |
|---------|--------|
| Vampire ~50ms acceptable for 20Hz navigation | No Prolog translation needed |
| OWL reasoners cannot express safety axioms | ELK/Reasonable rejected |
| KIF/SUMO remains single source of truth | 0% translation loss |

### Tiered Safety Architecture

| Tier | Layer | Latency | Function | Reasoner |
|------|-------|---------|----------|----------|
| 1 | Classical Control | <1ms | PID, motor control | None |
| 2 | Pre-computed Safety | <10ms | Obstacle buffers, geofence | Costmaps |
| 3 | Tactical Reasoning | ~50ms | NFZ violation, battery critical | **Vampire** |
| 4 | Mission Planning | ~1s | Route planning, compliance | **Vampire** |

See [Phase 3 Evaluation Report](../ontology/evaluation/EVALUATION_REPORT.qmd) for complete analysis.

## Directory Structure

Each phase is a self-contained directory with standardized files:

```
.phases/
├── phase-01-ontology-toolchain/         [COMPLETE]
│   ├── TASK.md
│   ├── inputs.json
│   ├── outputs.json
│   ├── verification.sh
│   └── dependencies.json
├── phase-02-uav-ontology/               [COMPLETE]
│   └── ...
├── phase-03-evaluation/                 [COMPLETE] - Reasoner benchmarking
│   └── ...
├── phase-04-execution-mode/             [RENAMED: Vampire Runtime Integration]
│   └── ...
├── phase-05-perception-bridge/          [Updated for Vampire/TPTP]
│   └── ...
├── phase-06-phase-transition/           [SIMPLIFIED: Mission Orchestration]
│   └── ...
├── phase-06a-isr-sensor-integration/    [NEW] F-11 ISR sensor packages
│   └── ...
├── phase-06b-simulation-training-env/   [NEW] Gymnasium RL environments
│   └── ...
├── phase-06c-diverse-training-worlds/   [NEW] Multiple Gazebo training worlds
│   └── ...
├── phase-06d-perception-pipeline/       [NEW] Camera to TPTP facts pipeline
│   └── ...
└── README.md
```

## Phase Organization

Phases are numbered sequentially. **Phases 1-3 complete, Phase 4+ updated for single-reasoner architecture:**

| Phase | Name | Description | Status |
|-------|------|-------------|--------|
| 01 | ontology-toolchain | SUMO + Vampire planning mode setup | COMPLETE |
| 02 | uav-ontology | UAV domain knowledge base in KIF | COMPLETE |
| 03 | evaluation | Runtime reasoner benchmarking & selection | **COMPLETE** |
| 04 | vampire-runtime | Vampire ARM64 + ROS 2 integration | COMPLETE |
| 05 | perception-bridge | ISR sensor-to-TPTP grounding nodes | COMPLETE |
| 06 | mission-orchestration | Mission lifecycle management (simplified) | COMPLETE |
| **6a** | **isr-sensor-integration** | **F-11 ISR sensor ROS 2 packages** | **NEW** |
| **6c** | **diverse-training-worlds** | **Multiple Gazebo worlds for domain randomization** | **NEW** |
| **6d** | **perception-pipeline** | **Camera to TPTP facts (YOLO + segmentation)** | **NEW** |
| **6b** | **simulation-training-env** | **Gymnasium RL environments + training loop** | **UPDATED** |
| 07 | mission-planner-rl | Level 1 SAC agent (10s horizon) | Pending |
| 08 | behavior-selector-rl | Level 2 PPO agent (1s horizon) | Pending |
| 09 | trajectory-optimizer-rl | Level 3 TD3 agent (100ms horizon) | Pending |
| 10 | aggm-integration | Automatic Goal Generation Model | Pending |
| 11 | simulation-benchmark | Benchmark suite and evaluation | Pending |
| 12 | project-drone-validation | Hardware testing on Jetson Orin Nano | Pending |
| 13 | flyby-f11-integration | Deployment to Flyby F-11/Orin NX | Pending |
| 14 | documentation-release | Paper, docs, open-source release | Pending |

### Key Changes from Original Plan

1. **Phase 3 renamed**: `translation` → `evaluation` (Prolog translation eliminated)
2. **Phase 4 rewritten**: Prolog runtime → Vampire ARM64 + ROS 2 integration
3. **Phase 5 updated**: Generic vision → F-11 ISR sensor grounding (Gremsy, LiDAR, thermal)
4. **Phase 6 simplified**: Container swapping → Simple mission orchestration
5. **Phase 6a added**: ISR sensor ROS 2 packages (flyby_f11_sensors, flyby_f11_bringup)
6. **Phase 6c added**: Diverse training worlds for domain randomization (prevents RL overfitting)
7. **Phase 6d added**: Perception pipeline converting camera images to TPTP facts for Vampire
8. **Phase 6b updated**: Now depends on 6c (worlds) and 6d (perception) for complete training env

See [APPROACH.qmd](../APPROACH.qmd) for complete architecture details.

## File Specifications

### TASK.md
Human and AI readable task description containing:
- **Overview**: Brief summary
- **Human Description**: Detailed explanation
- **AI Agent Instructions**: Step-by-step implementation guide
- **Success Criteria**: Checklist of completion requirements
- **Verification**: How to verify completion
- **Common Pitfalls**: Known issues and solutions
- **References**: Links to documentation
- **Dependencies**: Required prior phases
- **Next Phase**: What comes after

### inputs.json
Machine-readable specification of required inputs:

```json
{
  "phase": "phase-01-ontology-toolchain",
  "required_inputs": {
    "system_requirements": {
      "podman_installed": true,
      "disk_space_gb": 5
    }
  }
}
```

### outputs.json
Machine-readable specification of expected outputs:

```json
{
  "phase": "phase-01-ontology-toolchain",
  "required_outputs": {
    "containerfile": {
      "path": "ontology/Containerfile.planning",
      "type": "file",
      "verification": "file_exists"
    }
  }
}
```

### verification.sh
Automated verification script that:
- Checks all required outputs exist
- Validates file formats and content
- Tests functionality
- Reports pass/fail with exit codes (0 = success, 1 = failure)

### dependencies.json
Phase dependency specification:

```json
{
  "phase": "phase-04-vampire-runtime",
  "phase_number": 4,
  "depends_on": ["phase-01-ontology-toolchain", "phase-02-uav-ontology", "phase-03-evaluation"],
  "required_by": ["phase-05-perception-bridge"],
  "blocking": true
}
```

## AI Agent Workflow

AI agents can execute phases sequentially using the orchestration scripts:

### 1. Query Next Phase
```bash
bash scripts/next-phase.sh
# Output: "Recommended Next Phase: phase-04-execution-mode"
```

### 2. Read Task Definition
```bash
cat .phases/phase-04-execution-mode/TASK.md
cat .phases/phase-04-execution-mode/inputs.json
cat .phases/phase-04-execution-mode/outputs.json
```

### 3. Execute Implementation
AI agent follows instructions in TASK.md and creates required outputs.

### 4. Verify Completion
```bash
bash .phases/phase-04-execution-mode/verification.sh
# Exit code 0 = success, move to next phase
# Exit code 1 = failure, fix issues and retry
```

### 5. Move to Next Phase
```bash
bash scripts/next-phase.sh
# Dependency system ensures correct order
```

## Phase States

Each phase can be in one of three states:

- **Pending**: Dependencies not met, cannot start
- **Available**: Dependencies met, ready to start
- **Completed**: Verification passes

## Dependency Graph (Updated)

```
phase-01-ontology-toolchain [COMPLETE]
  ↓
phase-02-uav-ontology [COMPLETE]
  ↓
phase-03-evaluation [COMPLETE] ← Architectural decision point
  ↓
phase-04-vampire-runtime [COMPLETE] ← ARM64 + ROS 2 integration
  ↓
phase-05-perception-bridge [COMPLETE] ← F-11 ISR sensor grounding
  ↓
phase-06-mission-orchestration [COMPLETE] ← Simplified (no container swap)
  ↓
phase-06a-isr-sensor-integration ← ROS 2 sensor packages + Gazebo bridges
  ↓
  ├────────────────────────────────────────────────────────┐
  ↓                                                        ↓
phase-06c-diverse-training-worlds                   (provides camera topics)
  │   (urban, rural, industrial, coastal, randomized)      │
  ↓                                                        ↓
  └──────────────────┬─────────────────────────────────────┘
                     ↓
      phase-06d-perception-pipeline ← YOLO + TPTP facts (needs worlds + camera)
                     ↓
      phase-06b-simulation-training-env ← Gymnasium RL (needs perception + worlds)
                     ↓
  ┌─────────────────────────────────────┐
  ↓                   ↓                 ↓
phase-07-mission    phase-08-behavior  phase-09-trajectory
-planner-rl         -selector-rl       -optimizer-rl
  ↓                   ↓                 ↓
  └─────────────────────────────────────┘
                      ↓
         phase-10-aggm-integration
                      ↓
         phase-11-simulation-benchmark
                      ↓
         phase-12-project-drone-validation
                      ↓
         phase-13-flyby-f11-integration
                      ↓
         phase-14-documentation-release
```

**New Pre-RL Pipeline (6a → 6c → 6d → 6b)**:
- **6a**: ISR sensors provide camera topics
- **6c**: Diverse worlds provide objects to detect (buildings, vehicles, people)
- **6d**: Perception converts images to TPTP facts (what the ontology can reason about)
- **6b**: Training env ties everything together for RL

**Parallelization**: Phases 7, 8, 9 (RL agents) can be developed in parallel.

## Progress Tracking

Check overall progress:
```bash
bash scripts/verify-phase.sh
# Shows: "Completed: 3 / 14 (21%)"
```

View available phases:
```bash
bash scripts/next-phase.sh
# Shows completed, available, and blocked phases
```

## Verification Standards

All verification scripts must:
- Return exit code 0 on success, 1 on failure
- Check all required outputs from `outputs.json`
- Validate file existence and content
- Test functionality (not just file presence)
- Provide clear pass/fail messages

## References

- [Phase 3 Evaluation Report](../ontology/evaluation/EVALUATION_REPORT.qmd) - Reasoner selection
- [Orchestration Scripts](../scripts/) - Automation tools
- [APPROACH.qmd](../APPROACH.qmd) - Overall architecture
