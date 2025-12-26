# AI Agent Development Guide

This document explains how AI agents can autonomously develop the Flyby F-11 project using the machine-readable task definitions and orchestration scripts.

## Overview

The Flyby F-11 project is structured for **sequential AI agent task execution**. Each development phase has:

- Machine-readable task definitions (JSON)
- Automated verification scripts
- Dependency tracking
- Human-readable instructions

This enables AI agents to:
1. Query what needs to be done next
2. Read detailed task specifications
3. Execute implementation steps
4. Verify completion automatically
5. Move to the next phase

## Quick Start for AI Agents

### 1. Query Next Available Phase

```bash
bash scripts/next-phase.sh
```

Output format:
```
Recommended Next Phase: phase-01-ontology-toolchain

Overview:
Set up the formal reasoning infrastructure for mission planning...

Time Estimate:
2-4 hours (first-time setup, includes learning SUMO)

To start this phase:
  bash scripts/run-phase.sh phase-01-ontology-toolchain
```

### 2. Read Task Specification

Machine-readable inputs:
```bash
cat .phases/phase-01-ontology-toolchain/inputs.json
cat .phases/phase-01-ontology-toolchain/outputs.json
cat .phases/phase-01-ontology-toolchain/dependencies.json
```

Human-readable instructions:
```bash
cat .phases/phase-01-ontology-toolchain/TASK.md
```

### 3. Execute Implementation

Follow the "AI Agent Instructions" section in TASK.md:
- Create required files
- Build containers
- Run tests
- Generate outputs

### 4. Verify Completion

```bash
bash .phases/phase-01-ontology-toolchain/verification.sh
```

Exit codes:
- `0` = Success, phase complete
- `1` = Failure, fix issues and retry

### 5. Move to Next Phase

```bash
bash scripts/next-phase.sh
```

Dependency system ensures correct execution order.

## Project Structure

```
flyby-f11/
├── .phases/                    # Machine-readable phase definitions
│   ├── phase-01-ontology-toolchain/
│   │   ├── TASK.md            # Human & AI instructions
│   │   ├── inputs.json        # Required inputs
│   │   ├── outputs.json       # Expected outputs
│   │   ├── verification.sh    # Automated checks
│   │   └── dependencies.json  # Phase dependencies
│   ├── phase-02-uav-ontology/
│   └── ...
├── scripts/                    # Orchestration automation
│   ├── run-phase.sh           # Execute a phase
│   ├── verify-phase.sh        # Check completion status
│   └── next-phase.sh          # Suggest next phase
├── containers/                 # Container infrastructure
│   ├── compose/               # Development (Podman Compose)
│   └── quadlet/               # Production (systemd)
├── ontology/                   # Reasoning system
│   ├── planning_mode/         # SUMO (heavyweight)
│   ├── execution_mode/        # Prolog (lightweight)
│   └── translation/           # SUMO → Prolog
├── ros2_ws/                    # ROS 2 autonomy
├── simulation/                 # SITL and Gazebo
└── models/                     # Vision models
```

## Phase Organization

Development is organized into 14 sequential phases:

| Phase | Name | Dependencies | Time | Description |
|-------|------|--------------|------|-------------|
| 01 | ontology-toolchain | None | 2-4h | SUMO + Vampire setup for planning mode |
| 02 | uav-ontology | 01 | 8-12h | UAV domain knowledge base in KIF |
| 03 | translation | 01, 02 | 10-15h | SUMO to Prolog translator |
| 04 | execution-mode | 01-03 | 6-8h | SWI-Prolog runtime for real-time inference |
| 05 | perception-bridge | 04 | 10-14h | Vision-to-symbolic grounding nodes |
| 06 | phase-transition | 04, 05 | 8-10h | Planning/execution mode orchestration |
| 07 | mission-planner-rl | 04-06 | 12-16h | Level 1 SAC agent (10s horizon) |
| 08 | behavior-selector-rl | 06 | 10-14h | Level 2 PPO agent (1s horizon) |
| 09 | trajectory-optimizer-rl | 06 | 10-14h | Level 3 TD3 agent (100ms horizon) |
| 10 | aggm-integration | 05, 07-09 | 12-16h | Automatic Goal Generation Model |
| 11 | simulation-benchmark | 06, 10 | 15-20h | Benchmark suite and evaluation |
| 12 | project-drone-validation | 11 | 20-30h | Hardware testing on project-drone |
| 13 | flyby-f11-integration | 12 | 25-35h | Deployment to Flyby F-11 platform |
| 14 | documentation-release | 11-13 | 15-25h | Paper, docs, open-source release |

**Total estimated time**: 150-220 hours

**Parallelization opportunities**:
- Phases 7, 8, 9 can run in parallel (all RL agents)

See [APPROACH.qmd](APPROACH.qmd) for architecture details.

## JSON Specifications

### inputs.json

Specifies required inputs for a phase:

```json
{
  "phase": "phase-01-ontology-toolchain",
  "required_inputs": {
    "system_requirements": {
      "podman_installed": true,
      "disk_space_gb": 5
    }
  },
  "optional_inputs": {
    "sumo_version": {"default": "latest"}
  },
  "environment_variables": {
    "SUMO_HOME": "/opt/sumo"
  }
}
```

### outputs.json

Specifies expected outputs from a phase:

```json
{
  "phase": "phase-01-ontology-toolchain",
  "required_outputs": {
    "containerfile": {
      "path": "ontology/Containerfile.planning",
      "type": "file",
      "verification": "file_exists"
    },
    "container_image": {
      "name": "flyby-f11-planning:latest",
      "type": "podman_image",
      "verification": "podman images | grep flyby-f11-planning"
    }
  },
  "success_indicators": {
    "container_builds": "podman build exit code 0",
    "sumo_loads": "SUMO initialization without errors"
  }
}
```

### dependencies.json

Specifies phase dependencies:

```json
{
  "phase": "phase-03-translation",
  "phase_number": 3,
  "depends_on": [
    "phase-01-ontology-toolchain",
    "phase-02-uav-ontology"
  ],
  "required_by": [
    "phase-04-execution-mode"
  ],
  "blocking": true
}
```

## Verification Scripts

Every phase has a `verification.sh` script that:

1. Checks all required outputs exist
2. Validates file formats and content
3. Tests functionality
4. Returns exit code (0 = success, 1 = failure)

Example verification:

```bash
#!/bin/bash
set -e

# Check file exists
test -f ontology/Containerfile.planning

# Check container builds
podman build -f ontology/Containerfile.planning -t flyby-f11-planning .

# Check SUMO loads
podman run --rm flyby-f11-planning:latest bash -c 'test -d $SUMO_HOME'

# All checks passed
exit 0
```

## Orchestration Scripts

### scripts/run-phase.sh

Execute a phase interactively or automated:

```bash
# Interactive mode (displays task, asks for verification)
bash scripts/run-phase.sh phase-01-ontology-toolchain

# Verify only (for checking completion)
bash scripts/run-phase.sh phase-01-ontology-toolchain --verify-only

# Execute without verification
bash scripts/run-phase.sh phase-01-ontology-toolchain --skip-verification
```

### scripts/verify-phase.sh

Check completion status:

```bash
# Verify all phases (summary)
bash scripts/verify-phase.sh

# Verify specific phase
bash scripts/verify-phase.sh phase-01-ontology-toolchain

# Verbose output (detailed checks)
bash scripts/verify-phase.sh phase-01-ontology-toolchain --verbose
```

### scripts/next-phase.sh

Intelligent phase advisor:

```bash
bash scripts/next-phase.sh
```

Features:
- Shows completed phases
- Lists available phases (dependencies met)
- Shows blocked phases (dependencies not met)
- Recommends next phase to work on
- Displays progress bar

## Automated Workflow Example

AI agent autonomous execution:

```python
import subprocess
import json

def get_next_phase():
    """Query next available phase"""
    result = subprocess.run(
        ["bash", "scripts/next-phase.sh"],
        capture_output=True,
        text=True
    )
    # Parse output to extract phase name
    return extract_phase_name(result.stdout)

def read_task_spec(phase):
    """Read machine-readable task specification"""
    with open(f".phases/{phase}/inputs.json") as f:
        inputs = json.load(f)
    with open(f".phases/{phase}/outputs.json") as f:
        outputs = json.load(f)
    with open(f".phases/{phase}/TASK.md") as f:
        instructions = f.read()

    return inputs, outputs, instructions

def execute_phase(phase, instructions):
    """Execute implementation steps"""
    # Parse instructions
    # Create files
    # Build containers
    # Run tests
    pass

def verify_phase(phase):
    """Verify phase completion"""
    result = subprocess.run(
        ["bash", f".phases/{phase}/verification.sh"],
        capture_output=True
    )
    return result.returncode == 0

# Main workflow
while True:
    phase = get_next_phase()
    if phase is None:
        print("All phases complete!")
        break

    inputs, outputs, instructions = read_task_spec(phase)
    execute_phase(phase, instructions)

    if verify_phase(phase):
        print(f"Phase {phase} complete!")
    else:
        print(f"Phase {phase} verification failed, retrying...")
        # Implement retry logic
```

## Phase States

Each phase can be in one of three states:

1. **Pending**: Dependencies not met, cannot start
2. **Available**: Dependencies met, ready to execute
3. **Completed**: Verification passes

Use `scripts/verify-phase.sh` to check current state of all phases.

## Dependency Graph

The dependency graph ensures correct execution order:

```
phase-01-ontology-toolchain
  ↓
phase-02-uav-ontology
  ↓
phase-03-translation
  ↓
phase-04-execution-mode
  ↓
phase-05-perception-bridge
  ↓
phase-06-phase-transition
  ↓
  ├─────────────────────────────────────┐
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

**Notes**:
- Phases 7, 8, 9 (RL agents) can be developed in parallel
- Phase 10 (AGGM) integrates all three agents
- Phases 11-14 are strictly sequential (simulation → hardware → production → release)

Orchestration scripts automatically enforce this order.

## Common Patterns

### Pattern 1: Sequential Execution

Execute phases in dependency order:

```bash
for phase in $(ls -1 .phases/ | sort); do
    if bash scripts/next-phase.sh | grep -q "$phase"; then
        bash scripts/run-phase.sh "$phase" --skip-verification
        bash scripts/verify-phase.sh "$phase"
    fi
done
```

### Pattern 2: Resume After Failure

Resume from where you left off:

```bash
# Find first incomplete phase
bash scripts/next-phase.sh

# Execute and verify
bash scripts/run-phase.sh <phase-name>
bash scripts/verify-phase.sh <phase-name>
```

### Pattern 3: Parallel Execution (if independent)

Some phases may be independent and can run in parallel:

```bash
# Check dependencies.json for "can_run_parallel": true
cat .phases/phase-XX/dependencies.json

# Execute multiple phases concurrently (if safe)
bash scripts/run-phase.sh phase-XX &
bash scripts/run-phase.sh phase-YY &
wait
```

## Integration with Containers

Many phases involve building and testing containers:

```bash
# Build container
podman build -f ontology/Containerfile.planning -t flyby-f11-planning .

# Test container
podman run --rm -it \
  -v ./ontology:/workspace:z \
  flyby-f11-planning:latest

# Verify with compose
podman-compose -f containers/compose/podman-compose.yml up planning
```

## Progress Tracking

Track overall progress:

```bash
bash scripts/verify-phase.sh
```

Output:
```
Total phases:      14
Passed:            3
Failed:            0

Progress: [=======             ] 21%
```

## Error Handling

If verification fails:

1. Check verification output (verbose mode):
   ```bash
   bash scripts/verify-phase.sh <phase> --verbose
   ```

2. Review TASK.md "Common Pitfalls" section

3. Check logs in phase-specific log directories

4. Fix issues and re-run verification

## Human Collaboration

While designed for AI agents, humans can:

- Review progress: `bash scripts/verify-phase.sh`
- View next steps: `bash scripts/next-phase.sh`
- Read detailed docs: `less .phases/<phase>/TASK.md`
- Override decisions: Manually execute specific phases

## References

- [.phases/README.md](.phases/README.md) - Phase definition format
- [scripts/README.md](scripts/README.md) - Orchestration scripts
- [IMPLEMENTATION_ROADMAP.qmd](IMPLEMENTATION_ROADMAP.qmd) - Complete phase list
- [APPROACH.qmd](APPROACH.qmd) - Overall architecture
- [CONTAINERIZATION_STRATEGY.md](CONTAINERIZATION_STRATEGY.md) - Container infrastructure

## Next Steps

To start development:

```bash
# 1. Check what needs to be done
bash scripts/next-phase.sh

# 2. Start first phase
bash scripts/run-phase.sh phase-01-ontology-toolchain

# 3. Verify completion
bash scripts/verify-phase.sh phase-01-ontology-toolchain

# 4. Continue to next phase
bash scripts/next-phase.sh
```

The system will guide you through all 14 phases sequentially.
