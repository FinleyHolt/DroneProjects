# Flyby F-11 Development Phases

This directory contains machine-readable task definitions for the Flyby F-11 development workflow. The structure is optimized for **AI agent task execution**, enabling automated sequential implementation with verification.

## Directory Structure

Each phase is a self-contained directory with standardized files:

```
.phases/
├── phase-01-ontology-toolchain/
│   ├── TASK.md              # Human and AI readable task description
│   ├── inputs.json          # Required inputs (machine-readable)
│   ├── outputs.json         # Expected outputs (machine-readable)
│   ├── verification.sh      # Automated verification script
│   └── dependencies.json    # Phase dependencies
├── phase-02-uav-ontology/
│   └── ...
├── phase-03-translation/
│   └── ...
└── README.md               # This file
```

## Phase Organization

Phases are numbered sequentially and named descriptively:

| Phase | Name | Description | Est. Time |
|-------|------|-------------|-----------|
| 01 | ontology-toolchain | SUMO and Vampire setup | 2-4h |
| 02 | uav-ontology | UAV domain knowledge base | 8-12h |
| 03 | translation | SUMO to Prolog translator | 10-15h |
| 04 | execution-mode | Prolog runtime | 6-8h |
| 05 | mission-planner | Mission planning module | 10-12h |
| ... | ... | ... | ... |

See [IMPLEMENTATION_ROADMAP.qmd](../IMPLEMENTATION_ROADMAP.qmd) for the complete list of 14 phases.

## File Specifications

### TASK.md
Human and AI readable task description containing:
- **Overview**: Brief summary
- **Human Description**: Detailed explanation
- **AI Agent Instructions**: Step-by-step implementation guide
- **Success Criteria**: Checklist of completion requirements
- **Verification**: How to verify completion
- **Time Estimate**: Expected duration
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
  },
  "success_indicators": {
    "container_builds": "podman build exit code 0"
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
  "phase": "phase-03-translation",
  "phase_number": 3,
  "depends_on": ["phase-01-ontology-toolchain", "phase-02-uav-ontology"],
  "required_by": ["phase-04-execution-mode"],
  "blocking": true
}
```

## AI Agent Workflow

AI agents can execute phases sequentially using the orchestration scripts:

### 1. Query Next Phase
```bash
bash scripts/next-phase.sh
# Output: "Recommended Next Phase: phase-01-ontology-toolchain"
```

### 2. Read Task Definition
```bash
cat .phases/phase-01-ontology-toolchain/TASK.md
cat .phases/phase-01-ontology-toolchain/inputs.json
cat .phases/phase-01-ontology-toolchain/outputs.json
```

### 3. Execute Implementation
AI agent follows instructions in TASK.md and creates required outputs.

### 4. Verify Completion
```bash
bash .phases/phase-01-ontology-toolchain/verification.sh
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

Check status:
```bash
bash scripts/verify-phase.sh
```

## Dependency Graph

Phases have a directed acyclic graph (DAG) of dependencies:

```
phase-01-ontology-toolchain
  ↓
phase-02-uav-ontology
  ↓
phase-03-translation
  ↓
phase-04-execution-mode
  ↓
...
```

The orchestration scripts enforce this dependency order automatically.

## Creating New Phases

To add a new phase:

1. Create directory: `mkdir .phases/phase-XX-name`
2. Create `TASK.md` with complete instructions
3. Create `inputs.json` with required inputs
4. Create `outputs.json` with expected outputs
5. Create `verification.sh` with automated checks
6. Create `dependencies.json` with phase dependencies
7. Update `IMPLEMENTATION_ROADMAP.qmd`

## Human vs AI Usage

### Human Developers
- Read `TASK.md` for detailed instructions
- Manually implement following the guide
- Run `verification.sh` to check completion
- Use `scripts/next-phase.sh` to find what's next

### AI Agents
- Parse JSON files for structured inputs/outputs
- Execute steps from `TASK.md` AI Agent Instructions
- Run `verification.sh` for automated validation
- Use exit codes to determine success/failure
- Query `dependencies.json` for sequencing

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

- [Orchestration Scripts](../scripts/) - Automation tools
- [IMPLEMENTATION_ROADMAP.qmd](../IMPLEMENTATION_ROADMAP.qmd) - Complete phase list
- [APPROACH.qmd](../APPROACH.qmd) - Overall architecture
