# Flyby F-11 Development Scripts

This directory contains orchestration and automation scripts for the Flyby F-11 development workflow.

## Phase Management Scripts

The phase management scripts enable AI-agent-optimized sequential task execution:

### run-phase.sh
Execute a specific development phase with automated verification.

```bash
# Run a phase (interactive mode)
bash scripts/run-phase.sh phase-01-ontology-toolchain

# Verify a completed phase
bash scripts/run-phase.sh phase-01-ontology-toolchain --verify-only

# Execute without verification
bash scripts/run-phase.sh phase-02-uav-ontology --skip-verification
```

**Usage**:
- Checks phase dependencies
- Displays task overview
- Guides through implementation
- Runs automated verification

### verify-phase.sh
Check completion status of development phases.

```bash
# Verify all phases
bash scripts/verify-phase.sh

# Verify specific phase
bash scripts/verify-phase.sh phase-01-ontology-toolchain

# Verify with detailed output
bash scripts/verify-phase.sh --verbose
```

**Usage**:
- Validates phase outputs
- Checks for required artifacts
- Shows pass/fail status
- Provides progress summary

### next-phase.sh
Intelligent phase advisor that suggests what to work on next.

```bash
bash scripts/next-phase.sh
```

**Features**:
- Analyzes completed phases
- Checks dependencies
- Recommends next available phase
- Shows progress bar
- Displays time estimates

## AI Agent Workflow

These scripts are designed for both human and AI agent use:

1. **AI Agent Queries Next Phase**:
   ```bash
   bash scripts/next-phase.sh
   # Output: "Recommended Next Phase: phase-01-ontology-toolchain"
   ```

2. **AI Agent Reads Task Definition**:
   ```bash
   cat .phases/phase-01-ontology-toolchain/TASK.md
   cat .phases/phase-01-ontology-toolchain/inputs.json
   cat .phases/phase-01-ontology-toolchain/outputs.json
   ```

3. **AI Agent Executes Implementation**:
   - Create required files
   - Build containers
   - Run tests

4. **AI Agent Verifies Completion**:
   ```bash
   bash scripts/verify-phase.sh phase-01-ontology-toolchain
   # Exit code 0 = success, 1 = failure
   ```

5. **AI Agent Moves to Next Phase**:
   ```bash
   bash scripts/next-phase.sh
   # Recommends next phase based on dependencies
   ```

## Script Exit Codes

All scripts use standard exit codes:
- `0` - Success
- `1` - Failure or incomplete

This enables automated workflows:

```bash
# Example AI agent workflow
if bash scripts/next-phase.sh | grep -q "phase-01"; then
    # Execute phase 1
    bash scripts/run-phase.sh phase-01-ontology-toolchain

    # Verify
    if bash scripts/verify-phase.sh phase-01-ontology-toolchain; then
        echo "Phase 1 complete, moving to phase 2"
    fi
fi
```

## Human-Readable Output

While optimized for automation, all scripts provide rich human-readable output:
- Color-coded status (✓ green, ✗ red, ⊘ yellow)
- Progress bars
- Detailed error messages
- Next-step suggestions

## Directory Structure

```
scripts/
├── run-phase.sh          # Execute a phase
├── verify-phase.sh       # Verify phase completion
├── next-phase.sh         # Suggest next phase
└── README.md            # This file
```

## Adding New Scripts

When adding automation scripts:

1. **Make executable**: `chmod +x scripts/your-script.sh`
2. **Use standard exit codes**: 0 for success, 1 for failure
3. **Provide --help**: Usage information
4. **Add to this README**: Document purpose and usage

## Future Scripts

Planned automation scripts:

- `deploy-to-jetson.sh` - Automated Jetson deployment
- `run-tests.sh` - Full test suite execution
- `build-all-containers.sh` - Build all container images
- `check-dependencies.sh` - Validate system dependencies

## Integration with .phases/

The scripts in this directory work with the machine-readable phase definitions in [../.phases/](../.phases/):

- `TASK.md` - Human and AI readable instructions
- `inputs.json` - Required inputs (machine-readable)
- `outputs.json` - Expected outputs (machine-readable)
- `verification.sh` - Automated verification
- `dependencies.json` - Phase dependencies

This architecture enables:
- **Sequential task execution** by AI agents
- **Dependency tracking** and enforcement
- **Automated verification** of completion
- **Progress tracking** across development phases

## References

- [Phase Definitions](../.phases/) - Machine-readable task definitions
- [IMPLEMENTATION_ROADMAP.qmd](../IMPLEMENTATION_ROADMAP.qmd) - Overall development plan
