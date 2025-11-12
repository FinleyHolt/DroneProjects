# Phase 1 - Foundation

**Status**: In Progress
**Dependencies**: None (initial phase)

## Objective
Get PX4 SITL + Gazebo simulation working with automated setup scripts and clean version control that treats PX4 as an external dependency. This phase is purely about having a working drone simulator that we can iterate on without bloating the repository.

## Architectural Decisions
These decisions guide the repository structure and setup approach:
- **External Dependencies**: Treat PX4-Autopilot + Gazebo as external dependencies cloned into `.deps/` via scripts (not committed)
- **Configuration Versioning**: Keep all custom configs (params, mixers, launch files) in tracked folders with semantic versioning per drone variant
- **Command Standardization**: Use `Makefile` plus `setup/` scripts for reproducible setup and common operations
- **Workspace Separation**: ROS 2 packages in `ros2_ws/src`, LLM assets in `llm/` for clear ownership boundaries
- **Asset Management**: Git LFS for large Gazebo assets to avoid repo bloat while preserving version history
- **Logging Strategy**: SITL/log outputs stay outside repo by default, optional `logs/` for curated cases

## Development Assumptions
- Ubuntu 22.04 LTS (primary) or 24.04 LTS with ROS 2 Humble and PX4 SITL support
- **System Requirements**: Minimum 16GB RAM, 50GB disk space, NVIDIA GPU with 4GB+ VRAM (or CPU-only headless mode)
- NVIDIA/graphics drivers (≥525.x for CUDA 12) verified by setup scripts
- Network access available to fetch PX4, ROS dependencies, and model assets during setup
- Proxy-aware git/apt configuration for corporate/university networks
- LLM credentials managed via environment variables (never committed)
- Hardware-in-the-loop testing will reuse same config folders (no separate tree needed)

## Goals
1. PX4 SITL + Gazebo running with stock quadrotor
2. Automated setup scripts that work on fresh Ubuntu install
3. Clean `.gitignore` and dependency management (PX4 in `.deps/`)
4. Basic repository structure (no premature scaffolding)
5. Makefile for common operations

## Success Criteria
- [ ] `make setup` converges fresh Ubuntu to ready state
- [ ] `make sim` launches PX4 SITL + Gazebo with default drone
- [ ] Can manually fly drone using PX4 console commands
- [ ] `.deps/` directory properly ignored by git
- [ ] Documentation for troubleshooting and next steps

## Tasks

### 1.1 Repository Structure Setup

- [ ] Create `.gitignore` with entries for:
  - `.deps/` (external PX4-Autopilot repo)
  - Build artifacts (`build/`, `install/`, `log/`)
  - IDE files (`.vscode/`, `.idea/`)
  - Logs (`*.log`, `*.ulg`)
- [ ] Initialize minimal directory structure:
```
LLMDrone/
├─ .gitignore
├─ Makefile
├─ setup/
│  ├─ bootstrap.sh           # system dependencies
│  ├─ clone_px4.sh           # fetch PX4 into .deps/
│  └─ env.sh                 # environment variables
└─ .deps/                    # (git-ignored)
   └─ PX4-Autopilot/         # cloned by setup script
```
- [ ] Don't create folders we don't need yet (no premature structure)
- [ ] Focus only on what Phase 1 requires

**Deliverable**: Minimal working structure, not full scaffolding

### 1.2 PX4 Setup Automation

- [ ] Write `setup/check_system.sh`:
  - Verify Ubuntu version (22.04 or 24.04 LTS)
  - Check RAM ≥16GB, disk space ≥50GB available
  - Detect NVIDIA GPU and driver version (warn if missing, don't block)
  - Check for proxy configuration (http_proxy, https_proxy env vars)
  - Exit with clear error messages if requirements not met

- [ ] Write `setup/bootstrap.sh`:
  - Run `check_system.sh` first
  - Install build-essential, git, cmake
  - Install PX4 system dependencies per official docs
  - Create `.setup_state` directory to track installation progress
  - Make idempotent: skip steps if `.setup_state/bootstrap.done` exists
  - Keep it simple - just system packages

- [ ] Write `setup/clone_px4.sh`:
  - Check if `.deps/PX4-Autopilot` exists (idempotent)
  - Clone PX4-Autopilot into `.deps/PX4-Autopilot`
  - **Pin to v1.14.0** (stable release tested with this project)
  - Run `Tools/setup/ubuntu.sh --no-nuttx` for PX4 dependencies
  - Create `.setup_state/px4_cloned.done` on success
  - Document rollback: `rm -rf .deps .setup_state` for clean retry

- [ ] Write `setup/env.sh`:
  - Export `PX4_ROOT=${REPO_ROOT}/.deps/PX4-Autopilot`
  - Export `PX4_VERSION=v1.14.0` for tracking
  - Set minimal environment variables for SITL
  - Keep it lean - only what's needed for simulation

- [ ] Test on fresh Ubuntu 22.04 and 24.04 VMs
- [ ] Test re-running scripts (idempotency validation)
- [ ] Document forward compatibility strategy for future Ubuntu LTS releases

**Deliverable**: Idempotent scripts that go from fresh Ubuntu to working SITL with system validation

### 1.3 Build and Launch Verification

- [ ] Create `Makefile` with targets:
  - `setup`: Run bootstrap and clone_px4 scripts
  - `sim`: Build and launch PX4 SITL + Gazebo
  - `clean`: Clean PX4 build artifacts
  - `help`: Show available commands
- [ ] Verify successful SITL launch:
  - Gazebo window opens with default world
  - Quadrotor spawns and is visible
  - PX4 console shows "INFO [commander] Takeoff detected"
- [ ] Test manual flight commands in PX4 console with **quantitative validation**:
  - `commander takeoff` → reaches 2.5m altitude within 10s
  - `commander land` → descends and disarms within 15s
  - Position hold maintains ±0.5m XY accuracy for 30s
  - Record baseline flight metrics (settle time, position RMSE, battery drain rate)

**Deliverable**: `make setup && make sim` gives you a flying drone with documented baseline performance

### 1.4 Documentation

- [ ] Update root `README.md`:
  - Project overview (one paragraph)
  - Quick start: `make setup && make sim`
  - Link to Planning/ directory
- [ ] Document common issues in README:
  - GPU driver requirements
  - What to do if Gazebo doesn't open
  - Where logs are stored
- [ ] Keep it minimal - detailed docs come later

**Deliverable**: README with quickstart and troubleshooting

## Deliverables Checklist
- [ ] `.gitignore` properly excluding `.deps/`
- [ ] Setup scripts (`setup/bootstrap.sh`, `setup/clone_px4.sh`, `setup/env.sh`)
- [ ] Makefile with `setup`, `sim`, `clean`, `help` targets
- [ ] PX4 SITL + Gazebo launching successfully
- [ ] README with quickstart and troubleshooting
- [ ] Tested on fresh Ubuntu 22.04 installation

## Known Risks and Mitigation

### Risk: PX4 dependencies fail to install
**Impact**: Medium
**Mitigation**: Test on multiple Ubuntu versions, document version-specific workarounds

### Risk: Graphics/GPU drivers cause Gazebo issues
**Impact**: Medium
**Mitigation**: Document hardware requirements, provide headless mode option

### Risk: Setup scripts not idempotent (fail on re-run)
**Impact**: Medium (wastes hours debugging partial installs)
**Mitigation**: Use `.setup_state/` tracking files, check for existing installations, document cleanup procedure

### Risk: Corporate/university proxy blocks git submodule fetches
**Impact**: Medium
**Mitigation**: Detect proxy configuration, document manual workarounds, provide proxy-aware git config templates

### Risk: Ubuntu version drift (24.04+ LTS releases)
**Impact**: Low
**Mitigation**: Test on multiple LTS versions, document per-version workarounds, establish LTS update policy

## Phase Exit Criteria
Before moving to Phase 2, verify:
1. ✓ `make setup` works on fresh Ubuntu 22.04 and 24.04 installs
2. ✓ `make sim` launches Gazebo with flying quadrotor
3. ✓ Manual flight commands meet quantitative criteria (altitude ±0.5m, settle time <10s)
4. ✓ Setup scripts are idempotent (can re-run without errors)
5. ✓ `.deps/` is properly git-ignored
6. ✓ README explains how to get started
7. ✓ System requirements checker (`check_system.sh`) validates environment
8. ✓ PX4 version pinned to v1.14.0

## Next Phase
Once Phase 1 is complete, proceed to [Phase 2 - Environment & Mission Design](Phase-2-Environment-Mission-Design.md) to create custom drone configs, Gazebo worlds, and define example missions.

**Continuous Integration Setup**: Establish CI/CD pipeline in Phase 1 to prevent regressions in future phases. Add GitHub Actions or GitLab CI with:
- Automated `make setup` test on Ubuntu 22.04/24.04 containers
- SITL smoke test (launch, arm, takeoff, land)
- Baseline performance regression detection

## Notes and Findings
_Use this section to document implementation notes, decisions made during development, and lessons learned._

---

**Phase Started**: [Date]
**Phase Completed**: [Date]
**Key Decisions**:
- [Decision 1]
- [Decision 2]

**Blockers Encountered**:
- [Blocker 1 and resolution]

**Recommendations for Future Phases**:
- [Recommendation 1]
