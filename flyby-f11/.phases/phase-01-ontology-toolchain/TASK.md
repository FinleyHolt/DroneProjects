# Phase 1: Ontology Toolchain Setup

## Overview

Set up the formal reasoning infrastructure for mission planning and verification. This phase establishes the foundational tools for heavyweight logical reasoning using SUMO (Suggested Upper Merged Ontology) and Vampire theorem prover.

## Human Description

Install and configure the SUMO ontology framework and Vampire ATP (Automated Theorem Prover) inside a containerized environment. This creates the "planning mode" runtime where we can perform complex logical reasoning about mission constraints, safety properties, and semantic relationships.

The planning mode is heavyweight and only runs on-demand (not during flight). It generates verified mission plans that are then compiled into lightweight Prolog rules for execution mode.

## AI Agent Instructions

### Prerequisites
- Podman installed on development machine
- Basic understanding of container volumes and builds

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Create Planning Mode Container**
   - Location: `ontology/Containerfile.planning`
   - Base image: Ubuntu 22.04
   - Install SUMO dependencies (Java, SUMO-NL tools)
   - Install Vampire theorem prover
   - Configure SUMO knowledge base paths
   - Set up working directory structure

2. **Verify SUMO Installation**
   - Test SUMO can load base ontology files
   - Verify Vampire can execute basic inference
   - Check that SUMO-NL query interface works

3. **Create Test Ontology**
   - Create minimal UAV ontology file (`test_uav.kif`)
   - Define basic concepts: UAV, Mission, Constraint
   - Add simple axioms (e.g., "all UAVs have maximum altitude")

4. **Build and Test Container**
   - Build: `podman build -f ontology/Containerfile.planning -t flyby-f11-planning .`
   - Run: `podman run --rm -v ./ontology:/workspace:z flyby-f11-planning`
   - Execute verification script (see `verification.sh`)

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `ontology/Containerfile.planning` exists and builds successfully
- [ ] SUMO loads without errors inside container
- [ ] Vampire can prove a simple test theorem
- [ ] Container can access mounted ontology workspace
- [ ] Basic UAV concepts defined in KIF format

### Verification

Run automated verification:
```bash
bash .phases/phase-01-ontology-toolchain/verification.sh
```

### Time Estimate
2-4 hours (first-time setup, includes learning SUMO)

### Common Pitfalls

- **Java version issues**: SUMO requires Java 8+, verify with `java -version`
- **File permissions**: Use `:z` suffix on volume mounts for SELinux compatibility
- **SUMO paths**: Ensure `$SUMO_HOME` is set correctly in container
- **Vampire binary**: May need to compile from source or use pre-built binary

### References

- [SUMO Ontology](https://github.com/ontologyportal/sumo)
- [Vampire ATP](https://vprover.github.io/)
- [Podman Build Documentation](https://docs.podman.io/en/latest/markdown/podman-build.1.html)

### Dependencies
See `dependencies.json` - this is Phase 1, no prior dependencies.

### Next Phase
After completion, proceed to Phase 2: UAV Ontology Development
