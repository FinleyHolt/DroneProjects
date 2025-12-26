# Phase 13: Flyby-F11 Integration

## Overview

Deploy and validate the system on the Flyby F-11 platform (Jetson Orin NX 16GB) for MCTSSA collaboration. This phase adapts the validated system for the production platform and conducts mission-specific testing.

## Human Description

The Flyby F-11 is the production deployment platform:

- **Hardware**: Jetson Orin NX 16GB (50 TOPS), Flyby sensor suite
- **Mission Context**: GPS-denied, communications-limited (MCTSSA)
- **Compliance**: NDAA-compliant, defense-ready

This phase covers:
1. Platform adaptation from project-drone
2. Mission-specific configuration
3. Outdoor flight testing
4. Communications-denied scenarios
5. MCTSSA demonstration preparation

## AI Agent Instructions

### Prerequisites
- Phase 12 completed (project-drone validation)
- Flyby F-11 hardware access (via MCTSSA)
- Outdoor testing authorization
- Understanding of MCTSSA requirements
- NDAA compliance knowledge

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Platform Adaptation**
   - Adapt containers for Orin NX (16GB)
   - Configure Flyby-specific sensor suite
   - Adjust for 50 TOPS vs 67 TOPS
   - Leverage additional memory for larger models

2. **Mission Configuration**
   - MCTSSA mission profiles
   - Communications-denied parameters
   - Extended autonomy settings
   - Regulatory compliance checks

3. **Sensor Integration**
   - Flyby sensor suite drivers
   - Sensor fusion calibration
   - Redundancy configuration
   - Failure mode handling

4. **Outdoor Flight Testing**
   - Survey/inspection missions
   - Extended range testing
   - Dynamic obstacle scenarios
   - Multi-objective missions

5. **Communications-Denied Testing**
   - GPS-denied navigation validation
   - No-comms autonomous operation
   - Mission completion without link
   - Failsafe behavior validation

6. **MCTSSA Demonstration**
   - Demonstration package preparation
   - Mission-intent interpretation showcase
   - Explainability demonstration
   - Safety documentation

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] System runs on Flyby F-11 hardware
- [ ] Outdoor missions completed successfully
- [ ] Communications-denied operation validated
- [ ] MCTSSA demonstration ready
- [ ] All NDAA compliance verified
- [ ] Zero safety violations
- [ ] Mission-intent interpretation demonstrated

### Verification

Run automated verification:
```bash
bash .phases/phase-13-flyby-f11-integration/verification.sh
```

### Time Estimate
25-35 hours (includes hardware adaptation and outdoor testing sessions)

### Common Pitfalls

- **Hardware differences**: Orin NX vs Orin Nano Super
- **Sensor suite**: Different sensors than project-drone
- **Outdoor conditions**: Wind, lighting, GPS multipath
- **Regulatory requirements**: Airspace authorization
- **MCTSSA requirements**: Security and compliance
- **Documentation**: Defense-grade documentation needed

### References

- [Flyby F-11 Developer Documentation](https://flybyrobotics.com/)
- [MCTSSA Collaboration Guidelines](internal)
- [NDAA Compliance Requirements](https://www.congress.gov/bill/116th-congress/house-bill/133)
- [FAA Part 107](https://www.faa.gov/uas/commercial_operators/part_107)

### Dependencies
See `dependencies.json` - requires project-drone validation.

### Next Phase
After completion, proceed to Phase 14: Documentation and Release
