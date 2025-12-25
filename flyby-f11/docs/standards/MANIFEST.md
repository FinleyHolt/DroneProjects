# Standards Documentation Manifest

## Overview

This directory contains reference documentation for technical standards used in the flyby-f11 project.

**Location**: `/home/finley/Github/DroneProjects/flyby-f11/docs/standards/`

**Purpose**: Provide offline reference material for autonomous robotics standards and formal specification languages.

## Contents

### 1. IEEE 1872.2 Autonomous Robotics (AuR) Ontology

**File**: `IEEE_1872-2_REFERENCE.md`

**Description**: Comprehensive reference for IEEE 1872.2-2021 Standard for Autonomous Robotics Ontology, including:
- Core ontology concepts and structure
- OWL representation and reasoning capabilities
- Application to drone autonomy (sUAS)
- Integration with ROS 2 systems
- Python libraries (owlready2, rdflib, pyshacl)
- Links to ODP implementation repository

**Key Topics**:
- Autonomous system modeling
- Capability and task ontologies
- Environment representation
- Semantic reasoning for autonomous decision-making

**External Resources**:
- GitHub Repository: https://github.com/hsu-aut/IndustrialStandard-ODP-IEEE1872-2
  - ODP (Ontology Design Patterns) implementation
  - Modular ontology components
  - Example use cases and SPARQL queries
- IEEE Standards Purchase: https://ieeexplore.ieee.org/
- Related Standard: IEEE 1872-2015 (CORA - Core Ontology for Robotics and Automation)

**Use in flyby-f11**:
- Mission-intent interpretation
- Capability-task matching
- Semantic knowledge representation
- Runtime autonomous reasoning

---

### 2. TPTP (Thousands of Problems for Theorem Provers) Format

**File**: `TPTP_FORMAT_REFERENCE.md`

**Description**: Complete reference for TPTP language specifications, focusing on formal logic representations:
- FOF (First-Order Form) syntax and examples
- TFF (Typed First-order Form) with type system
- CNF (Clause Normal Form) for resolution provers
- THF (Typed Higher-order Form) for advanced reasoning

**Key Topics**:
- First-order logic syntax
- Type declarations and type systems
- Integration with automated theorem provers (Vampire, E Prover, Z3)
- Application to robotic mission verification
- Conversion from OWL ontologies to TPTP

**External Resources**:
- TPTP Website: http://www.tptp.org/
- TPTP Syntax BNF: http://www.tptp.org/TPTP/SyntaxBNF.html
- TPTP Problem Library: http://www.tptp.org/cgi-bin/SeeTPTP
- SystemOnTPTP: Online ATP interface

**Theorem Provers**:
- Vampire: High-performance first-order ATP
- E Prover: Equational theorem prover
- Z3: SMT solver with TPTP support
- Leo-III, Satallax: Higher-order ATPs

**Python Libraries**:
- tptp-lark-parser: Parse TPTP files
- pyres: Resolution theorem prover
- z3-solver: Z3 SMT solver bindings

**Use in flyby-f11**:
- Formal verification of mission safety constraints
- Flight rules and regulation modeling
- Behavior tree plan verification
- Runtime safety state checking
- Integration with ontology-based reasoning

---

## Related Documentation

### In Other Directories

- **Tools**: `/home/finley/Github/DroneProjects/flyby-f11/docs/tools/`
  - JetPack and Docker references
  - Quarto documentation rendering

- **Ontology**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/`
  - Project-specific ontology designs
  - Domain models for drone operations

- **ROS 2**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/`
  - ROS 2 integration patterns
  - Message and service definitions

## Download Instructions

### Cloning Referenced Repositories

Due to tool restrictions, repositories were not cloned. To manually download:

```bash
# Navigate to standards directory
cd /home/finley/Github/DroneProjects/flyby-f11/docs/standards/

# Clone IEEE 1872.2 ODP implementation
git clone --depth 1 https://github.com/hsu-aut/IndustrialStandard-ODP-IEEE1872-2.git

# Clone TPTP problem library (optional, very large)
# git clone https://github.com/TPTPWorld/TPTP-v8.0.0.git
```

### Downloading TPTP Resources

```bash
# Download specific TPTP problems
wget http://www.tptp.org/cgi-bin/SeeTPTP?Category=Problems&Domain=<domain>

# Or browse online at http://www.tptp.org/
```

## Usage Guidelines

### For Research and Development

1. **Ontology Design**: Reference IEEE 1872.2 for standard vocabulary and concepts
2. **Formal Verification**: Use TPTP format for encoding verification problems
3. **Reasoning**: Integrate theorem provers for mission safety validation
4. **Semantic Integration**: Connect ontologies with formal logic for complete reasoning

### For Implementation

1. **Python Integration**: Use owlready2 for ontology, z3-solver for TPTP reasoning
2. **ROS 2 Nodes**: Create semantic reasoning services using these standards
3. **Mission Planning**: Apply ontology-based task decomposition
4. **Safety Checking**: Use TPTP for formal constraint verification

## Maintenance

### Updating References

```bash
# Pull latest changes from repositories
cd IndustrialStandard-ODP-IEEE1872-2
git pull

# Check for updated IEEE standards (requires IEEE access)
# Visit https://ieeexplore.ieee.org/

# Check TPTP website for new syntax or features
# Visit http://www.tptp.org/
```

### Version Information

- **IEEE 1872.2**: 2021 edition (current as of documentation creation)
- **TPTP**: TPTP v8.0.0+ syntax
- **Documentation Created**: 2025-12-25

## Contact and Support

For questions about applying these standards to the flyby-f11 project:
- Project Lead: Finley Holt
- Repository: `/home/finley/Github/DroneProjects/flyby-f11/`

For questions about the standards themselves:
- IEEE Standards: https://standards.ieee.org/support/
- TPTP: Contact via http://www.tptp.org/

## License and Attribution

- **IEEE Standards**: Copyright IEEE, purchase required for full text
- **TPTP**: Free and open access at http://www.tptp.org/
- **Reference Documentation**: Created for flyby-f11 project, educational use

---

**Last Updated**: 2025-12-25
**Maintainer**: Finley Holt
