# Ontology and Reasoning Tools Documentation

This directory contains critical documentation and source code for ontology and reasoning tools used in the flyby-f11 autonomous decision-making system.

## Quick Start

Run the download script to populate this directory with all required tools:

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
chmod +x download_ontology_tools.sh
./download_ontology_tools.sh
```

This will download all Phase 1 (Weeks 1-6) priority tools and generate a `manifest.txt` file summarizing the downloads.

## Phase 1 Priority Tools (Weeks 1-6)

### 1. SUMO Ontology (Suggested Upper Merged Ontology)
- **Directory**: `sumo/`
- **Repository**: https://github.com/ontologyportal/sumo
- **Purpose**: Upper ontology providing foundational concepts for autonomous reasoning
- **Key Files**:
  - `Merge.kif` - Core merged ontology
  - `Mid-level-ontology.kif` - Mid-level concepts
  - `Spatial.kif` - Spatial reasoning primitives
  - `Military.kif` - Military domain concepts (relevant for drone operations)
- **Format**: Knowledge Interchange Format (.kif)
- **Integration**: Use for mission-intent interpretation, domain modeling, and semantic grounding

### 2. SWI-Prolog
- **Directory**: `swi-prolog-docs/`
- **Website**: https://www.swi-prolog.org
- **Purpose**: Prolog environment for logic programming and reasoning
- **Downloaded Documentation**:
  - `manual.html` - Complete manual overview
  - `foreign.html` - Foreign Language Interface (C/FFI) guide
  - `embedded.html` - Embedding SWI-Prolog in C++ applications
  - `c-api.html` - C API reference for integration
- **Integration Path**: Embed SWI-Prolog in ROS 2 C++ nodes for runtime reasoning
- **Use Cases**:
  - Mission planning with logical constraints
  - Spatial reasoning queries
  - Communications-denied decision logic

### 3. Vampire Theorem Prover
- **Directory**: `vampire/`
- **Repository**: https://github.com/vprover/vampire
- **Purpose**: High-performance first-order logic theorem prover
- **Key Documentation**: `README.md`, source code examples
- **Use Cases**:
  - Verify mission plan consistency
  - Prove safety properties
  - Validate constraint satisfaction before execution
- **Integration**: Command-line tool or library integration for pre-flight verification

### 4. Clingo (Potassco Answer Set Programming)
- **Directory**: `clingo/`
- **Repository**: https://github.com/potassco/clingo
- **Purpose**: Answer Set Programming (ASP) solver for declarative problem solving
- **Key Documentation**: `README.md`, examples in `examples/`
- **Use Cases**:
  - Mission planning under constraints
  - Route optimization with hard/soft constraints
  - Declarative specification of flight behaviors
- **Integration**: Python/C++ API for plan generation

### 5. E-Prover
- **Directory**: `eprover/`
- **Repository**: https://github.com/eprover/eprover
- **Purpose**: Equational theorem prover for first-order logic
- **Key Documentation**: `README.md`, `DOC/` directory
- **Use Cases**:
  - Formal verification of autonomy logic
  - Mathematical proofs for trajectory safety
  - Constraint solving for spatial reasoning
- **Integration**: Command-line tool for offline verification

## Directory Structure

After running the download script:

```
ontology/
├── README.md                          # This file
├── download_ontology_tools.sh         # Download script
├── manifest.txt                       # Download summary (generated)
├── sumo/                              # SUMO ontology
│   ├── Merge.kif
│   ├── Mid-level-ontology.kif
│   ├── Spatial.kif
│   ├── Military.kif
│   └── ... (additional .kif files)
├── swi-prolog-docs/                   # SWI-Prolog documentation
│   ├── manual.html
│   ├── foreign.html
│   ├── embedded.html
│   └── c-api.html
├── vampire/                           # Vampire theorem prover
│   ├── README.md
│   ├── Kernel/
│   └── ... (source code)
├── clingo/                            # Clingo ASP solver
│   ├── README.md
│   ├── libclingo/
│   ├── examples/
│   └── ... (source code)
└── eprover/                           # E-Prover
    ├── README.md
    ├── DOC/
    └── ... (source code)
```

## Integration Roadmap

### Week 1-2: Ontology Foundation
- Review SUMO ontology structure
- Extract relevant concepts (spatial, temporal, agent, mission)
- Create custom domain ontology extending SUMO for drone operations

### Week 3-4: Logic Programming Setup
- Study SWI-Prolog FFI documentation
- Prototype C++ embedding of SWI-Prolog in ROS 2 node
- Develop Prolog rules for basic mission reasoning

### Week 5-6: Formal Verification
- Learn Vampire and E-Prover input formats
- Formalize safety constraints in first-order logic
- Implement verification checks for mission plans

### Future Phases
- ASP-based planning with Clingo
- Integration with behavior tree execution
- Real-time reasoning during flight

## Development Notes

### Shallow Clones
All Git repositories are cloned with `--depth 1` to minimize disk usage. Full history is not needed for documentation purposes.

### Documentation Format
SWI-Prolog documentation is downloaded as HTML for offline reference. For PDF generation, use:
```bash
# Install wkhtmltopdf if needed
sudo apt install wkhtmltopdf

# Convert to PDF
wkhtmltopdf swi-prolog-docs/manual.html swi-prolog-docs/manual.pdf
```

### Building Tools
Most tools require compilation. Refer to each tool's README.md for build instructions. Docker-based builds are recommended for consistency.

### ROS 2 Integration
Tools will be integrated as:
1. **Offline**: Pre-flight verification (Vampire, E-Prover)
2. **Online**: Runtime reasoning (SWI-Prolog embedded in nodes)
3. **Planning**: Mission generation (Clingo for ASP-based planning)

## References

- **SUMO**: Niles, I., & Pease, A. (2001). "Towards a standard upper ontology"
- **SWI-Prolog**: Wielemaker, J., et al. (2012). "SWI-Prolog"
- **Vampire**: Kovács, L., & Voronkov, A. (2013). "First-order theorem proving and Vampire"
- **Clingo**: Gebser, M., et al. (2014). "Clingo = ASP + Control"
- **E-Prover**: Schulz, S. (2013). "System description: E 1.8"

## Contact

For questions about tool selection or integration strategy, refer to:
- `/home/finley/Github/DroneProjects/flyby-f11/APPROACH.qmd`
- `/home/finley/Github/DroneProjects/flyby-f11/ONTOLOGY_FOUNDATION.qmd`

Developer: Finley Holt
Project: flyby-f11 Autonomous Decision-Making System
