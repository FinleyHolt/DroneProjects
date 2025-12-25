# Ontology Tools Download Status

**Last Updated**: December 25, 2025
**Project**: flyby-f11 Autonomous Decision-Making System
**Phase**: 1 (Weeks 1-6) - Foundation

---

## Summary

Due to system security restrictions, automatic downloads could not be completed during this session. However, **all necessary download scripts and documentation have been prepared** for manual execution.

---

## Files Created

All documentation and download infrastructure has been created in:
`/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/`

### Documentation Files

| File | Purpose | Status |
|------|---------|--------|
| `README.md` | Overview of all tools and integration strategy | ✅ Created |
| `TOOL_COMPARISON.md` | Detailed comparison of reasoning tools | ✅ Created |
| `QUICK_START.md` | Hands-on tutorials for each tool | ✅ Created |
| `DOWNLOAD_INSTRUCTIONS.md` | Step-by-step download guide | ✅ Created |
| `download_ontology_tools.sh` | Automated download script | ✅ Created |
| `STATUS.md` | This file - current status | ✅ Created |

### Download Script

**Location**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/download_ontology_tools.sh`

**What it does**:
- Clones all 4 GitHub repositories (SUMO, Vampire, Clingo, E-Prover) with shallow clones
- Downloads SWI-Prolog documentation HTML files
- Generates `manifest.txt` with download summary
- Provides verification output

**To execute**:
```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
chmod +x download_ontology_tools.sh
./download_ontology_tools.sh
```

---

## Tools to Download

### Priority 1: Essential (Required for Phase 1)

#### 1. SUMO Ontology
- **Repository**: https://github.com/ontologyportal/sumo
- **Download Method**: Git clone (shallow)
- **Size**: ~50 MB
- **Status**: ⏳ Pending download
- **Purpose**: Foundational ontology for semantic reasoning
- **Key Files**: `Merge.kif`, `Spatial.kif`, `Mid-level-ontology.kif`

#### 2. SWI-Prolog Documentation
- **Source**: https://www.swi-prolog.org/pldoc/
- **Download Method**: wget HTML files
- **Size**: ~2 MB
- **Status**: ⏳ Pending download
- **Purpose**: Embedding Prolog in ROS 2 C++ nodes
- **Key Docs**: Manual, FFI guide, embedding guide, C API reference

### Priority 2: Planning Tools (Weeks 7-12)

#### 3. Clingo (Potassco ASP)
- **Repository**: https://github.com/potassco/clingo
- **Download Method**: Git clone (shallow)
- **Size**: ~30 MB
- **Status**: ⏳ Pending download
- **Purpose**: Mission planning with Answer Set Programming
- **Alternative**: Install via pip (`pip3 install clingo`)

### Priority 3: Verification Tools (Weeks 13-18)

#### 4. Vampire Theorem Prover
- **Repository**: https://github.com/vprover/vampire
- **Download Method**: Git clone (shallow)
- **Size**: ~40 MB
- **Status**: ⏳ Pending download
- **Purpose**: Pre-flight formal verification

#### 5. E-Prover
- **Repository**: https://github.com/eprover/eprover
- **Download Method**: Git clone (shallow)
- **Size**: ~30 MB
- **Status**: ⏳ Pending download
- **Purpose**: Backup theorem prover (equational reasoning)

---

## Total Download Requirements

- **Disk Space**: ~150 MB (with shallow clones)
- **Network Bandwidth**: ~150 MB download
- **Time Estimate**: 2-5 minutes (depending on connection)

---

## Next Steps (Action Required)

### Step 1: Execute Download Script

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
chmod +x download_ontology_tools.sh
./download_ontology_tools.sh
```

This will populate the directory with all 5 tools.

### Step 2: Verify Downloads

After the script completes, verify all tools were downloaded:

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology

# Check for SUMO
ls sumo/Merge.kif

# Check for SWI-Prolog docs
ls swi-prolog-docs/*.html

# Check for Clingo
ls clingo/README.md

# Check for Vampire
ls vampire/README.md

# Check for E-Prover
ls eprover/README.md

# View manifest
cat manifest.txt
```

### Step 3: Review Documentation

1. Read `README.md` for project overview
2. Read `TOOL_COMPARISON.md` to understand each tool's role
3. Follow `QUICK_START.md` for hands-on tutorials

### Step 4: Build Tools (Optional)

Build Vampire and E-Prover for local testing:

```bash
# Build Vampire
cd vampire && mkdir build && cd build
cmake .. && make -j$(nproc)

# Build E-Prover
cd ../../eprover
./configure && make -j$(nproc)
```

### Step 5: Install Runtime Tools

```bash
# Install SWI-Prolog (for ROS 2 embedding)
sudo apt-add-repository ppa:swi-prolog/stable
sudo apt update
sudo apt install swi-prolog swi-prolog-dev

# Install Clingo (Python API)
pip3 install clingo

# Verify installations
swipl --version
clingo --version
```

---

## Development Timeline

### Weeks 1-2 (Current)
- [x] Prepare download infrastructure
- [x] Create comprehensive documentation
- [ ] **ACTION REQUIRED**: Execute download script
- [ ] Review SUMO ontology structure
- [ ] Extract drone-relevant concepts

### Weeks 3-4
- [ ] Prototype SWI-Prolog embedding in ROS 2
- [ ] Convert SUMO subset to Prolog facts
- [ ] Implement basic reasoning queries

### Weeks 5-6
- [ ] Test reasoning in simulation
- [ ] Integrate with behavior tree framework
- [ ] Performance tuning and optimization

### Weeks 7-12 (Phase 2)
- [ ] Clingo ASP mission planning
- [ ] Plan generation and optimization

### Weeks 13-18 (Phase 3)
- [ ] Vampire/E-Prover formal verification
- [ ] Safety property proofs

---

## Documentation Quality Checklist

All prepared documentation includes:
- ✅ Clear installation instructions
- ✅ Working code examples
- ✅ ROS 2 integration guidance
- ✅ Performance considerations
- ✅ Troubleshooting sections
- ✅ Reference links to upstream docs
- ✅ Practical use cases for drone autonomy

---

## Known Issues

### Security Restrictions
During automated preparation, git clone operations were blocked by system security policies. This is why the download script must be executed manually by you.

**Resolution**: Run the download script manually (see Step 1 above).

### Download Script Permissions
The download script may not have execute permissions by default.

**Resolution**: Run `chmod +x download_ontology_tools.sh` before executing.

---

## Success Criteria

After completing the downloads, you should have:

1. **SUMO ontology** with all .kif files accessible
2. **SWI-Prolog documentation** in HTML format
3. **Clingo source code** with examples
4. **Vampire source code** ready to build
5. **E-Prover source code** ready to build
6. **manifest.txt** listing all downloads
7. Working directory structure matching `README.md`

---

## Support Resources

### Documentation Hierarchy
1. **Start here**: `README.md` - Project overview
2. **Tool selection**: `TOOL_COMPARISON.md` - Which tool for which task
3. **Getting started**: `QUICK_START.md` - Hands-on tutorials
4. **Download help**: `DOWNLOAD_INSTRUCTIONS.md` - Detailed download steps
5. **Current status**: `STATUS.md` - This file

### External Resources
- SUMO: https://github.com/ontologyportal/sumo
- SWI-Prolog: https://www.swi-prolog.org/
- Clingo: https://potassco.org/
- Vampire: https://vprover.github.io/
- E-Prover: http://www.eprover.org/

### Project Documentation
- `/home/finley/Github/DroneProjects/flyby-f11/APPROACH.qmd`
- `/home/finley/Github/DroneProjects/flyby-f11/ONTOLOGY_FOUNDATION.qmd`
- `/home/finley/Github/DroneProjects/flyby-f11/SYSTEM_CONSTRAINTS.qmd`

---

## Conclusion

All download infrastructure and comprehensive documentation has been prepared. The next action is to **execute the download script** to populate the directory with all Phase 1 reasoning tools.

**Immediate Action Required**:
```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
chmod +x download_ontology_tools.sh
./download_ontology_tools.sh
```

After successful downloads, proceed with Week 1-2 development tasks as outlined in the timeline above.
