# Ontology Tools Documentation Index

**Location**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/`
**Project**: flyby-f11 Autonomous Decision-Making System
**Phase**: 1 (Weeks 1-6) - Foundation

---

## Start Here

**New to the project?** Read documents in this order:

1. **STATUS.md** - Current download status and immediate next steps
2. **README.md** - Project overview and integration strategy
3. **DOWNLOAD_INSTRUCTIONS.md** - How to download all tools
4. **TOOL_COMPARISON.md** - Which tool to use for which task
5. **QUICK_START.md** - Hands-on tutorials and examples

---

## Document Guide

### STATUS.md
**Purpose**: Current status and action items
**Read if**: You want to know what's been done and what's next
**Key sections**:
- Files created
- Tools to download
- Next steps (immediate actions required)
- Development timeline

**Start here to understand what needs to be done now.**

---

### README.md
**Purpose**: Project overview and architecture
**Read if**: You want to understand the big picture
**Key sections**:
- Quick start (download script)
- Phase 1 priority tools (SUMO, SWI-Prolog, Vampire, Clingo, E-Prover)
- Directory structure
- Integration roadmap
- References

**Read this for the overall vision and strategy.**

---

### DOWNLOAD_INSTRUCTIONS.md
**Purpose**: Step-by-step download guide
**Read if**: You need help downloading the tools
**Key sections**:
- Quick download (automated script)
- Manual download (if script fails)
- Post-download verification
- Troubleshooting
- Build instructions (optional)

**Read this when you're ready to download all tools.**

---

### TOOL_COMPARISON.md
**Purpose**: Detailed comparison of all reasoning tools
**Read if**: You need to choose the right tool for a task
**Key sections**:
- Quick comparison matrix
- Detailed tool analysis (each tool's strengths/weaknesses)
- Recommended tool stack
- Integration architecture diagram
- Performance targets
- Development priorities

**Read this to understand which tool solves which problem.**

---

### QUICK_START.md
**Purpose**: Hands-on tutorials and code examples
**Read if**: You want to start coding immediately
**Key sections**:
- SUMO ontology quick start (viewing .kif files, converting to Prolog)
- SWI-Prolog quick start (installation, knowledge base creation, C++ embedding)
- Vampire quick start (building, writing TPTP proofs)
- Clingo quick start (ASP programs, Python integration)
- E-Prover quick start (building, running proofs)
- Integrated workflow example

**Read this for practical, copy-paste code examples.**

---

### download_ontology_tools.sh
**Purpose**: Automated download script
**Type**: Bash shell script
**Usage**:
```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
chmod +x download_ontology_tools.sh
./download_ontology_tools.sh
```

**What it does**:
- Clones all 5 tool repositories (shallow)
- Downloads SWI-Prolog documentation HTML
- Generates manifest.txt with summary
- Provides progress output

**Run this to download everything automatically.**

---

## Quick Reference

### Phase 1 Tools (Weeks 1-6)

| Tool | Purpose | Priority | Doc Section |
|------|---------|----------|-------------|
| **SUMO** | Ontology foundation | High | README.md, QUICK_START.md |
| **SWI-Prolog** | Runtime reasoning | High | TOOL_COMPARISON.md, QUICK_START.md |
| **Vampire** | Formal verification | Medium | TOOL_COMPARISON.md |
| **Clingo** | Mission planning | Medium | QUICK_START.md |
| **E-Prover** | Backup verifier | Low | QUICK_START.md |

### Common Tasks

| Task | Document to Read |
|------|------------------|
| Download all tools | DOWNLOAD_INSTRUCTIONS.md |
| Understand tool roles | TOOL_COMPARISON.md |
| Start coding with Prolog | QUICK_START.md (Section 2) |
| View SUMO ontology | QUICK_START.md (Section 1) |
| Build theorem provers | DOWNLOAD_INSTRUCTIONS.md |
| Check current status | STATUS.md |
| See integration plan | README.md (Integration Roadmap) |

---

## File Statistics

Total documentation created:
- **6 documents** (5 Markdown + 1 shell script)
- **~2,000 lines** of comprehensive documentation
- **All Phase 1 tools covered** (SUMO, SWI-Prolog, Vampire, Clingo, E-Prover)

---

## Directory Structure (After Downloads)

```
ontology/
├── INDEX.md                           # This file
├── STATUS.md                          # Current status
├── README.md                          # Project overview
├── TOOL_COMPARISON.md                 # Tool comparison
├── QUICK_START.md                     # Hands-on tutorials
├── DOWNLOAD_INSTRUCTIONS.md           # Download guide
├── download_ontology_tools.sh         # Download script
├── manifest.txt                       # Download summary (generated)
├── sumo/                              # SUMO ontology (to be downloaded)
│   ├── Merge.kif
│   ├── Spatial.kif
│   └── ... (.kif files)
├── swi-prolog-docs/                   # SWI-Prolog docs (to be downloaded)
│   ├── manual.html
│   ├── foreign.html
│   ├── embedded.html
│   └── c-api.html
├── vampire/                           # Vampire prover (to be downloaded)
│   ├── README.md
│   └── ... (source code)
├── clingo/                            # Clingo ASP (to be downloaded)
│   ├── README.md
│   └── ... (source code)
└── eprover/                           # E-Prover (to be downloaded)
    ├── README.md
    └── ... (source code)
```

---

## Workflow Diagram

```
┌─────────────────────────────────────────────────────────┐
│                  Start Here                             │
│                  READ: STATUS.md                        │
│                                                         │
│  Current status? What needs to be done next?            │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│              Understand the Project                     │
│              READ: README.md                            │
│                                                         │
│  What are we building? Why these tools?                 │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│              Download Tools                             │
│              RUN: download_ontology_tools.sh            │
│              REFERENCE: DOWNLOAD_INSTRUCTIONS.md        │
│                                                         │
│  Execute script to fetch all repositories               │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│              Choose the Right Tool                      │
│              READ: TOOL_COMPARISON.md                   │
│                                                         │
│  Which tool for runtime? Planning? Verification?        │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│              Start Coding                               │
│              READ: QUICK_START.md                       │
│                                                         │
│  Hands-on tutorials with working code examples          │
└─────────────────────────────────────────────────────────┘
```

---

## Immediate Next Action

**Execute the download script**:

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
chmod +x download_ontology_tools.sh
./download_ontology_tools.sh
```

After successful downloads, read `manifest.txt` for a summary, then proceed to `QUICK_START.md` for hands-on tutorials.

---

## Support

For questions about:
- **Tool selection**: Read TOOL_COMPARISON.md
- **Installation**: Read DOWNLOAD_INSTRUCTIONS.md
- **Usage**: Read QUICK_START.md
- **Integration**: Read README.md (Integration Roadmap)
- **Current status**: Read STATUS.md

For project-wide documentation:
- `/home/finley/Github/DroneProjects/flyby-f11/APPROACH.qmd`
- `/home/finley/Github/DroneProjects/flyby-f11/ONTOLOGY_FOUNDATION.qmd`
- `/home/finley/Github/DroneProjects/CLAUDE.md`

---

**Last Updated**: December 25, 2025
**Developer**: Finley Holt
**Project**: flyby-f11 Autonomous Drone System
