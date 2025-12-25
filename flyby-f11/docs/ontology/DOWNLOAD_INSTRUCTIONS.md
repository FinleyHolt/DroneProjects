# Download Instructions for Ontology Tools

## Quick Download

Run the automated download script to fetch all Phase 1 tools:

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
chmod +x download_ontology_tools.sh
./download_ontology_tools.sh
```

The script will:
1. Clone SUMO ontology repository (shallow clone)
2. Clone Vampire theorem prover
3. Clone Clingo ASP solver
4. Clone E-Prover
5. Download SWI-Prolog documentation (HTML)
6. Generate `manifest.txt` with download summary

**Estimated download size**: ~150 MB (with shallow clones)
**Estimated time**: 2-5 minutes (depending on network speed)

---

## Manual Download (if script fails)

If the automated script encounters issues, download tools manually:

### 1. SUMO Ontology

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
git clone --depth 1 https://github.com/ontologyportal/sumo.git sumo
```

**Verify**: Check for `Merge.kif` in `sumo/` directory
```bash
ls -lh sumo/Merge.kif
```

---

### 2. Vampire Theorem Prover

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
git clone --depth 1 https://github.com/vprover/vampire.git vampire
```

**Verify**: Check for Vampire source
```bash
ls vampire/README.md
```

---

### 3. Clingo (Potassco)

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
git clone --depth 1 https://github.com/potassco/clingo.git clingo
```

**Verify**: Check for Clingo README
```bash
ls clingo/README.md
```

---

### 4. E-Prover

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
git clone --depth 1 https://github.com/eprover/eprover.git eprover
```

**Verify**: Check for E-Prover source
```bash
ls eprover/README.md
```

---

### 5. SWI-Prolog Documentation

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology
mkdir -p swi-prolog-docs

# Download manual overview
wget -O swi-prolog-docs/manual.html "https://www.swi-prolog.org/pldoc/doc_for?object=manual"

# Download FFI guide
wget -O swi-prolog-docs/foreign.html "https://www.swi-prolog.org/pldoc/man?section=foreign"

# Download embedding guide
wget -O swi-prolog-docs/embedded.html "https://www.swi-prolog.org/pldoc/man?section=embedded"

# Download C API reference
wget -O swi-prolog-docs/c-api.html "https://www.swi-prolog.org/pldoc/man?section=foreigninclude"
```

**Verify**: Check downloaded HTML files
```bash
ls -lh swi-prolog-docs/*.html
```

---

## Post-Download Verification

After downloading all tools (automated or manual), verify the structure:

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology

# Check directory structure
tree -L 2 . || ls -R

# Expected output:
# .
# ├── clingo/
# │   ├── README.md
# │   └── ... (source files)
# ├── eprover/
# │   ├── README.md
# │   └── ... (source files)
# ├── sumo/
# │   ├── Merge.kif
# │   ├── Spatial.kif
# │   └── ... (ontology files)
# ├── swi-prolog-docs/
# │   ├── manual.html
# │   ├── foreign.html
# │   ├── embedded.html
# │   └── c-api.html
# ├── vampire/
# │   ├── README.md
# │   └── ... (source files)
# ├── download_ontology_tools.sh
# ├── README.md
# ├── TOOL_COMPARISON.md
# └── QUICK_START.md
```

---

## Troubleshooting

### Git clone fails with permission error

Check Git configuration:
```bash
git config --global --list
```

Try HTTPS instead of SSH:
```bash
# All repositories use HTTPS URLs already
# If still failing, check network connectivity:
ping github.com
```

### wget fails to download SWI-Prolog docs

Alternative: Use curl
```bash
curl -o swi-prolog-docs/manual.html "https://www.swi-prolog.org/pldoc/doc_for?object=manual"
```

Or download manually:
1. Open URL in browser
2. Save page as HTML
3. Move to `swi-prolog-docs/` directory

### Disk space issues

Check available space:
```bash
df -h /home/finley/Github/DroneProjects/flyby-f11
```

Shallow clones should only require ~150 MB total. If space is limited:
- Use `--depth 1` for all clones (already done in script)
- Skip E-Prover if needed (backup to Vampire)
- Download only essential SWI-Prolog docs (manual.html)

---

## Build Instructions (Optional)

After downloading, you can build the tools for local testing:

### Build Vampire

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/vampire
mkdir build && cd build
cmake ..
make -j$(nproc)

# Test
./bin/vampire --version
```

### Build E-Prover

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/eprover
./configure
make -j$(nproc)

# Test
./PROVER/eprover --version
```

### Install Clingo (Python)

```bash
# System-wide installation
pip3 install clingo

# Or use downloaded source
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/clingo
cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Install SWI-Prolog

```bash
# Ubuntu/Debian
sudo apt-add-repository ppa:swi-prolog/stable
sudo apt update
sudo apt install swi-prolog swi-prolog-dev

# Verify
swipl --version
```

---

## Next Steps

After successful download:

1. **Review tools**: Read `README.md` for overview
2. **Compare tools**: Read `TOOL_COMPARISON.md` for selection guidance
3. **Get started**: Follow `QUICK_START.md` for hands-on tutorials
4. **Build tools**: Compile Vampire and E-Prover (optional)
5. **Test setup**: Run test commands from `QUICK_START.md`

---

## Support

If you encounter issues:
1. Check `manifest.txt` for download status
2. Review error messages carefully
3. Try manual download commands above
4. Verify network connectivity to GitHub
5. Check available disk space

For tool-specific help:
- **SUMO**: https://github.com/ontologyportal/sumo/issues
- **SWI-Prolog**: https://www.swi-prolog.org/FAQ/
- **Vampire**: https://github.com/vprover/vampire/issues
- **Clingo**: https://potassco.org/doc/
- **E-Prover**: http://www.eprover.org/
