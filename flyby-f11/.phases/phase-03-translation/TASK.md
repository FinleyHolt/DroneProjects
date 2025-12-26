# Phase 3: SUMO to Prolog Translation

## Overview

Build the translation pipeline that converts heavyweight SUMO reasoning results into lightweight Prolog rules for real-time execution. This creates the bridge between planning mode (offline, heavyweight) and execution mode (online, lightweight).

## Human Description

SUMO is too computationally expensive to run during flight, but Prolog can execute logic queries in microseconds. This phase creates a translator that:
1. Takes verified mission plans from SUMO (planning mode)
2. Extracts the essential rules, constraints, and decision logic
3. Generates optimized Prolog predicates for execution mode
4. Validates that the translation preserves safety properties

The output is a set of `.pl` files that the drone loads at runtime for real-time constraint checking without needing the heavyweight reasoner.

## AI Agent Instructions

### Prerequisites
- Phase 1 completed (SUMO toolchain)
- Phase 2 completed (UAV ontology)
- Understanding of both KIF and Prolog syntax
- Familiarity with logic programming concepts

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Design Translation Schema**
   - Document mapping from SUMO concepts to Prolog predicates
   - Define Prolog predicate naming conventions
   - Specify how axioms become Prolog rules
   - Create translation specification document

2. **Implement Translator**
   - Location: `ontology/translation/sumo_to_prolog.py`
   - Parse SUMO KIF files (uav_domain.kif)
   - Extract class hierarchies, axioms, constraints
   - Generate Prolog predicates
   - Optimize for execution speed

3. **Create Prolog Templates**
   - Location: `ontology/execution_mode/templates/`
   - Base constraint checking template
   - Mission validation template
   - Safety monitoring template
   - Emergency handling template

4. **Generate Execution Rules**
   - Run translator on UAV ontology
   - Output location: `ontology/execution_mode/uav_rules.pl`
   - Include type definitions, constraints, safety rules
   - Add runtime query predicates

5. **Validate Translation**
   - Compare SUMO reasoning results with Prolog results
   - Ensure safety properties are preserved
   - Test with scenarios from Phase 2
   - Verify performance meets real-time requirements (<10ms query time)

6. **Document Translation Process**
   - Location: `ontology/translation/TRANSLATION.md`
   - Explain mapping rules
   - Document any SUMO features not translated
   - Provide examples of before/after translations

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `sumo_to_prolog.py` translator implemented and working
- [ ] Translation schema documented
- [ ] `uav_rules.pl` generated from UAV ontology
- [ ] Prolog rules match SUMO reasoning for test scenarios
- [ ] Query performance meets real-time requirements (<10ms)
- [ ] Safety properties preserved in translation
- [ ] Translation process documented

### Verification

Run automated verification:
```bash
bash .phases/phase-03-translation/verification.sh
```

### Time Estimate
10-15 hours (includes translator implementation, validation, optimization)

### Common Pitfalls

- **Incomplete translation**: Not all SUMO features map to Prolog - document what's excluded
- **Lost semantics**: Ensure critical safety properties survive translation
- **Performance issues**: Prolog queries must be optimized for real-time use
- **Type system mismatch**: SUMO's type system is richer than Prolog's
- **Quantifiers**: Universal/existential quantifiers need careful translation
- **Negation**: Prolog's negation-as-failure differs from classical logic

### References

- [SWI-Prolog Documentation](https://www.swi-prolog.org/pldoc/doc_for?object=manual)
- [KIF to Prolog Translation Patterns](http://www.cs.miami.edu/home/geoff/Courses/TPTP-Tutorial/)
- [Prolog Optimization Techniques](https://www.metalevel.at/prolog/optimization)
- [flyby-f11 ONTOLOGY_FOUNDATION.qmd](../ONTOLOGY_FOUNDATION.qmd)

### Dependencies
See `dependencies.json` - requires Phase 1 and Phase 2 completion.

### Next Phase
After completion, proceed to Phase 4: Execution Mode Runtime
