# TPTP (Thousands of Problems for Theorem Provers) Format Reference

## Overview

TPTP is a standardized library of test problems for automated theorem proving (ATP) systems. The TPTP language provides syntax for expressing logical formulas in various forms, particularly First-Order Logic (FOL).

**Official Website**: http://www.tptp.org/

## Key TPTP Formats

### 1. FOF (First-Order Form)

FOF is the classic first-order logic syntax used in TPTP.

#### Basic Syntax

```tptp
fof(name, role, formula).
```

- **name**: Unique identifier for the formula
- **role**: Purpose of the formula (axiom, hypothesis, conjecture, theorem, etc.)
- **formula**: First-order logic expression

#### Roles

- `axiom`: Assumed to be true
- `hypothesis`: Assumed true for a particular problem
- `conjecture`: Statement to be proved
- `theorem`: Previously proved statement
- `lemma`: Intermediate result
- `negated_conjecture`: Negation of conjecture (for refutation proofs)

#### Logical Operators

- `~`: Negation (NOT)
- `&`: Conjunction (AND)
- `|`: Disjunction (OR)
- `=>`: Implication
- `<=>`: Equivalence
- `<~>`: XOR (exclusive or)

#### Quantifiers

- `!`: Universal quantification (for all)
- `?`: Existential quantification (there exists)

#### Example FOF

```tptp
% All humans are mortal
fof(humans_mortal, axiom,
    ![X]: (human(X) => mortal(X))
).

% Socrates is human
fof(socrates_human, axiom,
    human(socrates)
).

% Prove Socrates is mortal
fof(socrates_mortal, conjecture,
    mortal(socrates)
).
```

### 2. TFF (Typed First-order Form)

TFF extends FOF with type information for terms and predicates.

#### Type Declarations

```tptp
tff(type_declaration, type, name: type_expression).
```

#### Built-in Types

- `$i`: Individual (default type)
- `$o`: Boolean (proposition)
- `$int`: Integer
- `$rat`: Rational
- `$real`: Real number

#### Example TFF

```tptp
% Type declarations
tff(person_type, type, person: $tType).
tff(human_decl, type, human: person > $o).
tff(mortal_decl, type, mortal: person > $o).
tff(socrates_decl, type, socrates: person).

% Axioms with types
tff(humans_mortal, axiom,
    ![X:person]: (human(X) => mortal(X))
).

tff(socrates_human, axiom,
    human(socrates)
).

% Conjecture
tff(socrates_mortal, conjecture,
    mortal(socrates)
).
```

### 3. CNF (Clause Normal Form)

CNF represents formulas in clausal form (disjunction of literals).

```tptp
cnf(name, role, clause).
```

#### Example CNF

```tptp
cnf(human_mortal, axiom, ~human(X) | mortal(X)).
cnf(socrates_human, axiom, human(socrates)).
cnf(prove_mortal, negated_conjecture, ~mortal(socrates)).
```

### 4. THF (Typed Higher-order Form)

THF supports higher-order logic with full type system.

```tptp
thf(name, role, formula).
```

## TPTP File Structure

### Standard Layout

```tptp
%------------------------------------------------------------------------------
% File     : problem_name.p
% Domain   : domain_name
% Problem  : problem_description
% Version  : version_number
% English  : natural_language_description
%------------------------------------------------------------------------------

% Include directives
include('Axioms/axiom_file.ax').

% Type declarations (for TFF/THF)
tff(type_decl, type, ...).

% Axioms
fof(axiom_1, axiom, ...).
fof(axiom_2, axiom, ...).

% Hypotheses
fof(hypothesis_1, hypothesis, ...).

% Conjecture
fof(goal, conjecture, ...).

%------------------------------------------------------------------------------
```

### Comments

- `%`: Line comment
- `/* ... */`: Block comment

## Advanced Features

### Conditional Formulas

```tptp
$ite(condition, then_formula, else_formula)
```

### Let Expressions

```tptp
$let(X := term, formula)
```

### Equality

```tptp
% Equality is built-in
fof(equality_example, axiom, a = b).
fof(inequality, axiom, a != c).
```

## Integration with Automated Theorem Provers

### Common ATP Systems Supporting TPTP

1. **Vampire**: High-performance first-order ATP
2. **E Prover**: Equational theorem prover
3. **SPASS**: FOL with equality
4. **Z3**: SMT solver with TPTP support
5. **CVC5**: SMT solver
6. **Leo-III**: Higher-order ATP
7. **Satallax**: Higher-order ATP

### Using TPTP with Python

```python
# Using PyRes (Python Resolution Theorem Prover)
from pyres import ResolutionProver

# Load TPTP file
prover = ResolutionProver()
result = prover.prove_from_file('problem.p')

# Or use tptp library
import tptp
problem = tptp.parse_file('problem.p')
```

### Using TPTP with System Calls

```bash
# Vampire
vampire --mode casc problem.p

# E Prover
eprover --auto problem.p

# Z3 with TPTP frontend
z3 -smt2 problem.p
```

## TPTP for Autonomous Robotics

### Modeling Robot Knowledge

```tptp
% Type declarations for robot domain
tff(location_type, type, location: $tType).
tff(robot_type, type, robot: $tType).
tff(at_decl, type, at: (robot * location) > $o).
tff(safe_decl, type, safe: location > $o).
tff(can_reach_decl, type, can_reach: (location * location) > $o).

% Axioms: Robot at safe location
tff(current_location, axiom, at(robot1, loc_a)).
tff(loc_a_safe, axiom, safe(loc_a)).

% Axiom: Can reach connected locations
tff(reachability, axiom,
    ![L1:location, L2:location]:
        (can_reach(L1, L2) =>
         ![R:robot]: (at(R, L1) => can_move_to(R, L2)))
).

% Goal: Prove robot can reach target safely
tff(safe_navigation, conjecture,
    ?[L:location]: (safe(L) & can_reach(loc_a, L))
).
```

### Mission Planning Verification

```tptp
% Verify mission plan satisfies constraints
tff(waypoint_type, type, waypoint: $tType).
tff(battery_type, type, battery_level: $int).
tff(distance_type, type, distance: (waypoint * waypoint) > $int).

% Constraint: Battery sufficient for mission
tff(battery_constraint, axiom,
    ![W1:waypoint, W2:waypoint]:
        (distance(W1, W2) =< battery_level)
).

% Conjecture: Mission is feasible
tff(mission_feasible, conjecture,
    can_complete_mission(robot1)
).
```

## TPTP Libraries and Resources

### Online Resources

- **TPTP Problem Library**: http://www.tptp.org/cgi-bin/SeeTPTP
- **TPTP Syntax Guide**: http://www.tptp.org/TPTP/SyntaxBNF.html
- **SystemOnTPTP**: Online interface to ATP systems

### Downloading TPTP

```bash
# Clone TPTP problem library (large)
git clone https://github.com/TPTPWorld/TPTP-v8.0.0.git

# Or download specific problems
wget http://www.tptp.org/cgi-bin/SeeTPTP?Category=Problems&Domain=...
```

### Python Libraries

```bash
pip install tptp-lark-parser
pip install pyres
pip install z3-solver
```

## Integration with Ontologies

### Converting OWL to TPTP

OWL ontologies can be translated to FOF/TFF for reasoning:

```python
from owlready2 import get_ontology
# Use tools like OWL2TPTP or custom converters
```

### Common Use Cases

1. **Consistency Checking**: Verify ontology has no contradictions
2. **Subsumption Testing**: Check class hierarchies
3. **Instance Checking**: Verify individuals belong to classes
4. **Query Answering**: Prove queries against knowledge base

## Best Practices

1. **Use Types**: Prefer TFF over FOF for better error detection
2. **Modular Axioms**: Break complex axioms into smaller pieces
3. **Document Formulas**: Use comments to explain axioms
4. **Test Incrementally**: Add axioms one at a time
5. **Use Standard Names**: Follow TPTP naming conventions
6. **Include Metadata**: Add problem description headers

## References

- TPTP Website: http://www.tptp.org/
- TPTP Syntax BNF: http://www.tptp.org/TPTP/SyntaxBNF.html
- Sutcliffe, G. (2017). "The TPTP Problem Library and Associated Infrastructure"
- CADE ATP System Competition (CASC): http://www.tptp.org/CASC/

## Notes for flyby-f11 Project

- Use TPTP for formal verification of mission safety constraints
- Model flight rules and regulations as axioms
- Verify behavior tree plans satisfy mission requirements
- Integrate with ontology-based reasoning for autonomous decision-making
- Consider using TPTP for runtime verification of safe states
