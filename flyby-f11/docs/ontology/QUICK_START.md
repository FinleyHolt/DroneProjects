# Ontology Tools Quick Start Guide

Fast-track guide to getting started with each reasoning tool for the flyby-f11 project.

## Prerequisites

```bash
# System dependencies
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    tree \
    libgmp-dev \
    zlib1g-dev

# Python dependencies (for Clingo)
pip3 install clingo
```

## 1. SUMO Ontology Quick Start

### Viewing SUMO Files

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/sumo

# Browse key ontology files
ls -lh *.kif

# View core concepts
head -100 Merge.kif

# Search for spatial concepts
grep -n "subclass.*Spatial" *.kif

# Search for agent concepts
grep -n "subclass.*Agent" *.kif
```

### Key SUMO Files for Drones

| File | Purpose | Relevance |
|------|---------|-----------|
| `Merge.kif` | Core ontology | Foundational concepts (Object, Process, Agent) |
| `Spatial.kif` | Spatial reasoning | Regions, trajectories, spatial relations |
| `Mid-level-ontology.kif` | Mid-level concepts | Motion, transportation |
| `Military.kif` | Military domain | Surveillance, reconnaissance (optional) |

### Example SUMO Concepts

```lisp
; From Merge.kif - Object hierarchy
(subclass PhysicalObject Object)
(subclass Agent PhysicalObject)
(subclass AutonomousAgent Agent)

; From Spatial.kif - Spatial relations
(subclass Region SpatialThing)
(subclass Trajectory Region)
```

### Converting SUMO to Prolog

Create a simple converter script:

```python
# sumo_to_prolog.py
import re

def kif_to_prolog(kif_file, prolog_file):
    """Convert SUMO .kif to Prolog facts."""
    with open(kif_file, 'r') as f:
        lines = f.readlines()

    with open(prolog_file, 'w') as out:
        for line in lines:
            line = line.strip()

            # Convert (subclass A B) to subclass(a, b).
            if line.startswith('(subclass '):
                match = re.match(r'\(subclass\s+(\w+)\s+(\w+)\)', line)
                if match:
                    out.write(f"subclass({match.group(1).lower()}, {match.group(2).lower()}).\n")

            # Convert (instance A B) to instance(a, b).
            elif line.startswith('(instance '):
                match = re.match(r'\(instance\s+(\w+)\s+(\w+)\)', line)
                if match:
                    out.write(f"instance({match.group(1).lower()}, {match.group(2).lower()}).\n")

# Usage
kif_to_prolog('Merge.kif', 'drone_ontology.pl')
```

---

## 2. SWI-Prolog Quick Start

### Installation

```bash
# Install SWI-Prolog
sudo apt-add-repository ppa:swi-prolog/stable
sudo apt update
sudo apt install swi-prolog

# Verify installation
swipl --version
```

### Basic Prolog Session

```bash
# Start interactive Prolog
swipl

# In Prolog REPL:
?- write('Hello from Prolog').
Hello from Prolog
true.

?- halt.  % Exit
```

### Creating a Knowledge Base

Create `drone_kb.pl`:

```prolog
% drone_kb.pl - Drone domain knowledge base

% Waypoint definitions
waypoint(home).
waypoint(alpha).
waypoint(bravo).
waypoint(charlie).

% Waypoint coordinates (X, Y, Altitude)
location(home, 0, 0, 50).
location(alpha, 100, 50, 100).
location(bravo, 200, 100, 80).
location(charlie, 150, 200, 120).

% Safe altitude constraints
minimum_altitude(30).
maximum_altitude(150).

% Geofence boundaries
geofence_bounds(-50, -50, 300, 300).

% Rules for safety checks
safe_altitude(WP) :-
    location(WP, _, _, Alt),
    minimum_altitude(MinAlt),
    maximum_altitude(MaxAlt),
    Alt >= MinAlt,
    Alt =< MaxAlt.

within_geofence(WP) :-
    location(WP, X, Y, _),
    geofence_bounds(MinX, MinY, MaxX, MaxY),
    X >= MinX, X =< MaxX,
    Y >= MinY, Y =< MaxY.

safe_waypoint(WP) :-
    waypoint(WP),
    safe_altitude(WP),
    within_geofence(WP).

% Mission planning
can_visit(WP) :-
    safe_waypoint(WP).

% Spatial reasoning (simplified Euclidean distance)
distance(WP1, WP2, Dist) :-
    location(WP1, X1, Y1, Z1),
    location(WP2, X2, Y2, Z2),
    Dist is sqrt((X2-X1)^2 + (Y2-Y1)^2 + (Z2-Z1)^2).
```

### Testing the Knowledge Base

```bash
swipl -s drone_kb.pl

# Query all waypoints
?- waypoint(X).
X = home ;
X = alpha ;
X = bravo ;
X = charlie.

# Check if waypoint is safe
?- safe_waypoint(alpha).
true.

# Find all safe waypoints
?- safe_waypoint(WP).
WP = home ;
WP = alpha ;
WP = bravo ;
WP = charlie.

# Calculate distance
?- distance(home, alpha, D).
D = 111.803.
```

### Embedding in C++ (ROS 2 Node)

Example skeleton:

```cpp
// prolog_reasoner_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <SWI-cpp.h>
#include <SWI-Prolog.h>

class PrologReasonerNode : public rclcpp::Node {
public:
    PrologReasonerNode() : Node("prolog_reasoner") {
        // Initialize Prolog engine
        const char *argv[] = {"prolog_reasoner", nullptr};
        if (!PL_initialise(1, (char **)argv)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Prolog");
            return;
        }

        // Load knowledge base
        PlCall("consult('drone_kb.pl')");

        RCLCPP_INFO(this->get_logger(), "Prolog reasoner initialized");
    }

    bool isSafeWaypoint(const std::string &wp_name) {
        PlTermv args(1);
        args[0] = PlTerm(wp_name.c_str());

        return PlCall("safe_waypoint", args);
    }

    ~PrologReasonerNode() {
        PL_cleanup(0);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrologReasonerNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 3. Vampire Theorem Prover Quick Start

### Building Vampire

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/vampire

# Build Vampire
mkdir build && cd build
cmake ..
make -j$(nproc)

# Test installation
./bin/vampire --version
```

### Example TPTP Problem

Create `mission_safety.p`:

```tptp
% mission_safety.p - Verify mission safety properties

% Axioms: All waypoints in mission must be safe
fof(waypoint_safety, axiom,
    ![WP]: (in_mission(WP) => safe(WP))
).

% Axioms: Safe waypoints must be within geofence
fof(geofence_constraint, axiom,
    ![WP]: (safe(WP) => within_geofence(WP))
).

% Facts: Mission waypoints
fof(mission_wp1, axiom, in_mission(alpha)).
fof(mission_wp2, axiom, in_mission(bravo)).

% Facts: Waypoint safety
fof(alpha_safe, axiom, safe(alpha)).
fof(bravo_safe, axiom, safe(bravo)).

% Conjecture: Mission is safe (all waypoints in geofence)
fof(mission_safe, conjecture,
    ![WP]: (in_mission(WP) => within_geofence(WP))
).
```

### Running Vampire

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/vampire/build

# Verify the conjecture
./bin/vampire --mode casc mission_safety.p

# Output will show if proof was found
# Look for "Refutation found" or "Timeout"
```

---

## 4. Clingo ASP Quick Start

### Installation

```bash
# Install via pip (easiest)
pip3 install clingo

# Verify
clingo --version
```

### Example ASP Program

Create `mission_plan.lp`:

```asp
% mission_plan.lp - Route planning with constraints

% Define waypoints
waypoint(home).
waypoint(alpha).
waypoint(bravo).
waypoint(charlie).

% Define connections with costs (fuel units)
edge(home, alpha, 10).
edge(home, bravo, 15).
edge(alpha, bravo, 8).
edge(alpha, charlie, 12).
edge(bravo, charlie, 7).

% Time steps for mission (0 to 4)
time(0..4).

% Initial position
at(home, 0).

% Generate: Choose at most one move per time step
{ move(From, To, T) : edge(From, To, _) } <= 1 :- time(T).

% Effect: Position after move
at(To, T+1) :- move(From, To, T), at(From, T).
at(Pos, T+1) :- at(Pos, T), not moved(T), time(T), T < 4.

% moved/1 helper predicate
moved(T) :- move(_, _, T).

% Goal: Visit all waypoints at least once
visited(WP) :- at(WP, _), waypoint(WP).
:- waypoint(WP), not visited(WP).

% Minimize total fuel cost
#minimize { Cost,From,To,T : move(From,To,T), edge(From,To,Cost) }.

% Display plan
#show move/3.
```

### Running Clingo

```bash
# Find optimal plan
clingo mission_plan.lp

# Output shows moves like:
# move(home,alpha,0) move(alpha,charlie,1) ...
```

### Python Integration

```python
import clingo

def solve_mission_plan():
    """Solve mission planning problem with Clingo."""
    ctl = clingo.Control()

    # Load ASP program
    ctl.load('mission_plan.lp')
    ctl.ground([("base", [])])

    # Solve
    solutions = []
    with ctl.solve(yield_=True) as handle:
        for model in handle:
            solution = [str(atom) for atom in model.symbols(shown=True)]
            solutions.append(solution)

    return solutions

# Usage
plans = solve_mission_plan()
print(f"Found {len(plans)} optimal plans")
for plan in plans[:1]:  # Show first plan
    print("Plan:", plan)
```

---

## 5. E-Prover Quick Start

### Building E-Prover

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/eprover

# Configure and build
./configure
make -j$(nproc)

# Test
./PROVER/eprover --version
```

### Example Problem (TPTP Format)

Create `trajectory_safety.p`:

```tptp
% trajectory_safety.p - Prove trajectory is collision-free

% Axiom: Safe trajectory has no intersections with obstacles
fof(safe_trajectory_def, axiom,
    ![T]: (safe_trajectory(T) <=> ~obstacle_intersection(T))
).

% Axiom: Planned trajectory
fof(planned_trajectory, axiom, trajectory(planned_path)).

% Axiom: No obstacles detected on planned path
fof(no_obstacles, axiom, ~obstacle_intersection(planned_path)).

% Conjecture: Planned path is safe
fof(path_safe, conjecture, safe_trajectory(planned_path)).
```

### Running E-Prover

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology/eprover

# Run prover
./PROVER/eprover trajectory_safety.p

# Look for "Proof found" in output
```

---

## Integrated Workflow Example

### Scenario: Verify and Execute Mission

1. **Define mission in Prolog**:
   ```prolog
   mission_waypoints([home, alpha, bravo, charlie]).
   ```

2. **Plan route with Clingo**:
   ```bash
   clingo mission_plan.lp > plan.txt
   ```

3. **Verify safety with Vampire**:
   ```bash
   vampire --mode casc mission_safety.p
   ```

4. **Execute with behavior tree** (ROS 2 integration)

---

## Next Steps

1. **Week 1**: Work through SWI-Prolog tutorial in `swi-prolog-docs/manual.html`
2. **Week 2**: Extract SUMO subset and convert to Prolog
3. **Week 3**: Prototype Prolog embedding in ROS 2 C++ node
4. **Week 4**: Test basic reasoning queries in simulation
5. **Week 5**: Integrate Clingo for mission planning
6. **Week 6**: Add Vampire pre-flight verification

---

## Troubleshooting

### SWI-Prolog doesn't find libraries
```bash
export LD_LIBRARY_PATH=/usr/lib/swi-prolog/lib/x86_64-linux:$LD_LIBRARY_PATH
```

### Vampire build fails
```bash
# Install dependencies
sudo apt install libz3-dev
```

### Clingo import error in Python
```bash
pip3 install --upgrade clingo
```

---

## Reference Documentation

- **SUMO**: `sumo/documentation.kif`
- **SWI-Prolog**: `swi-prolog-docs/manual.html`
- **Vampire**: `vampire/README.md`
- **Clingo**: `clingo/README.md` and https://potassco.org/doc/
- **E-Prover**: `eprover/DOC/MANUAL.txt`

---

## Testing Your Setup

Run this comprehensive test:

```bash
cd /home/finley/Github/DroneProjects/flyby-f11/docs/ontology

# 1. Test SWI-Prolog
echo "Testing SWI-Prolog..."
swipl --version && echo "✓ SWI-Prolog OK"

# 2. Test Clingo
echo "Testing Clingo..."
clingo --version && echo "✓ Clingo OK"

# 3. Test Vampire (if built)
echo "Testing Vampire..."
./vampire/build/bin/vampire --version 2>/dev/null && echo "✓ Vampire OK" || echo "✗ Vampire not built"

# 4. Test E-Prover (if built)
echo "Testing E-Prover..."
./eprover/PROVER/eprover --version 2>/dev/null && echo "✓ E-Prover OK" || echo "✗ E-Prover not built"

echo "Setup test complete!"
```

All tools are ready for Phase 1 development!
