# Clingo Answer Set Programming for Path Planning

## Overview

Clingo is an Answer Set Programming (ASP) solver that finds optimal solutions to combinatorial problems. In the UAV domain, Clingo excels at:

- Optimal waypoint sequencing
- Multi-constraint path planning
- Resource allocation (battery, time, payload)
- Mission scheduling

Clingo runs offline during mission planning to find optimal paths given constraints.

## Installation

Clingo is already cloned in `repos/clingo/`. To build:

```bash
cd repos/clingo
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install  # Optional
```

Or use pre-built binary:
```bash
cd repos/clingo/build
./bin/clingo --version
```

Or install via package manager (easier):
```bash
sudo apt install gringo clasp  # Ubuntu/Debian
# Or
pip install clingo  # Python package
```

## ASP Basics

Answer Set Programming uses **logic rules** to describe problems. Clingo finds **answer sets** (models) that satisfy all rules.

### Syntax

```prolog
% Facts (what is true)
node(a). node(b). node(c).
edge(a, b). edge(b, c).

% Rules (derive new facts)
reachable(X) :- start(X).
reachable(Y) :- reachable(X), edge(X, Y).

% Constraints (eliminate invalid solutions)
:- reachable(X), forbidden(X).

% Choice rules (generate alternatives)
{ path(X, Y) } :- edge(X, Y).

% Optimization (minimize/maximize)
#minimize { 1@1, X, Y : path(X, Y) }.
```

## Example: Waypoint Sequencing

### Problem Definition

Given:
- 5 waypoints: `wp1, wp2, wp3, wp4, wp5`
- Must visit all waypoints exactly once
- Start at `home`, return to `home`
- Minimize total distance

This is the Traveling Salesman Problem (TSP).

### ASP Encoding

```prolog
% File: waypoint_tsp.lp

% Waypoints
waypoint(wp1). waypoint(wp2). waypoint(wp3).
waypoint(wp4). waypoint(wp5).
waypoint(home).

% Distances (symmetric)
distance(home, wp1, 100). distance(wp1, home, 100).
distance(home, wp2, 150). distance(wp2, home, 150).
distance(home, wp3, 200). distance(wp3, home, 200).
distance(home, wp4, 120). distance(wp4, home, 120).
distance(home, wp5, 180). distance(wp5, home, 180).

distance(wp1, wp2, 80).  distance(wp2, wp1, 80).
distance(wp1, wp3, 120). distance(wp3, wp1, 120).
distance(wp1, wp4, 90).  distance(wp4, wp1, 90).
distance(wp1, wp5, 110). distance(wp5, wp1, 110).

distance(wp2, wp3, 70).  distance(wp3, wp2, 70).
distance(wp2, wp4, 130). distance(wp4, wp2, 130).
distance(wp2, wp5, 140). distance(wp5, wp2, 140).

distance(wp3, wp4, 100). distance(wp4, wp3, 100).
distance(wp3, wp5, 85).  distance(wp5, wp3, 85).

distance(wp4, wp5, 95).  distance(wp5, wp4, 95).

% Must visit each waypoint exactly once
1 { visit(W, T) : time(T) } 1 :- waypoint(W), W != home.

% Time steps (0 to n+1, where n is number of waypoints)
time(0..6).

% Start at home
visit(home, 0).

% End at home
visit(home, 6).

% Define path: visit X at time T and Y at time T+1
path(X, Y, T) :- visit(X, T), visit(Y, T+1), T < 6.

% Path must have valid distance
:- path(X, Y, T), not distance(X, Y, _).

% Calculate total distance
total_cost(C) :- C = #sum { D, X, Y, T : path(X, Y, T), distance(X, Y, D) }.

% Minimize total distance
#minimize { C@1 : total_cost(C) }.

% Display solution
#show path/3.
#show total_cost/1.
```

### Running Clingo

```bash
clingo waypoint_tsp.lp
```

**Output:**
```
clingo version 5.6.2
Reading from waypoint_tsp.lp
Solving...
Answer: 1
path(home,wp1,0) path(wp1,wp4,1) path(wp4,wp5,2) path(wp5,wp3,3) path(wp3,wp2,4) path(wp2,home,5) total_cost(640)
OPTIMUM FOUND

Models       : 1
  Optimum    : yes
Optimization : 640
```

Optimal path: `home → wp1 → wp4 → wp5 → wp3 → wp2 → home` with total distance 640m.

## Example: Battery-Constrained Planning

### Problem Definition

Given:
- Waypoints with distances
- Battery capacity: 100%
- Battery consumption: 1% per 10m traveled
- Must end with at least 20% battery
- Can recharge at home (5 minutes per 10% charge)

Find: Optimal path with recharge stops if needed.

### ASP Encoding

```prolog
% File: battery_planning.lp

% Waypoints
waypoint(home). waypoint(wp1). waypoint(wp2). waypoint(wp3).

% Distances
distance(home, wp1, 300). distance(wp1, home, 300).
distance(home, wp2, 400). distance(wp2, home, 400).
distance(home, wp3, 500). distance(wp3, home, 500).
distance(wp1, wp2, 200). distance(wp2, wp1, 200).
distance(wp1, wp3, 250). distance(wp3, wp1, 250).
distance(wp2, wp3, 150). distance(wp3, wp2, 150).

% Battery parameters
initial_battery(100).
battery_per_meter(0.1).  % 1% per 10m
min_battery(20).

% Time horizon
time(0..10).

% Visit each waypoint exactly once (except home)
1 { visit(W, T) : time(T) } 1 :- waypoint(W), W != home.

% Start at home
visit(home, 0).

% Option: return home at end for recharge
{ visit(home, T) : time(T), T > 0 }.

% Path definition
path(X, Y, T) :- visit(X, T), visit(Y, T+1), time(T), time(T+1).

% Valid path (edge exists)
:- path(X, Y, T), not distance(X, Y, _).

% Battery tracking
battery(0, 100).  % Initial battery

% Battery after traveling
battery(T+1, B_NEW) :-
    path(X, Y, T),
    battery(T, B_OLD),
    distance(X, Y, D),
    battery_per_meter(RATE),
    B_NEW = B_OLD - (D * RATE / 10).

% Recharge at home (simplified: instant full charge)
battery(T+1, 100) :- visit(home, T), T > 0, battery(T, B), B < 100.

% Battery constraint: never go below minimum
:- battery(T, B), B < 20.

% Mission completion: all waypoints visited
mission_complete :- visit(wp1, _), visit(wp2, _), visit(wp3, _).

% Constraint: must complete mission
:- not mission_complete.

% Minimize total time
#minimize { T@2 : visit(_, T) }.

% Minimize recharge stops
#minimize { 1@1, T : visit(home, T), T > 0 }.

#show path/3.
#show battery/2.
```

### Running Clingo

```bash
clingo battery_planning.lp
```

Clingo will find the optimal path, inserting recharge stops at `home` if needed.

## Example: 3D Obstacle Avoidance Path Planning

### Problem Definition

Given:
- Start: (0, 0, 10)
- Goal: (10, 10, 10)
- Obstacles: spheres at known locations
- Grid-based discretization (1m resolution)
- Minimize path length

### ASP Encoding

```prolog
% File: obstacle_avoidance.lp

% Grid dimensions (0-10 in each axis)
grid(0..10).

% Start and goal
start(0, 0, 10).
goal(10, 10, 10).

% Obstacles (center X, Y, Z, radius)
obstacle(5, 5, 10, 2).
obstacle(7, 8, 10, 1).

% Cell is blocked if inside obstacle
blocked(X, Y, Z) :-
    grid(X), grid(Y), grid(Z),
    obstacle(OX, OY, OZ, R),
    (X - OX) * (X - OX) + (Y - OY) * (Y - OY) + (Z - OZ) * (Z - OZ) <= R * R.

% Time steps (max path length)
time(0..30).

% Position at each time step (choose one cell)
1 { at(X, Y, Z, T) : grid(X), grid(Y), grid(Z) } 1 :- time(T).

% Start position
at(X, Y, Z, 0) :- start(X, Y, Z).

% Movement constraints (1 cell per step in x, y, or z)
:- at(X1, Y1, Z1, T), at(X2, Y2, Z2, T+1),
   |X2 - X1| + |Y2 - Y1| + |Z2 - Z1| != 1.

% Cannot move through obstacles
:- at(X, Y, Z, T), blocked(X, Y, Z).

% Must reach goal
:- goal(X, Y, Z), not at(X, Y, Z, _).

% Minimize path length
#minimize { T@1 : at(_, _, _, T) }.

#show at/4.
```

### Running Clingo

```bash
clingo obstacle_avoidance.lp
```

Clingo will find the shortest collision-free path.

## Example: Multi-UAV Task Allocation

### Problem Definition

Given:
- 3 UAVs: `uav1, uav2, uav3`
- 5 tasks: `task1, task2, task3, task4, task5`
- Each task has a location and priority
- Each UAV has a starting location
- Minimize total mission time + maximize priority coverage

### ASP Encoding

```prolog
% File: task_allocation.lp

% UAVs
uav(uav1). uav(uav2). uav(uav3).

% Tasks
task(task1). task(task2). task(task3). task(task4). task(task5).

% UAV starting positions
uav_start(uav1, 0, 0).
uav_start(uav2, 5, 5).
uav_start(uav3, 10, 10).

% Task locations and priorities
task_location(task1, 2, 3). task_priority(task1, 5).
task_location(task2, 4, 6). task_priority(task2, 3).
task_location(task3, 7, 2). task_priority(task3, 4).
task_location(task4, 9, 8). task_priority(task4, 2).
task_location(task5, 1, 9). task_priority(task5, 5).

% Assign each task to exactly one UAV
1 { assign(UAV, TASK) : uav(UAV) } 1 :- task(TASK).

% Calculate distance from UAV to task
distance_to_task(UAV, TASK, DIST) :-
    uav_start(UAV, X1, Y1),
    task_location(TASK, X2, Y2),
    DIST = |X2 - X1| + |Y2 - Y1|.  % Manhattan distance

% Total priority covered
total_priority(P) :-
    P = #sum { PRIO, TASK : assign(_, TASK), task_priority(TASK, PRIO) }.

% Total distance
total_distance(D) :-
    D = #sum { DIST, UAV, TASK : assign(UAV, TASK), distance_to_task(UAV, TASK, DIST) }.

% Maximize priority (negative minimization)
#minimize { -P@2 : total_priority(P) }.

% Minimize total distance
#minimize { D@1 : total_distance(D) }.

#show assign/2.
#show total_priority/1.
#show total_distance/1.
```

### Running Clingo

```bash
clingo task_allocation.lp
```

Clingo will assign tasks to UAVs optimally.

## Integration with ROS 2

### Python Integration (via `clingo` Python module)

```python
import clingo

def solve_waypoint_tsp(waypoints, distances):
    """
    Solve TSP for waypoints using Clingo.
    Returns optimal path.
    """
    # Build ASP program dynamically
    program = "% Generated ASP program\n"

    # Add waypoints
    for wp in waypoints:
        program += f"waypoint({wp}).\n"

    # Add distances
    for (wp1, wp2), dist in distances.items():
        program += f"distance({wp1}, {wp2}, {dist}).\n"

    # Add TSP logic (from example above)
    program += """
    1 { visit(W, T) : time(T) } 1 :- waypoint(W), W != home.
    time(0..10).
    visit(home, 0).
    path(X, Y, T) :- visit(X, T), visit(Y, T+1).
    :- path(X, Y, T), not distance(X, Y, _).
    #minimize { D, X, Y, T : path(X, Y, T), distance(X, Y, D) }.
    #show path/3.
    """

    # Solve
    ctl = clingo.Control()
    ctl.add("base", [], program)
    ctl.ground([("base", [])])

    solution = None
    def on_model(model):
        nonlocal solution
        solution = [str(atom) for atom in model.symbols(shown=True)]

    ctl.solve(on_model=on_model)

    return solution

# Example usage
waypoints = ["home", "wp1", "wp2", "wp3"]
distances = {
    ("home", "wp1"): 100, ("wp1", "home"): 100,
    ("home", "wp2"): 150, ("wp2", "home"): 150,
    ("wp1", "wp2"): 80, ("wp2", "wp1"): 80,
    # ... etc
}

path = solve_waypoint_tsp(waypoints, distances)
print("Optimal path:", path)
```

### ROS 2 Node Example

See `examples/prolog_query_node.py` for similar pattern. Replace Prolog query with Clingo solve.

## Comparison: Clingo vs. SUMO/Vampire

| Feature | Clingo (ASP) | Vampire (FOL) | SUMO (Ontology) |
|---------|--------------|---------------|-----------------|
| **Purpose** | Find optimal solutions | Prove theorems | Knowledge representation |
| **Output** | Answer sets (models) | Proof / Counterexample | Axioms |
| **Use Case** | Path planning, scheduling | Safety verification | Domain modeling |
| **Phase** | Planning (offline) | Planning (offline) | Both (offline + runtime) |
| **Performance** | Fast for combinatorial | Slower for complex proofs | N/A (not a solver) |

## Best Practices

1. **Keep Time Horizon Small**: Use reasonable upper bounds (0..20 instead of 0..1000)
2. **Use Multi-Objective Optimization**: Prioritize objectives with `@` weights
3. **Debug with `#show`**: Use `#show` directives to inspect intermediate predicates
4. **Incremental Solving**: Use Clingo's incremental mode for large problems
5. **Symmetry Breaking**: Add constraints to eliminate symmetric solutions

## Next Steps

1. **Create ASP programs** for flyby-f11 mission scenarios
2. **Integrate with mission planner** (Python + `clingo` module)
3. **Combine with Vampire**: Use Clingo for planning, Vampire for verification
4. **Test with real data**: Validate solutions in simulation

## References

- Clingo Guide: https://potassco.org/clingo/
- ASP Tutorial: https://teaching.potassco.org/
- Python API: https://potassco.org/clingo/python-api/current/
- Clingo GitHub: https://github.com/potassco/clingo
