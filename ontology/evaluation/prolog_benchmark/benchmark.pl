%% ==========================================================================
%% UAV Ontology Prolog Benchmark Harness
%% ==========================================================================
%%
%% Runs timed benchmark queries against uav_rules.pl and outputs JSON results
%% for comparison with Vampire theorem prover benchmarks.
%%
%% Usage: swipl -g run_all_benchmarks -t halt benchmark.pl
%%
%% Author: Finley Holt
%% Date: 2024-12-25
%%
%% ==========================================================================

:- use_module(library(lists)).
:- use_module(library(apply)).
:- use_module(library(statistics)).

%% Load the UAV rules
:- consult('uav_rules.pl').

%% -------------------------------------------------------------------------
%% BENCHMARK INFRASTRUCTURE
%% -------------------------------------------------------------------------

%% Run a single benchmark query with timing
%% run_benchmark(+Name, +Query, +Iterations, -Result)
run_benchmark(Name, Query, Iterations, Result) :-
    garbage_collect,
    get_time(StartWall),
    statistics(cputime, StartCPU),
    run_iterations(Query, Iterations, Solutions),
    statistics(cputime, EndCPU),
    get_time(EndWall),
    WallTime is (EndWall - StartWall) * 1000,  %% Convert to milliseconds
    CPUTime is (EndCPU - StartCPU) * 1000,
    AvgWallTime is WallTime / Iterations,
    AvgCPUTime is CPUTime / Iterations,
    length(Solutions, NumSolutions),
    Result = benchmark_result{
        name: Name,
        iterations: Iterations,
        total_wall_ms: WallTime,
        total_cpu_ms: CPUTime,
        avg_wall_ms: AvgWallTime,
        avg_cpu_ms: AvgCPUTime,
        solutions: NumSolutions,
        success: true
    }.

%% Run iterations of a query and collect solutions
run_iterations(Query, Iterations, Solutions) :-
    run_iterations(Query, Iterations, [], Solutions).

run_iterations(_, 0, Acc, Acc) :- !.
run_iterations(Query, N, Acc, Solutions) :-
    N > 0,
    (   findall(X, call(Query, X), Sols)
    ->  true
    ;   Sols = []
    ),
    N1 is N - 1,
    run_iterations(Query, N1, Sols, Solutions).

%% Run a boolean query (true/false result)
run_benchmark_bool(Name, Query, Iterations, Result) :-
    garbage_collect,
    get_time(StartWall),
    statistics(cputime, StartCPU),
    run_bool_iterations(Query, Iterations, Success),
    statistics(cputime, EndCPU),
    get_time(EndWall),
    WallTime is (EndWall - StartWall) * 1000,
    CPUTime is (EndCPU - StartCPU) * 1000,
    AvgWallTime is WallTime / Iterations,
    AvgCPUTime is CPUTime / Iterations,
    Result = benchmark_result{
        name: Name,
        iterations: Iterations,
        total_wall_ms: WallTime,
        total_cpu_ms: CPUTime,
        avg_wall_ms: AvgWallTime,
        avg_cpu_ms: AvgCPUTime,
        solutions: Success,
        success: true
    }.

run_bool_iterations(_, 0, 1) :- !.
run_bool_iterations(Query, N, Success) :-
    N > 0,
    N1 is N - 1,
    (   call(Query)
    ->  run_bool_iterations(Query, N1, Success)
    ;   Success = 0
    ).

%% -------------------------------------------------------------------------
%% DETAILED STATISTICS COLLECTION
%% -------------------------------------------------------------------------

%% Run benchmark with per-iteration timing for detailed statistics
%% run_benchmark_detailed(+Name, +Category, +Query, +Iterations, -Result)
run_benchmark_detailed(Name, Category, Query, Iterations, Result) :-
    garbage_collect,
    collect_iteration_times(Query, Iterations, Times, NumSolutions),
    compute_statistics(Times, Stats),
    Result = detailed_result{
        name: Name,
        category: Category,
        iterations: Iterations,
        solutions: NumSolutions,
        stats: Stats,
        success: true
    }.

%% Run boolean benchmark with per-iteration timing
run_benchmark_bool_detailed(Name, Category, Query, Iterations, Result) :-
    garbage_collect,
    collect_bool_iteration_times(Query, Iterations, Times, Success),
    compute_statistics(Times, Stats),
    Result = detailed_result{
        name: Name,
        category: Category,
        iterations: Iterations,
        solutions: Success,
        stats: Stats,
        success: true
    }.

%% Collect per-iteration wall-clock times (in microseconds for precision)
collect_iteration_times(Query, Iterations, Times, NumSolutions) :-
    collect_iteration_times_acc(Query, Iterations, [], Times, NumSolutions).

collect_iteration_times_acc(_, 0, Acc, Acc, 0) :- !.
collect_iteration_times_acc(Query, N, Acc, Times, NumSolutions) :-
    N > 0,
    get_time(Start),
    (   findall(X, call(Query, X), Sols) -> true ; Sols = [] ),
    get_time(End),
    TimeUs is (End - Start) * 1000000,  %% Convert to microseconds
    N1 is N - 1,
    length(Sols, NumSols),
    collect_iteration_times_acc(Query, N1, [TimeUs|Acc], Times, NumSolutions),
    (N1 =:= 0 -> NumSolutions = NumSols ; true).

%% Collect per-iteration times for boolean queries
collect_bool_iteration_times(Query, Iterations, Times, Success) :-
    collect_bool_iteration_times_acc(Query, Iterations, [], Times, 1, Success).

collect_bool_iteration_times_acc(_, 0, Acc, Acc, S, S) :- !.
collect_bool_iteration_times_acc(Query, N, Acc, Times, SAcc, Success) :-
    N > 0,
    get_time(Start),
    (   call(Query) -> QuerySuccess = 1 ; QuerySuccess = 0 ),
    get_time(End),
    TimeUs is (End - Start) * 1000000,
    N1 is N - 1,
    SAcc1 is SAcc * QuerySuccess,
    collect_bool_iteration_times_acc(Query, N1, [TimeUs|Acc], Times, SAcc1, Success).

%% Compute detailed statistics from a list of timing values
compute_statistics(Times, Stats) :-
    msort(Times, Sorted),
    length(Sorted, N),
    sum_list(Sorted, Sum),
    Mean is Sum / N,
    min_list(Sorted, Min),
    max_list(Sorted, Max),
    %% Median
    MidIdx is N // 2,
    nth0(MidIdx, Sorted, Median),
    %% P95 and P99
    P95Idx is min(N - 1, round(N * 0.95)),
    P99Idx is min(N - 1, round(N * 0.99)),
    nth0(P95Idx, Sorted, P95),
    nth0(P99Idx, Sorted, P99),
    %% Convert to milliseconds for output
    MinMs is Min / 1000,
    MaxMs is Max / 1000,
    MeanMs is Mean / 1000,
    MedianMs is Median / 1000,
    P95Ms is P95 / 1000,
    P99Ms is P99 / 1000,
    TotalMs is Sum / 1000,
    Stats = timing_stats{
        min_ms: MinMs,
        max_ms: MaxMs,
        mean_ms: MeanMs,
        median_ms: MedianMs,
        p95_ms: P95Ms,
        p99_ms: P99Ms,
        total_ms: TotalMs
    }.

%% -------------------------------------------------------------------------
%% BENCHMARK QUERIES
%% -------------------------------------------------------------------------

%% Query 1: Class hierarchy traversal - find all UAV subclasses
benchmark_subclass_traversal(Class) :-
    subclass_of(Class, uav).

%% Query 2: Instance classification - find all UAVs
benchmark_find_all_uavs(UAV) :-
    instance_of(UAV, uav).

%% Query 3: Capability inference - find all hovering UAVs
benchmark_can_hover(UAV) :-
    can_hover(UAV).

%% Query 4: Capability inference - find all VTOL UAVs
benchmark_can_vtol(UAV) :-
    can_vertical_takeoff(UAV).

%% Query 5: Regulatory compliance - find NDAA compliant UAVs
benchmark_ndaa_compliant(UAV) :-
    ndaa_compliant(UAV).

%% Query 6: Safety check - find UAVs with valid localization
benchmark_valid_localization(UAV) :-
    has_valid_localization(UAV).

%% Query 7: Safety violation - find geofence violations
benchmark_geofence_violation(UAV) :-
    geofence_violation(UAV).

%% Query 8: Safety violation - find NFZ violations
benchmark_nfz_violation(UAV) :-
    nfz_violation(UAV).

%% Query 9: Battery safety - find UAVs requiring return
benchmark_battery_return(UAV) :-
    battery_return_required(UAV).

%% Query 10: Collision detection - find collision imminent
benchmark_collision_imminent(UAV) :-
    collision_imminent(UAV).

%% Query 11: Safe state determination
benchmark_safe_state(UAV) :-
    safe_state(UAV).

%% Query 12: Must land determination
benchmark_must_land(UAV) :-
    must_land(UAV).

%% Query 13: Altitude violation
benchmark_altitude_violation(UAV) :-
    altitude_violation(UAV).

%% Query 14: Mission validity
benchmark_valid_mission(Mission) :-
    is_valid_mission(Mission).

%% Query 15: Combined safety check for specific UAV (boolean)
benchmark_uav_alpha_safe :-
    safe_state(uav_alpha).

%% Query 16: Combined safety check for specific UAV (boolean)
benchmark_uav_beta_safe :-
    safe_state(uav_beta).

%% Query 17: FlybyF11 classification chain
benchmark_flyby_f11_is_aircraft(UAV) :-
    instance_of(UAV, flyby_f11),
    instance_of(UAV, aircraft).

%% Query 18: Deep inheritance query
benchmark_deep_inheritance(UAV) :-
    instance(UAV, flyby_f11),
    subclass_of(flyby_f11, aircraft).

%% -------------------------------------------------------------------------
%% MAIN BENCHMARK RUNNER
%% -------------------------------------------------------------------------

%% Number of iterations for each benchmark
benchmark_iterations(100).

%% Run all benchmarks and output JSON
run_all_benchmarks :-
    benchmark_iterations(Iters),
    format('{"benchmark": "prolog_uav_ontology", "timestamp": "~w", "iterations": ~w, "results": [~n',
           [timestamp, Iters]),

    %% Run each benchmark
    run_benchmark('subclass_traversal', benchmark_subclass_traversal, Iters, R1),
    output_result(R1, true),

    run_benchmark('find_all_uavs', benchmark_find_all_uavs, Iters, R2),
    output_result(R2, true),

    run_benchmark('can_hover', benchmark_can_hover, Iters, R3),
    output_result(R3, true),

    run_benchmark('can_vtol', benchmark_can_vtol, Iters, R4),
    output_result(R4, true),

    run_benchmark('ndaa_compliant', benchmark_ndaa_compliant, Iters, R5),
    output_result(R5, true),

    run_benchmark('valid_localization', benchmark_valid_localization, Iters, R6),
    output_result(R6, true),

    run_benchmark('geofence_violation', benchmark_geofence_violation, Iters, R7),
    output_result(R7, true),

    run_benchmark('nfz_violation', benchmark_nfz_violation, Iters, R8),
    output_result(R8, true),

    run_benchmark('battery_return', benchmark_battery_return, Iters, R9),
    output_result(R9, true),

    run_benchmark('collision_imminent', benchmark_collision_imminent, Iters, R10),
    output_result(R10, true),

    run_benchmark('safe_state', benchmark_safe_state, Iters, R11),
    output_result(R11, true),

    run_benchmark('must_land', benchmark_must_land, Iters, R12),
    output_result(R12, true),

    run_benchmark('altitude_violation', benchmark_altitude_violation, Iters, R13),
    output_result(R13, true),

    run_benchmark('valid_mission', benchmark_valid_mission, Iters, R14),
    output_result(R14, true),

    run_benchmark_bool('uav_alpha_safe', benchmark_uav_alpha_safe, Iters, R15),
    output_result(R15, true),

    run_benchmark_bool('uav_beta_safe', benchmark_uav_beta_safe, Iters, R16),
    output_result(R16, true),

    run_benchmark('flyby_is_aircraft', benchmark_flyby_f11_is_aircraft, Iters, R17),
    output_result(R17, true),

    run_benchmark('deep_inheritance', benchmark_deep_inheritance, Iters, R18),
    output_result(R18, false),

    format(']}~n', []),
    !.

%% Output a single result as JSON
output_result(Result, HasMore) :-
    format('  {"name": "~w", "iterations": ~w, "total_wall_ms": ~3f, "total_cpu_ms": ~3f, "avg_wall_ms": ~6f, "avg_cpu_ms": ~6f, "solutions": ~w, "success": ~w}',
           [Result.name, Result.iterations, Result.total_wall_ms, Result.total_cpu_ms,
            Result.avg_wall_ms, Result.avg_cpu_ms, Result.solutions, Result.success]),
    (HasMore -> format(',~n', []) ; format('~n', [])).

%% -------------------------------------------------------------------------
%% INTERACTIVE TESTING
%% -------------------------------------------------------------------------

%% Quick test to verify rules load correctly
test_rules :-
    format('Testing UAV rules...~n', []),

    format('~n=== Class Hierarchy ===~n', []),
    findall(C, subclass_of(C, uav), Classes),
    format('UAV subclasses: ~w~n', [Classes]),

    format('~n=== UAV Instances ===~n', []),
    findall(U, instance_of(U, uav), UAVs),
    format('All UAVs: ~w~n', [UAVs]),

    format('~n=== Capabilities ===~n', []),
    findall(U, can_hover(U), Hovers),
    format('Can hover: ~w~n', [Hovers]),
    findall(U, can_vertical_takeoff(U), VTOLs),
    format('Can VTOL: ~w~n', [VTOLs]),
    findall(U, ndaa_compliant(U), NDAA),
    format('NDAA compliant: ~w~n', [NDAA]),

    format('~n=== Safety Status ===~n', []),
    findall(U, has_valid_localization(U), Localized),
    format('Valid localization: ~w~n', [Localized]),
    findall(U, safe_state(U), Safe),
    format('Safe state: ~w~n', [Safe]),
    findall(U, battery_return_required(U), LowBatt),
    format('Battery return required: ~w~n', [LowBatt]),
    findall(U, must_land(U), MustLand),
    format('Must land: ~w~n', [MustLand]),

    format('~n=== Violations ===~n', []),
    findall(U, geofence_violation(U), GeoViol),
    format('Geofence violations: ~w~n', [GeoViol]),
    findall(U, nfz_violation(U), NFZViol),
    format('NFZ violations: ~w~n', [NFZViol]),
    findall(U, collision_imminent(U), Collision),
    format('Collision imminent: ~w~n', [Collision]),
    findall(U, altitude_violation(U), AltViol),
    format('Altitude violations: ~w~n', [AltViol]),

    format('~n=== Mission Validity ===~n', []),
    findall(M, is_valid_mission(M), ValidMissions),
    format('Valid missions: ~w~n', [ValidMissions]),

    format('~nTest complete.~n', []).

%% -------------------------------------------------------------------------
%% END OF BENCHMARK HARNESS
%% -------------------------------------------------------------------------
