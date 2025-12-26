%% ==========================================================================
%% Flyby F-11 UAV Domain Ontology - Prolog Translation
%% ==========================================================================
%%
%% Translation of critical safety axioms from uav_domain.kif
%% Translated 20 key axioms for benchmarking Prolog vs Vampire reasoning
%%
%% Author: Finley Holt
%% Date: 2024-12-25
%% Source: ontology/planning_mode/uav_domain.kif
%%
%% ==========================================================================

:- discontiguous subclass/2.
:- discontiguous instance/2.
:- discontiguous can_hover/1.
:- discontiguous can_vertical_takeoff/1.
:- discontiguous ndaa_compliant/1.
:- discontiguous has_valid_localization/1.
:- discontiguous current_battery_level/2.
:- discontiguous battery_reserve_for_return/2.
:- discontiguous minimum_obstacle_distance/2.
:- discontiguous minimum_person_distance/2.
:- discontiguous distance_to/3.
:- discontiguous is_within/2.
:- discontiguous is_outside/2.
:- discontiguous altitude_agl/2.
:- discontiguous maximum_operating_altitude/2.
:- discontiguous has_flight_phase/2.
:- discontiguous assigned_uav/2.
:- discontiguous has_mission_area/2.
:- discontiguous has_position_quality/2.
:- discontiguous overlaps_with_nfz/1.
:- discontiguous has_adequate_battery/2.
:- discontiguous has_required_sensors/2.
:- discontiguous weather_permits/1.

%% -------------------------------------------------------------------------
%% SECTION 1: CLASS HIERARCHY (Axioms 1-7)
%% -------------------------------------------------------------------------
%% Translated from KIF subclass assertions

%% Axiom 1: UAV is a subclass of Aircraft
subclass(uav, aircraft).

%% Axiom 2: Multirotor is a subclass of UAV
subclass(multirotor, uav).

%% Axiom 3: FixedWingUAV is a subclass of UAV
subclass(fixed_wing_uav, uav).

%% Axiom 4: HybridVTOL is a subclass of UAV
subclass(hybrid_vtol, uav).

%% Axiom 5: Quadcopter is a subclass of Multirotor
subclass(quadcopter, multirotor).

%% Axiom 6: Hexacopter is a subclass of Multirotor
subclass(hexacopter, multirotor).

%% Axiom 7: FlybyF11 is a subclass of Quadcopter
subclass(flyby_f11, quadcopter).

%% Transitive closure of subclass hierarchy
%% This computes the transitive closure for inheritance reasoning
subclass_of(X, Y) :- subclass(X, Y).
subclass_of(X, Z) :- subclass(X, Y), subclass_of(Y, Z).

%% Instance membership through class hierarchy
instance_of(X, Class) :- instance(X, Class).
instance_of(X, SuperClass) :-
    instance(X, Class),
    subclass_of(Class, SuperClass).

%% -------------------------------------------------------------------------
%% SECTION 2: CAPABILITY RULES (Axioms 8-11)
%% -------------------------------------------------------------------------
%% Translated from KIF capability implications

%% Axiom 8: All multirotors can hover
%% KIF: (=> (instance ?UAV Multirotor) (canHover ?UAV))
can_hover(UAV) :-
    instance_of(UAV, multirotor).

%% Axiom 9: Multirotors can take off vertically
%% KIF: (=> (or (instance ?UAV Multirotor) (instance ?UAV HybridVTOL))
%%          (canVerticalTakeoff ?UAV))
can_vertical_takeoff(UAV) :-
    instance_of(UAV, multirotor).
can_vertical_takeoff(UAV) :-
    instance_of(UAV, hybrid_vtol).

%% Axiom 10: FlybyF11 is NDAA compliant
%% KIF: (=> (instance ?UAV FlybyF11) (NDAACompliant ?UAV))
ndaa_compliant(UAV) :-
    instance_of(UAV, flyby_f11).

%% Axiom 11: Valid localization from position quality
%% KIF: (=> (and (instance ?UAV UAV)
%%              (or (hasPositionQuality ?UAV HighQuality)
%%                  (hasPositionQuality ?UAV MediumQuality)
%%                  (hasPositionQuality ?UAV LowQuality)))
%%          (hasValidLocalization ?UAV))
has_valid_localization(UAV) :-
    instance_of(UAV, uav),
    has_position_quality(UAV, Quality),
    member(Quality, [high_quality, medium_quality, low_quality]).

%% -------------------------------------------------------------------------
%% SECTION 3: SAFETY RULES (Axioms 12-20)
%% -------------------------------------------------------------------------
%% Critical safety axioms that must never be violated

%% Axiom 12: Geofence violation detection
%% KIF: (=> (and (instance ?UAV UAV)
%%              (instance ?MISSION UAVMission)
%%              (assignedUAV ?MISSION ?UAV)
%%              (hasMissionArea ?MISSION ?GEOFENCE)
%%              (instance ?GEOFENCE Geofence)
%%              (isOutside ?UAV ?GEOFENCE))
%%          (geofenceViolation ?UAV))
geofence_violation(UAV) :-
    instance_of(UAV, uav),
    assigned_uav(Mission, UAV),
    has_mission_area(Mission, Geofence),
    instance(Geofence, geofence),
    is_outside(UAV, Geofence).

%% Axiom 13: No-fly zone violation detection
%% KIF: (=> (and (instance ?UAV UAV)
%%              (instance ?NFZ NoFlyZone)
%%              (isWithin ?UAV ?NFZ))
%%          (noFlyZoneViolation ?UAV))
nfz_violation(UAV) :-
    instance_of(UAV, uav),
    instance(NFZ, no_fly_zone),
    is_within(UAV, NFZ).

%% Axiom 14: Battery return requirement
%% KIF: (=> (and (CurrentBatteryLevel ?UAV ?CURRENT)
%%              (BatteryReserveForReturn ?UAV ?RESERVE)
%%              (lessThan ?CURRENT ?RESERVE))
%%          (mustReturnToLaunch ?UAV))
battery_return_required(UAV) :-
    current_battery_level(UAV, Current),
    battery_reserve_for_return(UAV, Reserve),
    Current < Reserve.

%% Synonym for compatibility
must_return_to_launch(UAV) :- battery_return_required(UAV).

%% Axiom 15: Collision imminent detection
%% KIF: (=> (and (instance ?UAV UAV)
%%              (instance ?OBS Obstacle)
%%              (distanceTo ?UAV ?OBS ?DIST)
%%              (MinimumObstacleDistance ?UAV ?MIN_DIST)
%%              (lessThan ?DIST ?MIN_DIST))
%%          (not (safeState ?UAV)))
collision_imminent(UAV) :-
    instance_of(UAV, uav),
    instance(Obstacle, obstacle),
    distance_to(UAV, Obstacle, Distance),
    minimum_obstacle_distance(UAV, MinDistance),
    Distance < MinDistance.

%% Axiom 16: Safe state determination
%% A UAV is in safe state if no violations are active
%% (Negation as failure semantics differ from classical logic here)
safe_state(UAV) :-
    instance_of(UAV, uav),
    \+ collision_imminent(UAV),
    \+ nfz_violation(UAV),
    \+ geofence_violation(UAV),
    \+ altitude_violation(UAV).

%% Axiom 17: Altitude violation detection
%% KIF: (=> (and (instance ?UAV UAV)
%%              (instance ?MISSION UAVMission)
%%              (assignedUAV ?MISSION ?UAV)
%%              (MaximumOperatingAltitude ?MISSION ?MAX_ALT)
%%              (altitudeAGL ?UAV ?CURRENT_ALT)
%%              (greaterThan ?CURRENT_ALT ?MAX_ALT))
%%          (altitudeViolation ?UAV))
altitude_violation(UAV) :-
    instance_of(UAV, uav),
    assigned_uav(Mission, UAV),
    maximum_operating_altitude(Mission, MaxAlt),
    altitude_agl(UAV, CurrentAlt),
    CurrentAlt > MaxAlt.

%% Axiom 18: Must land on localization loss
%% KIF: (=> (and (instance ?UAV UAV)
%%              (not (hasValidLocalization ?UAV)))
%%          (mustLand ?UAV))
must_land(UAV) :-
    instance_of(UAV, uav),
    \+ has_valid_localization(UAV).

%% Axiom 19: Mission area validity
%% KIF: (=> (and (instance ?MISSION UAVMission)
%%              (assignedUAV ?MISSION ?UAV)
%%              (hasMissionArea ?MISSION ?AREA)
%%              (instance ?AREA Geofence)
%%              (not (overlapsWithNoFlyZone ?AREA)))
%%          (hasMissionAreaValid ?MISSION))
has_mission_area_valid(Mission) :-
    instance(Mission, uav_mission),
    assigned_uav(Mission, _UAV),
    has_mission_area(Mission, Area),
    instance(Area, geofence),
    \+ overlaps_with_nfz(Area).

%% Axiom 20: Complete mission validity
%% KIF: (=> (and (instance ?MISSION UAVMission)
%%              (assignedUAV ?MISSION ?UAV)
%%              (instance ?UAV UAV)
%%              (hasMissionAreaValid ?MISSION)
%%              (hasAdequateBattery ?UAV ?MISSION)
%%              (hasRequiredSensors ?UAV ?MISSION)
%%              (weatherPermits ?MISSION))
%%          (isValidMission ?MISSION))
is_valid_mission(Mission) :-
    instance(Mission, uav_mission),
    assigned_uav(Mission, UAV),
    instance_of(UAV, uav),
    has_mission_area_valid(Mission),
    has_adequate_battery(UAV, Mission),
    has_required_sensors(UAV, Mission),
    weather_permits(Mission).

%% -------------------------------------------------------------------------
%% SECTION 4: FLYBY F-11 SPECIFIC DEFAULTS (Supplementary axioms)
%% -------------------------------------------------------------------------
%% Default safety parameters for FlybyF11 instances

%% Default battery constraints for FlybyF11
%% KIF: (=> (instance ?UAV FlybyF11)
%%          (and (MinimumBatteryLevel ?UAV 20.0)
%%               (BatteryReserveForReturn ?UAV 25.0)))
battery_reserve_for_return(UAV, 25.0) :-
    instance_of(UAV, flyby_f11),
    \+ battery_reserve_for_return_override(UAV, _).

%% Default safety distances for FlybyF11
%% KIF: (=> (instance ?UAV FlybyF11)
%%          (and (MinimumObstacleDistance ?UAV (MeasureFn 3.0 Meter))
%%               (MinimumPersonDistance ?UAV (MeasureFn 15.0 Meter))))
minimum_obstacle_distance(UAV, 3.0) :-
    instance_of(UAV, flyby_f11),
    \+ minimum_obstacle_distance_override(UAV, _).

minimum_person_distance(UAV, 15.0) :-
    instance_of(UAV, flyby_f11),
    \+ minimum_person_distance_override(UAV, _).

%% Placeholder for overrides (empty by default)
:- dynamic battery_reserve_for_return_override/2.
:- dynamic minimum_obstacle_distance_override/2.
:- dynamic minimum_person_distance_override/2.

%% -------------------------------------------------------------------------
%% SECTION 5: TEST FACTS FOR BENCHMARKING
%% -------------------------------------------------------------------------
%% Sample instances and state for running benchmark queries

%% UAV instances
instance(uav_alpha, flyby_f11).
instance(uav_beta, flyby_f11).
instance(uav_gamma, quadcopter).
instance(uav_delta, hexacopter).
instance(uav_epsilon, fixed_wing_uav).

%% Spatial regions
instance(geofence_area_1, geofence).
instance(geofence_area_2, geofence).
instance(nfz_airport, no_fly_zone).
instance(nfz_government, no_fly_zone).

%% Obstacles
instance(building_1, obstacle).
instance(tree_1, obstacle).
instance(person_1, obstacle).

%% Missions
instance(mission_1, uav_mission).
instance(mission_2, uav_mission).
instance(mission_3, uav_mission).

%% Mission assignments
assigned_uav(mission_1, uav_alpha).
assigned_uav(mission_2, uav_beta).
assigned_uav(mission_3, uav_gamma).

%% Mission areas
has_mission_area(mission_1, geofence_area_1).
has_mission_area(mission_2, geofence_area_2).
has_mission_area(mission_3, geofence_area_1).

%% Position quality for localization tests
has_position_quality(uav_alpha, high_quality).
has_position_quality(uav_beta, medium_quality).
has_position_quality(uav_gamma, low_quality).
%% uav_delta has no position quality (simulating localization loss)
has_position_quality(uav_epsilon, high_quality).

%% Battery levels for battery tests
current_battery_level(uav_alpha, 80.0).
current_battery_level(uav_beta, 20.0).    %% Below reserve threshold
current_battery_level(uav_gamma, 50.0).
current_battery_level(uav_delta, 15.0).   %% Critical
current_battery_level(uav_epsilon, 90.0).

%% Battery reserve for non-FlybyF11 (manual specification)
battery_reserve_for_return(uav_gamma, 25.0).
battery_reserve_for_return(uav_delta, 25.0).
battery_reserve_for_return(uav_epsilon, 30.0).

%% Altitude data
altitude_agl(uav_alpha, 50.0).
altitude_agl(uav_beta, 150.0).   %% Above typical max
altitude_agl(uav_gamma, 100.0).
altitude_agl(uav_delta, 30.0).
altitude_agl(uav_epsilon, 200.0).

%% Mission altitude limits
maximum_operating_altitude(mission_1, 120.0).
maximum_operating_altitude(mission_2, 120.0).
maximum_operating_altitude(mission_3, 80.0).

%% Distance to obstacles for collision detection
distance_to(uav_alpha, building_1, 50.0).   %% Safe
distance_to(uav_beta, building_1, 10.0).    %% Safe
distance_to(uav_gamma, tree_1, 2.0).        %% Collision imminent (< 3m for F11, but gamma is quadcopter)
distance_to(uav_delta, person_1, 5.0).      %% Need to check
distance_to(uav_epsilon, building_1, 100.0). %% Very safe

%% Minimum obstacle distance for non-default UAVs
minimum_obstacle_distance(uav_gamma, 3.0).
minimum_obstacle_distance(uav_delta, 3.0).
minimum_obstacle_distance(uav_epsilon, 5.0).

%% Spatial containment for geofence/NFZ tests
is_within(uav_alpha, geofence_area_1).
is_within(uav_beta, geofence_area_2).
is_within(uav_gamma, geofence_area_1).
is_outside(uav_delta, geofence_area_1).  %% Geofence violation scenario

%% NFZ containment tests
is_within(uav_epsilon, nfz_airport).  %% NFZ violation scenario

%% Mission validity prerequisites
has_adequate_battery(uav_alpha, mission_1).
has_adequate_battery(uav_gamma, mission_3).
has_required_sensors(uav_alpha, mission_1).
has_required_sensors(uav_gamma, mission_3).
weather_permits(mission_1).
weather_permits(mission_3).
%% mission_2 missing weather_permits (invalid mission scenario)

%% -------------------------------------------------------------------------
%% SECTION 6: UTILITY PREDICATES
%% -------------------------------------------------------------------------
%% Helper predicates for queries and debugging

%% List all UAV instances
all_uavs(UAVs) :-
    findall(UAV, instance_of(UAV, uav), UAVs).

%% List all safety violations for a UAV
safety_violations(UAV, Violations) :-
    findall(Violation, (
        (collision_imminent(UAV) -> Violation = collision_imminent ; fail) ;
        (nfz_violation(UAV) -> Violation = nfz_violation ; fail) ;
        (geofence_violation(UAV) -> Violation = geofence_violation ; fail) ;
        (altitude_violation(UAV) -> Violation = altitude_violation ; fail) ;
        (battery_return_required(UAV) -> Violation = low_battery ; fail) ;
        (must_land(UAV) -> Violation = must_land ; fail)
    ), Violations).

%% Check all safety conditions for a UAV
safety_check(UAV, Status) :-
    (safe_state(UAV) -> Status = safe ; Status = unsafe).

%% -------------------------------------------------------------------------
%% END OF UAV RULES
%% -------------------------------------------------------------------------
