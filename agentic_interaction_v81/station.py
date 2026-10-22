"""Offline station-route contract, not an enabled base controller.

Callbacks must check the entire native robot/support against the current object
and scene. An arm-only IK result cannot certify base/pedestal clearance.
The Agent supplies the desired station; this module never selects a view/pose.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from typing import Callable


@dataclass(frozen=True)
class Boundary:
    episode_id: str
    station_generation: int
    observation_generation: int
    simulation_timestamp_s: float
    base_xyzyaw: tuple[float, float, float, float]
    arm_q: tuple[float, ...]
    finger_q: tuple[float, ...]
    object_coordinate: float
    object_coordinate_unit: str
    robot_stationary: bool
    object_stationary: bool
    released: bool
    unloaded_verified: bool
    clearance_verified: bool
    safety_latch: str | None
    native_scene_sha256: str

    def __post_init__(self):
        # Native state arrays are commonly float32. Commit the measured values
        # as JSON-native scalars, with no rounding or precision claim.
        for name in ('base_xyzyaw','arm_q','finger_q'):
            object.__setattr__(self,name,tuple(float(x) for x in getattr(self,name)))
        for name in ('simulation_timestamp_s','object_coordinate'):
            object.__setattr__(self,name,float(getattr(self,name)))

    def check(self):
        if self.safety_latch is not None:
            raise ValueError('HARD_STOP_AUTHORITATIVE')
        conditions={name:getattr(self,name) for name in (
            'robot_stationary','object_stationary','released','unloaded_verified','clearance_verified')}
        if not all(conditions.values()):
            raise ValueError('VERIFIED_STOPPED_UNLOADED_CLEARANCE_REQUIRED:'+','.join(
                name for name,valid in conditions.items() if not valid))
        values=(*self.base_xyzyaw, *self.arm_q, *self.finger_q,
                self.object_coordinate, self.simulation_timestamp_s)
        if not all(math.isfinite(x) for x in values):
            raise ValueError('FINITE_MEASURED_STATE_REQUIRED')
        if len(self.base_xyzyaw) != 4 or len(self.arm_q) != 7 or not self.finger_q:
            raise ValueError('MEASURED_NATIVE_STATE_SHAPES_REQUIRED')
        if not self.episode_id or not self.native_scene_sha256 or self.object_coordinate_unit not in ('rad','m'):
            raise ValueError('EPISODE_NATIVE_SCENE_AND_UNITS_REQUIRED')
        if self.station_generation < 0 or self.observation_generation < 0:
            raise ValueError('NONNEGATIVE_GENERATIONS_REQUIRED')

    def commitment(self):
        self.check()
        return hashlib.sha256(json.dumps(asdict(self),sort_keys=True,allow_nan=False).encode()).hexdigest()


def smooth_translation(start, goal, speed, acceleration, sample_s):
    """Cubic rest-to-rest XY path with bounded speed/acceleration.

    Limits must come from the installed FREE policy and native carriage caps;
    this helper cannot raise them. No rotation/height authority is assumed.
    """
    if len(start) != 4 or len(goal) != 4:
        raise ValueError('BASE_XYZ_YAW_REQUIRED')
    if not all(math.isfinite(x) for x in (*start,*goal,speed,acceleration,sample_s)):
        raise ValueError('FINITE_PATH_INPUTS_REQUIRED')
    if min(speed,acceleration,sample_s) <= 0:
        raise ValueError('POSITIVE_POLICY_AND_SAMPLING_REQUIRED')
    if tuple(start[2:]) != tuple(goal[2:]):
        raise ValueError('XY_STAGE_CANNOT_CHANGE_HEIGHT_OR_YAW')
    delta=(goal[0]-start[0],goal[1]-start[1]);length=math.hypot(*delta)
    if length == 0:
        return [dict(t=0.0, base=list(start), velocity_xy=[0.0,0.0])]
    # s=3u^2-2u^3: max ds/dt=1.5/T, max |d2s/dt2|=6/T^2.
    duration=max(1.5*length/speed,math.sqrt(6*length/acceleration),sample_s)
    n=math.ceil(duration/sample_s)
    result=[]
    for i in range(n+1):
        u=i/n;s=u*u*(3-2*u);v=6*u*(1-u)/duration
        result.append(dict(t=duration*u,
            base=[start[0]+s*delta[0],start[1]+s*delta[1],*start[2:]],
            velocity_xy=[v*d for d in delta]))
    return result


def evaluate_station(boundary: Boundary, target, *, speed, acceleration,
                     collision_interval_s, validate_swept_route: Callable,
                     validate_destination: Callable):
    """Compare bounded direct/dogleg alternatives before any physical motion.

    Both callbacks receive actual released fingers, articulated state and base.
    Swept-route validation must include between-sample clearance/stopping distance.
    Destination validation includes legal unloaded parking and observation/IK
    opportunity. Their results are safety evidence, not a GT grasp generator.
    """
    boundary.check()
    start=boundary.base_xyzyaw;target=tuple(target)
    # Validate unsupported/nonfinite requests before asking a checker to use them.
    direct=smooth_translation(start,target,speed,acceleration,collision_interval_s)
    routes=[direct]
    for midpoint in ((target[0],start[1],*start[2:]),
                     (start[0],target[1],*start[2:])):
        first=smooth_translation(start,midpoint,speed,acceleration,collision_interval_s)
        second=smooth_translation(midpoint,target,speed,acceleration,collision_interval_s)
        offset=first[-1]['t']
        route=first+[dict(p,t=p['t']+offset) for p in second[1:]]
        if [p['base'] for p in route] != [p['base'] for p in direct]:
            routes.append(route)
    alternatives=[]
    for route in routes:
        swept=validate_swept_route(boundary,route)
        destination=validate_destination(boundary,target) if swept.get('valid') else None
        valid=bool(swept.get('valid') and destination and destination.get('valid'))
        alternatives.append(dict(valid=valid,path=route,swept=swept,
                                 destination=destination,duration_s=route[-1]['t']))
    viable=[a for a in alternatives if a['valid']]
    selected=min(viable,key=lambda a:a['duration_s']) if viable else None
    return dict(valid=selected is not None,status='OFFLINE_ROUTE_CANDIDATE' if selected else 'SAFE_RELOCATION_UNAVAILABLE',
        boundary_sha256=boundary.commitment(),target=list(target),selected=selected,
        alternatives=alternatives,physical_motion_executed=False,
        execution_requirements=['Verified native driven XY mount and measured base feedback',
            'Revalidate full route from current measured state immediately before motion',
            'Original arm/G1/force/joint/collision protection active at every physics step',
            'After motion: settle, measure base/camera FK, update all planner base frames',
            'Invalidate old view routes/grasp/approach and acquire fresh wrist RGB-D'])


def invalidate_after_relocation(previous: Boundary, measured: Boundary):
    """Require same-scene lineage; never transform a cached grasp into approval."""
    previous.check();measured.check()
    if (previous.episode_id != measured.episode_id or
            previous.object_coordinate_unit != measured.object_coordinate_unit or
            measured.simulation_timestamp_s <= previous.simulation_timestamp_s or
            measured.station_generation != previous.station_generation + 1 or
            measured.observation_generation <= previous.observation_generation):
        raise ValueError('MEASURED_SAME_EPISODE_NEW_STATION_LINEAGE_REQUIRED')
    return dict(station_generation=measured.station_generation,
                observation_generation=measured.observation_generation,
                reobserve_required=True,automatic_replan_required=True,
                invalidated=['view_routes','visual_plan','approach','closing_path','execution_approval'],
                object_coordinate_before=previous.object_coordinate,
                object_coordinate_after=measured.object_coordinate,
                no_object_reset=True,physical_base_execution_verified=False)
