"""Bounded clearance + parking, isolated from frozen manipulation sources.

All edges use the existing wrist-route planner and collision predicates.
Parking is a terminal task outcome, never an extra grasp/opening admission gate.
"""
import time
from types import SimpleNamespace
import numpy as np
from native_g1_wrist_views import path_to
from native_g1_contact_recovery import checked_route


PARK_DEFINITION = dict(
    regions='native robot rest configuration, then rest with shoulder yaw +/-20deg',
    measured_joint_cell_rad=.025,  # same discretization as the existing route checker
    unloaded_pad_n=.05,  # unchanged release criterion
    stationary_hold_s=.3,  # unchanged exit settling duration
    existing_joint_margin_rad=.05,
    collision='current measured door and all measured released finger joints; no target waiver',
    completion='executed full route into selected rest cell, settled unloaded, measured-state check',
    local_clearance_is_not_parking=True)


def parking_targets(g):
    home=np.asarray(g['model'].home).copy()
    yield 'native_rest',home
    for yaw in [-20.,20.]:
        q=home.copy();q[0]+=np.deg2rad(yaw)
        yield 'shoulder_rest_%+d'%yaw,q


def route_to_joint(g,start,end):
    """Use the frozen DIRECT/RRT implementation with an exact joint endpoint.

    These small interface adapters supply a joint goal (rather than solving a
    camera IK goal), current moving-body pose and measured finger articulation.
    The existing sampling, .05 limit, seed, step length and budget are intact.
    """
    fingers=np.asarray(g['robot'].get_joint_positions())[g['fingers']].copy()
    moving=np.asarray(g['moving']()).copy();model=g['model']
    m=SimpleNamespace(margin=model.margin,limits=model.limits,
        poses=lambda q,base,width: model.poses(q,base,finger_q=fingers))
    scene=SimpleNamespace(check=g['collision'].check,moving_reference=moving)
    solver=SimpleNamespace(solve=lambda *args,**kwargs:np.asarray(end).copy())
    c=dict(g.get('wrist_config',{}));c.setdefault('ik_starts',4);c.setdefault('route_iterations',600)
    return path_to(m,solver,scene,g['base'],np.asarray(start),np.eye(4),c,np.eye(4))


def plan_exit(g):
    """Do not execute the first clearance until its onward route is checked."""
    began=time.perf_counter();T=np.asarray(g['tcp']()).copy()
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']].copy()
    command=np.asarray(g.get('qtarget',measured))[g['arm']].copy()
    bridge=checked_route(g,[measured,command])
    result=dict(valid=False,definition=PARK_DEFINITION,command_bridge=bridge,candidates=[])
    if not bridge['valid']:return dict(result,reason='CURRENT_COMMAND_BRIDGE:'+bridge['reason'])
    # Pose measurements may differ from command equilibrium. Connect them
    # explicitly; execution retains equilibrium rather than resetting targets.
    directions=[('tool_backoff',-T[:3,2]),('observed_panel_outward',np.asarray(g['normal'])),
                ('outward_up',np.asarray(g['normal'])+np.array([0.,0.,1.])),
                ('tool_backoff_up',-T[:3,2]+np.array([0.,0.,1.]))]
    targets=list(parking_targets(g))
    for distance in [.04,.08]:
        for name,direction in directions:
            direction=np.asarray(direction,float);length=np.linalg.norm(direction)
            if length<1e-9:continue
            direction/=length;path=[measured.tolist()];seed=measured.copy();reason=None
            for d in np.linspace(0,distance,int(round(distance/.002))+1)[1:]:
                goal=T.copy();goal[:3,3]+=d*direction;seed=g['ik'](goal,seed)
                if seed is None:reason='NO_RETREAT_IK';break
                path.append(seed.tolist())
            audit=checked_route(g,[command.tolist()]+path[1:]) if reason is None else dict(valid=False,reason=reason)
            if audit['valid']:audit=checked_route(g,path)
            record=dict(kind=name,distance_m=distance,direction=direction.tolist(),clearance=audit,parking=[])
            result['candidates'].append(record)
            if not audit['valid']:continue
            for region,qpark in targets:
                onward,how=route_to_joint(g,path[-1],qpark)
                record['parking'].append(dict(region=region,reason=how,valid=onward is not None))
                if onward is None:continue
                full=checked_route(g,[command.tolist()]+path[1:]+onward[1:])
                if not full['valid']:
                    record['parking'][-1]['reason']=full['reason'];continue
                result.update(valid=True,clearance_path=path,parking_path=onward,parking_q=qpark.tolist(),
                    parking_region=region,route_method=how,whole_route_audit=full,
                    measured_start_q=measured.tolist(),measured_start_fingers_rad=np.asarray(g['robot'].get_joint_positions())[g['fingers']].tolist(),
                    moving_pose_at_plan=np.asarray(g['moving']()).tolist(),planning_wall_s=time.perf_counter()-began)
                return result
    return dict(result,reason='SAFE_EXIT_UNAVAILABLE: bounded clearance/rest routes exhausted',planning_wall_s=time.perf_counter()-began)


def verify_park(g,plan):
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    error=float(np.max(abs(measured-np.asarray(plan['parking_q']))))
    check=checked_route(g,[measured,measured])
    unloaded=bool(np.all(g['filtered']()<=PARK_DEFINITION['unloaded_pad_n']) and g['force_guard'].settled())
    return dict(reached=bool(check['valid'] and unloaded and error<=PARK_DEFINITION['measured_joint_cell_rad']),
        region=plan['parking_region'],measured_q=measured.tolist(),command_q=g['qtarget'][g['arm']].tolist(),
        joint_cell_error_rad=error,measured_collision_check=check,unloaded=unloaded,
        pad_loads_n=g['filtered']().tolist(),moving_pose=np.asarray(g['moving']()).tolist(),
        door_angle_deg=float(np.rad2deg(g['actual']())))
