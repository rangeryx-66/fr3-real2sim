"""Original exit first; fully checked joint return when already unloaded/free.

A camera observation pose need not admit a Cartesian backoff. This fallback
never waives contact or accepts local retreat as parking. Original executor
revalidates the route and original measured-state verify_park owns completion.
"""
import time
import numpy as np


def plan_unloaded_exit(g,original):
    result=original(g)
    if result['valid']:return result
    from native_g1_safe_park import PARK_DEFINITION,parking_targets,route_to_joint
    from native_g1_contact_recovery import checked_route
    if (np.max(g['filtered']())>PARK_DEFINITION['unloaded_pad_n']
            or not g['force_guard'].settled()):return result
    began=time.perf_counter()
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']].copy()
    command=np.asarray(g['qtarget'])[g['arm']].copy()
    bridge=checked_route(g,[measured,command])
    audit=dict(verified_unloaded=True,contact_waiver=False,command_bridge=bridge,candidates=[])
    result['unloaded_joint_return']=audit
    if not bridge['valid']:return result
    for region,target in parking_targets(g):
        route,reason=route_to_joint(g,measured,target)
        row=dict(region=region,reason=reason,valid=False);audit['candidates'].append(row)
        if route is None:continue
        check=checked_route(g,[command.tolist()]+route[1:]);row['check']=check
        if not check['valid']:continue
        row['valid']=True
        return dict(result,valid=True,reason=None,
            clearance_path=[measured.tolist(),command.tolist()],parking_path=route,
            parking_q=np.asarray(target).tolist(),parking_region=region,
            route_method='ALREADY_UNLOADED_FREE_JOINT_RETURN:'+str(reason),
            whole_route_audit=check,measured_start_q=measured.tolist(),
            moving_pose_at_plan=np.asarray(g['moving']()).tolist(),
            planning_wall_s=result.get('planning_wall_s',0)+time.perf_counter()-began,
            clearance_semantics='original-predicate-free measured/command bridge; no mandatory Cartesian backoff')
    return result
