"""A current observed candidate's task arc at a proposed station.

This is a bounded forecast, never an approach/closure/execution approval.
No new grasp, hidden handle surface or object dynamics enters this check.
"""
import math


def check_arc(articulation,T,moving,base,start,goal,width,solver,poses,collision,joint_margin_rad=.05):
    import numpy as np
    step=math.radians(3) if articulation.joint_type=='revolute' else .01
    reached=None;seed=None;minimum=None;reason=None;count=0
    for coordinate in np.linspace(start,goal,max(1,math.ceil(abs(goal-start)/step))+1):
        D=articulation.delta(float(coordinate-start))
        q=solver.solve(D@T,base,seed,starts=4 if seed is None else 1)
        if q is None:reason='NO_CONTINUOUS_IK_FOR_TESTED_CANDIDATE';break
        margin=solver.original_limits if hasattr(solver,'original_limits') else None
        if margin is not None:
            value=float(np.min(np.minimum(q-margin[:,0],margin[:,1]-q)))
            minimum=value if minimum is None else min(minimum,value)
            if value<=joint_margin_rad:reason='JOINT_MARGIN';break
        P=poses(q,base,width=width);valid,why=collision.check(P,D@moving,True)
        if not valid:reason=why;break
        seed=q;reached=float(coordinate);count+=1
    return dict(tested_current_candidate_only=True,goal_native=goal,reached_native=reached,
        full_arc_supported=reached is not None and abs(reached-goal)<1e-10,
        reason=reason,samples=count,minimum_joint_margin_rad=minimum,
        executable_route=False,requires_fresh_reobservation_regrasp_and_timed_path=True,
        native_geometry_use='collision safety only',hidden_dynamics_used=False)


def forecast(ops,xy,goal):
    import json,numpy as np
    if not ops.current_plan_ready:return dict(available=False,reason='CURRENT_OBSERVED_AUTOMATIC_PLAN_REQUIRED')
    r=ops.r;plan=json.loads((r.output/'visual_plan.json').read_text());a=ops.articulation()
    a.validate_target(dict(target_q=goal,coordinate_unit=a.units['coordinate']))
    from .collision import MobileScene
    scene=MobileScene(r.export,r.model,r.allowed,r.initial_base)
    solver=type(r.solver)(r.c)  # separate task/RNG state; frozen solver costs/limits
    base=r.base.copy();base[:2]=xy
    result=check_arc(a,np.asarray(plan['T']),r.moving().copy(),base,r.actual(),goal,
        float(plan['contact_geometry_audit']['physical_aperture_m']),solver,r.model.poses,scene,
        joint_margin_rad=r.policy.data['joint_margin_rad'])
    result.update(available=True,family=[plan[k] for k in ('section_fraction','insertion_m','roll_deg','pitch_deg')])
    return result
