"""Measured phase costs, not a blanket replay of a previous long action cost."""
import time


def record(ops,phase,started,travel=None):
    costs=getattr(ops,'_v81_phase_costs',{})
    elapsed=time.perf_counter()-started
    costs[phase]=max(costs.get(phase,0.),elapsed)
    if travel is not None and abs(travel)>1e-6:
        costs['healthy_motion_wall_s_per_native']=max(costs.get('healthy_motion_wall_s_per_native',0.),elapsed/abs(travel))
    ops._v81_phase_costs=costs


def admission(ops,actions):
    from .recovery_batch import measured_costs
    measured=measured_costs(ops)['observed_max_operation_wall_s']
    costs=getattr(ops,'_v81_phase_costs',{})
    remaining=ops.r.episode_budget.remaining(ops.r.tick*ops.r.dt)
    if not all(k in costs for k in ('acquire','release','healthy_motion_wall_s_per_native')):
        return dict(admitted=True,cost_unknown=True,remaining_work=remaining,
            exit_reserve_already_excluded=True,measured_phase_costs=costs)
    q=ops.r.actual();travel=0.
    for action in actions or []:
        travel+=abs(action['target_q']-q);q=action['target_q']
    motion=travel*costs['healthy_motion_wall_s_per_native']
    current=costs['acquire']+costs['release']+motion
    recovery=costs['acquire']+costs['release']+measured.get('prepare_grasp',0.)+measured.get('reobserve_and_replan',0.)+60.
    need=current if ops.r.job.get('v81_active_recovery') else current+recovery
    return dict(admitted=need<=remaining['wall_s'],cost_unknown=False,
        additional_recovery_reserve_available=current+recovery<=remaining['wall_s'],
        current_declared_motion_estimate_wall_s=motion,current_attempt_estimate_wall_s=current,
        recovery_reserve_wall_s=recovery,remaining_work=remaining,
        measured_phase_costs=costs,exit_reserve_already_excluded=True,
        estimate_not_a_physical_feasibility_guarantee=True)
