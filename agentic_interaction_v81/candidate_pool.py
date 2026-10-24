"""Cheap replanning from this generation's unexecuted automatic candidates.

Never a saved successful grasp, score change, object-pose correction or waiver.
"""
from copy import deepcopy
import json
from agent_command_protocol_v12 import atomic_json


def identity(ops):
    from .recovery_search import observation_identity
    return dict(observation_identity(ops),measured_object_q=float(ops.r.actual()),
        requested_preload=float(ops.r.closure_policy['preload_n']))


def keep(ops,candidates,horizon):
    ops._v81_current_candidates=dict(identity=identity(ops),horizon=horizon,
        candidates=deepcopy(candidates),historical_or_executed_pose=False)
    atomic_json(ops.r.output/'CURRENT_AUTOMATIC_CANDIDATE_POOL.json',ops._v81_current_candidates)


def select(ops,command):
    from .boundary_operations import wrist_evidence
    from .replanning import coverage,targets
    pool=getattr(ops,'_v81_current_candidates',None)
    if (not pool or pool['identity']!=identity(ops) or not wrist_evidence(ops,command)
            or not command.get('reason')):
        return dict(plan=False,recoverable=True,reason='CURRENT_UNEXECUTED_OBSERVATION_POOL_REQUIRED')
    actions=command['planned_segments'];required=targets(actions);audit=[]
    r=ops.r;prismatic=r.job['articulation_v7']['joint_type']=='prismatic'
    if prismatic:
        from .preload_extension import select as preload
        preload(ops,command)
        from interaction_runtime.prismatic_candidate_v75 import PrismaticCandidateCheckV75 as Checker
    else:
        from native_g1_execution_candidates import CandidateCheck as Checker
    checker=Checker(ops.g,r.policy,r.c,r.solver)
    # Keep the unchanged utility for the ORIGINAL far goal. A shorter approved
    # intermediate target does not erase or rescore away that original goal.
    candidates=sorted(pool['candidates'],key=lambda c:c['shared_selection_utility'],reverse=True)
    for candidate in candidates:
        bounds=coverage(candidate,required,pool['horizon'])
        first_checked=coverage(candidate,[actions[0]['target_q']])['full_requested_actions_feasible']
        if not bounds['full_requested_actions_feasible'] and not prismatic and not first_checked:
            audit.append(dict(valid=False,reason='REQUESTED_TARGET_NOT_FULLY_CHECKED',coverage=bounds));continue
        check=checker(candidate)
        audit.append(dict(family=[candidate[k]for k in ('section_fraction','insertion_m','roll_deg','pitch_deg')],check=check))
        if not check.get('valid'):continue
        candidate=deepcopy(candidate);candidate['execution_feasibility']=check
        if prismatic:
            # Generated candidates reach this seam before the frozen owner's
            # final serialization adds its SI coordinate label. Preserve the
            # current whole-chain certificate on a same-pool replan too.
            candidate['coordinate_unit']='m'
        bounds=coverage(candidate,required,pool['horizon'])
        if not bounds['full_requested_actions_feasible']:continue
        atomic_json(r.output/'visual_plan.json',candidate)
        atomic_json(r.output/'SAME_GENERATION_REPLAN_AUDIT.json',dict(candidates=audit,
            pool_identity=pool['identity'],original_global_planning_horizon=pool['horizon'],
            native_checks_not_skipped=True,scores_unchanged=True,physical_motion_executed=False))
        ops.current_plan_ready=True
        return dict(plan=True,geometry=True,reused_only_current_unexecuted_automatic_candidates=True,
            target_coverage=bounds,candidate_family=[candidate[k]for k in
            ('section_fraction','insertion_m','roll_deg','pitch_deg')],
            original_global_planning_horizon=pool['horizon'],checks=len(audit),physical_motion_executed=False)
    atomic_json(r.output/'SAME_GENERATION_REPLAN_AUDIT.json',dict(candidates=audit,pool_identity=pool['identity']))
    return dict(plan=False,recoverable=True,reason='CURRENT_POOL_NATIVE_SEARCH_EXHAUSTED',
        global_infeasibility_proven=False,candidates_checked=len(audit))
