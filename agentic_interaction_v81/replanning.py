"""Bounded same-episode planning recovery; never a collision override.

Geometric rejection and a physical hard latch are different events. No action
target is silently truncated, and no rejected path is sent to the actuators.
"""
from contextlib import contextmanager
from copy import deepcopy
import math
from unittest.mock import patch


def coverage(plan, targets, horizon=None):
    states=[float(p['state']) for p in plan.get('path',[])]
    valid=bool(states) and all(math.isfinite(v) for v in states)
    low,high=(min(states),max(states)) if valid else (None,None)
    # Floating-point comparison only, not a relaxed physical target tolerance.
    # The frozen prismatic owner checks an entire bidirectional chain, which
    # can cross the acquisition coordinate. Its first nominal arc alone is
    # not the whole safety certificate. Keep that established adapter intact.
    chain=(plan.get('execution_feasibility') or {}).get('v75_whole_chain') or {}
    certified=[]
    if plan.get('coordinate_unit')=='m' and chain.get('valid') is True:
        certified=[float(a['target_m']) for a in chain.get('declared_actions',[])
                   if type(a.get('target_m')) in (int,float) and math.isfinite(a['target_m'])]
    revolute=(plan.get('execution_feasibility') or {}).get('v81_revolute_chain') or {}
    if plan.get('coordinate_unit')!='m' and revolute.get('valid') is True:
        certified=[float(a['target_rad']) for a in revolute.get('endpoints',[])
                   if type(a.get('target_rad')) in (int,float) and math.isfinite(a['target_rad'])
                   and (a.get('exit_valid') is True or
                        revolute.get('staged_exit') is True and
                        a.get('motion_valid') is True and a.get('stopped_hold_checked') is True)]
    def includes(q):
        return ((valid and low-1e-10<=float(q)<=high+1e-10)
                or any(abs(float(q)-v)<=1e-10 for v in certified))
    missing=[float(q) for q in targets if not includes(q)]
    return dict(full_requested_actions_feasible=valid and not missing,
        checked_interval_native=[low,high],uncovered_targets_native=missing,
        requested_horizon_native=horizon,
        native_whole_chain_certified_targets=sorted(set(certified)),
        full_requested_horizon_feasible=includes(horizon) if horizon is not None else None,
        endpoint_is_not_physical_progress=True)


def alternatives(action):
    """All replacements are independent, predeclared FIT trajectories."""
    items=action.get('path_alternatives',[])
    if not isinstance(items,list) or len(items)>2:raise ValueError('AT_MOST_TWO_DECLARED_PATH_ALTERNATIVES')
    if items and not action['role'].startswith('FIT'):raise ValueError('VALIDATION_TARGET_MUST_NOT_BE_SUBSTITUTED')
    if not items:return []
    from agent_command_protocol_v12 import validate
    roles={action['role']}
    for item in items:
        if (not isinstance(item,dict) or 'path_alternatives' in item
                or not item.get('role','').startswith('FIT') or item['role'] in roles
                or item.get('coordinate_unit')!=action.get('coordinate_unit')):
            raise ValueError('DISTINCT_COMPLETE_FIT_ALTERNATIVE_REQUIRED')
        validate(dict(protocol_version=1,sequence=0,operation='attempt_grasp_and_probe',segments=[item]),0)
        roles.add(item['role'])
    return items


def targets(actions):
    return [item['target_q'] for action in actions for item in [action,*alternatives(action)]]


@contextmanager
def candidate_gate(actions,horizon,pool=None):
    """Existing ranked IK branches fall through when the full action is absent.

    The original native closing/approach/timed-route check remains mandatory.
    This predicate does not change geometry, utility or collision margins.
    """
    import native_g1_execution_candidates as native
    original=native.CandidateCheck.__call__
    required=targets(actions)
    audit=[]
    def checked(checker,candidate):
        if pool is not None:pool.append(deepcopy(candidate))
        bounds=coverage(candidate,required,horizon)
        prismatic=getattr(checker,'g',{}).get('job',{}).get('articulation_v7',{}).get('joint_type')=='prismatic'
        integrated=bool(getattr(checker,'g',{}).get('job',{}).get('v81_integrated_repair'))
        # A nominal arc covering its requested horizon is also a valid input
        # to whole-chain CHECKING when the first probe reverses across its
        # start. It is not permission to execute that uncovered reverse.
        first_checked=coverage(candidate,[actions[0]['target_q']])['full_requested_actions_feasible']
        check_whole_chain=integrated and not prismatic and (
            first_checked or bounds['full_requested_horizon_feasible'] is True)
        audit.append(dict(family=[candidate[k] for k in
            ('section_fraction','insertion_m','roll_deg','pitch_deg')],coverage=bounds,
            deferred_to_original_prismatic_whole_chain=prismatic,
            deferred_to_revolute_whole_chain=check_whole_chain))
        if not bounds['full_requested_actions_feasible'] and not prismatic and not check_whole_chain:
            result=dict(valid=False,reason='REQUESTED_TARGET_NOT_FULLY_CHECKED',coverage=bounds)
            checker.audit.append(dict(family=[candidate[k] for k in
                ('section_fraction','insertion_m','roll_deg','pitch_deg')],**result))
            return result
        result=original(checker,candidate)
        if not prismatic:
            result['target_coverage']=bounds
            if result.get('valid') and integrated:
                from .revolute_certificate import evaluate
                chain=evaluate(checker.g,candidate,actions,checker.policy,checker.config,checker.solver)
                result['v81_revolute_chain']=chain
                if not chain['valid']:result.update(valid=False,reason=chain['reason'])
                certified_plan=dict(candidate,execution_feasibility=result)
                bounds=coverage(certified_plan,required,horizon)
                result['target_coverage']=bounds
                if result['valid'] and not bounds['full_requested_actions_feasible']:
                    result.update(valid=False,reason='REQUESTED_TARGET_NOT_FULLY_CHECKED')
                checker.audit[-1].update(result)
                from agent_command_protocol_v12 import atomic_json
                atomic_json(checker.g['a'].output/'EXECUTION_CANDIDATE_AUDIT.json',dict(
                    candidates=checker.audit,weights_changed=False,
                    revolute_closing_lead_and_exit_checked=True,native_use='collision safety only'))
        return result
    with patch.object(native.CandidateCheck,'__call__',checked):yield audit


def state(ops):
    if not hasattr(ops,'_v81_replanning'):
        ops._v81_replanning=dict(state='READY',rejections=[],same_episode=True,
            new_episode_retry_authorized=False,physical_hard_stop_authoritative=True)
    return ops._v81_replanning


def reject(ops,route,action):
    record=dict(reason=route.get('reason'),requested_action=deepcopy(action),
        measured_coordinate=ops.r.actual(),station_generation=ops.r.base_generation,
        native_rejection_preserved=deepcopy(route))
    s=state(ops);s['rejections'].append(record);s['state']='REPLAN_REQUIRED'
    if hasattr(ops.r,'output'):
        from agent_command_protocol_v12 import atomic_json
        atomic_json(ops.r.output/'PATH_REJECTION_BACKLOG.json',dict(requests=s['rejections'],
            original_requests_preserved=True,shorter_alternative_is_not_original_completion=True))
    return record


def successor(ops,action,original):
    route=original(action)
    # An opt-in, one-shot engineering protocol fault. It is recorded as an
    # injected rejection, never misreported as an actual native collision.
    fixture=ops.r.job.get('v81_path_rejection_fixture')
    if fixture and route.get('valid') and not getattr(ops,'_v81_rejection_injected',False):
        if ops.r.job.get('v81_phase')!='engineering':raise ValueError('ENGINEERING_REJECTION_ONLY')
        ops._v81_rejection_injected=True
        route=dict(valid=False,reason='INJECTED_COLLISION_REJECTION',engineering_injected=True,
            original_native_check=route,robot_motion_executed=False)
    if route.get('valid'):return route
    record=reject(ops,route,action)
    if ops.r.supervisor.latched is not None:
        state(ops)['state']='HARD_STOP';return route
    previous=ops.r.record.segments[-1].get('result',{}) if ops.r.record.segments else {}
    from interaction_runtime.segments import continuation_allowed
    allowed,why=continuation_allowed(previous,action,None,
        ops.r.episode_budget.remaining(ops.r.tick*ops.r.dt))
    if not allowed or not previous.get('safely_stopped') or not previous.get('physical_transmission_valid'):
        record['alternative_checks_blocked']=why
        return dict(route,status='PLANNING_REJECTED',recoverable=True,
            reason=route.get('reason'),safe_alternative_boundary=False)
    start=float(ops.r.actual());goal=float(action['target_q'])
    # The frozen caller has already required safe stopped transmission via
    # continuation_allowed. No Agent inference or waiting occurs under load.
    checks=[]
    for other in alternatives(action):
        target=float(other['target_q'])
        if not min(start,goal)<target<max(start,goal):
            checks.append(dict(valid=False,reason='NOT_A_SHORTER_SAME_DIRECTION_ACTION',action=other));continue
        ops.articulation().validate_target(other)
        checked=original(other);checks.append(dict(action=deepcopy(other),check=deepcopy(checked)))
        if ops.r.supervisor.latched is not None:
            state(ops)['state']='HARD_STOP';return dict(valid=False,reason='HARD_STOP')
        if checked.get('valid'):
            selected=deepcopy(other)
            record['alternatives']=checks;record['selected_action']=selected
            action.clear();action.update(selected)
            state(ops)['state']='CHECKED_ALTERNATIVE_SELECTED'
            return dict(checked,requested_path_rejected=True,rejected_request=record,
                selected_action=selected,full_original_target_feasible=False,
                feasibility_scope='this checked/rejected path only',global_target_infeasibility_proven=False,
                full_selected_target_feasible=True)
    record['alternatives']=checks
    return dict(route,status='PLANNING_REJECTED',recoverable=True,
        episode_termination_required=False,global_target_infeasibility_proven=False,alternative_checks=checks)


def packet(ops):
    from .recovery_batch import measured_costs
    s=deepcopy(state(ops));s['hard_latch']=ops.r.supervisor.latched
    s['unloaded']=ops.g['recovery_state']=='UNLOADED_OBSERVATION'
    s['remaining_work']=ops.r.episode_budget.remaining(ops.r.tick*ops.r.dt)
    s['measured_costs']=measured_costs(ops)
    s['search_order']=['checked predeclared shorter successor',
        'same-station automatic IK branches and ranked observed grasp candidates',
        'fresh wrist observation if post-contact or stale',
        'Agent-selected evaluated base relocation and fresh regrasp']
    s['instruction']='Multiple safe replans within this scene are permitted. Never approve an uncovered full target or clear a hard latch.'
    return s


def admission(ops,actions=None):
    """Preserve one measured recovery opportunity before another acquisition.

    EpisodeBudget.remaining already subtracts the deterministic exit reserve.
    Unknown first-operation costs are reported, not invented as safety limits.
    """
    from .recovery_batch import measured_costs
    if getattr(ops.r,'job',{}).get('v81_integrated_repair'):
        from .costs import admission as measured_admission
        return measured_admission(ops,actions)
    cost=measured_costs(ops);remaining=ops.r.episode_budget.remaining(ops.r.tick*ops.r.dt)
    latest=cost['observed_max_operation_wall_s']
    names=('reobserve_and_replan','prepare_grasp','attempt_grasp_and_probe')
    reserve=sum(latest[n] for n in names) if all(n in latest for n in names) else None
    need=(reserve+latest['attempt_grasp_and_probe']) if reserve is not None else None
    return dict(admitted=need is None or need<=remaining['wall_s'],
        measured_current_attempt_plus_one_recovery_wall_s=need,
        remaining_work=remaining,exit_reserve_already_excluded=True,
        cost_unknown=need is None)
