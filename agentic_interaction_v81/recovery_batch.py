"""Finite deterministic recovery preparation; never physical grasp approval."""
import json


def measured_costs(ops):
    path=ops.r.output/'AGENT_ACTION_TRACE.json'
    rows=json.loads(path.read_text()) if path.is_file() else []
    latest={}
    for row in rows:
        op=row.get('command',{}).get('operation')
        result=row.get('result',{})
        if op=='attempt_grasp_and_probe' and not result.get('acquisition',{}).get('acquired'):continue
        if isinstance(row.get('wall_s'),(int,float)):
            latest[op]=max(latest.get(op,0),float(row['wall_s']))
    # A fresh initial actuator-executed view is a measured observation cost even
    # before the first relocation; do not disable admission merely because the
    # spelling of the prior observation operation differs.
    views=[latest[n]for n in ('reobserve_and_replan','observe_view','observe')if n in latest]
    if views:latest['reobserve_and_replan']=max(views)
    names=['reobserve_and_replan','prepare_grasp','attempt_grasp_and_probe']
    available=all(n in latest for n in names)
    lower=(sum(latest[n]for n in names)+latest.get('evaluate_station',0)
        +latest.get('relocate_station',0)) if available else None
    return dict(available=available,observed_max_operation_wall_s=latest,
        recovery_preparation_and_attempt_observed_cost_s=lower,
        Agent_wait_included=False,observed_cost_is_not_a_guarantee=True,
        exit_reserve_already_excluded_from_remaining_work=True)


def relocate_observe_prepare(ops,command):
    if ops.r.job['articulation_v7']['joint_type']!='prismatic':
        return dict(executed=False,reason='PRISMATIC_RECOVERY_BATCH_NOT_VALIDATED_FOR_REVOLUTE')
    from .declaration_preflight import check
    check(command.get('planned_segments'),ops.articulation())
    from .contact_following import selection
    selection(command,'prismatic')
    if 'native_g1_preload_request_n' in command:
        preload=command['native_g1_preload_request_n']
        if isinstance(preload,bool)or preload not in (.5,.75,1.,1.5):raise ValueError('TESTED_PRELOAD_MENU_ONLY')
    cost=measured_costs(ops)
    remaining=ops.r.episode_budget.remaining(ops.r.tick*ops.r.dt)
    if cost['available'] and cost['recovery_preparation_and_attempt_observed_cost_s']>remaining['wall_s']:
        return dict(executed=False,recoverable=True,reason='OBSERVED_RECOVERY_COST_EXCEEDS_WORK_BUDGET',
            costs=cost,remaining=remaining,physical_contact_approved=False)
    evaluated=ops.evaluate_station(command)
    if not evaluated.get('valid'):
        return dict(executed=False,evaluation=evaluated,reason='NO_VALIDATED_STATION_ROUTE')
    moved=ops.relocate_station(evaluated)
    if not moved.get('reached'):
        return dict(executed=False,movement=moved,reason='STATION_NOT_REACHED')
    observation=ops.reobserve()
    rows=ops.g['agent_observations']
    if not rows or not rows[-1].get('accepted_for_geometry'):
        return dict(executed=True,movement=moved,observation=observation,
            reason='FRESH_OBSERVATION_REQUIRES_AGENT_TARGET_SELECTION',
            physical_contact_approved=False,ready_for_Agent_review=True)
    # Candidate generation is tentative. The Agent sees actual new imagery AND
    # the proposed grasp together before any contact can be separately approved.
    planning={k:command[k]for k in ('planned_segments','planning_target_q',
        'native_g1_preload_request_n','transverse_unloading','contact_following')if k in command}
    planning['reason']=command['reason']
    proposal=ops.prepare_grasp(planning)
    return dict(executed=True,movement=moved,observation=observation,proposal=proposal,
        ready_for_Agent_review=True,physical_contact_approved=False,
        new_observation_must_be_inspected_before_any_grasp=True)
