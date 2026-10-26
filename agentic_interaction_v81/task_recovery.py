"""Episode-level recovery bookkeeping around unchanged physical operations.

Failures describe an action, not a proof of global infeasibility. The Agent
still selects and inspects every view/grasp; this module never drives a robot.
"""
import hashlib
import ast
import json
import math
from agent_command_protocol_v12 import atomic_json


def enabled(ops):
    return bool(getattr(getattr(ops,'r',None),'job',{}).get('v81_active_recovery'))


def state(ops):
    if not hasattr(ops, '_v81_task_recovery'):
        ops._v81_task_recovery=dict(status='READY', goal_native=ops.r.job['v81_task_goal_native'],
            events=[], failed_requests={}, terminal_fault=None, original_goal_preserved=True)
    return ops._v81_task_recovery


def unloaded(ops):
    r=ops.r
    return bool(ops.g['recovery_state']=='UNLOADED_OBSERVATION'
        and r.supervisor.latched is None and r.grasp_reference is None
        and math.isfinite(float(r.actual()))
        and max(r.filtered())<=r.policy.data['force']['unloaded_pad_n']
        and r.supervisor.force.settled())


def signature(ops, command):
    # A new reason string alone cannot authorize replay of an unchanged failure.
    ignored={'reason','sequence','protocol_version','recovery_strategy'}
    payload=dict(command={k:v for k,v in command.items() if k not in ignored},
        station_generation=ops.r.base_generation, coordinate=round(float(ops.r.actual()), 7),
        observations=[(v['folder'],v.get('world_sim_s')) for v in ops.g.get('agent_observations',[])])
    if command['operation']=='attempt_grasp_and_probe':
        p=ops.r.output/'visual_plan.json'
        payload['plan_sha256']=hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
    return hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()


def remaining(ops):
    return ops.r.episode_budget.remaining(ops.r.tick*ops.r.dt)


def completed(ops):
    progress=getattr(ops.r,'v81_task_progress',{})
    first_goal=progress.get('first_goal_record')
    # Early diagnostic reversals cannot satisfy the requested reverse after
    # reaching the task goal. Missing temporal evidence stays incomplete.
    after_goal=[s for s in ops.r.record.segments if first_goal is not None
                and s.get('first_step',-1)>=first_goal]
    reverse=sum(max(0.,-float(s.get('result',{}).get('gain_native',0.)))
        for s in after_goal
        if s.get('result',{}).get('status')=='TARGET_REACHED'
        and s.get('result',{}).get('physical_transmission_valid'))
    validation=any(s['role'].startswith('VALIDATION')
        and s.get('result',{}).get('status')=='TARGET_REACHED'
        and s.get('result',{}).get('requested_target_reached')
        and s.get('result',{}).get('physical_transmission_valid')
        for s in after_goal)
    return dict(actual_goal_reached=progress.get('maximum_measured_native',-math.inf)>=state(ops)['goal_native'],
        completed_reverse_native=reverse,
        meaningful_reverse=reverse>=ops.r.job['v81_minimum_reverse_native'],
        complete_independent_validation=validation)


def finish_check(ops, command):
    s=state(ops);done=completed(ops);resources=remaining(ops)
    from agentic_interaction_v87.persistent_goal import enabled as persistent,finish_assessment
    if persistent(ops.r):return finish_assessment(ops,done)
    if command.get('owner_requested_stop') is True:
        return dict(allowed=True,reason='EXPLICIT_OWNER_STOP',task_complete=False)
    if s['terminal_fault'] or ops.r.supervisor.latched is not None:
        return dict(allowed=True,reason='HARD_SAFETY_OR_UNTRUSTED_CONTROL',task_complete=False)
    if done['actual_goal_reached'] and done['meaningful_reverse'] and done['complete_independent_validation']:
        return dict(allowed=True,reason='MEASURED_TASK_LIFECYCLE_COMPLETE',task_complete=True,completion=done)
    if resources['wall_s']<=0 or resources['sim_s']<=0:
        return dict(allowed=True,reason='PROTECTED_EXIT_RESOURCE_BOUNDARY',task_complete=False,resources=resources)
    costs=getattr(ops,'_v81_phase_costs',{})
    minimum=costs.get('acquire',0.)+costs.get('release',0.)
    if minimum>0 and minimum>=resources['wall_s'] and ops.g.get('at_safe_park'):
        return dict(allowed=True,reason='NO_NEW_CONTACT_BLOCK_FITS_MEASURED_ACQUIRE_AND_EXIT_COST',
            task_complete=False,minimum_measured_block_wall_s=minimum,resources=resources)
    assessment=command.get('stop_assessment',{})
    # Every remaining recovery class needs a recorded, current-scene reason.
    # A retry count or one rejected candidate is deliberately insufficient.
    required={'change_view','change_candidate','change_motion','relocate_station'}
    rejected={row['sequence']:row for row in s['events'] if row['failure']
        and row['station_generation']==ops.r.base_generation
        and abs(row['measured_coordinate_native']-float(ops.r.actual()))<1e-4}
    covered=set()
    for item in assessment.get('considered',[]):
        refs=item.get('evidence_sequences',[])
        if item.get('reason') and refs and all(n in rejected for n in refs):
            if any(rejected[n]['strategy']==item.get('strategy') for n in refs):
                covered.add(item['strategy'])
    if assessment.get('kind')=='NO_SAFE_CONTINUATION' and required<=covered:
        return dict(allowed=True,reason='DOCUMENTED_SAFE_ALTERNATIVES_EXHAUSTED',
            task_complete=False,assessment=assessment,completion=done)
    return dict(allowed=False,status='CONTINUATION_REQUIRED',recoverable=True,
        reason='A_SINGLE_FAILURE_OR_RETRY_COUNT_DOES_NOT_END_THE_TASK',completion=done,
        missing_considered_strategies=sorted(required-covered),resources=resources,
        original_goal_native=s['goal_native'])


def before(ops, command):
    if not enabled(ops):return None
    from . import recovery_supervisor
    if recovery_supervisor.enabled(ops):
        blocked=recovery_supervisor.before(ops,command)
        if blocked is not None:return blocked
    s=state(ops);op=command['operation']
    if op=='finish':
        check=finish_check(ops,command)
        if not check['allowed']:return check
        s['termination_assessment']=check
        return None
    key=signature(ops,command)
    ops._v81_active_request_signature=key
    if op in ('evaluate_view','build_geometry','prepare_grasp','attempt_grasp_and_probe') and key in s['failed_requests']:
        return dict(status='STRATEGY_CHANGE_REQUIRED',recoverable=True,executed=False,
            reason='UNCHANGED_FAILED_REQUEST',previous_sequence=s['failed_requests'][key],
            original_goal_native=s['goal_native'],episode_termination_required=False)
    return None


def classify(ops, result):
    if ops.r.supervisor.latched is not None or result.get('status')=='SAFETY_STOP' or result.get('operation_status')=='SAFETY_STOP':
        return 'HARD_STOP'
    if result.get('status')=='CONTROL_STATE_UNTRUSTED':return 'CONTROL_STATE_UNTRUSTED'
    outcome=result.get('outcome')
    if outcome=='UNRECOVERABLE':return 'UNRECOVERABLE'
    if outcome in ('ACTION_REJECTED','RECOVERABLE_FAILURE'):
        if not unloaded(ops):return 'SAFE_BOUNDARY_UNAVAILABLE'
        if result.get('recovery',{}).get('exit_pending'):return 'RECOVERY_EXIT_PENDING'
        return 'REPLAN_REQUIRED' if outcome=='ACTION_REJECTED' else 'OPERATION_REPLAN_REQUIRED'
    error=result.get('error')
    if error and result.get('operation_status')!='BUDGET_EXHAUSTED':
        known=False
        if error.startswith('RuntimeError(') and error.endswith(')'):
            try:message=ast.literal_eval(error[len('RuntimeError('):-1])
            except (ValueError,SyntaxError):message=''
            known=isinstance(message,str) and message.startswith((
                'NO_CURRENT_PREGRASP_IK','APPROACH_IK','BILATERAL_HOLD_NOT_ESTABLISHED',
                'VISUAL_PREGRASP_LOCAL_IK_UNAVAILABLE','AGENT_EXECUTION_RETURN_OR_APPROACH_UNAVAILABLE',
                'MEASURED_STATE_APPROACH_UNAVAILABLE','MOTION_PATH_UNAVAILABLE:',
                'DANGEROUS_GEOMETRY_COLLISION:','SELF_COLLISION:'))
            known=known or bool(ops.r.job.get('v84_abc') and message=='LOCAL_IK_UNAVAILABLE')
        if not known:return 'CONTROL_STATE_UNTRUSTED'
    if not unloaded(ops):return 'SAFE_BOUNDARY_UNAVAILABLE'
    segments=result.get('segments',[])
    if any(s.get('status')=='SAFETY_STOP' for s in segments):return 'HARD_STOP'
    if any(s.get('status')=='TRANSMISSION_FAILURE' for s in segments):return 'CONTACT_RECOVERY_REQUIRED'
    if any(s.get('status')=='SAFE_PARTIAL_PROGRESS' for s in segments):return 'PROGRESS_RECOVERY_REQUIRED'
    if result.get('operation_status')=='BUDGET_EXHAUSTED' or result.get('status') in ('BUDGET_EXHAUSTED','OPERATION_BUDGET_EXHAUSTED') or any(s.get('status')=='BUDGET_EXHAUSTED' for s in segments):
        resources=remaining(ops)
        return 'OPERATION_REPLAN_REQUIRED' if resources['wall_s']>0 and resources['sim_s']>0 else 'WORK_BUDGET_EXHAUSTED'
    if any(s.get('status')=='VALIDATION_DEFERRED' for s in segments):return 'VALIDATION_PENDING'
    if result.get('operation_stopped') or result.get('error') or result.get('acquisition',{}).get('acquired') is False:
        return 'OPERATION_REPLAN_REQUIRED'
    if (any(result.get(k) is False for k in ('plan','valid','feasible','executed','observation'))
            or result.get('status') in ('PLANNING_REJECTED','RECOVERY_RESERVE_REQUIRED','STRATEGY_CHANGE_REQUIRED','CONTINUATION_REQUIRED')):
        return 'REPLAN_REQUIRED'
    return 'READY'


def after(ops, command, result):
    if not enabled(ops):return result
    s=state(ops);status=classify(ops,result);s['status']=status
    if status in ('HARD_STOP','CONTROL_STATE_UNTRUSTED','SAFE_BOUNDARY_UNAVAILABLE','UNRECOVERABLE'):
        s['terminal_fault']=dict(status=status,reason=result.get('reason'),sequence=command['sequence'])
    op=command['operation']
    strategy={
        'evaluate_view':'change_view','move_and_observe':'change_view','observe':'change_view',
        'build_geometry':'change_candidate','prepare_grasp':'change_candidate','validate_execution':'change_candidate',
        'redeclare_actions':'change_motion','attempt_grasp_and_probe':'change_motion',
        'evaluate_station':'relocate_station','relocate_station':'relocate_station'}.get(op,op)
    failure=status not in ('READY','VALIDATION_PENDING')
    if failure and op!='finish':
        key=getattr(ops,'_v81_active_request_signature',None)
        if key:s['failed_requests'][key]=command['sequence']
    s['events'].append(dict(sequence=command['sequence'],operation=op,strategy=strategy,
        status=status,failure=failure,reason=result.get('reason') or result.get('error'),
        measured_coordinate_native=float(ops.r.actual()),station_generation=ops.r.base_generation,
        physical_grasp_started=bool(result.get('physical_grasp_started')),
        segment_outcomes=[{k:a.get(k) for k in ('status','reason','joint_coordinate','gain_native','physical_transmission_valid')}
                          for a in result.get('segments',[])],
        safe_exit=result.get('safe_exit',{}).get('safe')))
    atomic_json(ops.r.output/'TASK_RECOVERY.json',s)
    result['task_recovery']=dict(status=status,original_goal_native=s['goal_native'],
        current_coordinate_native=float(ops.r.actual()),unloaded=unloaded(ops),
        episode_termination_required=s['terminal_fault'] is not None,
        instruction='Inspect current progress and feedback. Change view/candidate/motion; unload and regrasp or relocate when needed. Keep pending validation; never reset object state.')
    return result


def acquisition_allowance(ops, command):
    if not enabled(ops) or ops.acquisitions<int(ops.r.job['runtime_acquisition_budget']):return
    if not unloaded(ops) or not ops.current_plan_ready:
        raise ValueError('FRESH_VALIDATED_UNLOADED_RECOVERY_REQUIRED')
    # Time/simulation/exit budgets and all native admission checks still bind.
    # The initial acquisition allowance is a review point, not a task failure.
    old=ops.r.job['runtime_acquisition_budget']
    ops.r.job['runtime_acquisition_budget']=ops.acquisitions+1
    atomic_json(ops.r.output/'ACQUISITION_ALLOWANCE.json',dict(previous=old,
        current=ops.acquisitions+1,sequence=command.get('sequence'),reason=command.get('reason'),
        remaining=remaining(ops),finite_episode_budgets_unchanged=True,
        native_current_plan_required=True,not_a_new_episode=True))
