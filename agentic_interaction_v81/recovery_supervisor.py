"""Action outcomes and scene-local recovery; never launches/resets a World.

Only trusted control state permits the original stop/unload/exit primitives.
A rejected path is not a task failure. Unconfirmed parking can leave a
verified released hold available for observation and checked exit replanning.
"""
import ast
import numpy as np
from interaction_runtime.policy import SafetyStop

ACTION_REJECTED='ACTION_REJECTED'
RECOVERABLE_FAILURE='RECOVERABLE_FAILURE'
SAFETY_STOP='SAFETY_STOP'
EPISODE_COMPLETE='EPISODE_COMPLETE'
UNRECOVERABLE='UNRECOVERABLE'
PLANNING={'evaluate_view','evaluate_station','prepare_grasp','build_geometry',
    'validate_execution','redeclare_actions','select_observed_target','plan_panel_push'}
SENSING={'observe','reobserve_and_replan','fresh_mode_observation','inspect_contact_result','initialize_observation'}
WORKERS={'retry_model'}
EXIT_REPLAN={'inspect','observe','evaluate_view','move_and_observe','observe_view',
    'reobserve_and_replan','fresh_mode_observation','safe_release_retreat','retry_model',
    'extend_orchestration_wall','finish'}


def enabled(ops):return bool(getattr(getattr(ops,'r',None),'job',{}).get('v86_recovery_supervisor'))


def trusted(ops):
    r=ops.r
    if r.supervisor.latched is not None or not r.rows or r.stopped:return False
    row=r.rows[-1];ft=row.get('wrist_ft',{})
    if ft.get('valid') is not True:return False
    try:
        q=np.asarray(r.robot.get_joint_positions());dq=np.asarray(r.robot.get_joint_velocities())
        numbers=np.r_[q,dq,r.qtarget,r.filtered(),r.actual(),ft['raw_wrench_sensor']]
        return bool(np.isfinite(numbers).all()
            and r.model.margin(q[r.arm])>r.policy.data['joint_margin_rad']
            and r.model.margin(r.qtarget[r.arm])>r.policy.data['joint_margin_rad'])
    except Exception:return False  # failed state read is never permission to move


def released(ops):
    r=ops.r
    return bool(trusted(ops) and r.grasp_reference is None and ops.g.get('reference') is None
        and r.rows[-1]['aperture_m']>=.083
        and max(r.filtered())<=r.policy.data['force']['unloaded_pad_n']
        and r.supervisor.force.settled())


def released_hold(ops):
    if not released(ops):return False
    ops.r.hold(.3)  # original measured controller/force/collision supervision
    q=np.asarray(ops.r.robot.get_joint_positions())[ops.r.arm]
    return bool(released(ops) and ops.r.station_motion.ready(ops.r.rows[-1]['t'],q))


def exception_result(ops,command,error):
    op=command['operation'];message=str(error)
    if isinstance(error,SafetyStop):
        return dict(status=SAFETY_STOP,outcome=SAFETY_STOP,reason=error.reason,evidence=error.evidence)
    worker=op in WORKERS
    controlled=bool(ops.r.job.get('v87_continuous_recovery') and isinstance(error,RuntimeError)
        and message.startswith('CONTROLLED_EXECUTION_INTERRUPT:'))
    known_path=controlled or isinstance(error,RuntimeError) and message.startswith((
        'MOTION_PATH_UNAVAILABLE:','NO_CURRENT_PREGRASP_IK','APPROACH_IK',
        'VISUAL_PREGRASP_LOCAL_IK_UNAVAILABLE','AGENT_EXECUTION_RETURN_OR_APPROACH_UNAVAILABLE',
        'MEASURED_STATE_APPROACH_UNAVAILABLE','BILATERAL_HOLD_NOT_ESTABLISHED','LOCAL_IK_UNAVAILABLE'))
    # Pure planning/sensing/worker failures do not imply an actuator fault.
    recoverable=worker or op in PLANNING or op in SENSING or known_path
    if not recoverable or not trusted(ops):
        return dict(status='CONTROL_STATE_UNTRUSTED',outcome=UNRECOVERABLE,
            reason=repr(error),recovery=dict(control_trusted=False,automatic_motion_allowed=False))
    category=RECOVERABLE_FAILURE if controlled else ACTION_REJECTED if op in PLANNING or known_path else RECOVERABLE_FAILURE
    return dict(status=category,outcome=category,reason=repr(error),
        exception_type=type(error).__name__,worker_pending=worker,operation_stopped=True)


def failed(result):
    if result.get('outcome') in (ACTION_REJECTED,RECOVERABLE_FAILURE):return True
    if result.get('status') in ('PLANNING_REJECTED','STRATEGY_CHANGE_REQUIRED',
            'RECOVERY_RESERVE_REQUIRED','TRANSMISSION_FAILURE','SAFE_PARTIAL_PROGRESS'):
        return True
    if any(result.get(k) is False for k in ('plan','valid','feasible','executed','observation')):return True
    if result.get('operation_stopped') or result.get('acquisition',{}).get('acquired') is False:return True
    return any(s.get('status') in ('TRANSMISSION_FAILURE','SAFE_PARTIAL_PROGRESS','PLANNING_REJECTED')
        for s in result.get('segments',[]))


def before(ops,command):
    if getattr(ops,'_v86_exit_pending',False) and command['operation'] not in EXIT_REPLAN:
        return dict(status=RECOVERABLE_FAILURE,outcome=RECOVERABLE_FAILURE,
            reason='VERIFIED_RELEASED_HOLD_REQUIRES_CHECKED_EXIT_BEFORE_NEW_CONTACT_OR_BASE_MOTION',
            recovery=dict(hold_verified=True,park_verified=False,exit_pending=True),
            executed=False,episode_termination_required=False)
    if command['operation'] in ('attempt_grasp_and_probe','acquire_execute'):
        planned=getattr(ops,'_v86_plan_scene',None)
        if planned is not None:
            from .scene_stamp import capture,same
            current=capture(ops.r)
            if not same(planned,current):
                return dict(status=ACTION_REJECTED,outcome=ACTION_REJECTED,executed=False,
                    reason='MEASURED_COLLISION_SCENE_CHANGED_REPLAN_REQUIRED',
                    planned_scene=planned,current_scene=current)
    return None


def complete(ops,command,result):
    """One bounded original recovery per failed action, then return to Agent."""
    if result.get('outcome') in (SAFETY_STOP,UNRECOVERABLE) or result.get('status') in (
            'SAFETY_STOP','CONTROL_STATE_UNTRUSTED') or ops.r.supervisor.latched is not None:
        return result
    if result.get('error') and not result.get('outcome'):
        message=result['error']
        if isinstance(message,str) and message.startswith('RuntimeError(') and message.endswith(')'):
            try:message=ast.literal_eval(message[len('RuntimeError('):-1])
            except (ValueError,SyntaxError):pass
        normalized=exception_result(ops,command,RuntimeError(str(message)))
        result=dict(result,**normalized,original_operation_result=dict(result))
        if result.get('outcome')==UNRECOVERABLE:return result
    if command['operation']=='safe_release_retreat':
        from .terminal_safety import classify
        if classify(ops.r.supervisor.latched,result)['safe_park_verified'] and released(ops):
            ops._v86_exit_pending=False
        elif result.get('safe') is False:
            result=dict(result,outcome=RECOVERABLE_FAILURE)
    if command['operation']=='finish' and result.get('safe_exit'):
        from .terminal_safety import classify
        verified=classify(ops.r.supervisor.latched,result['safe_exit'])['safe_park_verified']
        if verified:
            from .task_recovery import finish_check
            assessment=finish_check(ops,command)
            return dict(result,outcome=EPISODE_COMPLETE if assessment.get('task_complete')
                else 'EPISODE_STOPPED',finish_authorized=assessment['allowed'])
        result=dict(result,status=RECOVERABLE_FAILURE,outcome=RECOVERABLE_FAILURE,
            reason='FINISH_REQUIRES_MEASURED_SAFE_PARK')
    if not failed(result):return result
    if result.get('recovery',{}).get('exit_pending'):return result
    category=result.get('outcome',ACTION_REJECTED if command['operation'] in PLANNING
        else RECOVERABLE_FAILURE)
    result=dict(result,outcome=category)
    if not trusted(ops):
        return dict(result,status='CONTROL_STATE_UNTRUSTED',outcome=UNRECOVERABLE,
            recovery=dict(control_trusted=False,automatic_motion_allowed=False))
    # Numerical lag cannot gate otherwise valid manipulation or consume a regrasp.
    if result.get('worker_pending'):
        return dict(result,status=RECOVERABLE_FAILURE,
            recovery=dict(control_trusted=True,worker_deferred=True,original_goal_preserved=True))
    r=ops.r;coordinate_before=float(r.actual())
    goal_before=r.job['v81_task_goal_native'];records_before=len(r.record.segments)
    if r.job.get('v87_continuous_recovery'):
        from agentic_interaction_v87.recovery import current_released_hold,invalidate
        from .terminal_safety import classify
        existing=result.get('safe_exit',{})
        already_parked=classify(None,existing)['safe_park_verified']
        planning_only=command['operation'] in PLANNING and not result.get('physical_motion_executed',False)
        if (planning_only or already_parked) and current_released_hold(ops):
            invalidate(ops)
            fresh=None
            if already_parked:
                try:fresh=ops.observe()
                except SafetyStop as error:return exception_result(ops,command,error)
                except Exception as error:fresh=dict(observation_error=repr(error))
            return dict(result,status=category,recoverable=True,episode_termination_required=False,
                recovery=dict(control_trusted=True,hold_verified=True,
                    park_verified=already_parked or bool(r.g.get('at_safe_park')),
                    repeated_physical_parking_skipped=True,pure_planning_rejection=planning_only,
                    fresh_observation=fresh,original_goal_preserved=True,
                    current_coordinate_native=float(r.actual()),World_recreated=False,
                    object_state_reset=False,history_preserved=True))
    try:
        if hasattr(ops,'_abc_retain'):ops._abc_retain=False
        from .physics_lifecycle import resume_for_exit
        resume_for_exit(r)
        r.stop_command()  # original guarded deceleration, never a target reset
        safe=ops.release_park()
        if safe.get('safe') is True and released(ops):
            # release_park owns measured native collision/parking verification.
            park=safe.get('park')or{}
            parked=bool(park.get('reached') and park.get('unloaded')
                and park.get('measured_collision_check',{}).get('valid'))
            if not parked:raise RuntimeError('RECOVERY_PARK_EVIDENCE_INCOMPLETE')
            ops._v86_exit_pending=False
        elif released(ops):
            if not released_hold(ops):
                raise RuntimeError('RECOVERY_STATIONARY_HOLD_NOT_CONFIRMED')
            parked=False;ops._v86_exit_pending=True
        else:raise RuntimeError('RECOVERY_UNLOAD_NOT_CONFIRMED')
    except SafetyStop as error:return exception_result(ops,command,error)
    except (TypeError,ValueError,KeyError,OSError) as error:
        # A software failure in exit planning/publication is not proof that
        # the controller failed. Only a fresh original guarded stationary hold
        # can retain the World; no new contact/base actuation is permitted.
        try:
            if not released_hold(ops):raise RuntimeError('RECOVERY_HOLD_UNCONFIRMED')
        except SafetyStop as hard:return exception_result(ops,command,hard)
        except Exception as failure:
            return dict(result,status='CONTROL_STATE_UNTRUSTED',outcome=UNRECOVERABLE,
                recovery=dict(reason=repr(failure),safe_park_verified=False,automatic_motion_allowed=False))
        safe=dict(safe=False,reason='EXIT_SOFTWARE_FAILURE_AT_VERIFIED_RELEASED_HOLD',error=repr(error))
        parked=False;ops._v86_exit_pending=True
    except Exception as error:
        return dict(result,status='CONTROL_STATE_UNTRUSTED',outcome=UNRECOVERABLE,
            recovery=dict(reason=repr(error),safe_park_verified=False,automatic_motion_allowed=False))
    ops.current_plan_ready=False
    if hasattr(ops,'_v81_plan_ticket'):ops._v81_plan_ticket=None
    ops.cache.clear()
    if hasattr(ops,'abc_state'):ops.abc_state='A';ops._abc_retain=False
    ops.g['recovery_state']='UNLOADED_OBSERVATION'
    fresh=None;observation_error=None
    try:fresh=ops.observe()  # actual current wrist, never initial/saved view replay
    except SafetyStop as error:return exception_result(ops,command,error)
    except Exception as error:observation_error=repr(error)
    assert r.job['v81_task_goal_native']==goal_before and len(r.record.segments)==records_before
    recovery=dict(control_trusted=True,hold_verified=True,park_verified=parked,
        exit_pending=not parked,source='original supervised stop/unload/park',
        coordinate_before_native=coordinate_before,current_coordinate_native=float(r.actual()),
        original_goal_native=goal_before,original_goal_preserved=True,
        history_preserved=True,object_state_reset=False,World_recreated=False,
        fresh_observation=fresh,observation_error=observation_error,
        next='change checked view/path/candidate; fresh regrasp or station evaluation after verified park')
    return dict(result,status=category,recoverable=True,episode_termination_required=False,
        safe_exit=safe,recovery=recovery)
