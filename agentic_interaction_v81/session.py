"""Finite Agent session with station requests at verified unloaded boundaries."""
import itertools,json,time
from agent_command_protocol_v12 import atomic_json
from .transport import receive,complete
from interaction_runtime.serialization import result_packet
from interaction_runtime.policy import SafetyStop
from interaction_runtime.segments import ExecutionBudgetExhausted,validate_action
from .operations import operations_class
from .session_status import write as status
from .debug_budget import window


def serve(r):
    if r.job['articulation_v7']['joint_type']=='prismatic':
        from interaction_runtime.prismatic_radial_session_v751 import PrismaticRadialOperationsV751 as Owner
    else:
        from interaction_runtime.probing_v7 import OperationsV7 as Owner
    ops=operations_class(Owner)(r);g=r.g
    from . import task_recovery
    commands=r.output/'agent_commands';responses=r.output/'agent_responses'
    commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True);trace=[];last=None;seq=-1
    try:
        from interaction_runtime.registered_observation_v71 import initialize
        initialize(r,ops)
        sequences=(itertools.count() if r.job.get('v81_active_recovery')
                   else range(int(r.job.get('maximum_agent_operations',100))))
        for seq in sequences:
            if r.job.get('v84_abc') and g['recovery_state']=='LOADED_CRITIQUE':
                from agentic_interaction_v84.session import loaded_command
                loaded_command(ops,seq)
            held=bool(r.job.get('v84_abc') and g['recovery_state']=='LOADED_CRITIQUE')
            if (g['recovery_state']!='UNLOADED_OBSERVATION' and not held) or r.supervisor.latched is not None:
                return dict(status='HARD_STOP_OR_NO_SAFE_DECISION_BOUNDARY',trace=trace,stop=r.supervisor.latched)
            wait,resources=window(r)
            if wait<=0:return dict(status='BUDGET_EXHAUSTED',safe_exit=ops.release_park(),trace=trace)
            try:
                packet=ops.decision_packet(seq,last) if hasattr(ops,'decision_packet') else dict(
                    sequence=seq,joint_coordinate=r.actual(),units=ops.articulation().units,
                    remaining_resources=resources,feedback=ops.feedback(),unloaded=True)
            except (json.JSONDecodeError,FileNotFoundError,TimeoutError) as error:
                if not task_recovery.enabled(ops) or not task_recovery.unloaded(ops):raise
                packet=dict(sequence=seq,joint_coordinate=r.actual(),units=ops.articulation().units,
                    remaining_resources=resources,unloaded=True,
                    feedback_status='READ_ONLY_FEEDBACK_UNAVAILABLE',feedback_error=repr(error),
                    original_goal_native=r.job['v81_task_goal_native'],
                    instruction='Preserve pending identification. Inspect current evidence or retry the numerical worker; no stale model is approved.')
            packet['station']=dict(generation=r.base_generation,measured_base=r.base.tolist(),
                operations=['evaluate_station','relocate_station'],actuation='native driven XY stage')
            if hasattr(r,'orchestration_extension'):
                packet['orchestration_extension']=r.orchestration_extension.packet()
            if task_recovery.enabled(ops):
                packet['task_recovery']=dict(task_recovery.state(ops),
                    completion=task_recovery.completed(ops))
            atomic_json(r.output/'DECISION_PACKET.json',packet)
            deadline=time.time()+wait
            status(r,seq,'WAITING',unloaded=not held,abc_contact=held,deadline_unix_s=deadline)
            atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=seq,unloaded=not held,abc_contact=held,timeout_s=wait,
                deadline_unix_s=deadline,requires_ACK_before_execution=True,
                live_status_file='AGENT_SESSION_STATUS.json'))
            began=time.perf_counter();wait_started_sim=r.tick*r.dt;wait_recoveries=[];path=commands/('%03d.json'%seq)
            heartbeat=began
            while not path.exists():
                if (not r.stopped and not (r.output/'STOP_AND_EXPORT').exists()
                        and time.perf_counter()-began>=wait and hasattr(r,'orchestration_extension')):
                    from .orchestration_extension import renew_parked_wait
                    renewal=renew_parked_wait(ops,deadline,sequence=seq)
                    if renewal.get('extended'):
                        if renewal.get('park_recovery_cost'):wait_recoveries.append(renewal['park_recovery_cost'])
                        # Keep began: finite authorized seconds add to this
                        # window instead of silently restarting its clock.
                        wait+=renewal['renewed_wait_s'];deadline+=renewal['renewed_wait_s']
                        packet['orchestration_extension']=r.orchestration_extension.packet()
                        packet['remaining_resources']=r.episode_budget.remaining(r.tick*r.dt)
                        if renewal.get('current_plan_invalidated'):
                            packet.update(joint_coordinate=float(r.actual()),current_plan_ready=False,plan_board=None,
                                instruction='Authorized wait recovered through measured safe park; revalidate a fresh automatic plan before contact. Original task remains active.')
                        atomic_json(r.output/'DECISION_PACKET.json',packet)
                        status(r,seq,'WAITING',unloaded=True,deadline_unix_s=deadline)
                        atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=seq,unloaded=True,
                            timeout_s=wait,deadline_unix_s=deadline,requires_ACK_before_execution=True,
                            live_status_file='AGENT_SESSION_STATUS.json',orchestration_renewal=renewal))
                    else:atomic_json(r.output/'ORCHESTRATION_WAIT_RENEWAL_REFUSAL.json',renewal)
                if r.stopped or (r.output/'STOP_AND_EXPORT').exists() or time.perf_counter()-began>=wait:
                    status(r,seq,'FINALIZING',unloaded=True,reason='decision deadline or owner stop')
                    return dict(status='ORCHESTRATION_TIMEOUT' if not r.stopped else 'OWNER_STOP',
                        safe_exit=ops.release_park(),trace=trace)
                if time.perf_counter()-heartbeat>=5:
                    status(r,seq,'WAITING',unloaded=True,deadline_unix_s=deadline)
                    heartbeat=time.perf_counter()
                time.sleep(.05)
            if r.job.get('v84_abc') and not held:
                rows=getattr(r,'abc_waits',[])
                duration=time.perf_counter()-began
                rows.append(dict(start_unix_s=time.time()-duration,end_unix_s=time.time(),
                    wall_s=duration,reason='UNLOADED_AGENT_COMMAND',dependency='Agent inspected current packet',
                    sequence=seq,start_sim_s=wait_started_sim,end_sim_s=r.tick*r.dt,
                    guarded_park_recoveries=wait_recoveries,
                    passive_wait_wall_s=max(0.,duration-sum(x['wall_s']for x in wait_recoveries))))
                r.abc_waits=rows;atomic_json(r.output/'ABC_WAITS.json',dict(waits=rows))
            status(r,seq,'ACCEPTING',unloaded=True)
            command,ack=receive(path,seq,responses)
            if command is None:trace.append(ack);continue
            op=command['operation'];started=time.perf_counter()
            status(r,seq,'RUNNING',operation=op,unloaded_decision=False)
            try:
                for action in command.get('segments',[]):validate_action(action);ops.articulation().validate_target(action)
                result=task_recovery.before(ops,command)
                if result is not None:pass
                elif op=='observe':result=ops.observe()
                elif op=='inspect':result=packet
                elif op=='evaluate_view':result=ops.evaluate_view(command['target']);ops.cache[seq]=result
                elif op=='move_and_observe':result=ops.move_and_observe(ops.cache.get(command['evaluated_command'],{}))
                elif op=='build_geometry':result=ops.request_plan(command)
                elif op=='prepare_grasp':result=ops.prepare_grasp(command)
                elif op=='export_matched_reference':
                    from .contact_fixture import export_reference
                    result=export_reference(ops,command)
                elif op=='select_observed_target':
                    from .target_association import accept_latest
                    result=accept_latest(ops,command)
                elif op=='observe_view':result=ops.observe_view(command)
                elif op=='redeclare_actions':result=ops.redeclare_actions(command)
                elif op=='validate_execution':result=ops.validate_execution()
                elif op=='attempt_grasp_and_probe':result=ops.attempt(command)
                elif op=='inspect_contact_result':result=ops.feedback()
                elif op=='retry_model':
                    from .validation_queue import retry_model
                    result=retry_model(ops,command)
                elif op=='extend_orchestration_wall':
                    from .orchestration_extension import handle
                    result=handle(ops,command,deadline)
                elif op in ('reobserve_and_replan','fresh_mode_observation'):result=ops.reobserve()
                elif op=='safe_release_retreat':result=ops.release_park()
                elif op=='evaluate_station':result=ops.evaluate_station(command);ops.cache[seq]=result
                elif op=='relocate_station':result=ops.relocate_station(ops.cache.get(command['evaluated_command'],{}))
                elif op=='relocate_observe_prepare':
                    from .recovery_batch import relocate_observe_prepare
                    result=relocate_observe_prepare(ops,command)
                elif op=='plan_panel_push':result=dict(valid=False,reason='Mobile support validation for intentional panel mode is not yet activated; handle reversal remains available')
                elif op=='finish':result=dict(safe_exit=ops.release_park())
                else:result=dict(executed=False,recoverable=True,reason='UNKNOWN_V81_OPERATION')
            except SafetyStop as e:result=dict(status='SAFETY_STOP',reason=e.reason,evidence=e.evidence)
            except ExecutionBudgetExhausted as e:
                result=dict(status='BUDGET_EXHAUSTED',reason=str(e),safe_exit=ops.release_park())
                if task_recovery.enabled(ops) and task_recovery.unloaded(ops):
                    available=task_recovery.remaining(ops)
                    if available['wall_s']>0 and available['sim_s']>0:
                        result.update(status='OPERATION_BUDGET_EXHAUSTED',recoverable=True,
                            episode_termination_required=False)
            except Exception as e:
                result=dict(operation_stopped=True,reason=repr(e),
                    recoverable=g['recovery_state']=='UNLOADED_OBSERVATION' and r.supervisor.latched is None)
                if op in ('prepare_grasp','build_geometry','validate_execution') and result['recoverable']:
                    from .replanning import state
                    state(ops)['state']='UNLOADED_REPLAN_REQUIRED'
                    result.update(status='PLANNING_REJECTED',episode_termination_required=False)
                elif task_recovery.enabled(ops):
                    if op in ('evaluate_view','observe_view','evaluate_station','redeclare_actions','retry_model') and task_recovery.unloaded(ops):
                        result.update(status='PLANNING_REJECTED',episode_termination_required=False)
                    else:
                        result.update(status='CONTROL_STATE_UNTRUSTED',recoverable=False)
            result=task_recovery.after(ops,command,result)
            entry=dict(command=command,result=result,wall_s=time.perf_counter()-started,unloaded_wait_s=started-began)
            complete(responses,seq,result_packet(entry));trace.append(entry);last=result
            atomic_json(r.output/'AGENT_ACTION_TRACE.json',result_packet(trace))
            status(r,seq,'COMPLETED',operation=op)
            if op=='finish' and result.get('status')!='CONTINUATION_REQUIRED':
                return dict(status='SESSION_COMPLETE',trace=trace,safe_exit=result.get('safe_exit'))
            if result.get('task_recovery',{}).get('episode_termination_required'):
                return dict(status=result['task_recovery']['status'],trace=trace)
            if r.supervisor.latched is not None or result.get('status') in ('SAFETY_STOP','BUDGET_EXHAUSTED'):
                return dict(status=result.get('status','SAFETY_STOP'),trace=trace)
        return dict(status='BOUNDED_OPERATION_BUDGET',safe_exit=ops.release_park(),trace=trace)
    finally:
        if hasattr(r,'orchestration_extension'):r.orchestration_extension.closed=True
        status(r,seq,'FINALIZING',reason='closing workers and deterministic actor export')
        try:ops.close_workers()
        finally:status(r,seq,'CLOSED',unloaded=g['recovery_state']=='UNLOADED_OBSERVATION')
