"""Finite Agent session with station requests at verified unloaded boundaries."""
import json,time
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
    commands=r.output/'agent_commands';responses=r.output/'agent_responses'
    commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True);trace=[];last=None;seq=-1
    try:
        from interaction_runtime.registered_observation_v71 import initialize
        initialize(r,ops)
        for seq in range(int(r.job.get('maximum_agent_operations',100))):
            if g['recovery_state']!='UNLOADED_OBSERVATION' or r.supervisor.latched is not None:
                return dict(status='HARD_STOP_OR_NO_SAFE_DECISION_BOUNDARY',trace=trace,stop=r.supervisor.latched)
            wait,resources=window(r)
            if wait<=0:return dict(status='BUDGET_EXHAUSTED',safe_exit=ops.release_park(),trace=trace)
            packet=ops.decision_packet(seq,last) if hasattr(ops,'decision_packet') else dict(
                sequence=seq,joint_coordinate=r.actual(),units=ops.articulation().units,
                remaining_resources=resources,feedback=ops.feedback(),unloaded=True)
            packet['station']=dict(generation=r.base_generation,measured_base=r.base.tolist(),
                operations=['evaluate_station','relocate_station'],actuation='native driven XY stage')
            atomic_json(r.output/'DECISION_PACKET.json',packet)
            deadline=time.time()+wait
            status(r,seq,'WAITING',unloaded=True,deadline_unix_s=deadline)
            atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=seq,unloaded=True,timeout_s=wait,
                deadline_unix_s=deadline,requires_ACK_before_execution=True,
                live_status_file='AGENT_SESSION_STATUS.json'))
            began=time.perf_counter();path=commands/('%03d.json'%seq)
            heartbeat=began
            while not path.exists():
                if r.stopped or (r.output/'STOP_AND_EXPORT').exists() or time.perf_counter()-began>=wait:
                    status(r,seq,'FINALIZING',unloaded=True,reason='decision deadline or owner stop')
                    return dict(status='ORCHESTRATION_TIMEOUT' if not r.stopped else 'OWNER_STOP',
                        safe_exit=ops.release_park(),trace=trace)
                if time.perf_counter()-heartbeat>=5:
                    status(r,seq,'WAITING',unloaded=True,deadline_unix_s=deadline)
                    heartbeat=time.perf_counter()
                time.sleep(.05)
            status(r,seq,'ACCEPTING',unloaded=True)
            command,ack=receive(path,seq,responses)
            if command is None:trace.append(ack);continue
            op=command['operation'];started=time.perf_counter()
            status(r,seq,'RUNNING',operation=op,unloaded_decision=False)
            try:
                for action in command.get('segments',[]):validate_action(action);ops.articulation().validate_target(action)
                if op=='observe':result=ops.observe()
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
            except ExecutionBudgetExhausted as e:result=dict(status='BUDGET_EXHAUSTED',reason=str(e),safe_exit=ops.release_park())
            except Exception as e:
                result=dict(operation_stopped=True,reason=repr(e),
                    recoverable=g['recovery_state']=='UNLOADED_OBSERVATION' and r.supervisor.latched is None)
                if op in ('prepare_grasp','build_geometry','validate_execution') and result['recoverable']:
                    from .replanning import state
                    state(ops)['state']='UNLOADED_REPLAN_REQUIRED'
                    result.update(status='PLANNING_REJECTED',episode_termination_required=False)
            entry=dict(command=command,result=result,wall_s=time.perf_counter()-started,unloaded_wait_s=started-began)
            complete(responses,seq,result_packet(entry));trace.append(entry);last=result
            atomic_json(r.output/'AGENT_ACTION_TRACE.json',result_packet(trace))
            status(r,seq,'COMPLETED',operation=op)
            if op=='finish':return dict(status='SESSION_COMPLETE',trace=trace,safe_exit=result.get('safe_exit'))
            if r.supervisor.latched is not None or result.get('status') in ('SAFETY_STOP','BUDGET_EXHAUSTED'):
                return dict(status=result.get('status','SAFETY_STOP'),trace=trace)
        return dict(status='BOUNDED_OPERATION_BUDGET',safe_exit=ops.release_park(),trace=trace)
    finally:
        status(r,seq,'FINALIZING',reason='closing workers and deterministic actor export')
        try:ops.close_workers()
        finally:status(r,seq,'CLOSED',unloaded=g['recovery_state']=='UNLOADED_OBSERVATION')
