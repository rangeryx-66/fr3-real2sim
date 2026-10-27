"""Reuse V8.1 unloaded dispatcher; bounded loaded C continues real physics."""
import time
from unittest.mock import patch
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet
from agentic_interaction_v81.session_status import write as status
from .operations import contact_trusted
from .waits import wait


def loaded_command(ops,sequence):
    """Finite observed C boundary; no passive freeze while waiting for Agent."""
    r=ops.r;start=time.monotonic();sim=r.tick*r.dt
    resources=r.episode_budget.remaining(sim)
    wall_cap=min(20.,resources['wall_s']);sim_cap=min(5.,resources['sim_s'])
    deadline=time.time()+wall_cap
    try:
        packet=result_packet(ops.packet())
        atomic_json(r.output/'ABC_CONTACT_PACKET.json',packet)
        status(r,sequence,'WAITING',unloaded=False,abc_contact=True,deadline_unix_s=deadline)
        atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=sequence,unloaded=False,
            abc_contact=True,timeout_s=wall_cap,deadline_unix_s=deadline))
    except (TypeError,ValueError,OSError) as error:
        # Only the read-only packet/publication boundary is caught here, never
        # hold(), actuation or a safety exception. Do not lose guarded exit to
        # a presentation/JSON error while the contact controller is healthy.
        if r.supervisor.latched is not None:raise
        ops._abc_retain=False;exit_result=ops.release_park()
        atomic_json(r.output/('ABC_C_METADATA_RECOVERY_%03d.json'%sequence),
            result_packet(dict(error=repr(error),safe_exit=exit_result,
                original_goal_preserved=True,metadata_error_not_controller_fault=True)))
        return None
    path=r.output/'agent_commands'/('%03d.json'%sequence)
    expired=False
    with wait(r,'GUARDED_CONTACT_CRITIQUE','Agent continuation or finite supervised unload',sequence=sequence):
        while not path.exists():
            if (r.stopped or (r.output/'STOP_AND_EXPORT').exists()
                    or time.monotonic()-start>=wall_cap or r.tick*r.dt-sim>=sim_cap
                    or (not contact_trusted(ops) and not ops.g.get('v87_hold_pending'))):
                expired=True;break
            try:
                r.hold(.05)
            except RuntimeError as error:
                from agentic_interaction_v87.constraint_feedback import enabled, PREFIX
                from interaction_runtime.segments import ExecutionBudgetExhausted
                resource_end=isinstance(error,ExecutionBudgetExhausted)
                if not resource_end and (not enabled(r) or not str(error).startswith(PREFIX)):
                    raise
                ops._abc_retain=False
                exit_result=ops.release_park()
                atomic_json(r.output/('ABC_C_CONTROLLED_RECOVERY_%03d.json'%sequence),
                    result_packet(dict(reason=str(error),safe_exit=exit_result,
                        original_goal_preserved=True,contact_hold_expired=False)))
                return None
    if expired:
        ops._abc_retain=False;exit_result=ops.release_park()
        atomic_json(r.output/('ABC_C_TIMEOUT_%03d.json'%sequence),
            dict(safe_exit=exit_result,contact_hold_expired=True,original_goal_preserved=True))
        return None
    return True


def serve(r):
    """Install only scoped opt-in front-end dispatch; frozen runtime untouched."""
    import agentic_interaction_v81.session as owner
    from .operations import operations_class
    from . import transport
    # V81 session imports task_recovery locally; bind the actual module.
    import agentic_interaction_v81.task_recovery as recovery
    before,after=recovery.before,recovery.after
    real_factory=owner.operations_class
    def factory(parent):
        result=operations_class(real_factory(parent))
        if r.job.get('v87_staged_exit'):
            from agentic_interaction_v87.staged_exit import operations_class as staged
            result=staged(result)
        if r.job.get('v87_graspgenx'):
            from agentic_interaction_v87.hybrid_grasp import operations_class as hybrid
            result=hybrid(result)
        if r.job.get('v88_coordination_policy'):
            from agentic_interaction_v88.operations import operations_class as coordinate
            result=coordinate(result)
        return result
    def before_command(ops,command):
        if ops.abc_state=='C' and command['operation'] not in ('continue_motion','safe_release_retreat','finish','inspect'):
            return dict(executed=False,status='CONTACT_BOUNDARY_OPERATION_NOT_ALLOWED')
        if command['operation']=='continue_motion':
            return ops.continue_motion(command)
        if command['operation']=='acquire_execute':
            blocked=before(ops,dict(command,operation='attempt_grasp_and_probe'))
            if blocked is not None:return blocked
            return ops.attempt(command)
        if command['operation']=='finish' and r.job.get('v84_short_comparison'):
            exercised={s.get('declared_action',{}).get('execution_strategy','continuous')
                for s in r.record.segments if s.get('completed')}
            if {'legacy','continuous'}<=exercised:
                recovery.state(ops)['termination_assessment']=dict(allowed=True,
                    reason='PREREGISTERED_FINITE_SHORT_COMPARISON_FINISHED',
                    task_complete=False,global_goal_preserved=True)
                return None
        return before(ops,command)
    def after_command(ops,command,result):
        if (ops.abc_state=='C' and r.supervisor.latched is None
                and result.get('outcome') not in ('UNRECOVERABLE','SAFETY_STOP')
                and result.get('status') not in ('CONTROL_STATE_UNTRUSTED','SAFETY_STOP')):
            result['task_recovery']=dict(status='LOADED_CRITIQUE',episode_termination_required=False,
                original_goal_native=r.job['v81_task_goal_native'])
            result['abc']=ops.packet()
            return result
        return after(ops,command,result)
    # V81 dispatcher remains authoritative for every unloaded observation,
    # native plan/exit/base route, failure and final cleanup. Its next boundary
    # is extended explicitly for finite supervised C through the seam below.
    outcome=None
    try:
        with patch.object(owner,'operations_class',factory),patch.object(owner,'receive',transport.receive),\
             patch.object(recovery,'before',before_command),patch.object(recovery,'after',after_command):
            outcome=owner.serve(r)
            return outcome
    finally:
        import sys
        from agentic_interaction_v81.terminal_safety import persist
        original_error=sys.exc_info()[1]
        try:persist(r,outcome,pause=True)
        except Exception as export_error:
            if original_error is None:raise
            original_error.add_note('Terminal safety finalization failed: '+repr(export_error))
