"""Reuse V8.1 unloaded dispatcher; bounded loaded C continues real physics."""
import time
from unittest.mock import patch
from agent_command_protocol_v12 import atomic_json
from agentic_interaction_v81.session_status import write as status
from .operations import contact_trusted
from .waits import wait


def loaded_command(ops,sequence):
    """Finite observed C boundary; no passive freeze while waiting for Agent."""
    r=ops.r;start=time.monotonic();sim=r.tick*r.dt
    resources=r.episode_budget.remaining(sim)
    wall_cap=min(20.,resources['wall_s']);sim_cap=min(5.,resources['sim_s'])
    deadline=time.time()+wall_cap
    packet=ops.packet();atomic_json(r.output/'ABC_CONTACT_PACKET.json',packet)
    status(r,sequence,'WAITING',unloaded=False,abc_contact=True,deadline_unix_s=deadline)
    atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=sequence,unloaded=False,
        abc_contact=True,timeout_s=wall_cap,deadline_unix_s=deadline))
    path=r.output/'agent_commands'/('%03d.json'%sequence)
    with wait(r,'GUARDED_CONTACT_CRITIQUE','Agent continuation or finite supervised unload',sequence=sequence):
        while not path.exists():
            if (r.stopped or (r.output/'STOP_AND_EXPORT').exists()
                    or time.monotonic()-start>=wall_cap or r.tick*r.dt-sim>=sim_cap
                    or not contact_trusted(ops)):
                ops._abc_retain=False;exit_result=ops.release_park()
                atomic_json(r.output/('ABC_C_TIMEOUT_%03d.json'%sequence),
                    dict(safe_exit=exit_result,contact_hold_expired=True,
                        original_goal_preserved=True))
                return None
            r.hold(.05)
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
        return operations_class(real_factory(parent))
    def before_command(ops,command):
        if ops.abc_state=='C' and command['operation'] not in ('continue_motion','safe_release_retreat','finish','inspect'):
            return dict(executed=False,status='CONTACT_BOUNDARY_OPERATION_NOT_ALLOWED')
        if command['operation']=='continue_motion':
            return ops.continue_motion(command)
        if command['operation']=='acquire_execute':
            blocked=before(ops,dict(command,operation='attempt_grasp_and_probe'))
            if blocked is not None:return blocked
            return ops.attempt(command)
        return before(ops,command)
    def after_command(ops,command,result):
        if ops.abc_state=='C' and r.supervisor.latched is None:
            result['task_recovery']=dict(status='LOADED_CRITIQUE',episode_termination_required=False,
                original_goal_native=r.job['v81_task_goal_native'])
            result['abc']=ops.packet()
            return result
        return after(ops,command,result)
    # V81 dispatcher remains authoritative for every unloaded observation,
    # native plan/exit/base route, failure and final cleanup. Its next boundary
    # is extended explicitly for finite supervised C through the seam below.
    with patch.object(owner,'operations_class',factory),patch.object(owner,'receive',transport.receive),\
         patch.object(recovery,'before',before_command),patch.object(recovery,'after',after_command):
        return owner.serve(r)
