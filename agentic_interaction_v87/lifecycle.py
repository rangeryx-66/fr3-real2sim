"""The session owns World lifetime. A hard action fault owns no destroy call.

Latched finite simulator states remain paused for read-only diagnosis. This is
not a verified hardware brake or parking. No reset/clear operation is exposed.
"""
import json,time
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def finite_state(r):
    try:
        q=np.asarray(r.robot.get_joint_positions());dq=np.asarray(r.robot.get_joint_velocities())
        return bool(q.size and dq.size and np.isfinite(np.r_[q,dq,r.actual()]).all())
    except Exception:return False


def snapshot(r):
    return dict(episode_id=r.job['episode_id'],instance_uuid=r.job.get('instance_uuid'),
        status='SAFETY_LATCHED_WORLD_RETAINED',fault=r.supervisor.latched,
        measured_object_coordinate_native=float(r.actual()),World_identity=id(r.world),
        world_recreated=False,object_reset=False,motion_authorized=False,
        simulation_paused=not r.world.is_playing(),
        physical_park_verified=False,hardware_stop_not_inferred=True,
        reset_qualification=dict(qualified=False,
            reason='NO_VALIDATED_RESET_OWNER; latch retained and every motion operation denied'))


def retain(ops,sequence,trace,last):
    r=ops.r
    if not finite_state(r):
        return dict(status='UNRECOVERABLE_SIMULATION_STATE',trace=trace,
            safe_exit=dict(safe=False,reason='NONFINITE_OR_UNREADABLE_NATIVE_STATE'))
    if r.supervisor.latched is None:
        uncertain=last.get('outcome')=='UNRECOVERABLE'
        r.supervisor.latched=dict(reason=last.get('reason')or(
            'CONTROLLED_STOP_OR_STATE_UNCONFIRMED'if uncertain else'UNCONFIRMED_HARD_STOP'),
            evidence=last.get('recovery')if uncertain else last.get('evidence'),
            source='unconfirmed controller state; not a measured force/speed hard event'if uncertain else'original action safety exception')
    r.world.pause()
    initial=snapshot(r);folder=r.output/'safety_events';folder.mkdir(exist_ok=True)
    event=folder/('hard_stop_%03d.json'%sequence)
    if not event.exists():
        with event.open('x')as f:json.dump(result_packet(initial),f,indent=2)
    from agentic_interaction_v81.session_status import write as status
    from agentic_interaction_v84.transport import receive
    from agentic_interaction_v81.transport import complete
    commands=r.output/'agent_commands';responses=r.output/'agent_responses'
    while True:
        resources=r.episode_budget.remaining(r.tick*r.dt)
        wait=max(0.,float(resources['wall_s']))
        deadline=time.time()+wait
        packet=snapshot(r);packet.update(sequence=sequence,remaining_resources=resources)
        atomic_json(r.output/'DECISION_PACKET.json',packet)
        status(r,sequence,'WAITING',unloaded=False,abc_contact=False,
            diagnostic_only=True,safety_latched=True,deadline_unix_s=deadline)
        atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=sequence,unloaded=False,
            diagnostic_only=True,timeout_s=wait,deadline_unix_s=deadline))
        path=commands/('%03d.json'%sequence)
        while not path.exists():
            if r.stopped or (r.output/'STOP_AND_EXPORT').exists() or time.time()>=deadline:
                return dict(status='LATCHED_DIAGNOSTIC_RESOURCE_OR_OWNER_END',trace=trace,
                    safe_exit=dict(safe=False,reason='HARD_LATCH_RETAINED_NOT_PARKED'),
                    retained_diagnostic_boundary=True)
            time.sleep(.05)
        command,ack=receive(path,sequence,responses)
        if command is None:trace.append(ack);sequence+=1;continue
        if not finite_state(r):
            return dict(status='UNRECOVERABLE_SIMULATION_STATE',trace=trace,
                safe_exit=dict(safe=False,reason='NATIVE_STATE_BECAME_UNTRUSTED'))
        if command['operation']=='inspect':result=snapshot(r)
        elif command['operation']=='finish':
            result=dict(status='OWNER_FINALIZED_HARD_STOP',safe_exit=dict(safe=False,
                reason='HARD_LATCH_RETAINED_NOT_PARKED'),task_complete=False,latch_cleared=False)
        else:result=dict(status='HARD_LATCH_MOTION_FORBIDDEN',executed=False,
            latch_cleared=False,world_retained=True,episode_termination_required=False)
        entry=dict(command=command,result=result)
        complete(responses,sequence,result_packet(entry));trace.append(entry)
        atomic_json(r.output/'AGENT_ACTION_TRACE.json',result_packet(trace))
        if command['operation']=='finish':
            return dict(status='HARD_STOP',trace=trace,safe_exit=result['safe_exit'])
        sequence+=1
