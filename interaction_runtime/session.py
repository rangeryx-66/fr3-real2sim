"""Explicit operation table and existing atomic protocol; waits only unloaded."""
import json,time
from pathlib import Path
from agent_command_protocol_v12 import receive,complete,atomic_json
from .operations import Operations
from .serialization import result_packet

def serve(r):
    ops=Operations(r);g=r.g;commands=r.output/'agent_commands';responses=r.output/'agent_responses'
    commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True);seq=0;trace=[]
    from native_g1_wrist_views import survey_candidates,select_route
    import numpy as np
    proposal=next(survey_candidates(r.model,g['wrist_config']))
    route,audit=select_route(r.model,r.solver,r.collision,r.base,np.asarray(r.robot.get_joint_positions())[r.arm],[proposal],g['wrist_config'],g['wrist_X'])
    if route is None:return dict(status='INITIAL_VIEW_UNAVAILABLE',audit=audit)
    r.move_path(route['q_path'][1:],'PREGRASP');r.hold(g['wrist_config']['settle_s']);g['wrist_routes'].append(route)
    first=ops.observe();atomic_json(r.output/'INITIAL_OBSERVATION_READY.json',first)
    while True:
        if g['recovery_state']!='UNLOADED_OBSERVATION':return dict(status='SAFE_UNLOADED_BOUNDARY_UNAVAILABLE',trace=trace)
        atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=seq,verified_unloaded=True,timeout_s=120,scene_preserved=True))
        path=commands/('%03d.json'%seq);began=time.perf_counter()
        while not path.exists():
            if r.stopped or (r.output/'STOP_AND_EXPORT').exists() or time.perf_counter()-began>120:return dict(status='BOUNDED_UNLOADED_AGENT_STOP',trace=trace)
            time.sleep(.05)
        command,ack=receive(path,seq,responses)
        if command is None:trace.append(ack);seq+=1;continue
        op=command['operation'];start=time.perf_counter()
        try:
            if op=='observe':result=ops.observe()
            elif op=='inspect':
                from native_g1_agent_observation import inspect
                result=inspect(Path(g['agent_observations'][-1]['folder']),g['agent_observations'])
            elif op=='evaluate_view':result=ops.evaluate_view(command['target']);ops.cache[seq]=result
            elif op=='move_and_observe':result=ops.move_and_observe(ops.cache.get(command['evaluated_command'],{}))
            elif op=='build_geometry':result=ops.build_geometry()
            elif op=='validate_execution':result=ops.validate_execution()
            elif op=='attempt_grasp_and_probe':result=ops.attempt(command)
            elif op=='safe_release_retreat':result=ops.release_park()
            elif op=='fresh_mode_observation' or op=='reobserve_and_replan':
                result=ops.reobserve()
            elif op=='plan_panel_push':result=ops.plan_push(command)
            elif op=='execute_panel_push':g['recovery_state']='LOADED_OPERATION';result=ops.push()
            elif op=='inspect_contact_result':result=dict(events=r.events,last_sample=dict(t=r.rows[-1]['t'],angle_deg=r.rows[-1]['door_angle_deg'],pad_loads=r.rows[-1]['forces_n']))
            elif op=='finish':
                exit_result=ops.release_park();result=dict(completed=bool(exit_result['safe']),safe_exit=exit_result)
            else:raise ValueError('No explicit operation handler')
        except BaseException as e:
            result=dict(error=repr(e),current_operation_stopped=True);r.g['qvelocity']=np.zeros(7)
            complete(responses,seq,result_packet(dict(command=command,result=result)));trace.append(result)
            return dict(status='OPERATION_STOP',trace=trace)
        entry=dict(command=command,result=result,wall_s=time.perf_counter()-start,wait_wall_s=start-began);complete(responses,seq,result_packet(entry));trace.append(entry)
        atomic_json(r.output/'AGENT_ACTION_TRACE.json',result_packet(trace))
        if op=='finish':return dict(status='SESSION_COMPLETE',trace=trace)
        seq+=1
