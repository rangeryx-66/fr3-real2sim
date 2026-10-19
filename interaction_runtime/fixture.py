"""Short reusable contact regression using cached measured surfaces, not cached grasps.

Only one pre-approved finite pull is executed; the Agent still inspects the
fresh panel RGB-D and selects its ROI. No full active-perception episode runs.
"""
import json,time,shutil
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import receive,complete,atomic_json
from .operations import Operations
from .serialization import result_packet

def serve_fixture(r):
    ops=Operations(r);g=r.g;fixture=Path(r.job['engineering_fixture'])
    audit=json.loads((fixture/'FIXTURE_RESULT.json').read_text())
    if not audit['ready']:return dict(status='OFFLINE_FIXTURE_UNAVAILABLE')
    geometry=Path(audit['geometry']);shutil.copytree(geometry,r.output/'agent_geometry',dirs_exist_ok=True)
    shutil.copy2(fixture/'visual_plan.json',r.output/'visual_plan.json')
    prior=Path(r.job['engineering_rgbd_reference'])
    g['agent_observations']=[dict(folder=str(prior),accepted_for_geometry=True,source='archived measured RGB-D engineering fixture, not a fresh perception result')]
    atomic_json(r.output/'ENGINEERING_FIXTURE_READY.json',dict(plan_source='new automatic re-ranking from measured RGB-D; no saved successful grasp',fixture=audit,GT_handle_grasp_input=False))
    commands=r.output/'agent_commands';responses=r.output/'agent_responses';commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True)
    trace=[];seq=0
    while seq<6:
        atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=seq,verified_unloaded=True,scope='SHORT_NATIVE_RUNTIME_REGRESSION',timeout_s=120))
        path=commands/('%03d.json'%seq);began=time.perf_counter()
        while not path.exists():
            if time.perf_counter()-began>120 or r.stopped:return dict(status='BOUNDED_FIXTURE_STOP',trace=trace)
            time.sleep(.05)
        command,ack=receive(path,seq,responses)
        if command is None:trace.append(ack);seq+=1;continue
        op=command['operation'];start=time.perf_counter()
        try:
            if op=='attempt_grasp_and_probe':
                r.event('SHORT_CONTACT_TEST_START');acquired=ops.acquire();segment=ops.interact(command['segments'][0]['target_deg'],max_sim_s=30);exited=ops.release_park();result=dict(acquisition=acquired,pull=segment,safe_exit=exited)
                if not exited['safe']:raise RuntimeError('SHORT_CONTACT_SAFE_EXIT_UNAVAILABLE')
            elif op=='fresh_mode_observation':
                from native_g1_wrist_views import survey_candidates,select_route
                from native_g1_hybrid_contact import current_view_scene
                from native_g1_wrist_input import save_rgbd
                proposal=next(survey_candidates(r.model,g['wrist_config']));route,audit=select_route(r.model,r.solver,current_view_scene(g),r.base,np.asarray(r.robot.get_joint_positions())[r.arm],[proposal],g['wrist_config'],g['wrist_X'])
                if route is None:result=dict(observed=False,reason='CURRENT_PANEL_VIEW_UNAVAILABLE',audit=audit)
                else:
                    r.move_path(route['q_path'][1:],'PREGRASP');r.hold(.5);folder=save_rgbd(g,'engineering_panel_observation');g['agent_observations'].append(dict(folder=str(folder),accepted_for_geometry=False,source='fresh actual wrist motion and metric RGB-D'))
                    result=dict(observed=True,folder=str(folder),no_scene_camera_depth=True)
            elif op=='plan_panel_push':result=ops.plan_push(command)
            elif op=='execute_panel_push':result=ops.push()
            elif op=='safe_release_retreat':result=ops.release_park()
            elif op=='finish':
                exit_result=ops.release_park();result=dict(completed=bool(exit_result['safe']),safe_exit=exit_result)
            else:result=dict(error='Operation outside short fixture scope')
        except BaseException as error:
            import traceback
            result=dict(error=repr(error),traceback=traceback.format_exc(),continued_after_safety_exception=False)
            complete(responses,seq,result_packet(dict(command=command,result=result)));trace.append(result)
            return dict(status='SHORT_CONTACT_REGRESSION_STOP',trace=trace)
        entry=dict(command=command,result=result,wall_s=time.perf_counter()-start);complete(responses,seq,result_packet(entry));trace.append(entry);atomic_json(r.output/'AGENT_ACTION_TRACE.json',result_packet(trace))
        if op=='finish':return dict(status='SHORT_CONTACT_REGRESSION_COMPLETE',trace=trace)
        seq+=1
    return dict(status='BOUNDED_SHORT_CONTACT_END',trace=trace)
