"""Small file-command interface for a real Codex-in-the-loop observation run.

This module does not choose next views. Codex inspects actual images externally
and supplies camera eye/look targets. Deterministic projection/IK/collisions
and frozen actuators execute them. No grasp or opening is executed here.
"""
import json,os,subprocess,time
from pathlib import Path
import numpy as np
from native_g1_wrist_input import install_camera,save_rgbd
from native_g1_wrist_views import survey_candidates,select_route,look_at,in_frame
from native_g1_agent_inspection import inspect
import native_g1_multiview_input as multi
import native_g1_metric_rgbd_geometry as metric

def write(path,data):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(data,indent=2));tmp.replace(path)

def observe(g):
    started=time.perf_counter();folder=save_rgbd(g,'agent_%02d'%len(g['agent_observations']))
    sam=g['job']['wrist_experiment']['sam3'];root=g['ROOT']
    command=[sam['python'],str(root/'scripts/infer_sam3_agent_observed.py'),'--folder',str(folder),'--prompt',g['job']['visual_prompt'],'--checkpoint',sam['checkpoint']]
    if not g['job'].get('_visual_proposal_used',False):command+=['--allow-proposal']
    env=dict(os.environ,PYTHONPATH=str(root)+':'+str(root/'scripts')+':'+sam['pythonpath'],CUDA_VISIBLE_DEVICES=str(g['job']['visual_gpu']),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1')
    with (folder/'sam3.log').open('w') as log:
        status=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
    detection=folder/'visual_detection_attempts.json'
    if detection.exists() and len(json.loads(detection.read_text())['attempts'])>1:g['job']['_visual_proposal_used']=True
    accepted=False;issue=None
    if (folder/'handle_mask.npy').exists():
        try:
            from native_g1_observed_regions import extend_observed_mask
            raw=np.load(folder/'handle_mask.npy');np.save(folder/'sam3_mask_raw.npy',raw)
            np.save(folder/'handle_mask.npy',raw&~np.load(folder/'robot_self_exclusion.npy'))
            extend_observed_mask(folder);v=metric.preserve(folder);metric.save_view(v,folder/'measured_geometry');accepted=True
        except Exception as error:issue=repr(error)
    row=dict(index=len(g['agent_observations']),folder=str(folder),t_sim=g['tick']*g['dt'],world_sim_s=float(g['world'].current_time),accepted_for_geometry=accepted,SAM_returncode=status,geometry_issue=issue,sensing_and_segmentation_wall_s=time.perf_counter()-started)
    g['agent_observations'].append(row)
    packet=inspect(folder,g['agent_observations']);row['inspection_wall_s']=packet['inspection_wall_s']
    write(g['a'].output/'AGENT_OBSERVATIONS.json',g['agent_observations']);return dict(observation=row,inspection=str(folder/'INSPECTION.json'),image_board=str(folder/'agent_inspection.png'))

def evaluate_view(g,target):
    C=np.asarray(target['T_camera'],float) if 'T_camera' in target else look_at(np.asarray(target['eye_world_m'],float),np.asarray(target['look_at_world_m'],float))
    q=np.asarray(g['robot'].get_joint_positions())[g['arm']];points=None
    if g['agent_observations']:
        p=Path(g['agent_observations'][-1]['folder'])/'cue_points_world.npy'
        if p.exists():points=np.load(p)
    route,audit=select_route(g['model'],g['solver'],g['collision'],g['base'],q,[dict(kind='explicit_Codex_view_request',T_camera=C)],g['wrist_config'],g['wrist_X'],points if points is not None and len(points) else None)
    return dict(feasible=route is not None,route=route,audit=audit,target=target)

def actuate(g,route,label):
    before=g['tick']*g['dt'];started=time.perf_counter()
    # Existing PREGRASP means unloaded/no intentional handle contact, including
    # the original runtime collision semantics. No low-level controller change.
    g['phase']='PREGRASP';g['profile_mark']('agent_wrist_motion')
    for q in route['q_path'][1:]:g['move'](np.asarray(q),.8)
    g['hold'](g['wrist_config']['settle_s'])
    g['wrist_routes'].append(dict(route,label=label,t_start=before,t_end=g['tick']*g['dt'],motion_wall_s=time.perf_counter()-started,measured_q=np.asarray(g['robot'].get_joint_positions())[g['arm']].tolist(),base=list(g['base'])))
    write(g['a'].output/'WRIST_ROBOT_TRAJECTORIES.json',g['wrist_routes'])

def move_and_observe(g,route):
    g['world'].play();actuate(g,route,'agent_adjustment_%d'%(g['agent_moves']+1));g['agent_moves']+=1
    result=observe(g);g['world'].pause();return result

def build_geometry(g):
    folders=[Path(r['folder']) for r in g['agent_observations'] if r['accepted_for_geometry']]
    if not folders:return dict(geometry=False,plan=False,reason='NO_ACCEPTED_COMPLETE_MASK; partial cues never become grasp masks')
    started=time.perf_counter();data=multi.build_geometry(folders[0],folders[1:],g['a'].output/'agent_geometry')
    # Capture-source metadata only; frozen geometric/scoring assumptions reused.
    data['capture_source']='Agent-selected actuator-executed rigid wrist RGB-D';data['spec']=dict(data['spec'],sensing=data['capture_source'])
    (g['a'].output/'agent_geometry/observed_geometry.json').write_text(json.dumps(data,indent=2))
    geom_wall=time.perf_counter()-started;started=time.perf_counter()
    try:
        candidate,_=multi.plan_observed(g,g['a'].output/'agent_geometry')
        result=dict(geometry=True,plan=candidate is not None,geometry_data=data,geometry_wall_s=geom_wall,planning_wall_s=time.perf_counter()-started,grasp_plan_origin='registered station/home; observation route can be reversed; no approach/grasp executed',GT_grasp_fallback=False)
    except Exception as error:result=dict(geometry=True,plan=False,geometry_data=data,geometry_wall_s=geom_wall,planning_wall_s=time.perf_counter()-started,reason=repr(error),GT_grasp_fallback=False)
    write(g['a'].output/'AGENT_GEOMETRY_PLAN_RESULT.json',result);return result

def prepare(g):
    install_camera(g);g.update(agent_observations=[],agent_moves=0);g['job']['_visual_proposal_used']=False
    root=g['a'].output;commands=root/'agent_commands';responses=root/'agent_responses';commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True)
    # Only the initial pose is predetermined; no fixed survey after observing.
    proposal=next(survey_candidates(g['model'],g['wrist_config']));q=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    route,audit=select_route(g['model'],g['solver'],g['collision'],g['base'],q,[proposal],g['wrist_config'],g['wrist_X'])
    write(root/'INITIAL_VIEW_PLAN.json',dict(route=route,audit=audit,source='unchanged coarse cabinet survey initializer, not saved successful view'))
    if route is None:raise RuntimeError('INITIAL_WRIST_VIEW_UNAVAILABLE')
    actuate(g,route,'initial');first=observe(g);g['world'].pause();write(root/'INITIAL_OBSERVATION_READY.json',first)
    cache={};trace=[];sequence=0
    while True:
        wait_started=time.perf_counter()
        write(root/'WAITING_FOR_AGENT.json',dict(sequence=sequence,pid=os.getpid(),world_paused=True,scene_preserved=True,adjustments_used=g['agent_moves'],adjustment_budget=4,latest_observation=g['agent_observations'][-1]))
        path=commands/('%03d.json'%sequence)
        while not path.exists():
            if g.get('_owner_stop_requested') or (root/'STOP_AND_EXPORT').exists():raise KeyboardInterrupt('OWNER_REQUESTED_PAUSE_EXPORT')
            if not g['app'].is_running():raise RuntimeError('SIMULATOR_CLOSED')
            g['app'].update();time.sleep(.2)
        command=json.loads(path.read_text());started=time.perf_counter();op=command['operation'];result=None
        try:
            if op=='evaluate_view':
                result=evaluate_view(g,command['target']);cache[sequence]=result
            elif op=='move_and_observe':
                evaluated=cache.get(command['evaluated_command'])
                if g['agent_moves']>=4:result=dict(executed=False,reason='registered four-adjustment observation budget exhausted')
                elif not evaluated or not evaluated['feasible']:result=dict(executed=False,reason='requested view has no existing collision/IK route; choose another')
                else:result=move_and_observe(g,evaluated['route'])
            elif op=='observe':result=observe(g)
            elif op=='inspect':result=inspect(Path(g['agent_observations'][-1]['folder']),g['agent_observations'])
            elif op=='build_geometry':result=build_geometry(g)
            elif op=='finish':result=dict(completed=True,reason=command.get('reason'),grasp_or_opening_executed=False)
            else:result=dict(error='unknown observation operation')
        except Exception as error:
            g['world'].pause();result=dict(error=repr(error),scene_preserved=True)
        entry=dict(sequence=sequence,command=command,agent_wait_wall_s=started-wait_started,operation_wall_s=time.perf_counter()-started,result=result)
        trace.append(entry);write(root/'AGENT_ACTION_TRACE.json',trace);write(responses/('%03d.json'%sequence),entry)
        if op=='finish':break
        sequence+=1
    write(root/'AGENT_EPISODE_RESULT.json',dict(observations=g['agent_observations'],agent_adjustments=g['agent_moves'],actual_robot_routes=len(g['wrist_routes']),grasp_attempts=0,object_opening_attempts=0,trace=str(root/'AGENT_ACTION_TRACE.json'),scene_restarts=0,GT_handle_or_scene_camera_input=False))
    raise RuntimeError('AGENT_OBSERVATION_COMPLETE_NO_MANIPULATION_ATTEMPT')
