"""Explicit approval boundary around the frozen Agent observation prototype.

No view policy, contact scoring or manipulation controller is changed. Return
and approach use the existing collision checker, .05 margin and actuators.
"""
import inspect,json,time
from pathlib import Path
import numpy as np
import native_g1_agent_observation as observation
import native_g1_visual_input as visual


def check_path(g,path):
    samples=0;minimum=float('inf')
    kwargs=dict(finger_q=np.asarray(g['robot'].get_joint_positions())[g['fingers']].copy()) if g.get('execution_measured_fingers',False) else dict(width=.084)
    for a,b in zip(path,path[1:]):
        a,b=np.asarray(a),np.asarray(b)
        for q in np.linspace(a,b,max(2,int(np.ceil(np.max(abs(b-a))/.025))+1)):
            samples+=1;minimum=min(minimum,g['model'].margin(q))
            if g['model'].margin(q)<=.05:return dict(valid=False,reason='LOW_JOINT_MARGIN',samples=samples)
            ok,why=g['collision'].check(g['model'].poses(q,g['base'],**kwargs),g['moving'](),False)
            if not ok:return dict(valid=False,reason=why,samples=samples)
    return dict(valid=True,samples=samples,minimum_margin_rad=minimum)


def approach_from(g,start,candidate):
    # Same straight pregrasp-to-grasp geometry and IK branch as frozen initial.
    pre=np.asarray(candidate['T']).copy();pre[:3,3]-=.04*pre[:3,2]
    path=[np.asarray(start).tolist()]+candidate['preplan'][1:]
    seed=np.asarray(path[-1]);seed=g['ik'](pre,seed)
    if seed is None:return dict(valid=False,reason='NO_PREGRASP_IK')
    approach=[]
    for f in np.linspace(0,1,21):
        target=pre.copy();target[:3,3]=(1-f)*pre[:3,3]+f*np.asarray(candidate['T'])[:3,3]
        seed=g['ik'](target,seed)
        if seed is None:return dict(valid=False,reason='NO_APPROACH_IK')
        approach.append(seed.tolist())
    audit=check_path(g,path+approach)
    return dict(audit,preplan=path,approach=approach)


def validate_execution(g,candidate=None,write=True):
    started=time.perf_counter();p=g['a'].output/'visual_plan.json'
    if candidate is None:
        if not p.exists():return dict(valid=False,reason='No current visual plan')
        candidate=json.loads(p.read_text())
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    # Include both actual q and the still-active equilibrium command; no reset.
    path=[measured.tolist(),g['qtarget'][g['arm']].tolist()]
    for route in reversed(g['wrist_routes']):path+=list(reversed(route['q_path']))
    path.append(candidate['preplan'][0])
    returning=check_path(g,path);approach=approach_from(g,path[-1],candidate)
    result=dict(valid=returning['valid'] and approach['valid'],return_path=path,return_audit=returning,approach_audit=approach,measured_start_q=measured.tolist(),wall_s=time.perf_counter()-started,source='current episode routes and fresh visual plan; not saved successful views/grasps')
    if write:observation.write(g['a'].output/'AGENT_EXECUTION_TRANSITION_CHECK.json',result)
    return result


def execute(g,command):
    # This method is called only by an explicit Codex command after inspecting
    # fresh fused surfaces AND the planned native gripper visualization.
    check=validate_execution(g)
    if not check['valid']:raise RuntimeError('AGENT_EXECUTION_RETURN_OR_APPROACH_UNAVAILABLE')
    observation.write(g['a'].output/'AGENT_EXECUTION_APPROVED.json',dict(command=command,t_sim=g['tick']*g['dt'],check=check))
    g['world'].play();g['phase']='PREGRASP';g['profile_mark']('agent_return_to_execution_start')
    for q in check['return_path'][2:]:g['move'](np.asarray(q),.8)
    g['hold'](g['wrist_config']['settle_s'])
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    candidate=json.loads((g['a'].output/'visual_plan.json').read_text())
    revalidated=approach_from(g,measured,candidate)
    # Recheck command-equilibrium to the first target too; no position reset.
    bridge=check_path(g,[g['qtarget'][g['arm']].tolist()]+revalidated.get('preplan',[]))
    observation.write(g['a'].output/'MEASURED_STATE_APPROACH_CHECK.json',dict(revalidated,command_bridge=bridge,measured_q=measured.tolist(),t_sim=g['tick']*g['dt']))
    if not revalidated['valid'] or not bridge['valid']:raise RuntimeError('MEASURED_STATE_APPROACH_UNAVAILABLE')
    candidate.update(preplan=revalidated['preplan'],approach=revalidated['approach'])
    data=json.loads((g['a'].output/'agent_geometry/observed_geometry.json').read_text())
    seed=Path(next(r['folder'] for r in g['agent_observations'] if r['accepted_for_geometry']))
    # Frozen visual state assignment only; no recapture, rescore, or GT poses.
    ns=dict(visual.__dict__,save_rgbd=lambda state,label:seed,segment=lambda *args:None,plan_observed=lambda state,folder:(candidate,data))
    exec(inspect.getsource(visual.prepare),ns);ns['prepare'](g)
    boundary=json.loads((g['a'].output/'visual_source_boundary.json').read_text())
    boundary.update(sensing='Codex-selected actuator-executed rigid wrist RGB-D',scene_camera_depth_used=False,contact_scores='frozen measured open patches plus separate rear hypotheses',GT_handle_surface_used=False)
    observation.write(g['a'].output/'visual_source_boundary.json',boundary)
    return dict(approved=True,returned_with_actuators=True,measured_approach_revalidated=True,t_sim=g['tick']*g['dt'],next='frozen approach/refinement/closure/one-second hold/short opening')


def prepare(g):
    # Keep the existing observation/action interface verbatim. Only introduce
    # explicit validate/execute operations and a finite Agent wait deadline.
    source=inspect.getsource(observation.prepare)
    source=source.replace("cache={};trace=[];sequence=0", "cache={};trace=[];sequence=0;executed=False")
    source=source.replace("while not path.exists():", "while not path.exists():\n            if time.perf_counter()-wait_started>g['job']['agent_wait_timeout_s']:raise KeyboardInterrupt('AGENT_INPUT_TIMEOUT_SAFE_EXPORT')")
    source=source.replace("elif op=='finish':", "elif op=='validate_execution':result=validate_execution(g)\n            elif op=='execute':\n                result=execute(g,command);executed=True\n            elif op=='finish':")
    source=source.replace("except Exception as error:\n            g['world'].pause();result=dict(error=repr(error),scene_preserved=True)", "except Exception as error:\n            g['world'].pause();result=dict(error=repr(error),scene_preserved=True)\n            if op=='execute':\n                observation.write(root/'AGENT_EXECUTION_FAILURE.json',dict(command=command,result=result));raise")
    source=source.replace("if op=='finish':break", "if op=='finish' or executed:break")
    source=source.replace("grasp_attempts=0,object_opening_attempts=0", "execution_handed_to_frozen_skill=executed,grasp_and_opening_outcome='see final report/telemetry; handoff is not success'")
    source=source.replace("raise RuntimeError('AGENT_OBSERVATION_COMPLETE_NO_MANIPULATION_ATTEMPT')", "if not executed:raise RuntimeError('AGENT_STOPPED_BEFORE_MANIPULATION')")
    ns=dict(observation.__dict__,observation=observation,validate_execution=validate_execution,execute=execute)
    exec(compile(source,__file__+'::frozen_observation_explicit_execution','exec'),ns);ns['prepare'](g)
