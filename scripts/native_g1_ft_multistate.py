"""Bounded temporal interaction blocks; frozen grasp/Pink/protections unchanged."""
import json,pickle,subprocess,sys,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
import native_g1_ft_identification as v0
from native_g1_wrist_input import save_rgbd
from xarm_ft_v1_scene import install_native_law


def directional_policy(g,direction):
    source=Path(g['spec']['shared_policy']).read_text().replace("g['qvelocity']=np.zeros(6)","g['qvelocity']=np.zeros(7)")
    changes={"maximum=g['actual']();start=maximum":"maximum=direction*g['actual']();start=maximum",
        'return theta,drift':'return direction*theta,drift',"while g['actual']()<g['goal']:":"while direction*g['actual']()<direction*g['goal']:",
        "g['actual']()-start<probe":"direction*g['actual']()-start<probe","g['body_at'](theta)":"g['body_at'](direction*theta)",
        "maximum=g['actual']();checkpoint":"maximum=direction*g['actual']();checkpoint"}
    for old,new in changes.items():source=source.replace(old,new)
    ns={'direction':direction};exec(compile(source,__file__+'::same_directional_progress','exec'),ns);return ns['open_with_contact_feedback']


def smooth(x):
    x=np.clip(x,0.,1.);return float(x*x*(3-2*x))


def profile(action,elapsed):
    base=float(action.get('reference_speed_m_s',.0015*action.get('speed_scale',1.)))
    ramp=.25+.75*smooth(elapsed/2.)
    variation=1. if action.get('profile','constant')=='constant' else .3+.7*np.sin(np.pi*elapsed/2.5)**2
    return base*ramp*variation


def segment(g,action,index):
    """Only temporal scaling of the same measured-progress Pink/controller."""
    theta=float(g['actual']());target=float(np.deg2rad(action['target_deg']))
    direction=1. if target>=theta else -1.;name=action['role']+'_%02d'%index
    original=g['step'];began=g['tick']*g['dt'];g.update(identification_segment=name,id_direction=direction,goal=target,
        angle_reference=theta,opening=True,twist=np.zeros(6),motion_scale=1.)
    def profiled_step():
        # Reference timing only. The Pink update factor remains exactly 1;
        # no joint-drive, IK gain, effort or physical safety threshold changes.
        g['identification_speed_m_s']=profile(action,g['tick']*g['dt']-began)
        g['identification_acceleration_m_s2']=float(action.get('reference_acceleration_m_s2',.002))
        return original()
    g['step']=profiled_step
    try:
        ok,status,events=directional_policy(g,direction)(g)
        if ok:
            # Smoothly reduce the drive after the target crossing. Retain the
            # original acquired grasp/reference; do not reset it to hide slip.
            start_speed=g['identification_speed_m_s'];g['goal']=float(g['actual']());g['angle_reference']=g['goal']
            for j in range(int(2./g['dt'])):
                g['identification_speed_m_s']=start_speed*(1-smooth((j+1)*g['dt']/2.));original()
        result=dict(action=action,label=name,ok=bool(ok),status=status,start_deg=float(np.rad2deg(theta)),end_deg=float(np.rad2deg(g['actual']())),simulation_s=g['tick']*g['dt']-began)
    finally:
        g['step']=original;g['opening']=False;g['qvelocity']=np.zeros(len(g['arm']));g['identification_segment']=None
    if action.get('hold',True):g['phase']='IDENTIFICATION_STATIC_PAUSE';g['hold'](1.)
    return result


def packet(g,block,first_row,previous):
    folder=g['a'].output/'identification_v1'/('attempt_%02d'%g['attempt_index'])/('block_%02d'%block);folder.mkdir(parents=True,exist_ok=True)
    with (folder/'measurements.pkl').open('wb') as f:pickle.dump(g['rows'][first_row:],f,protocol=5)
    (folder/'INPUT.json').write_text(json.dumps(dict(prior=v0.kinematic_prior(g),bias=g['job']['ft_calibration']['bias_sensor'],previous_packets=previous,candidate_blocks=g['job']['information_action_menu']),indent=2))
    save_rgbd(g,'identification_v1_attempt_%02d_block_%02d'%(g['attempt_index'],block))
    command=[sys.executable,str(g['ROOT']/'scripts/run_xarm_ft_v1_estimation.py'),str(folder)]
    with (folder/'worker.log').open('w') as log:
        worker=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT);began=time.perf_counter()
        while worker.poll() is None:
            v0.held_step(g,time.perf_counter())
            if time.perf_counter()-began>90:
                worker.terminate();worker.wait(timeout=10);break
    result=dict(folder=str(folder),worker_returncode=worker.returncode,sim_s=g['tick']*g['dt'],current_angle_deg=float(np.rad2deg(g['actual']())),passive_telemetry_failure_does_not_abort_manipulation=True)
    (folder/'PACKET_READY.json').write_text(json.dumps(result,indent=2));return result


def interact(g,unused):
    records=[];previous=[];first_row=len(g['rows']);index=0;root=g['a'].output
    block=g['job']['initial_information_block']
    for block_index in range(3):
        for action in block:
            # Bounded planner safety uses the current grasp/door state, not a
            # blanket 30deg certification or a new margin/contact criterion.
            result=segment(g,action,index);records.append(result);index+=1
            (root/('MULTISTATE_SEGMENTS_%02d.json'%g['attempt_index'])).write_text(json.dumps(records,indent=2))
            if not result['ok']:return False,result['status'],records
        info=packet(g,block_index,first_row,previous);previous.append(info['folder']);first_row=len(g['rows'])
        commands=root/'interaction_commands_v1'/('attempt_%02d'%g['attempt_index']);commands.mkdir(parents=True,exist_ok=True);path=commands/('block_%02d.json'%block_index)
        (root/'WAITING_FOR_V1_AGENT.json').write_text(json.dumps(dict(block=block_index,packet=info,command_path=str(path),physics_live=True,
            timeout_wall_s=120,permitted='choose a finite menu block or stop; original guards and max20deg registered envelope',attempt=g['attempt_index']),indent=2))
        if block_index>=2:break
        began=time.perf_counter()
        while not path.exists():
            v0.held_step(g,time.perf_counter())
            if time.perf_counter()-began>120:return False,'V1_AGENT_TIMEOUT_SAFE_EXIT',records
            if (root/'STOP_AND_EXPORT').exists():raise KeyboardInterrupt('OWNER_STOP')
        action=json.loads(path.read_text());(root/'V1_AGENT_ACTIONS.jsonl').open('a').write(json.dumps(dict(block=block_index,command=action,sim_s=g['tick']*g['dt']))+'\n')
        if action.get('stop'):break
        block=action['segments']
        if len(block)>4 or any(not (0<=float(a['target_deg'])<=20 and .25<=float(a['speed_scale'])<=1 and 0<float(a.get('reference_speed_m_s',.0015))<=.0035 and 0<float(a.get('reference_acceleration_m_s2',.002))<=.006 and a.get('profile','constant') in ['constant','smooth_variation']) for a in block):return False,'INVALID_V1_MENU_SAFE_EXIT',records
    return True,'BOUNDED_MULTISTATE_INTERACTIONS_COMPLETE',records


def run(g):
    native_update=install_native_law(g);original=g['step']
    def passive_scene_step():native_update();return original()
    g['step']=passive_scene_step
    with patch.object(v0,'bidirectional',interact):return v0.run(g)
