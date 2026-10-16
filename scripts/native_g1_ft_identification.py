"""Minimal V0 segment/Agent handoff around the frozen grasp and safe parking."""
import inspect,json,pickle,subprocess,time,traceback,sys
from pathlib import Path
from unittest.mock import patch
import numpy as np
import native_g1_contact_recovery as recovery
import native_g1_safe_termination as termination
from xarm_wrist_ft import install_passive
from native_g1_wrist_input import save_rgbd


def kinematic_prior(g):
    import xml.etree.ElementTree as ET
    manifest=g['model'].manifest
    # Explicit known-axis prior from the already allowed asset kinematic model.
    path=g['job']['asset_root']+'/urdf/'+g['job']['identification_asset']+'.urdf'
    j=ET.parse(path).getroot().find("joint[@name='%s']"%manifest['joint_name'])
    local=np.array(list(map(float,j.find('axis').get('xyz').split())))
    T=g['body_at'](0.)
    return dict(joint_type='revolute',axis_world=(T[:3,:3]@local).tolist(),origin_world_m=T[:3,3].tolist(),
        source='explicit KNOWN_HINGE_STATE prior; no hidden dynamics',angle_source='allowed actual articulation state V0',geometry_reference=g['job']['asset_root'])


def held_step(g,wall_start):
    g['opening']=False;g['qvelocity']=np.zeros(len(g['arm']));g['step']()
    # Keep physics live while the Agent/worker reasons. No load-bearing pause.
    time.sleep(max(0.,g['dt']-(time.perf_counter()-wall_start)))


def packet(g,index):
    folder=g['a'].output/'identification'/('attempt_%02d'%g['attempt_index'])/('%02d'%index);folder.mkdir(parents=True,exist_ok=True)
    rows=g['rows'][g['identification_first_row']:]
    with (folder/'measurements.pkl').open('wb') as f:pickle.dump(rows,f,protocol=5)
    settings=dict(prior=kinematic_prior(g),bias=g['job']['ft_calibration']['bias_sensor'])
    (folder/'INPUT.json').write_text(json.dumps(settings,indent=2))
    save_rgbd(g,'identification_state_%02d'%index)
    command=[sys.executable,str(g['ROOT']/'scripts/run_xarm_ft_estimation.py'),str(folder)]
    with (folder/'worker.log').open('w') as log:
        worker=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        start=time.perf_counter()
        while worker.poll() is None:
            held_step(g,time.perf_counter())
            if time.perf_counter()-start>90:
                worker.terminate();worker.wait(timeout=10);break
    result=dict(folder=str(folder),worker_returncode=worker.returncode,sim_s=g['tick']*g['dt'],angle_deg=float(np.rad2deg(g['actual']())),
        pad_loads_n=g['filtered']().tolist(),physics_live_during_analysis=True)
    (folder/'PACKET_READY.json').write_text(json.dumps(result,indent=2))
    return result


def bidirectional(g,open_function):
    records=[];origin=g['opening_start_angle'];g['identification_first_row']=len(g['rows']);g['id_direction']=1.
    segment=0;next_action=dict(target_gain_deg=3.,speed_scale=.6,role='FIT_OPEN_0')
    root=g['a'].output;commands=root/'interaction_commands'/('attempt_%02d'%g['attempt_index']);commands.mkdir(parents=True,exist_ok=True)
    while next_action:
        target=origin+np.deg2rad(float(next_action['target_gain_deg']));direction=1. if target>=g['actual']() else -1.
        name=next_action.get('role',('FIT' if direction>0 else 'VALIDATION')+'_%02d'%segment)
        g['identification_segment']=name;g['id_direction']=direction;g['motion_scale']=float(next_action['speed_scale']);g['goal']=target
        g['angle_reference']=g['actual']();g['opening']=True;g['twist']=np.zeros(6)
        # Same contact-aware policy, in a monotonic progress coordinate for
        # closing. The low-level target is still native joint/IK arm motion.
        # No object articulation command or artificial force is introduced.
        actual=g['actual']
        # Sign-normalize only progress comparisons, never state measurements,
        # force guards or body transforms. At direction=+1 this is the original.
        source=Path(g['spec']['shared_policy']).read_text().replace("g['qvelocity']=np.zeros(6)","g['qvelocity']=np.zeros(7)")
        source=source.replace("maximum=g['actual']();start=maximum", "maximum=direction*g['actual']();start=maximum")
        source=source.replace('return theta,drift','return direction*theta,drift')
        source=source.replace("while g['actual']()<g['goal']:","while direction*g['actual']()<direction*g['goal']:")
        source=source.replace("g['actual']()-start<probe","direction*g['actual']()-start<probe")
        source=source.replace("g['body_at'](theta)","g['body_at'](direction*theta)")
        source=source.replace("maximum=g['actual']();checkpoint", "maximum=direction*g['actual']();checkpoint")
        namespace={'direction':direction};exec(compile(source,__file__+'::directional_progress_only','exec'),namespace)
        try:
            ok,status,events=namespace['open_with_contact_feedback'](g)
        finally:
            g['opening']=False;g['qvelocity']=np.zeros(len(g['arm']));g['identification_segment']=None
        result=dict(segment=name,action=next_action,ok=bool(ok),status=status,final_angle_deg=float(np.rad2deg(actual())))
        records.append(result);(root/('IDENTIFICATION_SEGMENTS_%02d.json'%g['attempt_index'])).write_text(json.dumps(records,indent=2))
        if not ok:return False,status,records
        g['phase']='IDENTIFICATION_STATIC_PAUSE';g['hold'](1.)
        view=packet(g,segment);(root/'WAITING_FOR_INTERACTION_AGENT.json').write_text(json.dumps(dict(sequence=segment,result=result,packet=view,
            physics_live=True,command_path=str(commands/('%03d.json'%segment)),maximum_segments=5,remaining=4-segment,permitted='change direction/speed/range <=10deg or stop; original protections unchanged'),indent=2))
        if segment>=4:break
        path=commands/('%03d.json'%segment);began=time.perf_counter()
        while not path.exists():
            held_step(g,time.perf_counter())
            if time.perf_counter()-began>120:return False,'AGENT_INTERACTION_TIMEOUT_SAFE_EXIT',records
            if (root/'STOP_AND_EXPORT').exists():raise KeyboardInterrupt('OWNER_STOP')
        next_action=json.loads(path.read_text());(root/'INTERACTION_AGENT_TRACE.jsonl').open('a').write(json.dumps(dict(sequence=segment,command=next_action,sim_s=g['tick']*g['dt']))+'\n')
        if next_action.get('stop'):break
        if not (.25<=float(next_action['target_gain_deg'])<=10 and .25<=float(next_action['speed_scale'])<=1):
            return False,'INVALID_INTERACTION_COMMAND_SAFE_EXIT',records
        segment+=1
    return True,'BOUNDED_IDENTIFICATION_INTERACTIONS_COMPLETE',records


def run(g):
    install_passive(g);g['identification_segment']=None;observe=g['observe_step']
    def sample(state,contacts):
        observe(state,contacts);state['identification_segment']=g.get('identification_segment')
    g['observe_step']=sample
    # Minimal owner operation substitution, not a second grasp controller.
    source=inspect.getsource(recovery.attempt_grasp_and_probe).replace('def attempt_grasp_and_probe(', 'def attempt_identification(')
    anchor="ok,status,_=namespace['open_with_contact_feedback'](g)"
    if source.count(anchor)!=1:raise ValueError('Frozen attempt boundary changed before contact')
    source=source.replace(anchor,"g['_id_frozen_step']=g['step'];ok,status,_=bidirectional(g,namespace['open_with_contact_feedback'])")
    ns=dict(recovery.__dict__,bidirectional=bidirectional,safe_release_retreat=termination.safe_release_park)
    exec(compile(source,__file__+'::explicit_id_attempt','exec'),ns)
    with patch.object(recovery,'attempt_grasp_and_probe',ns['attempt_identification']):return termination.run(g)
