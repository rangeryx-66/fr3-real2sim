"""One closed-episode mechanism replay; no physics or hidden plant reads."""
import argparse,hashlib,json,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .monitor import factory,progress_coordinate
from interaction_runtime.articulation_v7 import Articulation
from interaction_runtime.probing_profiles_v3 import speed_fraction
from agent_command_protocol_v12 import atomic_json


def replay(episode,output):
    p=Path(episode);out=Path(output)
    if not (p/'PROCESS_COMPLETION.json').exists():raise ValueError('CLOSED_EPISODE_REQUIRED')
    if out.exists():raise ValueError('NEW_OUTPUT_REQUIRED')
    idx=json.loads((p/'telemetry/index.json').read_text());parts=[];hashes={}
    keys=['t','segment','joint_q','reference_q','tcp_pose','command_TCP_pose','phase']
    for ch in idx['chunks']:
        path=p/'telemetry'/ch['file'];h=hashlib.sha256(path.read_bytes()).hexdigest()
        if h!=ch['sha256']:raise ValueError('CHUNK_HASH_MISMATCH')
        hashes[ch['file']]=h
        with np.load(path) as x:parts.append({k:x[k] for k in keys})
    data={k:np.concatenate([x[k] for x in parts]) for k in keys}
    prior=json.loads((p/'ARTICULATION_INTERFACE.json').read_text())['prior']
    info=json.loads((p/'ARTICULATION_INTERFACE.json').read_text())['SI']
    a=Articulation(prior['joint_type'],tuple(prior['axis_world']),tuple(prior['origin_world_m']),tuple(info['limits_native']))
    phases=idx['metadata']['phases'];active=phases.index('OPEN_CONTINUE');results=[]
    for s in idx['segments']:
        ix=np.flatnonzero((data['segment']==s['id'])&(data['phase']==active))
        if not len(ix):continue
        t=data['t'][ix];q=data['joint_q'][ix];ref=data['reference_q'][ix]
        P=data['tcp_pose'][ix,:3];C=data['command_TCP_pose'][ix,:3]
        direction=1 if s['target_q']>=q[0] else -1
        pose=np.eye(4);pose[:3,3]=P[0];r=SimpleNamespace(tcp=lambda:pose.copy());captures=[]
        m=factory(r,a,captures)(q[0],direction,s['declared_action'])
        for j in range(len(ix)):
            pose[:3,3]=P[j];scale=a.path_scale(P[j]);action=s['declared_action'];local=t[j]-t[0]
            speed=min(.012,float(action.get('reference_speed_m_s',.012)))*speed_fraction(local,action)
            state=dict(lag_rad=direction*(ref[j]-q[j]),requested_rate_rad_s=direction*speed/scale)
            m.update(float(local),float(q[j]),state,1/240,direction*(s['target_q']-q[j]))
        tcp=np.unwrap([progress_coordinate(x,P[0],a) for x in P])+q[0]
        cmd=np.unwrap([progress_coordinate(x,P[0],a) for x in C])+q[0]
        row=dict(id=s['id'],role=s['role'],actual_status=s.get('result',{}).get('status'),
            replay_scope='OPEN_CONTINUE records; phase/clock reconstructed, not a replacement physical result',
            original_nominal_windows=sum(x['nominal_suspect'] for x in m.history),
            advisory_windows=sum(x['advisory'] for x in m.history),
            actual_bad_windows=sum(x['suspect'] for x in m.history),phase_components={})
        for name,sel in [('early',(t-t[0]>=1)&(t-t[0]<4)),('late',t>=t[-1]-3)]:
            row['phase_components'][name]={k:float(np.median(v[sel])) for k,v in {
                'reference_object_native':direction*(ref-q),'TCP_object_native':direction*(tcp-q),
                'command_TCP_native':direction*(cmd-tcp),'reference_command_native':direction*(ref-cmd)}.items()}
        results.append(row)
    result=dict(schema='v84-closed-mechanism-replay',episode_id=p.name,
        raw_private=True,chunk_hashes=hashes,segments=results,
        original_lead_and_frozen_source_unchanged=True,hidden_physics_read=False,
        counterfactual_is_not_a_physical_success=True)
    out.mkdir(parents=True);atomic_json(out/'REPLAY.json',result);return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(json.dumps(replay(a.episode,a.output),indent=2))
