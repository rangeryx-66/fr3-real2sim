"""Read-only, measured 3-D load/path audit; no dynamics truth or fitting.

All vector signs follow the wrist interface (robot-on-object after existing
compensation). Native finger reactions are separately negated for comparison.
Point reports are impulse locations, never pressure distributions.
"""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from diagnose_probing_v3 import columns, rows_at
from update_probing_v2_model import matrix


def components(force, axis):
    axis=np.asarray(axis,float);axis=axis/np.linalg.norm(axis)
    rail=np.asarray(force)@axis
    return rail,np.asarray(force)-rail[...,None]*axis


def analyze(episode, output):
    p=Path(episode);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    if not json.loads((p/'PROCESS_COMPLETION.json').read_text()).get('actor_exited'):
        raise ValueError('SEALED_ACTOR_REQUIRED')
    a,index=columns(p);settings=json.loads((p/'identification_input.json').read_text())
    axis=np.asarray(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    result=dict(episode=p.name,read_only=True,hidden_parameters_read=False,
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                numeric_index_sha256=hashlib.sha256((p/'telemetry/index.json').read_bytes()).hexdigest(),segments=[])
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,len(index['segments']),figsize=(5*len(index['segments']),9),squeeze=False)
    for j,s in enumerate(index['segments']):
        ix=np.flatnonzero((a['step']>=s['first_step'])&(a['step']<=s['last_step']))
        rows=rows_at(a,ix,index['metadata'],'READ_ONLY_VECTOR_DIAGNOSTIC')
        R=np.array([r['wrist_ft']['T_world_sensor'][:3,:3]for r in rows])
        raw=a['raw_ft'][ix]-settings['bias']
        w=np.c_[np.einsum('nij,nj->ni',R,raw[:,:3]),np.einsum('nij,nj->ni',R,raw[:,3:])]
        w+=np.array([r['wrist_ft']['tool_gravity_wrench_world']for r in rows])
        rail,trans=components(w[:,:3],axis)
        tcp=a['tcp_pose'][ix,:3];cmd=a['command_TCP_pose'][ix,:3]
        relative=(tcp-tcp[0])-(a['joint_q'][ix]-a['joint_q'][ix[0]])[:,None]*axis
        tracking=(cmd-tcp)-(cmd[0]-tcp[0])
        dt=a['t'][ix]-a['t'][ix[0]]
        # Rotation is measured, not the selected grasp transform. This reveals
        # whether transverse loading follows a jaw axis or another constraint.
        rotations=np.array([matrix(v)[:3,:3]for v in a['tcp_pose'][ix]])
        local=np.einsum('nji,nj->ni',rotations,w[:,:3])
        stop=s.get('result',{}).get('load_margin',{}).get('t',a['t'][ix[-1]])
        tail=np.flatnonzero((a['t'][ix]>stop-.15)&(a['t'][ix]<=stop))
        row=dict(role=s['role'],status=s.get('result',{}).get('status'),
                 causal_guard_compensation='bias + tool gravity, world axes at instantaneous sensor origin',
                 tail_time_s=[float(a['t'][ix[tail[0]]]),float(a['t'][ix[tail[-1]]])],
                 tail_force_world_N=w[tail,:3].mean(0).tolist(),tail_force_TCP_N=local[tail].mean(0).tolist(),
                 tail_rail_N=float(rail[tail].mean()),tail_transverse_norm_N=float(np.linalg.norm(trans[tail],axis=1).mean()),
                 measured_TCP_minus_drawer_change_world_mm=(relative[-1]*1000).tolist(),
                 command_minus_measured_TCP_error_change_world_mm=(tracking[-1]*1000).tolist(),
                 nominal_command_change_transverse_world_mm=components(cmd[-1]-cmd[0],axis)[1].__mul__(1000).tolist(),
                 jaw_measured_change_mm=float((a['aperture'][ix[-1]]-a['aperture'][ix[0]])*1000),
                 tail_pad_normal_N=a['pad_load'][ix[tail]].mean(0).tolist())
        native=-(a['normal_force_on_fingers_world_N'][ix]+a['friction_force_on_fingers_world_N'][ix]).sum(1)
        valid=np.all(a['friction_api_valid'][ix]>0,axis=1)
        row['native_crosscheck']=dict(complete_wrench=False,primary_estimation_input=False,
            tail_force_world_N=native[tail][valid[tail]].mean(0).tolist() if valid[tail].any()else None,
            caution='buffer absence is not proof of separation; native contact APIs do not supply a complete interaction wrench')
        result['segments'].append(row)
        for k,label in enumerate('xyz'):
            axes[0,j].plot(dt,w[:,k],label='world '+label)
            axes[1,j].plot(dt,relative[:,k]*1000,label='TCP-drawer '+label)
            axes[2,j].plot(dt,tracking[:,k]*1000,label='command-measured '+label)
        axes[0,j].set_title(s['role']+' / '+str(row['status']))
        for k,label in enumerate(['Causal wrench force (N)','Relative motion (mm)','Tracking-error change (mm)']):
            axes[k,j].set_ylabel(label);axes[k,j].grid(alpha=.2);axes[k,j].legend(fontsize=7)
        axes[2,j].set_xlabel('Seconds from segment start')
    fig.tight_layout();fig.savefig(out/'VECTOR_LOAD_AND_MOTION.png',dpi=120);plt.close(fig)
    (out/'VECTOR_DIAGNOSIS.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(analyze(a.episode,a.output),indent=2))
