"""Measurement-only numeric worker. No scene generator, object masses or GT torque."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from xarm_wrist_ft import tool_gravity_world
from xarm_ft_id_estimator import process
from xarm_ft_v1_estimator import regression_samples,fit,forward,coverage

def matrix(x):
    T=np.eye(4);T[:3,3]=x[:3];T[:3,:3]=Rotation.from_quat(x[3:]).as_matrix();return T

def decode(folder):
    index=json.loads((folder/'telemetry/index.json').read_text());settings=json.loads((folder/'identification_input.json').read_text());meta=index['metadata']
    complete={s['id']:s for s in index['segments'] if s['completed']};groups={sid:[] for sid in complete}
    for c in index['chunks']:
        with np.load(folder/'telemetry'/c['file'],allow_pickle=False) as a:
            ids=np.flatnonzero(np.isin(a['segment'],list(complete)))
            for i in ids:
                sid=int(a['segment'][i]);phase=meta['phases'][int(a['phase'][i])]
                if phase not in ['OPEN_CONTINUE','PANEL_PUSH']:continue
                bodies=[]
                for prop,pose in zip(meta['body_properties'],a['tool_poses'][i]):
                    T=matrix(pose);R=T[:3,:3];bodies.append(dict(name=prop['name'],mass_kg=prop['mass_kg'],T_world_body=T,com_world_m=R@prop['com_body_m']+T[:3,3],inertia_world_kg_m2=R@np.asarray(prop['inertia_body_kg_m2'])@R.T))
                T=matrix(a['sensor_pose'][i]);row=dict(t=float(a['t'][i]),door_angle_deg=float(np.rad2deg(a['door_q'][i])),phase=phase,
                    identification_segment=complete[sid]['role']+'_%02d'%sid,relative_translation_slip_m=float(a['relative_slip_m'][i]),forces_n=dict(L=float(a['pad_load'][i,0]),R=float(a['pad_load'][i,1])),
                    force_event=dict(status='HARD_STOP' if a['safety_stop'][i] else 'NORMAL'),informational_load_warning=bool(a['load_warning'][i]),
                    wrist_ft=dict(valid=bool(a['ft_valid'][i]),raw_wrench_sensor=a['raw_ft'][i],T_world_sensor=T,body_states=bodies,tool_gravity_wrench_world=tool_gravity_world(bodies,T[:3,3])))
                groups[sid].append(row)
    return groups,settings,complete

def update(folder):
    groups,settings,segments=decode(folder);pieces=[];provenance=[]
    for sid,rows in groups.items():
        # Each complete segment is processed independently, before any filtering.
        if len(rows)<247:provenance.append(dict(id=sid,records=len(rows),status='insufficient segment-local derivative support'));continue
        if np.max(abs(np.diff([r['t'] for r in rows])-1/240))>1e-6:
            provenance.append(dict(id=sid,status='sampling gap; retained but not smoothed across'));continue
        d=process(rows,settings['prior'],settings['bias']);d['valid'][:121]=False;d['valid'][-121:]=False;d['samples']=regression_samples(d);pieces.append(d)
        provenance.append(dict(id=sid,role=segments[sid]['role'],records=len(rows),valid=int(d['valid'].sum()),angle_range_deg=[min(r['door_angle_deg'] for r in rows),max(r['door_angle_deg'] for r in rows)]))
    result=dict(model='tau=I_eff*qdd+b*qd+C*sign(qd)',measurement='native six-axis wrist F/T, bias + tool gravity/inertia compensation, known hinge prior',
        hidden_dynamic_parameters_used=False,segment_local_derivatives=True,complete_validation_segments_excluded_from_fitting=True,segments=provenance,
        uncertainty='conditional, not calibrated hardware confidence',status='insufficient informative data')
    data=None
    if pieces:
        keys=['t','q','qd','qdd','torque','valid','segments'];data={k:np.concatenate([d[k] for d in pieces]) for k in keys};samples=sum([d['samples'] for d in pieces],[])
        train=[s for s in samples if s['segment'].startswith('FIT')];test=[s for s in samples if s['segment'].startswith('VALIDATION')]
        result.update(training_bins=len(train),validation_bins=len(test),coverage=coverage(data),measured_torque_peak_nm=float(np.max(abs(data['torque']))))
        if len(train)>=8:
            model=fit(train,'constant');result.update(status='updated',constant_model=model)
            model['parameter_names']=['I_eff_kg_m2','b_Nm_s_rad','C_Nm'];model['supported']=[bool(x and model['algebraic_rank']==3) for x in model['supported']]
            if test:
                X=np.array([[s['qdd'],s['qd'],s['direction']] for s in test]);y=np.array([s['tau'] for s in test]);model['heldout_torque_rmse_nm']=float(np.sqrt(np.mean((X@model['parameters']-y)**2)))
            model['heldout_motion']={str(label):forward(data,np.flatnonzero(data['segments']==label),model) for label in dict.fromkeys(data['segments']) if str(label).startswith('VALIDATION')}
            ix=data['valid'];rms=[float(np.sqrt(np.mean(data[k][ix]**2))) for k in ['qdd','qd']]
            result['excitation']=dict(acceleration_rms_rad_s2=rms[0],velocity_rms_rad_s=rms[1],inertia_torque_rms_nm=model['parameters'][0]*rms[0],damping_torque_rms_nm=model['parameters'][1]*rms[1],archived_sensor_residual_floor_nm=.004644)
            for j,key in enumerate(['inertia_torque_rms_nm','damping_torque_rms_nm']):
                if result['excitation'][key]<.004644:model['supported'][j]=False
    modeldir=folder/'model';modeldir.mkdir(exist_ok=True);atomic_json(modeldir/'estimate.json',result)
    history=modeldir/'updates.json';updates=json.loads(history.read_text()) if history.exists() else [];updates.append(result);atomic_json(history,updates)
    if data is not None:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(3,1,figsize=(10,7),sharex=True)
        for label in dict.fromkeys(data['segments']):
            ix=data['segments']==label;ax[0].plot(data['t'][ix],np.rad2deg(data['q'][ix]),label=label);ax[1].plot(data['t'][ix],data['torque'][ix],lw=.6);ax[2].plot(data['t'][ix],data['qd'][ix],lw=.6)
        ax[0].legend(fontsize=7);ax[0].set_ylabel('actual deg');ax[1].set_ylabel('F/T hinge torque Nm');ax[2].set_ylabel('rad/s');ax[2].set_xlabel('simulation seconds')
        for a in ax:a.grid(alpha=.2)
        fig.tight_layout();fig.savefig(modeldir/'feedback.png',dpi=100);plt.close(fig)
    return result

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('folder',type=Path);x=a.parse_args();print(json.dumps(update(x.folder)))
