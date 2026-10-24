"""Read-only residual diagnosis and private processed-data cache. No plant IO."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np


def summarize(x):
    x=np.asarray(x);return dict(n=len(x),mean=float(np.mean(x)),rmse=float(np.sqrt(np.mean(x*x))),p05=float(np.quantile(x,.05)),p95=float(np.quantile(x,.95))) if len(x) else dict(n=0)


def run(stage,episode,output):
    stage,episode,output=map(Path,(stage,episode,output));output.mkdir(parents=True,exist_ok=True)
    sys.path[:0]=[str(stage),str(stage/'scripts')]
    from update_probing_v7_model import load_segments
    from xarm_ft_v7_estimator import samples,matrix
    from diagnose_probing_v3 import columns
    from agentic_interaction_v81.feedback import motion_errors
    assert json.loads((episode/'PROCESS_COMPLETION.json').read_text())['actor_exited']
    pieces,settings,provenance=load_segments(episode,allow_interrupted_fit=True)
    seal=json.loads((episode/'validation_model_seals/06.json').read_text());model=seal['selected']
    raw,index=columns(episode);cache={};metadata=[];diagnostics=[]
    for i,d in enumerate(pieces):
        rows=samples(d,settings['estimation']['domain_native'],settings['estimation']['intervals'])
        predicted=matrix(rows,model)@np.asarray(model['parameters']);residual=np.asarray([r['tau']for r in rows])-predicted
        q=np.array([r['q']for r in rows]);v=np.array([r['qd']for r in rows]);acc=np.array([r['qdd']for r in rows]);sid=int(d['block_id'].split('#')[0].rsplit('_',1)[1])
        group=dict(block_id=d['block_id'],split='FIT' if d['segments'][0].startswith('FIT')else'DEVELOPMENT_DIAGNOSTIC_OLD_VALIDATION',segment_id=sid,records=len(d['t']),eligible=int(d['valid'].sum()),q_range_deg=np.rad2deg([d['q'].min(),d['q'].max()]).tolist(),velocity_rad_s=summarize(v),acceleration_rad_s2=summarize(acc),residual_Nm=summarize(residual),bins=[])
        for direction in [-1,1]:
            for lo,hi in zip([0,15,30,45,60,75],[15,30,45,60,75,100]):
                mask=(np.sign(v)==direction)&(np.rad2deg(q)>=lo)&(np.rad2deg(q)<hi)
                if mask.any():group['bins'].append(dict(direction=direction,range_deg=[lo,hi],residual_Nm=summarize(residual[mask]),velocity_rad_s=summarize(v[mask]),acceleration_rad_s2=summarize(acc[mask])))
        ix=np.flatnonzero(raw['segment']==sid)
        if len(ix):
            errors=motion_errors(raw['command_TCP_pose'][ix,:3],raw['tcp_pose'][ix,:3],raw['joint_q'][ix],settings['prior'])
            group['measured_contact']=dict(max_servo_error_m=float(np.linalg.norm(errors['command_minus_measured_world_m'],axis=1).max()),max_rigid_arc_residual_m=float(np.linalg.norm(errors['measured_minus_rigid_articulation_world_m'],axis=1).max()),relative_slip_m=summarize(raw['relative_slip_m'][ix]),aperture_m=summarize(raw['aperture'][ix]),raw_pad_peak_n=raw['pad_load'][ix].max(0).tolist(),hard_stop_samples=int(raw['safety_stop'][ix].sum()))
        for field in ['t','q','qd','qdd','direction','native_qd','torque','unfiltered_torque','valid']:
            cache[f'p{i}_{field}']=d[field]
        metadata.append(dict(block_id=d['block_id'],segments=[str(d['segments'][0])],processing=d['processing'],split=group['split']))
        diagnostics.append(group)
    np.savez_compressed(output/'PROCESSED_PRIVATE.npz',**cache)
    (output/'PROCESSED_METADATA.json').write_text(json.dumps(dict(pieces=metadata,configuration=settings['estimation'],prior={k:v for k,v in settings['prior'].items()if k in ['joint_type','axis_world','origin_world_m']}),indent=2))
    report=dict(schema='v83-read-only-residual-diagnosis-v1',episode_id=episode.name,raw_evidence_private=True,hidden_physics_read=False,old_validation_reclassified_as_development=True,baseline_model_sha256=seal['model_sha256'],baseline_parameters=model['parameters'],segments=diagnostics,quality_provenance=provenance,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output/'DIAGNOSIS.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage');p.add_argument('episode');p.add_argument('output');a=p.parse_args();run(a.stage,a.episode,a.output)
