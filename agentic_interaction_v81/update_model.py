"""Unchanged production fit plus measured FIT-readiness sidecar.

Captures references to already loaded/qualified pieces; no second raw load,
different sample selection, compensation, filtering or model coefficients.
"""
import argparse,hashlib,json,sys,time
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def update(folder,output=None,allow_interrupted_fit=False):
    import numpy as np
    import update_probing_v7_model as frozen
    from xarm_ft_v31_estimator import regression_samples
    from agent_command_protocol_v12 import atomic_json
    from agentic_interaction_v81.excitation import information
    folder=Path(folder);out=Path(output or folder/'model')
    settings=json.loads((folder/'identification_input.json').read_text())
    captured=[];load=frozen.load_segments
    def capture(*args,**kwargs):
        pieces,config,provenance=load(*args,**kwargs)
        captured.extend(pieces)
        return pieces,config,provenance
    began=time.perf_counter()
    with patch.object(frozen,'load_segments',capture):
        if settings['prior']['joint_type']=='prismatic':
            from update_prismatic_v73_model import update as qualified_update
            result=qualified_update(folder,out,allow_interrupted_fit)
        else:result=frozen.update(folder,out,allow_interrupted_fit)
    groups=[];rows=[]
    for piece in captured:
        if not piece['segments'][0].startswith('FIT'):continue
        data=regression_samples(piece);rows.extend(data)
        groups.append(dict(block=piece['block_id'],eligible_samples=int(piece['valid'].sum()),
            regression_bins=len(data),positive_bins=sum(r['qd']>1e-4 for r in data),
            negative_bins=sum(r['qd']<-1e-4 for r in data),
            coordinate_support_native=[float(np.min(piece['q'][piece['valid']])),float(np.max(piece['q'][piece['valid']]))] if piece['valid'].any() else None))
    positive=sum(r['qd']>1e-4 for r in rows);negative=sum(r['qd']<-1e-4 for r in rows)
    trajectories={r['segment'] for r in rows}
    completed=[s['id'] for s in result['segments'] if s['role'].startswith('FIT') and s['quality'].get('action_status') is not None]
    ready=dict(schema='v81-measured-FIT-readiness',estimator_version=result['estimator_version'],
        episode_id=json.loads((folder/'job.json').read_text())['episode_id'],
        numeric_index_sha256=hashlib.sha256((folder/'telemetry/index.json').read_bytes()).hexdigest(),
        model_sha256=hashlib.sha256((out/'estimate.json').read_bytes()).hexdigest(),
        latest_completed_FIT_id=max(completed,default=0),groups=groups,
        positive_bins=positive,negative_bins=negative,distinct_eligible_trajectories=len(trajectories),
        qualified_bidirectional=positive>=8 and negative>=8 and len(trajectories)>=2,
        rule='At least eight production regression bins in each measured velocity direction and two distinct FIT trajectories; experimental readiness, not a physical safety threshold',
        validation_used=False,production_coefficients_modified=False,raw_data_loaded_twice=False,
        processing_wall_s=time.perf_counter()-began)
    if len(rows)>=8:
        ready['information']=information([r['qdd'] for r in rows],[r['qd'] for r in rows],
            [r['direction'] for r in rows],settings['estimation']['parameter_scales'])
    atomic_json(out/'FIT_READINESS_V81.json',ready)
    return result,ready


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--allow-interrupted-fit',action='store_true');a=p.parse_args()
    result,readiness=update(a.episode,a.output,a.allow_interrupted_fit)
    print(json.dumps(dict(status=result['status'],qualified_bidirectional=readiness['qualified_bidirectional'],
        positive_bins=readiness['positive_bins'],negative_bins=readiness['negative_bins'])))
