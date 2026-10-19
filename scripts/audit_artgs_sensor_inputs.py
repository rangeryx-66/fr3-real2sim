"""Diagnose old modality/scale corruption from saved sensor data, without GT."""
import argparse,json
from pathlib import Path
import numpy as np


def audit(root):
    rows=[]
    for name in ['7320_recovery_v2','45746_recovery_v3_infrastructure_retry']:
        r=Path(root)/name;doc=json.loads((r/'multistate_capture.json').read_text())
        initial=np.concatenate([np.load(r/v['directory']/'point_cloud.npz')['points_world_m'] for v in doc['states'][0]['views']])
        initial_bounds=np.quantile(initial,[.001,.999],axis=0);extent=initial_bounds[1]-initial_bounds[0]
        for s in doc['states']:
            for v in s['views']:
                P=np.load(r/v['directory']/'point_cloud.npz')['points_world_m'];bounds=np.array([P.min(0),P.max(0)]);ratio=float(np.max((bounds[1]-bounds[0])/np.maximum(extent,.01)))
                rows.append({'object':doc['object_id'],'state':s['state_id'],'view':v['view_id'],'points':len(P),'bounds_m':bounds.tolist(),'span_ratio_to_initial':ratio,'far_below_initial_fraction':float(np.mean(P[:,2]<initial_bounds[0,2]-.15)),'background_contamination_suspected':bool(ratio>3 or np.mean(P[:,2]<initial_bounds[0,2]-.15)>.01)})
    return {'GT_used':False,'comparison':'observed initial geometry vs later per-view clouds','rows':rows,'warning':'not repair by GT clipping; reacquire synchronized RGB/depth and sensor masks','suspected_views':sum(r['background_contamination_suspected'] for r in rows)}
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();d=audit(a.root);a.output.write_text(json.dumps(d,indent=2));print('suspected_views',d['suspected_views'])
