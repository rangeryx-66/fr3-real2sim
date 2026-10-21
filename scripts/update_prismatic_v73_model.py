"""Prismatic FIT eligibility; frozen V3.1/V7 filtering and fitting unchanged.

A sliding-friction regressor is only eligible when its COMPLETE filter support
is sliding in a resolved direction. A filtered velocity can be nonzero while
its kernel contains unresolved static friction or contact-induced reversals.
Use the existing native velocity resolution and exact existing kernel support;
no truth-selected tolerance, force threshold, coefficient or filter change.
"""
import argparse,json,sys,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.ndimage import minimum_filter1d
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import update_probing_v7_model as frozen
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def qualify(piece):
    if not piece['segments'][0].startswith('FIT'):
        return dict(applied=False,reason='complete independent VALIDATION unchanged')
    velocity=np.asarray(piece['native_qd']);p=piece['processing'];radius=int(p['centered_future_support_samples']);floor=float(p['minimum_absolute_native_velocity'])
    positive=minimum_filter1d((velocity>floor).astype(np.uint8),size=2*radius+1,mode='constant',cval=0).astype(bool)
    negative=minimum_filter1d((velocity<-floor).astype(np.uint8),size=2*radius+1,mode='constant',cval=0).astype(bool)
    old=piece['valid'].copy();piece['valid']=old&(positive|negative)
    return dict(applied=True,block=piece['block_id'],original_valid_samples=int(old.sum()),qualified_samples=int(piece['valid'].sum()),excluded_samples=int((old&~piece['valid']).sum()),native_velocity_resolution=floor,filter_support_radius_samples=radius,tag='complete SG support must contain resolved sliding in one direction',execution_status_unchanged=True)


def update(folder,output=None,allow_interrupted_fit=False):
    folder=Path(folder);out=Path(output or folder/'model');original=frozen.load_segments;audit=[]
    def loader(*args,**kw):
        pieces,settings,provenance=original(*args,**kw)
        if settings['prior']['joint_type']!='prismatic':raise ValueError('Prismatic-only qualification; revolute baseline cannot activate it')
        for d in pieces:
            packet=qualify(d);audit.append(packet)
            if packet['applied']:
                for row in provenance:
                    if d['block_id'].startswith(row['role']+'_%02d'%row['id']+'#'):
                        block=int(d['block_id'].rsplit('#',1)[1])
                        row['blocks'][block]['prismatic_sliding_qualification']=packet
        return pieces,settings,provenance
    with patch.object(frozen,'load_segments',loader):result=frozen.update(folder,out,allow_interrupted_fit)
    result['FIT_qualification_v73']=dict(rule='existing velocity resolution across complete unchanged SG support',segments=audit,validation_mask_unchanged=True,filter_math_changed=False,tool_compensation_count=1,hidden_truth_used=False)
    import hashlib
    result['source_sha256']['scripts/update_prismatic_v73_model.py']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    atomic_json(out/'estimate.json',result_packet(result));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--output',type=Path);p.add_argument('--allow-interrupted-fit',action='store_true');a=p.parse_args()
    r=update(a.folder,a.output,a.allow_interrupted_fit);print(json.dumps(dict(status=r['status'],parameters=r.get('selected',{}).get('parameters'),worker_wall_s=r['worker_wall_s'])))
