"""Audit production's contiguous held-out prediction; never refits a model.

The frozen V7 predictor already uses the whole filter-supported interval,
including stationary/stiction samples, independently of the FIT-validity mask.
Verify that contract explicitly; this is not a new prediction correction.
"""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from update_probing_v7_model import load_segments
from xarm_ft_v7_estimator import samples,prediction
from agent_command_protocol_v12 import atomic_json


def evaluate(folder,model_path):
    if not (folder/'PROCESS_COMPLETION.json').exists():raise ValueError('Post-episode evaluator only; finish all online decisions first')
    pieces,settings,provenance=load_segments(folder,True);model=json.loads(model_path.read_text());c=settings['estimation'];result={}
    if settings['prior']['joint_type']!='prismatic':raise ValueError('Prismatic-only evaluator')
    for name in ('constant_model','extended_model','selected'):
        m=model.get(name)
        if not m:continue
        reports={}
        for d in pieces:
            if not d['segments'][0].startswith('VALIDATION'):continue
            samples(d,c['domain_native'],c['intervals'])
            edge=int(d['processing']['boundary_exclusion_samples']);valid=np.zeros(len(d['t']),bool);valid[edge:len(valid)-edge]=True
            q=dict(d,valid=valid);original=prediction(d,m);checked=prediction(q,m)
            if original!=checked:raise AssertionError('Production prediction unexpectedly depends on sliding-validity mask')
            reports[d['block_id']]=dict(checked,selection='production already uses entire contiguous local-filter-supported interval, including zero/reversing velocity; one initial measured state',original_motion_qualified_samples=int(d['valid'].sum()),supported_samples=int(valid.sum()),physical_action_complete=True,model_refitted=False,production_already_contiguous=True)
        result[name]=reports
    return dict(model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),model_refitted=False,validation_used_for_fitting=False,hidden_truth_used=False,model_file=str(model_path),predictions=result,limitation='SG boundary support excluded explicitly; complete physical action and numerical prediction interval reported separately')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--model',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();atomic_json(a.output,evaluate(a.episode,a.model or a.episode/'model/estimate.json'))
