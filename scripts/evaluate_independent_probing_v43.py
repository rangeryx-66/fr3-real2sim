"""Post-exit V4.3 evaluation; immutable online model and pre-validation snapshot.

No estimator tuning or simulator imports. The V4.1 evaluator provides the same
qualified-interval audit and optional frozen-V3 comparison on the new records.
"""
import argparse, contextlib, hashlib, io, json
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json


def evaluate(root):
    from evaluate_independent_probing_v41 import evaluate as evaluate_previous
    from update_probing_v31_model import load_segments, XY, motion_prediction
    from interaction_runtime.serialization import result_packet
    root=Path(root)
    # This checks actor exit before any hidden-file access.
    with contextlib.redirect_stdout(io.StringIO()):
        result=evaluate_previous(root)
    p=root/'episode_00'
    result.update(reference='5fa0f98',registration='f06e3ed',experiment='V4.3',
                  online_decisions_finished_before_truth=True)
    snapshot=p/'PREVALIDATION_ONLINE_MODEL.json'
    result['prevalidation_frozen_model']=dict(status='N/A: no captured pre-validation online model')
    if snapshot.exists():
        model=json.loads(snapshot.read_text())
        if model.get('validation_bins')!=0:
            raise ValueError('Pre-validation snapshot must contain no validation samples')
        pieces,train,test,quality=load_segments(p,allow_interrupted_fit=True)
        c=model.get('constant_model',{})
        row=dict(status='online FIT-only snapshot, frozen before validation admission',
                 source_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
                 parameters=c.get('parameters'),conditional_std=c.get('conditional_std'),
                 training_bins=model.get('training_bins'),validation_bins=0,
                 heldout_torque_rmse_nm=None,heldout_motion={})
        if test and c.get('parameters'):
            X,y=XY(test)
            row['heldout_torque_rmse_nm']=float(np.sqrt(np.mean((y-X@c['parameters'])**2)))
            for d in pieces:
                if not str(d['segments'][0]).startswith('VALIDATION'):continue
                edge=d['processing']['boundary_exclusion_samples']
                clone=dict(d,qd=d['native_qd'].copy())
                row['heldout_motion'][d['block_id']]=motion_prediction(clone,np.arange(edge,len(d['t'])-edge),c)
            row['evaluation']='complete independent trajectories only; one native initial state; measured F/T input; no future-state correction'
        if result['authored_reference'] is not None and c.get('parameters'):
            known=np.asarray(result['authored_reference']);beta=np.asarray(c['parameters'])
            row['posthoc_relative_error_percent']=(100*(beta/known-1)).tolist()
        result['prevalidation_frozen_model']=row
    capture=p/'ONLINE_ESTIMATE_CAPTURE.json'
    result['online_updates']=json.loads(capture.read_text()) if capture.exists() else dict(updates=[])
    result['assessment']=dict(status='REQUIRES_REVIEW',
        complete_validation=any(q['role'].startswith('VALIDATION') and q.get('blocks') for q in result['quality']),
        note='Numerical fit and safe lifecycle alone are not PASS; assess physical coverage, complete validation, identifiability and held-out prediction.')
    atomic_json(root/'RESULT.json',result_packet(result))
    print(json.dumps(result_packet({k:result[k] for k in ['experiment','assessment','comparison','prevalidation_frozen_model','storage','completion']}),indent=2))
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    evaluate(parser.parse_args().root)
