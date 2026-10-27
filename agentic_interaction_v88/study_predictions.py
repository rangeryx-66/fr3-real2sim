"""Evaluate frozen models on CLOSED held-out operations, without refitting."""
import argparse,hashlib,json,sys
from pathlib import Path


def evaluate(episode,models):
    p=Path(episode)
    if json.loads((p/'PROCESS_COMPLETION.json').read_text()).get('actor_exited') is not True:
        raise ValueError('CLOSED_EXPERIMENT_REQUIRED')
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'scripts'))
    from update_probing_v7_model import load_segments
    from xarm_ft_v7_estimator import prediction
    pieces,_,provenance=load_segments(p,True)
    results=[]
    for path in models:
        path=Path(path);raw=path.read_bytes();model=json.loads(raw)
        if (model.get('hidden_dynamics_used') is not False
                or model.get('validation_used_for_fit_or_selection') is not False):
            raise ValueError('FROZEN_FIT_ONLY_NONTRUTH_MODEL_REQUIRED')
        constant=model['constant_model']
        if constant.get('intervals')!=0 or constant.get('joint_type')!='prismatic':
            raise ValueError('PREDECLARED_STATIONARY_RAIL_MODEL_REQUIRED')
        results.append(dict(model_name=path.stem,model_sha256=hashlib.sha256(raw).hexdigest(),
            parameters=constant['parameters'],conditional_std=constant['conditional_std'],
            support=constant['support_intervals_native'],
            heldout={d['block_id']:prediction(d,constant) for d in pieces
                if d['segments'][0].startswith('VALIDATION')}))
    return dict(schema='frozen-physics-heldout-prediction/v1',episode=p.name,
        models=results,processing=provenance,hidden_dynamics_used=False,refitted=False,
        model_selected_using_validation=False,
        scope='original compensated F/T, SG121 and original open-loop prediction; one measured initial q/qd; no later state correction',
        model_preexecution_seal_must_be_verified_separately=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('episode',type=Path)
    p.add_argument('--model',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=evaluate(a.episode,a.model);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(episode=result['episode'],models=len(result['models']),refitted=False)))
