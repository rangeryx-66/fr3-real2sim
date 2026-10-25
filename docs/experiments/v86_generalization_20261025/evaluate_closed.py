"""Shared post-exit F/T evaluation; never fit, select actions, or modify models.

GT access is confined here after native-process closure and model hash sealing.
The same original V84 exact-seal predictor and selected-joint inertial reference
serve every object. Only errors, not hidden parameter values, are exported.
"""
import argparse
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):return json.loads(Path(path).read_text())


def helper(name):
    path=ROOT/'docs/experiments/v84_abc_20261025'/(name+'.py')
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module);return module


def evaluate(episode,output):
    episode=Path(episode);output=Path(output)
    if not (episode/'PROCESS_COMPLETION.json').exists() or not read(episode/'PROCESS_COMPLETION.json').get('actor_exited'):
        raise RuntimeError('CLOSED_EPISODE_REQUIRED_BEFORE_ANY_GT')
    process=read(episode/'PROCESS.json')
    if (Path('/proc')/str(process['pid'])).exists():
        raise RuntimeError('ACTOR_STILL_ONLINE')
    if output.exists():raise ValueError('PRESERVE_PREVIOUS_EVALUATION')
    output.mkdir(parents=True)
    modelpaths=sorted((episode/'numeric_packets').glob('*/model/estimate.json'))
    if (episode/'model/estimate.json').exists():modelpaths.append(episode/'model/estimate.json')
    seals=sorted((episode/'validation_model_seals').glob('*.json'))
    frozen={str(p):sha(p)for p in modelpaths+seals}
    manifest=dict(source_sha=process['source_commit'],sealed_before_GT=frozen,
        episode_id=episode.name,UTC=datetime.datetime.now(datetime.timezone.utc).isoformat())
    (output/'MODEL_EVALUATION_FREEZE.json').write_text(json.dumps(manifest,indent=2)+'\n')
    helper('derive_closed').audit(episode,output/'measured')
    measured=read(output/'measured/FINAL_AUDIT.json')
    predicted=helper('derive_frozen_validation').audit(episode,output/'FROZEN_PREDICTION.json')
    # All prediction/model provenance is computed before hidden-reference access.
    import numpy as np
    from xarm_ft_v7_estimator import basis
    from evaluate_cross_object_v7 import authored_inertial_reference
    truthpath=episode/'HIDDEN_AUTHORED_PHYSICS.json'
    truth=read(truthpath);access=datetime.datetime.now(datetime.timezone.utc).isoformat()
    job=read(episode/'job.json');kind=job['articulation_v7']['joint_type']
    inertia,extra=authored_inertial_reference(truth,job['asset_root'],kind)
    case=truth['configuration'];models=[]
    targets={sha(p):p for p in modelpaths}
    chosen=[('validation_'+p.stem,targets[read(p)['model_sha256']])for p in seals]
    if (episode/'model/estimate.json').exists():chosen.append(('final_online',episode/'model/estimate.json'))
    for label,p in chosen:
        data=read(p);model=data.get('selected')
        if not model:
            models.append(dict(label=label,status='NO_SUPPORTED_MODEL',sha256=sha(p)));continue
        if data.get('validation_used_for_fit_or_selection') is not False:
            raise RuntimeError('FIT_VALIDATION_ISOLATION_NOT_ESTABLISHED')
        params=np.asarray(model['parameters']);support=model['support_intervals_native']
        grid=np.concatenate([np.linspace(lo,hi,101)for lo,hi in support]) if support else np.array([])
        if len(grid):
            curve=(basis(grid,model['domain_native'],model['intervals'])[:,model['active_controls']]@params[2:]
                if model['intervals']else np.full(len(grid),params[2]))
            law=case['law'];reference=law['base']+law.get('amplitude',0.)*np.exp(-.5*((grid-law.get('center',0.))/law.get('width',1.))**2)
            curve_error=float(np.sqrt(np.mean((curve-reference)**2)))
        else:curve_error=None
        models.append(dict(label=label,model_sha256=sha(p),estimated_parameters=model['parameters'],
            conditional_std=model.get('conditional_std'),conditionally_supported=model.get('conditionally_supported'),
            scaled_condition=model.get('scaled_condition'),algebraic_rank=model.get('algebraic_rank'),
            support_intervals_native=support,
            inertial_relative_error_pct=100*(params[0]/inertia-1)if not extra else None,
            damping_relative_error_pct=100*(params[1]/case['b']-1),
            resistance_curve_RMSE_FIT_support=curve_error,
            extra_unrandomized_moving_bodies=extra,
            parameter_reference='Authored randomized selected-joint rigid-body combination and passive law; not independently measured effective PhysX dynamics.',
            unsupported_parameters_remain_unidentified=True,validation_used_for_selection=False))
    assert all(sha(path)==expected for path,expected in frozen.items())
    outcome=dict(schema='v86-postexit-independent-parameter-error-v1',episode_id=episode.name,
        asset_id=job['asset_id'],joint_type=kind,truth_access_UTC=access,truth_sha256=sha(truthpath),
        estimated_models_unchanged=True,GT_used_for_fit_or_online_decisions=False,
        raw_GT_values_exported=False,models=models,exact_frozen_predictions='FROZEN_PREDICTION.json',
        measured_outcome='measured/FINAL_AUDIT.json',
        operation_and_prediction_pass_are_separate=True)
    (output/'PARAMETER_ERRORS.json').write_text(json.dumps(outcome,indent=2,allow_nan=False)+'\n')
    return dict(asset_id=job['asset_id'],actual_max=measured['max_actual'],
        unit=measured['coordinate_display_unit'],hard_stop_records=measured['hard_stop_records'],
        parameter_models=len(models),complete_validation_segments=sum(s.get('status')=='SEALED_COMPLETE_VALIDATION'for s in predicted['segments']),
        output=str(output))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();print(json.dumps(evaluate(a.episode,a.output)))
