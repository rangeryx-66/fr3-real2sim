"""Independent post-freeze evaluator: sole V85 reader of hidden object GT.

Outputs errors, not raw hidden coefficients. Never imports fitting code,
changes models or chooses a candidate. A negative/partial authored reference
is kept explicit rather than treated as measured effective inertia.
"""
import argparse,datetime,hashlib,json
from pathlib import Path
import numpy as np
from evaluate_cross_object_v7 import authored_inertial_reference


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def evaluate(episode,frozen,output,expected_sha):
    if sha(frozen)!=expected_sha:raise ValueError('FROZEN_MODEL_HASH_CHANGED')
    if output.exists():raise ValueError('PRESERVE_PREVIOUS_INDEPENDENT_EVALUATION')
    completion=json.loads((episode/'PROCESS_COMPLETION.json').read_text())
    if not completion.get('actor_exited'):raise ValueError('ACTOR_STILL_ACTIVE')
    pid=json.loads((episode/'PROCESS.json').read_text())['pid']
    if Path('/proc',str(pid)).exists():raise ValueError('OWNED_ACTOR_STILL_ALIVE')
    model=json.loads(frozen.read_text());kind=model['joint_type']
    if model['schema']!='v85-native-state-development-only':raise ValueError('EXPLICIT_DIAGNOSTIC_SCOPE_REQUIRED')
    # All preconditions above precede the first GT read.
    accessed=datetime.datetime.now(datetime.timezone.utc).isoformat()
    gtpath=episode/'HIDDEN_AUTHORED_PHYSICS.json';truth=json.loads(gtpath.read_text())
    job=json.loads((episode/'job.json').read_text());inertial,extra=authored_inertial_reference(truth,job['asset_root'],kind)
    out=dict(schema='v85-independent-postfreeze-parameter-error-v1',truth_access_utc=accessed,
        frozen_model_sha256=expected_sha,hidden_reference_sha256=sha(gtpath),
        object_models_modified=False,truth_used_for_estimation_or_selection=False,
        state_source='privileged native object state diagnostic, not RGBD validation',
        reference='Authored randomized body combination and assigned passive joint law; not independently measured effective PhysX dynamics.',
        additional_unrandomized_moving_bodies=extra,models={})
    for label,m in model['models'].items():
        p=np.asarray(m['parameters']);grid=np.concatenate([np.linspace(lo,hi,101)for lo,hi in m['support_intervals_native']])
        law=truth['law'];c=law['base']+law.get('amplitude',0)*np.exp(-.5*((grid-law.get('center',0))/law.get('width',1))**2)
        b=truth['configuration']['b']
        out['models'][label]=dict(
            inertial_absolute_error=float(p[0]-inertial)if not extra else None,
            inertial_relative_error_pct=float(100*(p[0]/inertial-1))if not extra else None,
            damping_absolute_error=float(p[1]-b),damping_relative_error_pct=float(100*(p[1]/b-1)),
            resistance_curve_RMSE_on_FIT_support=float(np.sqrt(np.mean((p[2]-c)**2))),
            constant_C_is_not_the_position_dependent_law=True,
            conditional_std=m['conditional_std'],conditional_intervals_exclude_model_and_sensor_systematic_error=True)
    if sha(frozen)!=expected_sha:raise ValueError('MODEL_CHANGED_DURING_EVALUATION')
    output.write_text(json.dumps(out,indent=2,allow_nan=False));return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('frozen',type=Path);p.add_argument('output',type=Path);p.add_argument('--sha256',required=True);a=p.parse_args()
    print(json.dumps(evaluate(a.episode,a.frozen,a.output,a.sha256),indent=2))
