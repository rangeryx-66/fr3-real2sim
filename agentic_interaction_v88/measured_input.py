"""FIT-only demand and demonstrated contact summary, without plant/GT IO."""
import hashlib,json
from pathlib import Path
import numpy as np


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(model,pieces):
    # The study predeclares stationary rail laws. Use the original worker's
    # existing constant candidate, not a new fit or a held-out-selected model.
    constant=model.get('constant_model')
    if not constant or constant.get('joint_type')!='prismatic' or constant.get('intervals')!=0:
        return dict(model_usable=False,reason='NO_ORIGINAL_PRISMATIC_CONSTANT_MODEL')
    fit=[d for d in pieces if d['segments'][0].startswith('FIT') and np.any(d['valid'])]
    if not fit:return dict(model_usable=False,reason='NO_QUALIFIED_FIT')
    beta=np.asarray(constant['parameters'],float)
    X=np.concatenate([np.c_[d['qdd'][d['valid']],d['qd'][d['valid']],
        d['direction'][d['valid']]] for d in fit])
    raw=np.concatenate([d['unfiltered_torque'][d['valid']] for d in fit])
    residual=raw-X@beta;bound=float(np.percentile(abs(residual),99))
    bidirectional=bool(np.any(X[:,1]>0) and np.any(X[:,1]<0))
    trajectories=len(set(d['segments'][0] for d in fit))
    usable=bool(trajectories>=2 and bidirectional and constant['algebraic_rank']==3
        and all(constant['conditionally_supported']))
    return dict(model_usable=usable,demand_model=dict(parameters=beta.tolist(),
        covariance=constant['conditional_covariance'],residual_bound_n=bound,
        source='original worker constant candidate; eligible FIT only'),
        demand_model_choice='stationary-rail constant candidate predeclared for every method',
        original_main_selected_intervals=(model.get('selected') or {}).get('intervals'),
        FIT_trajectory_count=trajectories,actual_bidirectional_FIT=bidirectional,
        FIT_samples=len(X),FIT_raw_residual_abs99_n=bound,
        FIT_speed_m_s_minmax=[float(X[:,1].min()),float(X[:,1].max())],
        FIT_acceleration_abs95_m_s2=float(np.percentile(abs(X[:,0]),95)),
        conditional_parameter_support=constant['conditionally_supported'],
        physical_parameter_accuracy_not_established=True,
        uncertainty_scope='original conditional covariance plus empirical FIT residual; not calibrated total physical confidence')


def from_episode(episode,contact_capabilities):
    # Only these measurement/estimator files are used. Plant author outputs,
    # native parameter readbacks and episode job/physics files are never read.
    from update_probing_v7_model import load_segments
    from update_prismatic_v73_model import qualify
    p=Path(episode);model_path=p/'model/estimate.json';model=json.loads(model_path.read_text())
    if model.get('hidden_dynamics_used') is not False or model.get('validation_used_for_fit_or_selection') is not False:
        raise ValueError('ORIGINAL_FIT_ONLY_NONTRUTH_MODEL_REQUIRED')
    pieces,_,_=load_segments(p,True)  # original predeclared transmitting-prefix / interval eligibility
    for d in pieces:qualify(d)  # unchanged production full-window sliding rule
    index=json.loads((p/'telemetry/index.json').read_text())
    return dict(schema='physics-aware-measured-input/v1',hidden_dynamics_used=False,
        validation_used_for_fit_or_selection=False,**summarize(model,pieces),
        contact_capabilities=contact_capabilities,
        measurement_sources=dict(model_sha256=digest(model_path),
            identification_input_sha256=digest(p/'identification_input.json'),
            telemetry_index_sha256=digest(p/'telemetry/index.json'),
            chunks=[dict(file=c['file'],sha256=c['sha256']) for c in index['chunks']]))
