"""Opt-in FIT-only freeze and fresh held-out evaluation using V7 processing.

The only candidate change is a fixed, existing position-dependent resistance
structure. Physical owners, measurement processing and protection stay frozen.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from xarm_ft_v7_estimator import fit, samples, basis, prediction
from update_probing_v7_model import load_segments


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write_new(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x')as f:json.dump(data,f,indent=2)
    return dict(path=str(path),sha256=digest(path))


def fit_only(pieces,config):
    if any(not d['segments'][0].startswith('FIT')for d in pieces):
        raise ValueError('FIT_ONLY_TRAINING_REQUIRED')
    train=[]
    for d in pieces:train.extend(samples(d,config['domain_native'],config['intervals']))
    if len({r['segment']for r in train})<3:raise ValueError('THREE_DISTINCT_FIT_TRAJECTORIES_REQUIRED')
    model=fit(train,config,intervals=8,smoothness=.001)
    model.update(structure='existing V7 cubic position-resistance, 8 intervals, smoothness .001',validation_used=False)
    return model,len(train)


def freeze(episode,baseline_seal,output):
    episode=Path(episode);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    pieces,settings,provenance=load_segments(episode,allow_interrupted_fit=True)
    train=[d for d in pieces if d['segments'][0].startswith('FIT')]
    model,bins=fit_only(train,settings['estimation']);seal=read(baseline_seal);now=time.time()
    lineage=dict(training_episode_ids=[episode.name],training_roles=[d['segments'][0]for d in train],training_bins=bins,
        training_index_sha256=digest(episode/'telemetry/index.json'),quality_provenance=[p for p in provenance if p['role'].startswith('FIT')],
        old_validation_05_06_used_for_structure_selection=True,development_episode_ids=[episode.name],
        final_independent_validation_used=False,hidden_physics_used=False)
    common=dict(schema='v83-fixed-external-model-v1',frozen_at_unix_s=now,lineage=lineage,
        measurement_chain='unchanged V7-SI-common-SG121; original qualified FIT prefix selection',
        evaluation_contract=dict(primary=['high-angle reverse coordinate RMSE', 'absolute endpoint error of every completed reverse validation'],
            mandatory_secondary=['all intervals pooled RMSE','all high-angle intervals RMSE','forward RMSE','position, speed and acceleration strata','contact/quality association'],
            high_angle_threshold_rad=float(np.deg2rad(60)),state_correction='one measured initial q and qd per segment; no later correction',
            parameter_accuracy='unknown without separately authorized post-hoc hidden reference comparison',
            model_update_or_selection_on_new_episode=False),
        source_sha256=digest(__file__))
    candidate=dict(common,kind='candidate_position_resistance_8',model=model)
    baseline=dict(common,kind='original_exact_frozen_baseline',model=seal['selected'],
        original_frozen_model_sha256=seal['model_sha256'],original_seal_sha256=digest(baseline_seal))
    return dict(candidate=write_new(output/'CANDIDATE_FROZEN.json',candidate),baseline=write_new(output/'BASELINE_FROZEN.json',baseline))


def register(episode,candidate,baseline,roles,output):
    episode=Path(episode);idx=read(episode/'telemetry/index.json');status=read(episode/'AGENT_SESSION_STATUS.json')
    if status.get('state')!='WAITING' or status.get('unloaded',status.get('unloaded_decision'))is not True:
        raise ValueError('VERIFIED_UNLOADED_WAITING_BOUNDARY_REQUIRED')
    if any(not r.startswith('VALIDATION')for r in roles)or len(set(roles))!=len(roles):raise ValueError('DISTINCT_VALIDATION_ROLES_REQUIRED')
    if any(s['role']in roles for s in idx.get('segments',[])):raise ValueError('VALIDATION_ALREADY_STARTED')
    models=[read(f)for f in [candidate,baseline]]
    if any(episode.name in m['lineage']['training_episode_ids']+m['lineage']['development_episode_ids']for m in models):raise ValueError('FRESH_EPISODE_REQUIRED')
    now=time.time()
    if any(m['frozen_at_unix_s']>=now for m in models):raise ValueError('MODEL_NOT_ALREADY_FROZEN')
    registration=dict(schema='v83-external-heldout-registration-v1',episode_id=episode.name,registered_unix_s=now,
        model_sha256=dict(candidate=digest(candidate),baseline=digest(baseline)),roles=roles,
        telemetry_index_sha256=digest(episode/'telemetry/index.json'),
        prior_last_step=max((c['last_step']for c in idx.get('chunks',[])),default=-1),
        sequence_before_validation=status.get('sequence'),unloaded_before_validation=True,
        no_model_update_after_registration=True,validation_used_for_fit_or_selection=False)
    return write_new(output,registration)


def trace(piece,model):
    """Same semi-implicit friction integrator as the frozen production predictor."""
    edge=piece['processing']['boundary_exclusion_samples'];ix=np.arange(edge,len(piece['t'])-edge)
    if len(ix)<2:raise ValueError('INSUFFICIENT_RELIABLE_INTERIOR')
    t,q,v,effort=[piece[k][ix]for k in ['t','q','native_qd','torque']];mass,b=model['parameters'][:2]
    if mass<=1e-8:raise ValueError('UNSUPPORTED_INERTIAL_TERM')
    qp=[float(q[0])];vp=[float(v[0])]
    for j in range(1,len(t)):
        z=qp[-1]
        if not model['domain_native'][0]<=z<=model['domain_native'][1]:raise ValueError('PREDICTION_LEFT_DECLARED_DOMAIN')
        C=float(basis([z],model['domain_native'],model['intervals'])[0,model['active_controls']]@model['parameters'][2:])if model['intervals']else model['parameters'][2]
        dt=t[j]-t[j-1];p=mass*vp[-1]+dt*effort[j-1];vel=float(np.sign(p)*max(abs(p)-dt*C,0)/(mass+dt*b));vp.append(vel);qp.append(z+dt*vel)
    return dict(t=t,q=q,v=v,acc=piece['qdd'][ix],quality_valid=piece['valid'][ix],q_error=np.asarray(qp)-q,v_error=np.asarray(vp)-v)


def metrics(errors,mask=None):
    x=np.asarray(errors);x=x if mask is None else x[mask]
    return dict(samples=len(x),coordinate_rmse_rad=float(np.sqrt(np.mean(x*x))),coordinate_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(x*x)))))if len(x)else dict(samples=0)


def strata(d):
    q,v,acc=d['q'],d['v'],d['acc'];error=d['q_error'];out=dict(all=metrics(error),high_angle_ge60=metrics(error,q>=np.deg2rad(60)),high_angle_reverse=metrics(error,(q>=np.deg2rad(60))&(v<-1e-4)),reverse=metrics(error,v<-1e-4),forward=metrics(error,v>1e-4),quality_valid=metrics(error,d['quality_valid']),quality_rejected=metrics(error,~d['quality_valid']))
    for lo,hi in [(0,30),(30,60),(60,75),(75,100)]:out[f'position_deg_{lo}_{hi}']=metrics(error,(q>=np.deg2rad(lo))&(q<np.deg2rad(hi)))
    for lo,hi in [(0,.01),(.01,.02),(.02,1)]:out[f'absolute_velocity_rad_s_{lo}_{hi}']=metrics(error,(abs(v)>=lo)&(abs(v)<hi))
    for lo,hi in [(0,.005),(.005,.02),(.02,100)]:out[f'absolute_acceleration_rad_s2_{lo}_{hi}']=metrics(error,(abs(acc)>=lo)&(abs(acc)<hi))
    return out


def evaluate(episode,candidate,baseline,registration,output,development=False):
    episode=Path(episode);candidate,baseline=map(Path,(candidate,baseline));models=dict(candidate=read(candidate),baseline=read(baseline))
    if not read(episode/'PROCESS_COMPLETION.json')['actor_exited']:raise ValueError('CLOSED_EPISODE_REQUIRED')
    idx=read(episode/'telemetry/index.json')
    if development:
        roles={s['role']for s in idx['segments']if s['role'].startswith('VALIDATION')};reg=None
    else:
        reg=read(registration)
        if reg['episode_id']!=episode.name or reg['model_sha256']!=dict(candidate=digest(candidate),baseline=digest(baseline)):raise ValueError('REGISTRATION_MODEL_OR_EPISODE_MISMATCH')
        for m in models.values():
            if episode.name in m['lineage']['training_episode_ids']+m['lineage']['development_episode_ids']:raise ValueError('FRESH_VALIDATION_REQUIRED')
        roles=set(reg['roles'])
        for s in idx['segments']:
            if s['role']in roles and s['first_step']<=reg['prior_last_step']:raise ValueError('REGISTRATION_NOT_BEFORE_VALIDATION')
    pieces,settings,quality=load_segments(episode,allow_interrupted_fit=True);held=[d for d in pieces if d['segments'][0].rsplit('_',1)[0]in roles]
    from diagnose_probing_v3 import columns
    from agentic_interaction_v81.feedback import motion_errors
    raw,_=columns(episode)
    report=dict(schema='v83-fixed-model-heldout-comparison-v1',episode_id=episode.name,development_diagnostic_only=development,
        independent_validation=not development,registration_sha256=None if development else digest(registration),model_sha256=dict(candidate=digest(candidate),baseline=digest(baseline)),
        refitting_performed=False,model_selection_performed=False,validation_used_for_fit_or_selection=False,hidden_physics_used=False,
        segment_quality=[s for s in quality if s['role']in roles],models={})
    for name,payload in models.items():
        model=payload['model'];parts=[];segments={}
        for d in held:
            samples(d,settings['estimation']['domain_native'],settings['estimation']['intervals'])
            try:
                measured=trace(d,model);parts.append(measured);sid=int(d['block_id'].split('#')[0].rsplit('_',1)[1]);ix=np.flatnonzero(raw['segment']==sid)
                nearest=np.searchsorted(raw['t'],measured['t']);nearest=np.clip(nearest,0,len(raw['t'])-1);slip=raw['relative_slip_m'][nearest]
                contact=dict(reported_relative_slip_max_m=float(slip.max()),reported_relative_slip_median_m=float(np.median(slip)),
                    error_vs_reported_slip_correlation=float(np.corrcoef(measured['q_error'],slip)[0,1])if np.std(slip)>1e-12 and np.std(measured['q_error'])>1e-12 else None,
                    missing_positive_normal_is_not_proven_separation=True)
                if len(ix):
                    e=motion_errors(raw['command_TCP_pose'][ix,:3],raw['tcp_pose'][ix,:3],raw['joint_q'][ix],settings['prior'])
                    contact.update(max_servo_error_m=float(np.linalg.norm(e['command_minus_measured_world_m'],axis=1).max()),max_rigid_arc_residual_m=float(np.linalg.norm(e['measured_minus_rigid_articulation_world_m'],axis=1).max()),hard_stop_samples=int(raw['safety_stop'][ix].sum()))
                segments[d['block_id']]=dict(prediction=prediction(d,model),strata=strata(measured),contact=contact,final_coordinate_error_rad=float(measured['q_error'][-1]),final_coordinate_error_deg=float(np.rad2deg(measured['q_error'][-1])))
            except ValueError as exc:segments[d['block_id']]=dict(status='PREDICTION_FAILED',reason=str(exc))
        aggregate=strata({k:np.concatenate([p[k]for p in parts])for k in parts[0]})if parts else {}
        report['models'][name]=dict(segments=segments,aggregate=aggregate,parameters=payload['model']['parameters'],parameter_names=payload['model']['parameter_names'])
    completed={s['role']for s in idx['segments']if s.get('result',{}).get('status')=='TARGET_REACHED'and s.get('result',{}).get('physical_transmission_valid')}
    report['all_declared_validation_completed']=roles<=completed
    return write_new(output,report)


if __name__=='__main__':
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='action',required=True)
    f=sub.add_parser('freeze');f.add_argument('--episode',required=True);f.add_argument('--baseline-seal',required=True);f.add_argument('--output',required=True)
    r=sub.add_parser('register');r.add_argument('--episode',required=True);r.add_argument('--candidate',required=True);r.add_argument('--baseline',required=True);r.add_argument('--roles',nargs='+',required=True);r.add_argument('--output',required=True)
    e=sub.add_parser('evaluate');e.add_argument('--episode',required=True);e.add_argument('--candidate',required=True);e.add_argument('--baseline',required=True);e.add_argument('--registration');e.add_argument('--output',required=True);e.add_argument('--development',action='store_true')
    a=vars(p.parse_args());action=a.pop('action');result=globals()[action](**a);print(json.dumps(result,indent=2))
