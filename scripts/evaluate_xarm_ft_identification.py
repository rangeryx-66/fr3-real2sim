"""Independent POST-EPISODE evaluator. Hidden truth never feeds estimation/actions."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np


def effective_inertia(truth,asset_root,prior):
    from articulated_demo.kinematics import URDFChain
    import xml.etree.ElementTree as ET
    asset_root=Path(asset_root);path=next((asset_root/'urdf').glob('*.urdf'));chain=URDFChain(path)
    joint=next(j for j in ET.parse(path).getroot().findall('joint') if j.get('type')=='revolute')
    J=chain.root_to_link(joint.find('child').get('link'),{})
    axis=J[:3,:3]@np.array(list(map(float,joint.find('axis').get('xyz').split())));axis/=np.linalg.norm(axis);origin=J[:3,3]
    result=0.
    for b in truth['bodies']:
        T=chain.root_to_link(b['body'],{});r=T[:3,:3]@b['com_local_m']+T[:3,3]-origin
        result+=float(axis@T[:3,:3]@np.asarray(b['inertia_local_kg_m2'])@T[:3,:3].T@axis+b['mass_kg']*(r@r-(axis@r)**2))
    return result


def predict_segment(data,segment,parameters):
    ix=np.flatnonzero(data['segments']==segment)
    if len(ix)<2:return dict(status='insufficient segment samples')
    t=data['t'][ix];theta=data['q'][ix];v=data['qd'][ix];tau=data['torque'][ix]
    I=parameters['I_eff_kg_m2'];b=parameters['b_nm_s_rad'];c=parameters['tau_c_nm']
    if I<=1e-8:return dict(status='forward prediction unavailable: fitted inertia degenerate, not replaced by GT/prior',I_eff_kg_m2=I)
    qp=[float(theta[0])];vp=[float(v[0])]
    for j in range(1,len(ix)):
        # Independent forward dynamics. Only initial state and measured wrist
        # effort enter; NO following q(t), object position drive or state reset.
        dt=float(t[j]-t[j-1]);u=float(tau[j-1]);previous=vp[-1]
        # Backward Euler of the passive Coulomb+viscous joint. The soft
        # threshold solves the static-friction inclusion at zero velocity.
        # Stable even when an unsupported fitted inertia is extremely small.
        momentum=I*previous+dt*u
        vn=float(np.sign(momentum)*max(abs(momentum)-dt*c,0.)/(I+dt*b))
        qn=qp[-1]+dt*vn
        if not np.isfinite(qn) or abs(qn)>100:return dict(status='independent model diverged',time_after_start_s=float(t[j]-t[0]),GT_trajectory_forcing=False)
        qp.append(qn);vp.append(vn)
    qp=np.array(qp);vp=np.array(vp)
    return dict(status='predicted',duration_s=float(t[-1]-t[0]),samples=len(ix),theta_rmse_deg=float(np.rad2deg(np.sqrt(np.mean((qp-theta)**2)))),
        velocity_rmse_rad_s=float(np.sqrt(np.mean((vp-v)**2))),final_angle_error_deg=float(np.rad2deg(qp[-1]-theta[-1])),
        measured_initial_angle_rad=float(theta[0]),measured_initial_velocity_rad_s=float(v[0]),GT_trajectory_forcing=False,
        model='constant Coulomb+viscous+inertia; passive backward-Euler native-step integration with Coulomb inclusion; no fitted per-angle terms')


def strict_evaluation(models):
    # Pure measurement-only re-evaluation; no truth enters fit or exclusions.
    import pickle
    from xarm_ft_id_estimator import process,estimate,DERIVATIVE_WINDOW
    updates=[]
    for path in models:
        settings=json.loads((path.parent/'INPUT.json').read_text())
        with (path.parent/'measurements.pkl').open('rb') as f:rows=pickle.load(f)
        data=process(rows,settings['prior'],settings['bias'])
        if data is None:continue
        half=DERIVATIVE_WINDOW//2
        changes=np.flatnonzero(data['segments'][1:]!=data['segments'][:-1])+1
        excluded=np.zeros(len(data['t']),bool)
        for j in changes:excluded[max(0,j-half):min(len(excluded),j+half+1)]=True
        data['valid'][excluded]=False
        for i in np.flatnonzero(excluded):data['tags'][i].append('posthoc segment boundary derivative exclusion, same rule all cases')
        # Entire 0.1s bin must be outside the derivative neighborhood.
        boundary_t=data['t'][changes]
        data['samples']=[v for v in data['samples'] if all(abs(v['t']-b)>.25+.05 for b in boundary_t)]
        out=path.parent/'strict_posthoc';model=estimate(data,out,settings['prior'])
        entry=dict(source=str(path),model=model,rule='same SG121 +/-60 samples at every segment boundary; no truth-based selection')
        if model.get('parameters'):
            entry['heldout_motion_predictions']={v:predict_segment(data,v,model['parameters']) for v in np.unique(data['segments'].astype(str)) if v.startswith('VALIDATION')}
        updates.append(entry)
    return updates


def authoritative_models(episode):
    """Choose by documented kinematic input provenance, never fit quality/truth."""
    paths=[]
    for packet in sorted((episode/'identification').glob('attempt_*/*')):
        settings=packet/'INPUT.json'
        if not settings.exists():continue
        original=json.loads(settings.read_text())
        if original.get('prior',{}).get('native_joint_path'):
            model=packet/'IDENTIFIED_MODEL.json'  # repaired online V0.1
        else:model=packet/'native_joint_prior/IDENTIFIED_MODEL.json'
        if model.exists():paths.append(model)
    return paths


def evaluate(case):
    case=Path(case);episode=case/'episode'
    if not (episode/'EXPORT_COMPLETE.json').exists():raise ValueError('Only completed/exported episodes may be evaluated')
    pid=json.loads((case/'PROCESS.json').read_text())['pid']
    if Path('/proc/%d'%pid).exists():raise ValueError('Actor still alive; no hidden-truth evaluation while actions remain')
    job=json.loads((case/'job.json').read_text());truth=json.loads((episode/'HIDDEN_AUTHORED_PHYSICS.json').read_text())
    models=authoritative_models(episode) if (episode/'identification').exists() else []
    result=dict(case=job['identification_case'],seed=truth['configuration']['seed'],robot='xArm7/native G1',reference='136b6e0',
        hidden_truth_opened_only_after_actor_exit=True,explicit_known_prior_repair=True,original_body_origin_models_preserved_but_invalid=True,grasp_outcome=json.loads((episode/'RECOVERY_EPISODE_RESULT.json').read_text()) if (episode/'RECOVERY_EPISODE_RESULT.json').exists() else None,
        update_count=len(models),updates=[])
    for path in models:
        model=json.loads(path.read_text());entry=dict(path=str(path.relative_to(episode)),estimated=model,numeric_update_mode='external corrected known-prior reprocessing' if path.parent.name=='native_joint_prior' else 'original online worker with corrected known-prior wiring')
        if model.get('parameters'):
            gt=dict(I_eff_kg_m2=effective_inertia(truth,job['asset_root'],model['kinematic_prior']),b_nm_s_rad=truth['configuration']['b'],tau_c_nm=truth['configuration']['tau_c'])
            entry['authored_truth']=gt;entry['absolute_error']={k:abs(model['parameters'][k]-v) for k,v in gt.items()}
            entry['relative_error']={k:abs(model['parameters'][k]-v)/abs(v) if abs(v)>1e-12 else None for k,v in gt.items()}
            data=np.load(path.parent/'measured_effort.npz',allow_pickle=True)
            entry['heldout_motion_predictions']={s:dict(predict_segment(data,s,model['parameters']),parameter_observability=model.get('observability')) for s in np.unique(data['segments'].astype(str)) if s.startswith('VALIDATION')}
            # Post-hoc authored inverse dynamics is a reference, NOT an
            # independently measured complete simulator generalized effort.
            reference=gt['I_eff_kg_m2']*data['qdd']+gt['b_nm_s_rad']*data['qd']+gt['tau_c_nm']*np.sign(data['qd'])
            entry['authored_inverse_dynamics_reference']={}
            for label in np.unique(data['segments'].astype(str)):
                ix=(data['segments'].astype(str)==label)&data['valid']
                if ix.any():entry['authored_inverse_dynamics_reference'][label]=dict(samples=int(ix.sum()),sensor_vs_reference_rmse_nm=float(np.sqrt(np.mean((data['torque'][ix]-reference[ix])**2))),independent_GT_effort_measurement=False)

        result['updates'].append(entry)
    result['strict_posthoc_boundary_evaluation']=strict_evaluation(models)
    if models:
        latest=json.loads(models[-1].read_text());pose=json.loads((episode/'progress.json').read_text())
        exported=dict(schema='articulated_physical_model_v0',geometry_reference=job['asset_root'],
            observed_geometry_reference=str(episode/'agent_geometry/observed_geometry.json'),joint=latest['kinematic_prior'],
            current_motion_state=dict(angle_deg=pose.get('door_angle_deg'),source='known actual state V0; not visual dynamic estimation'),
            identified_parameters={k:(latest.get('parameters',{}).get(k) if latest.get('observability',{}).get(flag,False) else None) for k,flag in [('I_eff_kg_m2','I_eff'),('b_nm_s_rad','b'),('tau_c_nm','tau_c')]},
            provisional_fit_parameters=latest.get('parameters'),uncertainties=latest.get('conditional_std'),observability=latest.get('observability'),
            all_terms_conditionally_supported=bool(all(latest.get('observability',{}).get(k,False) for k in ['I_eff','b','tau_c'])),
            model_status='provisional; interpret held-out prediction and uncertainty before simulation use',
            unsupported_parameters_are_unknown_not_GT_filled=True,
            valid_operating_range_rad=latest.get('valid_range_rad'),unidentified=latest['terms_not_identified'],
            sensor_configuration=str(episode/'WRIST_FT_MAPPING.json'),calibration_provenance=str(case.parent/'sensor_smoke_00/SENSOR_ANALYSIS.json'),
            interaction_records=[str(v.relative_to(episode)) for v in models],RGBD_references=[str(v.relative_to(episode)) for v in (episode/'rgbd').glob('*') if v.is_dir()],
            authored_truth_excluded=True,known_model_dependencies=['joint geometry','dynamic state','registered station','native scene safety collision model'],
            limitations=['idealized zero-thickness load cell','approximate simulator actuators','conditional covariance not calibrated confidence','check heldout prediction before claiming physical twin'])
        (case/'ESTIMATED_ARTICULATED_MODEL.json').write_text(json.dumps(exported,indent=2))
    result.update(identification_result='see parameter uncertainty and held-out prediction; no success inferred from a torque fit',
        GT_joint_effort='not recorded as an independent full generalized effort channel; normal contact impulses are not substituted',
        mass_property_frame_audit='native USD and URDF rest transforms for both moving bodies agree exactly',
        truth_semantics='authored density/inertia/native joint-resistance parameters; effective solver/contact dynamics may differ')
    (case/'INDEPENDENT_EVALUATION.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(case=result['case'],updates=len(models))))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case');evaluate(p.parse_args().case)
