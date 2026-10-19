"""Observable-only train/validation selection with explicit structural ambiguity."""
import numpy as np
from interactive_twin.sysid import noise_normalized_distance
from interactive_twin_refinement.physics import score


def compare_actions(reference,predicted,noise):
    if set(reference)!={'P1','P2','P3'} or set(predicted)!={'P1','P2','P3'}:
        raise ValueError('ONLY_COMPLETE_TRAIN_VALIDATION_ACTIONS_ALLOWED_NO_P4')
    result={}
    for p in ('P1','P2','P3'):
        d=score(reference[p],predicted[p],noise)
        r=d['post_drive_drift_reference'];s=d['post_drive_drift_prediction']
        drift=abs(r['net_drift_m']-s['net_drift_m']) if r is not None and s is not None else 0.
        d['held_no_task_drive_drift_error_m']=drift
        # Trajectory/velocity/start-time already included; terminal held drift
        # adds one observable, noise-normalized response constraint.
        n=len(d['noise_normalized_metrics']);d['selection_loss']=(n*d['normalized_loss']+(drift/noise.position_m)**2)/(n+1)
        result[p]=d
    return {'actions':result,'train_loss':float(np.mean([result[p]['selection_loss'] for p in ('P1','P2')])),
            'validation_loss':result['P3']['selection_loss']}


def choose(reference,candidates,noise,config):
    if set(reference)!={'P1','P2','P3'}:raise ValueError('TEST_ACCESS_DENIED')
    good=[];failures=[]
    for c in candidates:
        if set(c['logs'])!={'P1','P2','P3'}:
            failures.append({k:c[k] for k in ('candidate_id','status')});continue
        good.append({k:c[k] for k in ('candidate_id','structure_id','tau_c','b')}|compare_actions(reference,c['logs'],noise))
    if not good:return {'status':'NO_COMPLETE_CONDITIONAL_TRAINING','failures':failures,'test_read':False}
    def select(rows):
        floor=min(r['train_loss'] for r in rows);plausible=[r for r in rows if r['train_loss']<=floor+config['selection']['profile_loss_delta']]
        best=min(plausible,key=lambda r:r['validation_loss'])
        return best,plausible
    prior=[r for r in good if (r['tau_c'],r['b'])==(config['wrong_prior']['tau_c'],config['wrong_prior']['b'])]
    fixed=next((r for r in prior if r['structure_id']=='S0'),None)
    structural,support_prior=select(prior) if prior else (None,[])
    joint,support=select(good)
    intervals={k:[min(r[k] for r in support),max(r[k] for r in support)] for k in ('tau_c','b')}
    full={k:[min(v),max(v)] for k,v in config['physics_grid'].items()}
    adequate=joint['validation_loss']**.5<=config['selection']['max_validation_rms_noise_units']
    unresolved_structures=sorted({r['structure_id'] for r in support})
    # Model selection can yield a useful predictor without unique physical
    # recovery. No calibrated torque channel exists; even a singleton is a grid
    # resolution, not a precise physical measurement.
    ambiguous=len(unresolved_structures)>1 or any(intervals[k][0]!=intervals[k][1] for k in intervals)
    status='STRUCTURE_PHYSICS_CONFUNDED' if ambiguous else 'GRID_PREFERRED_NOT_CONTINUOUSLY_IDENTIFIED'
    if not adequate:status='UNIDENTIFIABLE_MODEL_RESPONSE_RESIDUAL';intervals=full
    methods={'fixed_refined_wrong_prior':fixed,'uncertain_structure_same_prior':structural,'uncertain_structure_calibrated':joint}
    return {'status':status,'methods':methods,'parameter_intervals':intervals,'train_supported_structure_ids':unresolved_structures,
            'profile_support':[{k:r[k] for k in ('candidate_id','structure_id','tau_c','b','train_loss','validation_loss')} for r in support],
            'interval_semantics':'discrete empirical profile support; not statistical confidence interval or exact friction recovery',
            'validation_adequate':bool(adequate),'candidate_losses':good,'failures':failures,'test_read':False,
            'train_actions':['P1','P2'],'validation_actions':['P3'],'zero_drive_state':'physically grasped, task drive off; robot damping remains',
            'joint_truth_used':False,'reference_resistance_used':False,'hardware_torque_scale_calibrated':False}
