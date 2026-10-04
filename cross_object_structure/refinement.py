"""Measured-EE refinement with complete-action validation; frozen fitter reused."""
import copy
import json
from pathlib import Path
import numpy as np
from interactive_twin_refinement.fitting import fit_se3, predict_action, modality_consistency
from interactive_twin_refinement.data import action_poses
from interactive_twin_observable.uncertainty import observable, travel


def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False))


def measured_action(rows, phase):
    selected=[r for i,r in enumerate(rows) if r['phase']==phase and i%8==0]
    action={'phase':phase,'time_s':[r['t'] for r in selected],
            'ee_T':[r['T_tcp'] for r in selected],
            'observable_anomaly':[bool(r.get('relative_translation_slip_m',0)>.003 or min(r['forces_n'].values())<=0) for r in selected]}
    return action_poses(action) if selected else (np.empty((0,4,4)), {'phase':phase,'kept':0})


def refine(rows, discovery, policy, output):
    out=Path(output); train,audit=measured_action(rows,'ESTIMATED_FOLLOW')
    validation,va=measured_action(rows,'STRUCTURE_VALIDATION')
    reverse,ra=measured_action(rows,'REFINEMENT_REVERSE')
    result={'accepted':False,'GT_inputs':False,'heldout_read':False,'training_filter':audit,
            'validation_filter':va,'reverse_filter':ra,'reverse_observable':observable(reverse),
            'reverse_required':False,'training_travel_m':travel(train),'validation_travel_m':travel(validation)}
    if len(train)<12 or travel(train)<policy['selection']['minimum_train_motion_m'] or not observable(validation):
        result['status']='INSUFFICIENT_INDEPENDENT_STRUCTURE_MOTION'
        write(out/'structure_selection.json',result); return result
    fit=fit_se3(train,**policy['fitting'])
    noise={k:v for k,v in policy['fitting'].items() if 'noise' in k}
    mc=modality_consistency(train,**noise)
    predictions={name:predict_action(validation,f,**noise) for name,f in [('discovery',discovery),('refined',fit)]}
    segments=[]
    for name,T in [('opening_first_half',train[:len(train)//2]),('opening_second_half',train[len(train)//2:]),('reverse',reverse)]:
        if not observable(T):
            segments.append({'name':name,'status':'UNAVAILABLE_BELOW_UNCHANGED_1MM_OR_SAMPLE_THRESHOLD'});continue
        sf=fit_se3(T,**policy['fitting']); sm=modality_consistency(T,**noise)
        angle=float(np.rad2deg(np.arccos(np.clip(abs(np.dot(sf['revolute']['axis'],fit['revolute']['axis'])),0,1))))
        segments.append({'name':name,'status':'FITTED','fit':sf,'modality':sm,'axis_difference_deg':angle,
                         'consistent':angle<=3*(sm['uncertainty_scale_deg']+mc['uncertainty_scale_deg'])})
    rule=policy['selection']; new=predictions['refined']['normalized_rmse']; old=predictions['discovery']['normalized_rmse']
    accepted=bool(fit['optimizer']['success'] and mc['consistent_with_noise'] and
                  all(s.get('consistent',True) for s in segments) and
                  new<=(1-rule['minimum_validation_improvement_fraction'])*old and
                  new<=rule['maximum_validation_rms_noise_units'])
    result.update(status='REFINED_MODEL_ACCEPTED' if accepted else 'REFINEMENT_NOT_VALIDATED',
                  accepted=accepted,refined=fit,discovery=discovery,validation=predictions,
                  modality_consistency=mc,segments=segments,selection_rule=rule,
                  support=train.tolist(),observed_range_rad=fit['revolute']['angle_span_rad'])
    write(out/'refined_articulation.json',fit);write(out/'structure_selection.json',result)
    return result


def adopt(memory,result):
    """Update only structured memory. The established contact/control is untouched."""
    if not result['accepted']: raise RuntimeError('STRUCTURE_REFINEMENT_NOT_VALIDATED')
    fit=copy.deepcopy(result['refined']); old_axis=np.asarray(memory.estimate['revolute']['axis'])
    if old_axis@np.asarray(fit['revolute']['axis'])<0: memory.follow_sign*=-1.
    memory.estimate=fit
    memory.estimates.append({'fit':fit,'accepted':True,'observation_count':len(memory.poses),
                            'source':'robust_SE3_complete_action_validation','training_support_file':'structure_selection.json'})
    memory.save()
