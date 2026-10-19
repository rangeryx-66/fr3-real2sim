"""Preserve the old high-fidelity result and separately evaluate task utility."""
import json
import numpy as np
from active_structure.refinement import refine as high_fidelity
from cross_object_structure.refinement import measured_action
from operational_structure.prediction import prediction


def refine(rows,discovery,policy,output):
    result=high_fidelity(rows,discovery,policy,output)
    result['high_fidelity_accepted']=result['accepted']
    if 'refined' not in result:return result
    train,_=measured_action(rows,'ESTIMATED_FOLLOW');holdout,_=measured_action(rows,'STRUCTURE_VALIDATION')
    T1=prediction(holdout,discovery);T2=prediction(holdout,result['refined']);p=policy['active']['operational']
    # All scales originate in pre-TEST DEV calibration; no GT enters acceptance.
    reliable=bool(T2['observed_travel_m']>=policy['validation']['observability_m'] and
      T2['position_rmse_m']<=p['position_rmse_m'] and T2['rotation_rmse_rad']<=p['rotation_rmse_rad'] and
      T2['endpoint_error_m']<=p['endpoint_error_m'] and T2['tangent_error_deg'] is not None and T2['tangent_error_deg']<=p['tangent_error_deg'])
    nonworse=bool(T2['position_rmse_m']<=T1['position_rmse_m']+p['position_repeat_scale_m'] and
                 T2['rotation_rmse_rad']<=T1['rotation_rmse_rad']+p['rotation_repeat_scale_rad'])
    improved=bool(T2['position_rmse_m']<.95*T1['position_rmse_m'])
    stable=bool(result['refined']['optimizer']['success'] and result.get('candidate_consistency',{}).get('status')!='UNOBSERVABLE' and
                all(x.get('consistent',True) for x in result.get('segments',[])))
    train_old=prediction(train,discovery);train_new=prediction(train,result['refined'])
    training_nonworse=train_new['position_rmse_m']<=train_old['position_rmse_m']+p['position_repeat_scale_m']
    operational=bool(stable and reliable and nonworse and training_nonworse)
    result.update(accepted=operational,operational_predictive_accepted=operational,
      operational_requires_safe_continuation=True,operational_structure=False,
      status='OPERATIONAL_PREDICTION_ACCEPTED' if operational else 'OPERATIONAL_PREDICTION_REJECTED',
      task_prediction={'T1':T1,'T2':T2,'reliable':reliable,'nonworse':nonworse,'improved':improved,
                       'stable_family':stable,'training_nonworse':bool(training_nonworse),'heldout_used_in_fit':False},
      accuracy_threshold_m=.0003)
    (output/'structure_selection.json').write_text(json.dumps(result,indent=2));return result
