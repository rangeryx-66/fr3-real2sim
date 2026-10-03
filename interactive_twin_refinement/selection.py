"""Training/whole-reverse validation selection; no GT or held-out access."""
from pathlib import Path
import json
import numpy as np
from interactive_twin_refinement.data import action_poses
from interactive_twin_refinement.fitting import fit_se3,predict_action,modality_consistency

def read(p):return json.loads(Path(p).read_text())
def write(p,v):Path(p).parent.mkdir(parents=True,exist_ok=True);Path(p).write_text(json.dumps(v,indent=2,allow_nan=False))
def select(experiment,eid):
    out=experiment.out/eid;target=out/'structure_selection.json'
    if target.exists():return read(target)
    review=read(out/'historical_review.json');old=review['old'];folder=out/'refinement_once'
    records=read(folder/'measured_actions.json') if (folder/'measured_actions.json').exists() else {'actions':[]}
    opening=next((a for a in records['actions'] if a['phase']=='ESTIMATED_FOLLOW'),None)
    reverse=next((a for a in records['actions'] if a['phase']=='REFINEMENT_REVERSE'),None)
    policy=experiment.c['selection'];noise={k:v for k,v in experiment.c['fitting'].items() if 'noise' in k}
    diagnostics={};train=None;val=None
    if opening:
        train,diagnostics['training_filter']=action_poses(opening,experiment.c['refinement']['switch_exclusion_s'])
    if reverse:
        val,diagnostics['validation_filter']=action_poses(reverse,experiment.c['refinement']['switch_exclusion_s'])
    if train is None or len(train)<12:
        job,source=experiment.source(eid);train=np.asarray(read(source/'structured_memory.json')['supporting_observations'])
        diagnostics['training_fallback']='historical completed action; supplemental safety failure retained'
    if val is None or len(val)<12:
        if eid.startswith('dev'):
            path=experiment.old/'episodes'/eid/'final_updated_interaction/structured_memory.json'
            val=np.asarray(read(path)['supporting_observations']);diagnostics['validation_fallback']=str(path)
        else:
            result={'status':'NO_INDEPENDENT_VALIDATION_ACTION','accepted':False,'failure_stage':'structure_validation','GT_used':False,**diagnostics};write(target,result);return result
    candidate=fit_se3(train,**experiment.c['fitting']);modality=modality_consistency(train,**noise)
    predictions={k:predict_action(val,v,**noise) for k,v in [('old',old),('refined',candidate)]}
    # Segment estimates are a consistency diagnostic only, never relabeled as
    # independent test actions or additional motion-angle coverage.
    segments=[]
    for name,poses in [('opening_first_half',train[:len(train)//2]),('opening_second_half',train[len(train)//2:]),('reverse_validation',val)]:
        if len(poses)>=12:
            f=fit_se3(poses,**experiment.c['fitting']);a=np.asarray(f['revolute']['axis']);b=np.asarray(candidate['revolute']['axis'])
            mc=modality_consistency(poses,**noise)
            segments.append({'name':name,'fit':f,'axis_angle_to_training_deg':float(np.rad2deg(np.arccos(np.clip(abs(a@b),0,1)))),'modality':mc})
    segment_ok=all(s['axis_angle_to_training_deg']<=3*(s['modality']['uncertainty_scale_deg']+modality['uncertainty_scale_deg']) for s in segments)
    new_score=predictions['refined']['normalized_rmse'];old_score=predictions['old']['normalized_rmse']
    validation_travel=float(np.max(np.linalg.norm(val[:,:3,3]-val[0,:3,3],axis=1)))
    accepted=bool(validation_travel>=policy['minimum_reverse_motion_m'] and candidate['optimizer']['success'] and candidate['travel_m']>=policy['minimum_train_motion_m'] and modality['consistent_with_noise'] and segment_ok and new_score<=(1-policy['minimum_validation_improvement_fraction'])*old_score and new_score<=policy['maximum_validation_rms_noise_units'])
    result={'status':'REFINED_MODEL_ACCEPTED' if accepted else 'REFINEMENT_NOT_VALIDATED','accepted':accepted,
            'old':old,'refined':candidate,'validation':predictions,'modality_consistency':modality,'segment_diagnostics':segments,
            'validation_travel_m':validation_travel,'segment_consistency_passed':segment_ok,'GT_used':False,'test_response_used':False,
            'axis_vertical_prior':False,'selection_rule':policy,**diagnostics}
    write(out/'refined_articulation.json',candidate)
    chosen=candidate if accepted else old
    # Export separate discovery and calibration memory. Accepted fit support is
    # training only; validation poses never become reference/fitting samples.
    write(out/'estimated_articulation.json',chosen)
    support=train.tolist() if accepted else read(experiment.source(eid)[1]/'structured_memory.json')['supporting_observations'][:old['sample_count']]
    write(out/'structured_memory.json',{'GT_inputs':False,'source':'completed measured-EE actions; discovery/calibration separated','estimated_articulation':chosen,'fit_history':[{'fit':chosen,'accepted':True,'observation_count':len(support)}],'supporting_observations':support,'attempt_history':[]})
    write(target,result)
    return result
