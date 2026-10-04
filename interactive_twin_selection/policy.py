"""Complete-action batch ranking. No candidate is removed by an early probe."""
import numpy as np
from .metrics import score


def rank(candidates,bank,reference,calibration,policy):
    if 'stop_dwell' not in reference or len(reference)<3:raise ValueError('COMPLETE_AVAILABLE_PROTOCOL_REQUIRED')
    if any(x['split']=='heldout' for x in reference.values()):raise ValueError('HELDOUT_ACCESS_DENIED')
    rows=[];failures=[]
    for c in candidates:
        predictions=bank[c['candidate_id']]
        if any(k not in predictions or predictions[k]['log'] is None or not predictions[k]['safe'] for k in reference):
            failures.append({'candidate_id':c['candidate_id'],'reason':'CENSORED_NATIVE_PREDICTION'});continue
        actions={k:score(v,predictions[k]['log'],calibration) for k,v in reference.items()}
        train=[v['loss'] for k,v in actions.items() if k!='stop_dwell'];val=actions['stop_dwell']['loss']
        rows.append({**c,'train_loss':float(np.mean(train)),'validation_loss':val,
                     'combined_loss':float((np.mean(train)+val)/2),'actions':actions})
    if not rows:return {'status':'NO_COMPLETE_CANDIDATE','rows':[],'support':[],'failures':failures}
    rows.sort(key=lambda r:(r['combined_loss'],r['candidate_id']))
    # One complete action is the evidence unit; do not multiply by 240-Hz frames.
    logw=-.5*np.array([r['combined_loss'] for r in rows]);logw-=logw.max()
    w=np.exp(np.maximum(logw,-700));w/=w.sum()
    for r,l,p in zip(rows,logw,w):r.update(relative_log_weight=float(l),posterior_weight=float(p))
    support=[r for i,r in enumerate(rows) if i<policy['top_k'] or r['relative_log_weight']>=-policy['support_log_likelihood_delta']]
    adequate=rows[0]['validation_loss']**.5<=policy['adequacy_rms']
    retained=support if adequate else rows
    return {'status':'PREDICTOR_SELECTED_PARAMETERS_UNIDENTIFIABLE' if adequate else 'MODEL_MISMATCH',
            'rows':rows,'support':[{k:r[k] for k in ('candidate_id','structure_id','tau_c','b','tau_s','posterior_weight','relative_log_weight')} for r in support],
            'retained_parameters':[{k:r[k] for k in ('candidate_id','structure_id','tau_c','b','tau_s')} for r in retained],
            'parameter_intervals':{k:[min(r[k] for r in retained),max(r[k] for r in retained)] for k in ('tau_c','b','tau_s')},
            'best':rows[0],'model_adequate':bool(adequate),
            'all_candidates_validation_inadequate':all(r['validation_loss']**.5>policy['adequacy_rms'] for r in rows),
            'minimum_validation_rms':min(r['validation_loss']**.5 for r in rows),
            'failures':failures,'all_available_probes_scored_before_ranking':True,
            'sequential_pruning':False,'test_read':False,
            'support_semantics':'top-K union finite likelihood support; not calibrated continuous parameter confidence'}


def extension_anchors(ranking,config):
    grouped={}
    for row in ranking['rows']:grouped.setdefault(row['structure_id'],[]).append(row)
    groups=sorted(grouped.values(),key=lambda g:min(r['train_loss'] for r in g))[:config['maximum_structures']]
    result=[]
    for group in groups:
        for row in sorted(group,key=lambda r:(r['train_loss'],r['candidate_id']))[:config['anchors_per_structure']]:
            for extra in config['excess_static_effort_grid']:
                result.append({k:row[k] for k in ['candidate_id','structure_id','tau_c','b','asset']}|
                              {'candidate_id':row['candidate_id']+f'_static_{extra:g}',
                               'tau_s':row['tau_c']+extra,'model_family':'static_dynamic_viscous',
                               'anchor_candidate':row['candidate_id']})
    if len(result)>config['maximum_extra_parameter_points']:raise ValueError('STATIC_GRID_BUDGET_EXCEEDED')
    return result
