"""Blind complete-action input/response matching; no reference parameters input."""
import numpy as np
from interactive_twin.sysid import noise_normalized_distance,validate_log,_signature

def fit(reference, candidates, noise, policy):
    # P1/P2 training, P3 validation, P4 inaccessible here by API contract.
    if set(reference)!={'P1','P2','P3'}:raise ValueError('EXPECTED_TRAIN_AND_VALIDATION_ONLY')
    rows=[];vectors=[];X=[];failures=[]
    for item in candidates:
        if set(item.get('probes',{}))!={'P1','P2','P3'}:
            failures.append({'candidate_id':item['candidate_id'],'reason':item.get('status','CENSORED')});continue
        d={p:noise_normalized_distance(reference[p],item['probes'][p],noise) for p in ('P1','P2','P3')}
        rows.append({'candidate_id':item['candidate_id'],'tau_c':item['tau_c'],'b':item['b'],
                     'train_loss':float(np.mean([d[p]['normalized_loss'] for p in ('P1','P2')])),
                     'validation_loss':d['P3']['normalized_loss'],'metrics':d})
        vectors.append(np.concatenate([_signature(item['probes'][p],noise) for p in ('P1','P2')]))
        X.append([item['tau_c'],item['b']])
    full={k:[min(policy['grid'][k]),max(policy['grid'][k])] for k in ('tau_c','b')}
    if not rows:return {'status':'NO_COMPLETE_PHYSICS_ROLLOUTS','accepted_parameters':None,'parameter_intervals':full,'failures':failures}
    train_best=min(rows,key=lambda r:r['train_loss'])
    plausible=[r for r in rows if r['train_loss']<=train_best['train_loss']+policy['profile_loss_delta']]
    # Validation chooses among TRAIN-supported candidates only. Test is absent.
    best=min(plausible,key=lambda r:r['validation_loss'])
    intervals={k:[min(r[k] for r in plausible),max(r[k] for r in plausible)] for k in ('tau_c','b')}
    X=np.asarray(X);V=np.asarray(vectors);singular=[]
    if len(rows)>2 and np.all(np.ptp(X,axis=0)>0):
        A=(X-X.mean(0))/np.ptp(X,axis=0)
        J=np.linalg.lstsq(A,V-V.mean(0),rcond=None)[0]
        singular=(np.linalg.svd(J,compute_uv=False)/np.sqrt(V.shape[1])).tolist()
    rank=sum(v>1 for v in singular)
    complete=len(rows)==policy['maximum_candidates'] and not failures
    narrow=all(intervals[k][1]-intervals[k][0]<.8*(full[k][1]-full[k][0]) for k in full)
    # Evidence of a preferred grid point is not proof of calibrated torque scale
    # or of precise parameter recovery. Residual inadequacy prevents promotion.
    adequate=np.sqrt(best['validation_loss'])<=10.
    accepted=complete and rank==2 and narrow and adequate
    diagnostic_intervals=dict(intervals)
    if not complete or not adequate or rank<2:intervals=full
    return {'status':'IDENTIFIABLE_ON_FROZEN_GRID' if accepted else 'UNIDENTIFIABLE_UNDER_CURRENT_PROBE',
      'accepted_parameters':{k:best[k] for k in ('tau_c','b')} if accepted else None,
      'diagnostic_best':{k:best[k] for k in ('candidate_id','tau_c','b','train_loss','validation_loss')},
      'parameter_intervals':intervals,'diagnostic_grid_support':diagnostic_intervals,'interval_semantics':'discrete training profile support, not statistical confidence interval',
      'rank':rank,'normalized_singular_values':singular,'full_grid_complete':complete,'model_adequate_on_validation':bool(adequate),
      'candidate_losses':rows,'failures':failures,'train_actions':['P1','P2'],'validation_actions':['P3'],'test_read':False,
      'parameter_semantics':'effective simulator resistance parameters; no hardware torque scale',
      'current_or_effort_used':False,'J_eff_optimized':False}

def score(reference,predicted,noise):
    result=noise_normalized_distance(reference,predicted,noise)
    def drift(log):
        t=np.asarray(log['time_s']);cmd=log['commands'];fields=cmd['fields'];active=np.asarray(cmd['values'])[:,fields.index('drive_active')]
        pos=np.asarray(log['signals']['ee_T_world_tcp'])[:,:3,3]
        indices=np.flatnonzero(active>0)
        if not len(indices) or indices[-1]+1>=len(t):return None
        i=indices[-1]+1
        return {'duration_s':float(t[-1]-t[i]),'net_drift_m':float(np.linalg.norm(pos[-1]-pos[i])),
                'mean_speed_m_s':float(np.mean(np.linalg.norm(np.gradient(pos[i:],t[i:],axis=0),axis=1)))}
    result['post_drive_drift_reference']=drift(reference);result['post_drive_drift_prediction']=drift(predicted)
    return result
