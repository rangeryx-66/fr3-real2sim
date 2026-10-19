"""Three effective hinge parameters; wrist-only observations, no hidden plant."""
import numpy as np
from xarm_ft_v1_estimator import local_process,coverage,fit,features,forward,CONFIG


def analyze(data,prior):
    samples=data['samples'];train=[s for s in samples if not s['segment'].startswith('VALIDATION')];test=[s for s in samples if s['segment'].startswith('VALIDATION')]
    result=dict(model='I_eff*qdd+b*qd+Coulomb*sign(qd)',source='compensated wrist six-axis fixed-interface wrench',training_bins=len(train),validation_bins=len(test),coverage=coverage(data),hidden_dynamics_read=False,
        gravity='vertical known hinge: negligible projected gravity; nonzero gravity identification not claimed')
    if len(train)<8:return dict(result,status='insufficient independent fitting bins',identified_components=[])
    model=fit(train,'constant');result['fit']=model
    both=any(s['qd']>0 for s in train) and any(s['qd']<0 for s in train)
    contribution=np.sqrt(np.mean((features(train,'constant')*model['parameters'])**2,axis=0))
    result['identifiability']=dict(both_directions=both,rank=model['algebraic_rank'],contribution_RMS_Nm=contribution.tolist(),
        supported=[bool(s and c>CONFIG['effort_floor_nm'] and model['algebraic_rank']==3 and model['scaled_condition']<1e6 and (i==0 or both)) for i,(s,c) in enumerate(zip(model['supported'],contribution))],
        uncertainty='conditional regression only; bins correlated and load-cell idealized')
    if test:
        error=features(test,'constant')@model['parameters']-np.array([s['tau'] for s in test]);result['heldout_torque_RMSE_Nm']=float(np.sqrt(np.mean(error**2)))
        result['heldout_forward']={}
        for label in dict.fromkeys(data['segments']):
            if str(label).startswith('VALIDATION'):
                ids=np.flatnonzero((data['segments']==label)&data['valid'])
                if len(ids)>1:result['heldout_forward'][str(label)]=forward(data,ids,model)
    return result
