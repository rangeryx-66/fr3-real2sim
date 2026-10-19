"""EE-derived correlated structure candidates; no simulator truth inputs."""
import copy,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from interactive_twin_refinement.fitting import basis,unit,predict_action
from interactive_twin_refinement.data import action_poses

def build(selection,actions,policy):
    fit=selection['refined'];a=unit(fit['revolute']['axis']);c=np.asarray(fit['revolute']['point_on_axis']);B=basis(a)
    segments=[s['fit'] for s in selection['segment_diagnostics']]
    vectors=[]
    for s in segments:
        aa=unit(s['revolute']['axis']);aa*=1 if aa@a>=0 else -1
        cc=np.asarray(s['revolute']['point_on_axis']);cc+=aa*(a@(c-cc))/(a@aa)
        vectors.append(np.r_[B.T@(aa-a),B.T@(cc-c)])
    V=np.asarray(vectors);scale=np.sqrt(np.mean(V**2,axis=0));scale=np.maximum(scale,[1e-6,1e-6,1e-6,1e-6])
    _,sv,right=np.linalg.svd(V/scale,full_matrices=False)
    deltas=[np.zeros(4)]
    for i in range(min(2,len(sv))):
        v=right[i]*sv[i]/np.sqrt(len(V))*scale*policy['segment_scatter_fraction']
        deltas.extend([v,-v])
    train=action_poses(next(x for x in actions['actions'] if x['phase']=='ESTIMATED_FOLLOW'))[0]
    val=action_poses(next(x for x in actions['actions'] if x['phase']=='REFINEMENT_REVERSE'))[0]
    train_ref=predict_action(train,fit)['normalized_rmse'];val_old=predict_action(val,selection['old'])['normalized_rmse']
    records=[]
    for i,d in enumerate(deltas):
        new=copy.deepcopy(fit);aa=unit(a+B@d[:2]);cc=c+B@d[2:];cc-=aa*np.dot(aa,cc-c)
        new['revolute'].update(axis=aa.tolist(),point_on_axis=cc.tolist())
        tr=predict_action(train,new);va=predict_action(val,new)
        accepted=tr['normalized_rmse']<=train_ref*policy['maximum_training_ratio_to_refined'] and va['normalized_rmse']<=val_old*policy['maximum_validation_ratio_to_old']
        records.append({'structure_id':f'S{i}','estimate':new,'accepted':bool(accepted),'training':tr,'validation':va,'local_delta':d.tolist()})
    return {'source':'opening half / reverse EE segment scatter, correlated axis and axis-line changes','axis_line_gauge':'plane through refined axis point normal to refined axis','segment_vectors':V.tolist(),'scale':scale.tolist(),'singular_values':sv.tolist(),'scatter_fraction':policy['segment_scatter_fraction'],'ground_truth_read':False,'candidates':records,'interval_semantics':'local empirical uncertainty, not calibrated confidence interval'}
