"""Post-episode object-truth evaluation; never imported by native controller."""
import json,csv
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np
from scipy.spatial.transform import Rotation
from interactive_twin_refinement.fitting import fit_se3
from interaction_identification.fitting import evaluate
from operational_structure.prediction import prediction


def read(p,default=None):
    p=Path(p);return json.loads(p.read_text()) if p.exists() else default

def write(p,x):Path(p).write_text(json.dumps(x,indent=2))
def table(p,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(p).open('w') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows([{k:json.dumps(v) if isinstance(v,(dict,list)) else v for k,v in r.items()} for r in rows])

def object_prediction(E,O,fit):
    """Measured EE orientation supplies phase. No test position/reference fit."""
    E=np.asarray(E);O=np.asarray(O);a=np.asarray(fit['revolute']['axis']);a/=np.linalg.norm(a);c=np.asarray(fit['revolute']['point_on_axis'])
    theta=Rotation.from_matrix(E[:,:3,:3]@E[0,:3,:3].T).as_rotvec()@a
    R=Rotation.from_rotvec(theta[:,None]*a).as_matrix();p=c+np.einsum('nij,j->ni',R,O[0,:3,3]-c)
    ep=np.linalg.norm(p-O[:,:3,3],axis=1);er=Rotation.from_matrix((R@O[0,:3,:3])@O[:,:3,:3].transpose(0,2,1)).magnitude()
    return {'object_position_rmse_m':float(np.sqrt(np.mean(ep**2))),'object_rotation_rmse_rad':float(np.sqrt(np.mean(er**2))),'object_endpoint_error_m':float(ep[-1]),'predicted_object_positions':p.tolist(),'object_position_errors_m':ep.tolist(),'phase':'measured EE orientation, conditional geometric prediction, not autonomous dynamics','reference':'first held-out object observation; no held-out reference fit'}

def linear_prediction(E,O,probe):
    X=np.asarray(probe);_,_,v=np.linalg.svd(X[:,:3,3]-X[:,:3,3].mean(0));d=v[0]
    p=O[0,:3,3]+((E[:,:3,3]-E[0,:3,3])@d)[:,None]*d
    ep=np.linalg.norm(p-O[:,:3,3],axis=1);er=Rotation.from_matrix(O[:,:3,:3]@O[0,:3,:3].T).magnitude()
    return {'object_position_rmse_m':float(np.sqrt(np.mean(ep**2))),'object_rotation_rmse_rad':float(np.sqrt(np.mean(er**2))),'object_endpoint_error_m':float(ep[-1]),'predicted_object_positions':p.tolist(),'object_position_errors_m':ep.tolist(),'phase':'measured EE translation projected on frozen discovery tangent; no global hinge model'}

def selected(rows,phase):
    x=[r for r in rows if r['phase'] in phase]
    if len(x)<12:return []
    # Same frozen transition trimming as original action interface.
    return [r for r in x if r['t']>=x[0]['t']+.5 and r['t']<=x[-1]['t']-.5]

def trajectory_metrics(rows):
    phases={'EXPLORATORY','PROBE_HOLD','ESTIMATED_FOLLOW','STRUCTURE_VALIDATION_SETTLE','STRUCTURE_VALIDATION','STRUCTURE_MODEL_SELECTION','HELDOUT_MANIPULATION','HELDOUT_DWELL','OPERATIONAL_CONTINUATION','FINAL_HOLD'}
    x=[r for r in rows if r['phase'] in phases]
    if not x:return {}
    E=np.asarray([r['T_ee'] for r in x]);O=np.asarray([r['T_object'] for r in x]);rel=np.linalg.inv(O)@E
    drift=np.linalg.norm(rel[:,:3,3]-rel[0,:3,3],axis=1)
    rot=Rotation.from_matrix(rel[:,:3,:3]@rel[0,:3,:3].T).magnitude()
    return {'maximum_true_relative_translation_drift_m':float(drift.max()),'final_true_relative_translation_drift_m':float(drift[-1]),'maximum_true_relative_rotation_drift_deg':float(np.rad2deg(rot.max())),
      'interaction_path_m':float(np.linalg.norm(np.diff(E[:,:3,3],axis=0),axis=1).sum()),'interaction_duration_s':x[-1]['t']-x[0]['t'],'true_relative_drift_is_evaluation_only':True}

def analysis(folder,method,episode,asset):
    report=read(folder/'report.json',{});rows=read(folder/'evaluation_private/object_trajectory.json',[])
    out={'episode_id':episode,'asset_id':asset,'method':method,'status':report.get('status','NOT_RUN'),'bilateral_grasp':report.get('bilateral_hold_established',False),'original_task_success':report.get('success',False),
      'actual_opening_deg':report.get('evaluation',{}).get('actual_door_displacement_deg'), 'minimum_joint_margin_rad':report.get('minimum_joint_margin_rad'),'peak_load_n':report.get('peak_finger_handle_force_n'),
      'online_observability_limit':'contact-plane drift only; full drift unavailable online','video':str(folder/'contact_baseline.mp4')}
    out.update(trajectory_metrics(rows))
    if not rows:return out,{}
    train=selected(rows,{'ESTIMATED_FOLLOW'});probe=selected(rows,{'EXPLORATORY'});test=selected(rows,{'HELDOUT_MANIPULATION'});validation=selected(rows,{'STRUCTURE_VALIDATION'})
    out['heldout_reached']=len(test)>=12;out['hold_completed']=any(r['phase']=='FINAL_HOLD' for r in rows)
    out['probe_duration_s']=probe[-1]['t']-probe[0]['t'] if probe else 0.;out['probe_path_m']=float(np.linalg.norm(np.diff(np.asarray([r['T_ee'] for r in probe])[:,:3,3],axis=0),axis=1).sum()) if probe else 0.
    select=read(folder/'structure_selection.json',{});out['model_update_accepted']=select.get('accepted',False);out['controller_model']=select.get('controller_model','none' if method=='B0' else 'original')
    truth=read(folder/'evaluation_only.json',{}).get('ground_truth',{})
    discovery=read(folder/'discovery_articulation.json');refined=read(folder/'refined_articulation.json');start=read(folder/'heldout_start.json',{})
    models={'T1_discovery':discovery,'T2_refined':refined,'controller_model':start.get('selected_estimate')}
    curves={}
    if test:
        E=np.asarray([r['T_ee'] for r in test]);O=np.asarray([r['T_object'] for r in test])
        for name,model in models.items():
            if model is None or model.get('joint_type')!='revolute':continue
            p=object_prediction(E,O,model);ee=prediction(E,model);g=evaluate(model,truth,E[0,:3,3])
            curves[name]={'object':p,'EE':ee,'accuracy':g,'model':model}
            for k in ['object_position_rmse_m','object_rotation_rmse_rad','object_endpoint_error_m']:out[name+'_'+k]=p[k]
            out[name+'_ee_position_rmse_m']=ee['position_rmse_m'];out[name+'_axis_error_deg']=g.get('axis_angular_error_deg');out[name+'_axis_line_error_m']=g.get('axis_line_distance_m')
        if probe:
            p=linear_prediction(E,O,np.asarray([r['T_ee'] for r in probe]));curves['B0_local_linear']=p
            if method=='B0':out['controller_object_rmse_m']=p['object_position_rmse_m']
        if 'controller_model' in curves:out['controller_object_rmse_m']=curves['controller_model']['object']['object_position_rmse_m']
    write(folder/'post_evaluation_summary.json',out);write(folder/'post_evaluation_curves.json',curves)
    return out,curves

def upper_bound(folder,c):
    rows=read(folder/'evaluation_private/object_trajectory.json',[]);train=selected(rows,{'EXPLORATORY','ESTIMATED_FOLLOW'});test=selected(rows,{'STRUCTURE_VALIDATION'})
    if not train or not test:return {'case':folder.name,'status':'INSUFFICIENT_TRAJECTORY'}
    gt=read(folder/'evaluation_only.json')['ground_truth'];E=np.asarray([r['T_ee'] for r in train]);O=np.asarray([r['T_object'] for r in train]);Et=np.asarray([r['T_ee'] for r in test]);Ot=np.asarray([r['T_object'] for r in test]);result={'case':folder.name,'oracle_not_online_success':True,'training_period':'EXPLORATORY + ESTIMATED_FOLLOW','excluded_segment':'STRUCTURE_VALIDATION','prediction_interpretation':'conditional geometry; measured orientation supplies phase'}
    fits={}
    for name,X,Xt in [('EE',E,Et),('OBJECT_ORACLE',O,Ot)]:
        f=fit_se3(X,**c);g=evaluate(f,gt,X[0,:3,3]);p=object_prediction(Et,Ot,f);pc=prediction(Ot,f)
        result[name+'_axis_error_deg']=g['axis_angular_error_deg'];result[name+'_axis_line_error_m']=g['axis_line_distance_m'];result[name+'_object_rmse_EE_phase_m']=p['object_position_rmse_m'];result[name+'_object_rmse_OBJECT_phase_m']=pc['position_rmse_m'];result[name+'_training_residual_m']=f['revolute']['position_rmse_m'];fits[name]=f
    result.update(trajectory_metrics(rows));write(folder/'observation_upper_bound.json',{'metrics':result,'fits':fits});return result

def summarize(out,manifest):
    out=Path(out);original=Path(manifest['comparison_jobs'][0]['output']).parents[2] if manifest['comparison_jobs'] else out
    results=[];curves={};assets={e['episode_id']:e['asset_id'] for e in manifest['episodes']}
    for j in manifest['comparison_jobs']:
        eid=j['episode_id'].rsplit('_',1)[0];r,p=analysis(Path(j['output']),j['paper_method'],eid,assets[eid]);results.append(r);curves[eid+'_'+j['paper_method']]=p
    for r in manifest['shared_prefix_failures']:
        for method in manifest['config']['methods']:results.append({**r,'method':method,'original_task_success':False,'heldout_reached':False,'reused_frozen_shared_prefix_failure':True})
    table(out/'MAIN_TABLE.csv',results)
    groups=defaultdict(list)
    for r in results:groups[(r['asset_id'],r['method'])].append(r)
    pa=[{'asset_id':a,'method':m,'episodes':len(rs),'successful_configs':sum(bool(r.get('original_task_success')) for r in rs),'heldout_configs':sum(bool(r.get('heldout_reached')) for r in rs),'bilateral_grasps':sum(bool(r.get('bilateral_grasp')) for r in rs)} for (a,m),rs in groups.items()];table(out/'per_asset.csv',pa)
    counts=Counter((r['method'],r['status']) for r in results);table(out/'failure_breakdown.csv',[{'method':m,'reason':s,'count':n} for (m,s),n in counts.items()])
    cfg=read(Path(manifest['logging_only_jobs'][0]['output'])/'job_private.json')['structure_protocol']['fitting'];ub=[upper_bound(Path(j['output']),cfg) for j in manifest['logging_only_jobs']];table(out/'OBSERVATION_UPPER_BOUND.csv',ub)
    pair=[]
    for eid in assets:
        paths=[out/'comparisons'/eid/m/'observable_grasp_start.json' for m in manifest['config']['methods']];s=[read(p) for p in paths]
        if all(s):pair.append({'episode_id':eid,'maximum_start_q_difference_rad':max(float(np.max(np.abs(np.asarray(x['q'])-np.asarray(s[0]['q'])))) for x in s),'maximum_start_ee_translation_difference_m':max(float(np.linalg.norm(np.asarray(x['T_ee'])[:3,3]-np.asarray(s[0]['T_ee'])[:3,3])) for x in s),'initialization':'same physical prefix; no copied impulse or attachment'})
    table(out/'paired_grasp_states.csv',pair)
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    plots=out/'plots';plots.mkdir(exist_ok=True)
    for case,p in curves.items():
        if not p:continue
        fig,ax=plt.subplots(figsize=(8,3))
        for name,v in p.items():
            y=v.get('object',v).get('object_position_errors_m')
            if y is not None:ax.plot(np.arange(len(y)),np.asarray(y)*1000,label=name)
        ax.set(xlabel='held-out sampled frame',ylabel='object position error (mm)',title=case+' | conditional geometry');ax.legend();fig.tight_layout();fig.savefig(plots/(case+'.png'));plt.close(fig)
    summary={}
    for m in manifest['config']['methods']:
        rs=[r for r in results if r['method']==m];use=[r for r in rs if r.get('heldout_reached')]
        summary[m]={'all_episodes':len(rs),'task_success':sum(bool(r.get('original_task_success')) for r in rs),'heldout_reached':len(use),'actual_opening_5deg':sum((r.get('actual_opening_deg') or 0)>=5 for r in rs),'assets_with_task_success':len({r['asset_id'] for r in rs if r.get('original_task_success')})}
    write(out/'summary.json',{'methods':summary,'diagnostic_not_unseen':True,'observation_upper_bound':ub,'paired_states':pair})
    lines=['# Internal structure-update comparison','','Diagnostic cohort previously exposed: 6 assets x 2 configurations; no fresh unseen claim.','', '|Method|task success /12|actual >=5 deg /12|held-out reached|successful assets /6|','|---|---:|---:|---:|---:|']
    for m,v in summary.items():lines.append(f"|{m}|{v['task_success']}/12|{v['actual_opening_5deg']}/12|{v['heldout_reached']}|{v['assets_with_task_success']}/6|")
    lines+=['','Operation, model acceptance, and reconstruction errors are separate columns in MAIN_TABLE.csv.','Predictions use measured EE rotation or translation as phase: **conditional geometric prediction**, not autonomous dynamics or physics prediction.','Object-truth logs and relative transforms are evaluation-only; no object state is returned to controllers.','B0: frozen observed-direction compliant interaction, no global hinge update; local linear prediction is evaluated offline.','B1: one discovery estimate, frozen later. B2: existing online/refined update; rejected update falls back to discovery, never selected using final held-out test.','The historical 0.30 mm EE reconstruction metric is not object structure truth or a universal task gate.','No grasp/contact/proxy/base/physics/fitter/safety parameters were retuned.','All pre-contact failures stay in system denominators; paired post-grasp state matching is separately reported.','', '## Observation upper bound','See OBSERVATION_UPPER_BOUND.csv. Object-oracle fits are diagnostics, not method success.','', '## Paper evidence boundary','These comparisons are internal ablations, not official Act2See/Tac-Man reproduction.','>=5 degrees demonstrates small interaction, not full door opening.','Full relative drift is available only to the independent simulator evaluator; bilateral contact does not establish zero slip.','Final independent unseen tests, real-world friction measurement and a new perception stack are outside this round.']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
