"""Reporting-only evidence inventory; never loaded by control or model selection."""
from pathlib import Path
import sys,json,collections,csv,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from paper_structure.analysis import read,write,table,selected,object_prediction,linear_prediction
from interaction_identification.fitting import evaluate


def local_rotation_phase_prediction(E,O,probe):
    """No-global-hinge local geometric baseline; same held-out rotation input."""
    X=np.asarray(probe,float);rv=Rotation.from_matrix(X[:,:3,:3]@X[0,:3,:3].T).as_rotvec()
    _,_,V=np.linalg.svd(rv,full_matrices=False);axis=V[0];phase=rv@axis
    if np.ptp(phase)<.002:return None # existing minimum angular evidence
    A=np.c_[np.ones(len(X)),phase];slope=np.linalg.lstsq(A,X[:,:3,3],rcond=None)[0][1]
    theta=Rotation.from_matrix(E[:,:3,:3]@E[0,:3,:3].T).as_rotvec()@axis
    EP=np.tile(E[0],(len(E),1,1));EP[:,:3,3]=E[0,:3,3]+theta[:,None]*slope
    EP[:,:3,:3]=Rotation.from_rotvec(theta[:,None]*axis).as_matrix()@E[0,:3,:3]
    predicted=EP@np.linalg.inv(E[0])@O[0];error=np.linalg.norm(predicted[:,:3,3]-O[:,:3,3],axis=1)
    rotation=Rotation.from_matrix(predicted[:,:3,:3]@O[:,:3,:3].transpose(0,2,1)).magnitude()
    return {'object_position_rmse_m':float(np.sqrt(np.mean(error**2))),'object_rotation_rmse_rad':float(np.sqrt(np.mean(rotation**2))),'object_endpoint_error_m':float(error[-1]),'object_position_errors_m':error.tolist(),'phase':'measured EE orientation only; local probe slope frozen; no global hinge center','training':'probe EE only; no object/GT/held-out fit'}


def report(config):
    c=read(config);out=ROOT/c['output'];manifest=read(out/'frozen_comparison_manifest.json')
    raw=list(csv.DictReader(open(out/'MAIN_TABLE.csv')));episodes={e['episode_id']:e for e in manifest['episodes']};rows=[];trace={};joint_models={}
    for r in raw:
        eid=r['episode_id'];m=r['method'];folder=out/'comparisons'/eid/m
        rr=read(folder/'report.json',{});x=read(folder/'evaluation_private/object_trajectory.json',[])
        r.update(actual_5deg_small_interaction=(rr.get('evaluation',{}).get('actual_door_displacement_deg') or 0)>=5.,model_rejection_is_not_operation_zero=True)
        if x:
            dt=float(np.median(np.diff([z['t'] for z in x])));probe=[z for z in x if z['phase']=='EXPLORATORY'];ref=[z for z in x if z['phase']=='ESTIMATED_FOLLOW'];hold=[z for z in x if z['phase']=='FINAL_HOLD']
            r['active_probe_time_s']=len(probe)*dt;r['refinement_or_compliant_follow_time_s']=len(ref)*dt
            r['hold_duration_s']=hold[-1]['t']-hold[0]['t']+dt if hold else 0.;r['hold_2s_completed']=r['hold_duration_s']>=1.99
            r['measured_probe_extent_m']=float(np.max(np.linalg.norm(np.asarray([z['T_ee'] for z in probe])[:,:3,3]-np.asarray(probe[0]['T_ee'])[:3,3],axis=1))) if probe else 0.
            truth=read(folder/'evaluation_only.json',{}).get('ground_truth');model=rr.get('estimate')
            post=[z for z in x if z['phase'] in ['EXPLORATORY','ESTIMATED_FOLLOW','STRUCTURE_VALIDATION','HELDOUT_MANIPULATION','HELDOUT_DWELL','OPERATIONAL_CONTINUATION','FINAL_HOLD']]
            if truth and post:
                OP=np.asarray([z['T_object'] for z in post]);angle=Rotation.from_matrix(OP[:,:3,:3]@OP[0,:3,:3].T).as_rotvec()@np.asarray(truth['axis_world']);r['post_grasp_object_rotation_deg']=float(np.rad2deg(angle[-1]));r['maximum_post_grasp_object_rotation_deg']=float(np.rad2deg(angle.max()))
            if model and model.get('joint_type')=='revolute' and truth:
                ev=evaluate(model,truth,np.asarray(x[0]['T_ee'])[:3,3]);r['final_estimate_axis_line_error_m']=ev.get('axis_line_distance_m');r['final_estimate_axis_angular_error_deg']=ev.get('axis_angular_error_deg');r['joint_type_correct']=ev.get('type_correct')
            r['final_estimate_role']='post-episode diagnostic only' if m=='B0' else 'online estimate';
            if m=='B0':r['joint_type_correct']='N/A: no global online model'
            status=rr.get('status','');r['dangerous_contact_event']=any(a in status for a in ['DANGEROUS','COLLISION','LOADED_CONTACT']);r['physical_hard_stop']=r['dangerous_contact_event'] or any(a in status for a in ['SUSTAINED_CONTACT_LOSS','SUSTAINED_CONTACT_PLANE_DRIFT','LOW_JOINT_MARGIN','LIMIT','NO_SAFE_IK'])
            r['model_confidence_stop']='MODEL_CONFIDENCE' in status
            trace[(eid,m)]=x
            s=read(folder/'structure_selection.json',{});discovery=read(folder/'discovery_articulation.json');refined=read(folder/'refined_articulation.json')
            joint_models[(eid,m)]={'discovery':discovery,'refined':refined,'controller':read(folder/'heldout_start.json',{}).get('selected_estimate')}
            # Segment uncertainty uses train-only candidates, not GT/test scores.
            fits=[z['fit'] for z in s.get('segments',[]) if 'fit' in z]
            if refined and fits:
                axis=np.asarray(refined['revolute']['axis']);center=np.asarray(refined['revolute']['point_on_axis']);disp=[];line=[]
                for f in fits:
                    a=np.asarray(f['revolute']['axis']);p=np.asarray(f['revolute']['point_on_axis']);disp.append(float(np.rad2deg(np.arccos(np.clip(abs(axis@a),0,1)))));line.append(float(np.linalg.norm(np.cross(a,center-p))))
                r['training_segment_axis_dispersion_deg']=max(disp);r['training_segment_axis_line_dispersion_m']=max(line)
            r['uncertainty_interpretation']='train segment dispersion; not calibrated probabilistic coverage'
        if 'dangerous_contact_event' not in r:r['dangerous_contact_event']=any(a in r.get('status','') for a in ['DANGEROUS','COLLISION','LOADED_CONTACT'])
        rows.append(r)
    table(out/'SYSTEM_MAIN_TABLE.csv',rows)
    valid=[r for r in rows if float(r.get('measured_probe_extent_m') or 0)>=.005]
    table(out/'STRUCTURE_MAIN_TABLE.csv',valid)
    # Apply all frozen structures to ONE common excluded action per episode,
    # prioritizing B0. This keeps commands/responses identical for predictions.
    common=[];plot_data={}
    for eid in episodes:
        target=next((m for m in ['B0','B1','B2'] if selected(trace.get((eid,m),[]),{'HELDOUT_MANIPULATION'})),None)
        if target is None:continue
        test=selected(trace[(eid,target)],{'HELDOUT_MANIPULATION'});E=np.asarray([z['T_ee'] for z in test]);O=np.asarray([z['T_object'] for z in test]);probe=selected(trace[(eid,target)],{'EXPLORATORY'})
        candidates={'B1_once':joint_models.get((eid,'B1'),{}).get('discovery'),'B2_refined_proposal':joint_models.get((eid,'B2'),{}).get('refined'),'B2_adopted':joint_models.get((eid,'B2'),{}).get('controller')}
        gt=read(out/'comparisons'/eid/target/'evaluation_only.json')['ground_truth'];candidates['dataset_GT_structure_diagnostic']={'revolute':{'axis':gt['axis_world'],'point_on_axis':gt['origin_world']}}
        pp={}
        if probe:
            X=np.asarray([z['T_ee'] for z in probe]);pp['B0_translation_phase_diagnostic']=linear_prediction(E,O,X)
            lp=local_rotation_phase_prediction(E,O,X)
            if lp is not None:pp['B0_rotation_phase_local']=lp
        for name,f in candidates.items():
            if f is not None and f.get('revolute'):pp[name]=object_prediction(E,O,f)
        for name,p in pp.items():common.append({'episode_id':eid,'asset_id':episodes[eid]['asset_id'],'common_response_method':target,'frozen_predictor':name,'object_position_rmse_m':p['object_position_rmse_m'],'object_rotation_rmse_rad':p['object_rotation_rmse_rad'],'object_endpoint_error_m':p['object_endpoint_error_m'],'fit_usage':'none','prediction':'conditional geometry, measured EE phase','GT_structure':'diagnostic oracle only' if name.startswith('dataset_GT') else 'not used'})
        plot_data[eid]=pp
    table(out/'COMMON_ACTION_PREDICTION.csv',common)
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    for eid,ps in plot_data.items():
        fig,ax=plt.subplots(figsize=(9,3.8))
        for name,p in ps.items():ax.plot(np.arange(len(p['object_position_errors_m']))/30.,np.asarray(p['object_position_errors_m'])*1000,label=name)
        ax.set(xlabel='excluded action time (s)',ylabel='object conditional prediction error (mm)',title=eid+' | one common physical response');ax.legend(fontsize=7);fig.tight_layout();fig.savefig(out/'plots'/('common_'+eid+'.png'));plt.close(fig)
    # Phase-separated object/EE trajectories for all observed episodes.
    for (eid,m),x in trace.items():
        x=[z for z in x if z['phase'] in ['EXPLORATORY','ESTIMATED_FOLLOW','STRUCTURE_VALIDATION','HELDOUT_MANIPULATION','HELDOUT_DWELL','OPERATIONAL_CONTINUATION','FINAL_HOLD']]
        if not x:continue
        E=np.asarray([z['T_ee'] for z in x]);O=np.asarray([z['T_object'] for z in x]);t=np.array([z['t'] for z in x]);rel=np.linalg.inv(O)@E;drift=np.linalg.norm(rel[:,:3,3]-rel[0,:3,3],axis=1)
        fig,ax=plt.subplots(2,1,figsize=(9,5),sharex=True)
        ax[0].plot(t-t[0],np.linalg.norm(E[:,:3,3]-E[0,:3,3],axis=1)*1000,label='EE measured travel');ax[0].plot(t-t[0],np.linalg.norm(O[:,:3,3]-O[0,:3,3],axis=1)*1000,label='object origin travel');ax[0].set_ylabel('travel (mm)');ax[0].legend()
        ax[1].plot(t-t[0],drift*1000,label='true relative drift (evaluation only)');ax[1].set(xlabel='interaction time (s)',ylabel='relative drift (mm)');ax[1].legend();fig.suptitle(eid+' '+m);fig.tight_layout();fig.savefig(out/'plots'/(eid+'_'+m+'_interaction.png'));plt.close(fig)
    # This round freezes evidence; no independent test is scheduled automatically.
    changed=[p for p,h in manifest['inherited_sha256'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
    changed_new=[p for p,h in manifest['new_source_sha256'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
    write(out/'INTEGRITY.json',{'baseline_or_method_files_changed':changed,'frozen_new_source_changed':changed_new,'native_trial_budget':manifest['native_trial_budget'],'no_new_assets':True,'no_physics_fit':True})
    write(out/'PAPER_EVIDENCE.json',{'research_question':'Does small physical interaction plus online structure updating improve subsequent object prediction and manipulation over no update or one estimate?',
      'operation_evidence':'SYSTEM_MAIN_TABLE.csv; 5 degrees is small interaction only','model_value_evidence':'COMMON_ACTION_PREDICTION.csv; frozen predictors, same excluded object/EE response','reconstruction_evidence':'OBSERVATION_UPPER_BOUND.csv and STRUCTURE_MAIN_TABLE.csv','paired_state_evidence':'paired_grasp_states.csv',
      'limitations':['diagnostic cohort is already exposed','conditional geometric prediction is not autonomous physics prediction','simulator-only contact safety supervisor remains','full drift is evaluation-only','dataset GT prior diagnostic is an oracle, not visual Real2Sim','no real-world system identification evidence'], 'next_step':'freeze findings before deciding whether independent object observations are needed; no new perception work or threshold tuning in this round'})
    return {'rows':len(rows),'valid_interactions':len(valid),'common_prediction_rows':len(common),'frozen_sources_changed':changed+changed_new}

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/paper_structure.json');a=p.parse_args();print(json.dumps(report(a.config)))
