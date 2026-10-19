"""One preregistered existing successful deployment; no asset-specific controls."""
import copy,json,shutil
from pathlib import Path
import numpy as np
from run_interactive_twin_conditional import read,write,sha,ROOT
from interactive_twin_conditional.workflow import full
from interactive_twin_refinement.fitting import fit_se3,predict_action
from interactive_twin_refinement.data import action_poses


def run_transfer(e):
    if (e.out/'defer_transfer_for_common_start_heldout.json').exists():
        write(e.out/'transfer_deferred.json',{'reason':'Await frozen-model common-initial-state P4 check; no parameter reselection','status':'NOT_STARTED'});return
    eid=e.c['transfer_episode'];out=e.out/eid
    summary=read(ROOT/'results/interactive_twin_recovery_20261003_v2/mobile'/eid/'summary.json')
    attempt=(summary['mobile_attempts'] or summary['fixed_attempts'])[-1]
    source=Path(attempt['path']);job=read(source/'job_private.json')
    if read(source/'report.json')['status']!='SUCCESS':
        write(out/'stop.json',{'stage':'transfer_registration','reason':'REGISTERED_DEPLOYMENT_NOT_PREVIOUSLY_SUCCESSFUL'});return
    # Same snapshot extraction, independent of scene class or dimensions.
    rows=read(source/'observations.json');commands=read(source/'command_tape.json')
    i=next(i for i,r in enumerate(rows) if r['phase']=='COMPLIANT_SETTLE')-1;r=rows[i];cmd=commands[i]
    snap=out/'stable_grasp_snapshot.json'
    write(snap,{'q_rad':r['q'],'qdot_rad_s':((np.asarray(r['q'])-rows[i-1]['q'])*240).tolist(),'T_ee':r['T_tcp'],'arm_position_command':cmd['arm_position'],'arm_velocity_command':cmd['arm_velocity'],'finger_position_command':cmd['finger_position'],'finger_effort_n':cmd['finger_effort'],'source_time_s':r['t'],'source_observations_sha256':sha(source/'observations.json'),'provenance':{'joint_truth_used':False,'moving_trajectory_used':False,'part_pose_source':'initial prepared visual assembly; stable pre-probe grasp, no axis/angle observation','initial_part_pose_estimated_unchanged':True,'source_phase':r['phase'],'contact_forces_not_restored':True}})
    scene=read(source/'initial_scene_private.json')
    refjob={**job,'output':str(out/'reference/original'),'conditional_snapshot':str(snap),'initial_estimate':str(source/'estimated_articulation.json'),'initial_estimate_memory':str(source/'structured_memory.json'),'fixed_fixture':scene['fixture'],'mode':'physics_reference','role':'reference','physics_protocol':e.c['physics_protocol'],'continue_manipulation':False,'deadline_shanghai':e.c['deadline_shanghai'],'wall_clock_budget_s':e.c['native_wall_clock_budget_s']}
    write(out/'registration.json',{'existing_successful_deployment':str(source),'new_base_search':False,'same_method_config_sha256':sha(e.out/'frozen_config.json'),'structure_data':'existing completed EE opening plus new conditional closing P2; no GT fitting','only_original_asset_resistance_condition':True})
    result=e.native(refjob)
    if result['status']!='PHYSICS_PROTOCOL_COMPLETE':write(out/'stop.json',{'stage':'conditional_reference','status':result['status']});return
    # Build the same empirical uncertainty construction from available measured
    # opening / closing actions. Short reverse excitation remains a limitation,
    # not a reason to change thresholds or add interaction attempts.
    current=read(out/'reference/original/measured_actions.json')
    reverse=next(a for a in current['actions'] if a['phase']=='P2');reverse=copy.deepcopy(reverse);reverse['phase']='REFINEMENT_REVERSE'
    opening_rows=[r for i,r in enumerate(rows) if r['phase']=='ESTIMATED_FOLLOW' and i%8==0]
    opening={'phase':'ESTIMATED_FOLLOW','time_s':[r['t'] for r in opening_rows],'ee_T':[r['T_tcp'] for r in opening_rows],'observable_anomaly':[r['relative_translation_slip_m']>.003 or min(r['forces_n'].values())<=0 for r in opening_rows]}
    train=action_poses(opening)[0];val=action_poses(reverse)[0]
    fitting={'position_noise_m':2e-5,'rotation_noise_rad':2e-5,'maximum_samples':180,'max_nfev':160}
    try:
        from interaction_identification.fitting import fit_articulation
        reverse_seed=fit_articulation(val)
        if reverse_seed['joint_type']=='UNOBSERVABLE':
            raise ValueError('REVERSE_STRUCTURE_UNOBSERVABLE: '+reverse_seed['reason'])
        fit=fit_se3(train,**fitting);old=read(source/'estimated_articulation.json')
        selection={'old':old,'refined':fit,'segment_diagnostics':[{'fit':fit_se3(x,**fitting)} for x in (train[:len(train)//2],train[len(train)//2:],val)],'validation':{name:predict_action(val,f) for name,f in [('old',old),('refined',fit)]}}
    except (ValueError,KeyError) as error:
        write(out/'stop.json',{'stage':'transfer_structure_observability','reason':'TRANSFER_STRUCTURE_UNOBSERVABLE','detail':str(error),'measured_opening_travel_m':float(np.max(np.linalg.norm(train[:,:3,3]-train[0,:3,3],axis=1))),'measured_reverse_travel_m':float(np.max(np.linalg.norm(val[:,:3,3]-val[0,:3,3],axis=1))),'extra_trajectory_requested':False,'threshold_changed':False});return
    data_root=e.out/'transfer_inputs';dest=data_root/eid
    for n in ('job_private.json','initial_scene_private.json','observations.json','command_tape.json'):
        target=dest/'reference'/n;target.parent.mkdir(parents=True,exist_ok=True)
        if n=='job_private.json':write(target,refjob)
        elif not target.exists():target.symlink_to(source/n)
    write(dest/'structure_selection.json',selection)
    write(dest/'refinement_once/measured_actions.json',{'source':'measured EE only','actions':[opening,reverse]})
    write(dest/'structured_memory.json',{'supporting_observations':train.tolist(),'GT_inputs':False})
    prior_old=e.old;prior_conditions=e.c['hidden_reference_conditions']
    try:
        e.old=data_root;e.c={**e.c,'hidden_reference_conditions':{'original':job['plant']}}
        prepared=e.prepare(eid);prepared['source_job']=refjob
        full(e,eid,prepared)
    finally:e.old=prior_old;e.c={**e.c,'hidden_reference_conditions':prior_conditions}
