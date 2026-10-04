"""Conditioned experiment A and full-task experiment B are separate outcomes."""
import copy,json,concurrent.futures,time,csv
from pathlib import Path
import numpy as np
from run_interactive_twin_conditional import read,write,sha,ROOT
from interactive_twin.sysid import NoiseScales,candidate_grid
from interactive_twin_refinement.physics import score
from interactive_twin_conditional.selection import choose
from interactive_twin.twin import write_twins


def logs(folder,phases=('P1','P2','P3')):
    result={}
    for p in phases:
        f=Path(folder)/'observable'/f'{p}.json'
        if f.exists():
            x=read(f)
            if x['provenance'].get('complete'):result[p]=x
    return result


def run_jobs(e,jobs):
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(e.c['gpus'])) as pool:
        return list(pool.map(e.native,jobs))


def replay(e,source,out,asset,plant,tape,conditional=True):
    j={**source,'output':str(out),'asset_root':str(asset),'plant':plant,'mode':'replay','role':'twin','replay_commands':str(tape),'frozen_proxy_sha256':sha(Path(asset)/'manifest.json')}
    if not conditional:
        j.pop('conditional_snapshot',None);j['reference_safety_memory']=str(Path(tape).parent/'structured_memory.json')
    return j


def export_selected(e,eid,data,selections):
    out=e.out/eid;scene=read(e.old/eid/'reference/initial_scene_private.json');W=np.eye(4);W[:3,:3]=scene['asset_rotation'];W[:3,3]=scene['asset_xyz']
    space=read(out/'structure_candidates.json');source=data['source_job']['asset_root'];support=read(e.old/eid/'structured_memory.json')['supporting_observations']
    for condition,s in selections.items():
        for label,m in s.get('methods',{}).items():
            if m is None:continue
            dest=out/'updated_twins'/condition/label
            est=next(r['estimate'] for r in space['candidates'] if r['structure_id']==m['structure_id'])
            if not (dest/'twin_versions.json').exists():write_twins(source,dest,est,W,initial_physics_prior={k:m[k] for k in ('tau_c','b')},ee_poses=support)
            write(dest/'selection_semantics.json',{'selected_by_train_validation':True,'test_read':False,'parameters':{k:m[k] for k in ('tau_c','b')},'structure_id':m['structure_id'],'identifiability':s['status'],'parameter_intervals':s['parameter_intervals'],'simulation_predictor_only':True,'baseline_promoted':False,'source':'conditional A; full-task B separately reported'})


def full(e,eid,data):
    out=e.out/eid;source=data['source_job'];conditions=e.c['hidden_reference_conditions']
    # First condition generates one actual command tape. All reference variants
    # and all candidate plants receive that exact tape after cold contact rebuild.
    original=e.native(source)
    if original['status']!='PHYSICS_PROTOCOL_COMPLETE':
        write(out/'stop.json',{'stage':'conditional_reference_initialization','status':original['status']});return summarize(e,eid)
    tape=Path(source['output'])/'conditional_tape.json'
    reference_jobs=[]
    for condition,phi in conditions.items():
        if condition=='original':continue
        reference_jobs.append(replay(e,source,out/'reference'/condition,source['asset_root'],phi,tape)|{'role':'reference'})
    run_jobs(e,reference_jobs)
    grid=candidate_grid(e.c['physics_grid']['tau_c'],e.c['physics_grid']['b'],J_eff_prior='frozen_4558a28_mass_and_inertia',budget=9)
    jobs=[];candidates=[]
    for structure in data['structures']:
        for phi in grid:
            cid=structure['id']+'_'+phi['candidate_id'];folder=out/'conditional_grid'/cid
            jobs.append(replay(e,source,folder,structure['asset'],{k:phi[k] for k in ('tau_c','b')},tape))
            candidates.append({'candidate_id':cid,'structure_id':structure['id'],'tau_c':phi['tau_c'],'b':phi['b'],'path':str(folder)})
    write(out/'grid_frozen.json',{'candidates':candidates,'count':len(jobs),'maximum':45,'same_snapshot_sha256':sha(source['conditional_snapshot']),'same_command_tape_sha256':sha(tape),'test_response_read':False})
    run_jobs(e,jobs)
    calibration=read(ROOT/'results/interactive_twin_benchmark_20261003_v2/robot_calibration/robot_calibration.json');noise=NoiseScales(**calibration['noise_scales'])
    selections={}
    for condition in conditions:
        ref=logs(out/'reference'/condition)
        if set(ref)!={'P1','P2','P3'}:
            selections[condition]={'status':'REFERENCE_TRAIN_ACTION_INCOMPLETE','test_read':False};continue
        cs=[{**c,'logs':logs(c['path']),'status':read(Path(c['path'])/'report.json')['status']} for c in candidates]
        selections[condition]=choose(ref,cs,noise,{k:e.c[k] for k in ('selection','wrong_prior','physics_grid')})
    selection_path=out/'selection_frozen.json'
    if selection_path.exists():
        if read(selection_path)['conditions']!=selections:raise RuntimeError('FROZEN_SELECTION_CHANGED')
    else:write(selection_path,{'saved_unix_s':time.time(),'conditions':selections,'test_read':False,'input_phases':['P1','P2','P3']})
    export_selected(e,eid,data,selections)
    # All structural/physical choices above are saved before the first P4 read.
    testrows=[]
    for condition,s in selections.items():
        refs=logs(out/'reference'/condition,('P4',));predictions={}
        for label,m in s.get('methods',{}).items():
            row={'condition':condition,'method':label,'selected':m,'identifiability':s['status'],'parameter_intervals':s['parameter_intervals'],'status':'NO_TRAIN_VALIDATED_MODEL'}
            if m:
                p=out/'conditional_grid'/m['candidate_id'];pred=logs(p,('P4',));row['native_status']=read(p/'report.json')['status'];row['prediction_folder']=str(p)
                if refs and pred:
                    row.update(status='TEST_COMPLETE',test=score(refs['P4'],pred['P4'],noise));predictions[label]=pred['P4']
                else:row['status']='TEST_CENSORED'
            testrows.append(row)
        if refs and predictions:
            from interactive_twin.reporting import plot_heldout_predictions
            plot_heldout_predictions(refs['P4'],predictions,out/f'heldout_{condition}.png')
    write(out/'conditional_heldout.json',{'selection_sha256':sha(selection_path),'evaluation_after_selection':True,'rows':testrows})
    # B is run regardless of which A predictors win or fail. It begins from the
    # original home/object assembly and retains all warmup/approach failures.
    full_refs=[]
    for condition,phi in conditions.items():
        j={**source,'output':str(out/'full_task'/condition/'reference'),'plant':phi,'continue_manipulation':True,'wall_clock_budget_s':2200}
        j.pop('conditional_snapshot',None);j.pop('replay_commands',None);full_refs.append(j)
    run_jobs(e,full_refs)
    full_jobs=[]
    for refjob in full_refs:
        condition=Path(refjob['output']).parent.name
        if not (Path(refjob['output'])/'command_tape.json').exists():continue
        for label,m in selections.get(condition,{}).get('methods',{}).items():
            if m is None:continue
            st=next(s for s in data['structures'] if s['id']==m['structure_id'])
            j=replay(e,refjob,out/'full_task'/condition/label,st['asset'],{k:m[k] for k in ('tau_c','b')},Path(refjob['output'])/'command_tape.json',False)
            full_jobs.append(j)
    run_jobs(e,full_jobs)
    # Frozen predictor verification only: P4 is now also replayed from the same
    # initial grasp snapshot, excluding state errors accumulated during P1-P3.
    # It never changes the train/validation-selected models or parameters.
    from verify_conditional_heldout_start import run as verify_common_start
    common=verify_common_start(e,eid)
    result=summarize(e,eid)
    # Only frozen unseen metrics permit transfer; no method retuning here.
    good=[]
    for condition in conditions:
        rows={r['method']:r for r in (common or {}).get('rows',[]) if r['condition']==condition};a=rows.get('fixed_refined_wrong_prior',{}).get('test');b=rows.get('uncertain_structure_calibrated',{}).get('test')
        good.append(bool(a and b and b['metrics']['ee_position_rmse_m']<(1-e.c['selection']['improvement_fraction'])*a['metrics']['ee_position_rmse_m']))
    passed=all(good) and len(good)==len(conditions)
    write(out/'transfer_gate_common_start.json',{'passed':passed,'condition_prediction_improvements':good,'test_controls_expansion_only':True,'metric':'same-start P4 absolute EE RMSE','model_selection_changed':False})
    if passed and eid=='dev_7320_00':
        from interactive_twin_conditional.transfer import run_transfer
        run_transfer(e)
    return result


def summarize(e,eid):
    out=e.out/eid;rows=[];result_path=out/'conditional_heldout.json'
    tests=read(result_path)['rows'] if result_path.exists() else []
    common_path=out/'common_start_heldout/results.json'
    common={(r['condition'],r['method']):r for r in read(common_path)['rows']} if common_path.exists() else {}
    for row in tests:
        m=row['selected'] or {};test=row.get('test',{});metrics=test.get('metrics',{});pd=test.get('post_drive_drift_prediction') or {};rd=test.get('post_drive_drift_reference') or {}
        p=out/'full_task'/row['condition']/row['method']/'report.json';task=read(p) if p.exists() else {};ev=task.get('evaluation',{})
        # REPLAY_COMPLETE means tape completion, not necessarily a retained >=5° opening.
        opened=ev.get('actual_door_displacement_deg');grasp=task.get('bilateral_hold_established',False)
        if opened is not None and (p.parent/'job_private.json').exists():
            manifest=read(Path(read(p.parent/'job_private.json')['asset_root'])/'manifest.json')
            opened*=manifest.get('interactive_twin',{}).get('coordinate_sign_vs_prepared_prior',1)
        full_success=task.get('status')=='REPLAY_COMPLETE' and grasp and opened is not None and opened>=5 and ev.get('final_true_relative_translation_slip_m',1)<=read(ROOT/'config/semantic_interaction.json')['max_slip_m']
        rows.append({'condition':row['condition'],'method':row['method'],'conditional_status':row['status'],'structure':m.get('structure_id'),'heldout_RMSE_mm':1000*metrics['ee_position_rmse_m'] if metrics else None,'velocity_RMSE_mm_s':1000*metrics['ee_velocity_rmse_m_s'] if metrics else None,'start_delay_error_s':metrics.get('start_time_error_s'),'final_displacement_error_mm':1000*metrics['final_displacement_error_m'] if metrics else None,'held_stop_drift_mm':1000*pd['net_drift_m'] if pd else None,'held_stop_drift_error_mm':1000*abs(pd['net_drift_m']-rd['net_drift_m']) if pd and rd else None,'tau_c':m.get('tau_c'),'b':m.get('b'),'parameter_intervals':row['parameter_intervals'],'identifiability':row['identifiability'],'full_task_status':task.get('status','NOT_RUN'),'full_task_grasp':grasp,'full_task_angle_deg':opened,'full_task_success':full_success,'full_task_min_margin_rad':task.get('minimum_joint_margin_rad'),'full_task_final_slip_m':ev.get('final_true_relative_translation_slip_m'),'conditional_video':str(Path(row['prediction_folder'])/'contact_baseline.mp4') if row.get('prediction_folder') else None,'full_task_video':str(p.parent/'contact_baseline.mp4') if p.exists() else None})
        c=common.get((row['condition'],row['method']),{});ct=c.get('test',{});cm=ct.get('metrics',{});cp=ct.get('post_drive_drift_prediction') or {};cr=ct.get('post_drive_drift_reference') or {}
        rows[-1].update(sequence_heldout_scope='P4 after independently evolved P1-P3',common_start_status=c.get('status','NOT_RUN'),common_start_RMSE_mm=1000*cm['ee_position_rmse_m'] if cm else None,common_start_relative_RMSE_mm=c.get('action_relative_RMSE_mm'),common_start_velocity_RMSE_mm_s=1000*cm['ee_velocity_rmse_m_s'] if cm else None,common_start_delay_error_s=cm.get('start_time_error_s'),common_start_final_displacement_error_mm=1000*cm['final_displacement_error_m'] if cm else None,common_start_held_drift_mm=1000*cp['net_drift_m'] if cp else None,common_start_held_drift_error_mm=1000*abs(cp['net_drift_m']-cr['net_drift_m']) if cp and cr else None)
    write(out/'summary.json',{'mode':'SIM_TO_SIM_BLIND_SYSID','conditional_is_not_full_task':True,'rows':rows,'stop':read(out/'stop.json') if (out/'stop.json').exists() else None,'hardware_friction_recovered':False})
    if rows:
        with (out/'comparison.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    return rows
