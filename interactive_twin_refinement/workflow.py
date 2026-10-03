"""Orchestration only. Privileged setup and posthoc evaluation are separate calls.

The physics estimator receives only observable logs and candidate parameters;
reference values never enter fit(). P4 is scored only after frozen selection.
"""
import concurrent.futures,copy,json,time,hashlib,csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from run_interactive_twin_refinement import run_native
from run_interactive_twin_benchmark import read,write,sha
from interactive_twin_refinement.selection import select
from interactive_twin_refinement.physics import fit,score
from interactive_twin.sysid import NoiseScales,candidate_grid

def prewarm(asset):
    from interactive_twin.visual_import import compatible_urdf
    m=read(Path(asset)/'manifest.json');compatible_urdf(Path(asset)/'urdf'/f"{m['asset_id']}.urdf",Path(asset)/'visual_compatibility')

def source_context(e,eid):
    job,folder=e.source(eid);scene=read(folder/'initial_scene_private.json')
    W=np.eye(4);W[:3,:3]=scene['asset_rotation'];W[:3,3]=scene['asset_xyz']
    asset=Path(job['asset_root'])
    if job.get('initial_articulation_rad',0):
        from interactive_twin.initial_state import bake_initial_articulation
        asset=Path(bake_initial_articulation(asset,e.out/eid/'initial_zero',job['initial_articulation_rad'])['asset_root'])
    return job,folder,scene,W,asset

def compile_models(e,eid):
    from interactive_twin.twin import write_twins
    job,folder,scene,W,asset=source_context(e,eid);s=read(e.out/eid/'structure_selection.json')
    models={};support=read(e.out/eid/'structured_memory.json')['supporting_observations']
    for name,f in [('old',s['old']),('refined',s['refined'])]:
        dest=e.out/eid/'twins'/name
        versions=read(dest/'twin_versions.json') if (dest/'twin_versions.json').exists() else write_twins(asset,dest,f,W,initial_physics_prior=e.c['physics']['wrong_prior'],ee_poses=support)
        models[name]=Path(versions['versions']['T1']['asset_root']);prewarm(models[name])
    models['oracle']=asset;prewarm(asset)
    return models

def evaluate_structure(e,eid):
    # Called only after structure_selection.json is immutable on disk.
    from articulated_demo.kinematics import URDFChain
    job,folder,scene,W,asset=source_context(e,eid);m=read(asset/'manifest.json');chain=URDFChain(asset/'urdf'/f"{m['asset_id']}.urdf")
    joint=chain.joints[m['moving_link']];H=W@chain.root_to_link(joint.parent,{})@joint.origin
    ag=H[:3,:3]@joint.axis;cg=H[:3,3];s=read(e.out/eid/'structure_selection.json')
    support=np.asarray(read(e.out/eid/'structured_memory.json')['supporting_observations']);anchor=support[:,:3,3].mean(0);rows={}
    for name,f in [('old',s['old']),('refined',s['refined'])]:
        a=np.array(f['revolute']['axis']);c=np.array(f['revolute']['point_on_axis'])
        c=c+a*(ag@(anchor-c))/(ag@a)
        rows[name]={'axis_error_deg':float(np.rad2deg(np.arccos(np.clip(abs(a@ag),0,1)))),
             'axis_line_error_m':float(np.linalg.norm(np.cross(ag,c-cg))),'line_metric':'distance in GT-axis-normal plane through observation centroid; axial gauge removed'}
    write(e.out/eid/'structure_evaluation_only.json',{'evaluation_after_saved_selection':True,'rows':rows,'used_for_selection':False})
    return rows

def prepare_reference(e,eid,gpu):
    job,folder,scene,W,asset=source_context(e,eid);out=e.out/eid
    c=read(out/'structure_selection.json')
    # Discovery/control stays with the previously safe accepted estimate.
    # Structure calibration changes the twin only. This separates servo safety
    # from which calibration model wins independent validation.
    j={**job,'mode':'physics_reference','role':'reference','output':str(out/'reference'),
       'continue_manipulation':False,'gpu':gpu,'deadline_shanghai':e.c['deadline_shanghai'],'wall_clock_budget_s':1800,
       'initial_estimate':str(folder/'estimated_articulation.json'),'initial_estimate_memory':str(folder/'structured_memory.json'),
       'fixed_fixture':scene['fixture'],'physics_protocol':e.c['actions']}
    j.pop('import_spec',None)
    if j.get('mobile_route'):j['mobile_route']['deadline_unix_s']=__import__('datetime').datetime.fromisoformat(j['deadline_shanghai']).timestamp()
    prewarm(j['asset_root']);r=run_native(j)
    write(out/'reference_status.json',{'status':r['status'],'protocol':e.c['actions'],'test_use':'sealed until structure and physics selection saved'})
    return j,r

def replay_job(e,eid,reference,label,asset,params,gpu,tape=None,role='twin'):
    out=e.out/eid/label
    j={**reference,'mode':'replay','role':role,'output':str(out),'asset_root':str(asset),'plant':params,'gpu':gpu,
       'initial_articulation_rad':0.,'replay_commands':str(tape or Path(reference['output'])/'command_tape.json'),
       'reference_safety_memory':str(Path(reference['output'])/'structured_memory.json'),
       'frozen_proxy_sha256':sha(Path(asset)/'manifest.json')}
    # The reference initial nonzero articulation is baked once into all replay
    # assets, including oracle. No per-step state is replayed.
    j.pop('initial_estimate',None);j.pop('initial_estimate_memory',None)
    return j

def parallel_jobs(jobs,gpus):
    # One process per GPU; bounded chunks, no simultaneous jobs sharing a GPU.
    results=[]
    for start in range(0,len(jobs),len(gpus)):
        group=jobs[start:start+len(gpus)]
        for j,g in zip(group,gpus):j['gpu']=g
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(group)) as pool:
            results.extend(pool.map(run_native,group))
    return results

def complete_logs(folder,phases=('P1','P2','P3')):
    logs={}
    for p in phases:
        f=Path(folder)/'observable'/f'{p}.json'
        if not f.exists():return {}
        l=read(f)
        if not l['provenance'].get('complete'):return {}
        logs[p]=l
    return logs

def physics(e,eid,gpus,shared_future=None,publish_future=None):
    out=e.out/eid;selection=read(out/'structure_selection.json')
    if 'refined' not in selection:return {'status':selection['status']}
    models=compile_models(e,eid);evaluate_structure(e,eid)
    reference,r=prepare_reference(e,eid,gpus[0]);train=complete_logs(reference['output'])
    if not train:
        result={'status':'REFERENCE_PHYSICS_PROBE_FAILED','reason':r['status']};write(out/'physics_summary.json',result);return result
    # Freeze explicit action-level contract independently of legacy log schema
    # (which labels P3 "train"). All fitting functions here exclude P3 training.
    write(out/'action_split.json',{'train':['P1','P2'],'validation':['P3'],'test':['P4'],
           'legacy_P3_label_overridden_by_this_frozen_contract':True,'adjacent_frame_random_split':False})
    calibration=read(e.old/'robot_calibration/robot_calibration.json');noise=NoiseScales(**calibration['noise_scales'])
    # Reference parameters are privileged setup only. These values are not
    # passed to physics.fit; their use below is confined to oracle diagnostics.
    original_params=reference['plant']
    conditions={'original':(Path(reference['output']),original_params)}
    diagnostic_jobs=[]
    if eid.startswith('dev'):
        phi=e.c['physics']['additional_hidden_condition']
        j=replay_job(e,eid,reference,'hidden_nonzero_reference',models['oracle'],phi,gpus[0],role='reference')
        diagnostic_jobs.append(j)
        conditions['additional_nonzero']=(Path(j['output']),phi)
    for condition,(_,phi) in conditions.items():
        for name in ('oracle','old','refined'):
            diagnostic_jobs.append(replay_job(e,eid,reference,f'diagnostic/{condition}/{name}',models[name],phi,gpus[0],role='diagnostic_oracle'))
    parallel_jobs(diagnostic_jobs,gpus)
    from interactive_twin_refinement.assembly import compare
    # Post-warmup transforms are physical responses, not initial assembly.
    # Use short setup-only runs to compare authored native geometry at the same
    # prepared q=0 coordinate before the existing passive warmup.
    audit_jobs=[]
    for name,asset in models.items():
        aj=replay_job(e,eid,reference,'assembly_audit/'+name,asset,original_params,gpus[0])
        aj.update(mode='assembly_audit');aj.pop('replay_commands',None);aj.pop('reference_safety_memory',None)
        audit_jobs.append(aj)
    parallel_jobs(audit_jobs,gpus)
    base_audit=out/'assembly_audit/oracle/native_authored_assembly_private.json'
    audits=[]
    for aj in audit_jobs:
        target=Path(aj['output'])/'native_authored_assembly_private.json'
        audits.append({'run':aj['output'],**compare(read(base_audit),read(target))} if base_audit.exists() and target.exists() else {'run':aj['output'],'passed':False,'reason':'SETUP_AUDIT_MISSING'})
    write(out/'native_assembly_comparison.json',{'sampling':'authored initial state before passive warmup','comparisons':audits,'failed':any(not a['passed'] for a in audits)})
    if any(not a['passed'] for a in audits):
        result={'status':'INITIAL_ASSEMBLY_INCONSISTENT','failure_stage':'coordinate_writeback','audits':audits}
        write(out/'physics_summary.json',result);return result
    # Same commanded inputs for both reference resistances. Detect native
    # response separation before fitting; authored attributes alone are not proof.
    sensitivity={}
    if len(conditions)>1:
        a=complete_logs(conditions['original'][0]);b=complete_logs(conditions['additional_nonzero'][0])
        if a and b:
            diffs={p:score(a[p],b[p],noise) for p in ('P1','P2')}
            rms=float(np.sqrt(np.mean([v['normalized_loss'] for v in diffs.values()])))
            repeated=complete_logs(out/'diagnostic/original/oracle')
            repeat_rms=float(np.sqrt(np.mean([score(a[p],repeated[p],noise)['normalized_loss'] for p in ('P1','P2')]))) if repeated else None
            threshold=max(1.,3*repeat_rms) if repeat_rms is not None else float('inf')
            sensitivity={'status':'OBSERVABLE_RESPONSE' if rms>threshold else 'UNIDENTIFIABLE','rms_noise_units':rms,'repeat_rms_noise_units':repeat_rms,'actions':diffs,
                 'same_actual_command':True,'noise_source':'frozen robot calibration floors; previous deterministic repeats, not hardware sensing'}
        else:sensitivity={'status':'REFERENCE_RESISTANCE_CENSORED'}
        write(out/'new_resistance_sensitivity.json',sensitivity)
        if sensitivity.get('status')!='OBSERVABLE_RESPONSE':
            result={'status':'UNIDENTIFIABLE_UNDER_CURRENT_PROBE','sensitivity':sensitivity,'physics_fit_started':False};write(out/'physics_summary.json',result);return result
    # Do not alter the 9-point frozen parameter grid. Reuse each independently
    # evolving twin response for all same-input hidden reference conditions.
    structural_execution_failures=[]
    for condition in conditions:
        rr=read(out/f'diagnostic/{condition}/refined/report.json')
        if rr['status']!='REPLAY_COMPLETE':structural_execution_failures.append({'condition':condition,'status':rr['status'],'failure_phase':rr.get('failure_phase'),'bilateral_hold_established':rr.get('bilateral_hold_established')})
    write(out/'structure_physics_stop_gate.json',{'scope':'experiment oracle diagnostic only; never a controller/estimator input',
       'stop_calibration':bool(structural_execution_failures),'failures':structural_execution_failures,
       'reason':'Do not fit resistance to compensate a structure-induced physical execution failure'})
    grid=candidate_grid(e.c['physics']['grid']['tau_c'],e.c['physics']['grid']['b'],J_eff_prior='fixed_original_articulated_mass_inertia',budget=9)
    if structural_execution_failures:grid=[v for v in grid if v['candidate_id']=='phi_008']  # frozen wrong prior diagnostic only
    shared=None
    if shared_future is not None:
        shared=shared_future.result()
        ids={'phi_008',shared.get('diagnostic_best',{}).get('candidate_id')}
        grid=[v for v in grid if v['candidate_id'] in ids]
    jobs=[replay_job(e,eid,reference,'grid/'+v['candidate_id'],models['refined'],{k:v[k] for k in ('tau_c','b')},gpus[0]) for v in grid]
    parallel_jobs(jobs,gpus)
    candidates=[{**v,'probes':complete_logs(j['output']),'status':read(Path(j['output'])/'report.json')['status']} for v,j in zip(grid,jobs)]
    selections={}
    for condition,(folder,_) in conditions.items():
        logs=complete_logs(folder)
        if not logs:continue
        if structural_execution_failures:
            selections[condition]={'status':'UNIDENTIFIABLE_STRUCTURAL_EXECUTION_FAILURE','accepted_parameters':None,
                 'parameter_intervals':{k:[min(v),max(v)] for k,v in e.c['physics']['grid'].items()},
                 'interval_semantics':'full frozen search domain remains unresolved; no friction optimizer run',
                 'physics_candidates_optimized':0,'wrong_prior_diagnostic_only':True,'test_read':False,
                 'failure_stage':'structure_to_physical_twin','failures':structural_execution_failures}
            continue
        selections[condition]=fit(logs,candidates,noise,e.c['physics']) if shared is None else {**copy.deepcopy(shared),
            'transfer_parameter_source_episode':e.c['episodes'][1],'fitted_on_this_configuration':False,
            'transfer_condition':'same asset passive parameters; new q/base/start state, no parameter retuning'}
    write(out/'physics_selection_frozen.json',selections)  # BEFORE any P4 read
    if publish_future is not None and not publish_future.done():publish_future.set_result(selections.get('original',{'status':'SOURCE_CALIBRATION_FAILED'}))
    from interactive_twin.twin import write_twins
    job0,source0,scene0,W0,asset0=source_context(e,eid)
    chosen=read(out/'estimated_articulation.json');support=read(out/'structured_memory.json')['supporting_observations']
    for condition,estimate in selections.items():
        destination=out/'updated_twins'/condition
        if not (destination/'twin_versions.json').exists():
            write_twins(asset0,destination,chosen,W0,initial_physics_prior=e.c['physics']['wrong_prior'],physics_estimate=estimate,ee_poses=support)
    # Freeze test methods and hashes before evaluation. The test cannot choose
    # an axis, a candidate, a threshold, or a model version.
    test_plan={condition:{'fit_status':s['status'],'diagnostic_candidate':s.get('diagnostic_best'),
                'structure_selection_sha256':sha(out/'structure_selection.json')} for condition,s in selections.items()}
    write(out/'test_plan_frozen.json',test_plan)
    rows=[];predictions={}
    for condition,(folder,phi) in conditions.items():
        refs=complete_logs(folder,('P4',));ss=selections.get(condition,{})
        best=ss.get('diagnostic_best',{})
        specs=[('oracle',out/f'diagnostic/{condition}/oracle',phi),('old_structure',out/f'diagnostic/{condition}/old',phi),
               ('refined_structure',out/f'diagnostic/{condition}/refined',phi),
               ('refined_wrong_prior',out/'grid/phi_008',e.c['physics']['wrong_prior'])]
        if best:specs.append(('refined_fitted_physics',out/'grid'/best['candidate_id'],{k:best[k] for k in ('tau_c','b')}))
        else:specs.append(('refined_fitted_physics',out/'not_executed_calibration',{}))
        for label,folder_pred,params in specs:
            ps=complete_logs(folder_pred,('P4',));report=read(folder_pred/'report.json') if (folder_pred/'report.json').exists() else {}
            row={'episode':eid,'condition':condition,'method':label,'status':report.get('status','NOT_RUN_STRUCTURAL_BLOCKER' if structural_execution_failures else 'NOT_RUN'),'parameters':params,
                 'fit_status':ss.get('status') if label=='refined_fitted_physics' else ('FROZEN_WRONG_PRIOR' if label=='refined_wrong_prior' else 'ORACLE_PHYSICS_NOT_FIT'),'parameter_intervals':ss.get('parameter_intervals') if label=='refined_fitted_physics' else None,
                 'parameters_promoted_to_twin':bool(ss.get('accepted_parameters')) if label=='refined_fitted_physics' else False,
                 'minimum_joint_margin_rad':report.get('minimum_joint_margin_rad'),'peak_load_n':report.get('peak_finger_handle_force_n')}
            if refs and ps:
                row['test']=score(refs['P4'],ps['P4'],noise);predictions[label]=ps['P4']
            rows.append(row)
        if refs and predictions:
            from interactive_twin.reporting import plot_heldout_predictions
            plot_heldout_predictions(refs['P4'],predictions,out/f'heldout_{condition}.png')
    # The estimated URDF is frozen before this posthoc oracle comparison.
    result={'status':'EVALUATED','rows':rows,'sensitivity':sensitivity,'mode':'SIM_TO_SIM_BLIND_SYSID',
            'test_used_for_selection':False,'physics_reference_values_hidden_from_estimator':True,
            'interpretation':'diagnostic grid best is not a calibrated physical friction measurement'}
    write(out/'physics_summary.json',result)
    return result

def full(e):
    results=[];dev=e.c['episodes'][0]
    e.offline(dev);e.supplement(dev,e.gpus[0]);selection=select(e,dev)
    print(dev,'STRUCTURE',selection['status'],flush=True)
    if not selection.get('accepted'):
        write(e.out/'scientific_stop.json',{'stage':'structure_validation','reason':selection['status'],'new_physics_not_tuned_to_hide_structure':True})
    else:
        result=physics(e,dev,e.gpus);results.append({'episode':dev,**result});write(e.out/'batch_status.json',results)
        rows=result.get('rows',[])
        improvements=[]
        for condition in sorted({r['condition'] for r in rows}):
            by={r['method']:r for r in rows if r['condition']==condition}
            def error(name):return by.get(name,{}).get('test',{}).get('metrics',{}).get('ee_position_rmse_m')
            old,new,fitted,prior=(error(n) for n in ('old_structure','refined_structure','refined_fitted_physics','refined_wrong_prior'))
            improvements.append({'condition':condition,'structure_improved':bool(old is not None and new is not None and new<old),
                 'fitted_beats_wrong_prior':bool(fitted is not None and prior is not None and fitted<prior)})
        gate=bool(improvements and all(x['structure_improved'] and x['fitted_beats_wrong_prior'] for x in improvements))
        write(e.out/'dev_gate.json',{'passed':gate,'comparisons':improvements,'heldout_controls_expansion_only_not_model_selection':True})
        if gate:
            transfer=e.c['episodes'][1:]
            # All thresholds, protocol and budgets are already frozen. One
            # calibration per physical asset; remaining deployments validate it.
            with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
                list(pool.map(lambda item:e.supplement(*item),zip(transfer,e.gpus)))
            valid=[]
            for eid in transfer:
                s=select(e,eid);print(eid,'STRUCTURE',s['status'],flush=True)
                if s.get('accepted'):valid.append(eid)
                else:results.append({'episode':eid,'status':s['status'],'failure_stage':'structure_validation'})
            future=concurrent.futures.Future()
            if transfer[0] not in valid:future.set_result({'status':'SOURCE_CALIBRATION_FAILED','accepted_parameters':None})
            def execute(eid):
                try:return {'episode':eid,**physics(e,eid,e.gpus,
                    shared_future=None if eid==transfer[0] else future,
                    publish_future=future if eid==transfer[0] else None)}
                except Exception as error:
                    import traceback
                    failure={'episode':eid,'status':str(error),'failure_stage':'implementation_or_safety','traceback':traceback.format_exc()}
                    write(e.out/eid/'workflow_failure.json',failure);return failure
                finally:
                    if eid==transfer[0] and not future.done():future.set_result({'status':'SOURCE_CALIBRATION_FAILED','accepted_parameters':None})
            with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
                for r in pool.map(execute,valid):results.append(r);write(e.out/'batch_status.json',results)
        else:
            write(e.out/'scientific_stop.json',{'stage':'DEV_heldout','reason':'No validated DEV improvement; transfer calibration stopped','comparisons':improvements})
    write(e.out/'batch_status.json',results)
    from interactive_twin_refinement.summary import summarize
    summarize(e)
