"""Conditional coverage and accepted/provisional/refined outcomes kept separate."""
import csv,json
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np
from cross_object_structure.reporting import report as prior_report
from cross_object_structure.benchmark import read,write
from interactive_twin_refinement.fitting import predict_action


def table(path,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('w') as f:
        w=csv.DictWriter(f,fieldnames=keys,lineterminator='\n');w.writeheader();w.writerows(rows)


def report(b):
    prior_report(b)
    if not (b.out/'frozen_test_manifest.json').exists():return
    m=read(b.out/'frozen_test_manifest.json');rows=[]
    with (b.out/'per_episode.csv').open() as f:old={r['episode_id']:r for r in csv.DictReader(f)}
    for e in m['episodes']:
        sfile=b.out/'episodes'/e['episode_id']/'episode_summary.json'
        s=read(sfile) if sfile.exists() else {};folder=Path(s['selected_job']['output']) if s.get('selected_job') else None
        def get(name):return read(folder/name) if folder and (folder/name).exists() else {}
        r=old[e['episode_id']];d=get('discovery_decision.json');a=get('active_refinement.json');ss=get('structure_selection.json');nr=get('report.json')
        if not d and get('provisional_evidence.json'):d={'status':'UNOBSERVABLE','evidence':get('provisional_evidence.json')}
        entered=bool(folder and (folder/'refinement_entered.json').exists())
        observed=[]
        if entered and (folder/'observations.json').exists():
            observed=read(folder/'observations.json')
        active_T=np.asarray([x['T_tcp'] for i,x in enumerate(observed) if i%8==0 and x['phase']=='ESTIMATED_FOLLOW'])
        measured_span=None;measured_path=None
        if len(active_T)>1:
            from scipy.spatial.transform import Rotation
            measured_span=float(np.rad2deg(np.max(Rotation.from_matrix(active_T[:,:3,:3]@active_T[0,:3,:3].T).magnitude())))
            measured_path=float(np.linalg.norm(np.diff(active_T[:,:3,3],axis=0),axis=1).sum())
        r.update(discovery_status=d.get('status','NOT_REACHED'),discovery_admitted=d.get('status') in ('ACCEPTED','PROVISIONAL_REVOLUTE'),
                 accepted_discovery=d.get('status')=='ACCEPTED',entered_refinement=entered,
                 refinement_segments=len(a.get('segments',[])),refinement_path_m=a.get('path_m'),
                 refinement_sim_s=a.get('duration_s'),refinement_measured_rotation_deg=measured_span,refinement_measured_path_m=measured_path,discovery_rotation_span_deg=np.rad2deg(d.get('evidence',{}).get('fit',{}).get('revolute',{}).get('angle_span_rad',0)),
                 refinement_rotation_span_deg=np.rad2deg(ss['refined']['revolute']['angle_span_rad']) if ss.get('refined') else None,
                 final_structure_accepted=bool(ss.get('accepted')),end_to_end_success=bool(s.get('success')))
        failure=nr.get('first_failure_state') or {}
        r['stop_model_consistency_error_m']=failure.get('articulation_consistency_error_m')
        r['stop_contact_plane_drift_m']=failure.get('relative_translation_slip_m')
        if 'SLIP' in s.get('status',''):
            model_error=(failure.get('articulation_consistency_error_m') or 0)>.001
            contact_error=(failure.get('relative_translation_slip_m') or 0)>.001
            r['slip_guard_trigger']='BOTH' if model_error and contact_error else ('MODEL_CONSISTENCY' if model_error else 'CONTACT_PLANE_DRIFT' if contact_error else 'SEE_NATIVE_LOG')
        if entered and not ss.get('accepted'):
            status=s.get('status','')
            if status.startswith(('REFINEMENT_INSUFFICIENT','REFINEMENT_MODEL')):r['failure_class']=status
            elif status=='REFINEMENT_NO_SAFE_INCREMENT':r['failure_class']='REFINEMENT_REACHABILITY_OR_COLLISION'
            elif any(x in status for x in ('CONTACT','SLIP','LOAD','COLLISION','EFFORT','SPEED','MARGIN')):r['failure_class']='REFINEMENT_SAFETY_STOP'
            else:r['failure_class']='REFINEMENT_NOT_COMPLETED'
        # Independent geometric held-out validation is distinct from autonomous
        # command replay. It never feeds fitting, candidate selection or control.
        if folder and ss.get('accepted') and (folder/'heldout_completion.json').exists():
            obs=observed or read(folder/'observations.json');T=np.asarray([x['T_tcp'] for i,x in enumerate(obs) if i%8==0 and x['phase'] in ('HELDOUT_MANIPULATION','HELDOUT_DWELL')])
            if len(T)>=12:
                for tag,filename in [('T1','discovery_articulation.json'),('T2','refined_articulation.json')]:
                    metric=predict_action(T,read(folder/filename),**{k:v for k,v in b.c['fitting'].items() if 'noise' in k})
                    r[tag+'_heldout_geometric_position_RMSE_m']=metric['position_rmse_m'];r[tag+'_heldout_geometric_rotation_RMSE_rad']=metric['rotation_rmse_rad']
        rows.append(r)
    def truth(x):return x is True or x=='True'
    reachable=sum(truth(r['grasp_feasible']) for r in rows)
    useful=sum(truth(r['bilateral_grasp']) and truth(r['useful_motion']) for r in rows)
    admitted=sum(r['discovery_admitted'] for r in rows);entered=sum(r['entered_refinement'] for r in rows)
    accepted=sum(r['final_structure_accepted'] for r in rows);success=sum(r['end_to_end_success'] for r in rows)
    groups=defaultdict(list)
    for r in rows:groups[r['asset_id']].append(r)
    assets=[{'asset_id':aid,'configs':len(rs),'reachable':sum(truth(r['grasp_feasible']) for r in rs),
             'useful_interactions':sum(truth(r['bilateral_grasp']) and truth(r['useful_motion']) for r in rs),
             'admitted_discoveries':sum(r['discovery_admitted'] for r in rs),'entered_refinement':sum(r['entered_refinement'] for r in rs),
             'accepted_structures':sum(r['final_structure_accepted'] for r in rs),'end_to_end_successes':sum(r['end_to_end_success'] for r in rs),
             'at_least_one_success':any(r['end_to_end_success'] for r in rs),'both_configs_success':all(r['end_to_end_success'] for r in rs),
             'failures':';'.join(r['failure_class'] for r in rs)} for aid,rs in groups.items()]
    counts=Counter(r['failure_class'] for r in rows)
    result={'episodes':len(rows),'assets':len(assets),'deployment_coverage':[reachable,len(rows)],
            'interaction_coverage':[useful,reachable],'discovery_coverage':[admitted,useful],
            'refinement_success':[accepted,entered],'end_to_end_success':[success,len(rows)],
            'assets_at_least_one_success':sum(a['at_least_one_success'] for a in assets),'assets_both_configs_success':sum(a['both_configs_success'] for a in assets),
            'mobile_recovered':sum(truth(r['mobile_recovered']) for r in rows),'failures':dict(counts),
            'physics_fitted':False,'provisional_is_identification_success':False,'test_outcomes_used_for_tuning':False}
    table(b.out/'per_episode.csv',rows);table(b.out/'per_asset.csv',assets)
    table(b.out/'discovery_to_refinement.csv',[{k:r.get(k) for k in ['episode_id','asset_id','discovery_status','entered_refinement','discovery_rotation_span_deg','refinement_rotation_span_deg','refinement_measured_rotation_deg','refinement_measured_path_m','refinement_segments','refinement_path_m','final_structure_accepted','discovery_axis_error_deg','refined_axis_error_deg','discovery_axis_line_error_m','refined_axis_line_error_m','T1_heldout_geometric_position_RMSE_m','T2_heldout_geometric_position_RMSE_m','failure_class']} for r in rows])
    table(b.out/'failure_breakdown.csv',[{'failure':k,'episodes':v} for k,v in counts.items()]);write(b.out/'active_summary.json',result)
    lines=['# Provisional discovery → bounded refinement: fresh TEST','',
           'SIM_TO_SIM real physical contact. Original PhysX-Mobility visuals and frozen approximate interaction proxy. No physics fitting.',
           '',f"Frozen {len(assets)} fresh assets × 2 configs. Excludes all previous 25 prepared IDs and the preceding 57-asset source pool, including every previous main asset.",'',
           '| Metric | Numerator / denominator |','|---|---:|']
    for k in ['deployment_coverage','interaction_coverage','discovery_coverage','refinement_success','end_to_end_success']:
        x,y=result[k];lines.append(f'| {k} | {x}/{y}'+(' (undefined)' if y==0 else '')+' |')
    lines+=['',f"Assets with >=1 success: {result['assets_at_least_one_success']}/{len(assets)}; both configs: {result['assets_both_configs_success']}/{len(assets)}.",
            '', '## Per asset', '', '| Asset | Reachable | Useful motion | Provisional/accepted | Refined | End-to-end | Failures |','|---|---:|---:|---:|---:|---:|---|']
    for a in assets:lines.append(f"| {a['asset_id']} | {a['reachable']}/2 | {a['useful_interactions']}/2 | {a['admitted_discoveries']} | {a['accepted_structures']} | {a['end_to_end_successes']}/2 | {a['failures']} |")
    lines+=['','## Interpretation and boundaries','',
      '- PROVISIONAL grants only bounded information gathering; it never counts as accepted reconstruction.',
      '- All final geometry thresholds and physical safety guards remain enabled. Candidate directions/estimates use measured EE and robot/contact signals only.',
      '- Refinement has one fixed budget, 1 mm segments, incremental IK/collision checks and hypothesis consensus. No asset-specific offsets or retries.',
      '- GT metrics are computed only after saved estimates and stopped physical execution. GT angles never drive online stopping.',
      '- T0 is the dataset structure prior, near oracle in simulation. It is not a visual reconstruction claim.',
      '- Geometric held-out prediction and autonomous full-start same-command replay are separate metrics; incomplete replay has no complete RMSE.',
      '- Existing contact-plane/slip supervisor is simulator-based and not proof of complete slip observability on real hardware.',
      '- 7130 remains an excluded diagnostic failure: the existing RGB-D module tracks after-grasp patches, not a validated settle-to-grasp relocalization/replanning API; no GT reset or special fix is introduced.',
      '', '## Run','', '```bash','python scripts/run_active_structure_benchmark.py --config configs/active_structure.yaml','```','',
      'For replication, use a new output and deadline. The original run, method and TEST manifest are immutable. Videos are continuous under each native candidate folder.']
    (b.out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(result,indent=2),flush=True)
