"""All registered episodes remain in the denominator; conditional metrics explicit."""
import csv
import json
from collections import Counter,defaultdict
from pathlib import Path
from run_interactive_twin_benchmark import read,write


def taxonomy(s,r):
    status=s.get('status','NOT_RUN');phase=r.get('failure_phase') or (r.get('first_failure_state') or {}).get('phase') or s.get('failure_stage','deployment')
    if s.get('success'):return 'SUCCESS'
    if status in ('NOT_RUN','STARTED','CUTOFF_05_00') or status.startswith(('INITIAL_IMPORT','NATIVE_PROCESS','IMPLEMENTATION_ERROR')):return 'INFRASTRUCTURE_OR_BUDGET_STOP'
    if not s.get('grasp_feasible'):
        if any('APPROACH' in x and 'COLLISION' in x for x in s.get('fixed_plan_statuses',[])):return 'APPROACH_COLLISION'
        if 'NO_PREGRASP_IK' in s.get('fixed_plan_statuses',[]):return 'NO_PREGRASP_IK'
        return 'DEPLOYMENT_UNREACHABLE'
    if not s.get('grasp'):
        if s.get('any_approach_completed'):return 'BILATERAL_GRASP_FAIL'
        if phase in ('APPROACH','PREGRASP') and any(word in status for word in ('COLLISION','CONTACT','LOAD')):return 'APPROACH_COLLISION'
        return 'BILATERAL_GRASP_FAIL'
    if 'UNOBSERVABLE' in status:return 'ARTICULATION_UNOBSERVABLE'
    if r.get('accepted_estimate_count',0) and r.get('estimate',{}).get('joint_type') not in ('revolute','UNOBSERVABLE',None):return 'WRONG_JOINT_TYPE'
    if phase in ('EXPLORATORY','PROBE_HOLD','COMPLIANT_SETTLE'):return 'PROBE_SAFETY_STOP'
    if not s.get('refined_accepted'):return 'STRUCTURE_REFINEMENT_FAIL'
    return 'ESTIMATED_MODEL_MANIPULATION_FAIL'


def csvwrite(path,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('w') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)


def completed_evidence(summary):
    """Include rejected closure attempts without implying a successful grasp.

    This is post-episode reporting only. Native logs, acceptance rules and
    previously saved controller outcomes are never rewritten.
    """
    path=summary.get('selected_report')
    if not path and summary.get('attempts'):
        path=str(Path(summary['attempts'][-1]['output'])/'report.json')
    if not path or not Path(path).exists():return {},None,set()
    path=Path(path);report=read(path)
    phases={report.get('failure_phase') or (report.get('first_failure_state') or {}).get('phase')}
    if report.get('bilateral_hold_established'):phases.add('BILATERAL_HOLD')
    for attempt in summary.get('attempts',[]):
        f=Path(attempt['output'])/'report.json'
        if not f.exists() or f==path:continue
        r=read(f);phases.add(r.get('failure_phase') or (r.get('first_failure_state') or {}).get('phase'))
        if r.get('bilateral_hold_established'):phases.add('BILATERAL_HOLD')
    return report,path.parent,phases


def axis_line_at_observations(folder,fit):
    """Post-episode axis-line error with the axial gauge removed.

    Compare the two lines in the GT-axis-normal plane through the observed
    trajectory centroid, rather than comparing arbitrary origin points.
    """
    if folder is None or fit.get('joint_type')!='revolute':return None
    private=folder/'evaluation_only.json';observations=folder/'ee_probe_trajectory.json'
    if not private.exists() or not observations.exists():return None
    import numpy as np
    poses=np.asarray(read(observations))
    if len(poses)<2:return None
    gt=read(private)['ground_truth'];g=np.asarray(gt['axis_world']);o=np.asarray(gt['origin_world'])
    a=np.asarray(fit['revolute']['axis']);c=np.asarray(fit['revolute']['point_on_axis'])
    if abs(g@a)<1e-6:return None
    anchor=poses[:,:3,3].mean(0);predicted=c+a*(g@(anchor-c))/(g@a);truth=o+g*(g@(anchor-o))
    return float(np.linalg.norm(predicted-truth))


def report(b):
    manifest_path=b.out/'frozen_asset_manifest.json'
    if not manifest_path.exists():return
    manifest=read(manifest_path);rows=[]
    for e in manifest['episodes']:
        p=b.out/'episodes'/e['episode_id']/'episode_summary.json'
        s=read(p) if p.exists() else {'status':'NOT_RUN'}
        r,folder,phases=completed_evidence(s)
        d=s.get('discovery_evaluation',{});f=s.get('refined_evaluation',{});ev=s.get('post_episode_evaluation') or r.get('evaluation',{})
        provisional=r.get('estimate',{});rv=provisional.get('revolute',{})
        phase=r.get('failure_phase') or (r.get('first_failure_state') or {}).get('phase') or s.get('failure_stage')
        after_approach=bool(phases & {'SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD','BILATERAL_HOLD','COMPLIANT_SETTLE','EXPLORATORY','ESTIMATED_FOLLOW'})
        # Privileged startup logs are inspected only now, after report.json
        # exists. Passive motion before grasp must not be credited to interaction.
        requested=settled=None
        if folder and (folder/'setup_warmup_evaluation_private.json').exists():
            import math
            values=read(folder/'setup_warmup_evaluation_private.json').get('joint_angles_rad',[])
            if values:settled=math.degrees(values[-1])
            requested=math.degrees(read(folder/'job_private.json').get('initial_articulation_rad',0.))
        row={'episode_id':e['episode_id'],'asset_id':e['asset_id'],'configuration':e['configuration_index'],
             'initial_base':json.dumps(e['initial_base']),'fixed_base_feasible':s.get('fixed_base_feasible',False),
             'mobile_recovered':s.get('mobile_recovered',False),'mobile_selected_pose':json.dumps(s.get('mobile_selected_pose')),
             'mobile_travel_m':s.get('mobile_travel_m'),'grasp_feasible':s.get('grasp_feasible',False),
             'pregrasp_completed':after_approach or 'APPROACH' in phases,'approach_completed':after_approach,
             'attempted_candidates':len(s.get('attempts',[])),
             'bilateral_grasp':s.get('grasp',False),'useful_motion':bool(r.get('probe',{}).get('distance_m',0)>=.005),
             'probe_motion_m':r.get('probe',{}).get('distance_m'),'minimum_joint_margin_rad':r.get('minimum_joint_margin_rad'),
             'observed_contact_plane_slip_m':r.get('maximum_detected_contact_surface_drift_m'),
             'peak_estimated_model_consistency_error_m':r.get('peak_estimated_model_consistency_error_m'),
             'final_relative_slip_m':ev.get('final_true_relative_translation_slip_m'),
             'joint_type':provisional.get('joint_type'),'joint_type_correct':f.get('type_correct',d.get('type_correct',ev.get('type_correct',False))),
             'accepted_discovery':bool(r.get('accepted_estimate_count',0)),
             'provisional_position_residual_m':rv.get('position_rmse_m'),
             'provisional_rotation_residual_rad':rv.get('rotation_rmse_rad'),
             'observed_rotation_span_deg':r.get('probe',{}).get('observed_rotation_span_deg'),
             'provisional_axis_error_deg':ev.get('axis_angular_error_deg'),
             'provisional_axis_line_error_m':axis_line_at_observations(folder,provisional),
             'legacy_axis_point_to_gt_line_distance_m':ev.get('axis_line_distance_m'),
             'peak_finger_handle_force_n':r.get('peak_finger_handle_force_n'),
             'native_dangerous_contact_stop':any(x in s.get('status','') for x in ('FINGER_BACK','ROOT_HANDLE','DANGEROUS','ENVIRONMENT_CONTACT')),
             'refined_accepted':s.get('refined_accepted',False),'discovery_axis_error_deg':d.get('axis_angular_error_deg'),
             'refined_axis_error_deg':f.get('axis_angular_error_deg'),'discovery_axis_line_error_m':d.get('axis_line_distance_m'),
             'refined_axis_line_error_m':f.get('axis_line_distance_m'),'actual_opening_deg':ev.get('actual_door_displacement_deg'),
             'requested_initial_angle_deg':requested,'settled_before_motion_angle_deg':settled,
             'passive_startup_drift_deg':None if settled is None else settled-requested,
             'door_change_after_warmup_deg':None if settled is None or ev.get('actual_final_door_angle_deg') is None else ev['actual_final_door_angle_deg']-settled,
             'online_success':s.get('online_success',False),'end_to_end_success':s.get('success',False),
             'failure_class':taxonomy({**s,'any_approach_completed':after_approach},r),'native_status':s.get('status','NOT_RUN'),'failure_phase':phase,
             'video':s.get('video') or (str(folder/'contact_baseline.mp4') if folder else None)}
        for pred in s.get('heldout',{}).get('rows',[]):
            tag=pred['model'];row[tag+'_heldout_status']=pred['status'];row[tag+'_heldout_coverage']=pred.get('coverage',0)
            row[tag+'_heldout_RMSE_m']=pred.get('ee_position_rmse_m') if pred.get('full_heldout_complete') else None
            row[tag+'_final_object_center_RMSE_m']=(pred.get('post_episode_object_prediction') or {}).get('position_rmse_m')
        rows.append(row)
    groups=defaultdict(list)
    for r in rows:groups[r['asset_id']].append(r)
    assets=[]
    for aid,rs in groups.items():
        meta=next(a for a in manifest['selected_assets'] if a['asset_id']==aid)
        assets.append({'asset_id':aid,'category':meta['category'],'object_name':meta['object_name'],'episodes':len(rs),
                       'interaction_coverage':sum(r['bilateral_grasp'] and r['useful_motion'] for r in rs),
                       'refined_identification_success':sum(r['joint_type_correct'] and r['refined_accepted'] for r in rs),
                       'end_to_end_successes':sum(r['end_to_end_success'] for r in rs),
                       'at_least_one_success':any(r['end_to_end_success'] for r in rs),
                       'both_configurations_success':all(r['end_to_end_success'] for r in rs),
                       'failure_classes':';'.join(r['failure_class'] for r in rs)})
    failures=Counter(r['failure_class'] for r in rows)
    observed=sum(r['bilateral_grasp'] and r['useful_motion'] for r in rows);interacted=observed
    identified=sum(r['joint_type_correct'] and r['refined_accepted'] for r in rows);success=sum(r['end_to_end_success'] for r in rows)
    summary={'episodes':len(rows),'assets':len(assets),'executed_or_preflight_attempted':sum(r['native_status']!='NOT_RUN' for r in rows),
             'interaction_coverage':{'numerator':interacted,'denominator':len(rows)},
             'structure_identification':{'numerator':identified,'denominator':observed},
             'end_to_end':{'numerator':success,'denominator':len(rows)},
             'assets_at_least_one_success':sum(a['at_least_one_success'] for a in assets),
             'assets_both_success':sum(a['both_configurations_success'] for a in assets),
             'assets_with_interaction':sum(a['interaction_coverage']>0 for a in assets),
             'assets_with_refined_identification':sum(a['refined_identification_success']>0 for a in assets),
             'accepted_discoveries':sum(r['accepted_discovery'] for r in rows),
             'strict_discovery_conditioned_refinement':{'numerator':identified,'denominator':sum(r['accepted_discovery'] for r in rows)},
             'observable_denominator_definition':'bilateral grasp plus >=5 mm measured EE travel; not a claim that angular excitation passed the stricter discovery gate',
             'fixed_base_feasible':sum(r['fixed_base_feasible'] for r in rows),
             'mobile_recovered':sum(r['mobile_recovered'] and not r['fixed_base_feasible'] for r in rows),
             'failure_breakdown':dict(failures),'physics_fitted':False,'real_robot_claim':False,
             'regression_controls':read(b.out/'regression_controls.json') if (b.out/'regression_controls.json').exists() else {},
             'supplemental_structure_controls':read(b.out/'full_structure_controls.json') if (b.out/'full_structure_controls.json').exists() else {}}
    csvwrite(b.out/'per_episode.csv',rows);csvwrite(b.out/'per_asset.csv',assets)
    csvwrite(b.out/'failure_breakdown.csv',[{'failure_class':k,'episodes':v} for k,v in failures.items()]);write(b.out/'summary.json',summary)
    try:plots(b.out,rows)
    except Exception as error:write(b.out/'plot_error.json',{'error':str(error)})
    lines=['# Cross-object articulation structure benchmark','',
           'SIM_TO_SIM physical-contact benchmark on original PhysX-Mobility visuals with frozen approximate semantic interaction proxies. No physics fitting.',
           '',f"Frozen main set: {len(assets)} previously unprepared assets × 2 configurations; {len(rows)} episodes. DEV 7320 / 45621 are separate controls.",
           f"Interaction coverage: **{interacted}/{len(rows)}**. Correct type + accepted refinement among observable episodes: **{identified}/{observed}**. End-to-end: **{success}/{len(rows)}**.",
           f"Assets with >=1 success: **{summary['assets_at_least_one_success']}/{len(assets)}**; both configurations: **{summary['assets_both_success']}/{len(assets)}**.",
           f"Fixed-base feasible: {summary['fixed_base_feasible']}; physically repositioned recovery: {summary['mobile_recovered']}.",'',
           f"Discovery gates passed: {summary['accepted_discoveries']}. If zero, the stricter discovery-conditioned refinement rate is undefined, not a successful 0/0 result.",'',
           '| Asset | Category | Interaction | Refined ID | End-to-end | First failure stages |','|---|---|---:|---:|---:|---|']
    for a in assets:lines.append(f"| {a['asset_id']} | {a['object_name']} | {a['interaction_coverage']}/2 | {a['refined_identification_success']}/2 | {a['end_to_end_successes']}/2 | {a['failure_classes']} |")
    lines+=['','## Interpretation and limits','',
      '- Failures and unexecuted cutoff episodes stay in the denominator. Reachability is not counted as a fitter error.',
      '- Model acceptance uses measured EE only and a complete independent validation action. Reverse below 1 mm remains unavailable.',
      '- Held-out command is frozen before execution. Native predictions replay inputs, not q or object states; incomplete replays have no full-trajectory RMSE.',
      '- T0 is the dataset structure prior and is near oracle in this simulation. It is not a claimed visual Real2Sim reconstruction.',
      '- Contact supervision still uses the existing simulated contact sensor; full online slip observability on real PiPER is unproven.',
      '- The unchanged native SUSTAINED_RELATIVE_SLIP label combines contact-plane drift and estimated-model consistency error. It is not by itself evidence of true gripper slip; both signals and final post-evaluation relative slip are retained separately.',
      '- Initial assembly uses the dataset URDF; controller, mobile selection and fit do not receive GT hinge. GT metrics are computed after the native episode stops.',
      '- Mobile platform uses the frozen kinematic SE(2) route, then stops; wheel/navigation dynamics are not evaluated.',
      '- No prismatic controller was added; this table covers revolute opening objects only.',
      '- The conditional structure-ID denominator is retained grasps with >=5 mm EE travel. Some such trajectories still lack sufficient angular excitation or fail the frozen reconstruction residual gate.',
      '- A provisional correct revolute label is not an accepted discovery or a successful refined model.',
      '- Online stopping uses the estimated model and existing safety conditions. The actual door angle and final relative slip are read only after execution stops.',
      '- Per-episode CSV separates passive startup drift from subsequent door displacement. A door that moved before bilateral grasp is not credited as a successful physical interaction.',
      '', '## Physical motion and identification evidence','',
      '| Episode | Bilateral | EE travel mm | Door motion deg (post-eval) | Final relative slip mm | Min margin rad | Provisional position residual mm | First stop |',
      '|---|---:|---:|---:|---:|---:|---:|---|']
    def number(value,scale=1):return '—' if value is None else f'{value*scale:.3f}'
    for r in rows:
        if not r['attempted_candidates']:continue
        lines.append(f"| {r['episode_id']} | {r['bilateral_grasp']} | {number(r['probe_motion_m'],1000)} | {number(r['actual_opening_deg'])} | {number(r['final_relative_slip_m'],1000)} | {number(r['minimum_joint_margin_rad'])} | {number(r['provisional_position_residual_m'],1000)} | {r['native_status']} |")
    lines+=['','## Held-out structural prediction','']
    predicted=[r for r in rows if any(k.endswith('_heldout_status') for k in r)]
    if not predicted:
        lines.append('**NOT REACHED**: no accepted refined model completed the frozen held-out segment. T0/T1/T2 prediction improvement is unproven; no RMSE is fabricated or replaced by a training residual.')
    else:
        lines+=['| Episode | Model | Replay status | Coverage | Complete held-out EE RMSE mm |','|---|---|---|---:|---:|']
        for r in predicted:
            for key in r:
                if key.endswith('_heldout_status'):
                    tag=key.removesuffix('_heldout_status')
                    lines.append(f"| {r['episode_id']} | {tag} | {r[key]} | {number(r.get(tag+'_heldout_coverage'))} | {number(r.get(tag+'_heldout_RMSE_m'),1000)} |")
    lines+=['','## Regression controls','']
    for aid,r in summary['regression_controls'].items():lines.append(f"- {aid}: {r['status']}; grasp={r['grasp']}, opening={r.get('evaluation',{}).get('actual_door_displacement_deg')} degrees.")
    if summary['supplemental_structure_controls']:
        lines+=['','Supplemental controls use the same complete refinement protocol. They remain outside the unseen denominator:','']
        for aid,r in summary['supplemental_structure_controls'].items():
            lines.append(f"- {aid}: {r['status']}; robust refinement accepted={r.get('refined_accepted',False)}, actual opening={r.get('post_episode_evaluation',{}).get('actual_door_displacement_deg')} degrees.")
            native,_,_=completed_evidence(r)
            if r['status']=='SUSTAINED_RELATIVE_SLIP':
                lines.append(f"  - Stop-signal peaks: model consistency {number(native.get('peak_estimated_model_consistency_error_m'),1000)} mm; contact-plane drift {number(native.get('maximum_detected_contact_surface_drift_m'),1000)} mm. The existing guard uses their maximum; no safety threshold was changed.")
            for pred in r.get('heldout',{}).get('rows',[]):
                error=number(pred.get('ee_position_rmse_m'),1000) if pred.get('full_heldout_complete') else 'N/A (incomplete replay)'
                lines.append(f"  - {pred['model']}: {pred['status']}; held-out coverage={number(pred.get('coverage'))}, complete EE RMSE mm={error}.")
    lines+=['','## Infrastructure provenance','',
            'The selected assets, initial configurations, grasp/controller parameters and scientific thresholds were not changed after seeing outcomes. See infrastructure_history for archived failed runs, implementation repairs and every affected rerun. Reporting-only corrections are recorded separately and recompute all rows.',
            '', '## Reproduce','', '```bash','env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /data1/home/rangeryx/isaaclab-arena/.venv/bin/python scripts/run_cross_object_structure_benchmark.py --config configs/cross_object_structure.yaml','```','',
            'Frozen assets and all exclusions: frozen_asset_manifest.json, asset_preparation/source_pool.json and preparation_inventory.json. Full logs and continuous videos remain under episodes/<id>/candidate_*/.']
    (b.out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return summary


def plots(out,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    valid=[r for r in rows if r['refined_axis_error_deg'] is not None]
    fig,ax=plt.subplots(1,2,figsize=(12,4))
    for i,(old,new,label,scale) in enumerate([('discovery_axis_error_deg','refined_axis_error_deg','Axis angle (deg)',1),('discovery_axis_line_error_m','refined_axis_line_error_m','Axis-line distance (mm)',1000)]):
        for r in valid:ax[i].plot([0,1],[r[old]*scale,r[new]*scale],'-o',label=r['episode_id'])
        ax[i].set_xticks([0,1],['Discovery','Refined']);ax[i].set_ylabel(label);ax[i].grid(alpha=.25)
        if not valid:ax[i].text(.5,.5,'No episode reached robust refinement',ha='center',transform=ax[i].transAxes)
    if valid:ax[1].legend(fontsize=6)
    fig.tight_layout();fig.savefig(out/'discovery_vs_refinement.png',dpi=180);plt.close(fig)
    counts=Counter(r['failure_class'] for r in rows);fig,ax=plt.subplots(figsize=(10,4));ax.barh(list(counts),list(counts.values()));ax.set_xlabel('Registered episodes');fig.tight_layout();fig.savefig(out/'failure_breakdown.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(11,4))
    provisional=[r for r in rows if r['provisional_axis_error_deg'] is not None]
    if provisional:
        ax.bar([r['episode_id'].replace('test_','') for r in provisional],[r['provisional_axis_error_deg'] for r in provisional],color='#b27726')
        ax.tick_params(axis='x',rotation=35)
    else:ax.text(.5,.5,'No measurable articulation estimate',ha='center',transform=ax.transAxes)
    ax.set_ylabel('Post-evaluation axis angle error (deg)');ax.set_title('Provisional fits: not accepted refinement successes')
    fig.tight_layout();fig.savefig(out/'axis_errors.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(11,4))
    for r in rows:
        if r['observed_rotation_span_deg'] is None or r['provisional_position_residual_m'] is None:continue
        ax.scatter(r['observed_rotation_span_deg'],1000*r['provisional_position_residual_m'],label=r['episode_id'].replace('test_',''))
    # Display the pre-existing discovery limits; this plot changes no gate.
    ax.axvline(1.,ls='--',c='gray');ax.axhline(.3,ls='--',c='gray')
    ax.set_xlabel('Observed EE rotation span (deg)');ax.set_ylabel('Provisional circle position residual (mm)')
    ax.set_title('Discovery evidence; dashed lines are frozen gates')
    if ax.collections:ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(out/'discovery_observability.png',dpi=180);plt.close(fig)
    # Original dataset visuals rendered by the native scene loader, not newly
    # generated demonstration objects or a picture of the interaction proxy.
    manifest=read(out/'frozen_asset_manifest.json')
    fig,axs=plt.subplots(2,4,figsize=(16,8))
    for ax,asset in zip(axs.flat,manifest['selected_assets']):
        aid=asset['asset_id'];frame=out/'episodes'/f'test_{aid}_00'/'planning/preflight_view_0.png'
        if frame.exists():ax.imshow(plt.imread(frame))
        else:ax.text(.5,.5,'Frame unavailable',ha='center')
        ax.set_title(f"{aid}: {asset['object_name']}",fontsize=9);ax.axis('off')
    fig.suptitle('Frozen unseen PhysX-Mobility assets — native visual geometry')
    fig.tight_layout();fig.savefig(out/'asset_overview.jpg',dpi=130);plt.close(fig)
    import numpy as np
    curve_rows=list(rows)
    controls=read(out/'full_structure_controls.json') if (out/'full_structure_controls.json').exists() else {}
    for aid,s in controls.items():curve_rows.append({'episode_id':'control_'+aid,'video':s.get('video')})
    for r in curve_rows:
        if not r['video']:continue
        folder=Path(r['video']).parent/'structural_prediction'
        files=sorted(folder.glob('*_heldout.npz'))
        if not files:continue
        fig,axs=plt.subplots(1,2,figsize=(11,4))
        for file in files:
            data=np.load(file);t=data['time_s'];A=data['reference_ee'][:,:3,3];B=data['predicted_ee'][:,:3,3]
            if file==files[0]:axs[0].plot(t-t[0],1000*np.linalg.norm(A-A[0],axis=1),'k',label='Reference')
            label=file.stem.removesuffix('_heldout')
            axs[0].plot(t-t[0],1000*np.linalg.norm(B-A[0],axis=1),label=label)
            axs[1].plot(t-t[0],1000*np.linalg.norm(B-A,axis=1),label=label)
        axs[0].set_ylabel('EE displacement (mm)');axs[1].set_ylabel('EE prediction error (mm)')
        for ax in axs:ax.set_xlabel('Held-out time (s)');ax.legend();ax.grid(alpha=.2)
        fig.suptitle(r['episode_id']+' — native prediction (coverage in CSV)')
        fig.tight_layout();fig.savefig(out/(r['episode_id']+'_heldout.png'),dpi=160);plt.close(fig)
