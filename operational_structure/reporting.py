"""SYSTEM denominator and valid-interaction STRUCTURE denominator coexist."""
import json
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np
from active_structure.reporting import table
from cross_object_structure.benchmark import read,write
from scipy.spatial.transform import Rotation


def report(b):
    dev=[]
    for aid in b.c['diagnostic_dev']:
        folder=b.out/'dev'/aid
        if not (folder/'report.json').exists():continue
        r=read(folder/'report.json');ss=read(folder/'structure_selection.json') if (folder/'structure_selection.json').exists() else {}
        model=read(folder/'model_confidence.json') if (folder/'model_confidence.json').exists() else {}
        p=ss.get('task_prediction',{})
        dev.append({'asset_id':aid,'status':r['status'],'bilateral':r.get('bilateral_hold_established'),
          'independent_validation':bool(p),'soft_refits':sum(e['event']=='REFIT_REDUCE_STEP' for e in model.get('events',[])),
          'predictive_accepted':ss.get('operational_predictive_accepted',False),'operational_structure':bool(ss.get('operational_structure') and r.get('success')),
          'high_fidelity':ss.get('high_fidelity_accepted',False),
          'T1_position_RMSE_m':p.get('T1',{}).get('position_rmse_m'),'T2_position_RMSE_m':p.get('T2',{}).get('position_rmse_m'),
          'estimated_training_residual_m':ss.get('refined',{}).get('revolute',{}).get('position_rmse_m'),
          'actual_opening_deg':r.get('evaluation',{}).get('actual_door_displacement_deg'),
          'minimum_joint_margin_rad':r.get('minimum_joint_margin_rad'),'video':str(folder/'contact_baseline.mp4'),
          'not_unseen_TEST':True})
    table(b.out/'DEV.csv',dev)
    legacy=[]
    sources=[Path(p).parent for p in b.c['calibration_logs'].values()]
    sources += [Path('results/active_structure_20261005_v1/episodes')/('test_45134_'+i)/('candidate_'+j) for i,j in [('00','01'),('01','00')]]
    from cross_object_structure.benchmark import ROOT
    for folder in sources:
        p=ROOT/folder/'report.json'
        if not p.exists():continue
        r=read(p);x=r.get('first_failure_state') or {}
        if r.get('status')!='SUSTAINED_RELATIVE_SLIP':continue
        model=(x.get('articulation_consistency_error_m') or 0)>.001
        contact=(x.get('relative_translation_slip_m') or 0)>.001
        legacy.append({'source':str(p),'original_status':r['status'],
          'actual_guard_input':'BOTH' if model and contact else 'MODEL_CONFIDENCE' if model else 'CONTACT_PLANE_DRIFT' if contact else 'UNRESOLVED',
          'model_error_m':x.get('articulation_consistency_error_m'),'contact_plane_drift_m':x.get('relative_translation_slip_m'),
          'evaluation_only_final_true_slip_m':(r.get('evaluation') or {}).get('final_true_relative_translation_slip_m'),
          'original_log_modified':False,'model_error_is_slip_measurement':False})
    table(b.out/'legacy_stop_reclassification.csv',legacy)
    if not (b.out/'frozen_test_manifest.json').exists():
        manifest={'episodes':[],'selected_assets':[]}
    else:manifest=read(b.out/'frozen_test_manifest.json')
    rows=[];events=[];curves=[];videos=[]
    for e in manifest['episodes']:
        p=b.out/'episodes'/e['episode_id']/'episode_summary.json';s=read(p) if p.exists() else {}
        folder=Path(s['selected_job']['output']) if s.get('selected_job') else None
        def get(name):return read(folder/name) if folder and (folder/name).exists() else {}
        r=get('report.json');d=get('discovery_decision.json');ss=get('structure_selection.json');active=get('active_refinement.json')
        useful=bool(r.get('bilateral_hold_established') and r.get('probe',{}).get('distance_m',0)>=.005)
        row={'episode_id':e['episode_id'],'asset_id':e['asset_id'],'fixed_base_feasible':s.get('fixed_base_feasible',False),
             'reachable':s.get('grasp_feasible',False),'mobile_recovered':s.get('mobile_recovered',False),
             'bilateral_grasp':r.get('bilateral_hold_established',False),'valid_interaction':useful,
             'discovery_status':d.get('status','NOT_REACHED'),'entered_refinement':bool(folder and (folder/'refinement_entered.json').exists()),
             'heldout_validation_reached':bool(ss.get('task_prediction')),'predictive_accepted':ss.get('operational_predictive_accepted',False),
             'operational_structure':s.get('operational_structure',False),'high_fidelity_structure':ss.get('high_fidelity_accepted',False),
             'end_to_end_success':s.get('success',False),'status':s.get('status','NOT_RUN'),
             'refinement_segments':len(active.get('segments',[])), 'minimum_joint_margin_rad':r.get('minimum_joint_margin_rad'),
             'actual_opening_deg':(r.get('evaluation') or {}).get('actual_door_displacement_deg'),
             'max_contact_plane_drift_m':r.get('maximum_detected_contact_surface_drift_m'),
             'final_true_relative_slip_m':(r.get('evaluation') or {}).get('final_true_relative_translation_slip_m'),
             'reconstruction_rmse_m':ss.get('refined',{}).get('revolute',{}).get('position_rmse_m'),
             'accuracy_threshold_m':.0003,'video':str(folder/'contact_baseline.mp4') if folder else None}
        for name in ['discovery','refined']:
            ev=s.get(name+'_evaluation',{});row[name+'_axis_error_deg']=ev.get('axis_angular_error_deg',ev.get('axis_angle_error_deg'))
            row[name+'_axis_line_error_m']=ev.get('axis_line_distance_m')
        for name in ['T1','T2']:
            for k in ['position_rmse_m','rotation_rmse_rad','endpoint_error_m','tangent_error_deg']:
                row[name+'_'+k]=ss.get('task_prediction',{}).get(name,{}).get(k)
        row['heldout_improved']=ss.get('task_prediction',{}).get('improved',False)
        model=get('model_confidence.json');evs=model.get('events',[])
        row['soft_refits']=sum(x['event']=='REFIT_REDUCE_STEP' for x in evs)
        status=row['status']
        row['stop_layer']='MODEL_CONFIDENCE' if 'MODEL_CONFIDENCE' in status else 'PHYSICAL_SAFETY' if any(x in status for x in ['CONTACT','DRIFT','MARGIN','SPEED','EFFORT','COLLISION','LOAD']) else 'DEPLOYMENT' if not row['reachable'] else 'STRUCTURE_ACCEPTANCE' if 'PREDICTION_REJECTED' in status else 'NONE' if row['end_to_end_success'] else 'INTERACTION_OR_REACHABILITY'
        events.extend({'episode_id':e['episode_id'],**{k:v for k,v in x.items() if not isinstance(v,(list,dict))}} for x in evs)
        if folder and (folder/'observations.json').exists():
            obs=read(folder/'observations.json');initial=None
            after=[np.asarray(x['T_tcp']) for ii,x in enumerate(obs) if ii%8==0 and x['phase'] in ['HELDOUT_MANIPULATION','HELDOUT_DWELL','OPERATIONAL_CONTINUATION','FINAL_HOLD']]
            row['T2_measured_continuation_m']=float(max(np.linalg.norm(T[:3,3]-after[0][:3,3]) for T in after)) if after else None
            row['T2_useful_continuation']=bool(after and row['T2_measured_continuation_m']>=b.c['validation']['observability_m'])
            if row['operational_structure'] and not row['T2_useful_continuation']:
                row.update(operational_structure=False,end_to_end_success=False,status='T2_NO_OBSERVABLE_CONTINUATION',stop_layer='ESTIMATED_MODEL_EXECUTION')
            for i,x in enumerate(obs):
                if i%8 or x['phase'] not in ['EXPLORATORY','ESTIMATED_FOLLOW','STRUCTURE_VALIDATION','HELDOUT_MANIPULATION','HELDOUT_DWELL','OPERATIONAL_CONTINUATION']:continue
                if initial is None:initial=np.asarray(x['T_tcp'])
                angle=float(np.rad2deg(Rotation.from_matrix(np.asarray(x['T_tcp'])[:3,:3]@initial[:3,:3].T).magnitude()))
                curves.append({'episode_id':e['episode_id'],'t':x['t'],'phase':x['phase'],'measured_rotation_deg':angle,
                   'model_consistency_m':x.get('articulation_consistency_error_m'),
                   'global_reconstruction_consistency_m':x.get('global_reconstruction_consistency_error_m'),
                   'contact_plane_drift_m':x['relative_translation_slip_m'],
                   'margin_rad':x['margin_rad'],'position':np.asarray(x['T_tcp'])[:3,3].tolist()})
        rows.append(row)
        if row['video']:videos.append({'episode_id':e['episode_id'],'success':row['end_to_end_success'],'video':row['video']})
    valid=[r for r in rows if r['valid_interaction']];groups=defaultdict(list)
    for r in rows:groups[r['asset_id']].append(r)
    assets=[{'asset_id':aid,'configs':len(rs),'valid_interactions':sum(r['valid_interaction'] for r in rs),
             'operational':sum(r['operational_structure'] for r in rs),'high_fidelity':sum(r['high_fidelity_structure'] for r in rs),
             'successes':sum(r['end_to_end_success'] for r in rs),'at_least_one_success':any(r['end_to_end_success'] for r in rs),
             'both_configs_success':all(r['end_to_end_success'] for r in rs)} for aid,rs in groups.items()]
    system=[{'stage':k,'count':sum(bool(r[k]) for r in rows),'denominator':len(rows)} for k in ['reachable','mobile_recovered','bilateral_grasp','valid_interaction','end_to_end_success']]
    structure=[{'metric':k,'count':sum(bool(r[k]) for r in valid),'denominator':len(valid)} for k in ['entered_refinement','heldout_validation_reached','operational_structure','high_fidelity_structure','heldout_improved','end_to_end_success']]
    structure.insert(0,{'metric':'accepted_or_provisional_discovery','count':sum(r['discovery_status'] in ['ACCEPTED','PROVISIONAL_REVOLUTE'] for r in valid),'denominator':len(valid)})
    failure=Counter((r['stop_layer'],r['status']) for r in rows if not r['end_to_end_success'])
    for name,data in [('per_episode.csv',rows),('per_asset.csv',assets),('SYSTEM.csv',system),('STRUCTURE.csv',structure),('model_confidence_events.csv',events),
       ('discovery_to_refinement.csv',rows),('failure_breakdown.csv',[{'layer':l,'reason':s,'count':v} for (l,s),v in failure.items()])]:table(b.out/name,data)
    write(b.out/'stage_curves.json',curves);write(b.out/'videos.json',videos)
    plot(b.out,curves,rows)
    summary={'SYSTEM':system,'STRUCTURE':structure,'assets':assets,'fresh_test_count':len(rows),'GT_control_inputs':False,'physics_fitting':False}
    write(b.out/'operational_summary.json',summary)
    lines=['# Task-relevant structure acceptance','','SIM_TO_SIM physical interaction; approximate interaction proxy. No physics fitting.',
      '', '## SYSTEM (all frozen episodes)','', '| Stage | Count / all |','|---|---:|']
    lines += [f"| {x['stage']} | {x['count']}/{x['denominator']} |" for x in system]
    lines += ['','## STRUCTURE (stable grasp and useful interaction)','', '| Metric | Count / valid interactions |','|---|---:|']
    lines += [f"| {x['metric']} | {x['count']}/{x['denominator']} |" for x in structure]
    lines += ['','## Safety and interpretation','',
      'Physical contact-plane drift remains a hard safety condition. Model prediction mismatch triggers pause/refit/step reduction. The contact-plane signal is not complete real-world slip observability.',
      'HIGH_FIDELITY keeps the original 0.30 mm criterion. OPERATIONAL additionally requires an excluded action predicted reliably and a safe estimated-model continuation. A predictive acceptance alone is not manipulation success.',
      'Held-out structure metrics use measured EE rotation as phase, a frozen axis/axis line, and only the first held-out pose as anchor. They are conditional geometric prediction, not autonomous physics prediction.',
      '', '## Per asset','', '| Asset | Operational | High fidelity | Manipulation |','|---|---:|---:|---:|']
    lines += [f"| {a['asset_id']} | {a['operational']} | {a['high_fidelity']} | {a['successes']}/{a['configs']} |" for a in assets]
    lines += ['','## Failures','']+[f'- {l}: {s}: {v}' for (l,s),v in failure.items()]
    lines += ['','## DEV qualification (excluded from TEST denominator)','',
              '| Asset | Stop/result | Independent validation | Soft refits | Operational | High fidelity |','|---|---|---:|---:|---:|---:|']
    lines += [f"| {x['asset_id']} | {x['status']} | {x['independent_validation']} | {x['soft_refits']} | {x['operational_structure']} | {x['high_fidelity']} |" for x in dev]
    if not rows:lines+=['','Fresh TEST was not started/qualified; zero completed TEST episodes is not a measured 0% generalization result. See DEV.csv and orchestrator.log.']
    lines+=['','## Reproduce','','```bash','python scripts/run_operational_structure_benchmark.py --config configs/operational_structure.yaml','```',
            '', 'Use a new output and deadline for another run; frozen evidence is immutable.']
    (b.out/'REPORT.md').write_text('\n'.join(lines)+'\n');print(json.dumps(summary,indent=2),flush=True)


def plot(out,curves,rows):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    groups=defaultdict(list)
    for x in curves:groups[x['episode_id']].append(x)
    plots=out/'plots';plots.mkdir(exist_ok=True)
    for name,xs in groups.items():
        fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True);t=[x['t'] for x in xs]
        axes[0].plot(t,[x['measured_rotation_deg'] for x in xs]);axes[0].set_ylabel('EE rotation (deg)')
        axes[1].plot(t,[1000*(x['model_consistency_m'] or 0) for x in xs],label='model mismatch')
        axes[1].plot(t,[1000*(x['global_reconstruction_consistency_m'] or 0) for x in xs],label='global reconstruction diagnostic',alpha=.45)
        axes[1].plot(t,[1000*x['contact_plane_drift_m'] for x in xs],label='physical contact-plane drift');axes[1].axhline(1,color='gray',ls='--');axes[1].set_ylabel('mm');axes[1].legend()
        axes[2].plot(t,[x['margin_rad'] for x in xs]);axes[2].axhline(.05,color='red');axes[2].set_ylabel('joint margin (rad)');axes[2].set_xlabel('simulation time (s)')
        last=None
        for x in xs:
            if x['phase']!=last:
                for ax in axes:ax.axvline(x['t'],color='gray',alpha=.2)
                axes[0].text(x['t'],axes[0].get_ylim()[1],x['phase'],rotation=45,fontsize=6);last=x['phase']
        fig.suptitle(name);fig.tight_layout();fig.savefig(plots/(name+'_stages.png'));plt.close(fig)
    for row in rows:
        if not row['heldout_validation_reached']:continue
        p=out/'episodes'/row['episode_id']/'episode_summary.json'
        folder=Path(read(p)['selected_job']['output']);s=read(folder/'structure_selection.json')['task_prediction']
        fig,axes=plt.subplots(1,2,figsize=(10,3.5))
        for label in ['T1','T2']:
            axes[0].plot(np.asarray(s[label]['position_errors_m'])*1000,label=label)
            axes[1].plot(np.rad2deg(s[label]['rotation_errors_rad']),label=label)
        axes[0].set_ylabel('excluded-segment position error (mm)');axes[1].set_ylabel('rotation error (deg)')
        for ax in axes:ax.set_xlabel('held-out observation');ax.legend()
        fig.suptitle(row['episode_id']);fig.tight_layout();fig.savefig(plots/(row['episode_id']+'_heldout.png'));plt.close(fig)
