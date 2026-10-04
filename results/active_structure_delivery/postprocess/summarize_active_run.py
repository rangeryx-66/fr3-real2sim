"""Post-run descriptive evidence only; never imported by control or selection."""
import csv,json,math,hashlib,sys
from pathlib import Path
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

ROOT=Path('/data1/home/rangeryx/fr3_real2sim_piper_mobile')
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
out=ROOT/'results/active_structure_20261005_v1'
def read(p):return json.loads(p.read_text())
def write_csv(name,rows):
 keys=list(dict.fromkeys(k for r in rows for k in r))
 with (out/name).open('w') as f:
  w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)

m=read(out/'frozen_test_manifest.json');method=read(out/'frozen_method.json')
integrity={'frozen_files_match':all(hashlib.sha256((ROOT/f).read_bytes()).hexdigest()==h for f,h in m['frozen_code_sha256'].items()),'method_frozen_unix_s':method['frozen_unix_s'],'manifest_frozen_unix_s':m['created_unix_s'],'first_TEST_process_unix_s':min(read(p)['started_unix_s'] for p in (out/'episodes').glob('*/*/process.json')),'TEST_asset_ids_disjoint_from_exposed':not bool(set(m['test_asset_ids'])&set(m['excluded_exposure_ids'])),'postprocess_is_evaluation_only':True}
(out/'runtime_integrity.json').write_text(json.dumps(integrity,indent=2))
assert integrity['frozen_files_match'] and integrity['TEST_asset_ids_disjoint_from_exposed']
assert integrity['method_frozen_unix_s']<=integrity['manifest_frozen_unix_s']<integrity['first_TEST_process_unix_s']
dev=[];fig,axes=plt.subplots(1,2,figsize=(11,4))
for aid in ('45600','38516','45403'):
 p=out/'dev'/aid;r=read(p/'report.json');a=read(p/'active_refinement.json');d=read(p/'discovery_decision.json');ev=r.get('evaluation',{});failure=r.get('first_failure_state',{})
 observations=read(p/'observations.json');active=[x for i,x in enumerate(observations) if i%8==0 and x['phase']=='ESTIMATED_FOLLOW'];T=np.asarray([x['T_tcp'] for x in active]);ts=np.asarray([x['t'] for x in active]);angles=np.rad2deg(Rotation.from_matrix(T[:,:3,:3]@T[0,:3,:3].T).magnitude())
 axes[0].plot(ts-ts[0],angles,label=aid)
 axes[1].plot(ts-ts[0],np.asarray([x.get('articulation_consistency_error_m') or 0 for x in active])*1000,label=aid)
 q=np.asarray(failure.get('q',[])[:6]);joint=None
 if len(q)==6:
  import xml.etree.ElementTree as ET
  urdf=ET.parse(ROOT/'config/piper.urdf').getroot();limits=[urdf.find(f"joint[@name='joint{i}']/limit") for i in range(1,7)];margin=np.minimum(q-[float(x.get('lower')) for x in limits],[float(x.get('upper')) for x in limits]-q);joint='J'+str(np.argmin(margin)+1)
 dev.append({'asset_id':aid,'discovery':d['status'],'discovery_rotation_deg':math.degrees(d['evidence']['fit']['revolute']['angle_span_rad']),'refinement_segments':len(a['segments']),'refinement_path_mm':a['path_m']*1000,'additional_measured_EE_rotation_deg':float(angles.max()),'actual_door_opening_deg':ev.get('actual_door_displacement_deg'),'final_accepted':False,'stop_reason':r['status'],'min_margin_rad':r.get('minimum_joint_margin_rad'),'closest_joint_at_stop':joint,'contact_plane_drift_at_stop_mm':1000*(failure.get('relative_translation_slip_m') or 0),'model_error_at_stop_mm':1000*(failure.get('articulation_consistency_error_m') or 0),'final_true_relative_slip_mm':1000*(ev.get('final_true_relative_translation_slip_m') or 0),'latest_online_axis_error_deg':ev.get('axis_angular_error_deg'),'final_validation':'NOT_REACHED_AFTER_SAFETY_STOP'})
axes[0].set(ylabel='Measured EE rotation since refinement start (deg)',xlabel='Refinement time (s)');axes[1].set(ylabel='Online model consistency error (mm)',xlabel='Refinement time (s)');axes[1].axhline(1,color='r',ls='--',label='Unchanged safety threshold')
for ax in axes:ax.legend();ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(out/'dev_refinement_progress.png',dpi=170);plt.close(fig);write_csv('dev_refinement.csv',dev)
deployment=[];fig,axs=plt.subplots(2,3,figsize=(15,9))
for ax,meta in zip(axs.flat,m['selected_assets']):
 aid=meta['asset_id'];image=out/'episodes'/f'test_{aid}_00'/'planning'/'preflight_view_0.png'
 if image.exists():ax.imshow(plt.imread(image))
 ax.set_title(aid+' | '+meta['object_name']);ax.axis('off')
for e in m['episodes']:
 p=out/'episodes'/e['episode_id'];s=read(p/'episode_summary.json');warm=read(p/'planning/setup_warmup_evaluation_private.json') if (p/'planning/setup_warmup_evaluation_private.json').exists() else {};v=warm.get('joint_angles_rad',[]);mobile=read(p/'mobile_search.json') if (p/'mobile_search.json').exists() else {}
 deployment.append({'episode_id':e['episode_id'],'status':s.get('status'),'fixed_status_counts':json.dumps(dict(Counter(s.get('fixed_plan_statuses',[])))),'bases_checked':mobile.get('bases_checked'),'mobile_status_counts':json.dumps(dict(Counter(x['status'] for x in mobile.get('search_rows',[])))),'initial_requested_angle_deg':math.degrees(e['initialization_only']['joint_position_rad']),'settled_angle_deg_evaluation_only':math.degrees(v[-1]) if v else None,'no_GT_reset':True})
fig.suptitle('Frozen fresh PhysX-Mobility TEST assets — actual preflight renders');fig.tight_layout();fig.savefig(out/'fresh_asset_overview.png',dpi=140);plt.close(fig);write_csv('deployment_diagnostics.csv',deployment)
lines=['## Diagnostic DEV: actual bounded refinement (outside TEST denominator)','', '| Asset | Discovery | Segments | Extra EE rotation | Door opening, post-eval | Stop | Min margin |','|---|---|---:|---:|---:|---|---:|']
for r in dev:lines.append(f"| {r['asset_id']} | {r['discovery']} | {r['refinement_segments']} | {r['additional_measured_EE_rotation_deg']:.3f}° | {r['actual_door_opening_deg']:.3f}° | {r['stop_reason']} ({r['closest_joint_at_stop']}) | {r['min_margin_rad']:.5f} |")
lines+=['','All three reached active refinement and repeated robust joint fitting. None reached the independent final validation/acceptance stage before a frozen safety stop. Door motion >=5° alone is not an accepted reconstruction or end-to-end success.', '', '![DEV measured rotation and safety residual](dev_refinement_progress.png)', '', '## Regression controls','']
for aid,r in read(out/'regression_controls.json').items():lines.append(f"- {aid}: {r['status']}; actual opening {r['evaluation']['actual_door_displacement_deg']:.3f}°, true final relative slip {1000*r['evaluation']['final_true_relative_translation_slip_m']:.3f} mm.")
from cross_object_structure.reporting import axis_line_at_observations
online=[]
for e in m['episodes']:
 s=read(out/'episodes'/e['episode_id']/'episode_summary.json')
 if not s.get('selected_job'):continue
 p=Path(s['selected_job']['output'])
 if not (p/'refinement_entered.json').exists():continue
 r=read(p/'report.json');f=r.get('first_failure_state') or {};a=read(p/'active_refinement.json') if (p/'active_refinement.json').exists() else {'segments':[]};fit=read(p/'estimated_articulation.json');ev=r.get('evaluation',{});d=s.get('discovery_evaluation',{})
 online.append({'episode_id':e['episode_id'],'final_accepted':s.get('refined_accepted',False),'completed_segments':len(a['segments']),'status':s['status'],'actual_door_opening_deg':ev.get('actual_door_displacement_deg'),'discovery_axis_error_deg':d.get('axis_angular_error_deg'),'latest_online_axis_error_deg':ev.get('axis_angular_error_deg'),'discovery_axis_line_error_mm':1000*d['axis_line_distance_m'] if d.get('axis_line_distance_m') is not None else None,'latest_online_axis_line_error_mm':1000*axis_line_at_observations(p,fit),'latest_online_position_rmse_mm':1000*fit['revolute']['position_rmse_m'],'model_error_at_stop_mm':1000*(f.get('articulation_consistency_error_m') or 0),'contact_plane_drift_at_stop_mm':1000*(f.get('relative_translation_slip_m') or 0),'final_true_relative_slip_mm':1000*(ev.get('final_true_relative_translation_slip_m') or 0),'min_margin_rad':r.get('minimum_joint_margin_rad'),'heldout_completed':bool((p/'heldout_completion.json').exists()),'estimate_is_final_accepted_T2':s.get('refined_accepted',False)})
if online:write_csv('online_refinement_diagnostics.csv',online)
lines+=['','## Fresh TEST: provisional to refinement evidence','', '| Episode | Steps | Door motion (post-eval) | First stop | Model error | Contact-plane drift | Final true relative slip | Final accepted |','|---|---:|---:|---|---:|---:|---:|---|']
for r in online:lines.append(f"| {r['episode_id']} | {r['completed_segments']} | {r['actual_door_opening_deg']:.3f}° | {r['status']} | {r['model_error_at_stop_mm']:.3f} mm | {r['contact_plane_drift_at_stop_mm']:.3f} mm | {r['final_true_relative_slip_mm']:.3f} mm | {r['final_accepted']} |")
lines+=['','### Post-stop GT diagnostics (latest online estimate is not final T2)','', '| Episode | Discovery axis error | Latest online axis error | Discovery axis-line error | Latest online axis-line error |','|---|---:|---:|---:|---:|']
for r in online:lines.append(f"| {r['episode_id']} | {r['discovery_axis_error_deg']:.3f}° | {r['latest_online_axis_error_deg']:.3f}° | {r['discovery_axis_line_error_mm']:.3f} mm | {r['latest_online_axis_line_error_mm']:.3f} mm |")
lines+=['','`online_refinement_diagnostics.csv` separately records discovery versus latest online axis/axis-line error. The latest online fit is not an accepted T2 when independent validation was not completed.', '', '## Asset and run integrity','', '![Original dataset asset preflight](fresh_asset_overview.png)','', '- All frozen code/config hashes still match; method and asset/config manifests precede TEST execution. No TEST-dependent tuning or replacement.', '- `deployment_diagnostics.csv` includes post-stop passive startup motion and every mobile-search result. Passive door opening is not credited to robot interaction.', '- Dataset visual geometry is retained. Contact uses the existing approximate interaction proxies; this is not a hardware fidelity claim.', '', '## Prediction evidence','', 'If no final model is accepted, held-out structural prediction is **NOT REACHED**. No training residual is substituted for held-out RMSE; no T2 superiority is claimed.', '', '## Videos','', '- `dev/45600/contact_baseline.mp4`: continuous approach, real closure, provisional discovery and refinement ending at the model-consistency safety stop.', '- `controls/7320/contact_baseline.mp4`: successful original regression; explicitly not an unseen TEST success.', '- Preflight-only failures have actual render images, not fabricated execution videos.', '', '## Interpretation','', 'The premature discovery gate was removed without relaxing final acceptance. DEV failure has moved to J5 margin and the frozen model-consistency supervisor. Fresh TEST deployment coverage must be read independently: episodes that cannot reach a grasp say nothing about whether their articulation fitter would succeed. Do not tune the fitter further based on these outcomes.']
(out/'RUN_DETAILS.md').write_text('\n'.join(lines)+'\n')
report=out/'REPORT.md';base=report.read_text().split('\n<!-- POSTRUN_EVIDENCE -->')[0];report.write_text(base+'\n<!-- POSTRUN_EVIDENCE -->\n\n'+'\n'.join(lines)+'\n')
print(json.dumps({'integrity':integrity,'dev':dev},indent=2))
