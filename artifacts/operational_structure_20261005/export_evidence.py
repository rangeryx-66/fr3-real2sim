"""Collect immutable completed-run evidence; no online model/controller changes."""
from pathlib import Path
import json,csv,hashlib,shutil
from collections import Counter

ROOT=Path('/data1/home/rangeryx/fr3_real2sim_piper_mobile')
run=ROOT/'results/operational_structure_20261005_v3'
delivery=ROOT/'results/operational_structure_delivery'
delivery.mkdir(exist_ok=True)
def read(p):return json.loads(p.read_text())
def write(p,x):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,indent=2))
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def compact(x):
 if isinstance(x,dict):return {k:compact(v) for k,v in x.items() if k not in ['support','supporting_observations','poses','position_errors_m','rotation_errors_rad','predicted_positions_m','ee_trajectory','observations']}
 if isinstance(x,list):return [compact(v) for v in x]
 return x

for p in run.glob('*'):
 if p.is_file() and (p.suffix in ['.csv','.md','.json'] and p.name not in ['stage_curves.json']):shutil.copy2(p,delivery/p.name)
for name in ['plots','diagnostic_plots']:
 if (run/name).exists():shutil.copytree(run/name,delivery/name,dirs_exist_ok=True)
if (run/'stage_curves.json').exists():shutil.copy2(run/'stage_curves.json',delivery/'stage_curves.json')
files=[];failures=[];videos=[]
folders=[]
for g in ['dev','controls_full_structure']:
 for p in (run/g).glob('*'):folders.append((g,p.name,p))
for d in sorted((run/'episodes').glob('*')):
 if not (d/'episode_summary.json').exists():continue
 s=read(d/'episode_summary.json')
 write(delivery/'episodes'/d.name/'episode_summary.json',compact(s))
 for i,a in enumerate(s.get('attempts',[])):
  f=Path(a['output']);folders.append(('episodes',d.name+'/'+f.name,f))
  if (f/'report.json').exists():
   r=read(f/'report.json');x=r.get('first_failure_state') or {}
   failures.append({'episode_id':d.name,'candidate':f.name,'status':r.get('status'),'failure_phase':r.get('failure_phase'),
    'bilateral_hold_established':r.get('bilateral_hold_established'), 'minimum_joint_margin_rad':r.get('minimum_joint_margin_rad'),
    'contact_plane_drift_m':x.get('relative_translation_slip_m'),'model_error_m':x.get('articulation_consistency_error_m'),
    'actual_opening_deg':r.get('evaluation',{}).get('actual_door_displacement_deg'),'video':str(f/'contact_baseline.mp4')})
for group,label,d in folders:
 dst=delivery/group/label
 for name in ['report.json','structure_selection.json','active_refinement.json','discovery_decision.json','model_confidence.json','episode_summary.json','refined_articulation.json','discovery_articulation.json','structured_memory.json','heldout_completion.json','operational_continuation.json']:
  p=d/name
  if p.exists():
   x=read(p);write(dst/name,compact(x));files.append({'original':str(p),'sha256':digest(p),'delivery':str(dst/name),'compact_export':True})
 v=d/'contact_baseline.mp4'
 if v.exists():videos.append({'group':group,'label':label,'video':str(v),'sha256':digest(v)})
with (delivery/'candidate_physical_failures.csv').open('w') as f:
 if failures:
  w=csv.DictWriter(f,fieldnames=list(failures[0]));w.writeheader();w.writerows(failures)
f=read(run/'frozen_method.json');m=read(run/'frozen_test_manifest.json')
changed=[p for p,h in f['code_sha256'].items() if digest(ROOT/p)!=h]
old=read(ROOT/'results/active_structure_20261005_v1/frozen_test_manifest.json')['frozen_code_sha256']
baseline_changed=[p for p,h in old.items() if digest(ROOT/p)!=h]
excluded=set(map(str,f['config']['assets']['previously_prepared_ids']))
starts=[x.stat().st_mtime for x in (run/'episodes').glob('**/process.json')]
audit={'frozen_method_unix_s':f['frozen_unix_s'],'manifest_created_unix_s':m.get('created_unix_s'),
 'changed_frozen_code':changed,'baseline_changed':baseline_changed,'baseline_files_checked':len(old),
 'selected_assets':[a['asset_id'] for a in m['selected_assets']], 'frozen_episodes':len(m['episodes']),
 'previous_exposure_overlap':sorted(excluded & {a['asset_id'] for a in m['selected_assets']}),
 'first_TEST_process_unix_s':min(starts) if starts else None,
 'selection_before_TEST':m.get('no_TEST_outcomes_before_freeze'), 'physics_fitting':False,
 'prediction_scope':'conditional geometric prediction using measured EE rotation as phase; not autonomous dynamics replay',
 'source_commit':read(run/'source_commit.json') if (run/'source_commit.json').exists() else None}
write(delivery/'integrity_audit.json',audit);write(delivery/'evidence_index.json',{'original_run':str(run),'files':files,'videos':videos,'full_observations_and_commands_retained_on_server':True})
if (run/'per_episode.csv').exists():
 rows=list(csv.DictReader((run/'per_episode.csv').open()))
 post=[]
 for r in rows:
  slip=float(r['final_true_relative_slip_m']) if r.get('final_true_relative_slip_m') else None
  post.append({'episode_id':r['episode_id'],'online_stop':r['status'],'online_stop_layer':r['stop_layer'],
    'online_max_contact_plane_drift_m':r.get('max_contact_plane_drift_m'),
    'evaluation_only_final_true_relative_slip_m':slip,
    'final_true_slip_above_original_1mm_evaluation_bound':slip>.001 if slip is not None else None,
    'GT_used_online':False,'peak_true_slip_not_available':True})
 with (delivery/'POST_EVALUATION_SAFETY.csv').open('w') as file:
  w=csv.DictWriter(file,fieldnames=list(post[0]));w.writeheader();w.writerows(post)
 summary=read(run/'operational_summary.json')
 def layer(r):
  s=r['status']
  if s=='NO_MOBILE_RECOVERY' or not r['reachable']=='True':return 'DEPLOYMENT'
  if s=='UNOBSERVABLE':return 'DISCOVERY_OBSERVABILITY'
  if s=='BILATERAL_HOLD_NOT_ESTABLISHED':return 'GRASP_ESTABLISHMENT'
  if s=='OPERATIONAL_PREDICTION_REJECTED':return 'HELDOUT_MODEL_CONFIDENCE'
  if 'MODEL_CONFIDENCE' in s:return 'MODEL_CONFIDENCE'
  if any(x in s for x in ['DANGEROUS_LOADED_CONTACT','CONTACT_PLANE_DRIFT','LOW_JOINT_MARGIN','CONTACT_LOSS','SPEED','EFFORT']):return 'PHYSICAL_SAFETY'
  return r['stop_layer']
 failure_counts=Counter((layer(r),r['status'].split(':')[0]) for r in rows if r['end_to_end_success']!='True')
 with (delivery/'failure_breakdown_normalized.csv').open('w') as file:
  w=csv.DictWriter(file,fieldnames=['layer','reason','count']);w.writeheader()
  w.writerows({'layer':k[0],'reason':k[1],'count':v} for k,v in failure_counts.items())
 lines=['# Completed task-relevant structure experiment', '',
  'Method commit: 7143b7a, based on bc0da74. No physics fitting. All original 63 baseline files remain unchanged.', '',
  '## SYSTEM — every frozen fresh TEST episode', '', '| Stage | Count / all |','|---|---:|']
 lines += [f'| {x["stage"]} | {x["count"]}/{x["denominator"]} |' for x in summary['SYSTEM']]
 lines += ['', '## STRUCTURE — initial stable bilateral hold plus >=5 mm useful measured interaction', '', '| Metric | Count / valid interaction |','|---|---:|']
 lines += [f'| {x["metric"]} | {x["count"]}/{x["denominator"]} |' for x in summary['STRUCTURE']]
 lines += ['', '## Per-asset success', '',
  f'At least one successful configuration: {sum(a["at_least_one_success"] for a in summary["assets"])}/{len(summary["assets"])} assets.',
  f'Both configurations successful: {sum(a["both_configs_success"] for a in summary["assets"])}/{len(summary["assets"])} assets.', '',
  '## First stop layer — distinct from post-evaluation retention failure', '', '| Layer | Reason | Episodes |','|---|---|---:|']
 lines += [f'| {k[0]} | {k[1]} | {v} |' for k,v in failure_counts.items()]
 lines += ['', '## Excluded-segment prediction, physical outcome and reconstruction', '',
  '| Episode | T1/T2 pos RMSE mm | T1/T2 endpoint mm | T1/T2 tangent deg | EE reconstruction mm | Original HF flag | Actual opening deg | Final true relative slip mm | Result |',
  '|---|---:|---:|---:|---:|---|---:|---:|---|']
 def val(x,scale=1):return f'{float(x)*scale:.4f}' if x not in [None,''] else 'N/A'
 for r in rows:
  if r['heldout_validation_reached']!='True':continue
  lines.append(f'| {r["episode_id"]} | {val(r["T1_position_rmse_m"],1000)} / {val(r["T2_position_rmse_m"],1000)} | {val(r["T1_endpoint_error_m"],1000)} / {val(r["T2_endpoint_error_m"],1000)} | {val(r["T1_tangent_error_deg"])} / {val(r["T2_tangent_error_deg"])} | {val(r["reconstruction_rmse_m"],1000)} | {r["high_fidelity_structure"]} | {val(r["actual_opening_deg"])} | {val(r["final_true_relative_slip_m"],1000)} | {r["status"]} |')
 lines += ['', '## Interpretation boundaries', '',
  '- Original HIGH_FIDELITY is the retained EE reconstruction/consistency criterion, not proof that the GT hinge or object trajectory was recovered accurately. See post-episode axis/axis-line errors.',
  '- Direct model-validation stops are separate from physical safety stops. Post-evaluation true relative slip can reveal drift not observable in the frozen online contact-plane projection; it does not retroactively become an online signal.',
  '- True relative slip is measured from gripper-to-moving-link transforms at episode end; maximum true slip over the trajectory is unavailable. Contact-plane drift is reported independently.',
  '- The unchanged video overlay calls this contact-plane proxy "tactile drift"; it is neither complete slip measurement nor a claim that PiPER has independent tactile arrays.',
  '- Held-out predictions are conditional geometric predictions using measured EE rotation as phase; no held-out poses enter fitting. This is not autonomous physics/input-response prediction.',
  '- This fresh collection contains six Cabinet assets from PhysX-Mobility, two frozen configurations each. Approximate handle interaction proxies remain frozen; category-wide household generalization is not established.',
  '- Both 7320 and 45621 regression controls pass; DEV 45600 demonstrates safe operational manipulation with 0.368 mm reconstruction residual, above the unchanged 0.30 mm high-fidelity criterion.',
  '- DEV 45134 is retained as FINAL_TRUE_RELATIVE_SLIP rather than success despite opening 5.42 degrees.',
  '- No TEST outcome was used to change thresholds, candidate budgets, controller or fitter. Full commands, observations, contact logs, failures and continuous videos remain in the original server run.', '',
  '## Reproduce', '', '```bash',
  'python scripts/run_operational_structure_benchmark.py --config configs/operational_structure.yaml', '```',
  'For a future new run, use a new output directory and unexpired deadline; preserve this frozen evidence.', '',
  'See REPORT.md, SYSTEM.csv, STRUCTURE.csv, per_episode.csv, per_asset.csv, POST_EVALUATION_SAFETY.csv, plots/, diagnostic_plots/, and evidence_index.json.']
 (delivery/'SYNTHESIS.md').write_text('\n'.join(lines)+'\n')
 # Post-episode evaluation plots are never fed back to selection/control.
 import numpy as np
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 pp=delivery/'plots';pp.mkdir(exist_ok=True)
 fitted=[r for r in rows if r.get('refined_axis_error_deg')]
 if fitted:
  labels=[r['episode_id'].replace('test_','') for r in fitted];x=np.arange(len(fitted))
  fig,ax=plt.subplots(1,2,figsize=(12,4))
  for name,shift in [('discovery',-.18),('refined',.18)]:
   ax[0].bar(x+shift,[float(r[name+'_axis_error_deg']) for r in fitted],.36,label=name)
   ax[1].bar(x+shift,[1000*float(r[name+'_axis_line_error_m']) for r in fitted],.36,label=name)
  ax[0].set_ylabel('Axis angular error (deg)');ax[1].set_ylabel('Axis-line error in observed plane (mm)')
  for a in ax:a.set_xticks(x,labels,rotation=20);a.legend()
  fig.suptitle('Post-episode GT evaluation only: discovery vs refinement');fig.tight_layout();fig.savefig(pp/'post_evaluation_axis_errors.png');plt.close(fig)
 observed=[r for r in rows if r.get('final_true_relative_slip_m')]
 if observed:
  x=np.arange(len(observed));fig,ax=plt.subplots(figsize=(11,4))
  ax.bar(x-.18,[1000*float(r['max_contact_plane_drift_m'] or 0) for r in observed],.36,label='Max online contact-plane projection')
  ax.bar(x+.18,[1000*float(r['final_true_relative_slip_m']) for r in observed],.36,label='Final full relative translation — GT after episode')
  ax.axhline(1,color='red',ls='--',label='Original final-retention evaluation bound; not penetration tolerance')
  ax.set_xticks(x,[r['episode_id'].replace('test_','') for r in observed],rotation=20);ax.set_ylabel('Relative drift (mm)');ax.legend(fontsize=8)
  fig.suptitle('Online projection is not complete slip observability');fig.tight_layout();fig.savefig(pp/'post_evaluation_retention.png');plt.close(fig)
print(json.dumps({'delivery':str(delivery),'audit':audit,'videos':len(videos),'candidate_reports':len(failures)},indent=2))
