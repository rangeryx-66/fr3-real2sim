from pathlib import Path
import json,hashlib,tarfile,datetime
root=Path('/data1/home/rangeryx/fr3_real2sim_piper_mobile/results/active_structure_20261005_v1')
files=set()
for p in root.glob('*'):
 if p.is_file() and p.suffix in ('.json','.md','.csv','.png','.jpg'):files.add(p)
small={'episode_summary.json','report.json','job_private.json','identity.json','frozen_baseline.json','frozen_proxy.json','discovery_decision.json','provisional_evidence.json','refinement_entered.json','active_refinement.json','refinement_step_failure.json','structure_selection.json','discovery_articulation.json','refined_articulation.json','estimated_articulation.json','estimated_articulation.urdf','heldout_completion.json','online_outcome.json','evaluation_only.json','plan.json','mobile_search.json','deployment.json','initial_visual_handle_world.json','setup_warmup_evaluation_private.json','preflight_view_0.png','preflight_view_1.png','process.json'}
for group in ('dev','controls','episodes'):
 for p in (root/group).rglob('*'):
  if p.is_file() and (p.name in small or ('structural_prediction' in p.parts and p.name in ('results.json','T0_heldout.npz','T1_heldout.npz','T2_heldout.npz'))):
   if p.parent.name=='source':continue
   files.add(p)
for p in (root/'asset_preparation').glob('*.json'):files.add(p)
for p in (root/'dev_provenance').rglob('*'):
 if p.is_file() and (p.name in ('report.json','dev_results.json','regression_controls.json','repair.json','discovery_before.py')):files.add(p)
files.discard(root/'evidence_index.json')
index={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_root':str(root),'full_logs_and_videos_remain_on_server':True,'files':{str(p.relative_to(root)):{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size} for p in sorted(files)}}
(root/'evidence_index.json').write_text(json.dumps(index,indent=2));files.add(root/'evidence_index.json')
with tarfile.open('/tmp/active_structure_evidence.tar.gz','w:gz') as t:
 for p in sorted(files):t.add(p,arcname=str(p.relative_to(root)),recursive=False)
print('files',len(files),'bytes',Path('/tmp/active_structure_evidence.tar.gz').stat().st_size)
