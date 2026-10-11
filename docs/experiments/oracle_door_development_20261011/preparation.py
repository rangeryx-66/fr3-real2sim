from pathlib import Path
import json,hashlib,shutil,math,numpy as np
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');old=ROOT/'results/benchmarkA_30deg_20261011';out=ROOT/'results/oracle_door_development_20261011';out.mkdir(exist_ok=True)
ranking=json.loads((old/'DISCOVERY_GEOMETRY_RANKING.json').read_text())['ranking']
registered={'classification':'ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT','asset_ids':['45176','45203','45415'],'oracle_meshes':{'45176':'original-46','45203':'original-53','45415':'original-40'},'initial_trials_per_object':1,'additional_targeted_retests_total':1,'max_physical_trials':4,'planning_wall_budget_s_per_object':600,'controller_change':False}
(out/'REGISTERED_DEVELOPMENT.json').write_text(json.dumps(registered,indent=2))
if not (out/'FROZEN_BEFORE.json').exists():
 hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in old.rglob('*') if f.is_file() and not f.is_symlink()}
 for f in ROOT.joinpath('scripts').glob('*.py'):
  if f.name not in ['plan_oracle_door.py','run_oracle_door.py']:hashes[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
 (out/'FROZEN_BEFORE.json').write_text(json.dumps(hashes,indent=2))
for i,h in registered['oracle_meshes'].items():
 case=out/i;case.mkdir(exist_ok=True);asset=case/'asset';asset.mkdir(exist_ok=True)
 original=old/'discovery_assets'/i
 for f in original.iterdir():
  if f.is_dir() and f.name not in ['visual_compatibility','usd'] and not (asset/f.name).exists():(asset/f.name).symlink_to(f,target_is_directory=True)
 selection=next(x for x in ranking if x['asset_id']==i and x['mesh']==h)
 meta=json.loads((original/'manifest.json').read_text());meta['door_link']=selection['handle_link'];meta['interaction_geometry']={'method':'registered oracle handle mesh; unchanged cooked geometry','selection':selection,'pieces':[],'original_asset_root':str(original)}
 (asset/'manifest.json').write_text(json.dumps(meta,indent=2));source=case/'source';source.mkdir(exist_ok=True)
 src=json.loads((old/i/'source/report.json').read_text());export=json.loads((old/i/'setup/cooked_initial.json').read_text())
 entry=next(e for e in export['shapes'] if e.get('rigid_body_path','').endswith('/base_link'));T=np.array(entry['rigid_body_world_transform']);base=[*T[:3,3],float(np.rad2deg(np.arctan2(T[1,0],T[0,0])))];src['robot_base_pose']=base;src['task']='oracle handle development: fixed base closed to 30 degrees';(source/'report.json').write_text(json.dumps(src,indent=2))
 job=json.loads((old/i/'setup_job.json').read_text());job.update(source=str(source),asset_root=str(asset),output=str(case/'planning'),plan=str(case/'planning/grasp_plan.json'),whole_task_plan=str(case/'planning/whole_plan.json'),collision_export=str(old/i/'setup/cooked_initial.json'),grasp_geometry_candidates=[selection],development_root=str(out),classification=registered['classification'],gpu=registered['asset_ids'].index(i)+1)
 (case/'job.json').write_text(json.dumps(job,indent=2))
print(out)
