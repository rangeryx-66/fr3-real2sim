"""Post-run only: preserve all buffers, verify old baselines and owned cleanup."""
from pathlib import Path
import json,hashlib,zipfile
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/45415_closure_verification_20261011';old=ROOT/'results/oracle_door_development_20261011';benchmark=ROOT/'results/benchmarkA_30deg_20261011'
expected=json.loads((old/'FROZEN_BEFORE.json').read_text());expected.update(json.loads((benchmark/'FROZEN_BASELINES_AND_OLD_BENCHMARK_BEFORE.json').read_text()));changes=[]
for name,digest in expected.items():
 p=Path(name)
 if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:changes.append(name)
trial=out/'physical_01';actor=json.loads((trial/'actor_process.json').read_text());cleanup=json.loads((trial/'actor_cleanup.json').read_text());proof=json.loads((out/'INPUT_PROVENANCE.json').read_text())
unchanged={}
for name,digest in proof['unchanged_plan_source_manifest_hashes'].items():unchanged[name]=hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest
unchanged['copied_whole_plan']=hashlib.sha256((out/'whole_plan.json').read_bytes()).hexdigest()==proof['copied_whole_plan_sha256'];unchanged['copied_grasp_plan']=hashlib.sha256((out/'grasp_plan.json').read_bytes()).hexdigest()==proof['copied_grasp_plan_sha256']
infra=[]
for f in out.glob('infra_*/process.json'):
 d=json.loads(f.read_text());infra.append({'pid':d['pid'],'still_running':Path('/proc/'+str(d['pid'])).exists(),'physical_trials':0})
result={'classification':'PRESERVATION_AND_ONE_TRIAL_AUDIT','frozen_files_checked':len(expected),'frozen_file_changes':changes,'old_and_copied_inputs_unchanged':unchanged,'physical_trial_reservations':1,'physical_actor_pid':actor['pid'],'physical_actor_still_running':Path('/proc/'+str(actor['pid'])).exists(),'physical_actor_cleanup':cleanup,'infrastructure_test_processes':infra,'other_assets_executed':False,'automatic_retries':False};(out/'FINAL_AUDIT.json').write_text(json.dumps(result,indent=2))
files=[p for p in out.rglob('*') if p.is_file() and not p.is_symlink() and p.name not in ['EVIDENCE_SHA256.json','archive_summary.log']];hashes={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};(out/'EVIDENCE_SHA256.json').write_text(json.dumps(hashes,indent=2));archive=Path('/tmp/45415_closure_verification_20261011.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_STORED,allowZip64=True) as z:
 for p in files+[out/'EVIDENCE_SHA256.json']:z.write(p,str(p.relative_to(out)))
print(json.dumps({'audit':result,'archived_files':len(files),'archive_bytes':archive.stat().st_size,'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest()},indent=2))
