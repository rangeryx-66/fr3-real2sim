"""After the single trial: audit preservation and archive complete raw evidence."""
from pathlib import Path
import sys
import json,hashlib,zipfile
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/45176_contact_transmission_20261011';old=ROOT/'results/45415_precontact_continuity_20261011';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
expected=json.loads((ROOT/'results/oracle_door_development_20261011/FROZEN_BEFORE.json').read_text());expected.update(json.loads((ROOT/'results/benchmarkA_30deg_20261011/FROZEN_BASELINES_AND_OLD_BENCHMARK_BEFORE.json').read_text()));expected.update(json.loads((out/'FROZEN_INPUT_HASHES.json').read_text()))
changes=[p for p,h in expected.items() if not Path(p).is_file() or sha(Path(p))!=h]
baseline_archive=json.loads((old/'EVIDENCE_SHA256.json').read_text());old_changes=[p for p,h in baseline_archive.items() if not (old/p).is_file() or sha(old/p)!=h]
trial=out/'physical_01';cleanup=json.loads((trial/'actor_cleanup.json').read_text());actor=json.loads((trial/'actor_process.json').read_text());proof=json.loads((trial/'actor_start_provenance.json').read_text());job=json.loads((out/'job.json').read_text())
result={'classification':'POSTRUN_PRESERVATION_AUDIT','frozen_files_checked':len(expected),'frozen_file_changes':changes,'ee14128_archived_files_checked':len(baseline_archive),'ee14128_archive_changes':old_changes,'physical_attempts':1,'automatic_retries':False,'other_assets_executed':False,'actor_pid':actor['pid'],'actor_still_running':Path('/proc/'+str(actor['pid'])).exists(),'actor_cleanup':cleanup,'actual_runner_hash_matches':proof['runner_sha256']==job['expected_runner_sha256'],'actual_helper_hash_matches':proof['trajectory_helper_sha256']==job['expected_trajectory_helper_sha256'],'active_closure_hash_matches':proof['active_closure_sha256']==job['expected_active_closure_sha256'],'frozen_45415_plan_unchanged':sha(old/'whole_plan.json')==json.loads((out/'FROZEN_INPUT_HASHES.json').read_text())[str(old/'whole_plan.json')],'physical_plan_matches_offline':sha(out/'whole_plan.json')==json.loads((out/'PREFLIGHT.json').read_text())['source_config_hashes'][str(out/'whole_plan.json')]}
(out/'FINAL_AUDIT.json').write_text(json.dumps(result,indent=2))
if '--audit-only' in sys.argv:
 print(json.dumps(result,indent=2));sys.exit(0)
# Do not traverse the symlinked frozen source/asset/provenance directories.
files=[p for p in out.rglob('*') if p.is_file() and not p.is_symlink() and p.name not in ['EVIDENCE_SHA256.json','archive_summary.log','owner.log']]
hashes={str(p.relative_to(out)):sha(p) for p in files};(out/'EVIDENCE_SHA256.json').write_text(json.dumps(hashes,indent=2));archive=Path('/tmp/45176_contact_transmission_20261011.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_STORED,allowZip64=True) as z:
 for p in files+[out/'EVIDENCE_SHA256.json']:z.write(p,str(p.relative_to(out)))
print(json.dumps({'audit':result,'archived_files':len(files),'archive_bytes':archive.stat().st_size,'archive_sha256':sha(archive)},indent=2))
