"""After completion: freeze audit and complete raw/video archive."""
from pathlib import Path
import json,hashlib,zipfile
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/door_transfer4_20261012';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();batch=json.loads((out/'BATCH_COMPLETE.json').read_text());freeze=json.loads((out/'BATCH_SOURCE_FREEZE.json').read_text());expected=json.loads((ROOT/'results/oracle_door_development_20261011/FROZEN_BEFORE.json').read_text());expected.update(json.loads((ROOT/'results/benchmarkA_30deg_20261011/FROZEN_BASELINES_AND_OLD_BENCHMARK_BEFORE.json').read_text()));expected.update(json.loads((out/'FROZEN_INPUT_HASHES.json').read_text()));changed=[p for p,h in expected.items() if not Path(p).is_file() or sha(p)!=h];code_changed=[p for p,h in freeze['hashes'].items() if sha(p)!=h];audits=[]
for name in ['45415_precontact_continuity_20261011','45176_contact_transmission_20261011']:
 old=ROOT/'results'/name;manifest=json.loads((old/'EVIDENCE_SHA256.json').read_text());audits.append({'old_output':name,'files':len(manifest),'changes':[p for p,h in manifest.items() if sha(old/p)!=h]})
actors=[]
for row in batch['completed']:
 if row['physical_attempts']==0:continue
 i=row['asset_id'];trial=out/i/'physical_01';p=json.loads((trial/'actor_process.json').read_text());actors.append({'asset_id':i,'pid':p['pid'],'still_running':Path('/proc/'+str(p['pid'])).exists(),'cleanup':json.loads((trial/'actor_cleanup.json').read_text()),'export':json.loads((trial/'EXPORT_COMPLETE.json').read_text())['telemetry_issues']})
result={'frozen_files_checked':len(expected),'frozen_changes':changed,'batch_implementation_changes':code_changed,'previous_output_audits':audits,'actors':actors,'physical_episodes':batch['physical_attempts'],'no_retries':True,'no_replacement':True,'no_controller_or_environment_tuning_between_cases':not code_changed};(out/'FINAL_AUDIT.json').write_text(json.dumps(result,indent=2));files=[p for p in out.rglob('*') if p.is_file() and not p.is_symlink() and p.name not in ['EVIDENCE_SHA256.json','archive_summary.log','owner.log']];manifest={str(p.relative_to(out)):sha(p) for p in files};(out/'EVIDENCE_SHA256.json').write_text(json.dumps(manifest,indent=2));archive=Path('/tmp/door_transfer4_20261012.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_STORED,allowZip64=True) as z:
 for p in files+[out/'EVIDENCE_SHA256.json']:z.write(p,str(p.relative_to(out)))
print(json.dumps({'audit':result,'files':len(files),'archive_bytes':archive.stat().st_size,'archive_sha256':sha(archive)},indent=2))
