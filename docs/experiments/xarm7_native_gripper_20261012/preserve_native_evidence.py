"""Preserve completed, bounded native-gripper experiments; never runs physics."""
import json,hashlib,shutil,zipfile,time
from pathlib import Path
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/xarm7_native_gripper_20261012';saved=O/'shared_sources';saved.mkdir(exist_ok=True)
freeze=json.loads((O/'CONTACT_BATCH_FREEZE.json').read_text());after={}
for s,h in freeze['sources'].items():
 p=Path(s);after[s]=hashlib.sha256(p.read_bytes()).hexdigest();shutil.copy2(p,saved/p.name)
(O/'CONTACT_BATCH_SOURCE_AUDIT.json').write_text(json.dumps({'before':freeze['sources'],'after':after,'changed':[s for s,h in freeze['sources'].items() if after[s]!=h],'unchanged_shared_execution_across_cases':all(after[s]==h for s,h in freeze['sources'].items())},indent=2))
for n in ['prepare_native_jobs.py','plan_native_batch.py','native_preflight.py']:
 p=ROOT/'scripts'/n
 if p.exists():shutil.copy2(p,O/n)
files=sorted(p for p in O.rglob('*') if p.is_file() and p.name not in ['EVIDENCE_SHA256.json','ARCHIVE_SUMMARY.json'])
manifest={str(p.relative_to(O)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};(O/'EVIDENCE_SHA256.json').write_text(json.dumps(manifest,indent=2))
archive=O.with_suffix('.zip')
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in files+[O/'EVIDENCE_SHA256.json']:z.write(p,str(Path(O.name)/p.relative_to(O)))
summary={'archive':str(archive),'bytes':archive.stat().st_size,'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'files':len(files)+1,'lossless':True,'contains_original_native_videos_and_240hz_chunks':True,'archive_summary_is_external_to_avoid_self_reference':True}
(O/'ARCHIVE_SUMMARY.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
