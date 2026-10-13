from pathlib import Path
import json,hashlib,zipfile
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/native_g1_heldout_20261013'
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
reg=json.loads((O/'REGISTERED_OBJECTS.json').read_text());hashes=reg['frozen_source_sha256']|json.loads((O/'EXTRA_FROZEN_INPUT_HASHES.json').read_text());audit={p:digest(Path(p))==h for p,h in hashes.items()};assert all(audit.values())
files=[]
for a in ['45671','45783','45908','45916']:
 files += [p for p in (O/a).rglob('*') if p.is_file()]
for name in ['REGISTERED_OBJECTS.json','EXECUTION_PROTOCOL.json','ALL_PLANNING_COMPLETE.json','INPUT_IDENTITY_CORRECTION.json','MOVING_LINK_METADATA_CORRECTION.json','PHYSICAL_BATCH_FREEZE.json','PHYSICAL_BATCH_FREEZE_GPU2.json','PHYSICAL_BATCH_FREEZE_LINK_CORRECTED.json','GPU2_RESERVATION.json','EXTRA_FROZEN_INPUT_HASHES.json','INITIALIZATION_RETRY_AMENDMENT.json','PHYSICAL_BATCH_FREEZE_INIT_RETRY.json','FINAL_TEN_RESULTS.json','TEN_CASE_METRICS.json','TEN_CASE_EVIDENCE_INTEGRITY.json','LAST_FOUR_BATCH_COMPLETE.json','PHYSICAL_BATCH_FREEZE_LAST_FOUR.json','TEN_CASE_CONTACT_DIAGNOSTICS.json','TEN_CASE_RUNTIME_ANALYSIS.json','LAST_FOUR_EXTERNAL_WALL_SAMPLES.jsonl','USER_PARTIAL_CREDIT.json']:
 files.append(O/name)
index={str(p.relative_to(O)):{'bytes':p.stat().st_size,'sha256':digest(p)} for p in files}
out={'snapshot_scope':'last four valid closed-start physical episodes; first six preserved separately','frozen_manipulation_sources_unchanged':all(audit.values()),'source_hashes':{str(Path(p).relative_to(R)):h for p,h in hashes.items()},'raw_or_video_public_upload':False,'files':index};(O/'LAST_FOUR_EVIDENCE_INDEX.json').write_text(json.dumps(out,indent=2))
all_files=[q for asset in json.loads((O/'REGISTERED_OBJECTS.json').read_text())['asset_ids'] for q in (O/asset).rglob('*') if q.is_file()]
all_index={'source_hashes':out['source_hashes'],'frozen_manipulation_sources_unchanged':all(audit.values()),'scope':'all ten cases including preserved initialization failures; historical first-six archive is not overwritten','files':{str(q.relative_to(O)):{'bytes':q.stat().st_size,'sha256':digest(q)} for q in all_files}}
(O/'TEN_CASE_EVIDENCE_INDEX.json').write_text(json.dumps(all_index,indent=2))
archive=R/'results/native_g1_heldout_last_four_20261013.zip'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in files+[O/'LAST_FOUR_EVIDENCE_INDEX.json']:z.write(p,str(Path(O.name)/p.relative_to(O)))
summary={'private_archive':str(archive),'bytes':archive.stat().st_size,'sha256':digest(archive),'files':len(files)+1,'raw_and_video_public_upload':False};(O/'LAST_FOUR_ARCHIVE_SUMMARY.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
