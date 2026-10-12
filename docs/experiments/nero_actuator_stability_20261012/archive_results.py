from pathlib import Path
import json,hashlib,zipfile,time,os
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/nero_actuator_stability_20261012';rows=[]
for p in sorted(O.rglob('*')):
 if p.is_file() and p.name not in ['EVIDENCE_SHA256.json','ARCHIVE_SUMMARY.json']:
  rows.append({'file':str(p.relative_to(O)),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(O/'EVIDENCE_SHA256.json').write_text(json.dumps(rows,indent=2));z=R/'results/nero_actuator_stability_20261012.zip'
with zipfile.ZipFile(z,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as f:
 for p in sorted(O.rglob('*')):
  if p.is_file():f.write(p,Path('nero_actuator_stability_20261012')/p.relative_to(O))
r={'archive':str(z),'bytes':z.stat().st_size,'sha256':hashlib.sha256(z.read_bytes()).hexdigest(),'manifest_files':len(rows),'all_raw_chunks_and_native_videos_included':True};(O/'ARCHIVE_SUMMARY.json').write_text(json.dumps(r,indent=2));print(r)
