from pathlib import Path
import json,hashlib,zipfile
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/xarm7_shared_doors_20261012';manifest=[]
for p in sorted(O.rglob('*')):
 if p.is_file() and p.name not in ['EVIDENCE_SHA256.json','ARCHIVE_SUMMARY.json']:
  manifest.append({'path':str(p.relative_to(O)),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(O/'EVIDENCE_SHA256.json').write_text(json.dumps(manifest,indent=2));z=R/'results/xarm7_shared_doors_20261012.zip'
with zipfile.ZipFile(z,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as a:
 for p in sorted(O.rglob('*')):
  if p.is_file():a.write(p,Path('xarm7_shared_doors_20261012')/p.relative_to(O))
d={'archive':str(z),'bytes':z.stat().st_size,'sha256':hashlib.sha256(z.read_bytes()).hexdigest(),'files':len(manifest),'raw_chunk_streams_preserved':True,'missing_native_door_video':'Both door encoders failed at startup because ffmpeg was absent from PATH; preserve failure evidence, no extra physical trial','partial_actual_frames':'45443 actual_progress_frames starts after ~15 degrees; not a full native-rate video'};(O/'ARCHIVE_SUMMARY.json').write_text(json.dumps(d,indent=2));print(d)
