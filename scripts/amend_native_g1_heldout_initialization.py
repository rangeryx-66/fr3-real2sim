"""One-time infrastructure amendment; preserves original pre-physics failures.

Run only after confirming no owned Isaac actor is active. No geometry, plan,
controller or physical parameter is modified. IDs are registered failed imports.
"""
from pathlib import Path
import json,hashlib,time,xml.etree.ElementTree as ET
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/native_g1_heldout_20261013'
freeze=json.loads((O/'PHYSICAL_BATCH_FREEZE_LINK_CORRECTED.json').read_text());changes=[]
for a in ['45526','45638']:
 d=O/a; archive=d/'initialization_01';archive.mkdir(exist_ok=False)
 if (d/'physical_01/report.json').exists() or (d/'physical_01/EXECUTION_READY.json').exists():raise RuntimeError('physical episode already initialized; do not retry')
 mpath=d/'asset/manifest.json';m=json.loads(mpath.read_text());link=m['interaction_geometry']['selection']['handle_link']
 tree=ET.parse(m['prepared_urdf']);links={e.attrib['name']:e for e in tree.findall('link')}
 assert link in m['moving_links'] and links[link].find('visual') is not None
 for name in ['physical_01','physical_01.log','PROCESS.json','PROCESS_EXIT.json','SINGLE_ATTEMPT_RESERVED.json','GPU_PRELAUNCH.csv','GPU_OWNERSHIP.json','INITIALIZATION_FAILURE.json']:
  if (d/name).exists():(d/name).rename(archive/name)
 (archive/'manifest.json').write_bytes(mpath.read_bytes());old=m['door_link'];m['door_link']=link;mpath.write_text(json.dumps(m,indent=2))
 assert all(json.loads((archive/'manifest.json').read_text())[k]==m[k] for k in m if k!='door_link')
 changes.append({'asset_id':a,'old_door_link':old,'new_door_link':link,'reason':'registered moving visual handle link, verified in authored URDF','physical_attempts_consumed':0,'archived_initialization':str(archive)})
 (archive/'ATTEMPT_CLASSIFICATION_AMENDMENT.json').write_text(json.dumps(changes[-1],indent=2))
 for path in freeze['case_input_hashes'][a]:freeze['case_input_hashes'][a][path]=hashlib.sha256(Path(path).read_bytes()).hexdigest()
freeze.update(no_retries=False,initialization_retry_allowed=True,maximum_valid_physical_attempts_each=1,gpu_policy='free VRAM, shared permitted; no exclusive requirement',user_protocol_amendment='initialization failure does not consume physical attempt',initialization_corrections=changes)
(O/'PHYSICAL_BATCH_FREEZE_INIT_RETRY.json').write_text(json.dumps(freeze,indent=2));(O/'INITIALIZATION_RETRY_AMENDMENT.json').write_text(json.dumps({'timestamp':time.time(),'changes':changes,'no_manipulation_changes':True},indent=2))
print(json.dumps(changes,indent=2))
