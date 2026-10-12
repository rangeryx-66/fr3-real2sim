from pathlib import Path
import json,zipfile
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_shared_doors_20261012');Z=R.parent/'xarm7_report_evidence_20261012.zip';root_names={'XARM7_CONFIG.json','CONTACT_BATCH_SOURCE_FREEZE.json','CONTACT_BATCH_SOURCE_AUDIT.json','EFFECTIVE_HANDLE_INPUT_AUDIT.json','REGISTERED_CASES.json','TOOL_COMPATIBILITY.json','PHYSICAL_COMPARISON.png','ARCHIVE_SUMMARY.json','EVIDENCE_SHA256.json','xarm7_piper_sim.urdf','xarm7_dynamics_coordinates.urdf'};skip={'cooked_initial.json','native_authored_assembly_private.json','stage_prims.json','initial_scene_private.json','live.jsonl'}
with zipfile.ZipFile(Z,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as z:
 for p in sorted(R.rglob('*')):
  if not p.is_file():continue
  rel=p.relative_to(R)
  if len(rel.parts)==1 and p.name in root_names:z.write(p,rel);continue
  if rel.parts[0]=='shared_sources':z.write(p,rel);continue
  if any(x in rel.parts for x in ['native_asset','actual_progress_frames','observations_chunks','physics_steps_chunks','closure_commands_chunks','contacts_chunks']):
   if p.name=='index.json':z.write(p,rel.with_name(rel.parent.name+'_index.json'))
   continue
  if p.name in skip:continue
  if rel.parts[0] in ['45134','45443','robot_01','robot_02'] and (p.suffix in ['.json','.png','.py','.urdf']):z.write(p,rel)
print(Z,Z.stat().st_size)
