from pathlib import Path
import shutil,sys,json
root=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/door_transfer4_20261012');out=Path('/tmp/transfer4_compact');out.mkdir(exist_ok=True)
for ident in sys.argv[1:]:
 case=root/ident;p=case/'physical_01';dest=out/ident;dest.mkdir(exist_ok=True)
 for name in ['job.json','whole_plan.json','grasp_plan.json','planning_summary.json','PREFLIGHT.json']:
  if (case/name).exists():shutil.copy2(case/name,dest/name)
 for name in ['PHYSICAL_METRICS.json','CONTACT_ADAPTATION.json','report.json','sensor_metadata.json','source_config_hashes.json','active_source_provenance.json','actor_start_provenance.json','actor_cleanup.json','EXPORT_COMPLETE.json','acquired_grasp_reference.json','events.json','final_frame.png']:
  if (p/name).exists():shutil.copy2(p/name,dest/name)
 print(ident,flush=True)
shutil.make_archive('/tmp/transfer4_compact','zip',out)
