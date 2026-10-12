from pathlib import Path
import json,psutil,time
O=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/cross_robot_20261012');s=json.loads((O/'OFFLINE_PROGRESS.json').read_text());owned=[]
for p in psutil.process_iter(['pid','cmdline']):
 cmd=p.info['cmdline'] or []
 if len(cmd)>2 and cmd[2]=='/tmp/run_crossrobot_offline_batch.py':owned.append(p)
for p in owned:p.terminate()
for row in s['active']:
 try:
  p=psutil.Process(row['pid']);cmd=p.cmdline()
  if '/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/scripts/cross_robot_offline.py' not in cmd:raise ValueError('not an owned planner')
  p.terminate();owned.append(p)
 except psutil.NoSuchProcess:pass
psutil.wait_procs(owned,timeout=10)
(O/'ENGINEERING_DIMENSION_ADAPTER_CORRECTION.json').write_text(json.dumps({'reason':'alternate IK query reused home seed; same solution was duplicated instead of exploring a second branch','offline_only':True,'physical_attempts':0,'preserved_progress':s,'terminated_exact_owned_pids':[p.pid for p in owned]},indent=2))
for name in ['offline','STUDY_SOURCE_FREEZE.json','OFFLINE_BATCH_RESERVED.json','OFFLINE_PROGRESS.json','OFFLINE_BATCH_COMPLETE.json']:
 p=O/name
 if p.exists():p.rename(O/('engineering_prebranch_'+name))
