"""External passive observer; never signals or writes into an Isaac actor."""
from pathlib import Path
import json,time
O=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_heldout_20261013');ids=['45671','45783','45908','45916'];previous={};lastphase={};out=(O/'LAST_FOUR_EXTERNAL_WALL_SAMPLES.jsonl').open('a',buffering=1)
while True:
 for a in ids:
  p=O/a/'physical_01/progress.json'
  if not p.exists():continue
  try:
   st=p.stat();r=json.loads(p.read_text());key=r['t']
   if previous.get(a)==key:continue
   item={'asset_id':a,'observer_unix_s':time.time(),'source_file_mtime_ns':st.st_mtime_ns,'physics_t_s':r['t'],'phase':r['phase'],'angle_deg':r['door_angle_deg'],'left_censored_first_phase':a not in previous};out.write(json.dumps(item)+'\n');previous[a]=key
   if lastphase.get(a)!=r['phase']:print(a,r['phase'],r['t'],r['door_angle_deg'],flush=True);lastphase[a]=r['phase']
  except (OSError,ValueError):pass
 if all((O/a/'PROCESS_EXIT.json').exists() for a in ids):break
 time.sleep(1)
out.close()
