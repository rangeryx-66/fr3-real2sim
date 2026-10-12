"""Passive copies of existing actual progress images; not native-rate video."""
from pathlib import Path
import json,time,hashlib
O=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_shared_doors_20261012/45443/physical_01');D=O/'actual_progress_frames';D.mkdir(exist_ok=True);records=[];previous=None
while not (O/'report.json').exists():
 try:
  state=json.loads((O/'progress.json').read_text());t=state['t']
  if t!=previous:
   image=(O/'latest_close.png').read_bytes();name='%09d.png'%int(round(t*240));(D/name).write_bytes(image);records.append({'file':name,'reported_sim_time_s':t,'actual_angle_deg':state['door_angle_deg'],'phase':state['phase'],'sha256':hashlib.sha256(image).hexdigest(),'timing_qualification':'existing progress.json/image writes are sequential, not an atomic native camera sample'});previous=t
 except Exception:pass
 time.sleep(.2)
(D/'INDEX.json').write_text(json.dumps({'classification':'PARTIAL_ACTUAL_PROGRESS_IMAGES_NOT_CONTINUOUS_NATIVE_CAMERA_VIDEO','frames':records,'no_new_physics_or_control_commands':True},indent=2))
