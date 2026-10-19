import json,pickle,gzip,sys,inspect,numpy as np
from pathlib import Path
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from articulated_interaction.control import JawCenteredClosure
from piper_mobile_demo.model import Model
p=ROOT/'results/oracle_door_development_20261011/45415/development_01';job=json.loads((p/'job.json').read_text());m=Model(ROOT/'config/piper.urdf',job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()));policy=json.loads((p/'sensor_metadata.json').read_text())['original_control_policy']
ns={'np':np};source=inspect.getsource(JawCenteredClosure).replace('self.opening=float(opening) # pause further squeeze','self.opening=min(self.opening,float(opening)) # retain command');exec(source,ns);Fixed=ns['JawCenteredClosure']
controllers=None;counts=[0,0];peak=[0.,0.];last=[None,None];samples=0
for f in sorted((p/'observations_chunks').glob('*.gz')):
 for r in pickle.load(gzip.open(f,'rb')):
  if r['phase'] not in ['SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD']:continue
  if controllers is None:controllers=[JawCenteredClosure(np.array(r['T_tcp']),policy),Fixed(np.array(r['T_tcp']),policy)]
  centers=[(np.array(r['finger_world_poses'][n])@np.r_[m.pads[n].mean(0),1])[:3] for n in ['gripper_link1','gripper_link2']]
  for i,c in enumerate(controllers):
   command=c.update(r['T_tcp'],r['aperture_m'],r['telemetry']['pad_load_filtered_n'],centers,1/240)
   if command['mode']=='position':
    w=command['opening_m']
    if last[i] is not None and w>last[i]+1e-7:counts[i]+=1;peak[i]=max(peak[i],w-last[i])
    last[i]=w
  samples+=1
result={'classification':'OFFLINE_RECORDED_INPUT_COMMAND_REPLAY / NOT_PHYSICAL_VALIDATION','samples':samples,'original_aperture_reopening_count':counts[0],'repaired_aperture_reopening_count':counts[1],'maximum_original_reopening_m':peak[0],'maximum_repaired_reopening_m':peak[1],'physical_result_of_repair':'NOT_TESTED: targeted actor loaded original runner','gains_force_thresholds_and_policy_equal':True,'limitation':'Saved sensor measurements reflect the original physical commands; this replay only verifies removal of command discontinuity, not the resulting physical contact dynamics.'};(p/'CLOSURE_COMMAND_REPLAY.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
