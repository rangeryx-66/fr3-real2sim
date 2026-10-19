from pathlib import Path
import json,time,traceback,hashlib
from xarm7_native_gripper import ROOT
from xarm7_native_transfer_contact import plan
O=ROOT/'results/native_g1_heldout_20261013';reg=json.loads((O/'REGISTERED_OBJECTS.json').read_text());extra=json.loads((O/'EXTRA_FROZEN_INPUT_HASHES.json').read_text());hashes=reg['frozen_source_sha256']|extra
results=[]
for a in reg['asset_ids']:
 d=O/a;changed=[p for p,h in hashes.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
 if changed:raise RuntimeError('PRELAUNCH_FROZEN_INPUT_CHANGED:'+str(changed))
 # Exactly one planning invocation per registered case, unchanged 300 s budget.
 with (d/'PLANNING_RESERVED.json').open('x') as f:json.dump({'time':time.time(),'wall_s_argument':300,'no_IK_before_registration':True},f)
 started=time.time()
 try:
  result=plan(json.loads((d/'planning_job.json').read_text()),d/'planning',300)
  results.append({'asset_id':a,'found':result is not None,'elapsed_s':time.time()-started})
 except Exception as e:
  traceback.print_exc();item={'asset_id':a,'found':False,'stage':'offline_planning_or_input','error':repr(e),'elapsed_s':time.time()-started};results.append(item);(d/'PLANNING_ERROR.json').write_text(json.dumps(item,indent=2))
 print('CASE_COMPLETE',json.dumps(results[-1]),flush=True)
(O/'ALL_PLANNING_COMPLETE.json').write_text(json.dumps({'results':results,'registered_order':reg['asset_ids'],'retries':0},indent=2));print('ALL_PLANNING_COMPLETE',flush=True)
