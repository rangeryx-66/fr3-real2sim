import json,time,traceback
from pathlib import Path
from xarm7_native_gripper import ROOT,OUT
from xarm7_native_planning import plan
for asset in json.loads((OUT/'REGISTERED_CASES.json').read_text())['asset_ids']:
 p=OUT/asset;job=json.loads((p/'planning_job.json').read_text())
 try:plan(job,p/'planning',wall_s=300)
 except Exception as e:
  traceback.print_exc();(p/'PLANNING_ERROR.json').write_text(json.dumps({'error':repr(e),'planning_failure_retained':True}))
(OUT/'PLANNING_COMPLETE.json').write_text(json.dumps({'at':time.time()}))
