"""Recompute an engineering plan from archived measured RGB-D surfaces, never a saved grasp.

This fixture intentionally avoids repeating expensive active sensing in low-level
runtime regressions. It is not a new visual-generalization research episode.
"""
import json,sys,shutil,hashlib
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import numpy as np
from interaction_runtime.kinematics import FastNativeModel
from xarm7_native_execution import collision_scene
from revolute10_fixed import authored_handle_colliders
from native_g1_multiview_input import plan_observed
root=ROOT/'results/fast_runtime_v1_20261018';out=root/'contact_fixture';out.mkdir(exist_ok=True)
job=json.loads((root/'free_02/job.json').read_text());job['output']=str(out)
source=json.loads((Path(job['source'])/'report.json').read_text());model=FastNativeModel(None,job['asset_root'],source)
export=json.loads((root/'free_02/cooked_initial.json').read_text());allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection'])
shutil.copy2(root/'free_02/cooked_initial.json',out/'cooked_initial.json')
geometry=ROOT/'results/agent_pull_push_v12_20261018/episode/agent_geometry'
g=dict(ROOT=ROOT,a=SimpleNamespace(output=out),job=job,model=model,allowed=allowed)
try:
 candidate,data=plan_observed(g,geometry)
 result=dict(ready=True,geometry=str(geometry),geometry_sha256=hashlib.sha256((geometry/'observed_geometry.json').read_bytes()).hexdigest(),fresh_plan=str(out/'visual_plan.json'),no_saved_grasp_input=True,scene_contact_episodes=0)
except Exception as e:
 import traceback
 result=dict(ready=False,error=repr(e),traceback=traceback.format_exc())
(out/'FIXTURE_RESULT.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
