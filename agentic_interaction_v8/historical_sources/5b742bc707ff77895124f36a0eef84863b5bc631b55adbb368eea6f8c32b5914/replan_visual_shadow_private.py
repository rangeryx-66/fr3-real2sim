from pathlib import Path
from types import SimpleNamespace
import json, shutil
from xarm7_native_planning import NativeModel
from revolute10_fixed import authored_handle_colliders
from native_g1_visual_input import plan_observed
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007')
base=ROOT/'results/native_g1_visual_input_20261014'
for asset in ['45638','45600','45606']:
 d=base/asset/'shadow_01';job=json.loads((base/asset/'shadow_job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());model=NativeModel(None,job['asset_root'],source)
 export=json.loads((d/'cooked_initial.json').read_text());allowed=authored_handle_colliders(export,job['grasp_geometry_candidates'][0]);folder=next(p.parent for p in (d/'rgbd').glob('initial*/handle_points_world.npy'))
 if (d/'visual_plan.json').exists() and not (d/'visual_plan_goal3.json').exists():shutil.copyfile(d/'visual_plan.json',d/'visual_plan_goal3.json');shutil.copytree(d/'visual_planning',d/'visual_planning_goal3')
 print('REPLAN',asset,'original_goal',job.get('benchmark_goal_rad'),flush=True)
 try:
  candidate,data=plan_observed(dict(job=job,model=model,allowed=allowed,a=SimpleNamespace(output=d),ROOT=ROOT),folder)
  print('DONE',asset,candidate.get('planned_reached_state'),flush=True)
 except Exception as e:
  (d/'final_shadow_planning_failure.json').write_text(json.dumps({'error':repr(e)},indent=2));print('FAIL',asset,repr(e),flush=True)
