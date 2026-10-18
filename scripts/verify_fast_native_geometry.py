import json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import numpy as np
from interaction_runtime.kinematics import FastNativeModel
from xarm7_native_planning import NativeModel
job=json.loads(Path(sys.argv[1]).read_text());source=json.loads((Path(job['source'])/'report.json').read_text())
m=FastNativeModel(None,job['asset_root'],source);rng=np.random.default_rng(1818);worst=0.;old=fast=0.
for i in range(100):
 q=rng.uniform(m.limits[:,0]+.05,m.limits[:,1]-.05);f=rng.uniform(0,.85,6)
 t=time.perf_counter();a=NativeModel.poses(m,q,source['robot_base_pose'],finger_q=f);old+=time.perf_counter()-t
 t=time.perf_counter();b=m.poses(q,source['robot_base_pose'],finger_q=f);fast+=time.perf_counter()-t
 worst=max(worst,max(float(abs(a[k]-b[k]).max()) for k in a))
if worst>1e-12:raise RuntimeError('FK representation mismatch')
print(json.dumps(dict(samples=100,maximum_matrix_error=worst,original_fk_wall_s=old,cached_tree_fk_wall_s=fast,speedup=old/fast,physical_episodes=0,geometry_changed=False),indent=2))
