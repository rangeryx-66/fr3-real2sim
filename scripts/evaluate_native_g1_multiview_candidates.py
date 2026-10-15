"""Observed-only candidate comparison. GT evaluation is a separate program."""
import argparse,json,sys,time,shutil
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from native_g1_multiview_input import build_geometry,plan_observed,SPEC,fast_ray
from xarm7_native_planning import NativeModel

def main(a):
    old=ROOT/'results/native_g1_visual_reliability_20261015'/a.asset
    job=json.loads((old/'job.json').read_text())
    source=json.loads((Path(job['source'])/'report.json').read_text())
    folder=old/'physical_01/rgbd/initial'
    if not (folder/'handle_mask.npy').exists():folder=old/'physical_01/rgbd/initial_retry_1'
    views=sorted((ROOT/'results/native_g1_contact_geometry_study_20261015'/a.asset).glob('view_*')) if a.mode=='multi' else []
    a.output.mkdir(parents=True,exist_ok=True);began=time.perf_counter()
    data=build_geometry(folder,views,a.output/'geometry')
    job['visual_planning_wall_s']=a.wall_s
    model=NativeModel(None,job['asset_root'],source)
    # Names label allowed existing native collisions only, not scoring surfaces.
    archived=json.loads((old/'physical_01/visual_plan.json').read_text())
    allowed=archived['allowed_authored_colliders']
    shutil.copyfile(old/'physical_01/cooked_initial.json',a.output/'cooked_initial.json')
    g=dict(job=job,model=model,a=SimpleNamespace(output=a.output),allowed=allowed)
    candidate,_=plan_observed(g,a.output/'geometry')
    result=dict(asset=a.asset,mode=a.mode,wall_s=time.perf_counter()-began,geometry=data,found=candidate is not None,GT_used_in_selection=False)
    (a.output/'OFFLINE_RESULT.json').write_text(json.dumps(result,indent=2))
    print('OFFLINE_DONE',a.asset,a.mode,result['wall_s'],flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--asset',required=True);p.add_argument('--mode',choices=['single','multi'],required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wall-s',type=float,default=1200);main(p.parse_args())
