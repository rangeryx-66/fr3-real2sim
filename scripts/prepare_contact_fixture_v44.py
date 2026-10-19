"""Rebuild the V4.3 initial observed geometry; engineering fixture, not live sensing."""
import json,shutil,sys,tempfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from native_g1_incremental_semantics import build_geometry


def prepare(episode,output):
    output.mkdir(parents=True,exist_ok=False)
    with tempfile.TemporaryDirectory(prefix='v44-archived-rgbd-') as tmp:
        views=[]
        for folder in sorted((episode/'rgbd').glob('generation_00_agent_*')):
            v=Path(tmp)/folder.name;v.mkdir();shutil.copy2(folder/'camera.json',v/'camera.json')
            with np.load(folder/'checkpoint_arrays.npz') as packed:
                for name in packed.files:np.save(v/(name+'.npy'),packed[name])
            if (v/'handle_mask.npy').exists():views.append(v)
        build_geometry(views[0],views[1:],output/'agent_geometry')
    old=json.loads((episode/'selected_plan_trace/first_plan.json').read_text());check=json.loads((episode/'agent_responses/005.json').read_text())['result']['approach_audit']
    plan=dict(old,preplan=check['preplan'],approach=check['approach'],base_fixed_after_grasp=True,saved_GT_grasp_template_used=False)
    (output/'visual_plan.json').write_text(json.dumps(plan,indent=2))
    info=dict(scope='ARCHIVED_RGBD_ENGINEERING_FIXTURE; never a fresh perception or independent research outcome',source=str(episode),geometry=str(output/'agent_geometry'),initial_observation=str(episode/'rgbd/generation_00_agent_00'),plan_source='actual V4.3 failed first grasp; no hand-tuned pose')
    (output/'FIXTURE.json').write_text(json.dumps(info,indent=2));return info

if __name__=='__main__':print(json.dumps(prepare(*map(Path,sys.argv[1:3]))))
