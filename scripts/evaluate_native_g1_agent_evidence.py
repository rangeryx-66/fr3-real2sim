"""Post-episode evaluator only. Never used by Agent decisions or planning."""
import argparse,inspect,json,sys
from pathlib import Path
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not (a.run/'AGENT_EPISODE_RESULT.json').exists():raise SystemExit('Offline GT evaluator only after Agent episode finishes')
    import evaluate_native_g1_wrist_evidence as frozen
    ns=dict(frozen.__dict__);source=inspect.getsource(frozen.evaluate).replace("run/'wrist_geometry", "run/'agent_geometry")
    exec(compile(source,__file__+'::post_episode_evaluator','exec'),ns);ns['evaluate'](a.run,a.output)
    # Comparable geometry diagnostic at an identical archived reference pose.
    # This saved pose is read ONLY here, after all executable plans are saved.
    if (a.run/'agent_geometry/observed_patches.ply').exists():
        from xarm7_native_planning import NativeModel
        from evaluate_native_g1_multiview_predictions import first
        job=json.loads((a.run.parent/'job.json').read_text());model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
        old=ROOT/'results/native_g1_multiview_skill_20261015/45600/physical_01';T=np.asarray(json.loads((old/'visual_correction.json').read_text())['T_after']);base=json.loads((old/'visual_plan.json').read_text())['base']
        comparison={}
        for label,folder in [('scene_baseline',old/'multiview_geometry'),('agent_wrist',a.run/'agent_geometry')]:
            observed=trimesh.load(folder/'observed_patches.ply',process=False);rows={'observed_only':first(model,T,base,[observed])}
            for i,scale in enumerate([.75,1.,1.25]):
                rear=trimesh.load(folder/('uncertain_rear_%d.ply'%i),process=False);rows[str(scale)]=first(model,T,base,[observed,rear])
            comparison[label]=rows
        (a.output/'FIXED_REFERENCE_CONTACT_COMPARISON.json').write_text(json.dumps(dict(reference_pose_used_only_after_episode=True,no_execution_pose_correction=True,predictions=comparison),indent=2))
