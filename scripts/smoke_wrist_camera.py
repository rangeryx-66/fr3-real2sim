"""Validate sensor/adapter plumbing before paying for a full contact episode."""
import sys,argparse,json,numpy as np
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);a=p.parse_args();job=json.loads(a.job.read_text())
    from interactive_twin_refinement.assembly import install_setup_capture
    from articulated_interaction_skill.scene import install
    from interactive_twin_recovery.native import install as mobile
    install();install_setup_capture();mobile()
    from interactive_twin.plant import bootstrap_job
    args=SimpleNamespace(**{k:Path(job[k]) for k in ('source','asset_root','plan','output')},gpu=job['gpu'],candidate=job.get('candidate',0),ownership=None,no_operation=False,deadline_shanghai=job['deadline_shanghai'])
    args.output.mkdir(parents=True,exist_ok=True);base=json.loads((args.source/'report.json').read_text())['robot_base_pose'];scene=bootstrap_job(args,base,job)
    from wrist_reconstruction.scene import normal_background
    normal_background(scene['stage'])
    from wrist_reconstruction.capture import WristRecorder,snapshot
    from articulated_interaction_skill.capture import matrix
    recorder=WristRecorder(scene,args.output,job)
    for _ in range(30):scene['world'].step(render=True)
    # This smoke checks a mounted sensor's data pipeline at safe initialization;
    # it is not a physical grasp, scanning or articulated-motion success.
    view=dict(scene['scene_monitor_views'])['gripper_base'];p,q=view.get_world_poses();E=matrix(p[0],q[0]);recorder.sync(E)
    for _ in range(30):scene['world'].step(render=True)
    for camera in recorder.legacy_cameras+[recorder.camera]:
        rgb,depth,seg=snapshot(camera);print('SENSOR',rgb.shape,depth.shape,seg['data'].shape)
    recorder.initialize_observation()
    (args.output/'result.json').write_text(json.dumps({'status':'SENSOR_PIPELINE_PASSED','physical_capture_success':False,'observed_center_m':recorder.center.tolist(),'observed_extent_m':recorder.extent.tolist()},indent=2))
    scene['app'].close()
if __name__=='__main__':main()
