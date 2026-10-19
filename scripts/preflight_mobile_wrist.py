"""Offline feasibility from saved sensor cloud/cooked scene; no object joint model."""
import json,argparse,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--views',type=int,default=3);a=p.parse_args()
    from interaction_identification.contact_probe import robot_only_model
    from wrist_reconstruction.planner import MobileWristPlanner
    from wrist_reconstruction.geometry import calibration,coverage_views
    from articulated_interaction_skill.capture import backproject
    from interactive_twin_recovery.mobile import at_base
    config=json.loads(a.config.read_text());job=json.loads((a.capture/'frozen_wrist_job.json').read_text());base=json.loads((Path(job['source'])/'report.json').read_text())['robot_base_pose'];model=robot_only_model(ROOT/'config/piper.urdf');cal=calibration(ROOT/config['camera_calibration'])
    data=json.loads((a.capture/'cooked_initial.json').read_text());data['shapes']=[e for e in data['shapes'] if '/World/mobile_chassis' not in e['path']]
    folder=a.capture/'initial_sensor_observation';camera=json.loads((folder/'camera.json').read_text());P,_=backproject(np.load(folder/'depth_m.npy'),np.asarray(camera['K']),np.asarray(camera['T_world_camera_optical']),np.asarray(Image.open(folder/'mask.png'))>0,stride=4)
    trial=json.loads(Path(job['plan']).read_text())['trial_candidates'][0];normal=-np.asarray(trial['T'])[:3,2]
    a.output.mkdir(parents=True,exist_ok=True)
    capture=SimpleNamespace(config=config,cal=cal,observed_cloud=P,output=a.output)
    r=SimpleNamespace(base=list(base),capture=capture,arm_q=lambda:model.home.copy(),finger_q=lambda:np.array([.05,-.05]),deadline=time.time()+900,initial_visual={'outward_normal_world':normal.tolist()})
    recovery=SimpleNamespace(r=r,root=ROOT,model=model,current_D=np.eye(4),export_current=lambda D:at_base(data,base,r.base))
    planner=MobileWristPlanner(recovery);views=coverage_views(P,np.median(P,axis=0),normal,cal,config['capture']);rows=[]
    for v in views[:a.views]:
        try:choice=planner.plan(v['T_camera']);rows.append({'view_id':v['view_id'],'status':'PREFLIGHT_PASSED','choice':choice})
        except RuntimeError as error:rows.append({'view_id':v['view_id'],'status':str(error)})
        (a.output/'result.json').write_text(json.dumps({'robot_execution':False,'rows':rows},indent=2));print(rows[-1]['view_id'],rows[-1]['status'],flush=True)
if __name__=='__main__':main()
