"""Finite camera-elevation feasibility diagnostic from saved sensor geometry.

Does not execute the robot, query object joints, or count as wrist capture.
"""
import argparse,json,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from articulated_interaction_skill.capture import backproject
from wrist_reconstruction.geometry import calibration,coverage_views,optical_to_tcp
from wrist_reconstruction.planner import MobileWristPlanner
from interactive_twin_recovery.mobile import candidate_bases,scene_at,home_valid
from piper_mobile_demo.model import Model

def main():
 p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--base-budget',type=int,default=20);a=p.parse_args()
 job=json.loads(a.job.read_text());c=job['wrist_experiment'];cal=calibration(job['camera_calibration']);folder=a.capture/'initial_sensor_observation';meta=json.loads((folder/'camera.json').read_text());mask=np.load(folder/'sensor_mask.npy');P,_=backproject(np.load(folder/'depth_m.npy'),np.array(meta['K']),np.array(meta['T_world_camera_optical']),mask,stride=4)
 if 'initial_visual' in job:
  visual=job['initial_visual'];visual=json.loads(Path(visual).read_text()) if isinstance(visual,str) else visual
 else:
  # The preserved physical-contact entry derives its observed handle frame
  # from the existing prechecked grasp, rather than a separate visual file.
  plan=json.loads(Path(job['plan']).read_text());T=np.asarray(plan['trial_candidates'][0]['T'])
  visual={'outward_normal_world':(-T[:3,2]).tolist()}
 source=json.loads((Path(job['source'])/'report.json').read_text());model=Model(ROOT/'config/piper.urdf',job['asset_root'],source)
 initial=json.loads((a.capture/'reposition_history.json').read_text())[0]['initial_base'];export=json.loads((a.capture/'cooked_initial.json').read_text())
 policy=dict(c['capture'],pool_elevations_deg=[0,6,12],pool_azimuths_deg=[0],pool_distance_factors=[1.2]);geometry=json.loads((folder/'observed_geometry.json').read_text());views=coverage_views(P,geometry['center_m'],visual['outward_normal_world'],cal,policy)
 checker=MobileWristPlanner.__new__(MobileWristPlanner);checker.model=model;checker.r=SimpleNamespace(capture=SimpleNamespace(cal=cal))
 rows=[];a.output.parent.mkdir(parents=True,exist_ok=True)
 def save():a.output.write_text(json.dumps({'diagnostic_only':True,'counts_as_capture':False,'object_joint_queries':False,'source':'saved SAM3 RGB-D cloud and initial visual frame','base_budget_per_view':a.base_budget,'rows':rows},indent=2))
 for view in views:
  T=view['T_camera'];target=optical_to_tcp(T,cal['X']);normal=-T[:3,2].copy();normal[2]=0
  candidates=candidate_bases(initial,{'anchor_world_m':target[:3,3].tolist(),'outward_normal_world':normal.tolist()},c['mobile_scan'])[:a.base_budget]
  row={'elevation_deg':view['elevation_deg'],'T_camera':T.tolist(),'T_tcp':target.tolist(),'candidates':[]};rows.append(row);save()
  for base in candidates:
   item={'base':base};row['candidates'].append(item);save();q=model.ik(target,base,seed=model.home,starts=5)
   if q is None:item['status']='NO_IK';save();continue
   scene=scene_at(ROOT,export,model,initial,base);ok,why,_=home_valid(scene,model,base)
   if ok:ok,why,_=checker.check(scene,q,base,[.05,-.05])
   item.update(status='GOAL_VALID' if ok else why,joint_margin_rad=model.margin(q),q=q.tolist());save()
  print(view['elevation_deg'],sum(x['status']=='GOAL_VALID' for x in row['candidates']),len(row['candidates']),flush=True)
if __name__=='__main__':main()
