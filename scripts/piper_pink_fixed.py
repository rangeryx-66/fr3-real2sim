"""Experiment-only Pinocchio/Pink IK and fixed-station search for7320.
Same grasp geometry and existing collision/margin policy; frozen code untouched.
"""
import argparse,copy,json,time,sys
from pathlib import Path
import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask,PostureTask
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from articulated_demo.kinematics import transform

class PinkIK:
 def __init__(self,urdf,home):
  full=pin.buildModelFromUrdf(str(urdf));neutral=pin.neutral(full)
  keep=[f'joint{i}' for i in range(1,7)]
  locked=[i for i in range(1,full.njoints) if full.names[i] not in keep]
  self.robot=pin.buildReducedModel(full,locked,neutral)
  self.original_limits=np.column_stack((self.robot.lowerPositionLimit,self.robot.upperPositionLimit))
  self.robot.lowerPositionLimit+=.05;self.robot.upperPositionLimit-=.05
  self.robot.velocityLimit/=3 # same existing arm command velocity budget
  self.home=np.asarray(home);self.task=FrameTask('tcp_link',position_cost=1.,orientation_cost=.1,lm_damping=1e-8,gain=.75)
  self.posture=PostureTask(cost=1e-7)
  self.rng=np.random.default_rng(7320);self.calls=0;self.last_error=None
 def solve(self,target,base,seed=None,starts=1):
  self.calls+=1;B=transform(base[:3],[0,0,np.deg2rad(base[3])]);goal=np.linalg.inv(B)@target
  self.task.set_target(pin.SE3(goal[:3,:3],goal[:3,3]))
  seeds=[self.home if seed is None else np.asarray(seed)]
  seeds+=list(self.rng.uniform(self.robot.lowerPositionLimit,self.robot.upperPositionLimit,size=(max(0,starts-1),6)))
  for initial in seeds:
   q=np.clip(initial,self.robot.lowerPositionLimit,self.robot.upperPositionLimit)
   conf=pink.Configuration(self.robot,self.robot.createData(),q);self.posture.set_target(q)
   for _ in range(65):
    current=conf.get_transform_frame_to_world('tcp_link');pe=float(np.linalg.norm(current.translation-goal[:3,3]));re=float(Rotation.from_matrix(current.rotation@goal[:3,:3].T).magnitude())
    # Numerical solve termination, not a physical execution gate.
    if pe<1e-7 and re<1e-6:break
    try:v=pink.solve_ik(conf,[self.task,self.posture],.04,solver='daqp',safety_break=False)
    except Exception as error:self.last_error=repr(error);break
    conf.integrate_inplace(v,.04)
   current=conf.get_transform_frame_to_world('tcp_link');pe=float(np.linalg.norm(current.translation-goal[:3,3]));re=float(Rotation.from_matrix(current.rotation@goal[:3,:3].T).magnitude())
   if pe<.0008 and re<.004:return conf.q.copy() # inherited IK feasibility tolerances
  return None

def search(job,output):
 from piper_mobile_demo.model import Model
 from interactive_twin_recovery.mobile import scene_at
 from wrist_reconstruction.planner import camera_clearance
 from wrist_reconstruction.geometry import calibration
 out=Path(output);out.mkdir(parents=True,exist_ok=True)
 source=json.loads((Path(job['source'])/'report.json').read_text());model=Model(ROOT/'config/piper.urdf',job['asset_root'],source);ik=PinkIK(model.urdf,model.home)
 old=json.loads(Path(job['whole_task_plan']).read_text());T0=np.array(old['T_grasp']);export=json.loads(Path(job['pink_collision_export']).read_text());initial=list(old['base']);cal=calibration(job['camera_calibration']);joint=model.manifest['joint_name'];link=model.manifest['moving_link'];M0=model.asset_T@model.asset.root_to_link(link,{joint:0.})
 def delta(s):return model.asset_T@model.asset.root_to_link(link,{joint:float(s)})@np.linalg.inv(M0)
 def arc(b,spacing=3.,branch=0):
  sc=scene_at(ROOT,export,model,initial,b);q=ik.solve(T0,b,starts=4);path=[];block='NO_INITIAL_IK'
  if q is None:return path,block
  if branch:
   flipped=q.copy();flipped[3]+=np.pi if q[3]<0 else -np.pi;flipped[4]*=-1;flipped[5]+=np.pi if q[5]<0 else -np.pi
   q=ik.solve(T0,b,flipped)
   if q is None:return path,'NO_ALTERNATE_WRIST_BRANCH'
  for deg in np.arange(0,90+spacing/2,spacing):
   deg=min(90.,float(deg));s=np.deg2rad(deg);T=delta(s)@T0;q=ik.solve(T,b,q)
   if q is None:block='NO_CONTINUOUS_PINK_IK';break
   if model.margin(q)<=.05:block='EXISTING_JOINT_MARGIN';break
   P=model.poses(q,b,width=.024);ok,block=sc.check(P,delta(s)@sc.moving_reference,True)
   if ok:ok,block=camera_clearance(sc,P,cal)
   if not ok:break
   path.append({'state':float(s),'q':q.tolist(),'T_tcp':T.tolist(),'margin_rad':model.margin(q)})
  return path,None if path and path[-1]['state']>=np.deg2rad(90) else block
 def approach(b,path):
  if not path:return None
  sc=scene_at(ROOT,export,model,initial,b);pre=T0.copy();pre[:3,3]-=.04*T0[:3,2];q=ik.solve(pre,b,np.array(path[0]['q']),starts=2)
  if q is None:return None
  view=copy.copy(model)
  def check(qq,bb):
   P=model.poses(qq,bb,width=.1);ok,why=sc.check(P,sc.moving_reference,False)
   if ok:ok,why=camera_clearance(sc,P,cal)
   return ok,why,None
  view.check=check;free=Model.joint_plan(view,model.home,q,b,iterations=600)
  if free is None:return None
  rows=[]
  for f in np.linspace(0,1,21):
   E=pre.copy();E[:3,3]=(1-f)*pre[:3,3]+f*T0[:3,3];q=ik.solve(E,b,q)
   if q is None or not check(q,b)[0]:return None
   rows.append(q.tolist())
  return {'preplan':free,'approach':rows,'T_grasp':T0.tolist()}
 started=time.time();audit=[];ranked=[]
 # Bounded SE(2) search. Fixed root height, original grasp orientation/geometry.
 pool=[(initial,0),(initial,1)]
 for y in [-.80,-.65,-.50,-.35]:
  for x in [.0,.15,.30,.45,.60,.75]:
   for yaw in [-30.,30.,90.,150.]:
    for branch in (0,1):pool.append(([x,y,initial[2],yaw],branch))
 # Refine XY near the successful grasp station, including wrist alternatives.
 for x in np.arange(.175,.351,.025):
  for y in np.arange(-.85,-.549,.025):
   for branch in (0,1):pool.append(([float(x),float(y),initial[2],initial[3]],branch))
 for i,(b,branch) in enumerate(pool):
  path,block=arc(b,branch=branch);score=np.rad2deg(path[-1]['state']) if path else 0.
  entry={'index':i,'base':b,'branch':branch,'range_deg':float(score),'blocker':block,'minimum_margin':min((p['margin_rad'] for p in path),default=0.)};audit.append(entry)
  if i==0:(out/'pink_original_station.json').write_text(json.dumps({'summary':entry,'path':path},indent=2))
  if score>=15:ranked.append((entry,path))
  if i%16==0 or score>=90:print('STATION',i,score,b,block,flush=True)
  (out/'search_progress.json').write_text(json.dumps({'checked':len(audit),'best_deg':max(a['range_deg'] for a in audit),'wall_s':time.time()-started}))
  if score>=90:
   fine,why=arc(b,.5,branch);app=approach(b,fine)
   if fine and fine[-1]['state']>=np.deg2rad(90) and app is not None:
    entry['range_deg']=90.;ranked.insert(0,(entry,fine));break
  if time.time()-started>1800:break
 ranked.sort(key=lambda p:(p[0]['range_deg'],p[0]['minimum_margin']),reverse=True)
 selected=[]
 for entry,_ in ranked[:12]:
  path,why=arc(entry['base'],.5,entry['branch']);app=approach(entry['base'],path)
  if app is None:continue
  result={'classification':'KNOWN_MODEL_DIAGNOSTIC_PINOCCHIO_PINK_FIXED_BASE','base':entry['base'],'T_grasp':T0.tolist(),**app,'path':path,'goal_state':float(np.deg2rad(90)),'planned_reached_state':path[-1]['state'],'end_retreat':None,'single_station':True,'base_motion_after_grasp':False,'regrasps':0,'IK':'Pinocchio3.9.0/Pink3.3.0/DAQP','blocker':why,'selection_summary':entry}
  selected.append(result)
  (out/f'candidate_{len(selected)-1:02d}.json').write_text(json.dumps(result,indent=2))
  if len(selected)>=3 or path[-1]['state']>=np.deg2rad(90):break
 (out/'station_audit.json').write_text(json.dumps(audit,indent=2));(out/'planning_summary.json').write_text(json.dumps({'stations_checked':len(audit),'best_selected_deg':float(np.rad2deg(selected[0]['planned_reached_state'])) if selected else None,'selected_count':len(selected),'wall_s':time.time()-started,'global_optimality':'Authored90deg upper bound reached in planning' if selected and selected[0]['planned_reached_state']>=np.deg2rad(90) else 'best found in bounded SE(2) search; not a global maximum proof','physical_validation':'pending'},indent=2));print((out/'planning_summary.json').read_text(),flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();search(json.loads(a.job.read_text()),a.output)
