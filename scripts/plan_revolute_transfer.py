"""Small per-object plan adapter for the existing two-station revolute runner.
Known geometry only; returns the first adequate contact path and local switch.
Uses the existing robot IK, native cooked collision scene and physical margins.
"""
import argparse,copy,json,time,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model
from interactive_twin_recovery.mobile import scene_at,candidate_bases
from wrist_reconstruction.planner import camera_clearance
from wrist_reconstruction.geometry import calibration

def plan(job,export,output,wall_s=900):
 output=Path(output);output.mkdir(parents=True,exist_ok=True);started=time.time();audit=[]
 source=json.loads((Path(job['source'])/'report.json').read_text());model=Model(ROOT/'config/piper.urdf',job['asset_root'],source);meta=model.manifest
 joint=meta['joint_name'];link=meta['moving_link'];goal=float(meta['source_joint_limits_rad']['upper']);initial=source['robot_base_pose'];cal=calibration(job['camera_calibration']);selection=meta['interaction_geometry']['selection'];width=selection['dimensions_m'][1]
 normal=model.asset_T[:3,:3]@np.asarray(selection['outward_normal_root']);axis=model.asset_T[:3,:3]@np.asarray(selection['axis_root']);anchor=(model.asset_T@np.r_[selection['sections'][0]['anchor_root_m'],1])[:3]
 P=model.poses(model.home,initial,width=.04);offset=np.mean([(np.linalg.inv(P['tcp_link'])@P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in ('gripper_link1','gripper_link2')],axis=0)
 E0=np.eye(4);E0[:3,:3]=np.column_stack((axis,np.cross(-normal,axis),-normal));E0[:3,3]=anchor-E0[:3,:3]@offset
 M0=model.asset_T@model.asset.root_to_link(link,{joint:0.});owner=next(e['rigid_body_path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower());B0=np.asarray(next(e['rigid_body_world_transform'] for e in export['shapes'] if e.get('rigid_body_path')==owner))
 def delta(s):return model.asset_T@model.asset.root_to_link(link,{joint:float(s)})@np.linalg.inv(M0)
 def pose(s,L):return delta(s)@E0@L
 def check(sc,q,b,s,opening=width,hold=True):
  if model.margin(q)<=.05:return False,'LOW_JOINT_MARGIN'
  P=model.poses(q,b,width=opening);ok,why=sc.check(P,delta(s)@B0,hold)
  return camera_clearance(sc,P,cal) if ok else (ok,why)
 def arc(b,L,lo,hi,sc):
  seed=model.ik(pose(lo,L),b,seed=model.home,starts=4);rows=[];why='NO_IK'
  if seed is None:return rows,why
  for s in np.linspace(lo,hi,max(2,int(abs(hi-lo)/np.deg2rad(1))+1)):
   q=model.ik(pose(s,L),b,seed=seed,starts=1)
   if q is None:return rows,'NO_IK'
   ok,why=check(sc,q,b,s)
   if not ok:return rows,why
   rows.append({'state':float(s),'q':q.tolist(),'T_tcp':pose(s,L).tolist(),'margin_rad':model.margin(q)});seed=q
  return rows,None
 def escape(b,L,s,q,sc):
  E=pose(s,L);opening=float(job.get('resume',{}).get('released_aperture_m',min(.1,width+.02)))
  back=-E[:3,2];up=np.array([0.,0.,1.]);side=E[:3,0]
  for distance in (.03,.02,.01):
   for direction in (back,back+.5*up,up,back+.25*side,back-.25*side,back-.5*up,side,-side):
    direction=np.asarray(direction,float);direction/=np.linalg.norm(direction);seed=q.copy();rows=[]
    escape_reason=None
    for amount in np.linspace(0,distance,13):
     X=E.copy();X[:3,3]+=amount*direction;nq=model.ik(X,b,seed=seed,starts=1)
     if nq is None:escape_reason='NO_IK';break
     ok,escape_reason=check(sc,nq,b,s,opening,False)
     if not ok:break
     rows.append(nq.tolist());seed=nq
    audit.append({'kind':'retreat','state_rad':s,'distance_m':distance,'direction':direction.tolist(),'reached_steps':len(rows),'reason':escape_reason})
    (output/'candidate_audit.json').write_text(json.dumps(audit,indent=2))
    if len(rows)==13:return {'opening_m':opening,'q_path':rows,'direction_world':direction.tolist(),'distance_m':distance}
  return None
 def approach(b,L,s,locked,sc,opening):
  T=pose(s,L);pre=T.copy();pre[:3,3]-=.04*T[:3,2];q=model.ik(pre,b,seed=locked,starts=4)
  if q is None:return None
  view=copy.copy(model)
  def free_check(q,b):
   ok,why=check(sc,q,b,s,opening,False);return ok,why,None
  view.check=free_check;free=Model.joint_plan(view,locked,q,b,iterations=1200)
  if free is None:return None
  rows=[]
  for f in np.linspace(0,1,21):
   X=pre.copy();X[:3,3]=(1-f)*pre[:3,3]+f*T[:3,3];nq=model.ik(X,b,seed=q,starts=1)
   if nq is None or not check(sc,nq,b,s,opening,False)[0]:return None
   rows.append(nq.tolist());q=nq
  return {'preplan':np.asarray(free).tolist(),'approach':rows,'T_grasp':T.tolist()}
 def route(a,b,q,s,opening):
  yaw=(b[3]-a[3]+180)%360-180;n=max(2,int(np.linalg.norm(np.subtract(b[:2],a[:2]))/.025)+2);m=max(2,int(abs(yaw)/3)+2)
  for rotate_first in (False,True):
   xy=[[*(np.asarray(a[:2])*(1-f)+np.asarray(b[:2])*f),a[2],a[3]+(yaw if rotate_first else 0)] for f in np.linspace(0,1,n)]
   yy=[[b[0] if not rotate_first else a[0],b[1] if not rotate_first else a[1],a[2],a[3]+f*yaw] for f in np.linspace(0,1,m)];points=yy+xy[1:] if rotate_first else xy+yy[1:]
   if all(check(scene_at(ROOT,export,model,initial,p),q,p,s,opening,False)[0] for p in points):return {'waypoints':points,'arm_locked_q':q.tolist()}
  return None
 def bases(s):
  T=pose(s,np.eye(4));return candidate_bases(initial,{'anchor_world_m':T[:3,3].tolist(),'outward_normal_world':(-T[:3,2]).tolist()},job['wrist_experiment']['mobile_scan'])[:40]
 variants=[]
 for flip in (0,180):
  L=np.eye(4);L[:3,:3]=Rotation.from_euler('z',flip,degrees=True).as_matrix();variants.append(L)
 # Reuse the same local primitives if a physical episode requests another station.
 if job.get('resume'):
  r=job['resume'];sw=float(r['state']);a=list(r['base']);E0=np.linalg.inv(delta(sw))@np.asarray(r['T_tcp']);width=float(r['aperture_m']);Li=np.eye(4)
  sa=scene_at(ROOT,export,model,initial,a);ret=escape(a,Li,sw,np.asarray(r['q_arm']),sa);result=None
  if r.get('T_template_closed') is not None:E0=np.asarray(r['T_template_closed'])
  if ret is not None:
   locked=np.asarray(ret['q_path'][-1]);pool=bases(goal)+bases((sw+goal)/2)+bases(sw)
   for b in pool:
    if time.time()-started>wall_s:break
    sb=scene_at(ROOT,export,model,initial,b)
    for L2 in variants:
     p2,block=arc(b,L2,sw,goal,sb)
     if not p2 or p2[-1]['state']<=sw+np.deg2rad(5):continue
     rr=route(a,b,locked,sw,ret['opening_m'])
     if rr is None:continue
     app2=approach(b,L2,sw,locked,sb,ret['opening_m'])
     if app2 is None:continue
     result={'mode':'KNOWN_MODEL_DIAGNOSTIC','asset_id':meta['asset_id'],'goal_state':goal,'planned_reached_state':p2[-1]['state'],'base':a,'switch_state':sw,'switch_retreat':ret,'base_route':rr,'second_station':{'base':b,**app2,'path':p2},'end_retreat':None,'GT_planning':True,'suffix_blocker':block,'physical_resume_input':r}
     break
    if result:break
  summary={'found':result is not None,'wall_s':time.time()-started,'start_rad':sw,'planned_reached_rad':result['planned_reached_state'] if result else None,'blocker':None if result else ('NO_LOCAL_RETREAT' if ret is None else 'LOCAL_STATIONS_EXHAUSTED')}
  if result:(output/'whole_plan.json').write_text(json.dumps(result,indent=2))
  (output/'planning_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True);return summary
 candidates=[initial]+bases(0);result=None
 for ai,a in enumerate(candidates):
  if time.time()-started>wall_s:break
  sa=scene_at(ROOT,export,model,initial,a)
  for Li in variants:
   prefix,why=arc(a,Li,0,goal,sa);reached=prefix[-1]['state'] if prefix else 0
   row={'base':a,'reached_rad':reached,'blocker':why};audit.append(row)
   print('PREFIX',ai,round(np.rad2deg(reached),2),why,flush=True)
   (output/'candidate_audit.json').write_text(json.dumps(audit,indent=2))
   if reached<np.deg2rad(15):continue # planner candidate preference, never an execution gate
   app1=approach(a,Li,0,model.home,sa,.1)
   if app1 is None:row['approach']='NO_LOCAL_PATH';continue
   # A switch inside the actual feasible interval; no object-independent 45 deg.
   for fraction in (.75,.55):
    cut=max(1,int((len(prefix)-1)*fraction));p1=prefix[:cut+1];sw=p1[-1]['state'];ret=escape(a,Li,sw,np.asarray(p1[-1]['q']),sa)
    if ret is None:continue
    locked=np.asarray(ret['q_path'][-1]);pool=bases(goal)+bases((sw+goal)/2)+bases(sw)
    for bi,b in enumerate(pool):
     if time.time()-started>wall_s:break
     if np.linalg.norm(np.subtract(a[:2],b[:2]))<.01 and abs(a[3]-b[3])<1:continue
     sb=scene_at(ROOT,export,model,initial,b)
     for L2 in variants:
      p2,block=arc(b,L2,sw,goal,sb)
      if not p2 or p2[-1]['state']<sw+np.deg2rad(10):continue
      rr=route(a,b,locked,sw,ret['opening_m'])
      if rr is None:continue
      app2=approach(b,L2,sw,locked,sb,ret['opening_m'])
      if app2 is None:continue
      result={'mode':'KNOWN_MODEL_DIAGNOSTIC','asset_id':meta['asset_id'],'goal_state':goal,'planned_reached_state':p2[-1]['state'],'base':a,**app1,'path':p1,'switch_state':sw,'switch_retreat':ret,'base_route':rr,'second_station':{'base':b,**app2,'path':p2},'end_retreat':None,'minimum_margin_rad':min(x['margin_rad'] for x in p1+p2),'grasp_pose_follows_active_part':True,'switch_selection':'inside actual initial feasible interval','full_single_station_reachable':why is None,'suffix_blocker':block,'GT_planning':True}
      break
     if result:break
    if result:break
   if result:break
  if result:break
 summary={'asset_id':meta['asset_id'],'found':result is not None,'wall_s':time.time()-started,'initial_stations_tried':len(audit),'authored_target_rad':goal,'planned_reached_rad':result['planned_reached_state'] if result else None}
 if result:
  (output/'whole_plan.json').write_text(json.dumps(result,indent=2));(output/'grasp_plan.json').write_text(json.dumps({'trial_candidates':[{'base':result['base'],'T':result['T_grasp'],'preplan':result['preplan'],'approach':result['approach']}]},indent=2))
 (output/'planning_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
 return summary
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--export',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wall-s',type=float,default=900);a=p.parse_args();plan(json.loads(a.job.read_text()),json.loads(a.export.read_text()),a.output,a.wall_s)
