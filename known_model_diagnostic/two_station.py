"""Bounded one-switch whole-task diagnostic with explicit overlap/escape/route."""
import json,copy,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from piper_mobile_demo.model import Model
from interactive_twin_recovery.mobile import scene_at,candidate_bases
from wrist_reconstruction.planner import camera_clearance
from wrist_reconstruction.geometry import calibration

def plan(root,job,export,output,wall_s=600):
 root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=True);start=time.time()
 source=json.loads((Path(job['source'])/'report.json').read_text());model=Model(root/'config/piper.urdf',job['asset_root'],source);meta=model.manifest;kind=job['skill']['joint_type'];joint=meta['joint_name'];link=meta['moving_link'];goal=min(meta['source_joint_limits_rad']['upper'],np.pi/2 if kind=='revolute' else .15)
 template=json.loads(Path(job['diagnostic_template']).read_text());E0=np.asarray(template['T_world_handle_at_success'])@np.asarray(template['T_handle_TCP']);width=template['actual_aperture_m'];initial=source['robot_base_pose'];cal=calibration(job['camera_calibration'])
 M0=model.asset_T@model.asset.root_to_link(link,{joint:0.});owner=next(e['rigid_body_path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower());B0=np.asarray(next(e['rigid_body_world_transform'] for e in export['shapes'] if e.get('rigid_body_path')==owner))
 def delta(s):return model.asset_T@model.asset.root_to_link(link,{joint:float(s)})@np.linalg.inv(M0)
 def check(scene,q,b,s,opening=width,hold=True):
  if model.margin(q)<=.05:return False,'LOW_JOINT_MARGIN'
  P=model.poses(q,b,width=opening);ok,why=scene.check(P,delta(s)@B0,hold)
  if not ok:return False,why
  return camera_clearance(scene,P,cal)
 def path(b,L,lo,hi,scene):
  q=model.ik(delta(lo)@E0@L,b,seed=model.home,starts=5)
  if q is None:return None,'START_NO_IK'
  count=max(2,int(abs(hi-lo)/(np.pi/180 if kind=='revolute' else .002))+1);rows=[];old_s=lo
  for s in np.linspace(lo,hi,count):
   nq=model.ik(delta(s)@E0@L,b,seed=q,starts=1)
   if nq is None:return None,'CONTINUOUS_NO_IK@'+str(s)
   if np.max(abs(nq-q))>.2:return None,'BRANCH_DISCONTINUITY@'+str(s)
   for f in np.linspace(0,1,max(2,int(np.max(abs(nq-q))/.025)+2)):
    ok,why=check(scene,(1-f)*q+f*nq,b,(1-f)*old_s+f*s)
    if not ok:return None,why+'@'+str(s)
   rows.append({'state':float(s),'q':nq.tolist(),'T_tcp':(delta(s)@E0@L).tolist(),'margin_rad':model.margin(nq),'collision':'SAFE'});q=nq;old_s=float(s)
  return rows,None
 def escape(b,L,s,q,scene):
  E=delta(s)@E0@L
  for opening in (min(.1,width+.02),min(.1,width+.04)):
   for direction in (-E[:3,2],-E[:3,2]+[0,0,.5],[0,0,1.]):
    direction=np.asarray(direction,float);direction/=np.linalg.norm(direction);sq=q.copy();rows=[]
    for amount in np.linspace(0,.06,25):
     T=E.copy();T[:3,3]+=amount*direction;nq=model.ik(T,b,seed=sq,starts=1)
     if nq is None:break
     ok,why=check(scene,nq,b,s,opening,False)
     if not ok:break
     rows.append(nq.tolist());sq=nq
    if len(rows)==25:return {'opening_m':opening,'q_path':rows,'direction_world':direction.tolist()}
  return None
 def approach(b,L,s,locked,scene,opening):
  T=delta(s)@E0@L;pre=T.copy();pre[:3,3]-=.04*T[:3,2];q=model.ik(pre,b,seed=locked,starts=3)
  if q is None:return None
  outer=model
  class View:
   def __getattr__(self,n):return getattr(outer,n)
   def check(self,q,b):
    ok,why=check(scene,q,b,s,opening,False);return ok,why,None
  free=Model.joint_plan(View(),locked,q,b,iterations=1200)
  if free is None:return None
  rows=[]
  for f in np.linspace(0,1,21):
   X=pre.copy();X[:3,3]=(1-f)*pre[:3,3]+f*T[:3,3];nq=model.ik(X,b,seed=q,starts=1)
   if nq is None or not check(scene,nq,b,s,opening,False)[0]:return None
   rows.append(nq.tolist());q=nq
  return {'preplan':free,'approach':rows,'T_grasp':T.tolist()}
 def route(a,b,q,s,opening):
  yaw=(b[3]-a[3]+180)%360-180;n=max(2,int(np.linalg.norm(np.subtract(b[:2],a[:2]))/.025)+2);m=max(2,int(abs(yaw)/3)+2)
  for rotate_first in (False,True):
   xy=[[*(np.asarray(a[:2])*(1-f)+np.asarray(b[:2])*f),a[2],a[3]+(yaw if rotate_first else 0)] for f in np.linspace(0,1,n)]
   yy=[[b[0] if not rotate_first else a[0],b[1] if not rotate_first else a[1],a[2],a[3]+f*yaw] for f in np.linspace(0,1,m)];points=yy+xy[1:] if rotate_first else xy+yy[1:]
   if all(check(scene_at(root,copy.deepcopy(export),model,initial,p),q,p,s,opening,False)[0] for p in points):return {'valid':True,'waypoints':points,'arm_locked_q':q.tolist(),'translation_m':float(np.linalg.norm(np.subtract(a[:2],b[:2]))),'rotation_deg':abs(yaw)}
  return None
 summaries=[];solutions=[]
 # Prefix stations that already demonstrated the greatest complete interval.
 audits=json.loads(Path(job['prefix_audit']).read_text());rank=sorted(audits,key=lambda r:r.get('coarse_reached',0),reverse=True);first=[];seen=set()
 for r in rank:
  k=tuple(r['base'])
  if k not in seen:seen.add(k);first.append(r['base'])
  if len(first)>=10:break
 variants=[np.eye(4)]
 L=np.eye(4);L[:3,:3]=Rotation.from_euler('z',180,degrees=True).as_matrix();variants.append(L)
 for switch in ([np.pi/6,np.pi/4] if kind=='revolute' else [.06,.09]):
  overlap=np.pi/36 if kind=='revolute' else .01
  goalT=delta(goal)@E0;bases=candidate_bases(initial,{'anchor_world_m':goalT[:3,3].tolist(),'outward_normal_world':(-goalT[:3,2]).tolist()},job['wrist_experiment']['mobile_scan'])[:60];suffix=[]
  for b in bases:
   if time.time()-start>wall_s:break
   sc=scene_at(root,copy.deepcopy(export),model,initial,b)
   for vi,L in enumerate(variants):
    # Backwards screening finds branches that cannot reach the closed start.
    rows,why=path(b,L,goal,switch-overlap,sc);summaries.append({'stage':'suffix','base':b,'switch':switch,'blocker':why})
    if rows:
     rows.reverse();ret=escape(b,L,goal,np.asarray(rows[-1]['q']),sc)
     if ret:suffix.append((b,L,rows,ret,sc))
   if len(suffix)>=6:break
  for a in first:
   if time.time()-start>wall_s:break
   sa=scene_at(root,copy.deepcopy(export),model,initial,a)
   for Li in variants:
    p1,why=path(a,Li,0,switch+overlap,sa);summaries.append({'stage':'prefix','base':a,'switch':switch,'blocker':why})
    if not p1:continue
    actual1=[r for r in p1 if r['state']<=switch+1e-8];sw=actual1[-1]['state'];ret1=escape(a,Li,sw,np.asarray(actual1[-1]['q']),sa)
    if not ret1:continue
    start_approach=approach(a,Li,0,model.home,sa,.1)
    if not start_approach:continue
    locked=np.asarray(ret1['q_path'][-1])
    for b,L2,_,ret2,sb in suffix:
     p2,why=path(b,L2,sw,goal,sb)
     if not p2:continue
     rr=route(a,b,locked,sw,ret1['opening_m'])
     if not rr:continue
     app2=approach(b,L2,sw,locked,sb,ret1['opening_m'])
     if not app2:continue
     result={'mode':'KNOWN_MODEL_DIAGNOSTIC','GT_planning':True,'object_actuation':False,'attachments':False,'goal_state':goal,'units':'radians' if kind=='revolute' else 'meters','base':a,'T_grasp':start_approach['T_grasp'],'preplan':start_approach['preplan'],'approach':start_approach['approach'],'path':actual1,'end_retreat':ret2,'minimum_margin_rad':min(r['margin_rad'] for r in actual1+p2),'grasp_aperture_reference_m':width,'single_station':False,'switch_state':sw,'overlap_interval':[switch-overlap,switch+overlap],'switch_retreat':ret1,'base_route':rr,'second_station':{'base':b,**app2,'path':p2},'grasp_pose_follows_active_part':True}
     solutions.append(result);(output/'ranked_plans.json').write_text(json.dumps(solutions[:3],indent=2));break
    if solutions:break
   if solutions:break
  if solutions:break
 (output/'candidate_audit.json').write_text(json.dumps(summaries,indent=2));summary={'mode':'KNOWN_MODEL_DIAGNOSTIC','GT_planning':True,'complete_two_station_solutions':len(solutions),'checks':len(summaries),'elapsed_wall_s':time.time()-start,'bounded_station_switches':1,'blockers':summaries[-15:]};(output/'planning_summary.json').write_text(json.dumps(summary,indent=2));return summary
