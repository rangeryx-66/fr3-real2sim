"""Plan complete articulated excursions with frozen collision geometry.
GT articulation is explicitly permitted ONLY in this independent diagnostic.
No Isaac import or object actuation occurs in this module.
"""
import json,time,copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from piper_mobile_demo.model import Model
from interactive_twin_recovery.mobile import scene_at,candidate_bases
from wrist_reconstruction.planner import camera_clearance
from wrist_reconstruction.geometry import calibration


def plan(root,job,export,output,wall_s=600,max_bases=90):
 root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=True)
 source=json.loads((Path(job['source'])/'report.json').read_text());model=Model(root/'config/piper.urdf',job['asset_root'],source)
 kind=job['skill']['joint_type'];manifest=model.manifest;joint=manifest['joint_name'];link=manifest['moving_link']
 upper=float(manifest['source_joint_limits_rad']['upper']);goal=min(upper,np.pi/2 if kind=='revolute' else .15)
 initial_base=list(source['robot_base_pose']);cal=calibration(job['camera_calibration'])
 template_path=Path(job['diagnostic_template']);template=json.loads(template_path.read_text())
 E0=np.asarray(template['T_world_handle_at_success'])@np.asarray(template['T_handle_TCP'])
 M0=model.asset_T@model.asset.root_to_link(link,{joint:0.})
 moving_body=next(e['rigid_body_path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower())
 B0=np.asarray(next(e['rigid_body_world_transform'] for e in export['shapes'] if e.get('rigid_body_path')==moving_body))
 def delta(s):return model.asset_T@model.asset.root_to_link(link,{joint:float(s)})@np.linalg.inv(M0)
 def target(s,L):return delta(s)@E0@L
 def check(scene,q,base,s,width,hold=True):
  if model.margin(q)<=.05:return False,'LOW_JOINT_MARGIN'
  P=model.poses(q,base,width=width);ok,why=scene.check(P,delta(s)@B0,hold)
  if not ok:return False,why
  return camera_clearance(scene,P,cal)
 # Stations sample the whole swept handle region, not just its initial point.
 policy=job['wrist_experiment']['mobile_scan'];groups=[]
 for s in (0.,goal/2,goal):
  T=target(s,np.eye(4));normal=-T[:3,2]
  groups.append(candidate_bases(initial_base,{'anchor_world_m':T[:3,3].tolist(),'outward_normal_world':normal.tolist()},policy))
 # Interleave actual swept-state pools before truncating; modulo splitting a
 # concatenated list would reconstruct its original, start-biased ordering.
 chosen_bases=[initial_base];seen={tuple(np.round(initial_base,3))}
 for i in range(max(map(len,groups))):
  for g in groups:
   if i<len(g):
    b=g[i];key=tuple(np.round(b,3))
    if key not in seen:seen.add(key);chosen_bases.append(list(b))
 chosen_bases=job.get('diagnostic_frozen_base_candidates',chosen_bases)[:max_bases]
 variants=[]
 for roll,depth,slide in ((0,0,0),(180,0,0),(-5,0,0),(5,0,0),(0,-.004,0),(0,.004,0),(0,0,-.008),(0,0,.008)):
  L=np.eye(4);L[:3,:3]=Rotation.from_euler('z',roll,degrees=True).as_matrix();L[:3,3]=[slide,0,depth];variants.append(L)
 width=float(template['actual_aperture_m']);coarse=np.linspace(0,goal,13);dense=np.linspace(0,goal,91 if kind=='revolute' else 76)
 started=time.time();audits=[];solutions=[];partials=[]
 for bi,base in enumerate(chosen_bases):
  if time.time()-started>wall_s:break
  scene=scene_at(root,copy.deepcopy(export),model,initial_base,base)
  for vi,L in enumerate(variants):
   if vi not in job.get('diagnostic_grasp_variants',list(range(len(variants)))):continue
   if time.time()-started>wall_s:break
   row={'base_index':bi,'base':base,'grasp_variant':vi,'T_handle_template_perturbation':L.tolist(),'checked_states':[]};audits.append(row)
   seed=model.ik(target(0,L),base,seed=model.home,starts=4)
   if seed is None:row['blocker']='START_NO_IK';continue
   path=[];margins=[];blocked=None
   for s in coarse:
    q=model.ik(target(s,L),base,seed=seed,starts=1)
    if q is None:blocked=('NO_CONTINUOUS_IK',float(s));break
    ok,why=check(scene,q,base,s,width)
    row['checked_states'].append({'state':float(s),'margin_rad':model.margin(q),'collision':why})
    if not ok:blocked=(why,float(s));break
    seed=q;path.append(q);margins.append(model.margin(q))
   row['coarse_reached']=float(coarse[len(path)-1]) if path else 0.
   if blocked:
    row.update(blocker=blocked[0],blocker_state=blocked[1]);partials.append(row);continue
   seed=path[0];fine=[];previous_s=0.
   for s in dense:
    q=model.ik(target(s,L),base,seed=seed,starts=1)
    if q is None:blocked=('DENSE_NO_IK',float(s));break
    if np.max(abs(q-seed))>.2:blocked=('DENSE_IK_BRANCH_DISCONTINUITY',float(s));break
    for f in np.linspace(0,1,max(2,int(np.max(abs(q-seed))/.025)+2)):
     middle=(1-f)*seed+f*q;ss=(1-f)*previous_s+f*s;ok,why=check(scene,middle,base,ss,width)
     if not ok:blocked=(why,float(ss));break
    if blocked:break
    fine.append({'state':float(s),'q':q.tolist(),'T_tcp':target(s,L).tolist(),'margin_rad':model.margin(q),'collision':'SAFE'});seed=q;previous_s=float(s)
   if blocked:row.update(blocker=blocked[0],blocker_state=blocked[1]);continue
   # End release/escape is part of feasibility, not a later recovery surprise.
   endpoint=np.asarray(fine[-1]['T_tcp']);retreat=None
   for opening in (min(.1,width+.02),min(.1,width+.04)):
    for direction in (-endpoint[:3,2],-endpoint[:3,2]+np.array([0,0,.5]),np.array([0,0,1.])):
     direction=direction/np.linalg.norm(direction);escape=[];sq=seed.copy()
     for amount in np.linspace(0,.06,25):
      T=endpoint.copy();T[:3,3]+=amount*direction;q=model.ik(T,base,seed=sq,starts=1)
      if q is None:break
      ok,why=check(scene,q,base,goal,opening,False)
      if not ok:break
      escape.append(q.tolist());sq=q
     if len(escape)==25:retreat={'opening_m':opening,'q_path':escape,'direction_world':direction.tolist()};break
    if retreat:break
   if not retreat:row['blocker']='END_NO_RELEASE_AND_ESCAPE';row['blocker_state']=goal;continue
   # Check approach and robot-only free path from normal home before grasp.
   T0=target(0,L);pre=T0.copy();pre[:3,3]-=.04*T0[:3,2]
   pq=model.ik(pre,base,seed=np.asarray(fine[0]['q']),starts=3)
   if pq is None:row['blocker']='PREGRASP_NO_IK';continue
   edge=[];sq=pq;approach_ok=True
   for f in np.linspace(0,1,21):
    T=pre.copy();T[:3,3]=(1-f)*pre[:3,3]+f*T0[:3,3];q=model.ik(T,base,seed=sq,starts=1)
    if q is None:approach_ok=False;break
    ok,why=check(scene,q,base,0,.1,False)
    if not ok:approach_ok=False;break
    edge.append(q.tolist());sq=q
   if not approach_ok:row['blocker']='APPROACH_PATH_INVALID';continue
   outer=model
   class View:
    def __getattr__(self,name):return getattr(outer,name)
    def check(self,q,b):
     ok,why=check(scene,q,b,0,.1,False);return ok,why,None
   free=Model.joint_plan(View(),model.home,pq,base,iterations=1500)
   if free is None:row['blocker']='HOME_TO_PREGRASP_NO_PATH';continue
   minimum=min(x['margin_rad'] for x in fine)
   row.update(status='COMPLETE_TASK_AND_END_ESCAPE',minimum_margin_rad=minimum)
   result={'mode':'KNOWN_MODEL_DIAGNOSTIC','GT_planning':True,'autonomous_unknown_structure_success':False,'object_actuation':False,'attachments':False,'goal_state':goal,'units':'radians' if kind=='revolute' else 'meters','base':base,'T_grasp':T0.tolist(),'grasp_variant':vi,'q_home':model.home.tolist(),'preplan':free,'approach':edge,'path':fine,'end_retreat':retreat,'grasp_aperture_reference_m':width,'minimum_margin_rad':minimum,'score':[goal,float(vi==0),minimum,-float(np.linalg.norm(np.subtract(base[:2],initial_base[:2])))],'grasp_pose_follows_active_part':True}
   solutions.append(result);solutions.sort(key=lambda x:x['score'],reverse=True)
   (output/'ranked_plans.json').write_text(json.dumps(solutions[:3],indent=2))
  (output/'search_progress.json').write_text(json.dumps({'bases_checked':bi+1,'candidates':len(audits),'complete_solutions':len(solutions),'elapsed_wall_s':time.time()-started},indent=2))
 (output/'candidate_audit.json').write_text(json.dumps(audits,indent=2))
 summary={'mode':'KNOWN_MODEL_DIAGNOSTIC','GT_planning':True,'goal_state':goal,'goal_units':'radians' if kind=='revolute' else 'meters','complete_solutions':len(solutions),'candidates':len(audits),'bases_checked':bi+1 if audits else 0,'elapsed_wall_s':time.time()-started,'blockers':[{k:r.get(k) for k in ('base','grasp_variant','coarse_reached','blocker','blocker_state')} for r in sorted(partials,key=lambda r:r['coarse_reached'],reverse=True)[:20]],'single_station_preferred':True,'two_station_required':not bool(solutions)}
 (output/'planning_summary.json').write_text(json.dumps(summary,indent=2));return summary
