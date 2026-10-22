"""Bar-side pinch family preflight; actual closure remains an execution gate."""
import argparse,json,sys,itertools,time
from pathlib import Path
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model
from articulated_interaction.collision import InteractionScene

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for n in ['source','asset-root','native-export','association','calibration','output']:p.add_argument('--'+n,type=Path,required=True)
 p.add_argument('--semantic-proxy',action='store_true');p.add_argument('--nominal-only',action='store_true');p.add_argument('--roll-deg',type=float,nargs='+',default=[0,-10,10]);p.add_argument('--depth-mm',type=float,nargs='+',default=[0,-1.5,1.5]);p.add_argument('--max-trials',type=int,default=6);p.add_argument('--reference-plan',type=Path,help='Reuse only after rechecking every edge against this scene');p.add_argument('--complete-existing-plan',type=Path);p.add_argument('--family',choices=['bar-side-pinch','bar-front-back-pinch'],default='bar-side-pinch');p.add_argument('--local-bar-tangent',action='store_true',help='Use observed bar-centre samples to follow curved handle geometry');a=p.parse_args();
 if max(map(abs,a.roll_deg))>20 or max(map(abs,a.depth_mm))>5:raise ValueError('LOCAL_FAMILY_BOUNDS: roll <=20deg, depth <=5mm')
 a.output.mkdir(parents=True,exist_ok=True)
 source=json.loads((a.source/'report.json').read_text());base=source['robot_base_pose'];model=Model(ROOT/'config/piper.urdf',a.asset_root,source);s=model.manifest['interaction_geometry']['selection'];cal=json.loads(a.calibration.read_text());scene_type=InteractionScene
 if a.semantic_proxy:
  from articulated_interaction.semantic_collision import SemanticScene
  scene_type=SemanticScene
 owner=scene_type(ROOT,a.native_export,ROOT/'config/piper_contact_ownership.json',json.loads(a.association.read_text())['allowed_pad_targets'],s['moving_link'],calibration=cal)
 if s['dimensions_m'][0]/s['dimensions_m'][1]<1.6:raise RuntimeError('NO_SUPPORTED_BAR_FAMILY: do not substitute blind SE3 search')
 poses=model.poses(model.home,base,width=.04);pad=np.mean([(np.linalg.inv(poses['tcp_link'])@poses[n]@np.r_[np.asarray(owner.manifest['fingers'][n]['pad_vertices']).mean(0),1])[:3] for n in ('gripper_link1','gripper_link2')],axis=0)
 axis=model.asset_T[:3,:3]@np.asarray(s['axis_root']);normal=model.asset_T[:3,:3]@np.asarray(s['outward_normal_root']);R=np.column_stack((axis,np.cross(-normal,axis),-normal));rows=[];trials=[]
 def safe(q):return owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False)[0]
 def edge(x,y):return all(safe(z) for z in np.linspace(x,y,max(2,int(np.ceil(np.max(np.abs(np.array(y)-x))/.025))+1)))
 class PlanningView:
  def __getattr__(self,n):return getattr(model,n)
  def check(self,q,base):
   ok,why=owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False);return ok,why,None
 def joint_plan(start,goal):return Model.joint_plan(PlanningView(),start,goal,base)
 reference=None
 if a.reference_plan:
  record=json.loads(a.reference_plan.read_text());r=max(record['trial_candidates'],key=lambda z:z['minimum_joint_margin_rad']) if record.get('trial_candidates') else record['best'];candidate=r['preplan']
  if r['base']==base and all(edge(x,y) for x,y in zip(candidate,candidate[1:])):reference=candidate
 def save(done=False):
  (a.output/'plan.json').write_text(json.dumps({'mode':'fixed','base':base,'family':a.family,'rows':rows,'trial_candidates':trials,'best':trials[0] if trials else None,'planning_complete':done,'counts':dict(Counter(r['status'] for r in rows)),'closure':'native bilateral pad preload required; no estimated aperture endpoint','calibration':cal},indent=2))
 if a.complete_existing_plan:
  old=json.loads(a.complete_existing_plan.read_text());assert old['base']==base and all(old['calibration'][k]==cal[k] for k in ['ownership_sha256','penetration_tolerance_m']);rows=old['rows'];eligible=sorted([r for r in rows if r.get('q_grasp')],key=lambda r:-r['minimum_joint_margin_rad'])
  for r in eligible:
   if len(trials)>=a.max_trials:break
   if not all(safe(q) for q in r['approach']):r['status']='RECHECK_APPROACH_COLLISION';continue
   if r.get('preplan') and all(edge(x,y) for x,y in zip(r['preplan'],r['preplan'][1:])):trials.append(r);continue
   references=sorted(old['trial_candidates'],key=lambda z:np.linalg.norm(np.asarray(z['q_pre'])-r['q_pre']))
   path=None
   for ref in references:
    if not all(edge(x,y) for x,y in zip(ref['preplan'],ref['preplan'][1:])):continue
    local=joint_plan(np.asarray(ref['preplan'][-1]),np.asarray(r['q_pre']))
    if local is not None and all(edge(x,y) for x,y in zip(local,local[1:])):path=ref['preplan']+local[1:];break
   if path is not None:r.update(preplan=path,status='PENDING_REAL_CONTACT_CLOSURE');trials.append(r)
   else:r['status']='NO_RECHECKED_HOME_PLAN'
   save();print(r['variant'],r['status'],flush=True)
  save(True);return
 sections=sorted(s['sections'],key=lambda x:-x['visual_rear_gap_m'])
 if a.nominal_only:sections=[min(sections,key=lambda x:abs(x['fraction']))];a.roll_deg=[0];a.depth_mm=[0]
 for section in sections:
  anchor=(model.asset_T@np.r_[section['anchor_root_m'],1])[:3]
  localR=R.copy()
  if a.local_bar_tangent:
   anchors=np.array([(model.asset_T@np.r_[x['anchor_root_m'],1])[:3] for x in s['sections']]);coordinate=anchors@axis;degree=min(2,len(anchors)-1);fit=np.polyfit(coordinate,anchors,degree);derivative=np.array([np.polyval(np.polyder(fit[:,i]),anchor@axis) for i in range(3)]);derivative/=np.linalg.norm(derivative);n=normal-derivative*(normal@derivative);n/=np.linalg.norm(n);localR=np.column_stack((derivative,np.cross(-n,derivative),-n))
  if a.family=='bar-front-back-pinch' and section['visual_rear_gap_m']<section['bar_depth_m']:continue
  for flip,roll,depth in itertools.product((0,180),a.roll_deg,[x/1000 for x in a.depth_mm]):
   family_roll=roll+(90 if a.family=='bar-front-back-pinch' else 0)
   T=np.eye(4);T[:3,:3]=localR@Rotation.from_euler('xz',[family_roll,flip],degrees=True).as_matrix();T[:3,3]=anchor-T[:3,2]*depth-T[:3,:3]@pad
   row={'variant':f'bar_{len(rows):03d}','family':a.family,'T':T.tolist(),'base':base,'section_fraction':section['fraction'],'depth_delta_m':depth,'roll_about_bar_deg':roll,'finger_swap_deg':flip,'status':'NO_IK'};q=model.ik(T,base,starts=5)
   if q is not None:
    row['exact_ik_margin_rad']=model.margin(q);ok,why=owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False);row['status']=why
    if ok:
     pre=T.copy();pre[:3,3]-=.04*T[:3,2];current=model.ik(pre,base,q,starts=3);approach=[]
     if current is None:row['status']='NO_PREGRASP_IK'
     else:
      for f in np.linspace(0,1,21):
       target=T.copy();target[:3,3]=pre[:3,3]*(1-f)+T[:3,3]*f;current=model.ik(target,base,current,starts=1)
       if current is None:row['status']='NO_APPROACH_IK';break
       ok,why=owner.check_robot(model,current,base,finger_q=[.05,-.05],allow_pad=False)
       if not ok:row['status']='APPROACH_'+why;break
       approach.append(current.tolist())
      else:
       row.update(status='APPROACH_READY_PENDING_HOME_PLAN',q_pre=approach[0],q_grasp=current.tolist(),approach=approach,minimum_joint_margin_rad=min(model.margin(np.array(q)) for q in approach))
       if len(trials)<a.max_trials:
        if reference is not None:
         local=joint_plan(np.array(reference[-1]),np.array(approach[0]));plan=reference+local[1:] if local is not None else None
        else:plan=joint_plan(model.home,np.array(approach[0]))
        if plan is None:row['status']='NO_HOME_PLAN'
        elif not all(edge(x,y) for x,y in zip(plan,plan[1:])):row['status']='HOME_NATIVE_COLLISION'
        else:
         shortcut=[plan[0]];i=0
         while i<len(plan)-1:
          for j in range(len(plan)-1,i,-1):
           if edge(plan[i],plan[j]):break
          shortcut.append(plan[j]);i=j
         row.update(status='PENDING_REAL_CONTACT_CLOSURE',preplan=shortcut);trials.append(row)
   rows.append(row);save();print(row['variant'],row['status'],len(trials),flush=True)
 save(True)
if __name__=='__main__':main()
