"""One central bar-side pinch. Only equivalent finger swap; no pose search."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model
from articulated_interaction.semantic_collision import SemanticScene

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for n in ['source','asset-root','native-export','association','output']:p.add_argument('--'+n,type=Path,required=True)
 p.add_argument('--reference-plan',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 source=json.loads((a.source/'report.json').read_text());base=source['robot_base_pose'];model=Model(ROOT/'config/piper.urdf',a.asset_root,source);s=model.manifest['interaction_geometry']['selection'];owner=SemanticScene(ROOT,a.native_export,ROOT/'config/piper_contact_ownership.json',json.loads(a.association.read_text())['allowed_pad_targets'],s['moving_link'])
 def safe(q):return owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False)[0]
 def edge(x,y):return all(safe(z) for z in np.linspace(x,y,max(2,int(np.ceil(np.max(np.abs(np.array(y)-x))/.025))+1)))
 class PlanningView:
  def __getattr__(self,n):return getattr(model,n)
  def check(self,q,base):
   ok,why=owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False);return ok,why,None
 def joint_plan(x,y):return Model.joint_plan(PlanningView(),x,y,base)
 reference=None
 if a.reference_plan:
  d=json.loads(a.reference_plan.read_text());ref=max(d['trial_candidates'],key=lambda z:z['minimum_joint_margin_rad'])
  if ref['base']==base and all(edge(x,y) for x,y in zip(ref['preplan'],ref['preplan'][1:])):reference=ref['preplan']
 poses=model.poses(model.home,base,width=.04);offset=np.mean([(np.linalg.inv(poses['tcp_link'])@poses[n]@np.r_[np.asarray(owner.manifest['fingers'][n]['pad_vertices']).mean(0),1])[:3] for n in ('gripper_link1','gripper_link2')],axis=0);axis=model.asset_T[:3,:3]@np.asarray(s['axis_root']);normal=model.asset_T[:3,:3]@np.asarray(s['outward_normal_root']);R=np.column_stack((axis,np.cross(-normal,axis),-normal));section=min(s['sections'],key=lambda z:abs(z['fraction']));anchor=(model.asset_T@np.r_[section['anchor_root_m'],1])[:3];rows=[];trials=[]
 for flip in [0,180]:
  T=np.eye(4);T[:3,:3]=R@Rotation.from_euler('z',flip,degrees=True).as_matrix();T[:3,3]=anchor-T[:3,:3]@offset;row={'variant':f'nominal_swap_{flip}','family':'bar-side-pinch','base':base,'T':T.tolist(),'status':'NO_IK'};q=model.ik(T,base,starts=5)
  if q is not None:
   ok,why=owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False);row['status']=why
   if ok:
    pre=T.copy();pre[:3,3]-=.04*T[:3,2];current=model.ik(pre,base,q,starts=3);approach=[];row['status']='NO_PREGRASP_IK'
    if current is not None:
     for f in np.linspace(0,1,21):
      target=T.copy();target[:3,3]=pre[:3,3]*(1-f)+T[:3,3]*f;current=model.ik(target,base,current,starts=1)
      if current is None:row['status']='NO_APPROACH_IK';break
      ok,why=owner.check_robot(model,current,base,finger_q=[.05,-.05],allow_pad=False)
      if not ok:row['status']='APPROACH_'+why;break
      approach.append(current.tolist())
     else:
      local=joint_plan(np.asarray(reference[-1]) if reference else model.home,np.asarray(approach[0]));path=(reference+local[1:]) if reference and local is not None else local;row['status']='NO_HOME_PLAN'
      if path is not None and all(edge(x,y) for x,y in zip(path,path[1:])):
       shortcut=[path[0]];i=0
       while i<len(path)-1:
        for j in range(len(path)-1,i,-1):
         if edge(path[i],path[j]):break
        shortcut.append(path[j]);i=j
       row.update(status='PENDING_REAL_CLOSURE',q_pre=approach[0],q_grasp=current.tolist(),approach=approach,preplan=shortcut,minimum_joint_margin_rad=min(model.margin(np.asarray(z)) for z in approach));trials.append(row)
  rows.append(row)
  if trials:break
 result={'base':base,'rows':rows,'trial_candidates':trials,'best':trials[0] if trials else None,'planning_complete':True,'pose_search':False,'family':'bar-side-pinch nominal; symmetric finger swap only','raw_intersections':'diagnostic only','joint_margin_minimum_rad':.05};(a.output/'plan.json').write_text(json.dumps(result,indent=2));print(json.dumps({'planned':len(trials),'rows':[r['status'] for r in rows]}))
if __name__=='__main__':main()
