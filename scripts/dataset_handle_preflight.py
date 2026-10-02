"""Independent fixed-base trials for geometry-ranked existing dataset handles.

No object-ID rules. The manifest wrapper adds link metadata only and symlinks
unchanged prepared geometry. Aperture estimates never grant grasp acceptance.
"""
import argparse,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model,geometry
from piper_mobile_demo.owned_scene import OwnedFingerScene
from articulated_demo.kinematics import URDFChain

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ranking',type=Path,required=True);p.add_argument('--fixed-base-report',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--native-export',type=Path);p.add_argument('--association',type=Path);p.add_argument('--diagnose-rejected',action='store_true',help='If screening has no qualified asset, explicitly diagnose a rejected visual bar; never certify it');p.add_argument('--placement',type=float,nargs=3,default=[.41,0,-90]);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 ranking=json.loads(a.ranking.read_text());base=json.loads(a.fixed_base_report.read_text())['base_final']
 if not a.native_export:
  # Rank a NEW visual bar for diagnostic preflight; do not call it graspable.
  bars=[r for r in ranking['ranking'] if r['sections'] and 'NOT_A_PROTRUDING_BAR' not in r['screen_rejections'] and r['minimum_collision_rear_gap_m']<0 and min(s['visual_rear_gap_m'] for s in r['sections'])>.005 and r['dimensions_m'][0]>=.02543]
  qualified=[r for r in ranking['ranking'] if not r['screen_rejections']]
  if qualified:selected=qualified[0]
  elif a.diagnose_rejected and bars:
   bars.sort(key=lambda r:r['minimum_collision_rear_gap_m']);selected=bars[0]
  else:raise RuntimeError('NO_SCREEN_QUALIFIED_ASSET: no trial automatically approved; use --diagnose-rejected for an explicit negative control')
  
  if selected['joint_type']!='revolute':raise RuntimeError('PRISMATIC_REQUIRES_LINEAR_EXECUTION_AND_BLIND_PROBE; do not run degree-based door controller')
  original=Path(selected['asset_root']);wrapper=a.output/'asset';wrapper.mkdir(exist_ok=True)
  for path in original.iterdir():
   if path.name=='manifest.json':continue
   dest=wrapper/path.name
   if not dest.exists():dest.symlink_to(path.resolve(),target_is_directory=path.is_dir())
  meta=json.loads((original/'manifest.json').read_text());meta.update(moving_link=selected['moving_link'],door_link=selected['handle_link']);(wrapper/'manifest.json').write_text(json.dumps(meta,indent=2))
  source={'robot_base_pose':base,'asset_installation':dict(zip(('x_m','y_m','yaw_deg'),a.placement))|{'fixture_height_m':.04},'selection':selected,'selection_status':'PROVISIONAL_DIAGNOSTIC_NOT_GRASPABLE','no_geometry_or_physics_changes':True};(a.output/'report.json').write_text(json.dumps(source,indent=2))
  model=Model(ROOT/'config/piper.urdf',wrapper,source);root=ET.parse(model.asset_urdf).getroot();pieces=[]
  for link in root.findall('link'):
   for visual in link.findall('visual'):
    spec=visual.find('geometry/mesh')
    if spec is not None and Path(spec.get('filename')).stem==selected['mesh']:
     mesh=geometry(visual,model.asset_urdf.parent);mesh.apply_transform(model.asset_T@model.asset.root_to_link(link.get('name'),{}));pieces.append(mesh)
  import trimesh
  np.random.seed(20261002);points=trimesh.util.concatenate(pieces).sample(4000);np.save(a.output/'target_points.npy',points)
  (a.output/'plan.json').write_text(json.dumps({'mode':'fixed','rows':[],'best':None}))
  print('PROVISIONAL',selected['asset_id'],selected['mesh']);return
 source=json.loads((a.output/'report.json').read_text());model=Model(ROOT/'config/piper.urdf',a.output/'asset',source);s=source['selection'];owner=OwnedFingerScene(ROOT,a.native_export,ROOT/'config/piper_contact_ownership.json',json.loads(a.association.read_text())['allowed_pad_targets'],s['moving_link'])
 poses=model.poses(model.home,base,width=.04);centres=[]
 for n in ('gripper_link1','gripper_link2'):
  points=np.asarray(owner.manifest['fingers'][n]['pad_vertices']);centres.append((np.linalg.inv(poses['tcp_link'])@poses[n]@np.r_[points.mean(0),1])[:3])
 pad=np.mean(centres,axis=0);axis=model.asset_T[:3,:3]@np.asarray(s['axis_root']);normal=model.asset_T[:3,:3]@np.asarray(s['outward_normal_root']);R=np.column_stack((axis,np.cross(-normal,axis),-normal));rows=[];trials=[]
 for section in s['sections']:
  anchor=(model.asset_T@np.r_[section['anchor_root_m'],1])[:3]
  for roll,tilt_x,tilt_y in __import__('itertools').product((0,180,90,-90),(-15,0,15),(-15,0,15)):
   for depth in (-.003,0,.003):
    T=np.eye(4);T[:3,:3]=R@Rotation.from_euler('xyz',[tilt_x,tilt_y,roll],degrees=True).as_matrix();T[:3,3]=anchor+normal*depth-T[:3,:3]@pad
    row={'variant':f'dataset_{len(rows):03d}','T':T.tolist(),'base':base,'raw_rank':None,'offset_handle_m':[0,0,depth],'rpy_handle_deg':[tilt_x,tilt_y,roll],'status':'NO_IK'};q=model.ik(T,base,starts=5)
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
        plan=model.joint_plan(model.home,np.array(approach[0]),base)
        if plan is None:row['status']='NO_PREGRASP_PLAN'
        elif not all(owner.check_robot(model,z,base,finger_q=[.05,-.05],allow_pad=False)[0] for x,y in zip(plan,plan[1:]) for z in np.linspace(x,y,max(2,int(np.ceil(np.max(np.abs(np.array(y)-x))/.025))+1))):row['status']='OWNED_PREGRASP_COLLISION'
        else:
         shortcut=[plan[0]];i=0
         while i<len(plan)-1:
          for j in range(len(plan)-1,i,-1):
           if all(owner.check_robot(model,z,base,finger_q=[.05,-.05],allow_pad=False)[0] for z in np.linspace(plan[i],plan[j],max(2,int(np.ceil(np.max(np.abs(np.array(plan[j])-plan[i]))/.025))+1))):break
          shortcut.append(plan[j]);i=j
         row.update(status='PENDING_ACTUAL_CLOSURE',q_pre=approach[0],q_grasp=current.tolist(),approach=approach,preplan=shortcut,minimum_joint_margin_rad=min(model.margin(np.array(q)) for q in approach));trials.append(row)
    rows.append(row);print(row['variant'],row['status'],flush=True)
    (a.output/'search.json').write_text(json.dumps({'mode':'fixed','rows':rows,'trial_candidates':trials,'best':trials[0] if trials else None,'planning_complete':False},indent=2))
 (a.output/'search.json').write_text(json.dumps({'mode':'fixed','rows':rows,'trial_candidates':trials,'best':trials[0] if trials else None,'planning_complete':True},indent=2))
if __name__=='__main__':main()
