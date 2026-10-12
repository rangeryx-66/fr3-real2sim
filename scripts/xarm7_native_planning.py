"""Native hand geometry adapter to the frozen shared door planner.

Logical pad frames are coordinates only. All solids are unchanged official
xArm meshes; no PiPER solid, success pose or template enters planning.
"""
import copy, json, inspect, time
from pathlib import Path
import numpy as np
import fcl
from cross_robot_adapter import RobotModel,RobotPinkIK
from xarm7_native_gripper import configuration,NativeGripperGeometry,FINGER_BODIES,GRIPPER_JOINTS,ROOT,OUT
from articulated_demo.kinematics import transform
from piper_mobile_demo.model import fcl_geometry
from interactive_twin_recovery.mobile import physical_scene_class
import plan_shared_door_skill as shared

LOGICAL=['gripper_link1','gripper_link2']
PAD_FRAMES={
 'left_finger':np.array([[-1.,0,0,0],[0,0,1.,0],[0,1.,0,0],[0,0,0,1.]]),
 'right_finger':np.array([[1.,0,0,0],[0,0,-1.,0],[0,1.,0,0],[0,0,0,1.]])}

class NativeModel(RobotModel):
 def __init__(self,ignored,asset,source):
  self.native=NativeGripperGeometry();super().__init__(configuration(),asset,source)
  self.physical_hulls=self.robot_hulls.copy();self.physical_robot=self.robot.copy()
  self.robot=[];self.robot_hulls={};self.pads={};self.pad_triangles={}
  alias=dict(zip(FINGER_BODIES,LOGICAL))
  for n,h in self.physical_hulls.items():
   h=h.copy()
   if n in alias:
    h.apply_transform(np.linalg.inv(PAD_FRAMES[n]));logical=alias[n]
    raw=self.native.meshes[n].copy();raw.apply_transform(np.linalg.inv(PAD_FRAMES[n]));inner=raw.vertices[:,2].min();faces=np.max(abs(raw.triangles[:,:,2]-inner),axis=1)<1e-6
    pts=raw.triangles[faces].reshape(-1,3)
    if len(pts)<3:raise ValueError('OFFICIAL_NATIVE_INNER_CONTACT_SURFACE_NOT_FOUND')
    self.pads[logical]=np.array([pts.min(0),pts.max(0)]);self.pad_triangles[logical]=raw.triangles[faces]
   else:logical=n
   self.robot.append((logical,fcl_geometry(h)));self.robot_hulls[logical]=h
  self.allowed={frozenset(alias.get(n,n) for n in p) for p in self.allowed if p!=frozenset(LOGICAL)}
  for side,logical in zip(['left','right'],LOGICAL):
   self.allowed.add(frozenset((logical,side+'_inner_knuckle')))
   self.allowed.add(frozenset((side+'_inner_knuckle',side+'_outer_knuckle')))
 def poses(self,q,base,width=.084,finger_q=None):
  values=dict(zip(self.names,q));a=self.native.angle(width);values.update(self.native.values(a))
  if finger_q is not None:
   values.update(dict(zip(GRIPPER_JOINTS,np.asarray(finger_q))))
  B=transform(base[:3],[0,0,np.deg2rad(base[3])]);P={n:B@self.chain.root_to_link(n,values) for n in self.physical_hulls}
  P['tcp_link']=B@self.chain.root_to_link('tcp_link',values)
  for n,logical in zip(FINGER_BODIES,LOGICAL):P[logical]=P[n]@PAD_FRAMES[n]
  return P

def scene_factory(root,export,model,allowed):
 """Use the exact shared collision/contact rules with native transformed solids."""
 e=copy.deepcopy(export);e['shapes']=[x for x in e['shapes'] if not x['path'].startswith('/World/Piper')]
 P=model.poses(model.home,[0,0,0,0])
 for n,h in model.robot_hulls.items():
  e['shapes'].append({'path':'/World/Piper/native/'+n+'/collision','type':'Mesh','rigid_body_path':'/World/Piper/native/'+n,'world_transform':P[n].tolist(),'rigid_body_world_transform':P[n].tolist(),'convexes':[{'vertices':h.vertices.tolist(),'polygons':h.faces.tolist()}]})
 return physical_scene_class(root)(e,model,allowed)

def common_quality(model,surfaces,T,base,tangent):
 """Same pose and actual nonlinear aperture for both native finger patches."""
 from handle_contact_quality import ray_surface
 tangent=np.asarray(tangent);tangent/=np.linalg.norm(tangent);reports={};samples=[]
 for width in np.linspace(.084,.001641,35):
  P=model.poses(model.home,base,width=float(width));D=T@np.linalg.inv(P['tcp_link']);P={n:D@X for n,X in P.items()};rr={}
  for label,meshes in [('visual',[surfaces[0]]),('native',surfaces[1])]:
   sides=[];normals=[]
   for n in LOGICAL:
    b=model.pads[n];xy=np.array([[x,y] for x in np.linspace(*b[:,0],9) for y in np.linspace(*b[:,1],7)]);grid=np.c_[xy,np.full(len(xy),b[0,2])];origins=(np.c_[grid,np.ones(len(grid))]@P[n].T)[:,:3];direction=-P[n][:3,2]
    rays=[ray_surface(m,origins,direction) for m in meshes];dist=np.array([r[0] for r in rays]);which=dist.argmin(0);distance=dist[which,np.arange(len(grid))];normal=np.array([r[1] for r in rays])[which,np.arange(len(grid))];hit=np.isfinite(distance)
    interior=np.prod(np.sin(np.pi*(xy-b[0,:2])/(b[1,:2]-b[0,:2])),axis=1);band=(b[1,1]-b[0,1])/20;weights=np.where(hit,np.exp(-np.minimum(distance/band,30)**2),0)*interior;alignment=np.clip(-normal@direction,0,1);support=float(np.mean(weights*alignment**2));mean=(weights[:,None]*normal).sum(0)/max(weights.sum(),1e-30);mean/=max(np.linalg.norm(mean),1e-30);normals.append(mean)
    sides.append({'finger':n,'support_score':support,'interior_supported_area_m2':float(np.prod(b[1,:2]-b[0,:2])*np.mean(weights))})
   opposition=float(np.clip((1-normals[0]@normals[1])/2,0,1));normal_pull=max(0.,-normals[0]@tangent,-normals[1]@tangent);c=P[LOGICAL[0]][:3,2];transmission=(normal_pull+np.sqrt(max(0.,1-(c@tangent)**2)))/np.sqrt(2)
   rr[label]={'common_aperture_m':float(width),'bilateral_score':min(s['support_score'] for s in sides)*opposition*transmission,'sides':sides}
  samples.append(rr)
 best=max(samples,key=lambda s:min(s[k]['bilateral_score'] for k in ['visual','native']))
 return {k:{'best_common_pose_aperture':best[k],'samples':[s[k] for s in samples]} for k in ['visual','native']}|{'score':min(best[k]['bilateral_score'] for k in best),'classification':'NATIVE_GEOMETRY_RANK_ONLY','same_aperture_across_visual_native_and_both_fingers':True}

def plan(job,output,wall_s=300):
 text=Path(shared.__file__).read_text().split("if __name__=='__main__':",1)[0]
 text=text.replace('np.linspace(.1,0.,41)','np.linspace(.084,.001641,41)').replace('min(.1,float(selection', 'min(.084,float(selection').replace('width=.1','width=.084').replace('np.clip(quality[\'patch\'][\'native\'][\'best_common_pose_aperture\'][\'common_aperture_m\'],0,.1)','np.clip(quality[\'patch\'][\'native\'][\'best_common_pose_aperture\'][\'common_aperture_m\'],.001641,.084)')
 old="flip=q.copy();flip[3]+=np.pi if q[3]<0 else -np.pi;flip[4]*=-1;flip[5]+=np.pi if q[5]<0 else -np.pi\n            qf=model.ik(g['T'],base,flip)"
 text=text.replace(old,"qf=model.ik(g['T'],base,model.rng.uniform(model.limits[:,0]+.05,model.limits[:,1]-.05))")
 ns=dict(shared.__dict__);exec(compile(text,str(shared.__file__)+'::native_geometry_adapter','exec'),ns)
 ns['Model']=NativeModel;ns['PinkIK']=lambda ignored,home:RobotPinkIK(configuration());ns['physical_scene_class']=lambda root:lambda export,model,allowed:scene_factory(root,export,model,allowed);ns['common_contact_quality']=common_quality
 return ns['plan'](job,output,wall_s=wall_s)
