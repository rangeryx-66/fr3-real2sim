"""Robot dimensions/names only. Frozen PiPER model and Pink costs stay unchanged."""
import json,copy,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask,PostureTask
from scipy.spatial.transform import Rotation
from piper_mobile_demo.model import Model
from articulated_demo.kinematics import transform
from piper_mobile_demo.owned_scene import Shape

class RobotModel(Model):
 def __init__(self,config,asset_root,source):
  super().__init__(config['urdf'],asset_root,source)
  self.names=config['arm_joint_names'];root=ET.parse(self.urdf).getroot();j={x.get('name'):x for x in root.findall('joint')}
  self.limits=np.array([[float(j[n].find('limit').get(k)) for k in ['lower','upper']] for n in self.names]);self.home=np.asarray(config['home_q']);self.config=config

class RobotPinkIK:
 def __init__(self,config,fixed=None):
  self.config=config;self.names=config['arm_joint_names'];self.fixed=fixed or {};full=pin.buildModelFromUrdf(config['urdf']);neutral=pin.neutral(full)
  for name,value in self.fixed.items():neutral[full.joints[full.getJointId(name)].idx_q]=value
  locked=[i for i in range(1,full.njoints) if full.names[i] not in self.names or full.names[i] in self.fixed]
  self.robot=pin.buildReducedModel(full,locked,neutral);self.variable=[n for n in self.names if n not in self.fixed];self.indices=[self.names.index(n) for n in self.variable]
  self.original_limits=np.column_stack((self.robot.lowerPositionLimit,self.robot.upperPositionLimit));self.robot.lowerPositionLimit+=.05;self.robot.upperPositionLimit-=.05;self.robot.velocityLimit/=3
  self.home=np.asarray(config['home_q'])[self.indices];self.task=FrameTask('tcp_link',position_cost=1.,orientation_cost=.1,lm_damping=1e-8,gain=.75);self.posture=PostureTask(cost=1e-7);self.rng=np.random.default_rng(7320);self.calls=0;self.last_error=None
 def expand(self,q):
  full=np.asarray(self.config['home_q'],float).copy();full[self.indices]=q
  for n,v in self.fixed.items():full[self.names.index(n)]=v
  return full
 def solve(self,target,base,seed=None,starts=1):
  self.calls+=1;B=transform(base[:3],[0,0,np.deg2rad(base[3])]);goal=np.linalg.inv(B)@target;self.task.set_target(pin.SE3(goal[:3,:3],goal[:3,3]))
  seeds=[self.home if seed is None else np.asarray(seed)[self.indices]];seeds+=list(self.rng.uniform(self.robot.lowerPositionLimit,self.robot.upperPositionLimit,size=(max(0,starts-1),self.robot.nq)))
  for initial in seeds:
   q=np.clip(initial,self.robot.lowerPositionLimit,self.robot.upperPositionLimit);conf=pink.Configuration(self.robot,self.robot.createData(),q);self.posture.set_target(q)
   for _ in range(65):
    current=conf.get_transform_frame_to_world('tcp_link');pe=float(np.linalg.norm(current.translation-goal[:3,3]));re=float(Rotation.from_matrix(current.rotation@goal[:3,:3].T).magnitude())
    if pe<1e-7 and re<1e-6:break
    try:v=pink.solve_ik(conf,[self.task,self.posture],.04,solver='daqp',safety_break=False)
    except Exception as error:self.last_error=repr(error);break
    conf.integrate_inplace(v,.04)
   current=conf.get_transform_frame_to_world('tcp_link');pe=float(np.linalg.norm(current.translation-goal[:3,3]));re=float(Rotation.from_matrix(current.rotation@goal[:3,:3].T).magnitude())
   if pe<.0008 and re<.004:return self.expand(conf.q)
  return None
 def velocity_step(self,target,base,q,dt):
  self.calls+=1;B=transform(base[:3],[0,0,np.deg2rad(base[3])]);goal=np.linalg.inv(B)@target;self.task.set_target(pin.SE3(goal[:3,:3],goal[:3,3]));q=np.asarray(q)[self.indices];self.posture.set_target(q);conf=pink.Configuration(self.robot,self.robot.createData(),q)
  try:v=pink.solve_ik(conf,[self.task,self.posture],dt,solver='daqp',safety_break=False);conf.integrate_inplace(v,dt);return self.expand(conf.q)
  except Exception as error:self.last_error=repr(error);return None

def scene_factory(root,export,model,allowed):
 from interactive_twin_recovery.mobile import physical_scene_class
 scene=physical_scene_class(root)(export,model,allowed)
 if model.config['robot']=='piper':return scene
 # Keep exact native frozen hand and object solids; foreign arm hulls derive
 # from unchanged official URDF collision meshes. This is an offline proxy,
 # not falsely described as measured native robot cooking.
 fingers={'gripper_link1','gripper_link2','gripper_base','tcp_link','flange_link'}
 scene.robot=[s for s in scene.robot if s.body in fingers and s.body in model.chain.joints]
 # A foreign flange may have different geometry even when the name matches.
 scene.robot=[s for s in scene.robot if s.body in {'gripper_link1','gripper_link2','gripper_base'}]
 for link,mesh in model.robot_hulls.items():
  if link in {'gripper_link1','gripper_link2','gripper_base'}:continue
  scene.robot.append(Shape(mesh,np.eye(4),True,'/World/Piper/official_arm_proxy/'+link,link))
 return scene
