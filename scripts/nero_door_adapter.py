"""NERO-only model adapter; verified proxy pairs retain raw triangle guards."""
from pathlib import Path
import json,numpy as np,xml.etree.ElementTree as ET
from cross_robot_adapter import RobotModel,RobotPinkIK
from piper_mobile_demo.model import geometry
from piper_mobile_demo.owned_scene import Shape,intersects
from interactive_twin_recovery.mobile import physical_scene_class
ROOT=Path(__file__).resolve().parents[1]

class NeroModel(RobotModel):
 def __init__(self,config,asset_root,source):
  super().__init__(config,asset_root,source)
  self.corrected_pairs=[tuple(x['pair']) for x in config['verified_collision_semantics']['corrected_pairs']]
  self.allowed.update(frozenset(x) for x in self.corrected_pairs)
  tree=ET.parse(self.urdf).getroot();self.raw_pair_meshes={n:geometry(tree.find(f"link[@name='{n}']/collision"),self.urdf.parent) for pair in self.corrected_pairs for n in pair}

class RawCheckedScene:
 def __init__(self,scene,model):
  self.__dict__.update(scene.__dict__);self.original=scene;self.raw={n:Shape(m,np.eye(4),False,'official_raw_'+n,n) for n,m in model.raw_pair_meshes.items()};self.pairs=model.corrected_pairs
 def check(self,poses,moving_pose,allow_handle):
  ok,why=self.original.check(poses,moving_pose,allow_handle)
  if not ok:return ok,why
  for a,b in self.pairs:
   self.raw[a].place(poses[a]);self.raw[b].place(poses[b])
   if intersects(self.raw[a],self.raw[b]):return False,'ACTUAL_NERO_SELF_COLLISION:'+a+':'+b
  return True,'SAFE'
 def contact_guard(self,contacts,poses):return self.original.contact_guard(contacts,poses)

def scene_factory(root,export,model,allowed,native_runtime=False):
 sc=physical_scene_class(root)(export,model,allowed)
 if not native_runtime:
  sc.robot=[Shape(m,np.eye(4),True,'/World/Piper/nero_official/'+n,n) for n,m in model.robot_hulls.items()]
  sc.jaw_bounds={n:np.array([s.vertices.min(0),s.vertices.max(0)]) for s in sc.robot for n in [s.body] if n in ('gripper_link1','gripper_link2')}
 return RawCheckedScene(sc,model)


def configuration():
 out=ROOT/'results/nero_feasibility_20261012';c=json.loads((out/'NERO_MODEL.json').read_text());c['verified_collision_semantics']=json.loads((out/'VERIFIED_COLLISION_SEMANTICS.json').read_text());return c
