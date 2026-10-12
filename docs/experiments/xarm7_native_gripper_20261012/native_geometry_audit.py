import json,xml.etree.ElementTree as ET
import numpy as np,pinocchio as pin,fcl
from scipy.spatial.transform import Rotation
from xarm7_native_gripper import *
from piper_mobile_demo.model import geometry,fcl_geometry,obj
from cross_robot_adapter import RobotPinkIK
c=configuration();g=NativeGripperGeometry(c);root=ET.parse(c['sim_urdf']).getroot();rows=[]
pairs=[(s+'_finger',s+'_inner_knuckle') for s in ['left','right']]+[(s+'_inner_knuckle',s+'_outer_knuckle') for s in ['left','right']]
shapes={n:fcl_geometry(geometry(root.find("link[@name='%s']/collision"%n),Path(c['sim_urdf']).parent).convex_hull) for n in set(sum(([a,b] for a,b in pairs),[]))}
for a in [0,.1,.3,.5,.7,.85]:
 P={n:g.chain.root_to_link(n,g.values(a)) for n in shapes}
 for u,v in pairs:
  result=fcl.CollisionResult();count=fcl.collide(obj(shapes[u],P[u]),obj(shapes[v],P[v]),fcl.CollisionRequest(num_max_contacts=32,enable_contact=True),result)
  rows.append({'drive_rad':a,'pair':[u,v],'overlap':bool(count),'maximum_penetration_m':max((float(x.penetration_depth) for x in result.contacts),default=0),'source':'official four-bar geometry and official SRDF, internal joint semantics only'})
ik=RobotPinkIK(c);data=ik.robot.createData();frame=ik.robot.getFrameId('tcp_link');checks=[]
for asset in ['45134','45443','45249']:
 d=OUT/asset;p=d/'planning/candidate.json'
 if not p.exists():p=d/'planning_corrected_input/candidate.json'
 if not p.exists():continue
 plan=json.loads(p.read_text())
 for item in plan['path'][::5]:
  q=np.array(item['q']);pin.forwardKinematics(ik.robot,data,q);pin.updateFramePlacements(ik.robot,data);T=data.oMf[frame].homogeneous.copy();fk=g.chain.root_to_link('tcp_link',dict(zip(c['arm_joint_names'],q)));J=pin.computeFrameJacobian(ik.robot,data,q,frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).copy();F=np.zeros((6,7));eps=1e-6
  for j in range(7):
   qp=q.copy();qm=q.copy();qp[j]+=eps;qm[j]-=eps;P=g.chain.root_to_link('tcp_link',dict(zip(c['arm_joint_names'],qp)));M=g.chain.root_to_link('tcp_link',dict(zip(c['arm_joint_names'],qm)));F[:3,j]=(P[:3,3]-M[:3,3])/(2*eps);F[3:,j]=Rotation.from_matrix(P[:3,:3]@M[:3,:3].T).as_rotvec()/(2*eps)
  checks.append({'asset_id':asset,'state_rad':item['state'],'FK_translation_error_m':float(np.linalg.norm(T[:3,3]-fk[:3,3])),'FK_orientation_error_rad':float(Rotation.from_matrix(T[:3,:3]@fk[:3,:3].T).magnitude()),'Jacobian_max_error':float(np.max(abs(J-F)))})
(OUT/'NATIVE_GEOMETRY_AUDIT.json').write_text(json.dumps({'internal_fourbar_pairs':rows,'FK_JACOBIAN':checks,'diagnostic_only':True,'no_colliders_removed_or_modified':True,'official_schema_file':str(SOURCE.parent/'xarm7_gripper_moveit_config/config/xarm7_with_gripper.srdf')},indent=2));print('FKJ',max((x['Jacobian_max_error'] for x in checks),default=None));print('pairs',len(rows))
