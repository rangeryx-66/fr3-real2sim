from pathlib import Path
import json,numpy as np,pinocchio as pin,hashlib
from scipy.spatial.transform import Rotation
from cross_robot_adapter import RobotModel,RobotPinkIK
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/cross_robot_20261012';old=ROOT/'results/door_transfer4_20261012/45134';job=json.loads((old/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());result={}
for name,c in json.loads((O/'ROBOT_MODELS.json').read_text()).items():
 model=RobotModel(c,job['asset_root'],source);solver=RobotPinkIK(c);rng=np.random.default_rng(91);samples=[model.home]+list(rng.uniform(model.limits[:,0]+.06,model.limits[:,1]-.06,size=(12,c['dof'])));errs=[];jerrs=[];frame=solver.robot.getFrameId('tcp_link');data=solver.robot.createData()
 for q in samples:
  pin.forwardKinematics(solver.robot,data,q);pin.updateFramePlacements(solver.robot,data);P=data.oMf[frame].homogeneous.copy();R=model.fk(q);errs.append({'position_m':float(np.linalg.norm(P[:3,3]-R[:3,3])),'orientation_rad':float(Rotation.from_matrix(P[:3,:3]@R[:3,:3].T).magnitude())});J=pin.computeFrameJacobian(solver.robot,data,q,frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).copy();fd=[]
  for i in range(len(q)):
   lo=q.copy();hi=q.copy();lo[i]-=1e-6;hi[i]+=1e-6;A=model.fk(lo);B=model.fk(hi);fd.append(np.r_[(B[:3,3]-A[:3,3])/2e-6,Rotation.from_matrix(B[:3,:3]@A[:3,:3].T).as_rotvec()/2e-6])
  jerrs.append(float(np.max(abs(J-np.array(fd).T))))
 reduced=[]
 if c['dof']==7:
  for v in [-.6,0.,.6]:
   s=RobotPinkIK(c,{c['arm_joint_names'][2]:v});q=s.expand(s.home);d=s.robot.createData();pin.forwardKinematics(s.robot,d,s.home);pin.updateFramePlacements(s.robot,d);P=d.oMf[s.robot.getFrameId('tcp_link')].homogeneous;reduced.append({'fixed_joint':c['arm_joint_names'][2],'fixed_rad':v,'nq':s.robot.nq,'full_fk_position_error_m':float(np.linalg.norm(P[:3,3]-model.fk(q)[:3,3]))})
 result[name]={'samples':len(samples),'max_FK_position_error_m':max(e['position_m'] for e in errs),'max_FK_orientation_error_rad':max(e['orientation_rad'] for e in errs),'max_Jacobian_abs_error':max(jerrs),'joint_order':list(solver.robot.names)[1:],'arm_names':c['arm_joint_names'],'reduced_models':reduced,'gripper_pad_bounds':{k:v.tolist() for k,v in model.pads.items()},'geometry_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(c['urdf'])]},'not_physical_grasp_or_actuator_validation':True}
 print(name,result[name]['max_FK_position_error_m'],result[name]['max_Jacobian_abs_error'],flush=True)
(O/'FK_JACOBIAN_VERIFICATION.json').write_text(json.dumps(result,indent=2))
