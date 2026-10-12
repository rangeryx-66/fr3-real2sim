"""Explicit finite-effort APPROXIMATE NERO joint servo, not hardware dynamics.

Authoring: official native URDF inertias, Pinocchio gravity, simulation-design
bandwidth 20 rad/s and damping ratio 1.0. Torque ceilings 16/16/16/16/8/8/8 Nm
are conservative firmware-interface-derived simulation assumptions, not measured
continuous ratings. No PiPER arm gain is copied. Finger control stays the proven
same-geometry jaw position/effort logic with 10 N URDF effort ceilings.
"""
from pathlib import Path
import json,numpy as np,pinocchio as pin

class FiniteEffortNero:
 def __init__(self,robot,world,model_config,output=None):
  from isaacsim.core.utils.types import ArticulationAction
  self.Action=ArticulationAction;self.robot=robot;self.world=world;self.raw=robot.get_articulation_controller();self.names=list(robot.dof_names);self.arm=np.array([self.names.index(n) for n in model_config['arm_joint_names']]);self.fingers=np.array([self.names.index('gripper_joint1'),self.names.index('gripper_joint2')]);import xml.etree.ElementTree as ET
  tree=ET.parse(model_config['sim_urdf']);
  for joint in tree.getroot().findall('joint'):
   for mimic in joint.findall('mimic'):joint.remove(mimic)
  dyn_path=Path(model_config['sim_urdf']).with_name('nero_dynamics_coordinates.urdf');tree.write(dyn_path,encoding='utf-8',xml_declaration=True)
  self.pmodel=pin.buildModelFromUrdf(str(dyn_path));self.pdata=self.pmodel.createData();self.pin_to_native=[self.names.index(n) for n in list(self.pmodel.names)[1:]];q=np.asarray(robot.get_joint_positions());pq=q[self.pin_to_native];M=pin.crba(self.pmodel,self.pdata,pq);diag=np.diag(M)[[list(self.pmodel.names)[1:].index(n) for n in model_config['arm_joint_names']]];self.kp=np.minimum(500.,diag*20.**2);self.kd=np.minimum(5.,2.*np.sqrt(self.kp*diag));self.limit=np.array([16.,16.,16.,16.,8.,8.,8.]);self.desired=q[self.arm].copy();self.desired_velocity=np.zeros(7);self.records=[];self.step=0;self.last_torque=np.zeros(7)
  kp=np.zeros(len(self.names));kd=kp.copy();kp[self.fingers]=1000.;kd[self.fingers]=40.;self.raw.set_gains(kps=kp,kds=kd,save_to_usd=True);eff=np.zeros(len(self.names));eff[self.arm]=self.limit;eff[self.fingers]=10.;self.robot._articulation_view.set_max_efforts(eff.reshape(1,-1))
  self.metadata={'classification':'APPROXIMATE_ACTUATOR_NOT_HARDWARE_EQUIVALENT','equation':'tau = clip(Kp*(q_des-q)+Kd*(dq_des-dq)+g_URDF(q), +/-finite_torque)','kp_nm_rad':self.kp.tolist(),'kd_nm_s_rad':self.kd.tolist(),'reference_diagonal_inertia_kg_m2':diag.tolist(),'design_bandwidth_rad_s':20.,'design_damping_ratio':1.,'damping_cap_nm_s_rad':5.,'effective_reference_damping_ratio':(self.kd/(2*np.sqrt(self.kp*diag))).tolist(),'stiffness_cap':500.,'maximum_arm_effort_nm':self.limit.tolist(),'arm_implicit_drive_gains':{'kp':0.,'kd':0.},'gravity_feedforward':'Pinocchio from unmodified official native URDF','unmodelled':['motor electrical lag','gear friction/backlash','rotor inertia beyond CAD link inertia','hardware internal gain scaling','temperature/continuous torque limits'],'finger_position_gains':{'kp':1000.,'kd':40.},'finger_effort_ceiling_n':10.,'source_torque_interface':'https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/nero/firmware_reference.md','source_CAD_commit':model_config['source_commit'],'joint_names_native_order':self.names,'arm_native_indices':self.arm.tolist(),'finger_native_indices':self.fingers.tolist(),'pinocchio_to_native_indices':self.pin_to_native,'same_gripper_logic_not_PiPER_arm_gains':True}
  if output is not None:Path(output).mkdir(parents=True,exist_ok=True);(Path(output)/'APPROXIMATE_ACTUATOR.json').write_text(json.dumps(self.metadata,indent=2))
  world.add_physics_callback('nero_finite_effort_servo',self.physics_callback)
 def physics_callback(self,dt):
  q=np.asarray(self.robot.get_joint_positions(),float);dq=np.asarray(self.robot.get_joint_velocities(),float);g=pin.computeGeneralizedGravity(self.pmodel,self.pdata,q[self.pin_to_native]);gravity=np.array([g[list(self.pmodel.names)[1:].index(self.names[i])] for i in self.arm]);unc=self.kp*(self.desired-q[self.arm])+self.kd*(self.desired_velocity-dq[self.arm])+gravity;torque=np.clip(unc,-self.limit,self.limit);self.last_torque=torque
  self.raw.apply_action(self.Action(joint_efforts=torque,joint_indices=self.arm))
  self.records.append({'step':self.step,'dt':float(dt),'q':q.tolist(),'dq':dq.tolist(),'q_des_arm':self.desired.tolist(),'dq_des_arm':self.desired_velocity.tolist(),'tau_gravity_nm':gravity.tolist(),'tau_unclipped_nm':unc.tolist(),'tau_command_nm':torque.tolist(),'saturated':(abs(unc)>self.limit).tolist()});self.step+=1
 def apply_action(self,action):
  indices=np.arange(len(self.names)) if action.joint_indices is None else np.asarray(action.joint_indices)
  if all(i in self.arm for i in indices):
   positions=action.joint_positions;velocities=action.joint_velocities
   for k,i in enumerate(indices):
    at=int(np.where(self.arm==i)[0][0])
    if positions is not None:self.desired[at]=positions[k]
    if velocities is not None:self.desired_velocity[at]=velocities[k]
   return
  self.raw.apply_action(action)
 def set_gains(self,kps=None,kds=None,save_to_usd=False):
  # Baseline jaw-mode transitions are retained; no baseline arm PD is applied.
  kp=np.zeros(len(self.names)) if kps is None else np.asarray(kps,float).copy();kd=np.zeros(len(self.names)) if kds is None else np.asarray(kds,float).copy();kp[self.arm]=0.;kd[self.arm]=0.;self.raw.set_gains(kps=kp,kds=kd,save_to_usd=save_to_usd)


def apply_verified_filters(stage,root,config):
 from pxr import UsdPhysics
 paths={p.GetName():p for p in stage.Traverse() if str(p.GetPath()).startswith(root) and p.HasAPI(UsdPhysics.RigidBodyAPI)};rows=[]
 for entry in config['verified_collision_semantics']['corrected_pairs']:
  a,b=entry['pair'];api=UsdPhysics.FilteredPairsAPI.Apply(paths[a]);api.CreateFilteredPairsRel().AddTarget(paths[b].GetPath());rows.append({'pair':[a,b],'body_paths':[str(paths[a].GetPath()),str(paths[b].GetPath())],'raw_geometry_check_retained':True,'reason':entry['vendor_semantics']})
 # Native mechanically opposing fingers have the same designed jaw contact
 # exclusion as the frozen gripper, not a robot-arm collision exemption.
 api=UsdPhysics.FilteredPairsAPI.Apply(paths['gripper_link1']);api.CreateFilteredPairsRel().AddTarget(paths['gripper_link2'].GetPath())
 return rows
