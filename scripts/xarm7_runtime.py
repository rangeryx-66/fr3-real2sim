"""xArm7-only plant/drive adapter. Approximate simulation, virtual PiPER mount."""
from pathlib import Path
import json, hashlib, xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import pinocchio as pin

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/xarm7_shared_doors_20261012'

def prepare():
    OUT.mkdir(parents=True,exist_ok=True)
    config=json.loads((ROOT/'results/cross_robot_20261012/ROBOT_MODELS.json').read_text())['xarm7']
    old=Path(config['urdf']);tree=ET.parse(old);r=tree.getroot()
    for item in list(r):
        if (item.tag=='link' and item.get('name')=='gripper_link') or (item.tag=='joint' and item.get('name')=='gripper'):r.remove(item)
    for joint in r.findall('joint'):
        for mimic in joint.findall('mimic'):joint.remove(mimic)
    dynamics=OUT/'xarm7_dynamics_coordinates.urdf';tree.write(dynamics,encoding='utf-8',xml_declaration=True)
    ET.SubElement(r.find("joint[@name='gripper_joint2']"),'mimic',joint='gripper_joint1',multiplier='-1',offset='0')
    sim=OUT/'xarm7_piper_sim.urdf';tree.write(sim,encoding='utf-8',xml_declaration=True)
    import yaml
    source=Path('/data1/home/rangeryx/cross_robot_sources_20261012/xarm_ros/xarm_controller/config/xarm7/xarm7_controllers.yaml')
    pid=yaml.safe_load(source.read_text())
    config.update(sim_urdf=str(sim),dynamics_urdf=str(dynamics),arm_kp=[pid[n+'_position_controller']['pid']['p'] for n in config['arm_joint_names']],arm_kd=[pid[n+'_position_controller']['pid']['d'] for n in config['arm_joint_names']],arm_effort_limits=[float(x['limit']['effort']) for x in config['joint_parameters']],arm_speed_limits=[float(x['limit']['velocity']) for x in config['joint_parameters']],classification='XARM7 / APPROXIMATE_ACTUATOR_SIMULATION / VIRTUAL_PIPER_TOOL_ADAPTER',gain_reference=str(source),gain_reference_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),mount_status='VIRTUAL_COAXIAL_ADAPTER_NOT_VERIFIED_HARDWARE',gravity_bias=True,integral_control=False,manufacturer_servo_identification=False)
    (OUT/'XARM7_CONFIG.json').write_text(json.dumps(config,indent=2));return config

def configuration():return json.loads((OUT/'XARM7_CONFIG.json').read_text())

def configure_stage(stage,root,config,output):
    """Restore official mass tensors and NERO-independent mechanical semantics."""
    from pxr import UsdPhysics,PhysxSchema,Gf,Sdf
    urdf=ET.parse(config['sim_urdf']).getroot();bodies={p.GetName():p for p in stage.Traverse() if str(p.GetPath()).startswith(root) and p.HasAPI(UsdPhysics.RigidBodyAPI)}
    mass=[];filters=[];links=[l.get('name') for l in urdf.findall('link')];parent={n:n for n in links}
    def find(n):
        while parent[n]!=n:n=parent[n]
        return n
    for j in urdf.findall('joint'):
        if j.get('type')=='fixed':parent[find(j.find('child').get('link'))]=find(j.find('parent').get('link'))
    neighbours=set()
    for j in urdf.findall('joint'):neighbours.add(frozenset([find(j.find('parent').get('link')),find(j.find('child').get('link'))]))
    for a in bodies:
        for b in bodies:
            if a>=b:continue
            if find(a)==find(b) or frozenset([find(a),find(b)]) in neighbours or {a,b}=={'gripper_link1','gripper_link2'}:
                api=UsdPhysics.FilteredPairsAPI.Apply(bodies[a]);api.CreateFilteredPairsRel().AddTarget(bodies[b].GetPath());filters.append([a,b])
    for l in urdf.findall('link'):
        name=l.get('name');ins=l.find('inertial')
        if name not in bodies or ins is None:continue
        a=ins.find('inertia').attrib;I=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
        origin=ins.find('origin');rot=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix();I=rot@I@rot.T
        values,V=np.linalg.eigh(I)
        if np.linalg.det(V)<0:V[:,0]*=-1
        q=np.roll(Rotation.from_matrix(V).as_quat(),1);api=UsdPhysics.MassAPI.Apply(bodies[name]);before={'mass':api.GetMassAttr().Get(),'principal_axes':str(api.GetPrincipalAxesAttr().Get()),'diagonal':str(api.GetDiagonalInertiaAttr().Get())}
        m=float(ins.find('mass').get('value'));com=[float(x) for x in origin.get('xyz','0 0 0').split()]
        api.CreateMassAttr(m);api.CreateCenterOfMassAttr(Gf.Vec3f(*com));api.CreateDiagonalInertiaAttr(Gf.Vec3f(*values));api.CreatePrincipalAxesAttr(Gf.Quatf(*q))
        mass.append({'body':name,'before':before,'official_mass_kg':m,'official_tensor':I.tolist(),'principal_axes_wxyz':q.tolist(),'diagonal':values.tolist(),'tensor_reconstruction_error':float(abs(V@np.diag(values)@V.T-I).max())})
    joint_rows=[]
    for p in stage.Traverse():
        if not str(p.GetPath()).startswith(root) or p.GetName() not in config['arm_joint_names'] or not p.IsA(UsdPhysics.Joint):continue
        i=config['arm_joint_names'].index(p.GetName());drive=UsdPhysics.DriveAPI.Apply(p,'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(config['arm_kp'][i]*np.pi/180);drive.CreateDampingAttr(config['arm_kd'][i]*np.pi/180);drive.CreateMaxForceAttr(config['arm_effort_limits'][i])
        dyn=urdf.find("joint[@name='%s']/dynamics"%p.GetName());friction=float(dyn.get('friction','0')) if dyn is not None else 0.;viscous=float(dyn.get('damping','0')) if dyn is not None else 0.
        # URDF friction is an effort, not the deprecated load-dependent ratio.
        PhysxSchema.PhysxJointAPI.Apply(p).CreateJointFrictionAttr(0.)
        p.AddAppliedSchema('PhysxJointAxisAPI:angular')
        for key,value in {'staticFrictionEffort':friction,'dynamicFrictionEffort':friction,'viscousFrictionCoefficient':viscous*np.pi/180}.items():p.CreateAttribute('physxJointAxis:angular:'+key,Sdf.ValueTypeNames.Float).Set(value)
        joint_rows.append({'joint':p.GetName(),'URDF_coulomb_effort_nm':friction,'URDF_viscous_nm_s_rad':viscous,'implicit_kp_nm_rad':config['arm_kp'][i],'implicit_kd_nm_s_rad':config['arm_kd'][i],'finite_max_effort_nm':config['arm_effort_limits'][i]})
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene):
            p.CreateAttribute('physxScene:enableExternalForcesEveryIteration',Sdf.ValueTypeNames.Bool).Set(False);PhysxSchema.PhysxSceneAPI.Apply(p).CreateSolverTypeAttr('PGS')
    audit={'mass_properties':mass,'mechanical_connected_cluster_filters':filters,'new_nonadjacent_Never_exemptions':[],'joints':joint_rows,'robot_geometry_unchanged':True,'official_inertia_orientation_explicitly_authored':True,'solver':'PGS; evidence-backed correction for native/full-step velocity discrepancy, gains/limits unchanged','virtual_mount':config['mount_transform']}
    Path(output).mkdir(parents=True,exist_ok=True);(Path(output)/'XARM7_STAGE_AUDIT.json').write_text(json.dumps(audit,indent=2));return audit

class XArm7Drive:
    """Native implicit arm drives; preserve shared jaw position/effort modes."""
    def __init__(self,robot,config,output):
        from isaacsim.core.utils.types import ArticulationAction
        self.Action=ArticulationAction;self.robot=robot;self.raw=robot.get_articulation_controller();self.config=config;self.names=list(robot.dof_names);self.arm=np.array([self.names.index(n) for n in config['arm_joint_names']]);self.fingers=np.array([self.names.index(n) for n in ['gripper_joint1','gripper_joint2']]);self.kp=np.array(config['arm_kp']);self.kd=np.array(config['arm_kd']);self.model=pin.buildModelFromUrdf(config['dynamics_urdf']);self.data=self.model.createData();pn=list(self.model.names)[1:];self.pin_native=[self.names.index(n) for n in pn];self.pin_arm=[pn.index(n) for n in config['arm_joint_names']];self.last={}
        kp=np.zeros(len(self.names));kd=kp.copy();kp[self.arm]=self.kp;kd[self.arm]=self.kd;kp[self.fingers]=1000.;kd[self.fingers]=40.;self.raw.set_gains(kps=kp,kds=kd,save_to_usd=True)
        caps=np.zeros(len(self.names));caps[self.arm]=config['arm_effort_limits'];caps[self.fingers]=10.;robot._articulation_view.set_max_efforts(caps.reshape(1,-1));speeds=np.full(len(self.names),1.);speeds[self.arm]=config['arm_speed_limits'];robot._articulation_view.set_max_joint_velocities(speeds.reshape(1,-1))
        (Path(output)/'APPROXIMATE_ACTUATOR.json').write_text(json.dumps({'classification':config['classification'],'equation':'finite implicit PhysX force drive; q_drive=q_ref+g_URDF(q_ref)/Kp; no external arm torque servo','kp':self.kp.tolist(),'kd':self.kd.tolist(),'effort_limits':config['arm_effort_limits'],'speed_limits':config['arm_speed_limits'],'source':config['gain_reference'],'source_sha256':config['gain_reference_sha256'],'integral_term_not_implemented':True,'gravity_compensation_inside_same_drive':True,'jaw_control_unchanged':True,'actual_native_order':self.names,'native_arm_indices':self.arm.tolist(),'native_finger_indices':self.fingers.tolist(),'virtual_mount':True},indent=2))
    def apply_action(self,action):
        ix=np.arange(len(self.names)) if action.joint_indices is None else np.asarray(action.joint_indices)
        if len(ix)==len(self.arm) and np.array_equal(ix,self.arm) and action.joint_positions is not None:
            q=np.asarray(self.robot.get_joint_positions()).copy();reference=np.asarray(action.joint_positions);q[self.arm]=reference;g=pin.computeGeneralizedGravity(self.model,self.data,q[self.pin_native])[self.pin_arm].copy();target=reference+g/self.kp;vel=np.zeros(7) if action.joint_velocities is None else np.asarray(action.joint_velocities)
            self.raw.apply_action(self.Action(joint_positions=target,joint_velocities=vel,joint_efforts=np.zeros(7),joint_indices=self.arm));self.last={'reference':reference.tolist(),'velocity_reference':vel.tolist(),'gravity_bias_nm':g.tolist(),'implicit_position_target':target.tolist(),'external_effort_command_nm':[0.]*7}
        else:self.raw.apply_action(action)
    def set_gains(self,kps=None,kds=None,save_to_usd=False):
        kp,kd=self.raw.get_gains();kp=kp.copy() if kps is None else np.asarray(kps).copy();kd=kd.copy() if kds is None else np.asarray(kds).copy();kp[self.arm]=self.kp;kd[self.arm]=self.kd;self.raw.set_gains(kps=kp,kds=kd,save_to_usd=save_to_usd)

if __name__=='__main__':print(json.dumps(prepare(),indent=2))
