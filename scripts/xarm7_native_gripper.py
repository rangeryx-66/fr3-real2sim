"""Official xArm G1 hand; finite implicit position actuator, isolated from frozen tools.

Simulated contact reports are diagnostics, not advertised hardware feedback.
The SDK hand exposes position/speed; hold retains a position target.
"""
from pathlib import Path
import copy, json, hashlib, sys, xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
from articulated_demo.kinematics import URDFChain
from xarm7_runtime import configure_stage as configure_arm_stage

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/xarm7_native_gripper_20261012'
SOURCE=Path('/data1/home/rangeryx/cross_robot_sources_20261012/xarm_ros/xarm_description')
GRIPPER_JOINTS=['drive_joint','left_finger_joint','left_inner_knuckle_joint',
                'right_outer_knuckle_joint','right_finger_joint','right_inner_knuckle_joint']
FINGER_BODIES=['left_finger','right_finger']

def prepare():
    OUT.mkdir(parents=True,exist_ok=True)
    c=json.loads((ROOT/'results/xarm7_shared_doors_20261012/XARM7_CONFIG.json').read_text())
    sys.path.insert(0,'/data1/home/rangeryx/cross_robot_sources_20261012/python_deps')
    import xacro
    wrapper=OUT/'official_gripper_wrapper.xacro'
    wrapper.write_text(f'''<robot xmlns:xacro="http://ros.org/wiki/xacro" name="xarm_gripper">
      <xacro:property name="mesh_path" value="{SOURCE}/meshes"/>
      <xacro:property name="mesh_suffix" value="stl"/>
      <xacro:include filename="{SOURCE}/urdf/common/common.link.xacro"/>
      <xacro:include filename="{SOURCE}/urdf/gripper/xarm_gripper.urdf.xacro"/>
      <xacro:xarm_gripper_urdf attach_to="link_eef"/>
    </robot>''')
    gr=ET.fromstring(xacro.process_file(str(wrapper)).toxml())
    arm=ET.parse(c['sim_urdf']).getroot()
    keep={'world','base_link','link_base','link_eef'}|{f'link{i}' for i in range(1,8)}
    for e in list(arm):
        if e.tag=='link' and e.get('name') not in keep:arm.remove(e)
        elif e.tag=='joint' and (e.find('parent').get('link') not in keep or e.find('child').get('link') not in keep):arm.remove(e)
    for e in gr:arm.append(copy.deepcopy(e))
    # A frame alias only; physical manufacturer TCP transform is unchanged.
    ET.SubElement(arm,'link',name='tcp_link')
    alias=ET.SubElement(arm,'joint',name='shared_tcp_alias',type='fixed')
    ET.SubElement(alias,'parent',link='link_tcp');ET.SubElement(alias,'child',link='tcp_link')
    sim=OUT/'xarm7_native_gripper.urdf';ET.ElementTree(arm).write(sim,encoding='utf-8',xml_declaration=True)
    for e in arm.findall('joint'):
        for m in e.findall('mimic'):e.remove(m)
    dynamics=OUT/'xarm7_native_dynamics.urdf';ET.ElementTree(arm).write(dynamics,encoding='utf-8',xml_declaration=True)
    c.update(urdf=str(sim),sim_urdf=str(sim),dynamics_urdf=str(dynamics),
             classification='XARM7 / NATIVE_XARM_GRIPPER / APPROXIMATE_ACTUATOR_SIMULATION',
             mount_transform={'xyz_m':[0,0,0],'rpy_rad':[0,0,0],'source':'official xarm_device_macro.xacro: link_eef'},
             gripper_joint_names=GRIPPER_JOINTS,gripper_kp_nm_rad=10.,gripper_kd_nm_s_rad=.1,
             gripper_max_clamp_n=30.,gripper_max_velocity_rad_s=2.,
             gripper_feedback={'hardware':['position','status/error'],'simulation_only':['raw contact impulses','pad loads','joint efforts']},
             gripper_hold='retained finite-effort position command; no PiPER effort-hold',
             gripper_source_commit='aad7e1611c9c46eb719045414394bfdd42dcb0f8',
             gazebo_grasp_fix_plugin_used=False,virtual_adapter=False)
    (OUT/'NATIVE_CONFIG.json').write_text(json.dumps(c,indent=2))
    geometry=NativeGripperGeometry(c)
    samples=[{'drive_rad':float(a),'aperture_m':geometry.width(a),'finger_poses':{n:geometry.chain.root_to_link(n,geometry.values(a)).tolist() for n in FINGER_BODIES}} for a in np.linspace(0,.85,18)]
    (OUT/'OFFICIAL_GEOMETRY.json').write_text(json.dumps({'samples':samples,'mass_kg':sum(float(l.find('inertial/mass').get('value')) for l in gr.findall('link') if l.find('inertial') is not None),'sources':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [SOURCE/'urdf/gripper/xarm_gripper.urdf.xacro',SOURCE/'urdf/common/common.link.xacro',sim]},'no_plugin_attachment':True},indent=2))
    return c

def configuration():return json.loads((OUT/'NATIVE_CONFIG.json').read_text())

class NativeGripperGeometry:
    def __init__(self,c=None):
        self.c=c or configuration();self.chain=URDFChain(self.c['sim_urdf'])
        # Inner distal surface is measured from the official finger mesh.
        from piper_mobile_demo.model import geometry
        urdf=ET.parse(self.c['sim_urdf']).getroot()
        self.meshes={n:geometry(urdf.find("link[@name='%s']/collision"%n),Path(self.c['sim_urdf']).parent) for n in FINGER_BODIES}
        self.inner={n:(self.meshes[n].vertices[:,1].min() if n=='left_finger' else self.meshes[n].vertices[:,1].max()) for n in FINGER_BODIES}
        self.grid=np.linspace(0,.85,1001);self.width_grid=np.array([self.width(a) for a in self.grid])
    def values(self,a):return dict.fromkeys(GRIPPER_JOINTS,float(a))
    def width(self,a):
        p=[]
        B=np.linalg.inv(self.chain.root_to_link('xarm_gripper_base_link',{}))
        for n in FINGER_BODIES:
            T=B@self.chain.root_to_link(n,self.values(a));p.append((T@np.array([0,self.inner[n],0,1]))[:3])
        return float(p[0][1]-p[1][1])
    def angle(self,width):return float(np.interp(width,self.width_grid[::-1],self.grid[::-1]))
    def width_derivative(self,a):
        lo=max(0.,a-1e-5);hi=min(.85,a+1e-5)
        return (self.width(hi)-self.width(lo))/(hi-lo)

def configure_stage(stage,root,c,output):
    from pxr import UsdPhysics,PhysxSchema
    audit=configure_arm_stage(stage,root,c,output)
    joints={p.GetName():p for p in stage.Traverse() if str(p.GetPath()).startswith(root) and p.IsA(UsdPhysics.Joint)}
    # Isaac 6's converter emits NewtonMimicAPI; explicitly author the same
    # URDF relationship for the PhysX backend. This couples real dynamics.
    for n in GRIPPER_JOINTS[1:]:
        mimic=PhysxSchema.PhysxMimicJointAPI.Apply(joints[n],'rotX')
        mimic.CreateReferenceJointRel().SetTargets([joints['drive_joint'].GetPath()])
        mimic.CreateGearingAttr(-1.);mimic.CreateOffsetAttr(0.)
    bodies={p.GetName():p for p in stage.Traverse() if str(p.GetPath()).startswith(root) and p.HasAPI(UsdPhysics.RigidBodyAPI)}
    # The four-bar's internal paired links are mechanically connected; these
    # exact pairs also appear in the official xarm7_with_gripper.srdf.
    coupled_pairs=[(side+'_finger',side+'_inner_knuckle') for side in ['left','right']]+[(side+'_inner_knuckle',side+'_outer_knuckle') for side in ['left','right']]
    for a,b in coupled_pairs:UsdPhysics.FilteredPairsAPI.Apply(bodies[a]).CreateFilteredPairsRel().AddTarget(bodies[b].GetPath())
    for p in stage.Traverse():
        if str(p.GetPath()).startswith(root) and p.GetName() in GRIPPER_JOINTS and p.IsA(UsdPhysics.Joint):
            d=UsdPhysics.DriveAPI.Apply(p,'angular')
            d.CreateStiffnessAttr(c['gripper_kp_nm_rad']*np.pi/180 if p.GetName()=='drive_joint' else 0.)
            d.CreateDampingAttr(c['gripper_kd_nm_s_rad']*np.pi/180 if p.GetName()=='drive_joint' else 0.)
            d.CreateMaxForceAttr(50. if p.GetName()=='drive_joint' else 0.)
    audit['gripper']='official G1 meshes, inertias and five explicitly authored PhysX mimic constraints; master position drive only'
    audit['native_coupling_pairs']=coupled_pairs
    audit['native_coupling_source']=str(SOURCE.parent/'xarm7_gripper_moveit_config/config/xarm7_with_gripper.srdf')
    (Path(output)/'NATIVE_STAGE_AUDIT.json').write_text(json.dumps(audit,indent=2))
    return audit

class NativeDrive:
    def __init__(self,robot,c,output):
        from isaacsim.core.utils.types import ArticulationAction
        self.Action=ArticulationAction;self.robot=robot;self.raw=robot.get_articulation_controller();self.c=c
        self.names=list(robot.dof_names);self.arm=np.array([self.names.index(n) for n in c['arm_joint_names']]);self.master=self.names.index('drive_joint');self.fingers=np.array([self.names.index(n) for n in GRIPPER_JOINTS]);self.geometry=NativeGripperGeometry(c)
        self.kp=np.array(c['arm_kp']);self.kd=np.array(c['arm_kd']);self.model=pin.buildModelFromUrdf(c['dynamics_urdf']);self.data=self.model.createData()
        pn=list(self.model.names)[1:];self.pin_native=[self.names.index(n) for n in pn];self.pin_arm=[pn.index(n) for n in c['arm_joint_names']]
        kp=np.zeros(len(self.names));kd=kp.copy();kp[self.arm]=self.kp;kd[self.arm]=self.kd;kp[self.master]=c['gripper_kp_nm_rad'];kd[self.master]=c['gripper_kd_nm_s_rad'];self.raw.set_gains(kps=kp,kds=kd,save_to_usd=True)
        caps=np.zeros(len(self.names));caps[self.arm]=c['arm_effort_limits'];caps[self.master]=c['gripper_max_clamp_n']*abs(self.geometry.width_derivative(0.));robot._articulation_view.set_max_efforts(caps.reshape(1,-1))
        self.caps=caps;speeds=np.full(len(self.names),c['gripper_max_velocity_rad_s']);speeds[self.arm]=c['arm_speed_limits'];robot._articulation_view.set_max_joint_velocities(speeds.reshape(1,-1));self.last={};self.aperture=.08
        (Path(output)/'ACTUATOR_METADATA.json').write_text(json.dumps({'classification':c['classification'],'active_joint_order':self.names,'arm_kp':c['arm_kp'],'arm_kd':c['arm_kd'],'effort_limits':caps.tolist(),'gripper_drive':'finite implicit position drive; one motor, five mimic constraints','hand_kp':c['gripper_kp_nm_rad'],'hand_kd':c['gripper_kd_nm_s_rad'],'cap_formula':'30 N * absolute aperture Jacobian; master cap updated at measured angle','feedback':c['gripper_feedback'],'not_hardware_servo_identification':True},indent=2))
    def apply_action(self,action):
        ix=np.arange(len(self.names)) if action.joint_indices is None else np.asarray(action.joint_indices)
        if np.array_equal(ix,self.arm) and action.joint_positions is not None:
            q=np.asarray(self.robot.get_joint_positions()).copy();ref=np.asarray(action.joint_positions);q[self.arm]=ref;g=pin.computeGeneralizedGravity(self.model,self.data,q[self.pin_native])[self.pin_arm].copy();vel=np.zeros(7) if action.joint_velocities is None else np.asarray(action.joint_velocities)
            self.raw.apply_action(self.Action(joint_positions=ref+g/self.kp,joint_velocities=vel,joint_efforts=np.zeros(7),joint_indices=self.arm));self.last={'reference':ref.tolist(),'velocity_reference':vel.tolist(),'gravity_bias_nm':g.tolist(),'implicit_position_target':(ref+g/self.kp).tolist(),'external_effort_command_nm':[0.]*7}
        else:self.raw.apply_action(action)
    def command_width(self,width):
        self.aperture=float(width);a=self.geometry.angle(width);measured=float(self.robot.get_joint_positions()[self.master]);self.caps[self.master]=self.c['gripper_max_clamp_n']*abs(self.geometry.width_derivative(measured));self.robot._articulation_view.set_max_efforts(self.caps.reshape(1,-1))
        self.raw.apply_action(self.Action(joint_positions=np.array([a]),joint_velocities=np.zeros(1),joint_efforts=np.zeros(1),joint_indices=np.array([self.master])))
    def hold(self):self.command_width(self.aperture)
    def release(self,width=.084):self.command_width(width)
    def set_gains(self,kps=None,kds=None,save_to_usd=False):
        # Shared PiPER mode switches cannot turn a native position hand into
        # independent effort-controlled fingers, or overwrite arm gains.
        kp=np.zeros(len(self.names));kd=kp.copy();kp[self.arm]=self.kp;kd[self.arm]=self.kd
        kp[self.master]=self.c['gripper_kp_nm_rad'];kd[self.master]=self.c['gripper_kd_nm_s_rad']
        self.raw.set_gains(kps=kp,kds=kd,save_to_usd=save_to_usd)
    def feedback(self):
        q=np.asarray(self.robot.get_joint_positions());a=float(q[self.master]);return {'position_rad':a,'aperture_m':self.geometry.width(a),'commanded_aperture_m':self.aperture,'control_mode':'FINITE_POSITION_HOLD','mimic_error_rad':{n:float(q[self.names.index(n)]-a) for n in GRIPPER_JOINTS[1:]},'hardware_force_feedback_available':False}

if __name__=='__main__':print(json.dumps(prepare(),indent=2))
