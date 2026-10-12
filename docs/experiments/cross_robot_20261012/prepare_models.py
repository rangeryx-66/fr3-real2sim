"""Resolve official source URDFs and keep the frozen PiPER hand literal.
Foreign flange-to-hand proposals are explicitly offline-only until mechanically
validated. No motor model is silently transferred from the PiPER baseline.
"""
from pathlib import Path
import json,copy,hashlib,subprocess,re,sys,xml.etree.ElementTree as ET
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');SRC=Path('/data1/home/rangeryx/cross_robot_sources_20261012');OUT=ROOT/'results/cross_robot_20261012';OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(SRC/'python_deps'));import xacro
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
git=lambda p:subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
piper=ET.parse(ROOT/'config/piper.urdf').getroot();piper_links={e.get('name'):e for e in piper.findall('link')};piper_joints={e.get('name'):e for e in piper.findall('joint')};tool_names=['gripper_base','gripper_link','gripper_link1','gripper_link2','tcp_link'];tool_joints=['gripper','gripper_joint1','gripper_joint2','tcp_joint'];configs={}

def write(name,r,official,flange,home,mount_kind,mount_origin=None):
 d=OUT/'models'/name;d.mkdir(parents=True,exist_ok=True)
 root=copy.deepcopy(r)
 aliases={}
 if name=='fr3':
  aliases={f'fr3_joint{i}':f'joint{i}' for i in range(1,8)}
  for j in root.findall('joint'):
   if j.get('name') in aliases:j.set('name',aliases[j.get('name')])
 # Names of arm joints stay official; copied gripper subtree is byte-parameter
 # equivalent to frozen config. Only its mounting parent changes.
 if name!='piper':
  for n in tool_names:root.append(copy.deepcopy(piper_links[n]))
  for n in tool_joints:root.append(copy.deepcopy(piper_joints[n]))
  mount=copy.deepcopy(piper_joints['gripper_base_joint']);mount.find('parent').set('link',flange)
  if mount_origin is not None:
   mount.find('origin').set('xyz',mount_origin[0]);mount.find('origin').set('rpy',mount_origin[1])
  root.append(mount)
 # Continuous xArm joints are exposed in a finite manufacturer controller
 # workspace for comparable IK; source infinite rotation remains reported.
 continuous=[]
 for j in root.findall('joint'):
  if j.get('type')=='continuous':
   continuous.append(j.get('name'));j.set('type','revolute');lim=j.find('limit');lim.set('lower',str(-6.283185307179586));lim.set('upper',str(6.283185307179586))
 arm=[j.get('name') for j in root.findall('joint') if j.get('type') in ['revolute','continuous']]
 params=[]
 for j in root.findall('joint'):
  if j.get('name') not in arm:continue
  params.append({'joint':j.get('name'),'limit':j.find('limit').attrib,'dynamics':None if j.find('dynamics') is None else j.find('dynamics').attrib})
 path=d/'robot_piper_tool.urdf';ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
 missing=[m.get('filename') for m in root.iter('mesh') if not Path(m.get('filename')).is_file()]
 c={'robot':name,'urdf':str(path),'urdf_sha256':sha(path),'official_sources':official,'arm_joint_names':arm,'official_to_adapter_joint_names':aliases,'dof':len(arm),'home_q':home,'flange_frame':flange,'tcp_frame':'tcp_link','mount_status':mount_kind,'mount_transform':root.find("joint[@name='gripper_base_joint']/origin").attrib,'continuous_joint_finite_workspace':{'joints':continuous,'interval_rad':[-6.283185307179586,6.283185307179586],'reason':'bounded controller workspace, not mechanical position stops'} if continuous else None,'joint_parameters':params,'mesh_missing':missing,'same_hand_geometry':True,'hand_source_urdf_sha256':sha(ROOT/'config/piper.urdf'),'arm_base_height_m':-.1,'physics_launch_eligibility':'pending model/tool audit','actuator_model_not_substituted':True}
 configs[name]=c;(d/'MODEL_MANIFEST.json').write_text(json.dumps(c,indent=2))

write('piper',piper,[{'source':'frozen 81ab4f5 baseline','urdf_sha256':sha(ROOT/'config/piper.urdf')}],'flange_link',[0,1.15,-1.35,0,.2,0.],'FROZEN_PHYSICALLY_VALIDATED')
agx=SRC/'agx_arm_urdf';nero=ET.parse(agx/'nero/urdf/nero_description.urdf').getroot();f=ET.parse(agx/'nero/urdf/nero_with_flange_description.xacro').getroot()
for e in f:
 if e.tag in ('link','joint'):nero.append(copy.deepcopy(e))
for m in nero.iter('mesh'):m.set('filename',str(agx/m.get('filename').split('/agx_arm_urdf/')[-1]))
write('nero',nero,[{'repository':'https://github.com/agilexrobotics/agx_arm_urdf','commit':git(agx),'file':'nero/urdf/nero_description.urdf','sha256':sha(agx/'nero/urdf/nero_description.urdf')},{'file':'nero/urdf/nero_with_flange_description.xacro','sha256':sha(agx/'nero/urdf/nero_with_flange_description.xacro')}],'flange_link',[0,0,0,.8,0,0,0],'OFFICIAL_AGX_FLANGE_MOUNT; EXACT_OLD_PIPER_HAND; FINGER_CONVENTION_DIFF_RECORDED',('0 0 0.006','0 0 0'))
# Resolve ROS find substitutions into a separate generated description tree.
xrepo=SRC/'xarm_ros';derived=SRC/'xarm_resolved';derived.mkdir(exist_ok=True)
for p in (xrepo/'xarm_description').rglob('*'):
 if not p.is_file() or p.suffix not in ('.xacro','.yaml'):continue
 dest=derived/p.relative_to(xrepo);dest.parent.mkdir(parents=True,exist_ok=True)
 s=p.read_text();s=re.sub(r'\$\(find ([^)]+)\)',lambda m:str(derived/m[1]) if m[1]=='xarm_description' else str(xrepo/m[1]),s);dest.write_text(s)
file=derived/'xarm_description/urdf/xarm_device.urdf.xacro';doc=xacro.process_file(str(file),mappings={'dof':'7','add_gripper':'false','add_d435i_links':'false','limited':'false','mesh_suffix':'stl'});xroot=ET.fromstring(doc.toxml())
for m in xroot.iter('mesh'):
 name=m.get('filename')
 if name.startswith('package://'):name=str(xrepo/name.removeprefix('package://'))
 name=name.replace(str(derived/'xarm_description'),str(xrepo/'xarm_description'));m.set('filename',name)
for e in list(xroot):
 if e.tag not in ('link','joint','material'):xroot.remove(e)
flange='link_eef' if xroot.find("link[@name='link_eef']") is not None else 'link7'
write('xarm7',xroot,[{'repository':'https://github.com/xArm-Developer/xarm_ros','commit':git(xrepo),'file':'xarm_description/urdf/xarm7/xarm7.urdf.xacro','sha256':sha(xrepo/'xarm_description/urdf/xarm7/xarm7.urdf.xacro')}],flange,[0,-.5,0,1.,0,.5,0],'OFFLINE_ONLY_COAXIAL_TOOL_PROPOSAL; NO_VALIDATED_MECHANICAL_ADAPTER')
frsource=Path('/data1/home/rangeryx/fr3_moveit_grasp/config/fr3.urdf');frrepo=Path('/data1/home/rangeryx/fr3_moveit_grasp/franka_description');fr=ET.parse(frsource).getroot();remove={'fr3_hand','fr3_hand_tcp','fr3_leftfinger','fr3_rightfinger'}
for e in list(fr):
 if (e.tag=='link' and e.get('name') in remove) or (e.tag=='joint' and e.find('child').get('link') in remove):fr.remove(e)
write('fr3',fr,[{'repository':'https://github.com/frankarobotics/franka_description','commit':git(frrepo),'file':str(frsource),'sha256':sha(frsource),'dynamic_source':str(frrepo/'robots/fr3/dynamics.yaml'),'dynamic_sha256':sha(frrepo/'robots/fr3/dynamics.yaml')}],'fr3_link8',[0,-.569,0,-2.81,0,3.037,.741],'OFFLINE_ONLY_COAXIAL_TOOL_PROPOSAL; NO_VALIDATED_MECHANICAL_ADAPTER')
(OUT/'ROBOT_MODELS.json').write_text(json.dumps(configs,indent=2));print(json.dumps({k:{'dof':v['dof'],'missing_meshes':v['mesh_missing'],'mount_status':v['mount_status']} for k,v in configs.items()},indent=2))
