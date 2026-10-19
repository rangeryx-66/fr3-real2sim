"""Pinned official NERO assembly; no baseline edits or object physics changes."""
from pathlib import Path
import json,re,shutil,sys,xml.etree.ElementTree as E,hashlib
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');SRC=Path('/data1/home/rangeryx/cross_robot_sources_20261012');O=ROOT/'results/nero_feasibility_20261012';O.mkdir(exist_ok=True);sys.path.insert(0,str(SRC/'python_deps'));import xacro
orig=SRC/'agx_arm_urdf';resolved=O/'resolved_official'
for p in orig.rglob('*.xacro'):
 dest=resolved/p.relative_to(orig);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(p.read_text().replace('$(find agx_arm_description)/agx_arm_urdf',str(resolved)))
for p in orig.rglob('*.urdf'):
 dest=resolved/p.relative_to(orig);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
r=E.fromstring(xacro.process_file(str(resolved/'nero/urdf/nero_with_gripper_description.xacro')).toxml())
for m in r.iter('mesh'):m.set('filename',str(orig/m.get('filename').split('/agx_arm_urdf/')[-1]))
l=E.SubElement(r,'link',name='tcp_link');j=E.SubElement(r,'joint',name='tcp_joint',type='fixed');E.SubElement(j,'parent',link='gripper_base');E.SubElement(j,'child',link='tcp_link');E.SubElement(j,'origin',xyz='0 0 0.138',rpy='0 0 0')
for tag in ['kinematic','sim']:
 root=E.fromstring(E.tostring(r))
 if tag=='sim':
  for x in list(root):
   if (x.tag=='link' and x.get('name')=='gripper_link') or (x.tag=='joint' and x.get('name')=='gripper'):root.remove(x)
  for x in root.findall('joint'):
   mimic=x.find('mimic')
   if mimic is not None:x.remove(mimic)
 # Same physical finger meshes and inertias; simulated jaw positions explicitly
 # control the two native finger sliders, removing only virtual width joint.
 E.indent(root);p=O/('nero_'+tag+'.urdf');E.ElementTree(root).write(p,encoding='utf-8',xml_declaration=True)
limits=[x.find('limit').attrib for x in r.findall('joint') if x.get('name') in [f'joint{i}' for i in range(1,8)]]
c={'robot':'nero','urdf':str(O/'nero_kinematic.urdf'),'sim_urdf':str(O/'nero_sim.urdf'),'arm_joint_names':[f'joint{i}' for i in range(1,8)],'home_q':[0,0,0,.8,0,0,0],'dof':7,'tcp_frame':'tcp_link','flange_frame':'flange_link','mount_status':'OFFICIAL_NERO_NATIVE_GRIPPER_ASSEMBLY','joint_limits':limits,'arm_base_height_m':-.1,'source_commit':'983788b58d7eae76511177f768d85877534917fe','reference_commit':'4b73580','native_gripper_conventions':'official joint1 rpy pi/2,0,pi; joint2 rpy pi/2,0,0; q1=width/2,q2=-width/2','no_collision_pair_exemptions_without_audit':True};(O/'NERO_MODEL.json').write_text(json.dumps(c,indent=2));print(c)
