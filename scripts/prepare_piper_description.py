#!/usr/bin/env python3
"""Create a self-contained Piper+official gripper URDF/SRDF from vendored upstream files."""
from pathlib import Path
import copy, xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'third_party/agilex_piper/description'
base=ET.parse(SRC/'urdf/piper_description.urdf').getroot()
gripper_text=(SRC/'urdf/piper_with_gripper_description.xacro').read_text()
gripper=ET.fromstring(gripper_text)
for child in list(gripper):
    if child.tag.endswith('include'): continue
    base.append(copy.deepcopy(child))

# A grasp TCP centered on the official finger-pad end plane.  The value comes
# from the official official distal finger plane at joint origin z=0.138 m; it is not inherited from Franka.
ET.SubElement(base,'link',{'name':'tcp_link'})
j=ET.SubElement(base,'joint',{'name':'tcp_joint','type':'fixed'})
ET.SubElement(j,'parent',{'link':'gripper_base'})
ET.SubElement(j,'child',{'link':'tcp_link'})
ET.SubElement(j,'origin',{'xyz':'0 0 0.138','rpy':'0 0 0'})

for mesh in base.findall('.//mesh'):
    raw=mesh.attrib['filename']; name=raw.split('/meshes/',1)[1]
    mesh.attrib['filename']=str((SRC/'meshes'/name).resolve())
ET.indent(base)
ET.ElementTree(base).write(ROOT/'config/piper.urdf',encoding='unicode',xml_declaration=True)

robot=ET.Element('robot',{'name':'piper'})
group=ET.SubElement(robot,'group',{'name':'arm'}); ET.SubElement(group,'chain',{'base_link':'base_link','tip_link':'tcp_link'})
gg=ET.SubElement(robot,'group',{'name':'gripper'})
for name in ['gripper_base','gripper_link1','gripper_link2']: ET.SubElement(gg,'link',{'name':name})
ET.SubElement(gg,'joint',{'name':'gripper'})
for name in ['gripper_joint1','gripper_joint2']: ET.SubElement(robot,'passive_joint',{'name':name})
ET.SubElement(robot,'end_effector',{'name':'piper_gripper','parent_link':'tcp_link','group':'gripper','parent_group':'arm'})
for a,b,reason in [
 ('base_link','link1','Adjacent'),('link1','link2','Adjacent'),('link2','link3','Adjacent'),
 ('link3','link4','Adjacent'),('link4','link5','Adjacent'),('link5','link6','Adjacent'),
 ('link6','flange_link','Adjacent'),('flange_link','gripper_base','Adjacent'),
 ('gripper_base','gripper_link1','Adjacent'),('gripper_base','gripper_link2','Adjacent'),
 ('gripper_link1','gripper_link2','Default')]:
    ET.SubElement(robot,'disable_collisions',{'link1':a,'link2':b,'reason':reason})
ET.indent(robot); ET.ElementTree(robot).write(ROOT/'config/piper.srdf',encoding='unicode',xml_declaration=True)
print(ROOT/'config/piper.urdf'); print(ROOT/'config/piper.srdf')
