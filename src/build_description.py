from pathlib import Path
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'build_tools'))
import xacro
import xacro.substitution_args as substitutions
original=substitutions._find
# Resolve only this local package, without requiring ROS during asset conversion.
def find(resolved,a,args,context):
    if args == ['franka_description']:
        return resolved.replace('$(%s)' % a,str(ROOT/'franka_description'))
    return original(resolved,a,args,context)
substitutions._find=find
(ROOT/'config').mkdir(exist_ok=True)
for kind in ['urdf','srdf']:
    doc=xacro.process_file(str(ROOT/f'franka_description/robots/fr3/fr3.{kind}.xacro'), mappings={'hand':'true','with_sc':'false','include_self_collision_geometry':'false'})
    text=doc.toxml().replace('package://franka_description/', str(ROOT/'franka_description')+'/')
    (ROOT/f'config/fr3.{kind}').write_text(text)
robot=ET.parse(ROOT/'config/fr3.urdf')
print([(j.attrib['name'],j.attrib['type']) for j in robot.findall('joint')])
print('Description generated from official source; identical URDF used by simulation and MoveIt.')
