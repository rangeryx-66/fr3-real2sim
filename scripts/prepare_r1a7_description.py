#!/usr/bin/env python3
"""Combine pinned Unitree R1-7a and Dex1 URDFs without changing source geometry.

The R1-7a arm and Dex1 hand are separately published by Unitree. Their mutual
mount is not defined upstream. The fixed transform below puts the Dex1 mounting
face on the distal face of Link7, with its fingers pointing along arm -Y.
It must be checked against the physical adapter before hardware use.
"""
from pathlib import Path
import copy
import math
import os
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT / 'third_party/unitree_ros/robots'
ARM = OFFICIAL / 'r1_7a_description/R1_7a.urdf'
HAND = OFFICIAL / 'dexterous_hand_description/dex1_1/dex1_1.urdf'
OUT = ROOT / 'config/r1a7_dex1.urdf'
SRDF = ROOT / 'config/r1a7_dex1.srdf'

arm = ET.parse(ARM).getroot()
arm.set('name', 'r1a7_dex1')
hand = ET.parse(HAND).getroot()
ET.SubElement(arm, 'link', name='r1a7_world')
world_mount = ET.SubElement(arm, 'joint', name='r1a7_world_mount', type='fixed')
ET.SubElement(world_mount, 'parent', link='r1a7_world')
ET.SubElement(world_mount, 'child', link='base_link')
placement = [float(value) for value in os.environ.get(
    'R1A7_BASE_POSE', '0,0,0,90').split(',')]
if len(placement) != 4 or not all(math.isfinite(value) for value in placement):
    raise ValueError('R1A7_BASE_POSE must be x,y,z,yaw_deg')
x, y, z, yaw_deg = placement
ET.SubElement(world_mount, 'origin',
              xyz=f'{x:.9g} {y:.9g} {z:.9g}',
              rpy=f'0 0 {math.radians(yaw_deg):.12g}')
links = {e.get('name'): 'dex1_' + e.get('name') for e in hand.findall('link')}
joints = {e.get('name'): 'dex1_' + e.get('name') for e in hand.findall('joint')}
for element in list(hand):
    element = copy.deepcopy(element)
    if element.tag == 'link':
        element.set('name', links[element.get('name')])
    elif element.tag == 'joint':
        element.set('name', joints[element.get('name')])
        element.find('parent').set('link', links[element.find('parent').get('link')])
        element.find('child').set('link', links[element.find('child').get('link')])
        mimic = element.find('mimic')
        if mimic is not None:
            mimic.set('joint', joints[mimic.get('joint')])
    arm.append(element)

mount = ET.SubElement(arm, 'joint', name='r1a7_dex1_mount', type='fixed')
ET.SubElement(mount, 'parent', link='Link7')
ET.SubElement(mount, 'child', link='dex1_base_link')
ET.SubElement(mount, 'origin', xyz='0 -0.047736 0', rpy='0 0 3.141592653589793')

# Official Dex1 finger terminal joints put both pads at local y=0.097343 m,
# z=0.0142 m when the gripper is centered. +Z is approach; +Y is jaw width.
ET.SubElement(arm, 'link', name='r1a7_tcp')
tcp = ET.SubElement(arm, 'joint', name='r1a7_tcp_joint', type='fixed')
ET.SubElement(tcp, 'parent', link='dex1_base_link')
ET.SubElement(tcp, 'child', link='r1a7_tcp')
ET.SubElement(tcp, 'origin', xyz='0 0.097343 0.0142',
              rpy='0 -1.5707963267948966 -1.5707963267948966')

# The upstream URDFs use paths relative to their separate description roots.
# Isaac accepts absolute mesh paths; moveit.launch.py converts them to file://.
# Determine provenance from the containing link name, not the filename.
for link in arm.findall('link'):
    source = HAND.parent if link.get('name').startswith('dex1_') else ARM.parent
    for mesh in link.findall('.//mesh'):
        mesh.set('filename', str((source / mesh.get('filename')).resolve()))

ET.indent(arm)
OUT.write_text(ET.tostring(arm, encoding='unicode'))

srdf = ET.Element('robot', name='r1a7_dex1')
group = ET.SubElement(srdf, 'group', name='r1a7_arm')
ET.SubElement(group, 'chain', base_link='r1a7_world', tip_link='r1a7_tcp')
hand_group = ET.SubElement(srdf, 'group', name='dex1_hand')
for joint in ['dex1_Joint1_1', 'dex1_Joint2_1']:
    ET.SubElement(hand_group, 'joint', name=joint)
ET.SubElement(srdf, 'end_effector', name='dex1', parent_link='r1a7_tcp',
              group='dex1_hand', parent_group='r1a7_arm')
for a, b in [('base_link', 'Link1'), *[(f'Link{i}', f'Link{i+1}') for i in range(1, 7)],
             ('Link7', 'dex1_base_link'),
             ('dex1_base_link', 'dex1_Link1_1'), ('dex1_base_link', 'dex1_Link2_1'),
             ('dex1_Link1_1', 'dex1_Link1_2'), ('dex1_Link1_2', 'dex1_Link1_3'),
             ('dex1_Link2_1', 'dex1_Link2_2'), ('dex1_Link2_2', 'dex1_Link2_3'),
             ('dex1_Link1_3', 'dex1_Link2_3')]:
    ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b,
                  reason='Adjacent' if 'dex1_Link1_3' not in a else 'Never')
ET.indent(srdf)
SRDF.write_text(ET.tostring(srdf, encoding='unicode'))
print(OUT)
print(SRDF)
