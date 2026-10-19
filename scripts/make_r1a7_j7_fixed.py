#!/usr/bin/env python3
"""Diagnostic ablation: lock Unitree J7 at zero, preserving every other link."""
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
source = ROOT/'config/r1a7_dex1.urdf'
target = ROOT/'config/r1a7_dex1_j7_fixed.urdf'
robot = ET.parse(source).getroot()
joint = next(j for j in robot.findall('joint') if j.get('name') == 'J7')
joint.set('type', 'fixed')
for tag in ('axis', 'limit', 'dynamics', 'safety_controller'):
    child = joint.find(tag)
    if child is not None:
        joint.remove(child)
ET.indent(robot)
target.write_text(ET.tostring(robot, encoding='unicode'))
print(target)
