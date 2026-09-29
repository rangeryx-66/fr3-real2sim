#!/usr/bin/env python3
"""Offline description and sealed 40-scene input smoke; no physics claims."""
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from r1a7_calibration import load_calibration, width_to_finger_q


def main():
    calibration = load_calibration()
    model = ET.parse(ROOT / 'config/r1a7_dex1.urdf').getroot()
    joints = {j.get('name'): j for j in model.findall('joint')}
    for name, key in (('r1a7_dex1_mount', 'link7_to_dex1_base'),
                      ('r1a7_tcp_joint', 'dex1_base_to_tcp')):
        origin = joints[name].find('origin')
        for attribute, setting in (('xyz', 'xyz_m'), ('rpy', 'rpy_rad')):
            actual = [float(v) for v in origin.get(attribute).split()]
            expected = calibration[key][setting]
            assert all(abs(a-b) < 1e-10 for a, b in zip(actual, expected)), name
    for width in (.001067, .01, .05, .09):
        q = width_to_finger_q(width)
        assert -.02-1e-9 <= q <= .0245+1e-9
        assert math.isclose(width_to_finger_q(width), q)
    manifest = json.loads((ROOT / 'results/r1a7_arena_ab/input_manifest.json').read_text())
    rows = manifest['rows']
    assert len(rows) == 40 and len({r['seed'] for r in rows}) == 40
    assert len({r['target'] for r in rows}) == 8
    for row in rows:
        grasp_file = ROOT / f"results/r1a7_arena_ab/raw_grasps/seed_{row['seed']}_grasps.json"
        grasp = json.loads(grasp_file.read_text())
        assert len(grasp['grasps']) == row['grasp_count']
        assert grasp['T_B_C'] == row['camera_T_B_C']
        assert [g['rank'] for g in grasp['grasps']] == list(range(row['grasp_count']))
    print(json.dumps({'status': 'PASS', 'type': 'offline_static_smoke',
                      'official_model_joint_count': 13, 'scene_inputs': len(rows),
                      'object_classes': manifest['objects'],
                      'physics_execution': 'NOT_RUN'}))


if __name__ == '__main__':
    main()
