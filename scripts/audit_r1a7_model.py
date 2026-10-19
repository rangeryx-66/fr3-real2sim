#!/usr/bin/env python3
"""Static provenance/geometry audit of the generated R1-7a + Dex1 URDF."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'arm': ROOT/'third_party/unitree_ros/robots/r1_7a_description/R1_7a.urdf',
    'hand': ROOT/'third_party/unitree_ros/robots/dexterous_hand_description/dex1_1/dex1_1.urdf',
}


def joint_data(j):
    return {k: j.find(k).attrib if j.find(k) is not None else None
            for k in ('origin', 'axis', 'limit', 'dynamics', 'mimic')}


def main(output):
    combined = ET.parse(ROOT/'config/r1a7_dex1.urdf').getroot()
    joints = {j.get('name'): j for j in combined.findall('joint')}
    links = {link.get('name'): link for link in combined.findall('link')}
    report = {'official_commit': 'ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee',
              'sources': {name: str(path) for name, path in SOURCES.items()},
              'official_joints_preserved': {}, 'links': {}, 'mesh_paths_missing': [],
              'custom_fixed_joints': {}, 'all_passed': True}
    for source, path in SOURCES.items():
        original = ET.parse(path).getroot()
        for j in original.findall('joint'):
            name = ('dex1_' if source == 'hand' else '') + j.get('name')
            equal = name in joints and joint_data(j) == joint_data(joints[name])
            report['official_joints_preserved'][name] = equal
            report['all_passed'] &= equal
        for link in original.findall('link'):
            name = ('dex1_' if source == 'hand' else '') + link.get('name')
            current = links.get(name)
            if current is None:
                report['all_passed'] = False
                continue
            same_inertial = ET.canonicalize(ET.tostring(link.find('inertial')), strip_text=True) == ET.canonicalize(ET.tostring(current.find('inertial')), strip_text=True)
            data = {'inertial_preserved': same_inertial,
                    'visual_count': len(current.findall('visual')),
                    'collision_count': len(current.findall('collision'))}
            report['links'][name] = data
            report['all_passed'] &= data['inertial_preserved']
    for name in ('r1a7_world_mount', 'r1a7_dex1_mount', 'r1a7_tcp_joint'):
        report['custom_fixed_joints'][name] = joint_data(joints[name])
    for mesh in combined.findall('.//mesh'):
        if not Path(mesh.get('filename')).is_file():
            report['mesh_paths_missing'].append(mesh.get('filename'))
    report['all_passed'] &= not report['mesh_paths_missing']
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'all_passed': report['all_passed'],
                      'official_joint_count': len(report['official_joints_preserved']),
                      'link_count': len(report['links']),
                      'missing_meshes': len(report['mesh_paths_missing'])}))
    if not report['all_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, default=ROOT/'results/r1a7_model_audit.json')
    main(p.parse_args().output)
