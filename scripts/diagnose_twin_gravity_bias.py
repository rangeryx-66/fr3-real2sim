"""Posthoc gravity bias from a joint estimate; never a controller/fitter input.

Matches the frozen loader's approximate moving-geometry COM and source masses.
This analytic simulation-model diagnostic is not measured external torque.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from articulated_demo.kinematics import URDFChain


def gravity_bias(asset, world, gravity_m_s2):
    manifest = json.loads((asset/'manifest.json').read_text())
    if manifest.get('initial_state_bake'):
        raise ValueError('Baked initial states require their restored mass state; this diagnostic supports unbaked zero-state assets only')
    urdf = asset/'urdf'/f"{manifest['asset_id']}.urdf"
    chain = URDFChain(urdf)
    joint = next(j for j in chain.joints.values() if j.name == manifest['joint_name'])
    frame = world @ chain.root_to_link(joint.parent, {}) @ joint.origin
    axis, point = frame[:3, :3] @ joint.axis, frame[:3, 3]
    center_root = (np.asarray(manifest['moving_source_bounds']).mean(0)
                   * manifest['scale_source_to_meters'])
    center = (world @ np.r_[center_root, 1.])[:3]
    xml = ET.parse(urdf).getroot()
    mass = sum(float(xml.find(f"link[@name='{name}']/inertial/mass").get('value'))
               for name in set((manifest['moving_link'], manifest['door_link'])))
    torque = float(axis @ np.cross(center-point, [0., 0., -gravity_m_s2*mass]))
    return {'axis_world': axis.tolist(), 'axis_point_world_m': point.tolist(),
            'geometry_COM_world_m': center.tolist(), 'moving_mass_kg': mass,
            'initial_gravity_generalized_torque_nm': torque}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-asset', type=Path, required=True)
    parser.add_argument('--estimated-asset', type=Path, required=True)
    parser.add_argument('--initial-scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gravity-m-s2', type=float, default=9.81)
    args = parser.parse_args()
    scene = json.loads(args.initial_scene.read_text())
    world = np.eye(4)
    world[:3, :3], world[:3, 3] = scene['asset_rotation'], scene['asset_xyz']
    rows = [{'group': label, **gravity_bias(asset, world, args.gravity_m_s2)}
            for label, asset in [('GT', args.reference_asset), ('estimated', args.estimated_asset)]]
    result = {'scope': 'posthoc analytic simulation-model diagnostic only; not measured torque; never controller/fitter input',
              'mass_model': 'frozen loader geometry COM approximation and unchanged source masses',
              'gravity_assumption_m_s2': args.gravity_m_s2, 'rows': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
