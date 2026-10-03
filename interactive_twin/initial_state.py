"""Preparation-only coordinate re-zeroing with equivalent physical mass state.

Reference assets are never changed. A twin prior can start at q=0 while its
solids coincide with the reference's configured initial q. This is a coordinate
conversion, not an online observation, estimated articulation, or object drive.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from articulated_demo.kinematics import URDFChain
from interactive_twin.twin import _absolute_resources, _numbers, _origin, _set_origin


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _physical(urdf, positions):
    chain = URDFChain(urdf)
    root = ET.parse(urdf).getroot()
    out = {}
    def canonical(node):
        return (node.tag, tuple(sorted(node.attrib.items())), tuple(canonical(c) for c in node))
    for link in root.findall('link'):
        name = link.get('name')
        frame = chain.root_to_link(name, positions)
        for kind in ('visual', 'collision', 'inertial'):
            for index, element in enumerate(link.findall(kind)):
                value = copy.deepcopy(element)
                if value.find('origin') is not None:
                    value.remove(value.find('origin'))
                # Resource spelling may become absolute; compare resolved identities.
                _absolute_resources(value, Path(urdf))
                if kind != 'inertial':
                    for mesh in value.findall('./geometry/mesh'):
                        filename = mesh.get('filename')
                        if filename and '://' not in filename:
                            mesh.set('filename', str((Path(urdf).parent / filename).resolve()))
                out[f'{name}/{kind}/{index}'] = (frame @ _origin(element.find('origin')), canonical(value))
    return out


def audit_rezero(source_urdf, baked_urdf, joint_name, q_initial):
    errors = []
    for offset in (0., -.013, .021):
        old = _physical(source_urdf, {joint_name: float(q_initial) + offset})
        new = _physical(baked_urdf, {joint_name: offset})
        if set(old) != set(new):
            raise AssertionError('REZERO_PHYSICAL_ELEMENT_SET_CHANGED')
        for key in old:
            if old[key][1] != new[key][1]:
                raise AssertionError('REZERO_PHYSICAL_PROPERTY_CHANGED:' + key)
            difference = np.linalg.inv(old[key][0]) @ new[key][0]
            errors.append((np.linalg.norm(difference[:3, 3]), Rotation.from_matrix(difference[:3, :3]).magnitude()))
    translation = max((e[0] for e in errors), default=0.)
    rotation = max((e[1] for e in errors), default=0.)
    if max(translation, rotation) > 1e-9:
        raise AssertionError('INITIAL_STATE_REZERO_NOT_PHYSICALLY_EQUIVALENT')
    return {'max_translation_error_m': float(translation), 'max_rotation_error_rad': float(rotation),
            'physical_frames_compared': len(errors), 'geometry_shape_scale_inertia_attributes_unchanged': True,
            'tested_coordinate_relation': 'q_original = q_initial + q_baked',
            'comparison_coordinate_offsets_rad': [0., -.013, .021]}


def _reference_geometry_mass_state(root, chain, manifest, q_initial):
    """Reproduce the frozen loader's geometry MassAPI override, then move once.

    The loader computes COM at original q=0 and assigns diagonal inertia in each
    body's local axes. Reference initialization subsequently rotates these actual
    bodies. Recomputing an AABB inertia at q_initial would be a different model.
    """
    scale = float(manifest['scale_source_to_meters'])
    bounds = np.asarray(manifest['moving_source_bounds'], dtype=float) * scale
    center, size = bounds.mean(0), bounds[1] - bounds[0]
    links = {node.get('name'): node for node in root.findall('link')}
    states = []
    for name in dict.fromkeys((manifest['moving_link'], manifest['door_link'])):
        mass = float(links[name].find('inertial/mass').get('value'))
        local_com = (np.linalg.inv(chain.root_to_link(name, {})) @ np.r_[center, 1.])[:3]
        diagonal = mass / 12 * np.array([size[1] ** 2 + size[2] ** 2,
                                         size[0] ** 2 + size[2] ** 2,
                                         size[0] ** 2 + size[1] ** 2])
        before = chain.root_to_link(name, {})
        after = chain.root_to_link(name, {manifest['joint_name']: float(q_initial)})
        states.append({'link': name, 'mass_kg': mass,
                       'center_of_mass_root_m': (after @ np.r_[local_com, 1.])[:3].tolist(),
                       'inertia_tensor_root_kg_m2': (after[:3, :3] @ np.diag(diagonal) @ after[:3, :3].T).tolist(),
                       'original_loader_com_local_m': local_com.tolist(),
                       'original_loader_diagonal_inertia_kg_m2': diagonal.tolist(),
                       'original_root_T_link_at_zero': before.tolist(),
                       'reference_root_T_link_at_initial': after.tolist()})
    return states


def bake_initial_articulation(asset_root, output, q_initial, *, mass_model='geometry'):
    """Return a separate q=0 prior physically equal to reference at q_initial.

    Use the returned asset with write_twins(..., q_initial=0). Reference/Oracle
    episodes retain the original asset and original q_initial. Twin jobs must set
    initial_articulation_rad=0 and invoke restore_baked_mass_properties at setup.
    """
    source = Path(asset_root).resolve()
    output = Path(output).resolve()
    q_initial = float(q_initial)
    if not math.isfinite(q_initial):
        raise ValueError('INITIAL_STATE_NOT_FINITE')
    if mass_model != 'geometry':
        raise ValueError('REZERO_CURRENTLY_REQUIRES_FROZEN_GEOMETRY_MASS_MODEL')
    if output == source or source in output.parents:
        raise ValueError('INITIAL_PRIOR_OUTPUT_MUST_BE_OUTSIDE_REFERENCE_ASSET')
    manifest_file = source / 'manifest.json'
    metadata = json.loads(manifest_file.read_text())
    if metadata.get('initial_state_bake'):
        raise ValueError('NESTED_INITIAL_STATE_BAKE_NOT_ALLOWED')
    urdf = source / 'urdf' / f"{metadata['asset_id']}.urdf"
    identity = {'source_manifest_sha256': _sha(manifest_file), 'source_urdf_sha256': _sha(urdf),
                'q_initial_rad': q_initial, 'mass_model': mass_model}
    if output.exists() and any(output.iterdir()):
        saved = output / 'initial_state_bake_private.json'
        if saved.is_file() and json.loads(saved.read_text()).get('identity') == identity:
            cached = json.loads(saved.read_text())
            if (_sha(output / 'manifest.json') != cached['output_manifest_sha256'] or
                    _sha(output / 'urdf' / urdf.name) != cached['output_urdf_sha256']):
                raise ValueError('INITIAL_STATE_BAKE_CACHE_CHANGED')
            return cached
        raise ValueError('INITIAL_STATE_PRIOR_EXISTS_PRESERVE_PREVIOUS_RESULTS')
    root = ET.parse(urdf).getroot()
    chain = URDFChain(urdf)
    movable = [joint for joint in root.findall('joint') if joint.get('type') != 'fixed']
    if len(movable) != 1 or movable[0].get('type') != 'revolute':
        raise ValueError('INITIAL_STATE_BAKE_REQUIRES_SINGLE_REVOLUTE')
    joint = movable[0]
    if joint.get('name') != metadata['joint_name']:
        raise ValueError('PREPARED_JOINT_NAME_MISMATCH')
    limit = joint.find('limit')
    lower, upper = float(limit.get('lower')), float(limit.get('upper'))
    if not lower - 1e-12 <= q_initial <= upper + 1e-12:
        raise ValueError('INITIAL_STATE_OUTSIDE_SOURCE_LIMIT')
    mass_state = _reference_geometry_mass_state(root, chain, metadata, q_initial)
    axis_node = joint.find('axis')
    axis = np.fromstring(axis_node.get('xyz', '1 0 0') if axis_node is not None else '1 0 0', sep=' ')
    axis /= np.linalg.norm(axis)
    rotation = np.eye(4)
    rotation[:3, :3] = Rotation.from_rotvec(axis * q_initial).as_matrix()
    _set_origin(joint, _origin(joint.find('origin')) @ rotation)
    limit.set('lower', format(lower - q_initial, '.17g'))
    limit.set('upper', format(upper - q_initial, '.17g'))
    _absolute_resources(root, urdf)
    (output / 'urdf').mkdir(parents=True)
    baked = output / 'urdf' / urdf.name
    ET.indent(root)
    ET.ElementTree(root).write(baked, encoding='utf-8', xml_declaration=True)
    audit = audit_rezero(urdf, baked, metadata['joint_name'], q_initial)
    # Keep closed geometry bounds as immutable preparation/camera metadata. The
    # loader's mass override is superseded once by the invariant records below.
    meta = copy.deepcopy(metadata)
    meta.update(prepared_urdf=str(baked), prepared_geometry_sha256=_sha(baked),
                source_joint_limits_rad={'lower': lower - q_initial, 'upper': upper - q_initial})
    meta['initial_state_bake'] = {
        'schema': 'prepared-initial-state-rezero-v1', 'setup_only_not_controller_input': True,
        'original_initial_articulation_rad': q_initial, 'effective_initial_articulation_rad': 0.,
        'reference_asset_root': str(source), 'identity': identity,
        'mass_model': mass_model, 'restore_mass_properties_before_world_reset': True,
        'mass_state_root': mass_state,
        'legacy_bounds_scope': 'immutable original closed visual metadata, not a new inertia estimate',
        'audit': audit, 'geometry_scaled': False, 'reference_modified': False,
    }
    # Visual setup metadata must describe the new initial shape, rather than the
    # old closed coordinates. These are handle geometry axes, never hinge output.
    interaction = meta.get('interaction_geometry', {})
    selection = interaction.get('selection')
    if selection:
        link = selection['handle_link']
        delta = chain.root_to_link(link, {metadata['joint_name']: q_initial}) @ np.linalg.inv(chain.root_to_link(link, {}))
        for key in ('axis_root', 'outward_normal_root'):
            if key in selection:
                selection[key] = (delta[:3, :3] @ np.asarray(selection[key])).tolist()
        for section in selection.get('sections', []):
            section['anchor_root_m'] = (delta @ np.r_[section['anchor_root_m'], 1.])[:3].tolist()
        centerline = interaction.get('PCA_centerline_root', {})
        if 'center_m' in centerline:
            centerline['center_m'] = (delta @ np.r_[centerline['center_m'], 1.])[:3].tolist()
        if 'axis' in centerline:
            centerline['axis'] = (delta[:3, :3] @ np.asarray(centerline['axis'])).tolist()
    (output / 'manifest.json').write_text(json.dumps(meta, indent=2) + '\n')
    # Pure algebra audit catches mass restore mistakes before any Isaac run.
    local = baked_mass_restore_spec(baked, meta)
    current = URDFChain(baked)
    max_com, max_inertia = 0., 0.
    for actual, expected in zip(local, mass_state):
        W = current.root_to_link(actual['link'], {})
        com = (W @ np.r_[actual['center_of_mass_local_m'], 1.])[:3]
        A = Rotation.from_quat(actual['principal_axes_xyzw']).as_matrix()
        inertia = W[:3, :3] @ A @ np.diag(actual['diagonal_inertia_kg_m2']) @ A.T @ W[:3, :3].T
        max_com = max(max_com, float(np.linalg.norm(com - np.asarray(expected['center_of_mass_root_m']))))
        max_inertia = max(max_inertia, float(np.max(np.abs(inertia - expected['inertia_tensor_root_kg_m2']))))
    if max_com > 1e-9 or max_inertia > 1e-9:
        raise AssertionError('BAKED_MASS_STATE_NOT_EQUIVALENT')
    result = {'asset_root': str(output), 'effective_initial_articulation_rad': 0.,
              'original_initial_articulation_rad': q_initial, 'identity': identity, 'audit': audit,
              'mass_state_audit': {'maximum_com_error_m': max_com, 'maximum_inertia_tensor_error_kg_m2': max_inertia},
              'requires_mass_restore_hook': True, 'reference_modified': False,
              'output_manifest_sha256': _sha(output / 'manifest.json'), 'output_urdf_sha256': _sha(baked)}
    (output / 'initial_state_bake_private.json').write_text(json.dumps(result, indent=2) + '\n')
    if _sha(manifest_file) != identity['source_manifest_sha256'] or _sha(urdf) != identity['source_urdf_sha256']:
        raise AssertionError('REFERENCE_MODIFIED_DURING_INITIAL_STATE_BAKE')
    return result


def baked_mass_restore_spec(asset_urdf, manifest):
    """Map invariant initial physical mass state into any T0/T1/T2 link frame."""
    bake = manifest.get('initial_state_bake')
    if not bake:
        return []
    chain = URDFChain(asset_urdf)
    rows = []
    for state in bake['mass_state_root']:
        frame = chain.root_to_link(state['link'], {})
        com = (np.linalg.inv(frame) @ np.r_[state['center_of_mass_root_m'], 1.])[:3]
        inertia = frame[:3, :3].T @ np.asarray(state['inertia_tensor_root_kg_m2']) @ frame[:3, :3]
        inertia = (inertia + inertia.T) / 2
        values, axes = np.linalg.eigh(inertia)
        if np.any(values <= 0.) or not np.isfinite(values).all():
            raise ValueError('INVALID_REFERENCE_INERTIA_NOT_REPAIRED')
        if np.linalg.det(axes) < 0:
            axes[:, -1] *= -1
        rows.append({'link': state['link'], 'mass_kg': state['mass_kg'],
                     'center_of_mass_local_m': com.tolist(), 'diagonal_inertia_kg_m2': values.tolist(),
                     'principal_axes_xyzw': Rotation.from_matrix(axes).as_quat().tolist(),
                     'inertia_tensor_local_kg_m2': inertia.tolist()})
    return rows


def restore_baked_mass_properties(stage, asset_path, asset_urdf, manifest):
    """One setup call AFTER legacy geometry mass override, BEFORE world.reset.

    Call for every twin asset; no-op for an unbaked reference. It authors only mass
    coordinates which were already physical properties of the reference setup.
    It never sets an articulation state or an execution-time object force.
    """
    from pxr import Gf, UsdPhysics
    records = baked_mass_restore_spec(asset_urdf, manifest)
    result = []
    for row in records:
        matches = [p for p in stage.Traverse() if str(p.GetPath()).startswith(str(asset_path))
                   and p.GetName() == row['link'] and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(matches) != 1:
            raise RuntimeError('BAKED_MASS_BODY_ASSOCIATION_FAILED:' + row['link'])
        api = UsdPhysics.MassAPI.Apply(matches[0])
        q = row['principal_axes_xyzw']
        api.CreateMassAttr().Set(float(row['mass_kg']))
        api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*row['center_of_mass_local_m']))
        api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*row['diagonal_inertia_kg_m2']))
        api.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))
        result.append({**row, 'body_path': str(matches[0].GetPath()), 'restored_once_before_reset': True})
    return result
