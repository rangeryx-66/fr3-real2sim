"""Closed-study GT audit and offline force reference; never a policy input."""
import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def closed_episodes(root, cases):
    """Check every owner before opening any authored/native GT file."""
    gate = root / 'FORMAL_GATE_SEALED.json'
    if not read(gate).get('no_further_physical_actions_in_this_study'):
        raise ValueError('ONLINE_DECISIONS_NOT_SEALED')
    episodes = {}
    for label, name in cases.items():
        folder = root / 'runs' / name
        episode = folder / 'episodes' / name
        if not read(episode / 'PROCESS_COMPLETION.json').get('actor_exited'):
            raise ValueError('ACTOR_NOT_CLOSED:' + name)
        if not (folder / 'EVIDENCE_SEALED.json').is_file():
            raise ValueError('RAW_EVIDENCE_NOT_SEALED:' + name)
        if Path('/proc', str(read(episode / 'PROCESS.json')['pid'])).exists():
            raise ValueError('OWNER_PID_STILL_PRESENT:' + name)
        episodes[label] = episode
    return episodes


def native_moving_mass(truth, tensors):
    """Use named authored moving bodies, never the sum of the whole asset."""
    if not tensors['get_masses']['available']:
        raise ValueError('NATIVE_OBJECT_MASSES_UNAVAILABLE')
    names = tensors['link_names']
    masses = np.asarray(tensors['get_masses']['values'], float)[0]
    selected = {b['body']: float(b['mass_kg']) for b in truth['bodies']}
    actual = {name: float(masses[names.index(name)]) for name in selected}
    if not np.allclose(list(actual.values()), list(selected.values()), rtol=2e-6, atol=1e-7):
        raise ValueError('NATIVE_AUTHORED_MASS_MISMATCH')
    return sum(actual.values()), actual


def parameter_errors(model, truth):
    constant = model.get('constant_model')
    if not constant:
        return dict(available=False, reason='NO_ORIGINAL_FIT_MODEL')
    beta = np.asarray(constant['parameters'], float)
    gt = np.asarray(truth, float)
    std = np.asarray(constant['conditional_std'], float)
    return dict(available=True, parameters=beta.tolist(),
        absolute_error=abs(beta-gt).tolist(),
        relative_error_percent=(100*abs(beta-gt)/abs(gt)).tolist(),
        conditional_std=std.tolist(),
        conditional_95_interval=np.c_[beta-1.96*std, beta+1.96*std].tolist(),
        GT_inside_conditional_95=(abs(beta-gt)<=1.96*std).tolist(),
        uncertainty_is_not_calibrated_total_physical_confidence=True)


def evaluate(root, cases):
    root = Path(root)
    episodes = closed_episodes(root, cases)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    from evaluate_cross_object_v7 import authored_inertial_reference
    menu = read(root / 'models/COMMON_MENU.json')
    rows = []
    accessed = time.time()
    for label, p in episodes.items():
        truth = read(p / 'HIDDEN_AUTHORED_PHYSICS.json')
        case = truth['configuration']
        if case.get('spring') or case['law']['amplitude'] != 0:
            raise ValueError('STATIONARY_UNSPRUNG_RAIL_REFERENCE_ONLY')
        mass, extra = authored_inertial_reference(truth, read(p/'job.json')['asset_root'], 'prismatic')
        if extra:
            raise ValueError('UNRANDOMIZED_MOVING_MASS_REQUIRES_SEPARATE_ACCOUNTING')
        native_mass, bodies = native_moving_mass(truth, read(p/'PRIVATE_NATIVE_OBJECT_TENSORS.json'))
        if not np.isclose(mass, native_mass, rtol=2e-6):
            raise ValueError('MOVING_MASS_REFERENCE_MISMATCH')
        passive = read(p/'PRIVATE_STUDY_PASSIVE_READBACK.json')
        prop = np.asarray(passive['native_properties'])[0, passive['selected_dof']]
        fc, damping = case['tau_c'], case['b']
        fs = fc * case.get('static_rail_multiplier', 1.)
        if passive['object_drive_or_external_effort_added'] or not np.allclose(prop, [fs, fc, damping], rtol=2e-6):
            raise ValueError('NATIVE_PASSIVE_TREATMENT_MISMATCH')
        material = case['contact_material']
        material_reads = read(p/'PRISMATIC_CONTACT_MATERIAL_READBACK.json')
        checked = []
        for name, data in material_reads.items():
            if not data.get('available'):
                continue
            expected = [material['static_friction'], material['dynamic_friction'], 0.]
            if not np.allclose(data['raw_native_material_properties'], expected, rtol=2e-6, atol=1e-7):
                raise ValueError('NATIVE_CONTACT_MATERIAL_MISMATCH:' + name)
            checked.append(name)
        if not {'left_finger', 'right_finger'}.issubset(checked):
            raise ValueError('BOTH_NATIVE_PAD_MATERIAL_READBACKS_REQUIRED')
        grid = [dict(reference_speed_m_s=v, reference_acceleration_m_s2=a,
            moving_force_N=mass*a+damping*v+fc,
            conservative_start_and_motion_force_N=max(fs, mass*a+damping*v+fc))
            for v in menu['reference_speeds_m_s'] for a in menu['reference_accelerations_m_s2']]
        rows.append(dict(label=label, episode=p.name,
            authored_GT=dict(moving_mass_kg=mass, damping_N_s_m=damping,
                rail_dynamic_friction_N=fc, rail_static_friction_N=fs,
                pad_handle_material=material), native_moving_bodies_kg=bodies,
            native_material_readbacks_checked=checked,
            native_mass_and_passive_readback_confirmed=True,
            model_usable_before_GT=read(root/'models'/label/'MEASURED_INPUT.json')['model_usable'],
            parameters=parameter_errors(read(p/'model/estimate.json'), [mass, damping, fc]),
            offline_oracle_force_grid=grid,
            raw_seal_sha256=digest(p.parents[1]/'EVIDENCE_SEALED.json')))
    return dict(schema='physics-aware-posthoc-reference/v1', rows=rows,
        GT_evaluation_started_unix_s=accessed, all_owners_closed_before_GT=True,
        control_gate_sha256=digest(root/'FORMAL_GATE_SEALED.json'),
        source_sha=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        evaluator_sha256=digest(__file__), models_refitted=False,
        GT_used_for_online_controls=False,
        oracle_scope='offline stationary-rail effort reference, not an executed control or success bound',
        mass_scope='authored selected moving bodies checked against native object tensors; not a separate effective-dynamics experiment',
        contact_friction_scope='authored native material; used traction ratios are lower bounds, not friction point estimates')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--case', action='append', required=True, help='MODEL_LABEL=EPISODE_ID')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.root, dict(item.split('=', 1) for item in args.case))
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output), cases=len(result['rows']), refitted=False)))
