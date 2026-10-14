"""Offline contact forecast versus recorded contact. Never imported by actors."""
from pathlib import Path
import bisect
import gzip
import json
import pickle
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'results/native_g1_visual_reliability_20261015'
PREVIOUS = ROOT / 'results/native_g1_visual_input_20261014'


def mm(value):
    return None if value is None else value * 1000


def records(folder, name):
    index = folder / (name + '_chunks/index.json')
    if not index.exists():
        return []
    result = []
    for chunk in json.loads(index.read_text())['chunks']:
        with gzip.open(folder / (name + '_chunks') / chunk['file'], 'rb') as f:
            result.extend(pickle.load(f))
    return result


def summarize(folder):
    if not (folder / 'report.json').exists():
        return {'status': 'NOT_EXPORTED'}
    report = json.loads((folder / 'report.json').read_text())
    result = {'status': report['status'], 'grasp_acquisition': report.get('grasp_acquisition')}
    if (folder / 'visual_plan.json').exists():
        audit = json.loads((folder / 'visual_plan.json').read_text())['contact_geometry_audit']
        robust = audit['robust_contact']
        result['observed_model_forecast'] = {
            'expected_aperture_mm': robust['chosen_expected_aperture_m'] * 1000,
            'first_contact': robust.get('frozen_initial_contact_feature'),
            'first_contacts': audit.get('first_contacts'),
            'note': 'Approximate observed-surface forecast, not physical contact certification. Legacy native score label refers to the observed proxy in visual mode.'}
    commands = records(folder, 'closure_commands')
    observations = records(folder, 'observations')
    physics = records(folder, 'physics_steps')
    command_times = [r['t'] for r in commands]
    result['first_actual_positive_normal_contact'] = {}
    for finger in ['gripper_link1', 'gripper_link2']:
        row = next((r for r in physics if any(c.get('finger') == finger and c.get('allowed_pad_target') and c.get('force_n', 0) > 0 for c in r.get('contacts', []))), None)
        if row is None:
            result['first_actual_positive_normal_contact'][finger] = None
            continue
        i = min(max(bisect.bisect_right(command_times, row['t']) - 1, 0), len(commands) - 1)
        command = commands[i] if commands else {}
        contacts = [c for c in row['contacts'] if c.get('finger') == finger and c.get('allowed_pad_target') and c.get('force_n', 0) > 0]
        result['first_actual_positive_normal_contact'][finger] = {
            't_s': row['t'], 'phase': row['phase'], 'measured_aperture_mm': row['aperture_m'] * 1000,
            'commanded_aperture_mm': mm(command.get('aperture_command_m')),
            'normal_load_n': sum(c['force_n'] for c in contacts),
            'contacts': [{k: c.get(k) for k in ['contact_point_world_m', 'native_normal_world', 'target', 'collider', 'owner']} for c in contacts]}
    states = {}
    for row in commands:
        states.setdefault(row['state'], dict(t_s=row['t'], commanded_aperture_mm=mm(row['aperture_command_m']), measured_aperture_mm=mm(row['measured_aperture_m']), filtered_pad_loads_n=row['filtered_pad_loads_n']))
    result['first_closure_state_records'] = states
    result['aperture_command_reopenings'] = sum(bool(r['command_reopening']) for r in commands)
    if commands:
        last = commands[-1]
        result['final_closure'] = {k: last.get(k) for k in ['t', 'state', 'aperture_command_m', 'measured_aperture_m', 'filtered_pad_loads_n']}
    if observations:
        result['final_observation'] = {k: observations[-1].get(k) for k in ['t', 'phase', 'door_angle_deg', 'aperture_m', 'forces_n', 'relative_translation_slip_m', 'relative_rotation_slip_deg']}
        closing = [r for r in observations if r['phase'] in ['SLOW_CLOSE', 'CENTER', 'LOW_PRELOAD', 'FORCE_HOLD']]
        positive = [r for r in closing if any(c.get('allowed_pad_target') and c.get('force_n', 0) > 0 for c in r.get('contacts', []))]
        if positive:
            start = positive[0]
            relative = lambda r: np.linalg.inv(np.asarray(r['T_moving_link'])) @ np.asarray(r['T_tcp'])
            reference = relative(start)
            after = [r for r in closing if r['t'] >= start['t']]
            result['closure_relative_motion_from_first_contact'] = dict(
                start_s=start['t'],
                max_translation_mm=max(float(np.linalg.norm(relative(r)[:3, 3] - reference[:3, 3])) * 1000 for r in after),
                definition='Measured moving-body-relative TCP change during closure, including original centering. Not the post-acquisition slip metric or a new gate.')
    result['interpretation_limit'] = 'First positive normal contact may be transient. Full finger task-region ownership is not a measured pad pressure area or complete contact wrench.'
    return result


if __name__ == '__main__':
    result = {asset: {'previous_visual': summarize(PREVIOUS / asset / 'physical_01'), 'updated_visual': summarize(BASE / asset / 'physical_01')} for asset in ['45600', '45606', '45671']}
    (BASE / 'CONTACT_MISMATCH.json').write_text(json.dumps(result, indent=2))
