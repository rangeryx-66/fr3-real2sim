"""Evaluate command tracking independently from the waypoint look-ahead target."""
import json
from pathlib import Path
import numpy as np


def evaluate(output, model):
    output = Path(output)
    report = json.loads((output / 'report.json').read_text())
    rows = json.loads((output / 'observations.json').read_text())
    plan_path = output / 'aligned_whole_path.json'
    if not plan_path.exists():
        return {'executed_pull': False, 'stop': report['status']}
    arc = json.loads(plan_path.read_text())['known_model_whole_path']
    qpath = np.asarray([p['q'] for p in arc])
    states = np.asarray([p['state'] for p in arc])
    base = report['base_fixed']
    first = None
    peak_tcp = 0.
    first_state = None
    for row in rows:
        if row['phase'] not in ('OPEN_5_DEG', 'FORCE_HOLD', 'FINAL_HOLD') or row.get('planned_T_tcp') is None:
            continue
        command = np.asarray(row['command_q_arm'])
        expected = model.poses(command, base, width=row['aperture_m'])['tcp_link']
        actual = np.asarray(row['T_tcp'])
        error = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
        peak_tcp = max(peak_tcp, error)
        # Project the actual issued command onto the frozen dense reference.
        candidates = []
        for i in range(len(qpath)-1):
            dq = qpath[i+1]-qpath[i]
            f = float(np.clip(np.dot(command-qpath[i], dq)/max(np.dot(dq,dq),1e-20),0.,1.))
            candidates.append((np.linalg.norm(command-qpath[i]-f*dq), (1-f)*states[i]+f*states[i+1]))
        expected_state = float(min(candidates)[1]) if candidates else float(states[0])
        actual_state = float(row['door_angle_deg'])
        if report['actual_units']=='degrees':
            expected_state = float(np.rad2deg(expected_state))
        delta = abs(actual_state-expected_state)
        if error > .003 and first is None:
            first = dict(t=row['t'], phase=row['phase'], TCP_error_m=error,
                         actual_state=actual_state, issued_command_state=expected_state)
        if delta > (2. if report['actual_units']=='degrees' else .003) and first_state is None:
            first_state = dict(t=row['t'], phase=row['phase'], actual_state=actual_state,
                               issued_command_state=expected_state, error=delta)
    result = dict(executed_pull=True, mode=report['mode'],
                  first_command_actual_TCP_divergence=first,
                  first_command_actual_object_divergence=first_state,
                  maximum_TCP_command_tracking_error_m=peak_tcp,
                  note='Command FK and dense-path projection; distinct from waypoint look-ahead. Diagnostic only.')
    (output/'command_tracking_evaluation.json').write_text(json.dumps(result, indent=2))
    return result


def evaluate_zero_transition(output):
    """Locate the first deviation during the nominally stationary first point."""
    output = Path(output)
    rows = json.loads((output/'observations.json').read_text())
    begin = next((i for i,s in enumerate(rows) if s['phase']=='OPEN_5_DEG'), None)
    if begin is None or begin == 0:
        return {'zero_transition_present': False}
    initial = rows[begin-1]
    T0 = np.asarray(initial['T_tcp'])
    q0 = np.asarray(initial['command_q_arm'])
    first_command_change = first_actual_motion = first_unilateral_contact = None
    for s in rows[begin:]:
        target = s.get('planned_T_tcp')
        if target is None or np.linalg.norm(np.asarray(target)[:3,3]-T0[:3,3]) > 1e-5:
            break
        changed = float(np.max(abs(np.asarray(s['command_q_arm'])-q0)))
        moved = float(np.linalg.norm(np.asarray(s['T_tcp'])[:3,3]-T0[:3,3]))
        if changed>1e-5 and first_command_change is None:
            first_command_change = dict(t=s['t'],maximum_command_change_rad=changed)
        if moved>.0001 and first_actual_motion is None:
            first_actual_motion = dict(t=s['t'],TCP_motion_m=moved,planned_TCP_motion_m=0.)
        if min(s['forces_n'].values())<.05 and first_unilateral_contact is None:
            first_unilateral_contact = dict(t=s['t'],forces_n=s['forces_n'])
    result = dict(zero_transition_present=True,initial_t=initial['t'],
                  first_command_change=first_command_change,
                  first_actual_motion=first_actual_motion,
                  first_unilateral_contact=first_unilateral_contact,
                  definition='First nominal zero waypoint; 10 urad command change / 0.1mm measured motion. Descriptive provenance, no execution guard.')
    (output/'zero_transition_evaluation.json').write_text(json.dumps(result,indent=2))
    return result
