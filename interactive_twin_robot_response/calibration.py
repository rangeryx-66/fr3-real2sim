"""One shared response pole and latency, calibrated without object observations.

This is a prerequisite test, not an extra filter to apply to native predictions.
The native simulator already has robot dynamics; cascading this reduced model
onto its output would count robot response twice.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares
from scipy.signal import savgol_filter


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def extract(folder):
    """Export only robot state and actual controller inputs; never asset state."""
    folder = Path(folder)
    log = read(folder / 'observable/ROBOT_CALIBRATION.json')
    if log['split'] != 'calibration' or not log['provenance'].get('no_contact_supervisor_certified'):
        raise ValueError('INDEPENDENT_NO_CONTACT_CALIBRATION_REQUIRED')
    rows = read(folder / 'observations.json')
    tape = read(folder / 'command_tape.json')
    indices = [i for i, r in enumerate(rows) if r['phase'] == 'ROBOT_CALIBRATION']
    if len(indices) != len(log['time_s']) or indices[0] < 12:
        raise ValueError('MISSING_CALIBRATION_COMMAND_OR_INITIAL_STATE')
    if any(c['force_n'] > 0 for i in indices for c in rows[i]['contacts']):
        raise ValueError('CALIBRATION_CONTAINS_CONTACT')
    diag = [tape[i]['diagnostics'] for i in indices]
    refs = np.array([d['drive_reference_world_m'] for d in diag])
    dirs = np.array([d['direction_world'] for d in diag])
    if np.max(np.linalg.norm(dirs-dirs[0], axis=1)) > 1e-8:
        raise ValueError('CALIBRATION_DIRECTION_NOT_CONSTANT')
    dt = float(np.median(np.diff(log['time_s'])))
    # Legacy calibration tapes lack cartesian_input.active. Here the frozen
    # no-contact command increments reference on every active step; this is
    # certified against its actual recorded reference, not object motion.
    previous_ref = tape[indices[0]-1]['diagnostics']['drive_reference_world_m']
    active = np.linalg.norm(np.diff(np.vstack([previous_ref, refs]), axis=0), axis=1) > 1e-14
    pre = np.array([rows[i]['T_tcp'] for i in range(indices[0]-12, indices[0])])
    pre_t = np.arange(-11, 1)*dt
    v0 = np.polyfit(pre_t, pre[:, :3, 3], 1)[0]
    return {'source': str(folder), 'time_s': log['time_s'], 'dt_s': dt,
            'T': log['signals']['ee_T_world_tcp'], 'q_rad': log['signals']['q_rad'],
            'reference_world_m': refs.tolist(), 'direction_world': dirs[0].tolist(),
            'active': active.tolist(), 'initial_T': pre[-1].tolist(),
            'initial_velocity_m_s': v0.tolist(),
            'command_observation_timing': 'sample is after command integration; observer timestamp corrected by +dt in analysis only',
            'recovered_active': 'nonzero increments of recorded reference; no object state',
            'controller_id': log['provenance']['controller_id'],
            'robot_model_id': log['provenance']['robot_model_id'],
            'source_hashes': {name: sha(folder/name) for name in
                              ['observable/ROBOT_CALIBRATION.json', 'command_tape.json', 'observations.json']},
            'no_contact': True, 'GT_inputs': False}


def predict(record, parameters, config):
    """T v_dot + v = active*(K/D)*(reference-x), x_dot=v.

    K and D are frozen controller constants, not fitted gains. Inactive drive
    removes its stiffness, as the native controller does; it does not hold x.
    Delta delays only the exogenous command. One T covers start AND stop.
    """
    delay, pole = parameters
    d = np.asarray(record['direction_world'])
    origin = np.asarray(record['initial_T'])[:3, 3]
    reference = (np.asarray(record['reference_world_m'])-origin)@d
    t = np.asarray(record['time_s']); dt = record['dt_s']
    shifted = t-delay
    ix = np.searchsorted(t, shifted, side='right')-1
    active = np.asarray(record['active'])[np.maximum(ix, 0)] & (ix >= 0)
    target = np.interp(shifted, t, reference, left=0.)
    gain = config['K_n_m']/config['D_ns_m']
    transforms = {}
    for on in (False, True):
        augmented = np.zeros((3, 3))
        augmented[:2, :2] = [[0., 1.], [-gain/pole if on else 0., -1/pole]]
        augmented[1, 2] = gain/pole if on else 0.
        transforms[on] = expm(augmented*dt)
    x = np.array([0., np.asarray(record['initial_velocity_m_s'])@d, 1.])
    output = np.empty((len(t), 2))
    for i, (on, ref) in enumerate(zip(active, target)):
        x[2] = ref
        x = transforms[bool(on)]@x
        output[i] = x[:2]
    return output


def signals(record):
    d = np.asarray(record['direction_world'])
    p = np.asarray(record['T'])[:, :3, 3]
    x = (p-np.asarray(record['initial_T'])[:3, 3])@d
    t = np.asarray(record['time_s'])+record['dt_s']
    # Noise scales below use the same 25-sample window as frozen repeatability.
    v = savgol_filter(x, 25, 2, deriv=1, delta=record['dt_s'])
    return t, np.c_[x, v]


def scales(record, calibration):
    R = np.asarray(record['initial_T'])[:3, :3]
    d = R.T@np.asarray(record['direction_world'])
    covariance = np.asarray(calibration['response_covariance'])
    return np.sqrt([d@covariance[:3, :3]@d, d@covariance[6:, 6:]@d])


def fit(records, config, noise):
    if len(records) < 2 or any(not r['no_contact'] for r in records):
        raise ValueError('REPEATED_NO_CONTACT_INPUTS_REQUIRED')
    if len({r['controller_id'] for r in records}) != 1:
        raise ValueError('CONTROLLER_MISMATCH')
    lo, hi = config['train_window_s']
    data = [signals(r) for r in records]
    sigma = [scales(r, noise) for r in records]
    masks = [(t >= lo)&(t <= hi) for t, _ in data]
    def residual(x):
        return np.concatenate([((predict(r, x, config)-y)[m]/s).ravel()
                               for r, (t, y), s, m in zip(records, data, sigma, masks)])
    bounds = np.array([config['latency_bounds_s'], config['time_constant_bounds_s']]).T
    trials = [least_squares(residual, seed, bounds=bounds, diff_step=1e-3,
                            max_nfev=config['max_nfev_per_start'])
              for seed in config['initial_seeds_s']]
    best = min(trials, key=lambda s: np.mean(s.fun*s.fun))
    # This conditional covariance is deliberately not inflated by mismatch.
    # It measures local numerical identifiability, not total prediction error.
    signatures = {hashlib.sha256(np.asarray(r['T']).tobytes()).hexdigest() for r in records}
    # Bit-identical repeats confirm determinism; they are not independent data.
    jac = best.jac; covariance = np.linalg.pinv(jac.T@jac)*25.*len(records)/len(signatures)
    rows = evaluate(records, best.x, covariance, config, noise)
    return {'parameters': {'delta_t_s': float(best.x[0]), 'T_s': float(best.x[1])},
        'parameter_covariance': covariance.tolist(), 'jacobian_singular_values': np.linalg.svd(jac, compute_uv=False).tolist(),
        'multi_start': [{'x': s.x.tolist(), 'train_normalized_rms': float(np.sqrt(np.mean(s.fun*s.fun))),
                         'nfev': s.nfev, 'success': bool(s.success)} for s in trials],
        'records': rows, 'model_role': 'no-contact reduced response adequacy diagnostic; never cascaded onto native physics',
        'free_parameters': ['delta_t_s', 'T_s'], 'gain_fitted': False,
        'object_logs_used_for_fit': False, 'heldout_read': False,
        'effective_command_to_motion_delay_not_independent_communication_latency': True,
        'robot_calibration_coverage': 'one posture and direction, two identical-command repetitions',
        'covariance_limitation': 'conditional local parameter covariance; model bias is not calibration noise'}


def evaluate(records, parameters, covariance, config, noise):
    parameters = np.asarray(parameters)
    rows = []
    for r in records:
        t, y = signals(r); s = scales(r, noise)
        pred = predict(r, parameters, config)
        # Parameter uncertainty is propagated from training information only.
        step = np.array([1e-4, 1e-4])
        J = np.stack([(predict(r, parameters+np.eye(2)[i]*step[i], config)-pred)/step[i]
                      for i in range(2)], axis=-1)
        propagated = np.einsum('nki,ij,nkj->nk', J, covariance, J)
        total = np.sqrt(s*s+np.maximum(propagated, 0.))
        segments = {}
        for name, window in [('train', config['train_window_s']), ('validation', config['validation_window_s'])]:
            mask = (t > window[0])&(t <= window[1])
            active = np.asarray(r['active'])
            e = pred-y
            segments[name] = {'normalized_rms_repeat_only': float(np.sqrt(np.mean((e[mask]/s)**2))),
                'normalized_rms': float(np.sqrt(np.mean((e[mask]/total[mask])**2))),
                'position_rmse_m': float(np.sqrt(np.mean(e[mask, 0]**2))),
                'velocity_rmse_m_s': float(np.sqrt(np.mean(e[mask, 1]**2))),
                'active_position_rmse_m': float(np.sqrt(np.mean(e[mask&active, 0]**2))),
                'stop_position_rmse_m': float(np.sqrt(np.mean(e[mask&~active, 0]**2))),
                'stop_signed_mean_m': float(np.mean(e[mask&~active, 0])),
                'stop_final_error_m': float(e[np.flatnonzero(mask)[-1], 0])}
        rows.append({'source': r['source'], 'segments': segments, 'scales': s.tolist(),
                     'total_scales': total.tolist(),
                     'time_s': t.tolist(), 'measured': y.tolist(), 'predicted': pred.tolist()})
    return rows
