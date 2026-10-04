"""Train-only propagation; never estimates scales from candidate residuals."""
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
from interactive_twin_refinement.fitting import basis, unit
from interactive_twin_selection.metrics import active_mask, onset


def features(log, anchor):
    t = np.asarray(log['time_s']); T = np.asarray(log['signals']['ee_T_world_tcp'])
    R = np.asarray(anchor['signals']['ee_T_world_tcp'])[:, :3, :3]
    p = T[:, :3, 3]; rot = Rotation.from_matrix(T[:, :3, :3]@R.transpose(0, 2, 1)).as_rotvec()
    local = lambda x: np.einsum('nji,nj->ni', R, x)
    return np.c_[local(p-np.asarray(anchor['signals']['ee_T_world_tcp'])[:, :3, 3]),
                 local(rot), local(np.gradient(p, t, axis=0))]


def propagate(data, bank, space, repeat, robot, read, specs):
    center = next(s for s in data['structures'] if s['structure_id'] == 'observable_S0')
    est = read(center['estimate_path']); a = unit(est['axis_world'])
    c = np.asarray(est['point_on_axis_world_m']); B = basis(a)
    C = np.asarray(space['combined_covariance'])
    scales = np.sqrt(np.diag(C)); X = []; ids = []
    for s in data['structures']:
        f = read(s['estimate_path']); aa = unit(f['axis_world']); aa *= 1 if aa@a >= 0 else -1
        cc = np.asarray(f['point_on_axis_world_m']); cc += aa*(a@(c-cc))/(a@aa)
        X.append(np.r_[B.T@(aa-a), B.T@(cc-c)]/scales); ids.append(s['structure_id'])
    X = np.asarray(X); inverse = np.linalg.pinv(X, rcond=1e-8)
    normalized_C = C/np.outer(scales, scales)
    # The old covariance lives in exactly this fitted-axis basis and gauge.
    # Full-rank native structural secants propagate assembly/gravity response,
    # not merely an ideal arc. Physics is held at the same frozen wrong prior.
    if np.linalg.matrix_rank(X, tol=1e-8) < 4:
        raise ValueError('INCOMPLETE_STRUCTURE_UNCERTAINTY_RESPONSE_SPAN')
    struct = []; nuisance = []; delays = []; audits = []
    for spec in specs:
        if spec['name'] == 'stop_dwell': continue
        logs = [bank[s+'_phi_008'][spec['name']]['log'] for s in ids]
        anchor = logs[ids.index('observable_S0')]
        if any(x['split'] == 'heldout' for x in logs): raise ValueError('HELDOUT_SCALE_CALIBRATION_FORBIDDEN')
        F = np.asarray([features(log, anchor) for log in logs])
        derivative = np.einsum('ks,snj->njk', inverse, F-F[ids.index('observable_S0')])
        propagated = np.einsum('nik,kl,njl->nij', derivative, normalized_C, derivative)
        struct.append(propagated.mean(0))
        t = np.asarray(anchor['time_s']); T = np.asarray(anchor['signals']['ee_T_world_tcp'])
        R = T[:, :3, :3]; v = np.gradient(T[:, :3, 3], t, axis=0)
        omega = np.gradient(Rotation.from_matrix(R@R[0].T).as_rotvec(), t, axis=0)
        local = lambda x: np.einsum('nji,nj->ni', R, x)
        # Low-frequency derivative of an added delay / single response pole.
        # Native dynamics already reproduce the calibrated nominal response;
        # nominal correction is ZERO, avoiding a second servo filter.
        h = -np.c_[local(v), local(omega), local(np.gradient(v, t, axis=0))]
        J = np.stack([h, h], axis=-1)
        nuisance.append(np.einsum('nik,kl,njl->nij', J, np.asarray(robot['parameter_covariance']), J).mean(0))
        onset_values = []
        for log in logs:
            p = np.asarray(log['signals']['ee_T_world_tcp'])[:, :3, 3]
            value = onset(t, p, active_mask(log), repeat['onset_threshold_m'], repeat['onset_hold_s'])
            onset_values.append(value)
        valid = [i for i, x in enumerate(onset_values) if x is not None]
        if len(valid) >= 3:
            yy = np.asarray([onset_values[i] for i in valid]); xx = X[valid]
            beta = np.linalg.pinv(np.c_[np.ones(len(valid)), xx])@yy
            delays.append(float(beta[1:]@normalized_C@beta[1:]))
        audits.append({'action': spec['name'], 'physics_fixed': {'tau_c': .012, 'b': 1.},
                       'structural_points': len(logs), 'observed_reference_response_used': False})
    S = np.mean(struct, axis=0); N = np.mean(nuisance, axis=0); R = np.asarray(repeat['response_covariance'])
    result = dict(repeat)
    result.update(response_covariance=(R+S+N).tolist(),
        start_time_scale_s=float(np.sqrt(repeat['start_time_scale_s']**2+np.mean(delays or [0.])+robot['parameter_covariance'][0][0])),
        uncertainty_components={'repeat': R.tolist(), 'structure': S.tolist(), 'robot': N.tolist()},
        propagated_structure_rank=int(np.linalg.matrix_rank(X, tol=1e-8)),
        structure_parameter_dimension=4, covariance_source='existing EE-only train bootstrap/Hessian',
        validation_or_test_response_used=False, native_robot_filter_added=False,
        nominal_robot_delay_correction_s=0., q_is_diagnostic_only=True,
        propagation_audit=audits,
        limitations=['native secants are a local approximation at frozen wrong physics prior',
                     'all four structural covariance directions require native response derivatives',
                     'robot pole covariance uses low-frequency sensitivity; no model bias inflation'])
    result['calibration_sha256'] = hashlib.sha256(json.dumps(result, sort_keys=True).encode()).hexdigest()
    return result
