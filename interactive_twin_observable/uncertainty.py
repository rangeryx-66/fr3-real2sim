"""EE-only temporal/block uncertainty and local SE(3) information.

The 1 mm seed observability rule is inherited unchanged. Unavailable actions
are evidence about excitation, not a requirement to abort other actions.
"""
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from interactive_twin_refinement.fitting import fit_se3, predict_action, basis, unit
from interactive_twin_refinement.data import action_poses


def travel(T):
    T = np.asarray(T)
    return float(np.max(np.linalg.norm(T[:, :3, 3] - T[0, :3, 3], axis=1))) if len(T) else 0.


def observable(T):
    return len(T) >= 12 and travel(T) >= .001


def local_information(T, fit):
    """Finite-difference robust joint Hessian, profiling reference/phase.

    Covariance is a local approximation under the frozen noise model, not a
    hardware uncertainty certificate. Axis point has a two-coordinate gauge.
    """
    T = np.asarray(T)[fit['sample_indices']]
    a = unit(fit['revolute']['axis']); B = basis(a)
    c = np.asarray(fit['revolute']['point_on_axis'])
    ref = np.asarray(fit['reference_pose']); angles = np.asarray(fit['relative_angles_rad'])
    n = len(T); x = np.zeros(10+n-1)
    def residual(x):
        aa = unit(a+B@x[:2]); cc = c+B@x[2:4]
        cc -= aa*np.dot(aa, cc-c)
        p = ref[:3, 3]+x[4:7]
        R = Rotation.from_rotvec(x[7:10]).as_matrix()@ref[:3, :3]
        rr = Rotation.from_rotvec((angles+np.r_[0., x[10:]])[:, None]*aa).as_matrix()
        pp = cc+np.einsum('nij,j->ni', rr, p-cc)
        return np.c_[(pp-T[:, :3, 3])/fit['position_noise_m'],
                     Rotation.from_matrix((rr@R)@T[:, :3, :3].transpose(0, 2, 1)).as_rotvec()/fit['rotation_noise_rad']].ravel()
    r = residual(x); J = np.empty((len(r), len(x)))
    for j in range(len(x)):
        dx = x.copy(); dx[j] = 1e-6
        J[:, j] = (residual(dx)-residual(-dx))/(2e-6)
    # soft_l1 information weighting, matching the unchanged joint fitter.
    J *= (1+r*r)[:, None]**(-.75)
    Js, Jn = J[:, :4], J[:, 4:]
    S = Js-Jn@np.linalg.lstsq(Jn, Js, rcond=1e-10)[0]
    H = S.T@S
    cov = np.linalg.pinv(H, rcond=1e-12)
    return {'profile_information': H.tolist(), 'local_covariance': cov.tolist(),
            'singular_values': np.linalg.svd(S, compute_uv=False).tolist(),
            'nuisance_profiled': ['reference_pose', 'per_frame_phase'],
            'axis_line_gauge': 'plane through fitted axis point normal to fitted axis',
            'semantics': 'local robust Hessian approximation; not calibrated confidence interval'}


def build(actions, policy, center=None):
    usable = []; unavailable = []; data = []
    for action in actions:
        T, audit = action_poses(action)
        row = {**audit, 'travel_m': travel(T), 'observable': observable(T)}
        (usable if row['observable'] else unavailable).append(row)
        if row['observable']: data.append((action['phase'], T))
    if not data:
        return {'status': 'STRUCTURE_UNOBSERVABLE', 'unavailable': unavailable, 'candidates': []}
    # Longest complete existing action determines the reference; no reverse
    # prerequisite and no held-out dynamic action appears in this API.
    phase, train = max(data, key=lambda item: travel(item[1]))
    fit = center or fit_se3(train, **policy['fitting'])
    a = unit(fit['revolute']['axis']); B = basis(a); c = np.asarray(fit['revolute']['point_on_axis'])
    blocks = np.array_split(np.arange(len(train)), policy['temporal_blocks'])
    samples = [(f'time_segment_{i}', train[idx]) for i, idx in enumerate(blocks)]
    for i in range(len(blocks)):
        idx = np.concatenate([b for j, b in enumerate(blocks) if j != i])
        samples.append((f'leave_segment_{i}_out', train[idx]))
    rng = np.random.default_rng(policy['seed'])
    for i in range(policy['bootstrap_samples']):
        ids = rng.integers(0, len(blocks), len(blocks))
        # Ordered block bootstrap preserves SE(3) phase ordering and correlation.
        idx = np.sort(np.concatenate([blocks[j] for j in ids]))
        samples.append((f'block_bootstrap_{i}', train[idx]))
    samples.extend((name, T) for name, T in data if name != phase)
    vectors = []; records = []
    for name, T in samples:
        rec = {'name': name, 'travel_m': travel(T)}
        if not observable(T):
            rec['status'] = 'UNAVAILABLE_BELOW_UNCHANGED_1MM_OR_SAMPLE_THRESHOLD'
        else:
            try:
                f = fit_se3(T, **policy['fitting']); aa = unit(f['revolute']['axis'])
                aa *= 1 if aa@a >= 0 else -1
                cc = np.asarray(f['revolute']['point_on_axis']); cc += aa*(a@(c-cc))/(a@aa)
                v = np.r_[B.T@(aa-a), B.T@(cc-c)]; vectors.append(v)
                rec.update(status='FITTED', delta=v.tolist(), fit=f)
            except (ValueError, KeyError, np.linalg.LinAlgError) as err:
                rec.update(status='UNAVAILABLE_FIT', reason=str(err))
        records.append(rec)
    if not vectors:
        return {'status': 'UNCERTAINTY_UNOBSERVABLE', 'candidates': [], 'segments': records}
    info = local_information(train, fit)
    V = np.asarray(vectors)
    # Scatter around the common center includes bias/segment disagreement.
    empirical = V.T@V/len(V)
    cov = empirical + np.asarray(info['local_covariance'])
    scales = np.sqrt(np.maximum(np.diag(cov), 1e-16))
    eig, eigvec = np.linalg.eigh(cov/np.outer(scales, scales))
    deltas = [np.zeros(4)]
    for j in np.argsort(eig)[-2:][::-1]:
        d = eigvec[:, j]*np.sqrt(max(0., eig[j]))*scales*policy['scatter_fraction']
        deltas.extend([d, -d])
    reference_scores = [predict_action(T, fit)['normalized_rmse'] for _, T in data]
    candidates = []
    for i, d in enumerate(deltas):
        f = copy.deepcopy(fit); aa = unit(a+B@d[:2]); cc = c+B@d[2:]
        cc -= aa*np.dot(aa, cc-c)
        f['revolute'].update(axis=aa.tolist(), point_on_axis=cc.tolist())
        scores = [predict_action(T, f) for _, T in data]
        accepted = all(s['normalized_rmse'] <= max(r, 1.)*policy['maximum_training_ratio'] for s, r in zip(scores, reference_scores))
        candidates.append({'structure_id': f'S{i}', 'estimate': f, 'accepted': bool(accepted),
                           'delta': d.tolist(), 'action_prediction': scores})
    return {'status': 'COMPLETE', 'center_action': phase, 'usable_actions': usable,
            'unavailable_actions': unavailable, 'segments': records, 'local_information': info,
            'empirical_covariance': empirical.tolist(), 'combined_covariance': cov.tolist(),
            'candidate_scale': policy['scatter_fraction'], 'candidates': candidates,
            'ground_truth_read': False, 'reverse_required': False,
            'semantics': 'bounded local empirical distribution, not exact physical parameter confidence'}
