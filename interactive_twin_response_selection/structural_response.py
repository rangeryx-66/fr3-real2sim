"""Complete response derivatives, without adding selection candidates or refitting."""
import copy
from pathlib import Path
import numpy as np
from interactive_twin_refinement.fitting import basis, unit


def complete(root, runner, eid, data, bank, space, specs, read, write, public_cell):
    from interactive_twin.twin import write_twins
    from interactive_twin_conditional.workflow import run_jobs
    from interactive_twin.visual_import import compatible_urdf
    center = next(s for s in data['structures'] if s['structure_id'] == 'observable_S0')
    e = read(center['estimate_path']); a = unit(e['axis_world']); c = np.array(e['point_on_axis_world_m']); B = basis(a)
    C = np.asarray(space['combined_covariance']); scales = np.sqrt(np.diag(C)); X = []
    for s in data['structures']:
        e = read(s['estimate_path']); aa = unit(e['axis_world']); aa *= 1 if aa@a >= 0 else -1
        cc = np.array(e['point_on_axis_world_m']); cc += aa*(a@(c-cc))/(a@aa)
        X.append(np.r_[B.T@(aa-a), B.T@(cc-c)]/scales)
    X = np.array(X); U, singular, Vt = np.linalg.svd(X, full_matrices=True)
    rank = int(np.linalg.matrix_rank(X, tol=1e-8))
    registration = {'input_rank': rank, 'dimension': 4, 'singular_values': singular.tolist(),
                    'maximum_derivative_structures': 8, 'physics_prior': {'tau_c': .012, 'b': 1.},
                    'selection_candidates_added': 0, 'fitter_called': False, 'test_read': False,
                    'amendment': 'rank-deficient response map must not silently discard known structure covariance'}
    extra = []; plans = []
    Cn = C/np.outer(scales, scales)
    for j, v in enumerate(Vt[rank:]):
        step = v*np.sqrt(v@Cn@v)*scales*space['candidate_scale']
        for sign in [-1., 1.]:
            d = sign*step; aa = unit(a+B@d[:2]); cc = c+B@d[2:]; cc -= aa*np.dot(aa, cc-c)
            sid = f'uncertainty_null_{j}_{"plus" if sign > 0 else "minus"}'
            fit = copy.deepcopy(space['candidates'][0]['estimate'])
            fit['revolute'].update(axis=aa.tolist(), point_on_axis=cc.tolist())
            plans.append({'structure_id': sid, 'delta': d.tolist(), 'estimate': fit})
    registration['perturbations'] = [{'structure_id': p['structure_id'], 'delta': p['delta']} for p in plans]
    out = runner.out/eid/'structure_response'
    reg = out/'registration.json'
    if reg.exists() and read(reg) != registration: raise ValueError('DERIVATIVE_REGISTRATION_CHANGED')
    write(reg, registration)
    if not plans: return data
    scene = read(Path(data['source_job']['output'])/'initial_scene_private.json')
    W = np.eye(4); W[:3, :3] = scene['asset_rotation']; W[:3, 3] = scene['asset_xyz']
    source = data['source_job']; support = read(source['initial_estimate_memory'])['supporting_observations']
    jobs = []
    train_specs = [s for s in specs if s['name'] != 'stop_dwell']
    for p in plans:
        folder = out/'models'/p['structure_id']
        if not (folder/'twin_versions.json').exists():
            write_twins(source['asset_root'], folder, p['estimate'], W, initial_physics_prior=registration['physics_prior'], ee_poses=support)
        asset = folder/'T1'; manifest = read(asset/'manifest.json')
        compatible_urdf(asset/'urdf'/f"{manifest['asset_id']}.urdf", asset/'visual_compatibility')
        s = {'structure_id': p['structure_id'], 'asset': str(asset), 'estimate_path': str(folder/'estimated_articulation.json')}
        extra.append(s)
        candidate = {**s, 'candidate_id': p['structure_id']+'_phi_008', 'tau_c': .012, 'b': 1., 'tau_s': .012, 'model_family': 'dynamic_viscous'}
        for spec in train_specs: jobs.append(runner.job(data, out/'bank'/candidate['candidate_id']/spec['name'], spec, candidate=candidate))
    run_jobs(runner, jobs)
    for s in extra:
        cid = s['structure_id']+'_phi_008'
        bank[cid] = {spec['name']: public_cell(out/'bank'/cid/spec['name'], spec['probe_id']) for spec in train_specs}
        if not all(cell['safe'] for cell in bank[cid].values()): raise ValueError('STRUCTURE_UNCERTAINTY_DERIVATIVE_CENSORED')
    # Selection uses the original data['candidates'] and original structures.
    return {**data, 'structures': data['structures']+extra}
