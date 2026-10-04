"""Reuse native contact replay and batch selection; add bounded continuous fits."""
import concurrent.futures
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from .uncertainty import propagate


def run(root, config, read, write):
    sys.path.insert(0, str(root/'scripts'))
    from run_interactive_twin_selection import Experiment, public_cell
    from interactive_twin_selection.metrics import score
    from interactive_twin_selection.policy import rank
    from interactive_twin_conditional.workflow import run_jobs
    out = root/config['output']; old = root/config['prior_output']
    robot = read(out/'robot_calibration.json')
    if not (robot['numerical_adequacy_pass'] and robot['cross_asset_robot_validation']):
        raise ValueError('ROBOT_CALIBRATION_PREREQUISITE_FAILED')
    nc = read(old/'frozen_config.json')
    nc.update(output=str(out/'physics'), gpus=config['gpus'], deadline_shanghai=config['deadline_shanghai'])
    runner = Experiment(nc)
    repeat = read(old/'calibration/selection_noise.json')
    all_rows = []; decisions = []
    for eid in config['episodes']:
        dest = runner.out/eid; dest.mkdir(parents=True, exist_ok=True)
        data = read(old/eid/'prepared.json')
        data['source_job'].update(deadline_shanghai=config['deadline_shanghai'], wall_clock_budget_s=config['native_wall_clock_budget_s'])
        for spec in config['heldout']:
            data['command_tapes'][spec['name']] = str(out/eid/'commands'/(spec['name']+'.json'))
        base = data['candidates']
        bank = {}
        for c in base:
            folder = (Path(data['prior_output'])/'bank'/c['candidate_id'].removeprefix('observable_')
                      if c['structure_id'].startswith('observable_') else old/eid/'bank'/c['candidate_id'])
            bank[c['candidate_id']] = {s['name']: public_cell(folder/s['name'], s['probe_id']) for s in nc['probes']}
        inventory = read(old/eid/'available_probes.json')['conditions']
        refs = {condition: {r['name']: public_cell(r['path'], next(s['probe_id'] for s in nc['probes'] if s['name'] == r['name']))['log']
                            for r in items if r['usable']} for condition, items in inventory.items()}
        space = read(Path(data['prior_output'])/'structure_candidates.json')
        calibration = propagate(data, bank, space, repeat, robot, read, nc['probes'])
        write(dest/'total_uncertainty.json', calibration)
        initial = {condition: rank(base, bank, rr, calibration, nc['selection']) for condition, rr in refs.items()}
        write(dest/'grid_reselection.json', initial)
        static = read(old/eid/'static_grid_frozen.json')['candidates']
        for c in static:
            bank[c['candidate_id']] = {s['name']: public_cell(old/eid/'static_bank'/c['candidate_id']/s['name'], s['probe_id']) for s in nc['probes']}
        # Native responses are cached per physics point. No candidate is pruned.
        def optimize(condition, family, ranking):
            budget = config['continuous_optimization']; anchors = []
            for c in sorted(ranking['rows'], key=lambda r: (r['train_loss'], r['candidate_id'])):
                if c['structure_id'] not in {a['structure_id'] for a in anchors}: anchors.append(c)
                if len(anchors) >= budget['maximum_starts_per_family']: break
            frozen = dest/'continuous'/condition/family/'registration.json'
            registration = {'anchors': [{k: r[k] for k in ['candidate_id', 'structure_id', 'tau_c', 'b', 'tau_s']} for r in anchors],
                            'budget': budget, 'objective': 'train actions only', 'test_read': False,
                            'method': 'bounded normalized Nelder-Mead, initial simplex side 0.1',
                            'candidate_score': 'unchanged complete-action batch policy'}
            if frozen.exists() and read(frozen) != registration: raise ValueError('OPTIMIZER_REGISTRATION_CHANGED')
            write(frozen, registration)
            def one_start(anchor, index):
                candidates = {}; trace = []
                limits = np.array([budget['tau_c_bounds'], budget['b_bounds']]+([budget['tau_s_excess_bounds']] if family == 'M3' else []))
                lower, upper = limits[:, 0], limits[:, 1]
                seed = np.array([anchor['tau_c'], anchor['b']]+([anchor['tau_s']-anchor['tau_c']] if family == 'M3' else []))
                x0 = np.clip((seed-lower)/(upper-lower), 0, 1)
                def objective(x):
                    values = lower+np.clip(x, 0, 1)*(upper-lower)
                    phi = dict(tau_c=float(values[0]), b=float(values[1]), tau_s=float(values[0]+(values[2] if family == 'M3' else 0)))
                    identity = anchor['structure_id']+'_'+hashlib.sha256(json.dumps(phi, sort_keys=True).encode()).hexdigest()[:16]
                    c = {k: anchor[k] for k in ['structure_id', 'asset']}
                    c.update(phi, candidate_id=identity, model_family='static_dynamic_viscous' if family == 'M3' else 'dynamic_viscous')
                    folder = dest/'continuous'/condition/family/f'start_{index}'/identity
                    specs = [s for s in nc['probes'] if s['name'] in refs[condition] and s['name'] != 'stop_dwell']
                    run_jobs(runner, [runner.job(data, folder/s['name'], s, candidate=c) for s in specs])
                    cells = {s['name']: public_cell(folder/s['name'], s['probe_id']) for s in specs}
                    losses = [score(refs[condition][name], cell['log'], calibration)['loss'] if cell['safe'] else 1e30 for name, cell in cells.items()]
                    loss = float(np.mean(losses)); candidates[identity] = (c, cells, folder)
                    trace.append({'candidate': c, 'train_loss': loss})
                    write(dest/'continuous'/condition/family/f'start_{index}'/'trace.json', trace)
                    return loss
                simplex = np.tile(x0, (len(x0)+1, 1))
                for i in range(len(x0)): simplex[i+1, i] += .1 if x0[i] <= .9 else -.1
                optimum = minimize(objective, x0, method='Nelder-Mead', bounds=[(0., 1.)]*len(x0),
                                   options={'maxfev': budget['maximum_native_parameter_evaluations_per_start'], 'initial_simplex': simplex,
                                            'xatol': 1e-4, 'fatol': 1e-4})
                write(dest/'continuous'/condition/family/f'start_{index}'/'optimizer.json',
                      {'nfev': optimum.nfev, 'success': bool(optimum.success), 'message': str(optimum.message),
                       'budget_exhaustion_is_not_identifiability': True, 'train_minimum': float(optimum.fun)})
                return list(candidates.values())
            with concurrent.futures.ThreadPoolExecutor(max_workers=len(anchors)) as pool:
                points = sum(list(pool.map(lambda pair: one_start(pair[1], pair[0]), enumerate(anchors))), [])
            # Validation is opened only after both train optimizations finish.
            spec = next(s for s in nc['probes'] if s['name'] == 'stop_dwell')
            run_jobs(runner, [runner.job(data, folder/spec['name'], spec, candidate=c) for c, cells, folder in points])
            result = []
            for c, cells, folder in points:
                cells['stop_dwell'] = public_cell(folder/'stop_dwell', spec['probe_id'])
                bank[c['candidate_id']] = cells; result.append(c)
            return list({c['candidate_id']: c for c in result}.values())
        selections = {}
        selection_file = dest/'selection_frozen.json'
        if selection_file.exists():
            selections = read(selection_file)['conditions']
        else:
            for condition, rr in refs.items():
                points = optimize(condition, 'M2', initial[condition])
                C = rank(base+points, bank, rr, calibration, nc['selection'])
                prior = [r for r in C['rows'] if all(r[k] == nc['wrong_prior'][k] for k in ['tau_c', 'b']) and r['candidate_id'].endswith('_phi_008')]
                methods = {'A_wrong_prior': next(r for r in prior if r['structure_id'] == 'observable_S0'),
                           'B_structure_only': min(prior, key=lambda r: r['combined_loss']), 'C_M2': C['best']}
                write(dest/f'M2_support_{condition}.json', C)
                D = None
                if C['all_candidates_validation_inadequate']:
                    grid = rank(static, bank, rr, calibration, nc['selection'])
                    extra = optimize(condition, 'M3', grid)
                    D = rank(static+extra, bank, rr, calibration, nc['selection'])
                    methods['D_M3'] = D['best']; write(dest/f'M3_support_{condition}.json', D)
                selections[condition] = {'methods': methods, 'M2_adequate': C['model_adequate'],
                    'M3_triggered': D is not None, 'M3_adequate': bool(D and D['model_adequate']),
                    'M3_validation_improved': bool(D and D['best']['validation_loss'] < .95*C['best']['validation_loss'])}
            write(selection_file, {'conditions': selections, 'saved_unix_s': time.time(), 'test_read': False,
                                   'uncertainty_sha256': calibration['calibration_sha256']})
        # Already-seen regression is diagnostic only, after model choices freeze.
        legacy = {'conditions': {condition: {'methods': {
            {'C_M2': 'C_two_parameter', 'D_M3': 'D_static_extension'}.get(label, label): model
            for label, model in s['methods'].items()}} for condition, s in selections.items()}}
        runner.regression(eid, data, legacy, calibration)
        # Two new independent native tests; identical commands for all models.
        unique = {r['candidate_id']: r for s in selections.values() for r in s['methods'].values()}
        tests = {}; plots = []
        for spec in config['heldout']:
            testdir = dest/'heldout'/spec['name']
            jobs = [runner.job(data, testdir/'reference'/condition, spec, condition=condition) for condition in selections]
            jobs += [runner.job(data, testdir/'predictions'/cid, spec, candidate=m) for cid, m in unique.items()]
            write(testdir/'registration.json', {'spec': spec, 'selection_sha256': hashlib.sha256(selection_file.read_bytes()).hexdigest(),
                'command_sha256': hashlib.sha256(Path(data['command_tapes'][spec['name']]).read_bytes()).hexdigest(), 'selection_frozen_before_test': True})
            run_jobs(runner, jobs)
            for condition, selection in selections.items():
                ref = public_cell(testdir/'reference'/condition, 'P4'); model_logs = {}
                for label, m in selection['methods'].items():
                    pred = public_cell(testdir/'predictions'/m['candidate_id'], 'P4')
                    metrics = score(ref['log'], pred['log'], calibration, evaluation=True) if ref['safe'] and pred['safe'] else None
                    row = {'episode': eid, 'condition': condition, 'protocol': spec['name'], 'model': label,
                        'candidate_id': m['candidate_id'], 'parameters': {k: m[k] for k in ['tau_c', 'b', 'tau_s']},
                        'validation_rms': np.sqrt(m['validation_loss']), 'test': metrics,
                        'native_status': pred['status'], 'reference_status': ref['status']}
                    all_rows.append(row); tests[(condition, label, spec['name'])] = row
                    if pred['safe']: model_logs[label] = pred['log']
                if ref['safe']:
                    from interactive_twin.reporting import plot_heldout_predictions
                    plot_heldout_predictions(ref['log'], model_logs, testdir/f'prediction_{condition}.png')
        # No test outcome modifies candidates, whitening or acceptance threshold.
        for condition, s in selections.items():
            def improves(label):
                return all(tests[(condition, label, p['name'])]['test'] and
                    tests[(condition, 'B_structure_only', p['name'])]['test'] and
                    tests[(condition, label, p['name'])]['test']['metrics']['ee_position_rmse_m'] <
                    .95*tests[(condition, 'B_structure_only', p['name'])]['test']['metrics']['ee_position_rmse_m']
                    for p in config['heldout'])
            cgood = improves('C_M2'); dgood = 'D_M3' in s['methods'] and improves('D_M3')
            dbeatsc = 'D_M3' in s['methods'] and all(
                tests[(condition, 'D_M3', p['name'])]['test'] and tests[(condition, 'C_M2', p['name'])]['test'] and
                tests[(condition, 'D_M3', p['name'])]['test']['metrics']['ee_position_rmse_m'] <
                .95*tests[(condition, 'C_M2', p['name'])]['test']['metrics']['ee_position_rmse_m'] for p in config['heldout'])
            regression_file = dest/'regression/results.json'
            regression = read(regression_file)['rows'] if regression_file.exists() else []
            c_regression = all(r.get('within_tolerance', False) for r in regression if r['condition'] == condition and r['model'] == 'C_two_parameter')
            d_regression = all(r.get('within_tolerance', False) for r in regression if r['condition'] == condition and r['model'] == 'D_static_extension')
            passed = (cgood and s['M2_adequate'] and c_regression) or (dgood and dbeatsc and s['M3_adequate'] and s['M3_validation_improved'] and d_regression)
            decision = {'episode': eid, 'condition': condition, 'M2_both_tests_improved': cgood,
                'M3_both_tests_improved': dgood, 'status': 'PASS' if passed else ('BEHAVIORAL_ONLY' if cgood or dgood else 'MODEL_MISMATCH'),
                'M3_both_tests_better_than_M2': dbeatsc, 'M2_previous_regression_ok': c_regression, 'M3_previous_regression_ok': d_regression,
                'physical_parameter_accuracy_claimed': False,
                'statement': 'validated behavior predictor; report parameter support' if passed else
                    ('behavioral prediction improved, physical parameters remain unidentifiable' if cgood or dgood else 'MODEL_MISMATCH'),
                'no_additional_object_parameter_allowed': True}
            decisions.append(decision)
        write(runner.out/'results.json', {'rows': all_rows, 'decisions': decisions, 'test_changed_selection': False})
    return {'rows': all_rows, 'decisions': decisions, 'native_robot_dynamics_modified': False,
            'robot_response_covariance_included': True, 'scope': 'SIM_TO_SIM_BLIND_SYSID'}
