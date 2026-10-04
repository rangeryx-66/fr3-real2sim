"""Post-selection report only. Never changes inference, uncertainty or control."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def read(p): return json.loads(Path(p).read_text())
def write(p, d):
    p = Path(p); p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(d, indent=2, allow_nan=False)+'\n')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--output', default='results/interactive_twin_robot_response_20261004_v2')
    a = ap.parse_args(); out = Path(a.output); root = Path(__file__).resolve().parents[1]
    c = read(out/'protocol_frozen.json')['config']; results = read(out/'final_results.json')
    robot = read(out/'robot_calibration.json'); transfer = read(out/'robot_transfer_validation.json')
    # Add the already-seen 79b100c regression gate to the 3e5e856 diagnostic.
    regression = []
    for eid in c['episodes']:
        new = out/'physics'/eid/'regression/results.json'
        old = root/c['prior_output']/eid/'regression/results.json'
        if not new.exists() or not old.exists(): continue
        previous = read(old)['rows']
        for r in read(new)['rows']:
            prior = next((v for v in previous if v['condition'] == r['condition'] and v['model'] == 'C_two_parameter'), None)
            if not prior or 'prediction' not in r: continue
            baseline = prior['prediction']['metrics']['ee_position_rmse_m']
            current = r['prediction']['metrics']['ee_position_rmse_m']
            regression.append({'episode': eid, 'condition': r['condition'], 'model': r['model'],
                'previous_79_RMSE_m': baseline, 'new_RMSE_m': current,
                'absolute_tolerance_m': c['selection']['regression_absolute_tolerance_m'],
                'within_tolerance': current <= baseline+c['selection']['regression_absolute_tolerance_m'],
                'test_used_for_reselection': False})
    write(out/'previous_79_regression.json', regression)
    for d in results['decisions']:
        relevant = [r for r in regression if r['episode'] == d['episode'] and r['condition'] == d['condition']]
        passed = {r['model']: r['within_tolerance'] for r in relevant}
        d['previous_79_regression'] = passed
        if d['status'] == 'PASS' and relevant:
            kin = read(out/'physics'/d['episode']/'selection_frozen.json')['conditions'][d['condition']]
            cpass = d['M2_both_tests_improved'] and kin['M2_adequate'] and passed.get('C_two_parameter', False)
            dpass = d['M3_both_tests_improved'] and d['M3_both_tests_better_than_M2'] and kin['M3_adequate'] and kin['M3_validation_improved'] and passed.get('D_static_extension', False)
            if not (cpass or dpass):
                d['status'] = 'MODEL_MISMATCH'; d['statement'] = 'previous 79b100c regression gate failed; no parameter reselection'
    rows = []; support = []
    for r in results['rows']:
        path = out/'physics'/r['episode']/f"{'M3' if r['model']=='D_M3' else 'M2'}_support_{r['condition']}.json"
        s = read(path); m = (r.get('test') or {}).get('metrics', {})
        decision = next(d for d in results['decisions'] if d['episode'] == r['episode'] and d['condition'] == r['condition'])
        fitted = r['model'] in ('C_M2', 'D_M3')
        rows.append({'asset': r['episode'].split('_')[1], 'condition': r['condition'], 'model': r['model'], 'protocol': r['protocol'],
            'validation_normalized_residual': r['validation_rms'],
            'heldout_RMSE_mm': None if not m else m['ee_position_rmse_m']*1000,
            'start_delay_error_s': m.get('start_time_error_s'),
            'velocity_error_mm_s': None if not m else m['ee_velocity_rmse_m_s']*1000,
            'stop_dwell_error_mm': None if not m else m['dwell_incremental_position_rmse_m']*1000,
            'final_displacement_error_mm': None if not m else m['final_displacement_error_m']*1000,
            'parameters': r['parameters'], 'support_intervals': s['parameter_intervals'] if fitted else None,
            'support_count': len(s['support']) if fitted else None, 'identifiability': 'UNIDENTIFIABLE' if fitted else 'FIXED_PRIOR',
            'conclusion': decision['status'], 'native_status': r['native_status']})
    for p in sorted((out/'physics').glob('*/*_support_*.json')):
        s = read(p)
        support.append({'file': str(p), 'status': s['status'], 'intervals': s['parameter_intervals'],
                        'support': s['support'], 'retained': s['retained_parameters'],
                        'semantics': s['support_semantics']})
    write(out/'physics_support_sets.json', support)
    with (out/'main_table.csv').open('w') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    write(out/'main_table.json', rows)
    # Native safety and no replay/attachment contract; do not open object GT.
    native = []
    for p in out.rglob('report.json'):
        if not (p.parent/'process.json').exists(): continue
        r = read(p); entry = {'path': str(p.parent), 'status': r.get('status'),
            'minimum_joint_margin_rad': r.get('minimum_joint_margin_rad'),
            'bilateral_hold': r.get('bilateral_hold_established'),
            'frozen_baseline_sha256': hashlib.sha256((p.parent/'frozen_baseline.json').read_bytes()).hexdigest() if (p.parent/'frozen_baseline.json').exists() else None}
        for phase in ['P1', 'P2', 'P3', 'P4', 'ROBOT_CALIBRATION']:
            log = p.parent/'observable'/(phase+'.json')
            if log.exists():
                prov = read(log)['provenance']; entry['execution_contract'] = {k: prov[k] for k in ['initialization_only', 'robot_state_replayed', 'object_state_replayed', 'direct_object_actuation', 'attachment']}; break
        native.append(entry)
    write(out/'native_trial_audit.json', native)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for rec in [robot['records'][0], transfer['rows'][0], transfer['rows'][2]]:
        t = np.array(rec['time_s']); v = np.array(rec['measured']); pred = np.array(rec['predicted'])
        name = rec.get('episode', 'calibration')
        ax[0].plot(t, (pred[:, 0]-v[:, 0])*1e6, label=name)
        ax[1].plot(t, (pred[:, 1]-v[:, 1])*1000, label=name)
    ax[0].set_ylabel('Position error (um)'); ax[1].set_ylabel('Velocity error (mm/s)'); ax[1].set_xlabel('Time (s)')
    for a in ax: a.grid(alpha=.25); a.legend(); a.axvline(5, color='gray', linestyle='--')
    fig.suptitle('One shared robot calibration, no per-posture parameter fitting'); fig.tight_layout()
    fig.savefig(out/'robot_transfer_curves.png', dpi=160); plt.close(fig)
    # Describe the stop residual; do not use it to add parameters or tune weights.
    stop_shapes = []
    oldroot = root/c['prior_output']
    for eid in c['episodes']:
        selection = read(out/'physics'/eid/'selection_frozen.json')['conditions']
        inv = read(oldroot/eid/'available_probes.json')['conditions']
        prepared = read(oldroot/eid/'prepared.json')
        for condition, choice in selection.items():
            refpath = next(Path(r['path']) for r in inv[condition] if r['name'] == 'stop_dwell')
            ref = read(refpath/'observable/P3.json'); t = np.asarray(ref['time_s'])
            rp = np.asarray(ref['signals']['ee_T_world_tcp'])[:, :3, 3]
            lo = np.searchsorted(t, 3.); mask = t >= 3.; tt = t[mask]-3.
            fig, ax = plt.subplots(figsize=(10, 5))
            for label, model in choice['methods'].items():
                if label not in ('C_M2', 'D_M3'): continue
                cid = model['candidate_id']
                found = list((out/'physics'/eid/'continuous'/condition).glob(f'*/start_*/{cid}/stop_dwell'))
                if found: folder = found[0]
                elif '_static_' in cid: folder = oldroot/eid/'static_bank'/cid/'stop_dwell'
                elif cid.startswith('observable_'): folder = Path(prepared['prior_output'])/'bank'/cid.removeprefix('observable_')/'stop_dwell'
                else: folder = oldroot/eid/'bank'/cid/'stop_dwell'
                pred = read(folder/'observable/P3.json')
                pp = np.asarray(pred['signals']['ee_T_world_tcp'])[:, :3, 3]
                e = ((pp-pp[lo-1])-(rp-rp[lo-1]))[mask]
                late = tt >= tt[-1]-2.; slope = np.polyfit(tt[late], e[late], 1)[0]
                i = np.searchsorted(tt, .25)
                stop_shapes.append({'episode': eid, 'condition': condition, 'model': label,
                    'late_stop_drift_vector_m_s': slope.tolist(),
                    'error_after_250ms_m': float(np.linalg.norm(e[i])),
                    'final_stop_error_m': float(np.linalg.norm(e[-1])),
                    'incremental_stop_RMSE_m': float(np.sqrt(np.mean(e*e))),
                    'robot_pole_s': robot['parameters']['T_s'], 'postselection_diagnostic_only': True})
                ax.plot(tt, np.linalg.norm(e, axis=1)*1000, label=label)
            ax.set(xlabel='Time after drive removal (s)', ylabel='Incremental stop error norm (mm)', title=eid+' / '+condition)
            ax.grid(alpha=.25); ax.legend(); fig.tight_layout()
            fig.savefig(out/f'stop_residual_{eid}_{condition}.png', dpi=150); plt.close(fig)
    write(out/'stop_residual_shapes.json', stop_shapes)
    report = {'mode': 'SIM_TO_SIM_BLIND_SYSID', 'decisions': results['decisions'],
        'robot_parameters': robot['parameters'], 'robot_covariance': robot['parameter_covariance'],
        'native_trial_count': len(native), 'native_status_counts': {k: sum(r['status']==k for r in native) for k in sorted({r['status'] for r in native})},
        'all_native_baselines_identical': len({r['frozen_baseline_sha256'] for r in native}) == 1,
        'physics_support_set_count': len(support), 'rows': rows,
        'GT_friction_accuracy_proven': False, 'full_task_replay_claimed': False,
        'note': 'Predictive PASS never implies a unique or hardware-calibrated friction parameter.'}
    write(out/'delivery_summary.json', report)
    if (out/'summary.json').exists() and not (out/'robot_gate_summary.json').exists():
        write(out/'robot_gate_summary.json', read(out/'summary.json'))
    write(out/'summary.json', report)
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}, indent=2))


if __name__ == '__main__': main()
