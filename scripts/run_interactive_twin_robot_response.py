"""Robot-only prerequisite to object-physics calibration, independent of baseline.

Stops explicitly when the shared nuisance model lacks adequacy or independent
cross-posture evidence. Does not launch an object optimizer after that failure.
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from interactive_twin_robot_response.calibration import extract, fit, read, sha
from interactive_twin_observable.probes import make_tape


def write(path, data):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def freeze(config, output):
    f = output/'protocol_frozen.json'
    if f.exists():
        if read(f)['config'] != config:
            raise ValueError('FROZEN_CONFIG_CHANGED')
        return read(f)
    result = {'config': config, 'frozen_unix_s': time.time(), 'fit_started': False,
              'test_executed': False, 'tapes': []}
    for eid in config['episodes']:
        data = read(ROOT/config['prior_output']/eid/'prepared.json')
        source = data['source_job']
        snapshot = read(source['conditional_snapshot'])
        estimate = read(source['initial_estimate'])
        memory = read(source['initial_estimate_memory'])
        for spec in config['heldout']:
            path = output/eid/'commands'/(spec['name']+'.json')
            write(path, make_tape(snapshot, estimate, memory['supporting_observations'], spec))
            result['tapes'].append({'episode': eid, 'name': spec['name'],
                                   'path': str(path), 'sha256': sha(path)})
    write(f, result)
    return result


def plot(result, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    row = result['records'][0]
    t = np.asarray(row['time_s']); m = np.asarray(row['measured']); p = np.asarray(row['predicted'])
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for a in axes:
        a.axvspan(0, 5, color='#eeeeee', label='Calibration train')
        a.axvline(5, color='gray', linestyle='--')
        a.grid(alpha=.25)
    axes[0].plot(t, m[:, 0]*1000, label='Measured no-contact EE')
    axes[0].plot(t, p[:, 0]*1000, label='Shared delay + response pole')
    axes[0].set_ylabel('Directional travel (mm)'); axes[0].legend(loc='upper left')
    axes[1].plot(t, m[:, 1]*1000); axes[1].plot(t, p[:, 1]*1000)
    axes[1].set_ylabel('Velocity (mm/s)')
    axes[2].plot(t, (p[:, 0]-m[:, 0])*1000, color='#b34438')
    sig = np.asarray(row['total_scales'])[:, 0]*1000
    axes[2].fill_between(t, -10*sig, 10*sig, alpha=.25, color='gray', label='Unchanged 10-sigma gate')
    axes[2].set_ylabel('Position residual (mm)'); axes[2].legend(); axes[2].set_xlabel('Time (s)')
    fig.suptitle('Robot-only prerequisite: first pulse fit / second pulse validation\n'+result['status'])
    fig.tight_layout(); fig.savefig(output/'robot_response.png', dpi=170); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='configs/interactive_twin_robot_response.yaml')
    args = ap.parse_args()
    c = read(ROOT/args.config); out = ROOT/c['output']; out.mkdir(parents=True, exist_ok=True)
    freeze(c, out)
    files = sorted((ROOT/'interactive_twin_robot_response').glob('*.py'))+[Path(__file__).resolve()]
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in files}
    method = out/'method_frozen.json'
    if method.exists() and read(method)['source_hashes'] != hashes:
        raise RuntimeError('FROZEN_CALIBRATION_METHOD_CHANGED')
    if not method.exists():
        write(method, {'source_hashes': hashes, 'saved_unix_s': time.time(), 'heldout_read': False})
    cache = out/'calibration_inputs.json'
    if cache.exists():
        records = read(cache)
    else:
        records = [extract(ROOT/p) for p in c['robot_calibration']['sources']]
        write(cache, records)
    noise = read(ROOT/c['prior_output']/'calibration/selection_noise.json')
    result = fit(records, c['robot_calibration'], noise)
    numerical = all(r['segments']['validation']['normalized_rms'] <= c['selection']['adequacy_rms']
                    for r in result['records'])
    from interactive_twin_robot_response.transfer import validate
    transfer = validate(ROOT, c, result, noise, write) if numerical else {'passed': False, 'reason': 'CALIBRATION_FAILED'}
    write(out/'robot_transfer_validation.json', transfer)
    result.update(status='MODEL_MISMATCH' if not (numerical and transfer['passed']) else 'ROBOT_RESPONSE_VALIDATED',
                  numerical_adequacy_pass=numerical, cross_asset_robot_validation=transfer['passed'],
                  object_optimization_allowed=False, object_parameters_changed=False,
                  reason='SHARED_ROBOT_RESPONSE_VALIDATION_FAILED' if not numerical else
                         ('SHARED_RESPONSE_TRANSFER_FAILED' if not transfer['passed'] else 'ROBOT_GATE_PASSED'),
                  whitening={'repeat_covariance_source': str(ROOT/c['prior_output']/'calibration/selection_noise.json'),
                             'robot_uncertainty': 'training-only local parameter covariance propagation',
                             'structure_uncertainty': 'exactly zero in object-free calibration',
                             'systematic_model_bias_added_as_noise': False})
    write(out/'robot_calibration.json', result)
    plot(result, out)
    main_rows = []
    for asset, condition in [('7320', 'original'), ('7320', 'additional_nonzero'), ('45621', 'original')]:
        for model in ['A_wrong_prior', 'B_structure_only', 'C_M2', 'D_M3']:
            main_rows.append({'asset': asset, 'condition': condition, 'model': model,
                'validation_normalized_residual': None, 'new_speed_rmse_mm': None, 'new_restart_rmse_mm': None,
                'start_delay_error_s': None, 'dwell_error_mm': None,
                'physics_support': 'NOT_ESTIMATED_THIS_ROUND', 'identifiability': 'NOT_ASSESSED',
                'status': 'NOT_RUN_ROBOT_CALIBRATION_GATE', 'reason': result['reason']})
    with (out/'comparison.csv').open('w') as f:
        w = csv.DictWriter(f, fieldnames=list(main_rows[0])); w.writeheader(); w.writerows(main_rows)
    write(out/'summary.json', {'status': result['status'], 'stage': 'robot_response_calibration',
        'reason': result['reason'], 'object_model_mismatch_not_proven': True,
        'heldout_protocols_frozen': [s['name'] for s in c['heldout']], 'heldout_executed': False,
        'native_object_trials_this_round': 0, 'baseline_modified': False,
        'main_table': main_rows, 'robot_parameters': result['parameters'],
        'next_data_required': 'independent no-contact command/response logs at both deployment postures; do not fit those parameters from loaded stop residuals'})
    print(json.dumps({'status': result['status'], 'parameters': result['parameters'],
                      'validation': [r['segments']['validation'] for r in result['records']]}, indent=2))


if __name__ == '__main__':
    main()
