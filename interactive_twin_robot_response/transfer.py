"""Independent no-contact transport check; no per-asset robot fit."""
import concurrent.futures
import copy
import sys
from pathlib import Path

from .calibration import extract, evaluate, read, sha


def validate(root, config, result, noise, write):
    sys.path.insert(0, str(root/'scripts'))
    from run_interactive_twin_conditional import Experiment
    runner = Experiment(config)
    common = read(root/config['transfer']['robot_only_template_job'])
    template_source = read(Path(common['source'])/'report.json')
    jobs = []
    for eid in config['episodes']:
        original = read(root/config['prior_output']/eid/'prepared.json')['source_job']
        source = read(Path(original['source'])/'report.json')
        # Separate calibration scene only. Existing task scenes stay byte-identical.
        # Reuse the already-certified out-of-reach object placement for all assets.
        independent = copy.deepcopy(template_source)
        independent['robot_base_pose'] = source['robot_base_pose']
        snapshot = read(original['conditional_snapshot'])
        src = runner.out/'no_contact_transfer'/eid/'source'
        write(src/'report.json', independent)
        for repeat in range(config['transfer']['repeats_per_posture']):
            output = runner.out/'no_contact_transfer'/eid/f'repeat_{repeat}'
            job = {**common, 'source': str(src), 'plan': original['plan'],
                   'output': str(output), 'episode_id': 'ROBOT_ONLY_'+eid,
                   'robot_start_q': snapshot['q_rad'][:6],
                   'deadline_shanghai': config['deadline_shanghai'],
                   'wall_clock_budget_s': config['native_wall_clock_budget_s'],
                   'robot_calibration_id': 'SHARED_RESPONSE_TRANSFER_VALIDATION_ONLY',
                   'mobile_platform': original.get('mobile_platform', False)}
            jobs.append(job)
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(config['gpus'])) as pool:
        reports = list(pool.map(runner.native, jobs))
    records = []; failures = []
    for job, report in zip(jobs, reports):
        if report.get('status') != 'ROBOT_CALIBRATION_COMPLETE':
            failures.append({'output': job['output'], 'status': report.get('status')})
            continue
        record = extract(job['output'])
        record['episode'] = job['episode_id']; records.append(record)
    write(runner.out/'transfer_inputs.json', records)
    parameters = [result['parameters'][k] for k in ['delta_t_s', 'T_s']]
    rows = evaluate(records, parameters, result['parameter_covariance'], config['robot_calibration'], noise)
    for row, record in zip(rows, records):
        row['episode'] = record['episode']
        row['parameters_refit'] = False
    passed = not failures and len(rows) == len(jobs) and all(
        all(s['normalized_rms'] <= config['selection']['adequacy_rms'] for s in r['segments'].values())
        for r in rows)
    return {'passed': passed, 'rows': rows, 'failures': failures, 'parameters_refit': False,
            'all_sources_object_free': True, 'native_trial_count': len(jobs),
            'task_scenes_changed': False, 'object_parameters_used': False}
