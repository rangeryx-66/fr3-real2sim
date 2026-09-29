"""Replay recorded successful GraspGenX trajectories in Isaac and export MP4s.

This is a visual replay of logged joint commands, not an additional planner or
inference trial. The replay is checked against the benchmark lift/hold rule.
"""

import argparse
import json
import os
from pathlib import Path
import sys
import time

from run_r1a7_generalization import ROOT, SIM, ready, start, stop

sys.path.insert(0, str(ROOT / 'src'))
import r1a7_plant as plant
from r1a7_calibration import width_to_finger_q


def grip(width):
    joint_q = width_to_finger_q(width)
    result = plant.command({'op': 'trajectory',
                            'names': ['dex1_Joint1_1', 'dex1_Joint2_1'],
                            'points': [{'t': 1.0, 'q': [joint_q, joint_q]}],
                            'gripper': True}, timeout=60)
    if not result['ok']:
        raise RuntimeError(f'gripper trajectory failed: {result}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=4)
    parser.add_argument('--port', type=int, default=18792)
    parser.add_argument('--failed-attempt', action='store_true',
                        help='Replay the first executed attempt of a failed trial through micro-lift')
    args = parser.parse_args()
    plant.URL = f'http://127.0.0.1:{args.port}'
    trial = json.loads(args.trial.read_text())
    if trial.get('grasp_provider') != 'graspgenx':
        raise ValueError('expected a GraspGenX trial')
    if args.failed_attempt:
        if trial.get('success') or not trial.get('planning_succeeded'):
            raise ValueError('expected a failed trial with an executed plan')
    elif not trial.get('success'):
        raise ValueError('expected a completed successful GraspGenX trial')
    obj = trial['object']['id']
    seed = trial['scenario']['seed']
    output = args.output.resolve()
    env = {**os.environ, 'R1A7_BASE_POSE': '0.329,-0.175,0.237,56.295',
           'R1A7_PEDESTAL_SIZE': '0.10,0.10,0.20',
           'R1A7_PRESENTATION_CAMERA': '1',
           'R1A7_PLANT_PORT': str(args.port), 'OMNI_KIT_ACCEPT_EULA': 'YES',
           'ACCEPT_EULA': 'Y'}
    procs = []
    started = time.monotonic()
    try:
        procs.append(start([SIM, '-u', str(ROOT / 'src/r1a7_sim_server.py'),
                            '--gpu', str(args.gpu), '--port', str(args.port),
                            '--arena-target', obj], output.with_suffix('.sim.log'), env))
        ready(args.port, obj, procs)
        result = plant.command({'op': 'reset', 'seed': seed})
        if not result['ok']:
            raise RuntimeError(result)
        plant.settle(.8)
        grip(.09)
        initial_z = plant.state()['box'][2]
        result = plant.command({'op': 'video_start', 'path': str(output)})
        if not result['ok']:
            raise RuntimeError(result)
        plant.settle(.4)
        stages = []
        executions = trial['executions']
        # The benchmark may try another candidate after contact loss and reset
        # the object. A presentation clip replays only the successful attempt.
        if args.failed_attempt:
            executions = executions[:next((i + 1 for i, item in enumerate(executions)
                                           if item['stage'] == 'MICRO_LIFT'), len(executions))]
        elif len(trial.get('candidate_attempts', [])) > 1:
            starts = [i for i, item in enumerate(executions) if item['stage'] == 'PREGRASP']
            if len(starts) < 2:
                raise ValueError('cannot isolate the successful candidate trajectory')
            executions = executions[starts[-1]:]
        for execution in executions:
            stage = execution['stage']
            if stage == 'MICRO_LIFT':
                grip(.001067)
                plant.settle(.3)
            command = {'op': 'trajectory', 'names': execution['joint_names'],
                       'points': execution['points']}
            result = plant.command(command, timeout=180)
            stages.append({'stage': stage, 'result': result})
            if not result['ok']:
                raise RuntimeError(f'{stage} replay failed: {result}')
        plant.settle(2.2)
        state = plant.state()
        samples = [s for s in state['history'] if s['t'] >= state['t'] - 2.1]
        heights = [s['z'] for s in samples]
        result = plant.command({'op': 'video_stop'}, timeout=90)
        if not result['ok']:
            raise RuntimeError(f'video encoder failed: {result}')
        success = (len(samples) >= 2 and samples[-1]['t'] - samples[0]['t'] >= 2.0
                   and max(heights) - initial_z >= .08
                   and min(heights) - initial_z >= .07
                   and max(heights) - min(heights) <= .01)
        report = {'source_trial': str(args.trial.resolve()),
                  'source_success': bool(trial.get('success')),
                  'source_failure': trial.get('category') if args.failed_attempt else None,
                  'replayed_attempt': ('first executed attempt through micro-lift'
                                       if args.failed_attempt else 'final successful candidate only'),
                  'replay_success': success, 'object': obj, 'seed': seed,
                  'initial_z_m': initial_z, 'replay_lift_m': max(heights) - initial_z,
                  'hold_range_m': max(heights) - min(heights),
                  'frames': result['frames'], 'video': str(output),
                  'stages': stages, 'elapsed_wall_s': time.monotonic() - started}
        output.with_suffix('.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
        if not success and not args.failed_attempt:
            raise RuntimeError('recorded trajectory did not reproduce successful grasp')
    finally:
        stop(procs)


if __name__ == '__main__':
    main()
