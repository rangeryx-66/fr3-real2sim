#!/usr/bin/env python3
"""Run frozen-grasp MoveIt IK/path tolerance sweep around installation C.

The loaded MoveIt model must have its original world mount (0,0,0,+90 deg).
No grasp JSON or grasp pose is edited; TCP offsets represent calibration
ablation in the IK request only. This script never plans or executes motion.
"""
import argparse
import itertools
import json
import subprocess
import sys
from pathlib import Path

C = (0.329, -0.175, 0.237, 56.295)


def cases():
    yield 'nominal', C, (0., 0., 0.)
    for axis in range(3):
        for delta in (-.02, -.01, .01, .02):
            base = list(C)
            base[axis] += delta
            yield f'{"xyz"[axis]}_{delta:+.3f}', base, (0., 0., 0.)
    for delta in (-5., -3., 3., 5.):
        base = list(C)
        base[3] += delta
        yield f'yaw_{delta:+.0f}', base, (0., 0., 0.)
    for axis in range(3):
        for delta in (-.005, .005):
            offset = [0., 0., 0.]
            offset[axis] = delta
            yield f'tcp_{"xyz"[axis]}_{delta:+.3f}', C, offset
    # Eight simultaneous extreme corners probe whether single-axis tolerance
    # generalizes to a neighbourhood. This is a sample, not all 16 yaw corners.
    for index, signs in enumerate(itertools.product((-1, 1), repeat=3)):
        base = [C[i] + signs[i] * .02 for i in range(3)]
        base.append(C[3] + (5 if index % 2 else -5))
        yield f'corner_{index:02d}', base, (0., 0., (.005 if index % 2 else -.005))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--seeds', type=int, default=12)
    parser.add_argument('--branches', type=int, default=6)
    parser.add_argument('--timeout', type=float, default=.15)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, (name, base, offset) in enumerate(cases(), 1):
        path = args.output_dir / f'{name}.json'
        if not path.exists():
            command = [sys.executable,
                       str(Path(__file__).with_name('verify_r1a7_placement_moveit.py')),
                       '--base', *map(str, base), '--tcp-offset', *map(str, offset),
                       '--seeds', str(args.seeds), '--branches', str(args.branches),
                       '--timeout', str(args.timeout), '--output', str(path)]
            with (args.output_dir / f'{name}.log').open('w') as log:
                subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
        data = json.loads(path.read_text())
        rows.append(dict(name=name, base=list(base), tcp_offset_m=list(offset),
                         **data['summary']))
        (args.output_dir / 'summary.json').write_text(json.dumps(
            dict(conditions=dict(seeds=args.seeds, branches=args.branches,
                                 timeout_s=args.timeout, model_mount='0,0,0,90'),
                 cases=rows), indent=2) + '\n')
        summary = data['summary']
        print(f'{index:02d} {name}: IK {summary["grasp_ik"]}/11, '
              f'path {summary["all_interactions"]}/11, '
              f'safe .05/.08 {summary["safe_complete_005"]}/'
              f'{summary["safe_complete_008"]}', flush=True)


if __name__ == '__main__':
    main()
