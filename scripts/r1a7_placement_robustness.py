#!/usr/bin/env python3
"""Screen R1 base placements for frozen-grasp paths and interaction reserve.

This is a kinematic search, not a motion planner or robot controller.
Shortlisted placements must be checked against MoveIt self collision by
verify_r1a7_placement_moveit.py. Scene collision geometry is supplied there.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import numpy as np

from r1a7_workspace_diagnostics import (
    NumericalIK, JOINTS, HOME, base_targets, frozen_targets, screen_batch,
)

ROOT = Path(__file__).resolve().parents[1]
PRIOR = ROOT / 'results/r1a7_workspace/r1a7_workspace_placement.json'
LIFT_M = .10
PREGRASP_M = .08
STEP_M = .02
INTERACTION_M = .02
MAX_JOINT_STEP_RAD = .45
FOCUS = (4, 5, 6)  # J5/J6/J7


def shifted(target, delta):
    result = target.copy()
    result[:3, 3] += delta
    return result


def margin(q, solver):
    q = np.asarray(q)
    margins = np.minimum(q - solver.lo, solver.hi - q)
    return float(min(margins[i] for i in FOCUS))


def continuation(solver, start_q, poses, max_nfev=45):
    q = np.array(start_q)
    margins = []
    steps = []
    for pose in poses:
        solution = solver.solve(pose, [q], max_nfev=max_nfev)
        if not solution['ok']:
            return dict(ok=False, reason='NO_IK', steps=steps,
                        min_margin_rad=min(margins, default=None))
        next_q = np.array(solution['q'])
        jump = float(np.max(np.abs(next_q-q)))
        if jump > MAX_JOINT_STEP_RAD:
            return dict(ok=False, reason='JOINT_JUMP', steps=steps,
                        jump_rad=jump, min_margin_rad=min(margins, default=None))
        q = next_q
        m = margin(q, solver)
        margins.append(m)
        steps.append(dict(q=q.tolist(), margin_rad=m, jump_rad=jump))
    return dict(ok=True, steps=steps, min_margin_rad=min(margins, default=None))


def paths(solver, grasp, grasp_q):
    # Follow the same frozen orientation throughout each short Cartesian
    # segment. Work backward from grasp to pregrasp, then forward to lift.
    approach = grasp[:3, 2]
    backward = [shifted(grasp, -a*approach)
                for a in np.arange(STEP_M, PREGRASP_M+1e-8, STEP_M)]
    upward = [shifted(grasp, [0., 0., a])
              for a in np.arange(STEP_M, LIFT_M+1e-8, STEP_M)]
    pre = continuation(solver, grasp_q, backward)
    lift = continuation(solver, grasp_q, upward)
    interactions = {}
    for axis, vector in [('xp', [1,0,0]), ('xm', [-1,0,0]),
                         ('yp', [0,1,0]), ('ym', [0,-1,0]),
                         ('zp', [0,0,1]), ('zm', [0,0,-1])]:
        unit = np.array(vector)
        poses = [shifted(grasp, a*unit) for a in (INTERACTION_M/2, INTERACTION_M)]
        interactions[axis] = continuation(solver, grasp_q, poses)
    all_paths = [pre, lift, *interactions.values()]
    available = [p['min_margin_rad'] for p in all_paths if p['min_margin_rad'] is not None]
    return dict(pregrasp=pre, lift=lift, interactions=interactions,
                pregrasp_lift_ok=pre['ok'] and lift['ok'],
                interactions_ok=sum(p['ok'] for p in interactions.values()),
                full_ok=all(p['ok'] for p in all_paths),
                min_margin_rad=min(available+[margin(grasp_q,solver)]))


def find_grasp_solutions(solver, pose, seeds=5, max_nfev=75):
    found = []
    for seed in [HOME, *solver.rng.uniform(solver.lo+.01, solver.hi-.01, size=(seeds,7))]:
        row = solver.solve(pose, [seed], max_nfev=max_nfev)
        if not row['ok']:
            continue
        q = np.array(row['q'])
        if any(np.max(np.abs(q-np.array(r['q'])))<.05 for r in found):
            continue
        row['focus_margin_rad'] = margin(q, solver)
        found.append(row)
    return sorted(found, key=lambda r:r['focus_margin_rad'], reverse=True)


def evaluate(solver, targets, base, seeds=4, shortlist=3):
    rows = []
    for rank, grasp in enumerate(base_targets(targets, base)):
        solutions = find_grasp_solutions(solver, grasp, seeds=seeds)
        row = dict(rank=rank, grasp_ik=bool(solutions),
                   solution_count=len(solutions))
        if solutions:
            options = []
            for solution in solutions[:shortlist]:
                result = paths(solver, grasp, solution['q'])
                options.append(dict(grasp_q=solution['q'],
                                    grasp_margin_rad=solution['focus_margin_rad'],
                                    **result))
            # Prefer complete paths, then interaction coverage, then margins.
            chosen = max(options, key=lambda r:(r['full_ok'], r['pregrasp_lift_ok'],
                                                r['interactions_ok'], r['min_margin_rad']))
            row.update(chosen)
        rows.append(row)
    solved = [r for r in rows if r['grasp_ik']]
    complete = [r for r in solved if r.get('full_ok')]
    segments = [r for r in solved if r.get('pregrasp_lift_ok')]
    margins = [r['grasp_margin_rad'] for r in solved]
    path_margins = [r['min_margin_rad'] for r in complete]
    return dict(base=list(map(float,base)),
                grasp_ik_unique=len(solved),
                grasp_ik_candidates=10*len(solved),
                pregrasp_lift_unique=len(segments),
                interaction_full_unique=len(complete),
                interaction_directions=sum(r.get('interactions_ok',0) for r in solved),
                grasp_margin_median_rad=float(np.median(margins)) if margins else None,
                grasp_margin_min_rad=min(margins, default=None),
                full_path_margin_min_rad=min(path_margins, default=None),
                rows=rows)


def placements(count, center=None):
    rng = np.random.default_rng(20260928)
    if center is None:
        old = json.loads(PRIOR.read_text())
        anchors = [[.326,-.201,.147,83.753], [.3,-.2,.15,90.],
                   [.3,-.2,0.,45.], [.2,0.,0.,90.]]
        anchors.extend(r['base'] for r in old['finalists'])
    else:
        anchors = [list(map(float,center))]
    samples = [list(map(float,b)) for b in anchors]
    for i in range(count):
        parent = np.array(anchors[i % len(anchors)])
        if center is None:
            scale = np.array([.045,.06,.06,13.]) if i < count//2 else np.array([.09,.10,.10,25.])
        else:
            scale = np.array([.02,.025,.03,6.]) if i < count//2 else np.array([.04,.05,.05,12.])
        b = parent + rng.normal(size=4)*scale
        b[:3] = np.clip(b[:3], [.15,-.35,-.03], [.45,.15,.32])
        b[3] = np.clip(b[3], 20.,145.)
        samples.append(b.tolist())
    seen = set()
    for b in samples:
        key = tuple(round(v,3) for v in b)
        if key in seen: continue
        seen.add(key)
        yield b


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--samples', type=int, default=100)
    p.add_argument('--min-single-ik', type=int, default=9)
    p.add_argument('--max-shortlist', type=int, default=35)
    p.add_argument('--refine-center', nargs=4, type=float,
                   metavar=('X','Y','Z','YAW_DEG'))
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    solver = NumericalIK()
    targets = frozen_targets()
    screening = []
    for index, base in enumerate(placements(args.samples,args.refine_center)):
        quick = screen_batch(solver, targets, base, random_seeds=0, max_nfev=55)
        grasp = sum(r['ok'] for r in quick)
        margins = [min(r['margins_rad'][j] for j in ('J5','J6','J7'))
                   for r in quick if r['ok']]
        screening.append(dict(base=base, grasp_ik=grasp,
                              rough_focus_margin_median_rad=float(np.median(margins)) if margins else 0.))
        if (index+1)%20==0:
            print(f'screen {index+1}: >=10 IK {sum(r["grasp_ik"]>=10 for r in screening)}', flush=True)
    eligible = [r for r in screening if r['grasp_ik']>=args.min_single_ik]
    eligible.sort(key=lambda r:(r['grasp_ik']>=10,
                                r['grasp_ik'] + 2*r['rough_focus_margin_median_rad']),reverse=True)
    shortlisted = eligible[:args.max_shortlist]
    evaluated = []
    for i, entry in enumerate(shortlisted):
        row = evaluate(solver, targets, entry['base'])
        evaluated.append(row)
        print(f'path {i+1}/{len(shortlisted)} base={np.round(entry["base"],3)} '
              f'IK={row["grasp_ik_unique"]} segment={row["pregrasp_lift_unique"]} '
              f'full={row["interaction_full_unique"]} '
              f'margin={row["grasp_margin_median_rad"]}',flush=True)
    result = dict(refine_center=args.refine_center,
                  definitions=dict(pregrasp_m=PREGRASP_M, lift_m=LIFT_M,
                                   step_m=STEP_M, interaction_m=INTERACTION_M,
                                   max_joint_step_rad=MAX_JOINT_STEP_RAD,
                                   focus_joints=['J5','J6','J7']),
                  screening=screening, evaluated=evaluated)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print('DONE',args.output,flush=True)


if __name__=='__main__':
    main()
