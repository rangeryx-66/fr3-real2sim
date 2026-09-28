#!/usr/bin/env python3
"""Offline R1-7a workspace/base/TCP IK diagnostic; never plans or executes.

The 10 benchmark JSON files are byte-identical, so the 11 poses in the
checked-in frozen file represent all 110 candidates, with multiplicity 10.
Numerical FK/IK screens configurations; --moveit validates the selected
configuration against the running MoveIt compute_ik service.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from r1a7_frames import grasp_to_r1a7_tcp
from urdf_chain import KinematicChain

JOINTS = tuple(f'J{i}' for i in range(1, 8))
HOME = np.array([0., 1.3, 1., -1.3, 0., 0., 0.])
MODEL = ROOT / 'config/r1a7_dex1.urdf'
FROZEN = ROOT / 'results/r1a7_workspace/frozen_anygrasp_trial01.json'


def transform(x=0., y=0., z=0., yaw=0.):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def frozen_targets():
    data = json.loads(FROZEN.read_text())
    out = []
    for g in data['grasps']:
        T = grasp_to_r1a7_tcp(data['T_B_C'], g['rotation'], g['translation'], g['depth'])[1]
        out.append(T)
    return out


class NumericalIK:
    def __init__(self):
        root = ET.parse(MODEL).getroot()
        joints = {j.get('name'): j for j in root.findall('joint')}
        self.lo = np.array([float(joints[n].find('limit').get('lower')) for n in JOINTS])
        self.hi = np.array([float(joints[n].find('limit').get('upper')) for n in JOINTS])
        self.chain = KinematicChain(MODEL, 'base_link', 'r1a7_tcp', JOINTS)
        self.flange = KinematicChain(MODEL, 'base_link', 'Link7', JOINTS)
        self.flange_tcp = np.linalg.inv(self.flange.forward(dict(zip(JOINTS, HOME)))) @ self.chain.forward(dict(zip(JOINTS, HOME)))
        self.rng = np.random.default_rng(20260928)

    def target_for_variant(self, grasp_target, variant):
        if variant == 'flange':
            return grasp_target @ self.flange_tcp
        if variant.startswith('length_'):
            delta = float(variant.split('_')[1])
            shift = np.eye(4)
            shift[2, 3] = -delta
            return grasp_target @ shift
        return grasp_target

    def solve(self, target, seeds, max_nfev=65):
        """Best 6D residual with bounded joints and multi-start."""
        target_p = target[:3, 3]
        target_r = Rotation.from_matrix(target[:3, :3])
        best = None
        for seed in seeds:
            seed = np.clip(seed, self.lo + 1e-6, self.hi - 1e-6)
            def residual(q):
                actual = self.chain.forward(dict(zip(JOINTS, q)))
                rot = (Rotation.from_matrix(actual[:3, :3]) * target_r.inv()).as_rotvec()
                return np.r_[actual[:3, 3] - target_p, 0.12 * rot]
            fit = least_squares(residual, seed, bounds=(self.lo, self.hi),
                                max_nfev=max_nfev, ftol=1e-6, xtol=1e-6, gtol=1e-6)
            actual = self.chain.forward(dict(zip(JOINTS, fit.x)))
            dp = float(np.linalg.norm(actual[:3, 3] - target_p))
            dr = float((Rotation.from_matrix(actual[:3, :3]) * target_r.inv()).magnitude())
            score = dp + 0.12 * dr
            margin = np.minimum(fit.x - self.lo, self.hi - fit.x)
            row = dict(ok=dp < .0015 and dr < math.radians(1), dp_m=dp,
                       dr_deg=math.degrees(dr), q=fit.x.tolist(),
                       margins_rad=dict(zip(JOINTS, margin.tolist())),
                       nearest_limit=JOINTS[int(np.argmin(margin))], score=score)
            if best is None or score < best['score']:
                best = row
            if row['ok']:
                break
        return best

    def seeds(self, extra=2, prior=None):
        s = [HOME]
        if prior is not None:
            s.insert(0, np.array(prior))
        s.extend(self.rng.uniform(self.lo + .01, self.hi - .01, size=(extra, 7)))
        return s


def base_targets(world_targets, base):
    inverse = np.linalg.inv(transform(*base))
    return [inverse @ T for T in world_targets]


def screen_batch(solver, targets, base, variant='current', random_seeds=1, max_nfev=55):
    target_base = base_targets(targets, base)
    results = []
    prior = None
    for T in target_base:
        r = solver.solve(solver.target_for_variant(T, variant),
                         solver.seeds(random_seeds, prior), max_nfev)
        results.append(r)
        if r['ok']:
            prior = r['q']
    return results


def summarize(rows):
    return dict(unique_successes=sum(r['ok'] for r in rows),
                candidates_successes=10 * sum(r['ok'] for r in rows),
                total_candidates=110,
                nearest_limit_failures={j: sum(not r['ok'] and r['nearest_limit'] == j for r in rows)
                                        for j in JOINTS})


def scan_map(solver, targets, args):
    # All eleven real grasp orientations, centered at each world X/Z cell.
    x_values = np.round(np.linspace(.40, .55, 11), 5)
    z_values = np.round(np.linspace(.02, .25, 13), 5)
    base = [0., 0., 0., 90.]
    counts = []
    template = targets[0][:3, 3]
    for z in z_values:
        row = []
        for x in x_values:
            shifted = []
            for T in targets:
                U = T.copy()
                U[:3, 3] += [x - template[0], 0., z - template[2]]
                shifted.append(U)
            found = screen_batch(solver, shifted, base, random_seeds=args.seeds,
                                 max_nfev=args.max_nfev)
            row.append(sum(r['ok'] for r in found))
        counts.append(row)
        print(f'map z={z:.3f}: {row}', flush=True)
    return dict(x_m=x_values.tolist(), z_m=z_values.tolist(),
                per_cell_11_orientation_successes=counts,
                method='bounded numerical 6D IK; all 11 frozen grasp orientations')


def search_base(solver, targets, args):
    # Coarse structured search spans every base axis, then local search
    # around the best solutions. Fixed seed makes the result reproducible.
    rng = np.random.default_rng(20260928)
    placements = [[0., 0., 0., 90.]]
    for x in [-.20, 0., .15, .30]:
        for y in [-.20, 0., .20]:
            for z in [0., .15, .30]:
                for yaw in [45., 90., 135.]:
                    placements.append([x, y, z, yaw])
    rows = []
    for i, base in enumerate(placements):
        solved = screen_batch(solver, targets, base, random_seeds=0,
                              max_nfev=args.max_nfev)
        count = sum(r['ok'] for r in solved)
        rows.append(dict(base=base, successes=count))
        if i % 18 == 0:
            print(f'placement coarse {i+1}/{len(placements)} best={max(r["successes"] for r in rows)}', flush=True)
    top = sorted(rows, key=lambda r: r['successes'], reverse=True)[:8]
    local = []
    for parent in top:
        for _ in range(args.local_samples):
            b = np.array(parent['base']) + rng.normal(size=4) * [.05, .07, .06, 16.]
            b[:3] = np.clip(b[:3], [-.30, -.35, -.10], [.40, .35, .40])
            b[3] = np.clip(b[3], 0., 180.)
            base = b.tolist()
            solved = screen_batch(solver, targets, base, random_seeds=0,
                                  max_nfev=args.max_nfev)
            local.append(dict(base=base, successes=sum(r['ok'] for r in solved)))
    ranked = sorted(rows + local, key=lambda r: r['successes'], reverse=True)
    # Re-evaluate finalists with multi-start and a larger solve budget.
    finalists = []
    seen = set()
    for entry in ranked:
        base = entry['base']
        key = tuple(round(v, 3) for v in base)
        if key in seen:
            continue
        seen.add(key)
        solved = screen_batch(solver, targets, base, random_seeds=max(args.seeds, 3),
                              max_nfev=max(args.max_nfev, 100))
        finalists.append(dict(base=base, summary=summarize(solved), solutions=solved))
        if len(finalists) >= 12:
            break
    finalists.sort(key=lambda r: r['summary']['unique_successes'], reverse=True)
    return dict(coarse_count=len(placements), local_count=len(local),
                current=dict(base=[0.,0.,0.,90.], summary=summarize(screen_batch(
                    solver, targets, [0.,0.,0.,90.], random_seeds=max(args.seeds, 3), max_nfev=100))),
                finalists=finalists)


def tcp_ablation(solver, targets, base, args):
    variants = ['current', 'flange', 'length_-0.08', 'length_-0.04',
                'length_-0.02', 'length_0.02', 'length_0.04', 'length_0.08']
    report = {}
    for variant in variants:
        solved = screen_batch(solver, targets, base, variant=variant,
                              random_seeds=max(args.seeds, 3), max_nfev=100)
        report[variant] = dict(summary=summarize(solved), solutions=solved)
        print('TCP', variant, report[variant]['summary']['unique_successes'], flush=True)
    return report


def position_only(solver, targets, base):
    rows = []
    for T in base_targets(targets, base):
        target = T[:3, 3]
        best = None
        for seed in solver.seeds(extra=3):
            fit = least_squares(
                lambda q: solver.chain.forward(dict(zip(JOINTS, q)))[:3, 3] - target,
                seed, bounds=(solver.lo, solver.hi), max_nfev=100)
            error = float(np.linalg.norm(
                solver.chain.forward(dict(zip(JOINTS, fit.x)))[:3, 3] - target))
            best = error if best is None else min(best, error)
            if best < .002:
                break
        rows.append(dict(position_ik=best < .002, error_m=best))
    return dict(base=base, successes=sum(r['position_ik'] for r in rows),
                candidates_successes=10 * sum(r['position_ik'] for r in rows),
                acceptance_m=.002, results=rows)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--task', choices=['map', 'placement', 'tcp', 'single', 'position'], required=True)
    p.add_argument('--base', nargs=4, type=float, default=[0.,0.,0.,90.])
    p.add_argument('--seeds', type=int, default=1)
    p.add_argument('--max-nfev', type=int, default=55)
    p.add_argument('--local-samples', type=int, default=6)
    p.add_argument('--j6-extra-deg', type=float, default=0.,
                   help='hypothetical limit expansion for causal ablation only')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    targets = frozen_targets()
    solver = NumericalIK()
    solver.lo[5] -= math.radians(a.j6_extra_deg)
    solver.hi[5] += math.radians(a.j6_extra_deg)
    start = time.time()
    if a.task == 'map': report = scan_map(solver, targets, a)
    elif a.task == 'placement': report = search_base(solver, targets, a)
    elif a.task == 'tcp': report = tcp_ablation(solver, targets, a.base, a)
    elif a.task == 'position': report = position_only(solver, targets, a.base)
    else:
        rows = screen_batch(solver, targets, a.base, random_seeds=max(a.seeds, 6), max_nfev=120)
        report = dict(base=a.base, summary=summarize(rows), solutions=rows)
    report['elapsed_s'] = time.time() - start
    report['frozen_sha256'] = '50e0a2f758a46edb51f21dee75eae8283cc711f793c2fade7f2c2c9402ea86f3'
    report['hypothetical_j6_limit_extension_deg'] = a.j6_extra_deg
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2) + '\n')
    print('DONE', a.output, report['elapsed_s'], flush=True)


if __name__ == '__main__':
    main()
