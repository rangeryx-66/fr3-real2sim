"""Kinematic prescreen of a fixed nearby 47686 installation grid.

This is diagnostic only. Every selected pose still requires a fresh camera
capture, model inference, collision check and complete MoveIt plan.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src'))
from urdf_chain import KinematicChain
from articulated_demo.kinematics import URDFChain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diagnostic-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deadline-s', type=float, default=600)
    parser.add_argument('--variants', type=Path,
                        help='Dex1 collision-free variants to prescreen instead of raw grasps')
    parser.add_argument('--max-variants', type=int, default=60)
    parser.add_argument('--grid-mode', choices=('full','focused','expanded'), default='full')
    parser.add_argument('--arc',action='store_true',
                        help='prescreen pregrasp and 22 degrees relative to source joint angle; live preflight validates absolute 0→22')
    parser.add_argument('--asset-root',type=Path,
        default=Path('/data1/home/rangeryx/datasets/physx_mobility/prepared/47686_v1'))
    parser.add_argument('--initial-joint-rad',type=float,default=.068,
                        help='measured settled cabinet angle in the reference capture')
    args = parser.parse_args()
    source = args.diagnostic_dir
    model = ROOT / 'config/r1a7_dex1.urdf'
    names = tuple(f'J{i}' for i in range(1, 8))
    joints = {j.get('name'): j for j in ET.parse(model).findall('joint')}
    lo = np.array([float(joints[n].find('limit').get('lower')) for n in names]) + .01
    hi = np.array([float(joints[n].find('limit').get('upper')) for n in names]) - .01
    chain = KinematicChain(model, 'r1a7_world', 'r1a7_tcp', names)
    if args.arc:
        asset_chain=URDFChain(args.asset_root/'urdf/47686.urdf')
        manifest=json.loads((args.asset_root/'manifest.json').read_text())
        moving='abstract_2_1'
        joint_name=manifest['joint_name']
        root_moving0=asset_chain.root_to_link(moving,{joint_name:args.initial_joint_rad})
        source_to_world=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
        asset_height=.18-float(manifest['scale_source_to_meters'])*float(manifest['static_source_bounds'][0][1])
    from grasp_compare.adapters import read_candidates
    from grasp_compare.scene import load_scene
    scene = load_scene(source / 'oracle_scene.npz')
    candidates = {c.rank: c for c in read_candidates(source / 'graspgenx_native.json', 'graspgenx', scene)}
    common = json.loads((source / 'common_filter/candidates.json').read_text())
    ranks = [r['rank'] for r in common['candidates']['graspgenx']
             if r['checks']['dex1_scene_collision']['status'] == 'FREE']
    # Every raw collision-free pose is considered. No pose adaptation or object-specific target.
    if args.variants:
        variants = json.loads(args.variants.read_text())['candidates']
        # Cover distinct source grasps before filling with high-score variants.
        first_by_rank = {}
        for variant in variants:
            first_by_rank.setdefault(variant['raw_rank'], variant)
        ordered = list(first_by_rank.values())
        ordered.extend(variant for variant in variants if variant not in ordered)
        targets = [(f"{item['raw_rank']}:{item['variant']}", np.asarray(item['T_B_TCP']))
                   for item in ordered[:args.max_variants]]
    else:
        targets = [(rank, candidates[rank].T_B_TCP) for rank in ranks]
    reference_xyz = np.array([.45, .05, 0.])
    reference_yaw = -90.
    grid = ([(x, y, yaw) for x in (.20, .275, .35, .425, .50)
            for y in (-.10, -.025, .05, .125, .20)
            for yaw in (-180., -150., -120., -90., -60.)]
            if args.grid_mode == 'full' else
            [(x, y, yaw) for x in (.35,.425,.50)
             for y in (.125,.20,.275) for yaw in (-150.,-120.,-90.)]
            if args.grid_mode == 'expanded' else
            [(x, y, yaw) for x in (.20, .275, .35)
             for y in (.05, .125, .20)
             for yaw in (-150., -120., -90.)])
    rng = np.random.default_rng(47686)
    seeds = [np.array([0., 1.3, 1., -1.3, 0., 0., 0.]),
             *rng.uniform(lo, hi, size=(2, 7))]
    rows = []
    stop = time.monotonic() + args.deadline_s
    for x, y, yaw in grid:
        if time.monotonic() >= stop:
            break
        delta = Rotation.from_euler('z', yaw - reference_yaw, degrees=True).as_matrix()
        transformed = []
        if args.arc:
            T_world_asset=np.eye(4)
            T_world_asset[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@source_to_world
            T_world_asset[:3,3]=[x,y,asset_height]
            inv_world_asset=np.linalg.inv(T_world_asset)
        for rank, original in targets:
            T = np.eye(4)
            T[:3, :3] = delta @ original[:3, :3]
            T[:3, 3] = np.array([x, y, 0.]) + delta @ (original[:3, 3] - reference_xyz)
            transformed.append((rank, T))
        solutions = []
        for rank, T in transformed:
            def residual(q):
                F = chain.forward(dict(zip(names, q)))
                angle = (Rotation.from_matrix(T[:3, :3]).inv() *
                         Rotation.from_matrix(F[:3, :3])).as_rotvec()
                return np.r_[F[:3, 3] - T[:3, 3], .08 * angle]
            best = None
            for initial in seeds:
                fit = least_squares(residual, np.clip(initial, lo, hi), bounds=(lo, hi),
                                    max_nfev=80, ftol=1e-4, xtol=1e-4, gtol=1e-4)
                score = np.linalg.norm(fit.fun)
                if best is None or score < best[0]:
                    best = (score, fit.x, fit.fun)
                if np.linalg.norm(fit.fun[:3]) < .004 and np.linalg.norm(fit.fun[3:]) < .08 * np.deg2rad(3):
                    break
            position_error = float(np.linalg.norm(best[2][:3]))
            orientation_error = float(np.rad2deg(np.linalg.norm(best[2][3:]) / .08))
            solution = {'rank': rank, 'position_error_m': position_error,
                        'orientation_error_deg': orientation_error,
                        'joint_margin_rad': float(np.min(np.minimum(best[1]-lo, hi-best[1]))),
                        'q': best[1].tolist(),
                        'kinematic_feasible': position_error < .004 and orientation_error < 3.}
            if args.arc and solution['kinematic_feasible']:
                def fit_one(target, initial):
                    target_rot=Rotation.from_matrix(target[:3,:3])
                    def error(q):
                        actual=chain.forward(dict(zip(names,q)))
                        angle=(target_rot.inv()*Rotation.from_matrix(actual[:3,:3])).as_rotvec()
                        return np.r_[actual[:3,3]-target[:3,3],.08*angle]
                    fit=least_squares(error,np.clip(initial,lo,hi),bounds=(lo,hi),
                        max_nfev=60,ftol=1e-4,xtol=1e-4,gtol=1e-4)
                    return fit.x,np.linalg.norm(fit.fun[:3])<.005 and np.linalg.norm(fit.fun[3:])<.08*np.deg2rad(5)
                pre=T.copy();pre[:3,3]-=.08*T[:3,2]
                pre_q,pre_ok=fit_one(pre,best[1])
                solution['pregrasp_kinematic']=bool(pre_ok)
                q_robot=best[1]
                path_margin=float(min(np.min(np.minimum(best[1]-lo,hi-best[1])),
                                      np.min(np.minimum(pre_q-lo,hi-pre_q))))
                reached=0
                for angle in np.deg2rad(np.arange(2,24,2)):
                    moving_q=asset_chain.root_to_link(moving,{joint_name:args.initial_joint_rad+float(angle)})
                    target=T_world_asset@moving_q@np.linalg.inv(root_moving0)@inv_world_asset@T
                    q_robot,ok=fit_one(target,q_robot)
                    if not ok or time.monotonic()>=stop:break
                    path_margin=min(path_margin,float(np.min(np.minimum(q_robot-lo,hi-q_robot))))
                    reached=int(np.rad2deg(angle)+.1)
                solution['arc_reached_deg']=reached
                solution['path_joint_margin_rad']=path_margin
                solution['full_kinematic_path']=bool(pre_ok and reached>=22)
                solution['safe_full_kinematic_path']=bool(pre_ok and reached>=22 and path_margin>.05)
            solutions.append(solution)
        row = {'asset_x_m': x, 'asset_y_m': y, 'asset_yaw_deg': yaw,
               'feasible_count': sum(s['kinematic_feasible'] for s in solutions),
               'full_kinematic_path_count':sum(s.get('full_kinematic_path',False) for s in solutions),
               'safe_full_kinematic_path_count':sum(s.get('safe_full_kinematic_path',False) for s in solutions),
               'best_weighted_residual': min(s['position_error_m'] + .08*np.deg2rad(s['orientation_error_deg']) for s in solutions),
               'solutions': solutions}
        rows.append(row)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({'kind':'kinematic prescreen; oracle mask used only for diagnostics',
             'source_pose': [*reference_xyz[:2], reference_yaw], 'tested': len(rows),
             'grid_size': len(grid), 'rows': rows}, indent=2))
        print(f"{len(rows)}/{len(grid)} pose=({x:.3f},{y:.3f},{yaw:.0f}) feasible={row['feasible_count']}", flush=True)


if __name__ == '__main__':
    main()
