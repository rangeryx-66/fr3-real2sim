"""Evaluator-only MoveIt preflight for a bounded 47686 placement grid.

The source candidates were generated using an asset-derived mask, so this
cannot be reported as a SAM3 grasp trial. No robot or cabinet motion is sent.
"""
import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src'))
from articulated_demo.backend import ArticulatedBackend, Failure, matrix
from grasp_compare.adapters import read_candidates
from grasp_compare.scene import load_scene
import r1a7_plant as plant


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--diagnostic-dir', type=Path, required=True)
    p.add_argument('--asset-root', type=Path, required=True)
    p.add_argument('--x', type=float, required=True)
    p.add_argument('--y', type=float, required=True)
    p.add_argument('--yaw-deg', type=float, required=True)
    p.add_argument('--ranks', default='60')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    source = args.diagnostic_dir
    candidates = {c.rank:c for c in read_candidates(source/'graspgenx_native.json',
        'graspgenx',load_scene(source/'oracle_scene.npz'))}
    delta = Rotation.from_euler('z',args.yaw_deg+90.,degrees=True).as_matrix()
    old_origin = np.array([.45,.05,0.])
    new_origin = np.array([args.x,args.y,0.])
    rows = []
    result = {'kind':'evaluator-only placement preflight; no SAM3 or execution',
        'asset_pose':[args.x,args.y,args.yaw_deg], 'candidates':rows}
    rclpy.init()
    node = ArticulatedBackend(args.asset_root/'urdf/47686.urdf')
    try:
        node.scene()
        result['fk_tcp_check']=node.check_fk()
        state=plant.state()
        for rank in [int(value) for value in args.ranks.split(',')]:
            original = candidates[rank].T_B_TCP
            T=np.eye(4)
            T[:3,:3]=delta@original[:3,:3]
            T[:3,3]=new_origin+delta@(original[:3,3]-old_origin)
            row={'rank':rank,'target_tcp':T.tolist()}
            rows.append(row)
            try:
                preflight=node.candidate_plan(SimpleNamespace(T_B_TCP=T))
                row.update(status='PREGRASP_APPROACH_PLAN',
                    grasp_margin_rad=preflight['grasp_margin_rad'],
                    path_margin_rad=preflight['path_margin_rad'])
                moving=matrix(state['moving_pose']['position'],
                    state['moving_pose']['quaternion_wxyz'])
                arc=[]
                seed=node.ik(T,node.measured(),random_seeds=12)
                for angle in (5,10,15,20):
                    waypoint=node.chain.target_tcp(moving_link=state['moving_link'],
                        joint_name=state['joint_name'],q_now=state['joint_q'],
                        q_target=state['joint_q']+np.deg2rad(angle),
                        T_world_moving_now=moving,T_world_tcp_now=T)
                    try:
                        seed=node.ik(waypoint,seed,random_seeds=8)
                        arc.append({'door_delta_deg':angle,'status':'IK_AND_STATIC_SCENE_VALID',
                            'margin_rad':node.margin(seed)})
                    except Failure as error:
                        arc.append({'door_delta_deg':angle,'status':error.category,'detail':str(error)})
                        break
                row['arc']=arc
            except Failure as error:
                row.update(status=error.category,detail=str(error))
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(result,indent=2))
            print(rank,row['status'],row.get('arc'),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
