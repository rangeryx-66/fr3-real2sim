#!/usr/bin/env python3
"""Read-only MoveIt IK verification of frozen grasps under a base/TCP choice."""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from r1a7_workspace_diagnostics import NumericalIK, frozen_targets, transform, JOINTS, HOME


class Client(Node):
    def __init__(self):
        super().__init__('r1a7_workspace_ik_verification')
        self.client = self.create_client(GetPositionIK, '/compute_ik')
        if not self.client.wait_for_service(timeout_sec=60):
            raise RuntimeError('MoveIt compute_ik service unavailable')

    def ik(self, target, seed, collision, timeout):
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = 'r1a7_arm'
        ik.ik_link_name = 'r1a7_tcp'
        ik.avoid_collisions = collision
        ik.timeout.sec = int(timeout)
        ik.timeout.nanosec = int((timeout % 1) * 1e9)
        ik.robot_state = RobotState()
        ik.robot_state.joint_state = JointState()
        ik.robot_state.joint_state.name = list(JOINTS) + ['dex1_Joint1_1', 'dex1_Joint2_1']
        ik.robot_state.joint_state.position = list(map(float, seed)) + [-.02, -.02]
        pose = PoseStamped()
        pose.header.frame_id = 'r1a7_world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, target[:3, 3])
        q = Rotation.from_matrix(target[:3, :3]).as_quat()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
        ik.pose_stamped = pose
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout + 5)
        if not future.done() or future.exception():
            raise RuntimeError('MoveIt IK service timeout')
        response = future.result()
        if response.error_code.val != 1:
            return None, response.error_code.val
        values = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        return np.array([values[n] for n in JOINTS]), 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--base', nargs=4, type=float, required=True,
                   metavar=('X', 'Y', 'Z', 'YAW_DEG'))
    p.add_argument('--variant', default='current')
    p.add_argument('--seeds', type=int, default=12)
    p.add_argument('--timeout', type=float, default=.25)
    p.add_argument('--grid', action='store_true', help='x-z heatmap for all frozen orientations')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    solver = NumericalIK()
    rng = np.random.default_rng(20260928)
    # The loaded robot has world_mount yaw +90. Convert desired physical base
    # pose into this fixed model frame without changing either URDF or grasps.
    world_to_model = transform(yaw=90.) @ np.linalg.inv(transform(*args.base))
    rclpy.init()
    node = Client()
    try:
        if args.grid:
            xs = np.round(np.linspace(.40, .55, 11), 5)
            zs = np.round(np.linspace(.02, .25, 13), 5)
            raw = frozen_targets()
            reference = raw[0][:3, 3].copy()
            counts = []
            rng = np.random.default_rng(20260928)
            for z in zs:
                line = []
                for x in xs:
                    found = 0
                    for T in raw:
                        shifted = T.copy()
                        shifted[:3, 3] += [x-reference[0], 0., z-reference[2]]
                        target = world_to_model @ shifted
                        seeds = [HOME, *rng.uniform(solver.lo+.01, solver.hi-.01,
                                                    size=(args.seeds, 7))]
                        for seed in seeds:
                            q, _ = node.ik(target, seed, False, args.timeout)
                            if q is not None:
                                found += 1
                                break
                    line.append(found)
                counts.append(line)
                print(f'grid z={z:.3f}: {line}', flush=True)
            output = dict(base_xyz_yaw=list(args.base), x_m=xs.tolist(), z_m=zs.tolist(),
                          per_cell_11_orientation_successes=counts,
                          seeds=1+args.seeds, timeout_s=args.timeout,
                          orientation_source='frozen AnyGrasp trial_01, 11 unmodified orientations',
                          position_shift_reference='rank 0 TCP center')
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(output, indent=2) + '\n')
            print('DONE GRID', args.output, flush=True)
            return
        results = []
        for i, raw in enumerate(frozen_targets()):
            original_tcp_target = solver.target_for_variant(raw, args.variant)
            target = world_to_model @ original_tcp_target
            seeds = [HOME, *rng.uniform(solver.lo + .01, solver.hi - .01,
                                        size=(args.seeds, 7))]
            codes = []
            solution = None
            for seed in seeds:
                q, code = node.ik(target, seed, False, args.timeout)
                codes.append(code)
                if q is not None:
                    solution = q
                    break
            row = dict(rank=i, kinematic_ik=solution is not None, calls=len(codes), codes=codes)
            if solution is not None:
                valid, collision_code = node.ik(target, solution, True, args.timeout)
                margin = np.minimum(solution - solver.lo, solver.hi - solution)
                row.update(collision_on_ik=valid is not None,
                           collision_on_code=collision_code,
                           q=solution.tolist(),
                           joint_margins_rad=dict(zip(JOINTS, margin.tolist())),
                           nearest_limit=JOINTS[int(np.argmin(margin))])
            results.append(row)
            print(f'{i+1}/11 IK={row["kinematic_ik"]} codes={codes}', flush=True)
        kinematic = sum(r['kinematic_ik'] for r in results)
        collision = sum(r.get('collision_on_ik', False) for r in results)
        output = dict(base_xyz_yaw=list(args.base), variant=args.variant,
                      seeds=args.seeds, timeout_s=args.timeout,
                      unique_kinematic_ik=kinematic, candidates_kinematic_ik=10*kinematic,
                      unique_collision_on_ik=collision, candidates_collision_on_ik=10*collision,
                      results=results)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output, indent=2) + '\n')
        print('DONE', args.output, kinematic, collision, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
