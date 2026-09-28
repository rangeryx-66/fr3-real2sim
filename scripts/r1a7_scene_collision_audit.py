#!/usr/bin/env python3
"""Audit frozen grasp states against the live R1 MoveIt planning scene.

Requires Isaac, the R1 bridge, and MoveIt with the same R1A7_BASE_POSE.
It requests IK and state validity only; it never plans or executes motion.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from r1a7_backend import R1A7Backend, JOINTS, GROUP, TCP, stamped
from r1a7_frames import grasp_to_r1a7_tcp

FROZEN = ROOT / 'results/r1a7_workspace/frozen_anygrasp_trial01.json'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, default=12)
    args = parser.parse_args()
    data = json.loads(FROZEN.read_text())
    rclpy.init()
    node = R1A7Backend()
    try:
        node.scene()
        measured = node.measured()
        values = dict(zip(measured.joint_state.name, measured.joint_state.position))
        rng = np.random.default_rng(20260928)
        seeds = [measured]
        for _ in range(args.seeds):
            state = RobotState()
            state.joint_state = JointState()
            state.joint_state.name = list(measured.joint_state.name)
            joint_values = values.copy()
            for joint in JOINTS:
                lo, hi = node.limits[joint]
                joint_values[joint] = float(rng.uniform(lo + .01, hi - .01))
            state.joint_state.position = [joint_values[n] for n in state.joint_state.name]
            seeds.append(state)
        rows = []
        for grasp in data['grasps']:
            target = grasp_to_r1a7_tcp(data['T_B_C'], grasp['rotation'],
                                        grasp['translation'], grasp['depth'])[1]
            solutions = []
            codes = []
            for seed in seeds:
                request = GetPositionIK.Request()
                ik = request.ik_request
                ik.group_name = GROUP
                ik.ik_link_name = TCP
                ik.pose_stamped = stamped(target)
                ik.robot_state = seed
                ik.avoid_collisions = False
                ik.timeout.nanosec = 250_000_000
                response = node.call('compute_ik', request)
                codes.append(response.error_code.val)
                if response.error_code.val == 1:
                    solutions.append(response.solution)
            row = dict(rank=grasp['rank'], tcp_z_m=float(target[2, 3]),
                       kinematic_ik=bool(solutions), codes=codes)
            if solutions:
                state = max(solutions, key=node.margin)
                request = GetStateValidity.Request()
                request.group_name = GROUP
                request.robot_state = state
                response = node.call('check_state_validity', request)
                row['joint_margin_rad'] = node.margin(state)
                row['scene_collision_free'] = bool(response.valid)
                row['contacts'] = [dict(a=c.contact_body_1, b=c.contact_body_2,
                                        depth_m=float(c.depth)) for c in response.contacts]
            rows.append(row)
            print(grasp['rank'], row['kinematic_ik'], row.get('scene_collision_free'),
                  [(c['a'], c['b'], round(c['depth_m'], 4))
                   for c in row.get('contacts', [])], flush=True)
        output = dict(scene=['table', 'r1a7_pedestal', 'box'],
                      frozen_candidates=11, benchmark_multiplicity=10,
                      summary=dict(kinematic_ik=sum(r['kinematic_ik'] for r in rows),
                                   scene_collision_free=sum(r.get('scene_collision_free', False) for r in rows),
                                   table_collision=sum(any('table' in (c['a'], c['b'])
                                                           for c in r.get('contacts', [])) for r in rows),
                                   pedestal_collision=sum(any('r1a7_pedestal' in (c['a'], c['b'])
                                                              for c in r.get('contacts', [])) for r in rows)),
                      candidates=rows)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output, indent=2) + '\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
