#!/usr/bin/env python3
"""Exercise the real R1 MoveIt planner through one frozen pregrasp/approach.

This probe calls OMPL and Cartesian path services, but never sends a robot
execution goal. Use the R1 backend for the physical benchmark.
"""
import argparse
import json
import sys
from pathlib import Path

import rclpy
from moveit_msgs.srv import GetCartesianPath

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from r1a7_backend import R1A7Backend, GROUP, TCP, BASE, Failure, pose
from r1a7_frames import grasp_to_r1a7_tcp

FROZEN = ROOT / 'results/r1a7_workspace/frozen_anygrasp_trial01.json'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rank', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(FROZEN.read_text())
    grasp = next(g for g in data['grasps'] if g['rank'] == args.rank)
    pre, target, _, _ = grasp_to_r1a7_tcp(
        data['T_B_C'], grasp['rotation'], grasp['translation'], grasp['depth'])
    rclpy.init()
    node = R1A7Backend()
    result = dict(rank=args.rank)
    try:
        result['tcp_alignment'] = node.check_fk()
        node.scene()
        measured = node.measured()
        try:
            pre_state = node.ik(pre, measured)
            result['pregrasp_margin_rad'] = node.margin(pre_state)
            trajectory = node.plan(measured, pre_state)
            result['pregrasp_plan'] = dict(success=True,
                                           points=len(trajectory.joint_trajectory.points),
                                           min_margin_rad=node.trajectory_margin(trajectory))
            request = GetCartesianPath.Request()
            request.header.frame_id = BASE
            request.start_state = pre_state
            request.group_name = GROUP
            request.link_name = TCP
            request.waypoints = [pose(target)]
            request.max_step = .003
            request.jump_threshold = 1.5
            request.avoid_collisions = True
            response = node.call('compute_cartesian_path', request)
            result['approach'] = dict(fraction=float(response.fraction),
                                      code=response.error_code.val,
                                      complete=response.error_code.val == 1 and response.fraction >= .999,
                                      points=len(response.solution.joint_trajectory.points))
        except Failure as error:
            result['failure'] = dict(category=error.category, detail=str(error))
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
