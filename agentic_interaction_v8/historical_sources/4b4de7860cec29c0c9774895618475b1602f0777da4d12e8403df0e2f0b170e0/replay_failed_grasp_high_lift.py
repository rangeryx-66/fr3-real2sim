"""Continue a failed Isaac grasp through a higher MoveIt Cartesian lift.

This is a diagnostic continuation. It deliberately bypasses the benchmark's
early contact-loss stop, but keeps MoveIt collision checks and does not count
as a benchmark success trial.
"""

import argparse
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

from run_r1a7_generalization import ROOT, SIM, ready, start, stop

sys.path.insert(0, str(ROOT / 'src'))
import r1a7_plant as plant
from r1a7_grasp_adaptation import path_targets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=4)
    parser.add_argument('--port', type=int, default=18792)
    parser.add_argument('--ros-domain', type=int, default=232)
    parser.add_argument('--force-open-loop-ik', action='store_true',
                        help='Simulation-only: bypass MoveIt path collision after a rejected lift')
    args = parser.parse_args()
    trial = json.loads(args.trial.read_text())
    if trial.get('success') or trial.get('category') not in ('CONTACT_LOSS', 'COLLISION'):
        raise ValueError('expected a failed grasp with contact loss or collision')
    if not trial.get('selected_rank') and trial.get('selected_rank') != 0:
        raise ValueError('trial did not select an executed candidate')
    prior = trial['executions']
    through_micro = prior[:next((i + 1 for i, x in enumerate(prior)
                                 if x['stage'] == 'MICRO_LIFT'), len(prior))]
    if [x['stage'] for x in through_micro] != ['PREGRASP', 'APPROACH', 'MICRO_LIFT']:
        raise ValueError('unexpected recorded execution sequence')
    chosen = next(c for c in trial['candidates'] if c['rank'] == trial['selected_rank'])
    lift_target = path_targets(np.asarray(chosen['adapted_T_B_TCP']))[2]
    output = args.output.resolve()
    env = {**os.environ, 'R1A7_BASE_POSE': '0.329,-0.175,0.237,56.295',
           'R1A7_PEDESTAL_SIZE': '0.10,0.10,0.20',
           'R1A7_PRESENTATION_CAMERA': '1',
           'R1A7_PLANT_PORT': str(args.port),
           'ROS_DOMAIN_ID': str(args.ros_domain), 'ROS_LOCALHOST_ONLY': '1',
           'NO_PROXY': '127.0.0.1,localhost', 'no_proxy': '127.0.0.1,localhost',
           'OMNI_KIT_ACCEPT_EULA': 'YES', 'ACCEPT_EULA': 'Y'}
    for key in ('R1A7_BASE_POSE', 'R1A7_PEDESTAL_SIZE', 'R1A7_PLANT_PORT',
                'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'NO_PROXY', 'no_proxy'):
        os.environ[key] = env[key]
    plant.URL = f'http://127.0.0.1:{args.port}'
    processes = []
    node = None
    recording = False
    try:
        processes.append(start([SIM, '-u', str(ROOT / 'src/r1a7_sim_server.py'),
                                '--gpu', str(args.gpu), '--port', str(args.port),
                                '--arena-target', trial['object']['id']],
                               output.with_suffix('.sim.log'), env))
        processes.append(start(['ros2', 'launch', str(ROOT / 'src/r1a7_moveit.launch.py')],
                               output.with_suffix('.moveit.log'), env))
        processes.append(start([sys.executable, '-u', str(ROOT / 'src/r1a7_ros_bridge.py')],
                               output.with_suffix('.bridge.log'), env))
        ready(args.port, trial['object']['id'], processes)
        import rclpy
        from r1a7_backend import R1A7Backend, Failure
        rclpy.init()
        node = R1A7Backend()
        node.executions = []
        node.stage = 'RESET'
        seed = trial['scenario']['seed']
        reset = plant.command({'op': 'reset', 'seed': seed})
        if not reset['ok']:
            raise RuntimeError(reset)
        plant.settle(.8)
        node.scene()
        node.gripper(.09)
        state = plant.state()
        initial_z = state['box'][2]
        initial_tcp_z = state['tcp'][2]
        if not plant.command({'op': 'video_start', 'path': str(output)})['ok']:
            raise RuntimeError('video_start failed')
        recording = True
        plant.settle(.4)
        stage_data = []
        for execution in through_micro:
            if execution['stage'] == 'MICRO_LIFT':
                node.gripper(0.)
                plant.settle(.3)
            result = plant.command({'op': 'trajectory',
                                    'names': execution['joint_names'],
                                    'points': execution['points']}, timeout=180)
            if not result['ok']:
                raise RuntimeError(f'{execution["stage"]}: {result}')
            stage_data.append({'stage': execution['stage'], 'box_z_m': plant.state()['box'][2],
                               'tcp_z_m': plant.state()['tcp'][2]})
        after_micro = plant.state()
        node.scene()
        planning_error = None
        forced_ik = None
        try:
            node.stage = 'FORCED_DIAGNOSTIC_LIFT'
            trajectory = node.cartesian(node.measured(), lift_target)
            node.execute(trajectory, 'NO_PLAN')
        except Failure as error:
            planning_error = {'category': error.category, 'detail': str(error)}
            if args.force_open_loop_ik:
                from moveit_msgs.srv import GetPositionIK
                from r1a7_backend import GROUP, JOINTS, TCP, stamped
                seed_state = node.measured()
                grasp = np.asarray(chosen['adapted_T_B_TCP'], dtype=float)
                points = []
                rejected = None
                for index, dz in enumerate((.04, .06, .08, .10), 1):
                    target = grasp.copy(); target[2, 3] += dz
                    request = GetPositionIK.Request()
                    ik = request.ik_request
                    ik.group_name = GROUP; ik.ik_link_name = TCP
                    ik.pose_stamped = stamped(target)
                    ik.robot_state = seed_state
                    ik.avoid_collisions = False
                    ik.timeout.nanosec = 500_000_000
                    solved = node.call('compute_ik', request)
                    if solved.error_code.val != 1:
                        rejected = {'height_m': dz, 'ik_code': solved.error_code.val}
                        break
                    seed_state = solved.solution
                    values = dict(zip(seed_state.joint_state.name,
                                      seed_state.joint_state.position))
                    points.append({'t': 2.0 * index,
                                   'q': [values[name] for name in JOINTS]})
                if points:
                    result = plant.command({'op': 'trajectory', 'names': list(JOINTS),
                                            'points': points}, timeout=180)
                    forced_ik = {'collision_planning_bypassed': True,
                                 'target_heights_m': [v * .02 + .02 for v in range(1, len(points) + 1)],
                                 'trajectory_result': result,
                                 'rejected': rejected}
                else:
                    forced_ik = {'collision_planning_bypassed': True,
                                 'target_heights_m': [], 'rejected': rejected}
        plant.settle(2.2)
        final = plant.state()
        video_result = plant.command({'op': 'video_stop'}, timeout=90)
        recording = False
        if not video_result['ok']:
            raise RuntimeError(video_result)
        samples = [s for s in final['history'] if s['t'] >= after_micro['t']]
        report = {'kind': 'high-lift diagnostic continuation; excluded from benchmark',
                  'source_trial': str(args.trial.resolve()), 'source_category': trial['category'],
                  'seed': seed, 'object': trial['object']['id'],
                  'initial_box_z_m': initial_z, 'initial_tcp_z_m': initial_tcp_z,
                  'after_micro_box_z_m': after_micro['box'][2],
                  'after_micro_tcp_z_m': after_micro['tcp'][2],
                  'final_box_z_m': final['box'][2], 'final_tcp_z_m': final['tcp'][2],
                  'max_box_lift_m': max((s['z'] - initial_z for s in samples), default=None),
                  'planning_error': planning_error, 'forced_open_loop_ik': forced_ik,
                  'stages': stage_data,
                  'frames': video_result['frames'], 'video': str(output)}
        output.with_suffix('.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    finally:
        if recording:
            try:
                plant.command({'op': 'video_stop'}, timeout=90)
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass
        stop(processes)


if __name__ == '__main__':
    main()
