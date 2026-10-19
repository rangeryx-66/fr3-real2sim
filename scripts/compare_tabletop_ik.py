#!/usr/bin/env python3
"""Same 81 world-frame TCP poses through FR3, PiPER, and R1-7a MoveIt IK.

Start exactly one robot's MoveIt in an isolated ROS domain, then run this
script with the matching --robot. No simulator or grasp inference is needed.
"""
from __future__ import annotations
import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetStateValidity
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from urdf_chain import KinematicChain


def reference_pose(cell, quaternion):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    T[:3, 3] = [cell['x'], cell['y'], cell['z']]
    return T


def rotated(T, degrees):
    yield T
    for axis in np.eye(3):
        for sign in (-1, 1):
            out = T.copy()
            out[:3, :3] = T[:3, :3] @ Rotation.from_rotvec(axis * sign * math.radians(degrees)).as_matrix()
            yield out
    for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        out = T.copy()
        out[:3, :3] = T[:3, :3] @ Rotation.from_euler('xy', [sx*degrees/math.sqrt(2), sy*degrees/math.sqrt(2)], degrees=True).as_matrix()
        yield out


class IK(Node):
    def __init__(self, args):
        super().__init__(f'compare_ik_{args.robot}')
        self.args = args
        self.names = args.joints.split(',')
        self.home = np.array([float(x) for x in args.home.split(',')])
        if len(self.names) != len(self.home):
            raise ValueError('joint and home lengths differ')
        joints = {j.get('name'): j for j in ET.parse(args.urdf).getroot().findall('joint')}
        self.lo = np.array([float(joints[n].find('limit').get('lower')) for n in self.names])
        self.hi = np.array([float(joints[n].find('limit').get('upper')) for n in self.names])
        self.chain = KinematicChain(args.urdf, args.base, args.tip, tuple(self.names))
        self.rpc = {}
        for name, typ in [('compute_ik', GetPositionIK), ('compute_fk', GetPositionFK),
                          ('check_state_validity', GetStateValidity)]:
            client = self.create_client(typ, '/' + name)
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError('MoveIt service unavailable: ' + name)
            self.rpc[name] = client
        self.rng = np.random.default_rng(20260928)

    def call(self, name, request):
        future = self.rpc[name].call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done() or future.exception():
            raise RuntimeError(f'{name} failed or timed out')
        return future.result()

    def state(self, q):
        s = RobotState()
        s.joint_state = JointState()
        fingers = self.args.fingers.split(',') if self.args.fingers else []
        positions = [float(x) for x in self.args.finger_positions.split(',')] if fingers else []
        s.joint_state.name = self.names + fingers
        s.joint_state.position = list(map(float, q)) + positions
        return s

    def q(self, state):
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        return np.array([values[n] for n in self.names])

    def target_in_base(self, T):
        out = np.eye(4)
        out[:3, :3] = Rotation.from_euler('z', -self.args.base_yaw_deg, degrees=True).as_matrix()
        out[:3, 3] = -out[:3, :3] @ np.array(self.args.base_xyz)
        return out @ T

    def ik(self, T, seed, collision=False):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.args.group
        ik.ik_link_name = self.args.tip
        ik.robot_state = self.state(seed)
        ik.avoid_collisions = collision
        ik.timeout.sec = int(self.args.timeout)
        ik.timeout.nanosec = int((self.args.timeout % 1) * 1e9)
        p = PoseStamped()
        p.header.frame_id = self.args.base
        p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, T[:3, 3])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = map(float, q)
        ik.pose_stamped = p
        result = self.call('compute_ik', request)
        if result.error_code.val != 1:
            return None, result.error_code.val
        return self.q(result.solution), 1

    def fk(self, q):
        request = GetPositionFK.Request()
        request.header.frame_id = self.args.base
        request.fk_link_names = [self.args.tip]
        request.robot_state = self.state(q)
        result = self.call('compute_fk', request)
        if result.error_code.val != 1:
            raise RuntimeError(f'FK code {result.error_code.val}')
        p = result.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z])

    def valid(self, q):
        request = GetStateValidity.Request()
        request.group_name = self.args.group
        request.robot_state = self.state(q)
        result = self.call('check_state_validity', request)
        return result.valid, [(c.contact_body_1, c.contact_body_2) for c in result.contacts]

    def seeds(self):
        return [self.home, *self.rng.uniform(self.lo+.001, self.hi-.001, (self.args.random_seeds, len(self.names)))]

    def position_only(self, target, seeds):
        best = None
        for seed in seeds:
            fit = least_squares(lambda q: self.chain.forward(dict(zip(self.names, q)))[:3, 3]-target,
                                np.clip(seed, self.lo+1e-6, self.hi-1e-6),
                                bounds=(self.lo, self.hi), max_nfev=100)
            error = float(np.linalg.norm(self.fk(fit.x)-target))
            if best is None or error < best[1]:
                best = (fit.x, error)
            if error < .002:
                break
        return best


def run(args):
    targets = json.loads(args.targets.read_text())
    rclpy.init()
    node = IK(args)
    try:
        counts = {name: Counter() for name in ['exact_6d', 'relaxed_10deg', 'relaxed_20deg', 'position_only']}
        rows = []
        home_valid, home_contacts = node.valid(node.home)
        for i, cell in enumerate(targets['targets']):
            T = node.target_in_base(reference_pose(cell, targets['orientation_xyzw']))
            row = {'index': i, 'world_xyz_m': [cell['x'], cell['y'], cell['z']]}
            seeds = node.seeds()
            pose_modes = [('exact_6d', [T])]
            if not args.exact_only:
                pose_modes += [('relaxed_10deg', rotated(T, 10)),
                               ('relaxed_20deg', rotated(T, 20))]
            for name, poses in pose_modes:
                solution = None
                calls = 0
                codes = Counter()
                for candidate in poses:
                    for seed in seeds:
                        solution, code = node.ik(candidate, seed, collision=False)
                        calls += 1
                        codes[str(code)] += 1
                        if solution is not None:
                            break
                    if solution is not None:
                        break
                outcome = {'kinematic_ik': solution is not None, 'calls': calls,
                           'codes': dict(codes)}
                if solution is not None:
                    valid, contacts = node.valid(solution)
                    on, on_code = node.ik(candidate, solution, collision=True)
                    outcome.update(collision_free=bool(valid), collision_on_ik=on is not None,
                                   collision_on_code=on_code, contacts=contacts, q=solution.tolist(),
                                   joint_margin_rad={joint: float(min(solution[k]-node.lo[k], node.hi[k]-solution[k]))
                                                     for k, joint in enumerate(node.names)})
                counts[name]['kinematic_ik' if solution is not None else 'NO_KINEMATIC_IK'] += 1
                if solution is not None:
                    counts[name]['collision_free' if valid else 'IK_FOUND_BUT_COLLISION'] += 1
                row[name] = outcome
            if not args.exact_only:
                q, error = node.position_only(T[:3, 3], seeds)
                success = error < .002
                valid, contacts = node.valid(q) if success else (False, [])
                row['position_only'] = {'kinematic_ik': success, 'error_m': error,
                                        'collision_free': bool(valid), 'contacts': contacts}
                counts['position_only']['kinematic_ik' if success else 'NO_POSITION_IK'] += 1
                if success:
                    counts['position_only']['collision_free' if valid else 'IK_FOUND_BUT_COLLISION'] += 1
            rows.append(row)
            if (i+1) % 9 == 0:
                print(f'{args.robot}: {i+1}/81', flush=True)
        summary = {name: dict(c) for name, c in counts.items()}
        report = {'robot': args.robot, 'model_path': str(args.urdf), 'base_link': args.base,
                  'tip_link': args.tip, 'group': args.group, 'joints': node.names,
                  'joint_limits_rad': {n: [node.lo[i], node.hi[i]] for i, n in enumerate(node.names)},
                  'home': node.home.tolist(), 'home_valid': bool(home_valid),
                  'home_contacts': home_contacts, 'base_xyz_m': args.base_xyz,
                  'base_yaw_deg': args.base_yaw_deg, 'timeout_s': args.timeout,
                  'random_seeds': args.random_seeds, 'target_file': str(args.targets),
                  'summary': summary, 'cells': rows}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, default=float))
        print(json.dumps(summary, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--robot', required=True)
    p.add_argument('--urdf', required=True, type=Path)
    p.add_argument('--base', required=True)
    p.add_argument('--tip', required=True)
    p.add_argument('--group', required=True)
    p.add_argument('--joints', required=True)
    p.add_argument('--home', required=True)
    p.add_argument('--fingers', default='')
    p.add_argument('--finger-positions', default='')
    p.add_argument('--base-yaw-deg', type=float, default=0.0)
    p.add_argument('--base-xyz', nargs=3, type=float, default=[0., 0., 0.])
    p.add_argument('--targets', type=Path, default=ROOT/'config/ik_tabletop_81.json')
    p.add_argument('--timeout', type=float, default=.1)
    p.add_argument('--random-seeds', type=int, default=4)
    p.add_argument('--exact-only', action='store_true', help='short controlled ablation run')
    p.add_argument('--output', required=True, type=Path)
    run(p.parse_args())
