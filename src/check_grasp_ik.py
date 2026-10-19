"""Read-only MoveIt IK feasibility for common candidate JSON (no execution)."""

import argparse
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from r1a7_calibration import load_calibration, width_to_finger_q

ROOT = Path(__file__).resolve().parents[1]
JOINTS = [f'J{i}' for i in range(1, 8)]
FINGERS = ['dex1_Joint1_1', 'dex1_Joint2_1']


class IKChecker(Node):
    def __init__(self, seeds):
        super().__init__('grasp_candidate_ik_diagnostics')
        self.seeds = seeds
        self.open_width = float(load_calibration()['simulated_opening']['open_width_m'])
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.valid = self.create_client(GetStateValidity, '/check_state_validity')
        if not self.ik.wait_for_service(timeout_sec=15) or not self.valid.wait_for_service(timeout_sec=15):
            raise RuntimeError('MoveIt IK/state-validity services unavailable; launch src/r1a7_moveit.launch.py')
        model = ET.parse(ROOT / 'config/r1a7_dex1.urdf')
        mount = model.find("joint[@name='r1a7_world_mount']/origin")
        self.model_context = {'robot_model': str(ROOT / 'config/r1a7_dex1.urdf'),
                        'robot_model_sha256': hashlib.sha256((ROOT / 'config/r1a7_dex1.urdf').read_bytes()).hexdigest(),
                        'base_xyz_m': [float(v) for v in mount.get('xyz').split()],
                        'base_rpy_rad': [float(v) for v in mount.get('rpy').split()],
                        'ik_seeds': seeds}
        self.limits = {j.get('name'): (float(j.find('limit').get('lower')),
                                       float(j.find('limit').get('upper')))
                       for j in model.findall('joint') if j.get('name') in JOINTS}

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=4)
        if not future.done():
            raise TimeoutError(client.srv_name)
        return future.result()

    def check(self, transform, width_m=None):
        T = np.asarray(transform)
        rng = np.random.default_rng(20260929)  # identical multi-start seeds for every provider
        result = {'kinematic_ik': False, 'self_collision_free': None,
                  'j567_margin_rad': None, 'attempts': 0}
        for attempt in range(self.seeds):
            q = np.zeros(7) if attempt == 0 else np.array([
                rng.uniform(*self.limits[name]) for name in JOINTS])
            finger_q = width_to_finger_q(self.open_width if width_m is None else width_m)
            state = RobotState()
            state.joint_state = JointState(name=JOINTS + FINGERS,
                                           position=q.tolist() + [finger_q, finger_q])
            p = PoseStamped()
            p.header.frame_id = 'r1a7_world'
            p.pose.position.x, p.pose.position.y, p.pose.position.z = T[:3, 3]
            quat = Rotation.from_matrix(T[:3, :3]).as_quat()
            p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = quat
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'r1a7_arm'
            request.ik_request.ik_link_name = 'r1a7_tcp'
            request.ik_request.pose_stamped = p
            request.ik_request.robot_state = state
            request.ik_request.avoid_collisions = False
            request.ik_request.timeout.nanosec = 500_000_000
            response = self.call(self.ik, request)
            result['attempts'] = attempt + 1
            if response.error_code.val != 1:
                continue
            result['kinematic_ik'] = True
            vals = dict(zip(response.solution.joint_state.name,
                            response.solution.joint_state.position))
            margin = min(min(vals[name] - self.limits[name][0],
                             self.limits[name][1] - vals[name]) for name in JOINTS[-3:])
            validity = GetStateValidity.Request()
            validity.group_name = 'r1a7_arm'
            validity.robot_state = response.solution
            result['self_collision_free'] = bool(self.call(self.valid, validity).valid)
            result['j567_margin_rad'] = float(margin)
            if result['self_collision_free']:
                break
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seeds', type=int, default=8)
    args = parser.parse_args()
    data = json.loads(args.input.read_text())
    rclpy.init()
    node = IKChecker(args.seeds)
    data['ik_context'] = node.model_context
    started = time.monotonic()
    try:
        for model, candidates in data['candidates'].items():
            for c in candidates:
                scene_status = c['checks']['dex1_scene_collision']['status']
                if scene_status not in ('FREE', 'LOW_CLEARANCE'):
                    c['checks']['ik'] = {'status': 'SKIPPED_COLLISION_GATE'}
                    continue
                c['checks']['ik'] = node.check(c['T_B_TCP'], c['width_m'])
                c['checks']['ik']['scene_clearance_status'] = scene_status
            if 'raw_candidates' in data and model in data['raw_candidates']:
                by_rank = {c['rank']: c['checks'].get('ik') for c in candidates}
                for raw in data['raw_candidates'][model]:
                    if raw['rank'] in by_rank:
                        raw['checks']['ik'] = by_rank[raw['rank']]
    finally:
        node.destroy_node()
        rclpy.shutdown()
    data['ik_elapsed_s'] = time.monotonic() - started
    args.output.write_text(json.dumps(data, indent=2))


if __name__ == '__main__':
    main()
