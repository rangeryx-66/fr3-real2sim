"""Explicit phase collision policy; baseline source and recorded grasps stay intact."""
import argparse
import copy
import json
import subprocess
import os
import time
import numpy as np
import rclpy
from arena_backend import ArenaBackend, ROOT, TARGET
from backend import TOUCH, Failure
from clutter_backend import transform
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningSceneComponents, AllowedCollisionEntry
import plant


class RepairBackend(ArenaBackend):
    dynamic_attachment = False
    strict_acm = True
    mesh_gate = False
    lift_segment_m = 0.

    def execute(self, trajectory, category):
        if self.stage != 'LIFT' or self.lift_segment_m <= 0:
            return super().execute(trajectory, category)
        destination = self.selected_targets['LIFT']
        self.telemetry['lift_attachment_segments'] = []
        while True:
            s = plant.state()
            H = transform(s['tcp'], s['tcp_quat'])
            O = transform(s['box'], s['box_quat'])
            remaining = destination[2, 3] - H[2, 3]
            if remaining <= .0005:
                break
            if len(self.telemetry['lift_attachment_segments']) >= 30:
                raise Failure('APPROACH_FAIL', 'segmented lift failed to make bounded progress')
            self.attach(H, O)
            next_pose = destination.copy()
            next_pose[2, 3] = min(destination[2, 3], H[2, 3] + self.lift_segment_m)
            segment = dict(T_TCP_target=(np.linalg.inv(H) @ O).tolist(),
                           target_pose=O.tolist(), tcp_pose=H.tolist(), goal=next_pose.tolist())
            self.telemetry['lift_attachment_segments'].append(segment)
            start = time.monotonic()
            try:
                plan = self.cartesian(self.measured(), next_pose)
                segment['planning_valid'] = True
            except Failure as failure:
                segment.update(planning_valid=False, failure=failure.category, detail=str(failure))
                raise
            finally:
                self.planning_seconds += time.monotonic() - start
            original_goal = self.selected_targets['LIFT']
            self.selected_targets['LIFT'] = next_pose
            try:
                super().execute(plan, category)
            finally:
                self.selected_targets['LIFT'] = original_goal
            # The controller's normal completion tolerance is retained. Do not
            # repeatedly command a sub-millimetre residual caused by tracking
            # error; the unchanged physical lift/hold test decides success.
            if next_pose[2, 3] == destination[2, 3]:
                break

    def candidate(self, g, data):
        selected_before = copy.deepcopy(self.selected_targets)
        telemetry_before = copy.deepcopy(self.telemetry)
        detail, plan = super().candidate(g, data)
        detail['old_obb_gate_passed'] = detail['hand_geometry']['min_pad_coverage'] >= .045
        if self.mesh_gate:
            metric = self.mesh_metrics[str(g['rank'])]
            detail['mesh_hand_geometry'] = metric
            if plan is not None and not metric['passed']:
                detail.update(status='INSUFFICIENT_PAD_OVERLAP',
                              detail='Mesh surface feasibility: ' + ', '.join(metric['reasons']))
                plan = None
                self.selected_targets = selected_before
                self.telemetry = telemetry_before
        return detail, plan

    def phase(self, name):
        if name == 'LIFT' and self.dynamic_attachment:
            plant.settle(.2)
            s = plant.state()
            H = transform(s['tcp'], s['tcp_quat'])
            O = transform(s['box'], s['box_quat'])
            old = self.telemetry.get('attachment_close_T_TCP_target')
            self.attach(H, O)
            self.telemetry['attachment_micro_T_TCP_target'] = (np.linalg.inv(H) @ O).tolist()
            self.telemetry['attachment_micro_state'] = s
            self.telemetry['attachment_close_T_TCP_target'] = old
        super().phase(name)

    def target_contact_policy(self, allowed=()):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.call('get_planning_scene', req).scene.allowed_collision_matrix
        for name in ['box', *allowed]:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                row = AllowedCollisionEntry()
                row.enabled = [False] * len(acm.entry_names)
                acm.entry_values.append(row)
        i = acm.entry_names.index('box')
        # Clear the entire target row, including any inherited default allowance.
        for j, name in enumerate(acm.entry_names):
            acm.entry_values[i].enabled[j] = name in allowed
            acm.entry_values[j].enabled[i] = name in allowed
        for j, name in enumerate(acm.default_entry_names):
            if name == 'box':
                acm.default_entry_values[j] = False
        update = ApplyPlanningScene.Request()
        update.scene.is_diff = True
        update.scene.allowed_collision_matrix = acm
        self.apply(update)

    def reset_scene(self):
        super().reset_scene()
        if self.strict_acm:
            self.target_contact_policy()

    def attach(self, H, O):
        # Candidate screening uses a hypothetical bilateral grasp. Execution gets
        # here only after the existing sustained bilateral physical-contact test.
        touches = list(TOUCH)
        if self.stage != 'CANDIDATE_CHECK':
            forces = plant.state()['forces']
            touches = [name for name, force in zip(TOUCH, forces) if force > .1]
            if len(touches) != 2:
                raise Failure('BAD_CONTACT', 'attachment requires actual bilateral finger contact')
        if self.strict_acm:
            self.target_contact_policy(touches)
        att = super().attach(H, O)
        assert list(att.touch_links) == touches
        if self.stage == 'CLOSE':
            self.telemetry['attachment_close_T_TCP_target'] = (np.linalg.inv(H) @ O).tolist()
        return att

    def perception(self, seed):
        source = ROOT / 'results/arena_complex40/inputs' / f'seed_{seed:04d}_grasps.json'
        if not source.exists():
            data, path = super().perception(seed)
        else:
            data, path = json.loads(source.read_text()), str(source)
        self.original_data = copy.deepcopy(data)
        if self.replay:
            old = json.loads((ROOT / 'results/arena_complex40' / f'B_seed_{seed:04d}.json').read_text())
            data['grasps'] = [g for g in data['grasps'] if g['rank'] == old['selected_rank']]
            assert len(data['grasps']) == 1
        if self.mesh_gate:
            start = time.monotonic()
            request = self.output / f'mesh_request_{seed:04d}.json'
            output = self.output / f'mesh_metrics_{seed:04d}.json'
            O = transform(self.initial['box'], self.initial['box_quat'])
            request.write_text(json.dumps(dict(target=TARGET, data=data, T_B_target=O.tolist())))
            env = {**os.environ, 'PYTHONPATH': '', 'LD_LIBRARY_PATH': '',
                   'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}
            subprocess.run(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',
                            str(ROOT / 'src/mesh_hand_geometry.py'), '--input', str(request),
                            '--output', str(output)], env=env, check=True, timeout=180)
            self.mesh_metrics = json.loads(output.read_text())
            self.telemetry['mesh_geometry_seconds'] = time.monotonic() - start
        return data, path

    def episode(self, seed, mode='B'):
        result = super().episode(seed, mode)
        result['repair_configuration'] = {'strict_open_hand_target_collision': self.strict_acm,
                                          'mesh_gate': self.mesh_gate, 'dynamic_attachment': self.dynamic_attachment,
                                          'lift_segment_m': self.lift_segment_m}
        result['old_obb_filtered_candidates'] = sum(not c.get('old_obb_gate_passed', True) for c in result['candidates'])
        result['mesh_filtered_candidates'] = sum(c['status'] == 'INSUFFICIENT_PAD_OVERLAP' and 'mesh_hand_geometry' in c for c in result['candidates'])
        if self.mesh_gate:
            result['geometry_adapter']['pad_metric'] = 'GT triangle closing sweeps over URDF pads; old OBB coverage recorded only'
        (self.output / f'B_seed_{seed:04d}.json').write_text(json.dumps(result, indent=2))
        return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--seeds', required=True)
    parser.add_argument('--replay', action='store_true')
    parser.add_argument('--dynamic-attachment', action='store_true')
    parser.add_argument('--legacy-acm-diagnostic', action='store_true')
    parser.add_argument('--mesh-gate', action='store_true')
    parser.add_argument('--lift-segment-m', type=float, default=0.)
    args = parser.parse_args()
    rclpy.init()
    node = RepairBackend(args.output, geometry_filter=not args.mesh_gate, replay=args.replay,
                         min_pad_coverage=.045, lift_gain_scale=2.)
    node.dynamic_attachment = args.dynamic_attachment
    node.strict_acm = not args.legacy_acm_diagnostic
    node.mesh_gate = args.mesh_gate
    node.lift_segment_m = args.lift_segment_m
    try:
        for seed in map(int, args.seeds.split(',')):
            if not rclpy.ok():
                raise RuntimeError('ROS context stopped')
            if (node.output / f'B_seed_{seed:04d}.json').exists():
                continue
            result = node.episode(seed)
            if result['category'] == 'SYSTEM_ERROR':
                raise RuntimeError(result['detail'])
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
