"""Placement/budget invariants; small metadata fixtures are not benchmark objects."""
import json
import sys
import tempfile
import types
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from interactive_twin.planning import (FIXED_BASE, make_deployment_template,
                                       plan_grasps, prepare_deployment)


def asset(root, asset_id, anchor):
    path = root / asset_id
    (path / 'urdf').mkdir(parents=True)
    (path / 'urdf' / f'{asset_id}.urdf').write_text('''<robot name="test">
      <link name="base"/><link name="door"/>
      <joint name="j" type="revolute"><parent link="base"/><child link="door"/>
      <origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="0" upper="1"/>
      </joint></robot>''')
    (path / 'manifest.json').write_text(json.dumps({
        'asset_id': asset_id, 'joint_name': 'j', 'scale_source_to_meters': 1.,
        'static_source_bounds': [[-.2, 0, -.2], [.2, 1., .2]],
        'interaction_geometry': {'selection': {
            'handle_link': 'door', 'axis_root': [0, 1, 0], 'outward_normal_root': [0, 0, 1],
            'sections': [{'fraction': 0, 'anchor_root_m': anchor}], 'dimensions_m': [.15, .02, .02]}}
    }))
    return path


def main():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        dev = asset(root, 'dev', [.1, .3, .2])
        test = asset(root, 'unseen', [-.2, 1.2, -.1])
        source = {'robot_base_pose': FIXED_BASE, 'asset_installation': {
            'x_m': .3, 'y_m': .2, 'yaw_deg': 15., 'fixture_height_m': .1}}
        template = make_deployment_template(dev, source)
        episode = {'episode_id': 'test_unseen_00', 'initialization_only': {'joint_position_rad': 0.},
                   'deployment': {'base_reposition_allowed': False, 'along_handle_offset_m': 0.,
                                  'yaw_perturbation_deg': 0., 'translation_in_initial_handle_frame_m': [0., 0., 0.]}}
        output = prepare_deployment(episode, test, template, root / 'deployment')
        a = np.asarray(output['initial_visual_handle_world']['anchor_world_m'])
        b = np.asarray(template['initial_visual_handle_world']['anchor_world_m'])
        assert np.allclose(a[:2], b[:2])
        assert a[2] > b[2] and output['placement']['fixture_height_m'] == 0.
        assert output['fixture_floor_applied']
        assert output['source_report']['robot_base_pose'] == FIXED_BASE
        assert np.allclose(output['initial_visual_handle_world']['outward_normal_world'],
                           template['initial_visual_handle_world']['outward_normal_world'])
        serialized = json.dumps(output['initial_visual_handle_world'])
        assert 'joint_position_rad' not in serialized and 'joint_name' not in serialized
        episode['initialization_only']['joint_position_rad'] = .02
        shifted = prepare_deployment(episode, test, template, root / 'deployment2')
        assert np.allclose(shifted['initial_visual_handle_world']['outward_normal_world'],
                           template['initial_visual_handle_world']['outward_normal_world'])

        calls = []
        class Model:
            home = np.zeros(6)
            pads = {n: np.array([[0., 0., 0.], [0., .02543, .01]]) for n in ('gripper_link1', 'gripper_link2')}
            def margin(self, q): return .2
            def ik(self, target, base, seed=None, starts=5): return np.asarray(target[:3, 3].tolist() + [0., 0., 0.])
            def poses(self, q, base, width=.1, finger_q=None):
                return {n: np.eye(4) for n in ('gripper_link1', 'gripper_link2', 'tcp_link')}
        class Scene:
            def check(self, poses, moving, allow_handle):
                assert not allow_handle
                return True, 'SAFE'
        def joint_plan(self, start, goal, base, iterations):
            assert iterations == 1500
            assert self.check(start, base)[0] and self.check(goal, base)[0]
            calls.append(iterations)
            return [start.tolist(), goal.tolist()]
        original = sys.modules.get('piper_mobile_demo.model')
        sys.modules['piper_mobile_demo.model'] = types.SimpleNamespace(Model=types.SimpleNamespace(joint_plan=joint_plan))
        try:
            result = plan_grasps(Model(), Scene(), np.eye(4), template['initial_visual_handle_world'], FIXED_BASE, budget=4)
            assert len(result['rows']) == 4 and len(result['trial_candidates']) == 4
            assert not result['real_closure_verified']
            assert calls == [1500] * 4
            assert all(r['minimum_joint_margin_rad'] > .05 for r in result['trial_candidates'])
            try:
                plan_grasps(Model(), Scene(), np.eye(4), template['initial_visual_handle_world'], FIXED_BASE, budget=13)
            except ValueError:
                pass
            else:
                raise AssertionError('candidate budget not enforced')
        finally:
            if original is None:
                sys.modules.pop('piper_mobile_demo.model', None)
            else:
                sys.modules['piper_mobile_demo.model'] = original
    print('interactive twin planning invariants passed')


if __name__ == '__main__':
    main()
