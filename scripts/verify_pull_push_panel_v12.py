"""Offline current-state panel route using the SAME runtime panel planner.

Closed archived wrist measurements are rigidly propagated by the permitted
known hinge prior ONLY for this preflight. Live execution must measure a fresh
opened panel; these proposals are never loaded as executable contact targets.
"""
import argparse
import copy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import time
import numpy as np
from scipy.spatial.transform import Rotation
from hybrid_panel_preflight import native_model, patch_from_rgbd, FINGERS, PAD
from piper_mobile_demo.owned_scene import meshes
from interactive_twin_recovery.mobile import physical_scene_class
from cross_robot_adapter import RobotPinkIK
import native_g1_hybrid_contact as hybrid


def verify(episode, observation, regions, prior, output):
    started = time.perf_counter(); output.mkdir(parents=True, exist_ok=True)
    export = json.loads((episode/'cooked_initial.json').read_text())
    model, mapped = native_model(episode, export)
    allowed = json.loads((episode/'sensor_metadata.json').read_text())['allowed_handle_colliders']
    scene = physical_scene_class(Path(__file__).resolve().parents[1])(mapped, model, allowed)
    base = json.loads((episode/'attempts/00/selected_plan.json').read_text())['base']
    native = {}
    raw = {}
    for n, logical in zip(FINGERS, ('gripper_link1', 'gripper_link2')):
        e = next(e for e in export['shapes'] if e.get('rigid_body_path','').split('/')[-1] == n)
        mesh = meshes(e, False)[0]
        mesh.apply_transform(np.linalg.inv(e['rigid_body_world_transform']) @ e['world_transform'])
        native[n] = mesh
        mesh = mesh.copy(); mesh.apply_transform(np.linalg.inv(PAD[n])); raw[logical] = mesh
    model.native = SimpleNamespace(meshes=native)
    # Frozen native nonlinear aperture lookup, recovered from the archived
    # official URDF/inner mesh surfaces by native_model. No constant jaw proxy.
    P = model.poses(model.home, [0,0,0,0], angle=.85)
    ends = [(np.linalg.inv(P['xarm_gripper_base_link']) @ P[n] @
        np.r_[0, native[n].vertices[:,1].min() if n=='left_finger' else native[n].vertices[:,1].max(), 0, 1])[:3] for n in FINGERS]
    compact = float(ends[0][1] - ends[1][1])
    controller = SimpleNamespace(master=7, geometry=SimpleNamespace(width=lambda a: compact))
    robot = SimpleNamespace(get_joint_positions=lambda: np.r_[model.home, np.zeros(6)])
    prior = copy.deepcopy(prior)
    axis = np.asarray(prior['axis_world']); origin = np.asarray(prior['origin_world_m']); angle = np.deg2rad(10)
    D = np.eye(4); D[:3,:3] = Rotation.from_rotvec(axis*angle).as_matrix(); D[:3,3] = origin-D[:3,:3]@origin
    moving = D @ scene.moving_reference
    g = dict(a=SimpleNamespace(output=output), model=model, base=base, collision=hybrid.PanelScene(scene, raw),
        controller=controller, robot=robot, arm=np.arange(7), fingers=np.arange(7,13),
        recovery_state='UNLOADED_OBSERVATION', agent_observations=[dict(folder=str(observation))],
        solver=RobotPinkIK(model.config), moving=lambda: moving, actual=lambda: angle)
    records=[]
    for region in regions:
        measured = patch_from_rgbd(observation, region['rectangle'])
        propagated = dict(measured, points=measured['points']@D[:3,:3].T+D[:3,3],
            center=(D@np.r_[measured['center'],1])[:3], normal=D[:3,:3]@measured['normal'])
        np.savez_compressed(output/(region['name']+'_prior_propagated.npz'),
            observed_closed=measured['points'], propagated_10deg=propagated['points'])
        # A small finite orientation family, same native geometry and checks.
        for roll in (0, 90, 180, 270):
            cmd = dict(rectangle=region['rectangle'], roll_deg=roll, target_deg=2)
            with patch.object(hybrid, 'patch_from_rgbd', lambda *args: propagated), \
                 patch.object(hybrid, 'kinematic_prior', lambda state: prior):
                result = hybrid.panel_plan(g, cmd)
            records.append(dict(region=region['name'], roll_deg=roll, result=result))
            if result.get('valid'):
                report = dict(complete_route_valid=True, selected=records[-1], tests=records,
                    physical_episodes=0, observed_at_closed_not_10=True,
                    execution_reuses_archived_targets=False, wall_s=time.perf_counter()-started)
                (output/'FULL_PANEL_PREFLIGHT.json').write_text(json.dumps(report, indent=2, default=lambda x:x.tolist()))
                return report
    report = dict(complete_route_valid=False, tests=records, physical_episodes=0,
        observed_at_closed_not_10=True, wall_s=time.perf_counter()-started)
    (output/'FULL_PANEL_PREFLIGHT.json').write_text(json.dumps(report, indent=2, default=lambda x:x.tolist()))
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--observation',type=Path,required=True);p.add_argument('--regions',type=Path,required=True)
    p.add_argument('--prior',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=verify(a.episode,a.observation,json.loads(a.regions.read_text()),json.loads(a.prior.read_text()),a.output)
    print(json.dumps(dict(complete_route_valid=r['complete_route_valid'],wall_s=r['wall_s'],tests=len(r['tests']))))
