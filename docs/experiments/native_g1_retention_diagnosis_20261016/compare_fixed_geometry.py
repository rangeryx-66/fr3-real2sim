"""Offline only: fixed recorded poses, unchanged scorer; never creates an actor.

Run with repository root and scripts on PYTHONPATH in the numerical environment.
All private mesh exports remain under --output; compact scores may be published.
"""
import argparse, hashlib, inspect, json, types
from pathlib import Path
import numpy as np
import trimesh
import xarm7_native_planning as native
import native_g1_multiview_input as multi
import plan_shared_door_skill as shared
from evaluate_native_g1_multiview_predictions import first
from piper_mobile_demo.owned_scene import meshes


def main(a):
    a.output.mkdir(parents=True, exist_ok=True)
    read = lambda p: json.loads(p.read_text())
    job = read(a.failed / 'job.json')
    model = native.NativeModel(None, job['asset_root'], read(Path(job['source']) / 'report.json'))
    config = read(Path(job['robust_uncertainty_config']))
    base = read(a.failed / 'episode/visual_plan.json')['base']
    link = model.manifest['moving_link']; joint = model.manifest['joint_name']
    J = model.asset_T @ model.asset.root_to_link(link, {joint: 0.})
    origin = J[:3, 3]; axis = J[:3, :3] @ model.asset.joints[link].axis
    # Exact native adapter's closure aperture grid; no candidate pose search.
    code = inspect.getsource(shared.closing_quality).replace('np.linspace(.1,0.,41)', 'np.linspace(.084,.001641,41)')
    ns = dict(shared.__dict__); exec(code, ns)
    common_ns = dict(native.__dict__, ray_surface=multi.fast_ray)
    exec(inspect.getsource(native.common_quality).replace(' from handle_contact_quality import ray_surface\n', ''), common_ns)
    sources = {str(Path(m.__file__).relative_to(Path.cwd())): hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
               for m in [native, multi, shared]}
    poses = {}
    for label, root in [('success', a.success), ('failed', a.failed)]:
        plan = read(root / 'episode/visual_plan.json'); corr = read(root / 'episode/visual_correction.json')
        poses[label] = dict(selected=plan['T'], refined=corr['T_after'], before=corr['T_before'],
                            D_visual=corr['D_visual'], section=plan['section_fraction'])
    results = []
    for geom_label, root in [('incremental', a.failed), ('first_seed', a.success)]:
        folder = root / 'episode/agent_geometry'
        surfaces = multi.rank_surfaces(folder); sc = multi.closing_scene(model, folder)
        candidates = []
        for label in poses:
            T = np.asarray(poses[label]['selected'])
            tangent = np.cross(axis, T[:3, 3] - origin); tangent /= np.linalg.norm(tangent)
            quality = ns['closing_quality'](model, sc, T, base, .084)
            quality['patch'] = common_ns['common_quality'](model, surfaces, T, base, tangent)
            quality['support_score'] = quality['patch']['score'] / max(1., 1+quality['root_contact_count']) if quality['non_handle_closing_collision'] is None else 0.
            quality['rank'] += quality['support_score']
            g = dict(T=T, contact=quality, section=poses[label]['section'], insertion_m=0., roll=0., pitch=0., fixed_pose_label=label)
            candidates.append(g)
        ranked = multi.rescore(candidates, model, surfaces, base, origin, axis, config, a.output / geom_label)
        observed = trimesh.load(folder / 'observed_patches.ply', process=False)
        for g in ranked:
            label = g['fixed_pose_label']; predictions = {}
            for stage in ['selected', 'refined']:
                T = np.asarray(poses[label][stage])
                pred = dict(observed_only=first(model, T, base, [observed]))
                for i, scale in enumerate([.75, 1., 1.25]):
                    pred[str(scale)] = first(model, T, base, [observed, trimesh.load(folder / f'uncertain_rear_{i}.ply', process=False)])
                predictions[stage] = pred
            results.append(dict(geometry=geom_label, pose=label, contact=g['contact'], predictions=predictions))
    cooked = read(a.failed / 'episode/cooked_initial.json')
    from handle_contact_quality import handle_surfaces
    from revolute10_fixed import authored_handle_colliders
    selection = job['grasp_geometry_candidates'][0]
    gt, gt_native = handle_surfaces(model, cooked, authored_handle_colliders(cooked, selection), selection)
    gt_predictions = {}
    for label in poses:
        gt_predictions[label] = {stage: dict(visual=first(model, np.asarray(poses[label][stage]), base, [gt]),
                                            cooked=first(model, np.asarray(poses[label][stage]), base, gt_native, True))
                                 for stage in ['selected', 'refined']}
    # Private calibration bundle for local measured-contact reconstruction.
    bundle = dict(pads={n:v.tolist() for n,v in model.pads.items()}, PAD_FRAMES={n:v.tolist() for n,v in native.PAD_FRAMES.items()},
                  width_angle_grid_rad=model.native.grid.tolist(), width_grid_m=model.native.width_grid.tolist(),
                  hinge_origin=origin.tolist(), hinge_axis=axis.tolist())
    (a.output / 'PRIVATE_GEOMETRY_CALIBRATION.json').write_text(json.dumps(bundle))
    gt.export(a.output / 'PRIVATE_GT_HANDLE_WORLD.ply')
    for i,m in enumerate(gt_native): m.export(a.output / f'PRIVATE_GT_COOKED_HANDLE_WORLD_{i}.ply')
    result = dict(offline_only=True, pose_search=False, config=config, poses=poses, rows=results,
                  GT_separate_evaluation_only=gt_predictions, source_hashes=sources,
                  attribution='2x2 crossed pose/representation scoring is diagnostic; physical outcomes are confounded, not causal intervention')
    (a.output / 'FIXED_POSE_SCORES.json').write_text(json.dumps(result, indent=2))
    print('FIXED_POSE_COMPARISON_COMPLETE', flush=True)


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--success',type=Path,required=True);p.add_argument('--failed',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    main(p.parse_args())
