"""Offline robot-only cooked-pad/contact audit; never an executable pose input.

Reported contact points and their impulse-weighted centroids are not measured
pressure distributions. Object geometry and hidden dynamics are not consumed.
"""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]


def boundary_distance(points, polygon):
    """Unsigned distance to an ordered convex polygon boundary, in metres."""
    a = np.asarray(polygon); d = np.roll(a, -1, axis=0) - a
    r = np.asarray(points)[:, None, :] - a
    u = np.clip(np.sum(r * d, axis=2) / np.sum(d * d, axis=1), 0, 1)
    return np.linalg.norm(r - u[:, :, None] * d, axis=2).min(axis=1)


def audit(run, output):
    from xarm7_native_planning import PAD_FRAMES
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    run = Path(run); export_path = run / 'planning_current_scene.json'
    export = json.loads(export_path.read_text())
    index = json.loads((run / 'contact_evidence/index.json').read_text())
    packets = [np.load(run / 'contact_evidence' / row['file']) for row in index['files']]
    result = dict(robot_geometry_only=True, GT_object_geometry_used=False,
                  raw_points_are_not_pressure_areas=True,
                  native_export_sha256=hashlib.sha256(export_path.read_bytes()).hexdigest(), fingers=[])
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for j, (physical, logical) in enumerate(zip(['left_finger', 'right_finger'], ['gripper_link1', 'gripper_link2'])):
        shapes = [s for s in export['shapes'] if s['path'].startswith('/World/Piper/')
                  and s.get('rigid_body_path', '').split('/')[-1] in (physical, logical)]
        vertices = []
        for shape in shapes:
            X = np.linalg.inv(PAD_FRAMES[physical]) @ np.linalg.inv(shape['rigid_body_world_transform']) @ np.array(shape['world_transform'])
            for hull in shape.get('convexes', []):
                V = np.asarray(hull['vertices']); vertices.extend(V @ X[:3, :3].T + X[:3, 3])
        V = np.asarray(vertices); plane = V[:, 2].min()
        face = V[np.isclose(V[:, 2], plane, atol=1e-7, rtol=0), :2]
        polygon = face[ConvexHull(face).vertices]
        bounds = np.asarray(index['pad_bounds'][logical])
        points = np.concatenate([p['point_pad'][(p['finger'] == j) & (p['kind'] == 0)] for p in packets])
        distance = boundary_distance(points[:, :2], polygon)
        result['fingers'].append(dict(name=logical, inner_plane_m=float(plane),
            cooked_face_polygon_m=polygon.tolist(), cooked_planar_area_m2=float(ConvexHull(face).volume),
            official_pad_rectangle_area_m2=float(np.prod(bounds[1, :2] - bounds[0, :2])),
            native_reported_points=len(points),
            native_report_to_cooked_boundary_mm_quantiles=np.quantile(distance * 1000, [.05, .5, .95]).tolist(),
            interpretation='native reported manifold locations only; no inference of full physical pressure area'))
        ax = axes[j]; outline = np.vstack([polygon, polygon[0]]) * 1000
        ax.plot(outline[:, 0], outline[:, 1], label='Actual cooked inner face')
        rectangle = np.array([[bounds[x, 0], bounds[y, 1]] for x, y in [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]]) * 1000
        ax.plot(rectangle[:, 0], rectangle[:, 1], '--', label='Official pad bounding rectangle')
        ax.scatter(points[:, 0] * 1000, points[:, 1] * 1000, s=3, alpha=.4,
                   color='#c52626', zorder=3, label='Native positive-normal reports')
        ax.set(title=logical, xlabel='Lateral pad coordinate (mm)', ylabel='Longitudinal pad coordinate (mm)')
        ax.axis('equal'); ax.legend(fontsize=6); ax.grid(alpha=.2)
    fig.suptitle('Robot collision geometry and archived contact locations; not pressure maps')
    fig.tight_layout(); fig.savefig(str(output) + '.png', dpi=150); plt.close(fig)
    Path(str(output) + '.json').write_text(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('run', type=Path); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); print(json.dumps(audit(a.run, a.output), indent=2))
