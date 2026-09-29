"""Scene collision checks using the official Dex1-1 URDF collision STLs."""

from functools import lru_cache
import numpy as np

from .dex1_geometry import Dex1Geometry, transform_points


class Dex1SceneCollision:
    def __init__(self, scene, geometry=None, clearance_m=0.003, approach_m=0.08):
        self.scene = scene
        self.geometry = geometry or Dex1Geometry()
        self.clearance_m = float(clearance_m)
        self.approach_m = float(approach_m)
        if self.clearance_m < 0 or self.approach_m < 0:
            raise ValueError('clearance and approach distance must be nonnegative')
        self.obstacles_B = (scene.points_B[~scene.target_mask] if scene.target_mask is not None
                            else None)
        self.target_B = (scene.points_B[scene.target_mask] if scene.target_mask is not None
                         else None)

    @lru_cache(maxsize=12)
    def _contact_bounds(self, width_m):
        pads = [mesh.bounds for name, mesh in self.geometry.meshes_in_tcp(width_m)
                if name in ('Link1_3', 'Link2_3')]
        if len(pads) != 2:
            raise RuntimeError('official Dex1 terminal-pad meshes unavailable')
        negative, positive = sorted(pads, key=lambda b: b[:, 1].mean())
        low = np.array([max(negative[0, 0], positive[0, 0]),
                        negative[1, 1], max(negative[0, 2], positive[0, 2])])
        high = np.array([min(negative[1, 0], positive[1, 0]),
                         positive[0, 1], min(negative[1, 2], positive[1, 2])])
        if np.any(high <= low):
            raise RuntimeError('Dex1 finger meshes do not define an aperture')
        tolerance = np.array([.003, 0.0, .003])
        return low - tolerance, high + tolerance

    @lru_cache(maxsize=12)
    def _raycast(self, width_m):
        import open3d as o3d
        pieces = []
        for name, mesh in self.geometry.meshes_in_tcp(width_m):
            tensor_mesh = o3d.t.geometry.TriangleMesh(
                o3d.core.Tensor(np.asarray(mesh.vertices, dtype=np.float32)),
                o3d.core.Tensor(np.asarray(mesh.faces, dtype=np.uint32)))
            ray = o3d.t.geometry.RaycastingScene()
            ray.add_triangles(tensor_mesh)
            pieces.append((name, ray, np.asarray(mesh.bounds), bool(mesh.is_watertight)))
        bounds = np.stack([np.min([p[2][0] for p in pieces], axis=0),
                           np.max([p[2][1] for p in pieces], axis=0)])
        return pieces, bounds

    def check(self, T_B_TCP, width_m=None):
        if self.obstacles_B is None:
            return {'status': 'UNKNOWN_NO_TARGET_MASK', 'reason': 'no target mask to separate contact from obstacles'}
        if width_m is not None and float(width_m) > self.geometry.open_width + 1e-6:
            return {'status': 'WIDTH_UNREACHABLE', 'requested_width_m': float(width_m),
                    'dex1_open_width_m': self.geometry.open_width}
        width = self.geometry.open_width if width_m is None else float(width_m)
        width = min(self.geometry.open_width, max(self.geometry.closed_width, width))
        pieces, bounds = self._raycast(round(width, 5))
        import open3d as o3d
        T_B_TCP = np.asarray(T_B_TCP)
        contact_low, contact_high = self._contact_bounds(round(width, 5))
        target_TCP = transform_points(np.linalg.inv(T_B_TCP), self.target_B)
        allowed_contact = np.all((target_TCP >= contact_low) & (target_TCP <= contact_high), axis=1)
        obstacles_B = np.concatenate((self.obstacles_B, self.target_B[~allowed_contact]), axis=0)
        if len(obstacles_B) == 0:
            return {'status': 'UNKNOWN_NO_OBSTACLES', 'reason': 'scene has no points outside the Dex1 aperture'}
        low_clearance = None
        retreats = np.linspace(self.approach_m, 0.0,
                               max(2, int(np.ceil(self.approach_m / .01)) + 1))
        for retreat in retreats:
            pose = T_B_TCP.copy()
            pose[:3, 3] -= retreat * T_B_TCP[:3, 2]
            obstacles_TCP = transform_points(np.linalg.inv(pose), obstacles_B)
            nearby = obstacles_TCP[np.all((obstacles_TCP >= bounds[0] - self.clearance_m)
                                            & (obstacles_TCP <= bounds[1] + self.clearance_m), axis=1)]
            if not len(nearby):
                continue
            for name, ray, mesh_bounds, watertight in pieces:
                subset = nearby[np.all((nearby >= mesh_bounds[0] - self.clearance_m)
                                       & (nearby <= mesh_bounds[1] + self.clearance_m), axis=1)]
                if not len(subset):
                    continue
                points = o3d.core.Tensor(np.asarray(subset, dtype=np.float32))
                distance = (ray.compute_signed_distance(points).numpy() if watertight
                            else ray.compute_distance(points).numpy())
                closest = float(np.min(distance))
                detail = {'retreat_m': float(retreat), 'mesh_link': name,
                            'min_signed_distance_m' if watertight else 'min_surface_distance_m': closest,
                            'obstacle_points_checked': len(subset),
                            'allowed_target_contact_points': int(allowed_contact.sum()),
                            'mesh_watertight': watertight}
                # A positive 2.7 mm surface distance is not mesh penetration.
                # In particular, base_link is not watertight, so its unsigned
                # distance cannot establish whether a point is inside it.
                if watertight and closest <= 0:
                    return {'status': 'COLLISION', **detail}
                if closest <= self.clearance_m and (low_clearance is None or
                        closest < low_clearance.get('min_distance_m', float('inf'))):
                    low_clearance = {'status': 'LOW_CLEARANCE', **detail,
                                     'min_distance_m': closest,
                                     'required_clearance_m': self.clearance_m}
        if low_clearance is not None:
            return low_clearance
        return {'status': 'FREE',
                'non_watertight_collision_links': [n for n, _, _, ok in pieces if not ok],
                'obstacle_points_total': len(obstacles_B),
                'allowed_target_contact_points': int(allowed_contact.sum()),
                'path_samples_m': retreats.tolist()}
