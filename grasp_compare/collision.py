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
    def _contact_bounds(self, width_m, finger_q=None):
        shifts=self.geometry.finger_translations_in_tcp(finger_q) if finger_q is not None else {}
        pads = [mesh.bounds + shifts.get(name,0) for name, mesh in self.geometry.meshes_in_tcp(
                self.geometry.open_width if finger_q is not None else width_m)
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
    def _raycast(self, width_m, finger_q=None):
        import open3d as o3d
        pieces = []
        for name, mesh in self.geometry.meshes_in_tcp(width_m, finger_q):
            tensor_mesh = o3d.t.geometry.TriangleMesh(
                o3d.core.Tensor(np.asarray(mesh.vertices, dtype=np.float32)),
                o3d.core.Tensor(np.asarray(mesh.faces, dtype=np.uint32)))
            ray = o3d.t.geometry.RaycastingScene()
            ray.add_triangles(tensor_mesh)
            pieces.append((name, ray, np.asarray(mesh.bounds), bool(mesh.is_watertight)))
        bounds = np.stack([np.min([p[2][0] for p in pieces], axis=0),
                           np.max([p[2][1] for p in pieces], axis=0)])
        return pieces, bounds

    def check(self, T_B_TCP, width_m=None, *, approach=True, contact_mask=None, finger_q=None):
        if self.obstacles_B is None:
            return {'status': 'UNKNOWN_NO_TARGET_MASK', 'reason': 'no target mask to separate contact from obstacles'}
        if width_m is not None and float(width_m) > self.geometry.open_width + 1e-6:
            return {'status': 'WIDTH_UNREACHABLE', 'requested_width_m': float(width_m),
                    'dex1_open_width_m': self.geometry.open_width}
        width = self.geometry.open_width if width_m is None else float(width_m)
        width = min(self.geometry.open_width, max(self.geometry.closed_width, width))
        shifts=self.geometry.finger_translations_in_tcp(finger_q) if finger_q is not None else {}
        pieces, bounds = self._raycast(round(self.geometry.open_width if finger_q is not None else width, 5))
        if shifts:
            pieces=[(name,ray,mesh_bounds+shifts[name],watertight) for name,ray,mesh_bounds,watertight in pieces]
            bounds=np.stack((np.min([p[2][0] for p in pieces],axis=0),np.max([p[2][1] for p in pieces],axis=0)))
        import open3d as o3d
        T_B_TCP = np.asarray(T_B_TCP)
        contact_low, contact_high = self._contact_bounds(round(width, 5), finger_q)
        target_TCP = transform_points(np.linalg.inv(T_B_TCP), self.target_B)
        allowed_contact = (np.all((target_TCP >= contact_low) & (target_TCP <= contact_high), axis=1)
                           if contact_mask is None else np.asarray(contact_mask, dtype=bool))
        # Target contact is permitted only on the official distal pads.
        # Finger bodies must still be checked against every target point.
        obstacles_B = np.concatenate((self.obstacles_B, self.target_B), axis=0)
        pad_obstacles_B = np.concatenate((self.obstacles_B, self.target_B[~allowed_contact]), axis=0)
        if len(obstacles_B) == 0:
            return {'status': 'UNKNOWN_NO_OBSTACLES', 'reason': 'scene has no points outside the Dex1 aperture'}
        low_clearance = None
        retreats = np.linspace(self.approach_m, 0.0,
                               max(2, int(np.ceil(self.approach_m / .01)) + 1)) if approach else np.array([0.])
        for retreat in retreats:
            pose = T_B_TCP.copy()
            pose[:3, 3] -= retreat * T_B_TCP[:3, 2]
            obstacles_TCP = transform_points(np.linalg.inv(pose), obstacles_B)
            nearby = obstacles_TCP[np.all((obstacles_TCP >= bounds[0] - self.clearance_m)
                                            & (obstacles_TCP <= bounds[1] + self.clearance_m), axis=1)]
            if not len(nearby):
                continue
            for name, ray, mesh_bounds, watertight in pieces:
                link_points = (transform_points(np.linalg.inv(pose), pad_obstacles_B)
                               if name in ('Link1_3', 'Link2_3') and retreat <= 1e-6 else nearby)
                subset = link_points[np.all((link_points >= mesh_bounds[0] - self.clearance_m)
                                       & (link_points <= mesh_bounds[1] + self.clearance_m), axis=1)]
                if not len(subset):
                    continue
                points = o3d.core.Tensor(np.asarray(subset-shifts.get(name,0), dtype=np.float32))
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

    def check_closure(self, T_B_TCP):
        """Sweep official finger bodies to the observed pad-contact aperture.

        This is a point-cloud prefilter, not a substitute for closed-state
        MoveIt and physical contact validation. Never exempt finger bodies.
        """
        if self.target_B is None:
            return {'status': 'UNKNOWN_NO_TARGET_MASK'}
        local = transform_points(np.linalg.inv(T_B_TCP), self.target_B)
        low, high = self._contact_bounds(round(self.geometry.open_width, 5))
        contact = np.all((local >= low) & (local <= high), axis=1)
        if not contact.any():
            return {'status': 'BAD_GRASP_GEOMETRY', 'reason': 'no target in official pad aperture'}
        span = float(np.ptp(local[contact, 1]))
        stop = max(self.geometry.closed_width,
                   self.geometry.open_width - (high[1] - low[1] - span))
        widths = np.linspace(self.geometry.open_width, stop,
                             max(2, int(np.ceil((self.geometry.open_width-stop)/.002))+1))
        for width in widths:
            result = self.check(T_B_TCP, float(width), approach=False, contact_mask=contact)
            if result['status'] != 'FREE':
                return {**result, 'stage': 'closure', 'width_m': float(width),
                        'estimated_contact_width_m': stop}
        return {'status': 'CLOSURE_UNVERIFIED', 'stage': 'closure',
                'actual_closure_verified': False,
                'reason': 'point-cloud span is an estimate; replay measured closure before accepting grasp',
                'estimated_contact_width_m': stop, 'width_samples_m': widths.tolist(),
                'allowed_contact_links': ['Link1_3', 'Link2_3']}

    def check_actual_closure(self, samples):
        """Validate measured asymmetric fingers and measured TCP at each step.

        Pad target-contact region is fixed at the initially open aperture;
        the target remains an obstacle for every metal body throughout closure.
        """
        from scipy.spatial.transform import Rotation
        rows=[]; contact=None
        for sample in samples:
            T=np.eye(4);T[:3,3]=sample['tcp']
            w,x,y,z=sample['tcp_quat'];T[:3,:3]=Rotation.from_quat([x,y,z,w]).as_matrix()
            if contact is None:
                low,high=self._contact_bounds(round(self.geometry.open_width,5))
                local=transform_points(np.linalg.inv(T),self.target_B)
                contact=np.all((local>=low)&(local<=high),axis=1)
            q=tuple(sample['finger_q'])
            result=self.check(T,approach=False,contact_mask=contact,finger_q=q)
            rows.append({'t':sample['t'],'finger_q':q,'actual_aperture_m':sample['finger_aperture_m'],**result})
        return {'status':'FREE' if all(r['status']=='FREE' for r in rows) and rows else 'REJECTED',
                'samples':rows,'endpoint_source':'measured joint positions; not point-cloud width'}
