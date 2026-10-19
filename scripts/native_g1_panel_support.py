"""Observed patch/native contact correspondence; never moves measured points."""
import numpy as np
from piper_mobile_demo.owned_scene import meshes
from hybrid_panel_preflight import rays


def correspondence(scene,moving,patch):
    center=np.asarray(patch['center']);normal=np.asarray(patch['normal']);points=np.asarray(patch['points']);diagnostics=[];targets=[]
    for path,entry in scene.entries.items():
        if entry.get('rigid_body_path','').split('/')[-1]!=scene.moving_link:continue
        local=np.linalg.inv(np.asarray(entry['rigid_body_world_transform']))@np.asarray(entry['world_transform'])
        hits=[]
        for mesh in meshes(entry):
            mesh.apply_transform(np.asarray(moving)@local)
            hits.append(rays(mesh,[center+.01*normal],-normal)[0])
        distance=min(hits,default=float('inf'));matched=bool(np.isfinite(distance) and abs(distance-.01)<.003)
        diagnostics.append(dict(path=path,ray_m=float(distance) if np.isfinite(distance) else None,normal_offset_error_m=float(distance-.01) if np.isfinite(distance) else None,matched=matched))
        if matched:targets.append(path)
    return dict(target_paths=targets,measured_center_world_m=center.tolist(),normal_world=normal.tolist(),points=len(points),
                plane_residual_P95_m=float(np.quantile(abs((points-center)@normal),.95)),
                native_candidates=diagnostics,point_snapping=False,existing_match_tolerance_m=.003,
                reason=None if targets else 'OBSERVED_PATCH_HAS_NO_NATIVE_MOVING_CONTACT_SUPPORT')
