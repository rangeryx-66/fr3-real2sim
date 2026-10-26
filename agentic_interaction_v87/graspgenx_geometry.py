"""Derive GX conditioning from the current official xArm native G1, not Dex1."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def derive(config_path,legal_open_width_m=.084):
    from xarm7_native_gripper import NativeGripperGeometry,FINGER_BODIES
    config_path=Path(config_path);config=json.loads(config_path.read_text())
    gripper=NativeGripperGeometry(config)
    base=np.linalg.inv(gripper.chain.root_to_link('xarm_gripper_base_link',{}))
    tcp=base@gripper.chain.root_to_link('tcp_link',{})
    B_GX=np.eye(4);B_GX[:3,:3]=tcp[:3,:3]@Rotation.from_euler('z',90,degrees=True).as_matrix()
    GX_TCP=np.linalg.inv(B_GX)@tcp
    def pad_box(width):
        values=gripper.values(gripper.angle(width));patches=[]
        for name in FINGER_BODIES:
            mesh=gripper.meshes[name]
            faces=np.max(abs(mesh.triangles[:,:,1]-gripper.inner[name]),axis=1)<1e-6
            points=mesh.triangles[faces].reshape(-1,3)
            if len(points)<3:raise ValueError('NATIVE_INNER_PAD_SURFACE_UNAVAILABLE')
            transform=np.linalg.inv(B_GX)@base@gripper.chain.root_to_link(name,values)
            points=points@transform[:3,:3].T+transform[:3,3]
            patches.append(np.array([points.min(0),points.max(0)]))
        low=np.maximum(patches[0][0],patches[1][0]);high=np.minimum(patches[0][1],patches[1][1])
        if np.any(high[1:]<=low[1:]):raise ValueError('NATIVE_PAD_SHARED_YZ_REGION_MISSING')
        measured=gripper.width(gripper.angle(width))
        return dict(extents=[measured,*(high[1:]-low[1:])],
            offset=[float(np.mean([p[:,0].mean()for p in patches])),*(.5*(high[1:]+low[1:]))],
            fingertip=max(p[1,2]for p in patches),pad_bounds=[p.tolist()for p in patches])
    closed=gripper.width(.85);mechanical_open=gripper.width(0.)
    if not closed<legal_open_width_m<=mechanical_open:raise ValueError('NATIVE_LEGAL_APERTURE_REQUIRED')
    # Match the unchanged planner's 84mm operational opening, rather than the
    # larger mechanical travel at drive=0. Mid is aperture, not midpoint angle.
    opened=float(legal_open_width_m);mid=.5*(opened+closed);a=pad_box(opened);b=pad_box(mid)
    if a['fingertip']<=0:raise ValueError('GX_APPROACH_PLUS_Z_CONVENTION_INVALID')
    params=dict(extents_open=a['extents'],offset_open=a['offset'],extents_mid=b['extents'],
        offset_mid=b['offset'],gripper_type=0,fingertip_depth=float(a['fingertip']))
    h=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    return dict(schema='xarm-native-g1-graspgenx-geometry-v1',T_GX_TCP=GX_TCP.tolist(),
        T_gripper_base_GX=B_GX.tolist(),T_gripper_base_TCP=tcp.tolist(),sweep_params=params,
        pad_diagnostics=dict(open=a,mid=b),provenance=dict(gripper='XARM7_NATIVE_G1',
            config_sha256=h(config_path),urdf_sha256=h(config['sim_urdf']),
            frame='GX +Z approach, +X closing; original TCP +Z approach, +Y closing',
            TCP_and_pad_center_are_distinct=True,
            native_aperture_range_m=[closed,mechanical_open],
            operational_opening_m=opened,operational_mid_opening_m=mid,
            dex1_or_r1_calibration_used=False,conditioning_is_not_collision_clearance=True,
            full_native_closing_sweep_still_required=True))


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--config',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);args=p.parse_args()
    if args.output.exists():raise ValueError('PRESERVE_EXISTING_GEOMETRY_EXPORT')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(derive(args.config),indent=2)+'\n')
