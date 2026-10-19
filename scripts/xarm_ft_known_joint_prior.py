"""Known-kinematics input adapter, independent of object physical parameters.

A moving body's origin need not lie on its joint axis. Use native joint
anchors relative to the measured moving body; never read mass/resistance.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def native_joint_prior(asset_root,T_world_moving):
    from pxr import Usd,UsdGeom,UsdPhysics
    root=Path(asset_root);manifest=json.loads((root/'manifest.json').read_text())
    files=sorted((root/'usd').rglob(manifest['asset_id']+'.usda'))
    if len(files)!=1:raise ValueError('Known native articulation asset is ambiguous')
    stage=Usd.Stage.Open(str(files[0]));cache=UsdGeom.XformCache()
    matches=[p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName()==manifest['joint_name']]
    # Frozen sample()['T_moving_link'] actually reads scene['door_contacts'],
    # whose rigid body is the contact-bearing door link, not the abstract
    # revolute child. Record that association explicitly instead of guessing.
    measured_body=manifest['door_link']
    moving=[p for p in stage.Traverse() if p.GetName()==measured_body and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(matches)!=1 or len(moving)!=1:raise ValueError('Native known joint/moving-body mapping is ambiguous')
    joint=UsdPhysics.RevoluteJoint(matches[0]);parent=stage.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
    parentT=np.asarray(cache.GetLocalToWorldTransform(parent)).T
    movingT=np.asarray(cache.GetLocalToWorldTransform(moving[0])).T
    quat=joint.GetLocalRot0Attr().Get();local=np.eye(4)
    local[:3,:3]=Rotation.from_quat([*quat.GetImaginary(),quat.GetReal()]).as_matrix();local[:3,3]=joint.GetLocalPos0Attr().Get()
    moving_to_joint=np.linalg.inv(movingT)@parentT@local
    world=np.asarray(T_world_moving)@moving_to_joint
    axis=world[:3,:3]@np.eye(3)['XYZ'.index(joint.GetAxisAttr().Get())]
    return dict(joint_type='revolute',axis_world=axis.tolist(),origin_world_m=world[:3,3].tolist(),
        source='explicit KNOWN_MODEL native joint anchor relative to measured moving body; no dynamics read',
        angle_source='allowed actual articulation state V0',geometry_reference=str(root),
        native_joint_path=str(matches[0].GetPath()),native_asset_sha256=hashlib.sha256(files[0].read_bytes()).hexdigest(),
        measured_moving_body=measured_body,legacy_measurement_field='T_moving_link',T_moving_joint=moving_to_joint.tolist())
