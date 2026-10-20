"""Explicit renderer-only lights; frozen scene/physics owners remain untouched."""
import hashlib
import json
import math
from pathlib import Path


def load_config(path):
    data=json.loads(Path(path).read_text())
    for name in ('dome_intensity','key_intensity'):
        if not math.isfinite(data[name]) or data[name]<=0:raise ValueError('Positive finite light intensity required')
    if any(data.get(n) for n in ('camera_exposure_override','material_override','physics_override')):
        raise ValueError('Lighting configuration must not override camera, material or physics')
    return data


def lights(stage):
    from pxr import UsdLux
    rows=[]
    for prim in stage.Traverse():
        if prim.HasAPI(UsdLux.LightAPI):
            api=UsdLux.LightAPI(prim)
            rows.append(dict(path=str(prim.GetPath()),type=prim.GetTypeName(),
                intensity=api.GetIntensityAttr().Get(),exposure=api.GetExposureAttr().Get(),
                color=list(api.GetColorAttr().Get()),texture=str(prim.GetAttribute('inputs:texture:file').Get())))
    return rows


def physics_digest(stage):
    # Light additions cannot silently change authored dynamics or geometry.
    rows=[]
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            name=attr.GetName()
            if name.startswith(('physics:','physx','drive:','material:binding:physics')) or name in ('points','faceVertexCounts','faceVertexIndices'):
                rows.append((str(prim.GetPath()),name,str(attr.Get())))
    return hashlib.sha256(json.dumps(sorted(rows)).encode()).hexdigest()


def install(stage,config_path):
    from pxr import Gf,UsdGeom,UsdLux
    data=load_config(config_path);before=physics_digest(stage);previous=lights(stage)
    root=data['root'];UsdGeom.Xform.Define(stage,root)
    dome=UsdLux.DomeLight.Define(stage,root+'/ambient')
    key=UsdLux.DistantLight.Define(stage,root+'/key')
    for light,intensity in ((dome,data['dome_intensity']),(key,data['key_intensity'])):
        light.CreateIntensityAttr(float(intensity));light.CreateExposureAttr(float(data['exposure']))
        light.CreateColorAttr(Gf.Vec3f(*data['color_rgb']))
    transform=UsdGeom.Xformable(key.GetPrim())
    rotations=[op for op in transform.GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeRotateXYZ]
    op=rotations[0] if rotations else transform.AddRotateXYZOp()
    op.Set(Gf.Vec3f(*data['key_rotation_xyz_deg']))
    after=physics_digest(stage)
    if before!=after:raise RuntimeError('Lighting changed physics/mesh attributes')
    return dict(configuration=data,configuration_sha256=hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),
        lights_before=previous,lights_after=lights(stage),physics_mesh_sha256_before=before,
        physics_mesh_sha256_after=after,physics_mesh_unchanged=before==after,
        changes='Explicit white dome fill plus oblique distant key; no camera exposure/material/image manipulation')
