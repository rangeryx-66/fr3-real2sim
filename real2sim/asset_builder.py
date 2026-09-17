"""Build a self-contained Isaac Sim USD from visual/collision/inertial products."""
from __future__ import annotations
import argparse
import json
import shutil
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt


def _mesh(stage, path, source: trimesh.Trimesh):
    prim = UsdGeom.Mesh.Define(stage, path)
    prim.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in source.vertices])
    prim.CreateFaceVertexCountsAttr([len(f) for f in source.faces])
    prim.CreateFaceVertexIndicesAttr(np.asarray(source.faces, dtype=np.int32).reshape(-1).tolist())
    prim.CreateSubdivisionSchemeAttr("none")
    if source.vertex_normals is not None and len(source.vertex_normals) == len(source.vertices):
        prim.CreateNormalsAttr([Gf.Vec3f(*map(float, n)) for n in source.vertex_normals])
        prim.SetNormalsInterpolation("vertex")
    return prim


def _material(stage, texture_path: Path):
    material = UsdShade.Material.Define(stage, "/Object/Looks/Textured")
    shader = UsdShade.Shader.Define(stage, "/Object/Looks/Textured/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.45)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.)
    texture = UsdShade.Shader.Define(stage, "/Object/Looks/Textured/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_path.name))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    reader = UsdShade.Shader.Define(stage, "/Object/Looks/Textured/PrimvarReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def build(name: str, visual_path: Path, texture_path: Path, collision_manifest: Path,
          inertial_path: Path, output: Path, scale_m=1.0, T_object_mesh=None) -> dict:
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    visual_source = Path(visual_path); texture_source = Path(texture_path)
    shutil.copy2(visual_source, output / visual_source.name)
    shutil.copy2(texture_source, output / texture_source.name)
    collision = json.loads(Path(collision_manifest).read_text())
    copied_parts=[]
    for item in collision["parts"]:
        source=Path(item); target=output/source.name; shutil.copy2(source,target);copied_parts.append(target)
    inertial=json.loads(Path(inertial_path).read_text())
    mesh=trimesh.load(visual_source,force="mesh",process=False)
    if T_object_mesh is not None:mesh.apply_transform(np.asarray(T_object_mesh))
    mesh.apply_scale(scale_m)
    stage=Usd.Stage.CreateNew(str(output/f"{name}.usda"));stage.SetMetadata("metersPerUnit",1.0);stage.SetMetadata("upAxis","Z")
    root=UsdGeom.Xform.Define(stage,"/Object");root.GetPrim().SetMetadata("kind","component")
    visual=_mesh(stage,"/Object/Visual",mesh);visual.CreatePurposeAttr("render")
    material=_material(stage,output/texture_source.name);UsdShade.MaterialBindingAPI.Apply(visual.GetPrim()).Bind(material)
    # BundleSDF OBJ UVs are indexed per face corner. Preserve them when trimesh exposes UVs.
    uv=getattr(mesh.visual,"uv",None)
    if uv is not None and len(uv)==len(mesh.vertices):
        pv=UsdGeom.PrimvarsAPI(visual).CreatePrimvar("st",Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex)
        pv.Set([Gf.Vec2f(float(x),float(1-y)) for x,y in uv])
    collision_paths=[]
    for index,path in enumerate(copied_parts):
        part=trimesh.load(path,force="mesh",process=False);part.apply_scale(scale_m)
        geom=_mesh(stage,f"/Object/Collisions/part_{index:03d}",part)
        geom.CreatePurposeAttr("guide");UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")
        collision_paths.append(str(geom.GetPath()))
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mass_api=UsdPhysics.MassAPI.Apply(root.GetPrim());mass_api.CreateMassAttr(float(inertial["mass"]))
    com=np.asarray(inertial["center_of_mass"],float)*scale_m
    inertia=np.asarray(inertial["inertia_matrix"],float)*(scale_m**2)
    values,vectors=np.linalg.eigh((inertia+inertia.T)/2)
    if np.linalg.det(vectors)<0:vectors[:,0]*=-1
    quat=Rotation.from_matrix(vectors).as_quat();quat=np.roll(quat,1)
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,com)))
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,values)))
    mass_api.CreatePrincipalAxesAttr(Gf.Quatf(float(quat[0]),Gf.Vec3f(*map(float,quat[1:]))))
    stage.GetRootLayer().customLayerData={
        "schema":"fr3_real2sim_asset/v1","objectFrame":"Object","scaleMeters":float(scale_m),
        "visualMesh":visual_source.name,"collisionMethod":collision["method"],
        "inertialEstimator":inertial.get("estimator","unknown"),
    }
    stage.SetDefaultPrim(root.GetPrim());stage.GetRootLayer().Save()
    metadata={"schema":"fr3_real2sim_asset/v1","name":name,"usd":str(output/f"{name}.usda"),
              "visual_mesh":str(output/visual_source.name),"texture":str(output/texture_source.name),
              "collision_parts":[str(x) for x in copied_parts],"object_frame":"Object","scale_m":scale_m,
              "mass_kg":inertial["mass"],"center_of_mass_m":com.tolist(),"inertia_kg_m2":inertia.tolist()}
    (output/"metadata.json").write_text(json.dumps(metadata,indent=2));return metadata


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--name",required=True);p.add_argument("--visual",type=Path,required=True);p.add_argument("--texture",type=Path,required=True);p.add_argument("--collision",type=Path,required=True);p.add_argument("--inertial",type=Path,required=True);p.add_argument("--output",type=Path,required=True);p.add_argument("--scale",type=float,default=1.)
    a=p.parse_args();print(json.dumps(build(a.name,a.visual,a.texture,a.collision,a.inertial,a.output,a.scale),indent=2))
