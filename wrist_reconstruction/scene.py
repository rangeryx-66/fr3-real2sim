"""Normal room background; no object/robot dynamics or geometry changes."""
def normal_background(stage):
    from pxr import UsdGeom,UsdPhysics,Gf
    for name,pos,size in [('back',[0,3,1.5],[6,.08,4.5]),('front',[0,-3,1.5],[6,.08,4.5]),('left',[-3,0,1.5],[.08,6,4.5]),('right',[3,0,1.5],[.08,6,4.5]),('ceiling',[0,0,3.5],[6,6,.08])]:
        c=UsdGeom.Cube.Define(stage,'/World/capture_room/'+name);c.GetSizeAttr().Set(1.);x=UsdGeom.Xformable(c);x.AddTranslateOp().Set(Gf.Vec3d(*pos));x.AddScaleOp().Set(Gf.Vec3d(*size));c.GetDisplayColorAttr().Set([Gf.Vec3f(.62,.63,.65)]);UsdPhysics.CollisionAPI.Apply(c.GetPrim())
