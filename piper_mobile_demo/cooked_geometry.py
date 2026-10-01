"""Read-only PhysX collision export; never substitutes authored or synthetic hulls."""
import json
import numpy as np


def export_cooked(stage, output):
    from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdUtils, PhysicsSchemaTools, Usd
    from omni.physx import get_physx_cooking_interface
    cooking=get_physx_cooking_interface()
    stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    rows=[]
    debug=[{'path':str(p.GetPath()),'type':p.GetTypeName(),'schemas':p.GetAppliedSchemas(),'instance':p.IsInstance(),'proxy':p.IsInstanceProxy()} for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies())]
    output.with_name('stage_prims.json').write_text(json.dumps(debug,indent=2))
    import omni.physics.tensors as tensors
    try:
        simulation=tensors.create_simulation_view('numpy',stage_id=stage_id);simulation.set_subspace_roots('/')
    except Exception as error:
        simulation=None
        output.with_name('runtime_export_error.json').write_text(json.dumps({'error':repr(error),'stage_id':stage_id}))
    runtime={}
    for body in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not body.HasAPI(UsdPhysics.RigidBodyAPI):continue
        path=str(body.GetPath())
        try:
            view=simulation.create_rigid_body_view(path)
            runtime[path]={'contact_offsets':np.asarray(view.get_contact_offsets()).tolist(),'rest_offsets':np.asarray(view.get_rest_offsets()).tolist(),'materials':np.asarray(view.get_material_properties()).tolist()}
        except Exception as error:runtime[path]={'error':repr(error)}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
        path=str(prim.GetPath())
        entry={'path':path,'type':prim.GetTypeName(), 'world_transform':np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T.tolist(),
               'attributes':{a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('physics:','physxCollision:','physxConvexHullCollision:','physxConvexDecompositionCollision:'))},
               'offsets':{},'convexes':[]}
        api=PhysxSchema.PhysxCollisionAPI(prim)
        for name,attr in [('contactOffset',api.GetContactOffsetAttr()),('restOffset',api.GetRestOffsetAttr())]:
            entry['offsets'][name]={'schema_value':attr.Get(),'authored':attr.HasAuthoredValueOpinion()}
        parent=prim
        while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):parent=parent.GetParent()
        if parent:
            entry['rigid_body_path']=str(parent.GetPath())
            entry['rigid_body_world_transform']=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(parent)).T.tolist()
            entry['runtime_body_shapes']=runtime.get(str(parent.GetPath()),{})
        if prim.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(prim)
            entry['raw_points']=np.asarray(mesh.GetPointsAttr().Get()).tolist()
            entry['raw_counts']=list(mesh.GetFaceVertexCountsAttr().Get())
            entry['raw_indices']=list(mesh.GetFaceVertexIndicesAttr().Get())
            entry['approximation']=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            def callback(result, convexes):
                entry['cooking_result']=str(result)
                for convex in convexes:
                    vertices=[[v.x,v.y,v.z] for v in convex.vertices]
                    polygons=[list(convex.indices[p.index_base:p.index_base+p.num_vertices]) for p in convex.polygons]
                    entry['convexes'].append({'vertices':vertices,'polygons':polygons})
            try:
                cooking.request_convex_collision_representation(stage_id=stage_id,collision_prim_id=PhysicsSchemaTools.sdfPathToInt(prim.GetPath()),run_asynchronously=False,on_result=callback)
            except Exception as error:entry['export_error']=repr(error)
        elif prim.IsA(UsdGeom.Cube):entry['analytic']={'shape':'box','size':UsdGeom.Cube(prim).GetSizeAttr().Get()}
        elif prim.IsA(UsdGeom.Cylinder):
            c=UsdGeom.Cylinder(prim);entry['analytic']={'shape':'cylinder','radius':c.GetRadiusAttr().Get(),'height':c.GetHeightAttr().Get(),'axis':c.GetAxisAttr().Get()}
        rows.append(entry)
    output.write_text(json.dumps({'schema':'physx-cooked-export-v1','method':'request_convex_collision_representation synchronous; native convex vertices/polygons in mesh-local coordinates; world transform includes scale','shapes':rows},indent=2))
    return rows
