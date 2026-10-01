"""Native collider ownership; no contact-position classification.

The official distal surface is an independent four-triangle DAE geometry.
A tetrahedral/pyramidal interior partition gives that surface its own collider.
The partition changes neither the exterior nor the union of the original hull.
It does NOT claim that the mathematical interior cut is a real pad thickness.
All other exposed surfaces are forbidden finger body, regardless of material.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull

FINGERS=('gripper_link1','gripper_link2')


def hull(points):
    h=ConvexHull(points)
    vertices=np.asarray(points)[h.vertices]
    h=ConvexHull(vertices)
    faces=h.simplices.copy()
    for i,f in enumerate(faces):
        a,b,c=vertices[f]
        if np.dot(np.cross(b-a,c-a),h.equations[i,:3])<0:faces[i]=f[::-1]
    return vertices,faces


def clip(vertices,normal,offset):
    """Exact convex halfspace partition in double precision, never an inset."""
    h=ConvexHull(vertices);edges=set()
    for f in h.simplices:
        for a,b in [(f[0],f[1]),(f[1],f[2]),(f[2],f[0])]:edges.add(tuple(sorted((a,b))))
    d=vertices@normal+offset;points=list(vertices[d<=0])
    for a,b in edges:
        if (d[a]<0<d[b]) or (d[b]<0<d[a]):points.append(vertices[a]+d[a]/(d[a]-d[b])*(vertices[b]-vertices[a]))
    if len(points)<4:return None
    try:return hull(np.asarray(points))[0]
    except Exception:return None


def official_pad_surface(path):
    r=ET.parse(path).getroot();ns={'c':'http://www.collada.org/2005/11/COLLADASchema'};candidates=[]
    for geometry in r.findall('.//c:geometry',ns):
        for triangles in geometry.findall('.//c:triangles',ns):
            if int(triangles.get('count'))!=4:continue
            vertex=next(x for x in triangles.findall('c:input',ns) if x.get('semantic')=='VERTEX')
            vertices=r.find('.//c:vertices[@id="'+vertex.get('source')[1:]+'"]',ns)
            source=vertices.find('c:input',ns).get('source')[1:]
            floats=r.find('.//c:source[@id="'+source+'"]/c:float_array',ns)
            points=np.array(list(map(float,floats.text.split()))).reshape(-1,3)
            stride=max(int(x.get('offset')) for x in triangles.findall('c:input',ns))+1
            indices=np.array(list(map(int,triangles.find('c:p',ns).text.split()))).reshape(-1,stride)[:,int(vertex.get('offset'))].reshape(-1,3)
            candidates.append((points,indices,geometry.get('id'),triangles.get('material')))
    if len(candidates)!=1:raise RuntimeError('official distal surface ownership is ambiguous')
    return candidates[0]


def build_manifest(root,output,source_cooked_export):
    import trimesh,hashlib
    source_export=json.loads(source_cooked_export.read_text());source_sha=hashlib.sha256(source_cooked_export.read_bytes()).hexdigest()
    result={'native_source_export_sha256':source_sha,'schema':'piper-contact-ownership-v1','runtime_classification':'native collider path ONLY; unknown collider is forbidden','partition_rule':'official four-triangle distal DAE surface forms base of interior pyramid; apex at source hull vertex centroid; all complement pieces forbidden','physical_pad_thickness_claim':False,'fingers':{}}
    for name in FINGERS:
        stl=root/'third_party/agilex_piper/description/meshes'/f'{name}.stl';dae=stl.parent/'dae'/f'{name}.dae';mesh=trimesh.load(stl,force='mesh');points,faces,gid,material=official_pad_surface(dae)
        # DAE has lower coordinate precision than STL. Resolve its vertices to
        # the official collision STL without moving any official vertex.
        ids=np.argmin(np.linalg.norm(points[:,None,:]-mesh.vertices[None,:,:],axis=2),axis=1);delta=np.linalg.norm(points-mesh.vertices[ids],axis=1)
        if delta.max()>1e-7:raise RuntimeError('DAE/STL distal surface mismatch')
        points=mesh.vertices[ids];apex=mesh.convex_hull.vertices.mean(0);depth=float(apex[2])
        pv,pf=hull(np.vstack([points,apex]));planes=ConvexHull(pv).equations
        remaining=mesh.convex_hull.vertices.copy();pieces=[]
        for plane in planes:
            exterior=clip(remaining,-plane[:3],-plane[3])
            if exterior is not None and abs(trimesh.Trimesh(*hull(exterior),process=False).volume)>1e-18:pieces.append(('metal',*hull(exterior)))
            remaining=clip(remaining,plane[:3],plane[3])
            if remaining is None:raise RuntimeError('pad partition unexpectedly empty')
        pieces.append(('pad',*hull(remaining)))
        tetrahedra=[]
        for owner,vertices,triangles in pieces:
            if owner=='pad':
                # One convex pad avoids artificial internal pad contact faces.
                tetrahedra.append((owner,vertices,triangles));continue
            center=vertices.mean(0)
            for triangle in triangles:
                v,f=hull(np.vstack([center,vertices[triangle]]));tetrahedra.append((owner,v,f))
        pieces=tetrahedra
        source_shapes=[e for e in source_export['shapes'] if e.get('rigid_body_path','').split('/')[-1]==name and e.get('raw_points')]
        if len(source_shapes)!=1:raise RuntimeError('native source export must have one original collider per finger')
        native_raw=np.asarray(source_shapes[0]['raw_points']);distances=np.linalg.norm(native_raw[:,None,:]-mesh.vertices[None,:,:],axis=2)
        # USD importer duplicates vertices at normal/material seams. Compare
        # both geometric point sets, not unwelded versus welded vertex counts.
        if distances.min(axis=1).max()>1e-7 or distances.min(axis=0).max()>1e-7:raise RuntimeError('native source collider does not match official STL vertex geometry')
        offsets=np.asarray(source_shapes[0]['runtime_body_shapes']['contact_offsets']).reshape(-1);rests=np.asarray(source_shapes[0]['runtime_body_shapes']['rest_offsets']).reshape(-1)
        if len(offsets)!=1 or len(rests)!=1:raise RuntimeError('native source offset identity is ambiguous')
        volume=sum(abs(trimesh.Trimesh(v,f,process=False).volume) for _,v,f in pieces);original=abs(mesh.convex_hull.volume)
        if abs(volume-original)>1e-12:raise RuntimeError('partition does not conserve hull volume')
        original_pad_faces=[]
        pad_vertex_ids=set(ids.tolist())
        for i,f in enumerate(mesh.faces):
            if set(f.tolist())<=pad_vertex_ids:original_pad_faces.append(i)
        if len(original_pad_faces)!=4:raise RuntimeError('official STL surface ownership not exactly four faces')
        result['fingers'][name]={'source_contact_offset_m':float(offsets[0]),'source_rest_offset_m':float(rests[0]),'tessellation':'metal centroid/facet tetrahedra; one convex pad; exterior and union conserved','stl_sha256':hashlib.sha256(stl.read_bytes()).hexdigest(),'dae_sha256':hashlib.sha256(dae.read_bytes()).hexdigest(),'official_surface_geometry_id':gid,'official_surface_material':material,'dae_to_stl_max_vertex_error_m':float(delta.max()),'official_pad_face_ids':original_pad_faces,'pad_vertices':points.tolist(),'partition_apex_z_m':depth,'source_hull_volume_m3':original,'partition_volume_m3':float(volume),'source_bounds':mesh.bounds.tolist(),'pieces':[{'owner':owner,'vertices':v.tolist(),'faces':f.tolist()} for owner,v,f in pieces]}
    output.write_text(json.dumps(result,indent=2));return result


def install_owned_colliders(stage,manifest):
    """Replace whole-finger hull by union-preserving owned colliders in memory."""
    from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Sdf,Gf,Vt
    import hashlib
    installed={}
    # Material and rigid-body mass/inertia stay inherited from the original.
    for name,spec in manifest['fingers'].items():
        official=Path(__file__).resolve().parents[1]/'third_party/agilex_piper/description/meshes'/f'{name}.stl'
        if hashlib.sha256(official.read_bytes()).hexdigest()!=spec['stl_sha256']:raise RuntimeError('official geometry changed since ownership manifest generation')
        bodies=[p for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()) if p.GetName()==name and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(bodies)!=1:raise RuntimeError('finger rigid body missing or ambiguous')
        body=bodies[0];bodypath=str(body.GetPath())
        original=[p for p in Usd.PrimRange(body,Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI)]
        if len(original)!=1:raise RuntimeError('unexpected source collider count')
        mass_api=UsdPhysics.MassAPI(body);mass_before={a.GetName():str(a.Get()) for a in body.GetAttributes() if a.GetName().startswith('physics:') and any(k in a.GetName().lower() for k in ['mass','inertia','centerofmass','principalaxes','density'])}
        sourcepath=str(original[0].GetPath());sourceT=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(original[0])).T;bodyT=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(body)).T;relative=np.linalg.inv(bodyT)@sourceT
        # Flatten only this finger's instance ancestry before replacing shape.
        p=stage.GetPrimAtPath(sourcepath)
        while p and p!=body:
            if p.IsInstanceProxy():
                ancestor=p.GetParent()
                while ancestor.IsInstanceProxy():ancestor=ancestor.GetParent()
                ancestor.SetInstanceable(False);p=stage.GetPrimAtPath(sourcepath)
            else:p=p.GetParent()
        p=stage.GetPrimAtPath(sourcepath)
        from pxr import UsdShade
        material=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial('physics')[0]
        # Remove old collider API ONLY after an equivalent complete replacement
        # has been authored; its visual/source mesh remains untouched.
        paths=[]
        for i,piece in enumerate(spec['pieces']):
            path=bodypath+'/contact_owned/'+piece['owner']+'_'+str(i);m=UsdGeom.Mesh.Define(stage,path);prim=m.GetPrim();v=np.asarray(piece['vertices']);v=v@relative[:3,:3].T+relative[:3,3]
            m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)));m.CreateFaceVertexCountsAttr([3]*len(piece['faces']));m.CreateFaceVertexIndicesAttr(np.asarray(piece['faces']).reshape(-1).tolist());m.CreateSubdivisionSchemeAttr('none');m.CreateVisibilityAttr('invisible')
            UsdPhysics.CollisionAPI.Apply(prim);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull');PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateMinThicknessAttr(0.);api=PhysxSchema.PhysxCollisionAPI.Apply(prim);api.CreateContactOffsetAttr(spec['source_contact_offset_m']);api.CreateRestOffsetAttr(spec['source_rest_offset_m'])
            prim.CreateAttribute('contact:owner',Sdf.ValueTypeNames.Token).Set(piece['owner']);prim.CreateAttribute('contact:finger',Sdf.ValueTypeNames.Token).Set(name)
            if material:UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose='physics')
            paths.append(path)
        p.RemoveAPI(UsdPhysics.CollisionAPI)
        PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.)
        mass_after={a.GetName():str(a.Get()) for a in body.GetAttributes() if a.GetName() in mass_before}
        if mass_after!=mass_before:raise RuntimeError('ownership replacement changed mass/inertia')
        installed[name]={'body':bodypath,'source_collider':sourcepath,'colliders':paths,'mass_inertia_before':mass_before,'mass_inertia_after':mass_after,'contact_offset_m':spec['source_contact_offset_m'],'rest_offset_m':spec['source_rest_offset_m']}
    return installed


class NativeOwnershipReports:
    def __init__(self,stage,target_paths,physics_dt,allowed_pad_targets=None,world=None,sample_provider=None):
        from pxr import PhysicsSchemaTools,Usd
        import omni.physx
        self.dt=physics_dt;self.targets=set(target_paths);self.allowed_pad_targets=set(allowed_pad_targets or target_paths);self.rows=[];self.paths={};self.decode=PhysicsSchemaTools.intToSdfPath
        for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
            owner=p.GetAttribute('contact:owner').Get()
            if owner:self.paths[str(p.GetPath())]=(p.GetAttribute('contact:finger').Get(),owner)
        self.world=world;self.sample_provider=sample_provider;self.window_dt=0.;self.window_steps=0;self.physics_time=0.;self.physics_steps=[]
        if world is not None:world.add_physics_callback('piper_owned_contact_clock',self.on_physics_step)
        self.subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self.callback)

    def clear(self):self.rows=[];self.window_dt=0.;self.window_steps=0

    def on_physics_step(self,dt):
        if self.physics_steps:self.physics_steps[-1]['ownership']=self.summarize(self.physics_steps[-1]['contacts'],self.physics_steps[-1]['physics_dt_s'])
        dt=float(dt);self.window_dt+=dt;self.window_steps+=1;self.physics_time+=dt
        if self.sample_provider is not None:
            sample=self.sample_provider();sample.update(t=self.physics_time-dt,physics_dt_s=dt,geometry_pose_time='native rigid-body tensor pose immediately before physics integration',contacts=[])
            self.physics_steps.append(sample)

    def callback(self,headers,data):
        for h in headers:
            a,b=str(self.decode(h.collider0)),str(self.decode(h.collider1))
            matches=lambda path:any(path==t or path.startswith(t+'/') for t in self.targets)
            if a in self.paths and matches(b):owned=a;target=b
            elif b in self.paths and matches(a):owned=b;target=a
            else:
                bodies={path.rsplit('/contact_owned/',1)[0]:finger for path,(finger,owner) in self.paths.items()}
                unknown=next(((path,other,finger) for path,other in [(a,b),(b,a)] for body,finger in bodies.items() if path.startswith(body+'/') and matches(other)),None)
                if unknown is None:continue
                owned,target,finger=unknown;self.paths[owned]=(finger,'unknown')
            finger,owner=self.paths[owned]
            for d in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]:
                impulse=np.array([d.impulse.x,d.impulse.y,d.impulse.z]);force=float(np.linalg.norm(impulse)/self.dt)
                if owned==b:impulse=-impulse
                contact={'collider':owned,'target':target,'finger':finger,'owner':owner,'allowed_pad_target':any(target==t or target.startswith(t+'/') for t in self.allowed_pad_targets),'force_n':force,'impulse_world_ns':impulse.tolist(),'separation_m':float(d.separation),'face0':int(d.face_index0),'face1':int(d.face_index1)}
                self.rows.append(contact)
                if self.physics_steps:self.physics_steps[-1]['contacts'].append(contact)

    def summarize(self,rows,dt):
        active=[x for x in rows if x['force_n']>0]
        if dt<=0:raise RuntimeError('native physics contact interval was not measured')
        impulses={n:np.sum([x['impulse_world_ns'] for x in active if x['finger']==n and x['owner']=='pad'],axis=0) if any(x['finger']==n and x['owner']=='pad' for x in active) else np.zeros(3) for n in FINGERS}
        return {'contacts':rows.copy(),'pad_forces_n':{n:float(np.linalg.norm(v)/dt) for n,v in impulses.items()},'pad_force_impulse_sum_over_nominal_dt_n':{n:sum(x['force_n'] for x in active if x['finger']==n and x['owner']=='pad') for n in FINGERS},'metal_contacts':sum(x['owner']!='pad' for x in active),'pad_target_violations':sum(x['owner']=='pad' and not x['allowed_pad_target'] for x in active),'unknown_allowed':False,'classification':'native collider ID; no position projection','force_rule':'net native impulse divided by measured physics interval, not by control-loop dt'}

    def state(self):
        dt=self.window_dt if self.world is not None else self.dt
        result=self.summarize(self.rows,dt);result.update(physics_window_dt_s=dt,physics_substeps=self.window_steps,physics_time_s=self.physics_time)
        if self.physics_steps:self.physics_steps[-1]['ownership']=self.summarize(self.physics_steps[-1]['contacts'],self.physics_steps[-1]['physics_dt_s'])
        return result
