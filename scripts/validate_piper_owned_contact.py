"""Collider-ID geometry validation using native cooked exports and official face IDs.

Contact offset is a physics envelope, not permission for metal contact. Raw
body triangles remain forbidden regardless of what cooking does to them.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np,trimesh,fcl
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from validate_piper_cooked_contact import mesh_from
from piper_mobile_demo.model import fcl_geometry,obj
from piper_mobile_demo.contact_ownership import FINGERS


def query(A,TA,B,TB,convex_a=False,convex_b=False,broadphase_clearance=0.):
    A=A.copy();A.apply_transform(TA);B=B.copy();B.apply_transform(TB)
    gap=np.maximum(np.maximum(A.bounds[0]-B.bounds[1],B.bounds[0]-A.bounds[1]),0.);lower_bound=float(np.linalg.norm(gap))
    if lower_bound>broadphase_clearance:return {'intersects':False,'distance_m':lower_bound,'distance_is_lower_bound':True,'contacts':0,'triangle_ids':[]}
    def geometry(mesh,convex):
        if not convex:return fcl_geometry(mesh)
        faces=np.column_stack([np.full(len(mesh.faces),3),mesh.faces]).reshape(-1).astype(np.int32)
        return fcl.Convex(np.asarray(mesh.vertices,dtype=np.float64),len(mesh.faces),faces)
    a=obj(geometry(A,convex_a),np.eye(4));b=obj(geometry(B,convex_b),np.eye(4));c=fcl.CollisionResult();fcl.collide(a,b,fcl.CollisionRequest(num_max_contacts=4096,enable_contact=True),c)
    d=fcl.DistanceResult();distance=float(fcl.distance(a,b,fcl.DistanceRequest(enable_nearest_points=True),d))
    return {'intersects':bool(c.is_collision),'distance_m':distance,'contacts':len(c.contacts),'triangle_ids':[[int(x.b1),int(x.b2)] for x in c.contacts]}


def contact_envelope(entry):
    explicit=entry.get('offsets',{}).get('contactOffset',{})
    if explicit.get('authored') and explicit.get('schema_value') is not None:return float(explicit['schema_value']),'authored actual collider offset'
    values=np.asarray(entry.get('runtime_body_shapes',{}).get('contact_offsets',[])).reshape(-1)
    values=values[np.isfinite(values)]
    if len(values):return float(values.max()),'conservative maximum of native runtime body shape offsets; order not assumed'
    raise RuntimeError('unresolved actual contact offset for '+entry['path'])


def target_mesh(entry,cooked):
    if entry.get('analytic',{}).get('shape')=='box':return [trimesh.creation.box([float(entry['analytic']['size'])]*3)]
    return mesh_from(entry,cooked)


def validate(data,manifest,target_body,poses=None):
    shapes=data['shapes'];fingers=[s for s in shapes if s.get('finger') in FINGERS and s.get('owner') in ['metal','pad']]
    targets=[s for s in shapes if (s.get('rigid_body_path','').split('/')[-1]==target_body or s['path'].split('/')[-1]==target_body) and (s.get('raw_points') or s.get('analytic'))]
    if not targets or {s['finger'] for s in fingers}!=set(FINGERS):raise RuntimeError('incomplete owned shape export')
    counts={n:sum(s['finger']==n for s in fingers) for n in FINGERS}
    if any(counts[n]!=len(manifest['fingers'][n]['pieces']) for n in FINGERS):raise RuntimeError('ownership manifest/cooked collider count mismatch')
    rows=[];raw=[]
    for finger in fingers:
        n=finger['finger'];T=np.asarray(finger['world_transform'])
        if poses:T=poses[n]@np.linalg.inv(np.asarray(finger['rigid_body_world_transform']))@T
        for target in targets:
            U=np.asarray(target['world_transform'])
            if poses and '__target__' in poses and target.get('rigid_body_world_transform') is not None:U=poses['__target__']@np.linalg.inv(np.asarray(target['rigid_body_world_transform']))@U
            for A in mesh_from(finger):
                for B in target_mesh(target,True):
                    fa,fp=contact_envelope(finger);ta,tp=contact_envelope(target);check=query(A,T,B,U,True,True,fa+ta);rows.append({'collider':finger['path'],'finger':n,'owner':finger['owner'],'target':target['path'],'actual_contact_envelope_m':fa+ta,'offset_provenance':[fp,tp],'within_contact_envelope':check['distance_m']<=fa+ta,**check})
    for n in FINGERS:
        entry=next(s for s in fingers if s['finger']==n);T=np.asarray(entry['rigid_body_world_transform']);T=poses[n] if poses else T
        mesh=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{n}.stl',force='mesh');pad_ids=set(manifest['fingers'][n]['official_pad_face_ids']);metal=mesh.copy();metal.update_faces(np.array([i not in pad_ids for i in range(len(mesh.faces))]))
        for target in targets:
            U=np.asarray(target['world_transform'])
            if poses and '__target__' in poses and target.get('rigid_body_world_transform') is not None:U=poses['__target__']@np.linalg.inv(np.asarray(target['rigid_body_world_transform']))@U
            for mode in ['raw_target','cooked_target']:
                for B in target_mesh(target,mode=='cooked_target'):raw.append({'finger':n,'target':target['path'],'mode':mode,'owner':'metal','ownership_rule':'official surface face IDs; no projection or distance tolerance',**query(metal,T,B,U,False,mode=='cooked_target' or bool(target.get('analytic')))})
    forbidden=[r for r in rows if r['owner']=='metal' and r['intersects']];potential=[r for r in rows if r['owner']=='metal' and r['within_contact_envelope']];rawforbidden=[r for r in raw if r['intersects']]
    return {'geometry_safe':not forbidden and not rawforbidden,'cooked_metal_intersections':sum(r['intersects'] for r in rows if r['owner']=='metal'),'cooked_metal_contact_envelope_potential_pairs':len(potential),'contact_envelope_is_not_loaded_contact':True,'raw_metal_intersections':len(rawforbidden),'finger_shape_counts':counts,'target_shape_count':len(targets),'cooked_pairs':rows,'raw_official_audit':raw,'pad_contact_force_source':'native PhysX report; geometric overlap does not prove loaded contact','no_contact_position_classification':True,'collision_tolerance_added':False}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('export',type=Path);p.add_argument('output',type=Path);p.add_argument('--ownership',type=Path,default=ROOT/'config/piper_contact_ownership.json');p.add_argument('--target-body',default='l_1');p.add_argument('--observations',type=Path);a=p.parse_args();data=json.loads(a.export.read_text());manifest=json.loads(a.ownership.read_text());r=validate(data,manifest,a.target_body)
    if a.observations:
        states=json.loads(a.observations.read_text());replay=[];owner_registry={e['path']:e.get('owner') for e in data['shapes']}
        for i,s in enumerate(states):
            poses={n:np.array(t) for n,t in s['finger_world_poses'].items()}
            if s.get('T_moving_link'):poses['__target__']=np.array(s['T_moving_link'])
            v=validate(data,manifest,a.target_body,poses);native=s.get('ownership',{});native_classification=all(c.get('owner')=='pad' and c.get('allowed_pad_target',True) for c in native.get('contacts',[]) if c['force_n']>0);replay.append({'sample':i,'phase':s['phase'],'aperture_m':s['aperture_m'],'geometry_safe':v['geometry_safe'],'cooked_metal_intersections':v['cooked_metal_intersections'],'cooked_metal_contact_envelope_potential_pairs':v['cooked_metal_contact_envelope_potential_pairs'],'raw_metal_intersections':v['raw_metal_intersections'],'native_metal_contacts':native.get('metal_contacts'),'native_identity_contact_safe':native_classification,'unified_safe':v['geometry_safe'] and native_classification,'minimum_cooked_metal_distance_m':min(x['distance_m'] for x in v['cooked_pairs'] if x['owner']=='metal')})
        r['replay']=replay;r['geometry_safe_all_samples']=all(x['geometry_safe'] for x in replay);r['unified_safe_all_samples']=all(x['unified_safe'] for x in replay);r['native_contact_owner_mismatches']=sum(any(c.get('owner')!=owner_registry.get(c.get('collider')) for c in s.get('ownership',{}).get('contacts',[])) for s in states);r['native_metal_contact_samples']=sum((x['native_metal_contacts'] or 0)>0 for x in replay)
    a.output.write_text(json.dumps(r,indent=2));print({k:v for k,v in r.items() if k not in ['cooked_pairs','raw_official_audit','replay']})
if __name__=='__main__':main()
