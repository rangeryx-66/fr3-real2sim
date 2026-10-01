"""Offline audit of exported PhysX convexes, with strict official flat-pad witnesses."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import trimesh,fcl
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import fcl_geometry,obj
from piper_mobile_demo.collision_surface import intersection_points


def mesh_from(entry,cooked=True):
    pieces=[]
    if cooked:
        if not entry.get('convexes') or entry.get('export_error'):raise RuntimeError('missing native cooking export: '+entry['path'])
        for c in entry['convexes']:
            faces=[[p[0],p[k],p[k+1]] for p in c['polygons'] for k in range(1,len(p)-1)]
            pieces.append(trimesh.Trimesh(vertices=c['vertices'],faces=faces,process=False))
    else:
        faces=[];start=0
        for count in entry['raw_counts']:
            p=entry['raw_indices'][start:start+count];start+=count
            faces.extend([[p[0],p[k],p[k+1]] for k in range(1,count-1)])
        pieces=[trimesh.Trimesh(vertices=entry['raw_points'],faces=faces,process=True)]
    return pieces


def strict_pair(A,TA,B,TB,pad_triangles,finger_frame):
    worldA=A.copy();worldA.apply_transform(TA);worldB=B.copy();worldB.apply_transform(TB)
    ga=fcl_geometry(worldA);gb=fcl_geometry(worldB)
    result=fcl.CollisionResult();fcl.collide(obj(ga,np.eye(4)),obj(gb,np.eye(4)),fcl.CollisionRequest(num_max_contacts=4096,enable_contact=True),result)
    violations=[];pad_points=0;unclassified=0
    inv=np.linalg.inv(finger_frame);triA=A.triangles@TA[:3,:3].T+TA[:3,3];triB=B.triangles@TB[:3,:3].T+TB[:3,3]
    for contact in result.contacts:
        if not(0<=contact.b1<len(triA) and 0<=contact.b2<len(triB)):
            unclassified+=1;continue
        witnesses=intersection_points(triA[contact.b1],triB[contact.b2])
        if len(witnesses)==0:unclassified+=1;continue
        for world in witnesses:
            local=(inv@np.r_[world,1])[:3]
            distance=float(np.min(np.linalg.norm(trimesh.triangles.closest_point(pad_triangles,np.tile(local,(len(pad_triangles),1)))-local,axis=1)))
            if distance>1e-6:violations.append({'point_world':world.tolist(),'point_finger':local.tolist(),'distance_from_original_pad_m':distance,'finger_triangle':int(contact.b1),'target_triangle':int(contact.b2)})
            else:pad_points+=1
    return {'fcl_contact_count':len(result.contacts),'pad_witness_count':pad_points,'nonpad_witnesses':violations,'unclassified_contacts':unclassified,
            'strict_valid':len(violations)==0 and unclassified==0}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('export',type=Path);p.add_argument('output',type=Path);p.add_argument('--target-body',default='l_1');a=p.parse_args()
    data=json.loads(a.export.read_text());shapes=data['shapes'];rows=[];pairs=[]
    fingers=[s for s in shapes if s.get('rigid_body_path','').split('/')[-1] in ['gripper_link1','gripper_link2'] and s.get('raw_points')]
    targets=[s for s in shapes if (s.get('rigid_body_path','').split('/')[-1]==a.target_body or s['path'].split('/')[-1]==a.target_body) and (s.get('raw_points') or s.get('analytic'))]
    if len(fingers)!=2 or not targets:raise RuntimeError('incomplete export: require both finger colliders and at least one target shape')
    for entry in fingers+targets:
        if not entry.get('raw_points'):continue
        raw=mesh_from(entry,False)[0];cooked=mesh_from(entry);hull=raw.convex_hull
        row={'path':entry['path'],'raw_vertex_count':len(raw.vertices),'raw_volume_m3':abs(float(raw.volume)),'raw_hull_vertex_count':len(hull.vertices),'raw_hull_volume_m3':abs(float(hull.volume)), 'cooked_shapes':[],'offsets':entry['offsets'],'runtime_body_shapes':entry.get('runtime_body_shapes')}
        for c in cooked:
            distance=trimesh.proximity.closest_point_naive(hull,c.vertices)[1]
            reverse=trimesh.proximity.closest_point_naive(c,hull.vertices)[1]
            row['cooked_shapes'].append({'vertices':len(c.vertices),'triangles':len(c.faces),'volume_m3':abs(float(c.volume)),'bounds':c.bounds.tolist(),'bounds_delta_from_raw_m':(c.bounds-raw.bounds).tolist(),'max_vertex_distance_to_raw_hull_m':float(distance.max()),'max_raw_hull_vertex_distance_to_cooked_m':float(reverse.max()),'volume_change_relative_to_raw_hull':float(abs(c.volume)/abs(hull.volume)-1)})
        rows.append(row)
    for finger in fingers:
        name=finger['rigid_body_path'].split('/')[-1];official=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{name}.stl',force='mesh')
        inner=official.vertices[:,2].min();pads=official.triangles[np.max(abs(official.triangles[:,:,2]-inner),axis=1)<1e-6]
        TA=np.asarray(finger['world_transform']);frame=np.asarray(finger.get('rigid_body_world_transform',TA))
        for target in targets:
            TB=np.asarray(target['world_transform'])
            if target.get('analytic',{}).get('shape')=='box':
                size=float(target['analytic']['size']);targets_raw=[trimesh.creation.box([size]*3)];targets_cooked=targets_raw
            else:targets_raw=mesh_from(target,False);targets_cooked=mesh_from(target)
            for mode,As,Bs in [('physx_cooked',mesh_from(finger),targets_cooked),('raw_finger_raw_target',mesh_from(finger,False),targets_raw),('raw_finger_cooked_target',mesh_from(finger,False),targets_cooked),('cooked_finger_raw_target',mesh_from(finger),targets_raw),('old_full_hull_raw_target',[mesh_from(finger,False)[0].convex_hull],targets_raw)]:
                for i,A in enumerate(As):
                    for j,B in enumerate(Bs):
                        check=strict_pair(A,TA,B,TB,pads,frame)
                        if check['fcl_contact_count']:
                            pairs.append({'finger':name,'target_path':target['path'],'mode':mode,'convex_pair':[i,j],**check})
    result={'export':str(a.export),'target_shape_count':len(targets),'finger_shape_count':len(fingers),'geometry':rows,'intersection_audit':pairs,'strict_pad_tolerance_m':1e-6,'uses_contact_offset_to_allow_metal':False}
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps({'targets':len(targets),'fingers':len(fingers),'intersecting_pairs_by_mode':{m:sum(x['mode']==m for x in pairs) for m in ['physx_cooked','raw_finger_raw_target','old_full_hull_raw_target']},'nonpad_witnesses_by_mode':{m:sum(len(x['nonpad_witnesses']) for x in pairs if x['mode']==m) for m in ['physx_cooked','raw_finger_raw_target','old_full_hull_raw_target']}},indent=2))
if __name__=='__main__':main()
