"""GT geometric oracle and frozen refinement worker; contains no asset branches."""
import argparse,json,os,sys
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from mesh_hand_geometry import target_mesh,evaluate
from grasp_refinement import torque_metrics,generate
from grasp_family_expansion import generate as generate_families

def oracle(name,O):
    mesh=target_mesh(name);rng=np.random.default_rng(20260912)
    palm=trimesh.load_mesh(ROOT/'franka_description/meshes/robot_ee/franka_hand_white/collision/hand.stl')
    palm_vertices=np.asarray(palm.vertices)-[0,0,.1034]
    hull=ConvexHull(palm_vertices);equations=hull.equations
    # An oracle grasp must leave room for physical closure/settling at the palm.
    # This affects only GT proposals, never the AnyGrasp or frozen mesh gate.
    palm_clearance=.005
    palm_lower=palm_vertices.min(0)-palm_clearance;palm_upper=palm_vertices.max(0)+palm_clearance
    fi=rng.choice(len(mesh.faces),1024,p=mesh.area_faces/mesh.area)
    uv=rng.uniform(size=(1024,2));flip=uv.sum(1)>1;uv[flip]=1-uv[flip]
    tri=mesh.triangles[fi];surface=tri[:,0]+uv[:,0,None]*(tri[:,1]-tri[:,0])+uv[:,1,None]*(tri[:,2]-tri[:,0])
    normal=mesh.face_normals[fi]
    hit,ri,faces=mesh.ray.intersects_location(surface-normal*.0001,-normal,multiple_hits=False)
    seeds=[];palm_rejected=0
    for point,idx,face in zip(hit,ri,faces):
        other=surface[idx];width=float(np.linalg.norm(point-other))
        if not .003<=width<=.078 or float(normal[idx]@mesh.face_normals[face])>-.5:continue
        midpoint=O[:3,:3]@((other+point)/2)+O[:3,3]
        closing=O[:3,:3]@(other-point)/width
        down=np.array([0.,0.,-1.]);approach=down-closing*np.dot(down,closing)
        if np.linalg.norm(approach)<.4:continue
        approach/=np.linalg.norm(approach)
        for angle in [0.,-25.,25.]:
            z=Rotation.from_rotvec(closing*np.deg2rad(angle)).apply(approach)
            if z[2]>-.4:continue
            for sign in [1.,-1.]:
                y=closing*sign;x=np.cross(y,z);H=np.eye(4);H[:3,:3]=np.column_stack([x,y,z]);H[:3,3]=midpoint-H[:3,:3]@np.array([0,0,.00025])
                T=np.linalg.inv(H)@O;v=mesh.vertices@T[:3,:3].T+T[:3,3]
                near=v[((v>=palm_lower)&(v<=palm_upper)).all(1)]
                if len(near) and ((near@equations[:,:3].T+equations[:,3])<=palm_clearance).all(1).any():
                    palm_rejected+=1;continue
                # Cheap physical ranking before the unchanged detailed mesh check.
                com=O[:3,:3]@mesh.center_mass+O[:3,3];lever=np.linalg.norm(np.cross(com-midpoint,y))
                seeds.append((float(lever),H,width))
    seeds.sort(key=lambda x:x[0]);unique=[];seen=set()
    for _,H,width in seeds:
        key=tuple(np.round(np.r_[H[:3,3]/.003,Rotation.from_matrix(H[:3,:3]).as_rotvec()/.1]).astype(int))
        if key in seen:continue
        seen.add(key);unique.append((H,width))
    rows=[]
    for idx,(H,width) in enumerate(unique[:512]):
        geometry=evaluate(name,H,O);torque=torque_metrics(name,H,O,geometry)
        if geometry['passed'] and torque['valid']:
            rows.append(dict(T_B_TCP=H.tolist(),width=width,geometry=geometry,torque=torque,proposal_id=idx))
    rows.sort(key=lambda r:(r['torque']['torque_cost'],r['proposal_id']))
    selected=[]
    for row in rows:
        H=np.asarray(row['T_B_TCP'])
        if any(np.linalg.norm(H[:3,3]-np.asarray(x['T_B_TCP'])[:3,3])<.004 and (Rotation.from_matrix(H[:3,:3]).inv()*Rotation.from_matrix(np.asarray(x['T_B_TCP'])[:3,:3])).magnitude()<np.deg2rad(12) for x in selected):continue
        row['rank']=len(selected);row['score']=None;selected.append(row)
        if len(selected)==20:break
    return dict(method='GT antipodal mesh geometry; no category templates',sampled_surface_points=1024,
                raw_pose_count=len(seeds),oracle_palm_clearance_m=palm_clearance,palm_hull_rejected=palm_rejected,mesh_checked=min(512,len(unique)),mesh_passed=len(rows),grasps=selected)

def main():
    p=argparse.ArgumentParser();p.add_argument('--request',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    request=json.loads(Path(a.request).read_text());name=request['target'];O=np.asarray(request['T_B_target'])
    if request['operation']=='oracle':out=oracle(name,O)
    elif request['operation']=='refine':out=generate(name,np.asarray(request['T_B_TCP']),O)
    elif request['operation']=='family':out=generate_families(name,O)
    else:
        out=[]
        for parent in request['parents']:
            H=np.asarray(parent['T_B_TCP']);geometry=evaluate(name,H,O);torque=torque_metrics(name,H,O,geometry)
            out.append({**parent,
                        'refinement_id':parent.get('refinement_id',0),
                        'offset_translation_TCP_m':parent.get('offset_translation_TCP_m',[0,0,0]),
                        'offset_rotation_TCP_deg':parent.get('offset_rotation_TCP_deg',[0,0,0]),
                        'geometry_passed':bool(geometry['passed'] and torque['valid']),
                        'mesh_hand_geometry':geometry,'torque':torque})
    Path(a.output).write_text(json.dumps(out))
    print('GEOMETRY_DONE',request['operation'],flush=True)
if __name__=='__main__':main()
