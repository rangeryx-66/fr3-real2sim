"""GT triangle surface sweeps over URDF rubber pads, without moving grasps.

Distances are in metres in fr3_hand_tcp. Pad inner faces are q - 20 um:
URDF finger origin z=.0584, TCP z=.1034, rubber tip centre
(0,.00758,.04525), size (.0175,.0152,.0185). Right finger is rotated pi.
This is a geometric necessary-condition check, not a force-closure certificate.
"""
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from hand_geometry import PAD_X, PAD_Z

ROOT=Path(__file__).resolve().parents[1]
CACHE={}

def non_pad_sweep(mesh,T,side,pad_joint):
    """URDF non-pad boxes: detect body contact earlier than the rubber face.

    Sample their leading surfaces along the true closing direction. The right
    finger's pi joint rotation mirrors x/y; its diagonal box has the authored
    equivalent opposite rotation. Contacts within 1 mm are not separated by
    this coarse diagnostic, and are left to physical validation.
    """
    inv=np.linalg.inv(T);rows=[]
    parts=[('screw_mount',[.022,.015,.020],[0,.0185,.011],0.),
           ('carriage',[.022,.0088,.0038],[0,.0068,.0022],0.),
           ('diagonal',[.0175,.007,.0235],[0,.0159,.02835],np.pi/6)]
    for name,size,center,rx in parts:
        size=np.array(size);center=np.array(center);center[2]-=.045
        R=Rotation.from_euler('x',rx).as_matrix()
        corners=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*size/2
        bounds=corners@R.T+center
        xs=np.arange(bounds[:,0].min()+.0005,bounds[:,0].max(),.001)
        zs=np.arange(bounds[:,2].min()+.0005,bounds[:,2].max(),.001)
        x,z=np.meshgrid(xs,zs);orig=np.column_stack([x.ravel(),np.full(x.size,-.1),z.ravel()])
        origin_local=(orig-center)@R;direction=np.array([0,1,0])@R
        lo=np.full(len(orig),-np.inf);hi=np.full(len(orig),np.inf);valid=np.ones(len(orig),dtype=bool)
        for axis in range(3):
            if abs(direction[axis])<1e-10:
                valid &= abs(origin_local[:,axis])<=size[axis]/2
            else:
                a=(-size[axis]/2-origin_local[:,axis])/direction[axis]
                b=(size[axis]/2-origin_local[:,axis])/direction[axis]
                lo=np.maximum(lo,np.minimum(a,b));hi=np.minimum(hi,np.maximum(a,b))
        valid &= lo<=hi
        front=orig[valid];front[:,1]+=lo[valid]
        front[:,:2]*=side
        rays=front.copy();rays[:,1]+=side*.04
        dirs=np.tile([0,-side,0],(len(rays),1))
        points,ri,fi=mesh.ray.intersects_location(rays@inv[:3,:3].T+inv[:3,3],dirs@inv[:3,:3].T,multiple_hits=True)
        points=np.asarray(points).reshape(-1,3)
        p=points@T[:3,:3].T+T[:3,3]
        travel=np.einsum('ij,ij->i',p-rays[ri],dirs[ri])
        valid=(travel>=0)&(travel<=.04)
        q=.04-travel[valid]
        first=float(q.max()) if len(q) else None
        rows.append(dict(part=name,first_contact_joint_m=first,
                         precedes_pad=first is not None and first>pad_joint+.001))
    return rows

def target_mesh(name):
    if name not in CACHE:
        d=np.load(ROOT/f'assets/arena_complex/{name}_mesh.npz')
        CACHE[name]=trimesh.Trimesh(vertices=d['vertices'],faces=d['triangles'],process=False)
    return CACHE[name]

def evaluate(name,H,O,pitch=.001,patch_depth=.001):
    mesh=target_mesh(name)
    T=np.linalg.inv(H)@O
    nx=int(np.ceil((PAD_X[1]-PAD_X[0])/pitch))
    nz=int(np.ceil((PAD_Z[1]-PAD_Z[0])/pitch))
    xs=np.linspace(*PAD_X,nx+1);zs=np.linspace(*PAD_Z,nz+1)
    xx,zz=np.meshgrid((xs[:-1]+xs[1:])/2,(zs[:-1]+zs[1:])/2)
    cell_area=(xs[1]-xs[0])*(zs[1]-zs[0])
    sides=[]
    # Express rays in target frame; reuse its spatial index for all candidates.
    inv=np.linalg.inv(T)
    for side in [1,-1]:
        origins=np.column_stack([xx.ravel(),np.full(xx.size,side*.03998),zz.ravel()])
        directions=np.tile([0,-side,0],(len(origins),1))
        origins_O=origins@inv[:3,:3].T+inv[:3,3]
        directions_O=directions@inv[:3,:3].T
        points,ray_ids,tri_ids=mesh.ray.intersects_location(origins_O,directions_O,multiple_hits=True)
        points=np.asarray(points).reshape(-1,3)
        points_H=points@T[:3,:3].T+T[:3,3]
        distances=np.einsum('ij,ij->i',points_H-origins[ray_ids],directions[ray_ids])
        hits={}
        for j in np.argsort(distances):
            ri=int(ray_ids[j]);distance=float(distances[j])
            if ri not in hits and -1e-8<=distance<=.04000001:
                hits[ri]=int(j)
        if not hits:
            sides.append(dict(side=side,hit_count=0,patch_area_m2=0.,normal_patch_area_m2=0.,contacts=[]))
            continue
        ids=np.array(list(hits.values()));p=points_H[ids]
        normals=mesh.face_normals[tri_ids[ids]]@T[:3,:3].T
        # First plane reached by this translating rigid pad; deeper surfaces are
        # not simultaneously reachable unless the object deforms or is pushed.
        plane=float(np.max(side*p[:,1]));patch=(plane-side*p[:,1])<=patch_depth
        aligned=normals[:,1]*side>=1/np.sqrt(1+.7**2)
        valid=patch&aligned
        mean=normals[valid].mean(0) if valid.any() else np.zeros(3)
        if np.linalg.norm(mean)>0:mean/=np.linalg.norm(mean)
        contacts=[dict(point_TCP=p[j].tolist(),normal_TCP=normals[j].tolist(),pad_cell=int(ray_ids[ids[j]]),in_first_patch=bool(patch[j]),in_friction_cone=bool(aligned[j])) for j in range(len(ids))]
        sides.append(dict(side=side,hit_count=len(ids),first_contact_joint_m=plane+.00002,patch_area_m2=float(patch.sum()*cell_area),normal_patch_area_m2=float(valid.sum()*cell_area),normal=mean.tolist(),contacts=contacts))
    bilateral=all(s['normal_patch_area_m2']>=4*cell_area for s in sides)
    opposition=float(np.dot(sides[0].get('normal',[0,0,0]),sides[1].get('normal',[0,0,0])))
    reasons=[]
    if not all(s['hit_count'] for s in sides):reasons.append('SINGLE_FINGER_CONTACT')
    if not bilateral:reasons.append('INSUFFICIENT_SURFACE_PATCH')
    if opposition>-.5:reasons.append('NON_OPPOSED_NORMALS')
    body=[dict(side=s['side'],parts=non_pad_sweep(mesh,T,s['side'],s.get('first_contact_joint_m',0))) for s in sides]
    if any(p['precedes_pad'] for s in body for p in s['parts']):reasons.append('NON_PAD_CONTACT_BEFORE_PAD')
    return dict(passed=not reasons,reasons=reasons,sides=sides,normal_opposition=opposition,
                non_pad_sweep=body,
                contact_width_m=sum(s.get('first_contact_joint_m',0) for s in sides),
                first_contact_asymmetry_m=abs(sides[0].get('first_contact_joint_m',0)-sides[1].get('first_contact_joint_m',0)),
                pad_cell_area_m2=float(cell_area),grid_shape=[nz,nx],patch_depth_m=patch_depth,
                T_TCP_target=T.tolist(),palm_clearance_check='strict MoveIt open-hand collision required separately')


if __name__ == '__main__':
    import argparse, json
    from frames import grasp_to_tcp
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    request=json.loads(Path(args.input).read_text());data=request['data']
    result={}
    for g in data['grasps']:
        _,H,_=grasp_to_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'])
        result[str(g['rank'])]=evaluate(request['target'],H,np.array(request['T_B_target']))
    Path(args.output).write_text(json.dumps(result))
