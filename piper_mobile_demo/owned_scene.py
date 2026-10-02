"""Cached native finger solids and official non-pad surfaces; no contact waiver.

The cache changes query cost, not geometry. An allowed pad collider never
allows a raw official non-pad triangle to intersect the target.
"""
import json
from pathlib import Path
import numpy as np
import trimesh
import fcl
from .model import fcl_geometry, obj
from .contact_ownership import FINGERS


def meshes(entry, cooked=True):
    if entry.get('analytic', {}).get('shape') == 'box':
        return [trimesh.creation.box([float(entry['analytic']['size'])]*3)]
    if cooked:
        if not entry.get('convexes'):raise ValueError('missing native convexes: '+entry['path'])
        return [trimesh.Trimesh(c['vertices'], [[p[0],p[i],p[i+1]] for p in c['polygons'] for i in range(1,len(p)-1)], process=False) for c in entry['convexes']]
    counts=entry['raw_counts'];indices=entry['raw_indices'];faces=[];offset=0
    for count in counts:
        p=indices[offset:offset+count];offset+=count
        faces.extend([[p[0],p[i],p[i+1]] for i in range(1,count-1)])
    return [trimesh.Trimesh(entry['raw_points'],faces,process=True)]


class Shape:
    def __init__(self, mesh, T, convex, path, body, owner=None):
        mesh=mesh.copy();mesh.apply_transform(T)
        self.vertices=np.asarray(mesh.vertices);self.path=path;self.body=body;self.owner=owner
        if convex:
            faces=np.column_stack((np.full(len(mesh.faces),3),mesh.faces)).reshape(-1).astype(np.int32)
            self.geometry=fcl.Convex(self.vertices.astype(np.float64),len(mesh.faces),faces)
        else:self.geometry=fcl_geometry(mesh)
        self.object=obj(self.geometry,np.eye(4));self.place(np.eye(4))

    def place(self,T):
        self.object.setTransform(fcl.Transform(T[:3,:3],T[:3,3]))
        v=self.vertices@T[:3,:3].T+T[:3,3];self.bounds=np.array([v.min(0),v.max(0)])


def intersects(a,b):
    if np.any(a.bounds[0]>b.bounds[1]) or np.any(b.bounds[0]>a.bounds[1]):return False
    return fcl.collide(a.object,b.object)>0


class OwnedFingerScene:
    def __init__(self,root,export,ownership,allowed_targets,moving_link):
        self.root=Path(root);self.data=json.loads(Path(export).read_text());self.manifest=json.loads(Path(ownership).read_text())
        self.allowed_targets=set(allowed_targets);self.kinematic_link=moving_link
        owners={e.get('rigid_body_path','').split('/')[-1] for e in self.data['shapes'] if e['path'] in self.allowed_targets}
        if len(owners)!=1 or not next(iter(owners)):raise ValueError('target rigid-body association ambiguous')
        self.moving_link=next(iter(owners));moving_link=self.moving_link
        self.fingers=[];self.raw=[];self.scene=[];self.raw_scene=[];self.planes=[];self.other_robot=[]
        self.moving_reference=None;self.reference_angle_rad=0.
        for e in self.data['shapes']:
            path=e['path'];body=e.get('rigid_body_path','').split('/')[-1]
            if e.get('finger'):
                T=np.linalg.inv(np.asarray(e['rigid_body_world_transform']))@np.asarray(e['world_transform'])
                self.fingers.extend(Shape(m,T,True,path,e['finger'],e['owner']) for m in meshes(e))
            elif path.startswith('/World/Piper'):
                if e.get('convexes'):
                    T=np.linalg.inv(np.asarray(e['rigid_body_world_transform']))@np.asarray(e['world_transform'])
                    self.other_robot.extend(Shape(m,T,True,path,body) for m in meshes(e))
            elif e['type']=='Plane':
                # The existing ground-plane export is Z-up; only test against
                # its actual authored height. No collider is disabled.
                self.planes.append(float(np.asarray(e['world_transform'])[2,3]))
            else:
                T=np.asarray(e['world_transform'])
                if body==moving_link:
                    B=np.asarray(e['rigid_body_world_transform']);self.moving_reference=B;T=np.linalg.inv(B)@T
                self.scene.extend(Shape(m,T,True,path,body) for m in meshes(e))
                self.raw_scene.extend(Shape(m,T,bool(e.get('analytic')),path,body) for m in meshes(e,False))
        for name in FINGERS:
            expected=len(self.manifest['fingers'][name]['pieces'])
            if sum(x.body==name for x in self.fingers)!=expected:raise ValueError('native ownership count mismatch')
            m=trimesh.load(self.root/'third_party/agilex_piper/description/meshes'/f'{name}.stl',force='mesh')
            permitted=set(self.manifest['fingers'][name]['official_pad_face_ids']);m.update_faces(np.array([i not in permitted for i in range(len(m.faces))]))
            self.raw.append(Shape(m,np.eye(4),False,'official_nonpad_'+name,name,'metal'))
        if self.moving_reference is None:raise ValueError('moving body missing from native export')
        self.place_scene(self.moving_reference)

    def place_scene(self,moving_pose):
        for s in self.scene+self.raw_scene:s.place(moving_pose if s.body==self.moving_link else np.eye(4))

    def check(self,poses,moving_pose=None,allow_pad=True):
        self.place_scene(self.moving_reference if moving_pose is None else moving_pose)
        for a in self.fingers+self.raw:a.place(np.asarray(poses[a.body]))
        for a in self.fingers:
            if any(a.bounds[0,2]<=z for z in self.planes):return False,'FLOOR_COLLISION:'+a.path
            for b in self.scene:
                if a.owner=='pad' and allow_pad and b.path in self.allowed_targets:continue
                if intersects(a,b):return False,'COOKED_'+a.owner.upper()+':'+a.path+':'+b.path
        for a in self.raw:
            for b in self.raw_scene+self.scene:
                if intersects(a,b):return False,'RAW_NONPAD:'+a.body+':'+b.path
        return True,'OWNED_GEOMETRY_SAFE'

    def check_robot(self,model,q,base,angle=0.,finger_q=None,allow_pad=True):
        ok,why,_=arm_check(model,q,base,angle,finger_q)
        if not ok:return False,why
        poses=model.poses(q,base,finger_q=finger_q)
        initial=model.asset_T@model.asset.root_to_link(self.moving_link,{model.manifest['joint_name']:self.reference_angle_rad})
        moving=model.asset_T@model.asset.root_to_link(self.moving_link,{model.manifest['joint_name']:angle})
        ok,why=self.check(poses,moving@np.linalg.inv(initial)@self.moving_reference,allow_pad)
        if not ok:return False,why
        for b in self.other_robot:
            b.place(poses[b.body])
            for a in self.fingers:
                if frozenset((a.body,b.body)) not in model.allowed and intersects(a,b):return False,'OWNED_SELF_COLLISION:'+a.path+':'+b.path
        return True,'OWNED_GEOMETRY_SAFE'

    def check_self(self,poses,allowed):
        for a in self.fingers:a.place(np.asarray(poses[a.body]))
        for b in self.other_robot:
            b.place(np.asarray(poses[b.body]))
            for a in self.fingers:
                if frozenset((a.body,b.body)) not in allowed and intersects(a,b):return False,'OWNED_SELF_COLLISION:'+a.path+':'+b.path
        return True,'OWNED_SELF_SAFE'


def arm_check(model,q,base,angle=0.,finger_q=None):
    """Existing collision constraints for non-finger links; IK unchanged."""
    original=model.robot
    try:
        model.robot=[(n,g) for n,g in original if n not in FINGERS]
        return model.check(q,base,angle=angle,finger_q=finger_q)
    finally:model.robot=original
