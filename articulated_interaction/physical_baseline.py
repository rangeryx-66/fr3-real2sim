"""Task collision checks for official whole fingers; no ownership partition gate."""
import numpy as np
import trimesh
import fcl
from pxr import Usd,UsdPhysics,PhysxSchema
from piper_mobile_demo.owned_scene import Shape,meshes,intersects
from piper_mobile_demo.contact_ownership import NativeOwnershipReports

FINGERS=('gripper_link1','gripper_link2')

class WholeFingerReports(NativeOwnershipReports):
    def __init__(self,stage,export,allowed,world,dt,provider):
        super().__init__(stage,['/World'],dt,allowed,world,provider)
        self.entries={e['path']:e for e in export['shapes']}
        for e in export['shapes']:
            if not e['path'].startswith('/World/Piper'):continue
            body=e.get('rigid_body_path','').split('/')[-1]
            self.paths[e['path']]=(body,'whole_finger' if body in FINGERS else 'robot_body')
            p=stage.GetPrimAtPath(e.get('rigid_body_path',''))
            if p:PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
        self.finger_body_paths={e['rigid_body_path']:e['rigid_body_path'].split('/')[-1] for e in export['shapes'] if e['path'] in self.paths}

    def summarize(self,rows,dt):
        impulses={n:np.sum([c['impulse_world_ns'] for c in rows if c['finger']==n and c['allowed_pad_target']],axis=0) if any(c['finger']==n and c['allowed_pad_target'] for c in rows) else np.zeros(3) for n in FINGERS}
        return {'contacts':rows.copy(),'finger_handle_forces_n':{n:float(np.linalg.norm(v)/dt) for n,v in impulses.items()},'physics_window_dt_s':dt,'classification':'whole official finger; task-region contacts, not pad triangle ownership'}


class PhysicalScene:
    def __init__(self,export,model,allowed):
        self.model=model;self.allowed=set(allowed);self.robot=[];self.scene=[];self.entries={e['path']:e for e in export['shapes']};self.moving_reference=None
        owners={e.get('rigid_body_path','').split('/')[-1] for e in export['shapes'] if e['path'] in self.allowed}
        if len(owners)!=1:raise RuntimeError('MOVING_BODY_ASSOCIATION_FAILED')
        self.moving_link=next(iter(owners))
        self.jaw_bounds={}
        for e in export['shapes']:
            if e['type']=='Plane':continue
            body=e.get('rigid_body_path','').split('/')[-1]
            if not e.get('convexes') and not e.get('analytic'):continue
            T=np.asarray(e['world_transform']);B=np.asarray(e.get('rigid_body_world_transform',np.eye(4)))
            if e['path'].startswith('/World/Piper'):
                local=np.linalg.inv(B)@T
                for m in meshes(e):self.robot.append(Shape(m,local,True,e['path'],body))
                if body in FINGERS:self.jaw_bounds[body]=np.array([np.min([s.vertices.min(0) for s in self.robot if s.body==body],axis=0),np.max([s.vertices.max(0) for s in self.robot if s.body==body],axis=0)])
            else:
                if body==self.moving_link:T=np.linalg.inv(B)@T;self.moving_reference=B
                for m in meshes(e):self.scene.append(Shape(m,T,True,e['path'],body))
        counts={n:sum(s.body==n for s in self.robot) for n in FINGERS}
        if counts!={n:1 for n in FINGERS}:raise RuntimeError('OFFICIAL_UNSPLIT_FINGER_REQUIRED:'+str(counts))
        if any('/contact_owned/' in s.path for s in self.robot):raise RuntimeError('OWNERSHIP_COLLIDERS_NOT_ALLOWED')

    def check(self,poses,moving_pose,allow_handle):
        for r in self.robot:r.place(np.asarray(poses[r.body]))
        for s in self.scene:s.place(moving_pose if s.body==self.moving_link else np.eye(4))
        for i,a in enumerate(self.robot):
            for b in self.robot[:i]:
                if a.body==b.body or frozenset((a.body,b.body)) in self.model.allowed:continue
                if intersects(a,b):return False,'SELF_COLLISION:'+a.body+':'+b.body
            for b in self.scene:
                if a.body in ('base_link','link1') and 'pedestal' in b.path:continue
                # Only handle primitive identity may admit distal finger contact.
                if allow_handle and a.body in FINGERS and b.path in self.allowed:continue
                if intersects(a,b):return False,'DANGEROUS_GEOMETRY_COLLISION:'+a.body+':'+b.path
        return True,'SAFE'

    def contact_guard(self,contacts,poses):
        for c in contacts:
            if c['force_n']<=0:continue
            target=self.entries.get(c['target']);other=target.get('rigid_body_path','').split('/')[-1] if target else None
            if other and frozenset((c['finger'],other)) in self.model.allowed:continue
            if c['finger'] in ('base_link','link1') and 'pedestal' in c['target']:continue
            if c['finger'] in FINGERS and c['target'] in self.allowed:
                p=c.get('contact_point_world_m')
                if p is None:return False,'MISSING_CONTACT_DIAGNOSTIC'
                local=(np.linalg.inv(np.asarray(poses[c['finger']]))@np.r_[p,1])[:3];bounds=self.jaw_bounds[c['finger']]
                # Broad distal inner half and holding edges, not four triangle IDs.
                distal=self.model.pads[c['finger']][0,1]
                if local[1]>=distal and local[2]<=(bounds[0,2]+bounds[1,2])/2:continue
                return False,'FINGER_BACK_OR_ROOT_HANDLE_LOAD:'+c['finger']
            # Loaded finger-panel/palm/table/support/arm contacts are never waived.
            return False,'DANGEROUS_LOADED_CONTACT:'+c['finger']+':'+c['target']
        return True,'TASK_CONTACT_ALLOWED'
