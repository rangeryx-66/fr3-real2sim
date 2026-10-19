"""Semantic proxy: native ownership accepts contact; raw intersections diagnose."""
import numpy as np,fcl
from piper_mobile_demo.owned_scene import OwnedFingerScene,intersects,arm_check

class SemanticScene(OwnedFingerScene):
    def __init__(self,*args,calibration=None,**kwargs):
        super().__init__(*args,**kwargs);self.offsets={}
        for e in self.data['shapes']:
            values=np.asarray(e.get('runtime_body_shapes',{}).get('contact_offsets',[])).ravel();values=values[np.isfinite(values)&(values>=0)]
            authored=e['offsets']['contactOffset']['schema_value']
            if authored is not None and authored>=0:values=np.r_[values,authored]
            self.offsets[e['path']]=float(values.max()) if len(values) else .002
    def check(self,poses,moving_pose=None,allow_pad=True):
        self.place_scene(self.moving_reference if moving_pose is None else moving_pose)
        for a in self.fingers:
            a.place(np.asarray(poses[a.body]))
            if any(a.bounds[0,2]<=z for z in self.planes):return False,'FLOOR_COLLISION:'+a.path
            if allow_pad:continue # PhysX actual impulse/ownership gates execution
            for b in self.scene:
                envelope=self.offsets.get(a.path,.002)+self.offsets.get(b.path,.002)
                if np.any(a.bounds[0]-envelope>b.bounds[1]) or np.any(b.bounds[0]>a.bounds[1]+envelope):continue
                if fcl.distance(a.object,b.object)<=envelope:return False,'NATIVE_APPROACH_CONTACT_ENVELOPE:'+a.path+':'+b.path
        return True,'SEMANTIC_PROXY_NATIVE_CONTACT_REQUIRED'

    def check_self(self,poses,allowed):
        groups={}
        for a in self.fingers:
            a.place(np.asarray(poses[a.body]));groups.setdefault(a.body,[]).append(a)
        bounds={n:(np.min([a.bounds[0] for a in g],axis=0),np.max([a.bounds[1] for a in g],axis=0)) for n,g in groups.items()}
        for b in self.other_robot:
            b.place(np.asarray(poses[b.body]))
            for n,g in groups.items():
                if frozenset((n,b.body)) in allowed:continue
                lo,hi=bounds[n]
                if np.any(lo>b.bounds[1]) or np.any(b.bounds[0]>hi):continue
                for a in g:
                    if intersects(a,b):return False,'OWNED_SELF_COLLISION:'+a.path+':'+b.path
        return True,'OWNED_SELF_SAFE'
    def check_robot(self,model,q,base,angle=0.,finger_q=None,allow_pad=True):
        ok,why,_=arm_check(model,q,base,angle,finger_q)
        if not ok:return False,why
        poses=model.poses(q,base,finger_q=finger_q)
        initial=model.asset_T@model.asset.root_to_link(self.moving_link,{model.manifest['joint_name']:self.reference_angle_rad})
        moving=model.asset_T@model.asset.root_to_link(self.moving_link,{model.manifest['joint_name']:angle})
        ok,why=self.check(poses,moving@np.linalg.inv(initial)@self.moving_reference,allow_pad)
        if not ok:return False,why
        return self.check_self(poses,model.allowed)
