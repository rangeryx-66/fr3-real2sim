"""Native contact-envelope clearance for unloaded parking route planning only.

FCL disjointness alone does not exclude positive PhysX hand contact reports.
Use the hand's exported envelopes for unloaded return; preserve the original
whole-body checks and all physical guards without adding a new mount policy.
"""
import numpy as np


def contact_offset(entry):
    values=entry.get('runtime_body_shapes',{}).get('contact_offsets')
    if values is None:
        authored=entry.get('offsets',{}).get('contactOffset',{})
        values=[authored['schema_value']] if authored.get('authored') else None
    if values is None:raise ValueError('NATIVE_CONTACT_OFFSET_REQUIRED:'+entry['path'])
    values=np.asarray(values,float)
    if not values.size or not np.isfinite(values).all() or np.any(values<0):
        raise ValueError('INVALID_NATIVE_CONTACT_OFFSET:'+entry['path'])
    # Body tensors do not establish shape order: use their conservative maximum.
    return float(values.max())


class ParkingClearance:
    def __init__(self,scene):
        self.original=scene
        self.inner=getattr(scene,'inner',scene)
        self.hand=[s for s in self.inner.robot
            if s.body in ('gripper_link1','gripper_link2','xarm_gripper_base_link')]
        self.offsets={s.path:contact_offset(self.inner.entries[s.path])
            for s in self.hand+self.inner.scene}

    def __getattr__(self,name):return getattr(self.original,name)

    def check(self,poses,moving,allow_handle):
        valid,why=self.original.check(poses,moving,allow_handle)
        if not valid or allow_handle:return valid,why
        import fcl
        def separated(a,b):
            margin=self.offsets[a.path]+self.offsets[b.path]
            if np.any(a.bounds[0]>b.bounds[1]+margin) or np.any(b.bounds[0]>a.bounds[1]+margin):return True
            return fcl.distance(a.object,b.object)>margin
        for a in self.hand:
            for b in self.inner.scene:
                if not separated(a,b):return False,'NATIVE_PARK_CONTACT_ENVELOPE:'+a.body+':'+b.path
        return True,'NATIVE_PARK_CONTACT_CLEARANCE'
