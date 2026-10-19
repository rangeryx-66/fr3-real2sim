"""Same URDF transforms, evaluated once per joint instead of once per link ancestor."""
import numpy as np
from xarm7_native_planning import NativeModel,GRIPPER_JOINTS,FINGER_BODIES,LOGICAL,PAD_FRAMES
from articulated_demo.kinematics import transform

class FastNativeModel(NativeModel):
    def poses(self,q,base,width=.084,finger_q=None):
        values=dict(zip(self.names,q));values.update(self.native.values(self.native.angle(width)))
        if finger_q is not None:values.update(dict(zip(GRIPPER_JOINTS,np.asarray(finger_q))))
        cache={self.chain.root:np.eye(4)}
        def local(n):
            if n not in cache:
                j=self.chain.joints[n];cache[n]=local(j.parent)@j.pose(values.get(j.name,0.))
            return cache[n]
        B=transform(base[:3],[0,0,np.deg2rad(base[3])]);P={n:B@local(n) for n in self.physical_hulls};P['tcp_link']=B@local('tcp_link')
        for n,l in zip(FINGER_BODIES,LOGICAL):P[l]=P[n]@PAD_FRAMES[n]
        return P
