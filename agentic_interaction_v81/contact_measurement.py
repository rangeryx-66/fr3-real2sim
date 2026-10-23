"""Passive full rigid-motion contact diagnostic; no control/protection input."""
import numpy as np
from .contact_diagnosis import point_velocity


def relative_contact_speed(previous_fingers,fingers,previous_body,body,contacts,dt):
    if not np.isfinite(dt) or abs(dt-1/240)>1e-7:return None
    speeds=[]
    for c in contacts:
        side=c.get('finger')
        if (side not in previous_fingers or side not in fingers
                or not c.get('allowed_pad_target') or c.get('force_n',0)<=0):continue
        p=np.asarray(c.get('contact_point_world_m',[np.nan]*3))
        n=np.asarray(c.get('native_normal_world',[np.nan]*3))
        if not np.isfinite(np.r_[p,n]).all() or np.linalg.norm(n)<1e-9:continue
        n=n/np.linalg.norm(n)
        v=point_velocity(previous_fingers[side],fingers[side],p,dt)-point_velocity(previous_body,body,p,dt)
        speeds.append(float(np.linalg.norm(v-n*(v@n))))
    return float(np.median(speeds)) if speeds else None


def body_pose_field(previous,row):
    from interaction_runtime.compact_v2 import pose7
    values=previous(row)
    if 'moving_link_pose' not in values:values['moving_link_pose']=pose7(row['T_moving_link'])
    return values
