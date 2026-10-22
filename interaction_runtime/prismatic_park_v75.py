"""Isolated equivalent-angle parking adapter; native checks remain authoritative.

This is a development candidate until archived native geometry and actual
release/parking have been validated. No revolute caller imports this module.
"""
from unittest.mock import patch
import numpy as np
from .prismatic_lifecycle_v75 import equivalent_joint_targets


def parking_targets(g, original):
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    # Explicit actual URDF joint types are supplied by the isolated runtime.
    periodic=g['v75_periodic_arm_indices']
    speed=np.asarray(g['v75_arm_speed_limits'])/3
    rows=[]
    for region,q in original(g):
        for row in equivalent_joint_targets(q,measured,g['model'].limits,periodic,.05,speed):
            rows.append((row['joint_travel_lower_bound_s'],region,row))
    for _,region,row in sorted(rows,key=lambda x:x[0]):
        name=region+'/winding['+','.join(str(k) for k in row['winding'])+']'
        yield name,np.asarray(row['q'])


def plan_exit(g):
    import native_g1_safe_park as frozen
    original=frozen.parking_targets
    with patch.object(frozen,'parking_targets',lambda state:parking_targets(state,original)):
        result=frozen.plan_exit(g)
    result['definition']=dict(result['definition'],
        regions='original native/shoulder rest cells, including legal equivalent revolute joint windings',
        endpoint_equivalence_is_not_route_approval=True)
    result['prismatic_v75_winding_adapter']=True
    return result
