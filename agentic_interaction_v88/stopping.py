"""Read-only preview of the existing coupled stop, before submitting motion.

The actual stop controller and independent native hard guards are unchanged.
Checking only TCP displacement can miss cancelling base/arm motion near a wall.
"""
from copy import copy
import numpy as np


def check_stop(q, velocity, governor, speeds, acceleration, motion, check):
    preview=copy(governor)
    preview.previous_twist=governor.previous_twist.copy()
    q=np.asarray(q,float).copy();velocity=np.asarray(velocity,float).copy()
    count=0
    for _ in range(int(5/preview.dt)+1):
        valid,why=check(q)
        count+=1
        if not valid:return dict(valid=False,reason=str(why),checked=count)
        if np.max(abs(velocity))<=1e-12:return dict(valid=True,checked=count)
        try:velocity=preview.velocity(q,np.zeros_like(velocity),velocity,speeds,acceleration,motion)
        except RuntimeError as error:
            if not str(error).startswith('CONTROLLED_EXECUTION_INTERRUPT:'):raise
            return dict(valid=False,reason='NUMERICAL_STOP_PREVIEW_UNAVAILABLE:'+str(error),checked=count)
        q+=velocity*preview.dt
    return dict(valid=False,reason='COUPLED_CONTROLLED_STOP_NOT_VERIFIED',checked=count)
