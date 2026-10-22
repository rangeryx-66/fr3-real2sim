"""Experimental rail-orthogonal unloading, explicitly selected while unloaded.

Not a jaw servo, force-limit change, integral IK, or a symmetric-load gate.
Only the measured jaw-centre axis perpendicular to the known rail is moved.
The total closure + correction stays in the original centering ball. Each
trailing-load window permits at most one original centering step. All original
IK, collision, command-acceleration, force and finite-budget checks still own
execution. This module must not be imported by the frozen revolute runner.
"""
import numpy as np


class TransverseUnloading:
    def __init__(self, policy, rail, period_steps, closure_offset):
        self.policy=policy;self.rail=np.asarray(rail,float).copy()
        self.rail/=np.linalg.norm(self.rail)
        self.period=max(1,int(period_steps));self.last=None
        self.initial=np.asarray(closure_offset,float).copy();self.offset=np.zeros(3)

    def update(self,tick,dt,force_robot_on_object_world,pad_centres):
        force=np.asarray(force_robot_on_object_world,float)
        centres=np.asarray(pad_centres,float)
        if force.shape!=(3,)or centres.shape!=(2,3)or not np.isfinite(force).all()or not np.isfinite(centres).all():
            raise ValueError('INVALID_TRANSVERSE_UNLOADING_MEASUREMENT')
        if self.last is not None and tick-self.last<self.period:return None
        self.last=int(tick)
        axis=centres[0]-centres[1];axis-=self.rail*(axis@self.rail)
        if np.linalg.norm(axis)<1e-9:return dict(applied=False,reason='NO_RAIL_TRANSVERSE_JAW_AXIS')
        axis/=np.linalg.norm(axis);load=float(force@axis)
        # Same centering proportional slope as the frozen closure. Apply only
        # one finite step per existing causal load window, rather than stacking
        # delayed corrections every native step. No pad-zero failure rule.
        speed=np.clip(-.003*load,-self.policy['max_centering_speed_m_s'],self.policy['max_centering_speed_m_s'])
        distance=float(np.clip(speed*self.period*dt,-self.policy['max_centering_step_m'],self.policy['max_centering_step_m']))
        delta=axis*distance;current=self.initial+self.offset
        bound=self.policy['max_centering_displacement_m']
        aa=float(delta@delta);bb=float(current@delta);cc=float(current@current-bound**2)
        fraction=0. if cc>1e-15 else (float(np.clip((-bb+np.sqrt(max(0.,bb*bb-aa*cc)))/aa,0,1))if aa>0 else 0.)
        delta*=fraction;self.offset+=delta
        return dict(applied=bool(np.linalg.norm(delta)>1e-12),net_jaw_axis_object_load_N=load,
                    step_world_m=delta.tolist(),offset_world_m=self.offset.tolist(),
                    total_closure_and_unloading_world_m=(self.initial+self.offset).tolist(),
                    rail_component_m=float(self.offset@self.rail),jaw_command_changed=False)


def causal_world_wrench(row,bias):
    f=row['wrist_ft'];R=np.asarray(f['T_world_sensor'])[:3,:3]
    raw=np.asarray(f['raw_wrench_sensor'])-np.asarray(bias)
    return np.r_[R@raw[:3],R@raw[3:]]+np.asarray(f['tool_gravity_wrench_world'])
