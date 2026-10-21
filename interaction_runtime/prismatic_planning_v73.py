"""Prismatic collision/reachability preflight using existing native predicates.

Native geometry validates safety only. Observed-geometry scores are unchanged.
No joint/force guard is relaxed, and no historical grasp is supplied.
"""
import numpy as np
from native_g1_execution_candidates import CandidateCheck


def representable_joint_bounds(limits,margin,dtype=np.float32):
    """Strict original guard, represented in the native command precision.

    Pink's inclusive .05 boundary can round OUTSIDE the runtime's strict .05
    predicate on assignment to the native float32 command buffer. Choose the
    first representable interior value, not another physical safety margin.
    """
    limits=np.asarray(limits);low=(limits[:,0]+margin).astype(dtype);high=(limits[:,1]-margin).astype(dtype)
    low=np.where(low.astype(float)<=limits[:,0]+margin,np.nextafter(low,dtype(np.inf)),low)
    high=np.where(high.astype(float)>=limits[:,1]-margin,np.nextafter(high,dtype(-np.inf)),high)
    return low.astype(float),high.astype(float)


def native_precision_ik_class():
    from cross_robot_adapter import RobotPinkIK
    class PrismaticNativePrecisionIK(RobotPinkIK):
        def __init__(self,*a,**kw):
            super().__init__(*a,**kw)
            low,high=representable_joint_bounds(self.original_limits,.05)
            self.robot.lowerPositionLimit=low;self.robot.upperPositionLimit=high
    return PrismaticNativePrecisionIK


def predicted_aperture(candidate):
    q=candidate.get('contact_geometry_audit',{})
    r=q.get('robust_contact',{})
    return float(r.get('chosen_expected_aperture_m',q.get('physical_aperture_m',.001641)))


def closing_envelope(g,q,aperture,policy):
    """Check the unchanged closure's permitted CENTER translation envelope.

    This is a conservative route check against prohibited native collisions,
    not a bilateral-force criterion. Authorized pad/handle contact stays legal.
    """
    model=g['model'];base=g['base'];q=np.asarray(q);P=model.poses(q,base,width=.084)
    centers=[(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in g['FINGERS']]
    axis=centers[1]-centers[0];axis/=np.linalg.norm(axis);bound=float(policy['max_centering_displacement_m'])
    target=P['tcp_link'];widths=np.linspace(.084,float(np.clip(aperture,.001641,.084)),41);audit=[]
    for offset in [0.,-bound,bound]:
        T=target.copy();T[:3,3]+=offset*axis;seed=q if offset==0 else g['solver'].solve(T,base,q)
        if seed is None:return dict(valid=False,reason='CENTER_ENVELOPE_IK',offset_m=offset,samples=audit)
        for width in widths:
            if model.margin(seed)<=g['runtime_policy'].data['joint_margin_rad']:
                return dict(valid=False,reason='CENTER_ENVELOPE_JOINT_MARGIN',offset_m=offset,samples=audit)
            ok,why=g['collision'].check(model.poses(seed,base,width=float(width)),g['moving'](),True)
            if not ok:return dict(valid=False,reason='NATIVE_CLOSING_ENVELOPE:'+str(why),offset_m=offset,aperture_m=float(width),samples=audit)
        audit.append(dict(offset_m=offset,checked_apertures=len(widths)))
    return dict(valid=True,samples=audit,minimum_checked_aperture_m=float(widths[-1]),centering_bound_m=bound,native_geometry_use='prohibited-collision safety only; not grasp scoring')


class PrismaticCandidateCheck(CandidateCheck):
    def __call__(self,candidate):
        result=super().__call__(candidate)
        if result['valid']:
            q=np.asarray(candidate['approach'][-1]);sweep=closing_envelope(self.g,q,predicted_aperture(candidate),self.g['policy'])
            result['closing_envelope']=sweep
            if not sweep['valid']:result.update(valid=False,reason=sweep['reason'])
            self.audit[-1].update(result)
            from agent_command_protocol_v12 import atomic_json
            atomic_json(self.g['a'].output/'EXECUTION_CANDIDATE_AUDIT.json',dict(candidates=self.audit,weights_changed=False,closing_center_envelope_checked=True,native_use='collision safety only'))
        return result


def reachable_interval(r,articulation,target,action):
    """Predict actual-state IK AND biased command feasibility, before loading.

    A rejected endpoint offers a stopped feasible prefix; it does not authorize
    an incomplete VALIDATION or equate a partial action to target completion.
    """
    start=r.actual();T=r.tcp().copy();M=r.moving().copy();j=np.asarray(r.robot.get_joint_positions());seed=j[r.arm].copy();fingers=j[r.fingers];bias=r.qtarget[r.arm]-seed
    speed=min(float(action.get('reference_speed_m_s',r.policy.motion('LOADED')['speed_m_s'])),r.policy.motion('LOADED')['speed_m_s'])
    acceleration=min(float(action.get('reference_acceleration_m_s2',r.policy.motion('LOADED')['acceleration_m_s2'])),r.policy.motion('LOADED')['acceleration_m_s2'])
    stop_distance=speed*speed/(2*acceleration)+speed*r.dt
    count=max(2,int(np.ceil(abs(target-start)/.001))+1);checked=[];reason=None
    for value in np.linspace(start,target,count):
        D=articulation.delta(value-start);seed=r.solver.solve(D@T,r.base,seed)
        if seed is None:reason='CURRENT_STATE_PREFIX_IK';break
        for mode,q in [('measured_kinematic',seed),('retained_command_bias',seed+bias)]:
            if r.model.margin(q)<=r.policy.data['joint_margin_rad']:reason='PREDICTED_'+mode.upper()+'_JOINT_MARGIN';break
            ok,why=r.collision.check(r.model.poses(q,r.base,finger_q=fingers),D@M,True)
            if not ok:reason='PREDICTED_'+mode.upper()+':'+why;break
        if reason:break
        checked.append(float(value))
    reached=bool(checked and abs(checked[-1]-target)<1e-9)
    direction=np.sign(target-start);prefix=checked[-1] if checked else start
    if not reached:
        wanted=prefix-direction*stop_distance
        prefix=max(start,wanted) if direction>0 else min(start,wanted)
    return dict(valid=reached,requested_target_q=float(target),actual_start_q=float(start),feasible_target_q=float(prefix),reason=reason,checked_states=len(checked),stopping_distance_m=stop_distance,command_bias_rad=bias.tolist(),native_coordinate_unit='m')
