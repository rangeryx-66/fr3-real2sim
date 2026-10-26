"""Verified no-motion boundaries and typed controlled interruptions."""
import numpy as np


PREFIX='CONTROLLED_EXECUTION_INTERRUPT:'


def current_released_hold(ops):
    from agentic_interaction_v81.recovery_supervisor import released,trusted
    r=ops.r
    if (not trusted(ops) or r.grasp_reference is not None or ops.g.get('reference') is not None
            or r.rows[-1]['aperture_m']<.083
            or max(r.filtered())>r.policy.data['force']['unloaded_pad_n']):return False
    try:
        if np.max(abs(r.g['qvelocity']))>1e-10:return False
        q=np.asarray(r.robot.get_joint_positions());dq=np.asarray(r.robot.get_joint_velocities())
        if not np.isfinite(np.r_[q,dq]).all():return False
        ok,_=r.collision.check(r.model.poses(q[r.arm],r.base,finger_q=q[r.fingers]),r.moving(),False)
        if not ok:return False
        if released(ops) and np.max(abs(dq[r.arm]))<.01:return True
        # A single native velocity sample or an unsettled unloaded filter is
        # not evidence that a rejected plan needs a physical parking cycle.
        # Keep the current command; use the ORIGINAL supervised settling
        # duration and measured stationarity test. Failure still falls back
        # to the existing stop/unload/park owner. Hard safety propagates.
        r.hold(.3)
        measured=np.asarray(r.robot.get_joint_positions())[r.arm]
        return bool(released(ops) and r.station_motion.ready(r.rows[-1]['t'],measured))
    except (AttributeError,KeyError,TypeError,ValueError):return False


def invalidate(ops):
    ops.current_plan_ready=False
    if hasattr(ops,'_v81_plan_ticket'):ops._v81_plan_ticket=None
    ops.cache.clear()
    if hasattr(ops,'abc_state'):ops.abc_state='A';ops._abc_retain=False


def annotate_interrupted_end(end):
    def record(result=None):
        import sys
        error=sys.exc_info()[1]
        if isinstance(error,RuntimeError) and str(error).startswith(PREFIX):
            result=dict(ok=False,status='RECOVERABLE_FAILURE',reason=str(error),
                physical_transmission_valid=False,fit_prefix_transmission_valid=False,
                requested_target_reached=False,complete_independent_validation=False,
                FIT_exclude_entire_segment=True)
        return end(result)
    return record
