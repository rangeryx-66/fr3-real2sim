"""Verified no-motion boundaries and typed controlled interruptions."""
import numpy as np


PREFIX='CONTROLLED_EXECUTION_INTERRUPT:'


def current_released_hold(ops):
    from agentic_interaction_v81.recovery_supervisor import released
    r=ops.r
    if not released(ops):return False
    try:
        if np.max(abs(r.g['qvelocity']))>1e-10:return False
        q=np.asarray(r.robot.get_joint_positions());dq=np.asarray(r.robot.get_joint_velocities())
        if not np.isfinite(np.r_[q,dq]).all() or np.max(abs(dq[r.arm]))>=.01:return False
        ok,_=r.collision.check(r.model.poses(q[r.arm],r.base,finger_q=q[r.fingers]),r.moving(),False)
        return bool(ok)
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
