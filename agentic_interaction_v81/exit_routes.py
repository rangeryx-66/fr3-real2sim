"""Original exit search with executed-timing checks and one unloaded replan.

Never reset a latch or continue manipulation. A coarse DIRECT result can be
invalid for the actual timed return; reject it while the existing joint exit
search still has other clearance/parking alternatives to consider.
"""
from contextlib import contextmanager
from unittest.mock import patch
import numpy as np


@contextmanager
def timed_returns(r):
    import native_g1_safe_park as owner
    from interaction_runtime.motion import timed_path
    from interaction_runtime.policy import SafetyStop
    original=owner.route_to_joint;history=[]
    def route(g,start,end):
        # Apply the same measured native envelope to DIRECT/RRT search and
        # timed verification. Never install this proxy as the runtime guard.
        from .exit_clearance import ParkingClearance
        scene=ParkingClearance(g['collision'])
        path,why=original(dict(g,collision=scene),start,end)
        record=dict(coarse_route=why,coarse_valid=path is not None)
        history.append(record)
        if path is None:return path,why
        measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
        actual_start=np.max(abs(np.asarray(start)-measured))<1e-9
        first=g['qtarget'][g['arm']].copy() if actual_start else np.asarray(start).copy()
        moving=np.asarray(g['moving']()).copy()
        fingers=np.asarray(g['robot'].get_joint_positions())[g['fingers']].copy()
        def checked(q):
            q=np.asarray(q,dtype=g['qtarget'].dtype)
            if g['model'].margin(q)<=r.policy.data['joint_margin_rad']:return False,'JOINT_MARGIN'
            return scene.check(g['model'].poses(q,g['base'],finger_q=fingers),moving,False)
        try:
            samples=timed_path(r.solver,r.base,[first]+path[1:],r.dt,
                r.policy.motion('SAFE_PARK_MOTION'),np.asarray(r.c['arm_speed_limits'])/3,
                r.policy.data['arm_command_acceleration_rad_s2'][r.policy.phase('SAFE_PARK_MOTION')],checked)
            count=0
            for q,_ in samples:
                ok,why=checked(q)
                if not ok:raise RuntimeError('EXECUTED_PARK_SAMPLE:'+why)
                count+=1
        except SafetyStop:raise
        except RuntimeError as error:
            record.update(timed_valid=False,reason=str(error),
                start_source='current equilibrium'if actual_start else'future clearance endpoint')
            return None,'TIMED_RETURN_ROUTE_UNAVAILABLE:'+str(error)
        record.update(timed_valid=True,timed_samples=count,
            start_source='current equilibrium'if actual_start else'future clearance endpoint')
        return path,why
    with patch.object(owner,'route_to_joint',route):yield history


def confirmed_unloaded(r):
    return bool(r.supervisor.latched is None and r.grasp_reference is None
        and r.rows and r.rows[-1]['aperture_m']>=.083
        and max(r.filtered())<=.05 and r.supervisor.force.settled())


def release_park(r,operation):
    """Only a planning rejection at a verified unloaded state is recoverable."""
    from interaction_runtime.policy import SafetyStop
    results=[]
    with timed_returns(r) as routes:
        for attempt in range(2):
            try:result=operation()
            except SafetyStop:raise  # original immediate protection is final
            except RuntimeError as error:
                if not str(error).startswith('MOTION_PATH_UNAVAILABLE:'):raise
                result=dict(safe=False,reason=str(error),preexecution_timed_route_rejected=True)
            results.append(dict(safe=result.get('safe'),reason=result.get('reason')))
            if result.get('safe') or attempt==1:break
            reason=str(result.get('reason',''))
            known_route_failure=(reason.startswith('MOTION_PATH_UNAVAILABLE:') or
                                 reason.startswith('TIMED_RETURN_ROUTE_UNAVAILABLE:'))
            if not known_route_failure or not confirmed_unloaded(r):break
            # No Agent inference, load-bearing reset or new object-driving move.
            # Original guarded deceleration and hold precede a new measured-state
            # clearance+parking search with its original bounded alternatives.
            r.event('V81_UNLOADED_EXIT_REPLAN',prior_reason=reason)
            r.stop_command();r.hold(.3)
        return dict(result,v81_exit_planning=dict(original_results=results,
            timed_routes=routes,maximum_local_replans=1,force_history_reset=False,
            manipulation_continued=False,original_guards_unchanged=True))
