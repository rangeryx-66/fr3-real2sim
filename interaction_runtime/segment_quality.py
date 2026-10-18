"""Selection metadata only; never changes action success or V3.1 mathematics."""
import numpy as np


def interval_indices(segment,arrays,indices,windows,valid_until=float('inf')):
    relative=(arrays['step'][indices].astype(np.float64)-segment['first_step'])/240;keep=np.zeros(len(indices),bool)
    for lo,hi in windows:keep|=(relative>=lo)&(relative<=min(hi,valid_until))
    keep&=arrays['safety_stop'][indices]==0
    return indices[keep]


def selection(segment,arrays,indices,enabled=False):
    result=segment.get('result',{});action=segment.get('declared_action',{});windows=action.get('fit_intervals_s',[])
    original=dict(action_ok=bool(result.get('ok')),action_status=result.get('status',result.get('reason')))
    if segment.get('completed') and result.get('ok'):
        if segment['role'].startswith('VALIDATION') and not result.get('requested_target_reached',True):
            return indices[:0],dict(original,label='INCOMPLETE_VALIDATION_REQUESTED_TARGET_NOT_REACHED')
        if enabled and segment['role'].startswith('FIT') and windows:
            return interval_indices(segment,arrays,indices,windows),dict(original,label='COMPLETE_ACTION_DECLARED_FIT_INTERVALS',declared_intervals_s=windows)
        return indices,dict(original,label='COMPLETE_ACTION')
    if not enabled:return indices[:0],dict(original,label='INCOMPLETE_EXCLUDED_LEGACY')
    if not segment['role'].startswith('FIT'):
        return indices[:0],dict(original,label='INCOMPLETE_VALIDATION_EXCLUDED')
    if not (segment.get('completed') and result.get('status')=='TIMEOUT' and result.get('safely_stopped') and result.get('physical_transmission_valid') and windows):
        return indices[:0],dict(original,label='INTERRUPTED_FIT_NOT_QUALIFIED')
    # Invalid samples/gaps are split before centered filtering. Informational
    # warnings or one missing pad report alone do not invalidate transmission.
    return interval_indices(segment,arrays,indices,windows,result['fit_valid_until_s']),dict(original,label='PREDECLARED_SAFE_INTERRUPTED_FIT',declared_intervals_s=windows,action_remains_failed=True)
