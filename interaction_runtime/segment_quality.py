"""Selection metadata only; never changes action success or V3.1 mathematics."""
import numpy as np


def selection(segment,arrays,indices,enabled=False):
    result=segment.get('result',{});action=segment.get('declared_action',{})
    original=dict(action_ok=bool(result.get('ok')),action_status=result.get('status',result.get('reason')))
    if segment.get('completed') and result.get('ok'):
        return indices,dict(original,label='COMPLETE_ACTION')
    if not enabled:return indices[:0],dict(original,label='INCOMPLETE_EXCLUDED_LEGACY')
    if not segment['role'].startswith('FIT'):
        return indices[:0],dict(original,label='INCOMPLETE_VALIDATION_EXCLUDED')
    windows=action.get('fit_intervals_s',[])
    if not (segment.get('completed') and result.get('status')=='TIMEOUT' and result.get('safely_stopped') and result.get('physical_transmission_valid') and windows):
        return indices[:0],dict(original,label='INTERRUPTED_FIT_NOT_QUALIFIED')
    relative=arrays['t'][indices]-segment['first_step']/240
    keep=np.zeros(len(indices),bool)
    for lo,hi in windows:keep|=(relative>=lo)&(relative<=min(hi,result['fit_valid_until_s']))
    # Invalid samples/gaps are split before any centered filtering. Informational
    # warnings or one missing pad report alone do not invalidate transmission.
    keep&=arrays['safety_stop'][indices]==0
    return indices[keep],dict(original,label='PREDECLARED_SAFE_INTERRUPTED_FIT',declared_intervals_s=windows,action_remains_failed=True)
