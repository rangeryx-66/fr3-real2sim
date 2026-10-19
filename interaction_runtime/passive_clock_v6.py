"""Renderer rational-clock synchronization; world float accumulation is diagnostic."""
import math

def check_pair(cameras,tick,world_s,origin_s=None,previous_s=None,dt=1/240):
    refs=[c.get('reference_time',{}) for c in cameras]
    equal=len(refs)==2 and refs[0]==refs[1]
    valid=equal and refs[0].get('referenceTimeDenominator',0)>0
    if not valid:return dict(synchronized=False,reason='MISSING_OR_MISMATCHED_RENDER_REFERENCE')
    ref=refs[0];den=ref['referenceTimeDenominator'];seconds=ref['referenceTimeNumerator']/den
    if origin_s is None:origin_s=seconds-tick*dt
    residual=seconds-(origin_s+tick*dt)
    # Quantized rational timestamps can differ from nominal ticks by <=one
    # quantum. No world-time absolute epsilon: that clock accumulates float32 dt.
    tolerance=2/den+4*math.ulp(max(abs(seconds),1.))
    fresh=previous_s is None or seconds>previous_s
    return dict(synchronized=bool(abs(residual)<=tolerance and fresh),reference_timestamp_s=seconds,reference_physics_origin_s=origin_s,
                reference_minus_nominal_tick_s=residual,tolerance_s=tolerance,world_clock_difference_s=seconds-world_s,
                reason=None if abs(residual)<=tolerance and fresh else 'STALE_OR_WRONG_PHYSICS_TICK')
