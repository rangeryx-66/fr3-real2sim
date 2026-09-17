"""Payload-only fresh stability gate; historical grasp outcomes remain shadow data.
No object classes, GT dynamics, pose adjustments, or estimator parameters.
"""
import numpy as np
from settling_gate import compute_metrics

def evaluate_start(monitor):
    old=monitor.query()
    result={'passed':False,'old_support':old,'reason':'SHORT_RECENT_WINDOW'}
    if not monitor.armed or not monitor.rows:return result
    end=monitor.rows[-1]['t']
    rows=[r for r in monitor.rows if r['t']>=end-.8001]
    if len(rows)<120 or rows[-1]['t']-rows[0]['t']<.79:return result
    if any(r['mesh_bottom_z_m']<=.002 for r in rows):
        result['reason']='TABLE_SUPPORT';return result
    if any(min(r['forces'])<=.1 for r in rows):
        result['reason']='CONTACT_LOSS';return result
    m=compute_metrics(rows);result['recent_metrics']=m
    if m['max_cumulative_translation_m']>.003 or m['max_cumulative_rotation_deg']>5.:
        result['reason']='RELATIVE_SLIP';return result
    for age in [0.,.1,.2,.3]:
        window=[r for r in rows if end-age-.2001<=r['t']<=end-age+1e-8]
        if len(window)<40:return result
        w=compute_metrics(window)['windows']['200ms']
        v=w['relative_translation_velocity_m_s'];a=w['relative_angular_velocity_deg_s']
        if not np.isfinite([v,a]).all():result['reason']='NONFINITE';return result
        if v>.00025 or a>.5:
            result['reason']='RECENT_MOTION';return result
    result.update(passed=True,reason='RECENT_FREE_SPACE_STABLE')
    return result
