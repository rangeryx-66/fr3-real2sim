"""Conservative independent quality routing, without changing motion success."""
from contextlib import contextmanager
from unittest.mock import patch


@contextmanager
def selection_scope():
    import interaction_runtime.segment_quality as quality
    import update_probing_v7_model as model
    original=quality.selection
    def selection(segment,arrays,indices,enabled=False):
        if (segment['role'].startswith('FIT') and
                segment.get('result',{}).get('abc_feedback',{}).get('FIT_exclude_entire_segment')):
            return indices[:0],dict(label='ABC_NOMINAL_CONTACT_UNCERTAINTY_FIT_EXCLUDED',
                action_ok=bool(segment.get('result',{}).get('ok')),
                action_status=segment.get('result',{}).get('status'),
                operation_success_preserved=True)
        return original(segment,arrays,indices,enabled)
    with patch.object(quality,'selection',selection):
        # Some frozen versions import the function by alias.
        alias=getattr(model,'selection',None)
        if alias is original:
            with patch.object(model,'selection',selection):yield
        else:yield
