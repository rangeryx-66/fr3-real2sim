"""Bounded physical observer diagnostic; not multistate/reconstruction success."""
import json
import numpy as np
from wrist_reconstruction.session import run as original


def run(runtime):
    r=runtime
    def observe_capture(label,value,*args,**kwargs):
        if label=='closed_after_real_grasp':
            (r.capture.output/'validation_scope.json').write_text(json.dumps({'mode':'physical release-observer validation','not_WRIST_CLEAN_dataset':True,'not_full_task_success':True,'uses_original_nominal_grasp':True,'initial_base_unchanged':True,'estimated_target_deg':r.policy['targets'][-1],'GT_online':False},indent=2))
            return
        recovery=r.recover.__self__
        guess=r.tcp()@np.linalg.inv(recovery.grasp_reference_ee)@recovery.grasp_reference_D
        result=recovery.release_failed_closure(guess)
        (r.capture.output/'release_validation.json').write_text(json.dumps({'scope':'actual contact, short estimated-model pull, release and retreat; NOT validation at 15 degrees','estimated_state_before_release':value,**result},indent=2))
        raise RuntimeError('PHYSICAL_RELEASE_OBSERVER_VALIDATION_COMPLETE')
    r.capture.capture=observe_capture
    return original(r)
