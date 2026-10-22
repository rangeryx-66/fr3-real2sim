"""Scoped use of the already validated, identical robot FK implementation.

No IK, collision, grasp score, random generator or controller is replaced.
The original planner constructs the same model, with ancestor products cached
within one poses() call rather than recomputed separately for every body.
"""
from contextlib import contextmanager
from unittest.mock import patch


@contextmanager
def cached_planning_fk():
    import xarm7_native_planning as native
    from interaction_runtime.kinematics import FastNativeModel
    if not issubclass(FastNativeModel, native.NativeModel):
        raise RuntimeError('UNEXPECTED_NATIVE_MODEL_OWNER')
    with patch.object(native, 'NativeModel', FastNativeModel):
        yield
