"""Scope restoration; numerical identity is a separate robot-URDF check."""
import sys,unittest
from types import ModuleType
from unittest.mock import patch
from agentic_interaction_v81.performance import cached_planning_fk


class PerformanceScopeTests(unittest.TestCase):
    def test_original_owner_restored_after_success_and_exception(self):
        class Original:pass
        class Cached(Original):pass
        native=ModuleType('xarm7_native_planning');native.NativeModel=Original
        fast=ModuleType('interaction_runtime.kinematics');fast.FastNativeModel=Cached
        with patch.dict(sys.modules,{'xarm7_native_planning':native,'interaction_runtime.kinematics':fast}):
            with cached_planning_fk():self.assertIs(native.NativeModel,Cached)
            self.assertIs(native.NativeModel,Original)
            with self.assertRaises(ValueError):
                with cached_planning_fk():raise ValueError('candidate rejected')
            self.assertIs(native.NativeModel,Original)

    def test_unrelated_model_owner_rejected(self):
        native=ModuleType('xarm7_native_planning');native.NativeModel=type('Other',(),{})
        fast=ModuleType('interaction_runtime.kinematics');fast.FastNativeModel=type('Cached',(),{})
        with patch.dict(sys.modules,{'xarm7_native_planning':native,'interaction_runtime.kinematics':fast}):
            with self.assertRaises(RuntimeError):
                with cached_planning_fk():pass


if __name__=='__main__':unittest.main()
