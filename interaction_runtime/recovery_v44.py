"""Thin current-state planning activation; frozen manipulation and V3.1 reused."""
from .estimator_v31 import OperationsV31
from .probing_v2 import serve

class OperationsV44(OperationsV31):
    def __init__(self,r):
        super().__init__(r);self.g['execution_measured_fingers']=True
    def build_geometry(self):
        from native_g1_execution_candidates import context
        with context(self.g,self.r.policy,self.r.c,self.r.solver):return super().build_geometry()

def serve_v44(r):return serve(r,operations_factory=OperationsV44)
