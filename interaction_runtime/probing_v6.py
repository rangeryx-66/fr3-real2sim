"""V6 passive sensors + local estimator. Frozen robot operations inherited."""
from .probing_v5 import RuntimeV5,OperationsV5
from .probing_v2 import serve
from xarm_ft_v6_estimator import VERSION

class RuntimeV6(RuntimeV5):
    def __init__(self,job):
        super().__init__(job)
        from .passive_rgbd_v6 import PassiveRGBDPair
        self._passive_writer=PassiveRGBDPair(self)
        assert not any('passive' in k or 'fixed_rgbd' in k for k in self.g if k!='job')
    def step(self):
        row=super().step()
        if getattr(self,'_passive_writer',None):self._passive_writer.after_step()
        return row
    def close(self):
        if getattr(self,'_passive_writer',None):self._passive_writer.close()
        super().close()

class OperationsV6(OperationsV5):
    worker_script='scripts/update_probing_v6_model.py'
    worker_version=VERSION

def serve_v6(r):return serve(r,operations_factory=OperationsV6)
