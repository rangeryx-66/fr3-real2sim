"""Thin V5 sensor/estimator activation; V4.4 motion and safety inherited."""
import hashlib,json,subprocess,sys,time
from agent_command_protocol_v12 import atomic_json
from .serialization import result_packet
from .probing_v3 import RuntimeV3
from .recovery_v44 import OperationsV44
from .probing_v2 import serve

class RuntimeV5(RuntimeV3):
    def __init__(self,job):
        super().__init__(job)
        from xarm_ft_v1_scene import install_native_law
        # Existing per-step seam runs before physics; no object effort/target.
        if self.engineering_before_physics is not None:raise RuntimeError('passive plant owner conflict')
        self.engineering_before_physics=install_native_law(self.g)

class OperationsV5(OperationsV44):
    def _update(self,ready):
        started=time.perf_counter();folder=self.r.output/'model';folder.mkdir(exist_ok=True)
        if not ready.wait(30):return dict(error='numeric export not ready')
        if not (self.r.output/'identification_input.json').exists():return dict(error='no interaction prior yet')
        source=self.r.root/getattr(self,'worker_script','scripts/update_probing_v5_model.py');status=dict(worker_script=str(source),worker_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
        try:
            command=[sys.executable,str(source),str(self.r.output)]
            if self.r.job.get('allow_interrupted_fit'):command.append('--allow-interrupted-fit')
            with (folder/'worker.log').open('w') as log:p=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=60)
            from xarm_ft_v5_estimator import VERSION
            r=json.loads((folder/'estimate.json').read_text()) if p.returncode==0 else {}
            status.update(returncode=p.returncode,estimator_ready=r.get('estimator_version')==getattr(self,'worker_version',VERSION),loaded_sources=r.get('source_sha256',{}))
        except Exception as e:status.update(error=repr(e),estimator_ready=False)
        status['wall_s']=time.perf_counter()-started;atomic_json(folder/'WORKER_STATUS.json',result_packet(status));return status

def serve_v5(r):return serve(r,operations_factory=OperationsV5)
