"""Estimator-only V3.1 integration. Frozen V3 motion/Agent operations inherited."""
import hashlib,json,subprocess,sys,time
from agent_command_protocol_v12 import atomic_json
from .probing_v3 import OperationsV3
from .probing_v2 import serve

WORKER_SCRIPT='scripts/update_probing_v31_model.py'
ESTIMATOR_VERSION='V3.1-common-SG2'
WORKER_TIMEOUT_S=60
EXPORT_TIMEOUT_S=30


class OperationsV31(OperationsV3):
    def _update(self,ready):
        began=time.perf_counter();folder=self.r.output/'model';folder.mkdir(exist_ok=True)
        if not ready.wait(EXPORT_TIMEOUT_S):return dict(error='numeric packet export not ready')
        if not (self.r.output/'identification_input.json').exists():return dict(error='no interaction prior yet')
        source=self.r.root/WORKER_SCRIPT
        status=dict(estimator_version=ESTIMATOR_VERSION,worker_script=WORKER_SCRIPT,worker_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),timeout_s=WORKER_TIMEOUT_S)
        try:
            with (folder/'worker.log').open('w') as log:
                command=[sys.executable,str(source),str(self.r.output)]
                if getattr(self.r,'job',{}).get('allow_interrupted_fit',False):command.append('--allow-interrupted-fit')
                status['command']=command
                child=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=WORKER_TIMEOUT_S)
            status['returncode']=child.returncode
            result=json.loads((folder/'estimate.json').read_text()) if child.returncode==0 else {}
            status['estimator_ready']=child.returncode==0 and result.get('estimator_version')==ESTIMATOR_VERSION
            if child.returncode==0 and not status['estimator_ready']:status['error']='unexpected estimator version; no legacy fallback'
            status['loaded_sources']=result.get('source_sha256',{})
        except Exception as error:status.update(error=repr(error),estimator_ready=False)
        status['wall_s']=time.perf_counter()-began;atomic_json(folder/'WORKER_STATUS.json',status);return status


def serve_v31(runtime):return serve(runtime,operations_factory=OperationsV31)
