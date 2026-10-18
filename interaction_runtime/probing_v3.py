"""V3 excitation and passive calibration seams; frozen manipulation reused."""
import subprocess,sys,time
from pathlib import Path
from agent_command_protocol_v12 import atomic_json
from .probing_v2 import ProbingRuntime,ProbingOperations,serve
from .compact_v2 import CompactRecorder
from .probing_profiles_v3 import speed_fraction

class RecorderV3(CompactRecorder):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.calibration=False
        self.metadata['unloaded_calibration']='240 Hz during existing final approach; no new calibration motion'
    def append(self,row):
        super().append(row)
        if self.calibration and self.pre:self.pre[-1][1]=True
    def begin(self,action,mode):
        super().begin(action,mode);self.segments[-1]['declared_action']=action.copy()

class RuntimeV3(ProbingRuntime):
    def __init__(self,job):super().__init__(job,record_factory=RecorderV3)
    def motion_limits(self,phase):
        m=dict(self.policy.motion(phase));a=self.probe_profile
        if a and self.policy.phase(phase)=='LOADED':
            m['speed_m_s']=min(m['speed_m_s'],float(a.get('reference_speed_m_s',m['speed_m_s']))*speed_fraction((self.tick-self.profile_started)*self.dt,a))
            m['acceleration_m_s2']=min(m['acceleration_m_s2'],float(a.get('reference_acceleration_m_s2',m['acceleration_m_s2'])))
        return m
    def move_path(self,path,phase):
        # Keep one numeric copy of an already executed unloaded near-grasp move.
        self.record.calibration=phase=='APPROACH'
        try:return super().move_path(path,phase)
        finally:self.record.calibration=False

class OperationsV3(ProbingOperations):
    def _update(self,ready):
        began=time.perf_counter();folder=self.r.output/'model';folder.mkdir(exist_ok=True)
        if not ready.wait(30):return dict(error='numeric packet export not ready')
        if not (self.r.output/'identification_input.json').exists():return dict(error='no interaction prior yet')
        try:
            with (folder/'worker.log').open('w') as log:
                p=subprocess.run([sys.executable,str(self.r.root/'scripts/update_probing_v3_model.py'),str(self.r.output)],stdout=log,stderr=subprocess.STDOUT,timeout=60)
            status=dict(returncode=p.returncode,wall_s=time.perf_counter()-began)
        except Exception as e:status=dict(error=repr(e),wall_s=time.perf_counter()-began)
        atomic_json(folder/'WORKER_STATUS.json',status);return status

def serve_v3(r):return serve(r,operations_factory=OperationsV3)
