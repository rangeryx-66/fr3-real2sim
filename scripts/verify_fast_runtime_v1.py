"""Deterministic policy, motion, transport and lossless-export regressions; zero Isaac actors."""
import argparse, ast, hashlib, json, sys, tempfile, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import numpy as np
from interaction_runtime.policy import Policy,ForceMonitor,SafetySupervisor,SafetyStop,PHASES
from interaction_runtime.motion import CartesianReference
from interaction_runtime.evidence import Recorder,RenderSchedule

def verify(out):
    out.mkdir(parents=True,exist_ok=True);p=Policy.load(ROOT/'configs/interaction_runtime_v1.json');dt=1/240;checks={}
    def sample(phase,**kw):
        args=dict(phase=phase,q=[0.]*7,dq=[0.]*7,tcp_speed=0.,pads={'L':0.,'R':0.},dt=dt,t=0.,wrench=[0.]*6)
        args.update(kw);return args
    def supervisor():return SafetySupervisor(p,[(-3,3)]*7,[2.]*7,[20.]*7)
    a=supervisor()
    for phase in PHASES:
        speed=.008248790 if p.phase(phase)=='LOADED' else .005068036
        a.check(**sample(phase,tcp_speed=speed,pads={'L':2.0002655159,'R':0.}))
    checks['legacy_numbers_all_phases_nonfatal']=True
    for reason,args in [('COLLISION',dict(collision_ok=False,collision_reason='injected_native_pair')),('JOINT_MARGIN',dict(q=[2.99]+[0.]*6)),('JOINT_VELOCITY',dict(dq=[1.]+[0.]*6)),('ACTUATOR_EFFORT',dict(actuator_efforts=[21.]+[0.]*6)),('EMERGENCY_PAD_OVERLOAD',dict(pads={'L':10.,'R':0.})),('EMERGENCY_WRIST_FORCE',dict(wrench=[10.,0,0,0,0,0])),('PHASE_TCP_VELOCITY',dict(tcp_speed=1.))]:
        try:supervisor().check(**sample('LOADED',**args));raise AssertionError('Unsafe condition not rejected: '+reason)
        except SafetyStop as e:assert e.reason.startswith(reason),e.reason
    checks['fault_injections']='collision/joint/velocity/actuator/emergency pad/emergency wrist/phase speed stop'
    a=supervisor();stopped=None
    for i in range(100):
        try:a.check(**sample(['PREGRASP','APPROACH','PANEL_PUSH','RELEASE'][i%4],pads={'L':4.5,'R':0.},t=i*dt))
        except SafetyStop as e:stopped=(i,e.reason);break
    assert stopped and stopped[1]=='SUSTAINED_OVERLOAD' and stopped[0]>=34,stopped
    checks['sustained_history_crosses_modes_without_reset']=stopped
    ref=CartesianReference(np.eye(4));goal=np.eye(4);goal[0,3]=.04;goal[:3,:3]=__import__('scipy').spatial.transform.Rotation.from_rotvec([0,.3,0]).as_matrix();vel=[];prev=np.eye(4)
    for _ in range(2400):
        target=ref.advance(goal,dt,p.motion('LOADED'));vel.append(ref.velocity.copy());prev=target
    v=np.array(vel);assert np.linalg.norm(v[:,:3],axis=1).max()<=p.motion('LOADED')['speed_m_s']+1e-12
    assert np.linalg.norm(v[:,3:],axis=1).max()<=p.motion('LOADED')['angular_speed_rad_s']+1e-12
    assert np.linalg.norm(np.diff(v[:,:3],axis=0)/dt,axis=1).max()<=p.motion('LOADED')['acceleration_m_s2']+1e-10
    checks['independent_rotation_translation_and_acceleration']=True
    from interaction_runtime.runtime import Runtime
    stop=object.__new__(Runtime);stop.dt=dt;stop.arm=np.arange(7);stop.qtarget=np.zeros(7);stop.policy=p
    stop.g=dict(phase='PANEL_PUSH',qvelocity=np.array([.012,-.027,.004,0.,.009,-.015,.002]));samples=[stop.g['qvelocity'].copy()]
    stop.check_q=lambda q:(True,'isolated offline stop-command regression, not collision certification')
    stop.step=lambda:samples.append(stop.g['qvelocity'].copy());stop.stop_command()
    amax=float(np.max(abs(np.diff(samples,axis=0)))/dt)
    assert amax<=p.data['arm_command_acceleration_rad_s2']['LOADED']+1e-12 and np.max(abs(stop.g['qvelocity']))==0
    checks['actual_stop_command_joint_acceleration_rad_s2']=amax
    import inspect
    from interaction_runtime.operations import Operations
    push=inspect.getsource(Operations.push)
    rotation=inspect.getsource(Operations.rotate_segment)
    assert push.index('self.rotate_segment(')<push.index('finally:')
    import ast,textwrap
    assert 'self.r.stop_command()' in rotation and not any(isinstance(n,ast.ExceptHandler) for n in ast.walk(ast.parse(textwrap.dedent(rotation))))
    assert "self.g['qvelocity']=np.zeros(7)" not in push[push.index('finally:'):]
    checks['normal_panel_stop_precedes_separation_and_exceptions_do_not_continue']=True
    from types import SimpleNamespace
    trial=Operations.__new__(Operations)
    with tempfile.TemporaryDirectory() as folder:
        root=Path(folder);(root/'visual_plan.json').write_text(json.dumps(dict(section_fraction=.3,insertion_m=0.,roll_deg=180.,pitch_deg=0.)))
        trial.r=SimpleNamespace(job={},output=root,record=SimpleNamespace(count=1),supervisor=SimpleNamespace(latched=None))
        trial.g=dict(qvelocity=np.ones(7)*.01,contact_attempts=[]);trial.acquisitions=0
        def failed_acquisition():raise RuntimeError('offline recoverable local IK failure')
        trial.acquire=failed_acquisition
        def checked_release():
            assert np.all(trial.g['qvelocity']==.01),'Recoverable path discarded velocity before guarded stopping'
            return dict(safe=True)
        trial.release_park=checked_release
        assert trial.attempt(dict(segments=[]))['safe_exit']['safe']
    checks['recoverable_failure_preserves_velocity_for_guarded_release']=True
    # Every active source is compiled; old guards remain in frozen files only.
    sources={}
    for f in sorted((ROOT/'interaction_runtime').glob('*.py')):
        s=f.read_text();compile(s,str(f),'exec');sources[str(f.relative_to(ROOT))]=hashlib.sha256(s.encode()).hexdigest()
        assert 'TemporalForceGuard' not in s and 'PROBE_CARTESIAN_SPEED_LIMIT' not in s and 'EXISTING_SOFT_FORCE_WARNING_RETURN_FOR_UNLOAD' not in s,f
    checks['active_source_audit']=True
    schedule=RenderSchedule('FAST_DEV',p.data['modes']['FAST_DEV']);assert not any(schedule.due(i*dt) for i in range(240));schedule.event_clip(1.);assert any(schedule.due(1+i*dt) for i in range(240))
    checks['FAST_DEV_idle_render_requests']=0
    r=Recorder(out/'lossless_fixture',128);rows=[dict(t=i*dt,q=np.arange(13,dtype=np.float64),wrench=np.arange(6,dtype=np.float32),contacts=[dict(impulse=[.1,.2,.3],collider='native')],nan_optional=float('nan')) for i in range(513)]
    for row in rows:r.append(row)
    idx=r.close();import gzip,pickle
    restored=[]
    for c in idx['chunks']:
        with gzip.open(out/'lossless_fixture'/c['file'],'rb') as f:restored.extend(pickle.load(f))
    assert len(restored)==len(rows) and all(x['q'].tobytes()==y['q'].tobytes() and x['contacts']==y['contacts'] for x,y in zip(rows,restored))
    checks['chunked_native_numeric_contact_roundtrip_records']=len(restored)
    from verify_pull_push_protocol_v12 import verify as protocol
    checks['protocol']=protocol()
    result=dict(classification='OFFLINE_ENGINEERING_REGRESSION_NOT_PHYSICAL_VALIDATION',policy_sha256=p.sha256,checks=checks,sources=sources,physical_actors=0)
    (out/'OFFLINE_REGRESSION.json').write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();print(json.dumps(verify(a.output),indent=2))
