"""Deterministic production-contract regressions, no Isaac or hidden dynamics."""
import ast,hashlib,json,math,sys,tempfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.segments import *
from interaction_runtime.policy import Policy
from interaction_runtime.segment_quality import selection
from interaction_runtime.checkpoints import allocate
from agent_command_protocol_v12 import validate,VERSION


def verify():
    assert not slip_stall_confirmed(8,math.radians(.041))
    assert slip_stall_confirmed(3,math.radians(.499))
    motion=Policy.load(ROOT/'configs/interaction_runtime_v1.json').motion('LOADED');dt=1/240
    action=dict(target_deg=15.,role='FIT_SLOW',profile='constant',hold=True,reference_speed_m_s=.001,reference_acceleration_m_s2=.01,fit_intervals_s=[[1.,80.]])
    validate(dict(protocol_version=VERSION,sequence=0,operation='attempt_grasp_and_probe',segments=[action]),0)
    plan=segment_plan(0,math.radians(15),action,motion,.425,dt,dict(wall_s=1000,sim_s=800))
    assert plan['nominal_duration_s']>45 and plan['valid'] and plan['max_sim_s']==800
    clock=ProgressClock(0,math.radians(15));monitor=TransmissionMonitor(0,1,action);q=0.;events=[]
    for i in range(30000):
        ref,state=clock.advance(q,.001,.425,dt,.08,.01,.2);q=ref
        assert monitor.update(i*dt,q,state,dt,clock.target-q) is None
        if q>=clock.target:break
    healthy_s=i*dt;assert healthy_s>45
    clock=ProgressClock(0,.3);monitor=TransmissionMonitor(0,1,{})
    for i in range(10000):
        _,state=clock.advance(0,.008,.425,dt,.08,.04,.2)
        if monitor.update(i*dt,0,state,dt,.3):break
    stall_s=i*dt;assert stall_s<8 and monitor.failure and monitor.valid_until==0
    short=ProgressClock(0,.003);near_end=TransmissionMonitor(0,1,{})
    for i in range(2400):
        _,state=short.advance(0,.008,.425,dt,.08,.04,.2)
        if near_end.update(i*dt,0,state,dt,.003):break
    assert near_end.failure and i*dt<8
    r=dict(status='SAFE_PARTIAL_PROGRESS',safely_stopped=True,physical_transmission_valid=True,ok=False)
    assert continuation_allowed(r,dict(continue_after=['SAFE_PARTIAL_PROGRESS']),None,dict(wall_s=20,sim_s=20))[0]
    for status in ['TRANSMISSION_FAILURE','SAFETY_STOP','BUDGET_EXHAUSTED']:
        assert not continuation_allowed(dict(r,status=status),dict(continue_after=['SAFE_PARTIAL_PROGRESS']),None,dict(wall_s=20,sim_s=20))[0]
    assert not continuation_allowed(r,{},None,dict(wall_s=20,sim_s=20))[0]
    assert not continuation_allowed(r,dict(continue_after=['SAFE_PARTIAL_PROGRESS']),dict(reason='force'),dict(wall_s=20,sim_s=20))[0]
    # Incomplete validation excluded even with a good transmitting prefix.
    a=dict(step=np.arange(1000),t=np.arange(1000)/240,door_q=np.arange(1000)*1e-5,door_dq=np.ones(1000)*.0024,safety_stop=np.zeros(1000))
    segment=dict(role='FIT_PREFIX',first_step=0,completed=True,declared_action=dict(fit_intervals_s=[[.1,3.8]]),result=dict(status='TRANSMISSION_FAILURE',ok=False,safely_stopped=True,fit_prefix_transmission_valid=True,fit_valid_until_s=2.8))
    ix,label=selection(segment,a,np.arange(1000),True);assert len(ix)>242 and a['t'][ix].max()<=2.8 and not label['action_ok']
    segment['role']='VALIDATION_PREFIX';ix,val=selection(segment,a,np.arange(1000),True);assert len(ix)==0
    fake=[0.];budget=EpisodeBudget(dict(episode_wall_budget_s=100,safe_exit_wall_reserve_s=20,episode_sim_budget_s=100,safe_exit_sim_reserve_s=10),now=lambda:fake[0]);fake[0]=81
    assert budget.remaining(0)['wall_s']==0 and budget.remaining(91)['sim_s']==0
    from types import SimpleNamespace
    from interaction_runtime.runtime import Runtime
    from interaction_runtime.policy import SafetyStop
    runtime=Runtime.__new__(Runtime);runtime.episode_budget=budget;runtime.tick=0;runtime.dt=dt;runtime.supervisor=SimpleNamespace(latched=None);runtime._decelerating=False;runtime.budget_exit_only=False;runtime.event=lambda *a,**k:None
    try:runtime.ensure_execution_budget('OPEN_CONTINUE');raise AssertionError('work allowed inside exit reserve')
    except ExecutionBudgetExhausted:pass
    assert runtime.budget_exit_only
    runtime.ensure_execution_budget('RELEASE');runtime.ensure_execution_budget('SAFE_PARK_MOTION')
    runtime.supervisor.latched=dict(reason='EMERGENCY_WRIST_FORCE')
    try:runtime.ensure_execution_budget('RELEASE');raise AssertionError('budget bypassed force latch')
    except SafetyStop:pass

    with tempfile.TemporaryDirectory() as tmp:
        import concurrent.futures
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as workers:
            paths=list(workers.map(lambda unused:allocate(tmp,'repeat'),range(12)))
        assert len(set(paths))==12
        x=allocate(tmp,'pregrasp');(x/'rgb.png').write_bytes(b'old');y=allocate(tmp,'pregrasp');assert x!=y and (x/'rgb.png').read_bytes()==b'old'
    active=['interaction_runtime/operations.py','interaction_runtime/probing_v2.py','scripts/agent_command_protocol_v12.py']
    for path in active:
        s=(ROOT/path).read_text();ast.parse(s);assert 'max_sim_s=45' not in s and "get('max_sim_s',45" not in s
    return dict(healthy_nominal_sim_s=plan['nominal_duration_s'],healthy_actual_sim_s=healthy_s,blocked_contact_stop_s=stall_s,partial_is_not_success=True,conditional_successor_checked=True,incomplete_validation=val,transmitting_prefix=label,checkpoint_no_overwrite=True,exit_reserve=True,active_source_audit=True,ProgressClock_sha256=hashlib.sha256((ROOT/'interaction_runtime/progress.py').read_bytes()).hexdigest())
if __name__=='__main__':
    data=verify();Path(sys.argv[1]).write_text(json.dumps(data,indent=2));print(json.dumps(data,indent=2))
