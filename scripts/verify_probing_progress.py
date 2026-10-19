"""Deterministic clock, completion, partial FIT and unchanged guard regressions."""
import json,sys,tempfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.progress import ProgressClock,hinge_radius,segment_plan,validate_action
from interaction_runtime.probing_profiles_v3 import speed_fraction
from interaction_runtime.segment_quality import selection
from interaction_runtime.policy import Policy


def simulate(target,lag_s=.15,stalled=False):
    dt=1/240;theta=0.;c=ProgressClock(theta,target);m=Policy.load(ROOT/'configs/interaction_runtime_v1.json').motion('LOADED');rows=[]
    a=dict(velocity_program='plateaus',reference_speed_m_s=.012,low_speed_fraction=.125,ramp_s=.45,plateau_s=.55)
    for i in range(240*45):
        ref,state=c.advance(theta,.012*speed_fraction(i*dt,a),.424,dt,m['angular_speed_rad_s'],m['acceleration_m_s2'],m['angular_acceleration_rad_s2'])
        if not stalled:theta+=(ref-theta)*(1-np.exp(-dt/lag_s))
        rows.append([theta,ref,c.velocity,state['tracking_limited']])
        if c.direction*(target-theta)<=0:break
    return np.array(rows),len(rows)*dt


def verify():
    m=Policy.load(ROOT/'configs/interaction_runtime_v1.json').motion('LOADED');a=dict(role='FIT_CLOCK',target_deg=13.,max_sim_s=45.,velocity_program='plateaus',reference_speed_m_s=.012,low_speed_fraction=.125,ramp_s=.45,plateau_s=.55)
    assert hinge_radius([1,0,5],[0,0,0],[0,0,1])==1
    results=[]
    for target in [np.deg2rad(13),-np.deg2rad(13)]:
        x,t=simulate(target);assert np.sign(target)*x[-1,0]>=abs(target);assert np.max(abs(np.diff(x[:,2])))*240<=min(.04/.424,.2)+1e-9
        results.append(dict(target_deg=float(np.rad2deg(target)),duration_s=t,max_lag_deg=float(np.rad2deg(abs(x[:,1]-x[:,0])).max()),limited_fraction=float(x[:,3].mean())))
    x,t=simulate(.3,stalled=True);assert abs(x[-1,2])<1e-10 and x[-1,1]<np.deg2rad(.501)
    nominal=segment_plan(0,np.deg2rad(13),a,m,.424);assert nominal['valid']
    impossible=segment_plan(0,np.deg2rad(170),dict(a,max_sim_s=2),m,.424);assert not impossible['valid']
    clipped=segment_plan(0,np.deg2rad(170),dict(a,max_sim_s=2,allow_target_truncation=True),m,.424);assert clipped['valid'] and clipped['target_truncated'] and clipped['selected_target_rad']<np.deg2rad(170)
    # Signed changes start at the measured state with zero reference velocity.
    forward=ProgressClock(.2,.1);assert forward.reference==.2 and forward.velocity==0
    arrays=dict(step=np.arange(2400),t=np.arange(2400)/240,safety_stop=np.zeros(2400,dtype=int));indices=np.arange(2400)
    s=dict(id=1,first_step=0,completed=True,role='FIT_CLOCK',declared_action=dict(fit_intervals_s=[[1,8]]),result=dict(ok=False,status='TIMEOUT',safely_stopped=True,physical_transmission_valid=True,fit_valid_until_s=9))
    selected,label=selection(s,arrays,indices,True);assert len(selected)==1681 and not s['result']['ok'];assert label['action_remains_failed']
    assert len(selection(s,arrays,indices,False)[0])==0
    complete=dict(s,result=dict(ok=True,status='TARGET_REACHED'))
    assert len(selection(complete,arrays,indices,True)[0])==1681
    assert len(selection(complete,arrays,indices,False)[0])==2400
    for change in [dict(role='VALIDATION_CLOCK'),dict(result=dict(s['result'],physical_transmission_valid=False)),dict(result=dict(s['result'],safely_stopped=False))]:
        assert not len(selection(dict(s,**change),arrays,indices,True)[0])
    validation=dict(s,role='VALIDATION_CLIPPED',result=dict(ok=True,requested_target_reached=False))
    assert not len(selection(validation,arrays,indices,True)[0])
    arrays['safety_stop'][1000]=1;selected,_=selection(s,arrays,indices,True);assert 1000 not in selected
    for a in [dict(role='VALIDATION',fit_intervals_s=[[0,1]]),dict(role='FIT',max_sim_s=-1),dict(role='FIT',fit_intervals_s=[[2,1]])]:
        try:validate_action(a);raise AssertionError('Invalid protocol accepted')
        except ValueError:pass
    return dict(clock_simulation_not_Isaac=results,nominal_plan=nominal,stalled_clock_final_deg=float(np.rad2deg(x[-1,1])),impossible_target_rejected_before_motion=True,truncation_explicit=clipped,partial_FIT_selection=True,incomplete_VALIDATION_excluded=True,physical_episodes=0)

if __name__=='__main__':
    d=verify();out=Path(sys.argv[1]);out.parent.mkdir(exist_ok=True,parents=True);out.write_text(json.dumps(d,indent=2));print(json.dumps(d,indent=2))
