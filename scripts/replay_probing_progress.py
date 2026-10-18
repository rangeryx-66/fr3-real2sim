"""Recorded-state replay; no object dynamics prediction and no truth inputs."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns
from interaction_runtime.motion import progress_reference
from interaction_runtime.progress import ProgressClock,hinge_radius
from interaction_runtime.policy import Policy
from interaction_runtime.probing_profiles_v3 import speed_fraction


def replay(folder):
    a,idx=columns(folder);prior=json.loads((folder/'identification_input.json').read_text())['prior'];axis=np.array(prior['axis_world']);origin=np.array(prior['origin_world_m']);m=Policy.load(ROOT/'configs/interaction_runtime_v1.json').motion('LOADED');results=[]
    for seg in idx['segments']:
        ix=np.flatnonzero(a['segment']==seg['id'])
        if not len(ix):continue
        action=seg.get('declared_action',{});target=np.deg2rad(seg['target_deg']);q=a['door_q'][ix];t=a['t'][ix];ref=a['reference_q'][ix];direction=1 if target>=q[0] else -1
        clock=ProgressClock(q[0],target);new=[];old_error=[];radius=[]
        for j,i in enumerate(ix):
            if not np.isfinite(ref[j]):continue
            nominal=min(m['speed_m_s'],action.get('reference_speed_m_s',m['speed_m_s']))*speed_fraction(t[j]-seg['first_step']/240,action)
            r=hinge_radius(a['tcp_pose'][i,:3],origin,axis);radius.append(r)
            x,state=clock.advance(q[j],nominal,r,1/240,m['angular_speed_rad_s'],m['acceleration_m_s2'],m['angular_acceleration_rad_s2']);new.append([x,state['tracking_limited']])
            if j+1<len(ix):
                old=progress_reference(q[j],ref[j],target,direction,nominal,np.linalg.norm(a['tcp_pose'][i,:3]-origin),1/240,np.deg2rad(.5))
                old_error.append(old-ref[j+1])
        new=np.array(new);lag=direction*(ref-q);duration=t[-1]-t[0]
        results.append(dict(id=seg['id'],role=seg['role'],original_result=seg['result'],duration_s=float(duration),actual_gain_deg=float(np.rad2deg(q[-1]-q[0])),median_old_lag_deg=float(np.rad2deg(np.nanmedian(lag))),median_old_divisor=float(np.nanmedian(1+np.maximum(0,lag)/np.deg2rad(.05))),legacy_one_step_RMS_error_deg=float(np.rad2deg(np.sqrt(np.mean(np.array(old_error)**2)))),perpendicular_radius_m=float(np.median(radius)),new_reference_end_on_frozen_measured_states_deg=float(np.rad2deg(new[-1,0])),new_limited_fraction_on_frozen_slow_states=float(new[:,1].mean()),
            warning='Recorded slow states are unchanged; replay cannot establish physical speedup or new target completion'))
    return dict(source=str(folder),kind='RECORDED_STATE_REFERENCE_REPLAY_ONLY',segments=results)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folders',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=[replay(f) for f in a.folders];a.output.parent.mkdir(exist_ok=True,parents=True);a.output.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
