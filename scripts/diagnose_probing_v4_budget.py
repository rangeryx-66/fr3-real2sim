"""Post-exit motion-budget audit; no fit, truth access, or runtime modification."""
import argparse,inspect,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_v31_estimator import process
from interaction_runtime.motion import progress_reference
from interaction_runtime.probing_profiles_v3 import speed_fraction,information


def diagnose(p,segment_id):
    completion=json.loads((p/'PROCESS_COMPLETION.json').read_text());pid=json.loads((p/'PROCESS.json').read_text())['pid']
    if not completion['actor_exited'] or Path('/proc/%d'%pid).exists():raise RuntimeError('Post-exit diagnostic only')
    a,idx=columns(p);settings=json.loads((p/'identification_input.json').read_text());prior=settings['prior'];origin=np.array(prior['origin_world_m']);axis=np.array(prior['axis_world'])
    s=next(s for s in idx['segments'] if s['id']==segment_id);ix=np.flatnonzero(a['segment']==segment_id);t=a['t'][ix];elapsed=t-s['first_step']/240;action=s['declared_action'];ref=a['reference_q'][ix];q=a['door_q'][ix]
    offset=a['tcp_pose'][ix,:3]-origin;radius=np.linalg.norm(offset,axis=1);lead=inspect.signature(progress_reference).parameters['lead_rad'].default
    lag=np.maximum(0,ref-q);factor=1+lag/lead;v=np.array([action['reference_speed_m_s']*speed_fraction(x,action) for x in elapsed]);rate=np.diff(ref)*240;interior=(elapsed[1:]>2)&(elapsed[1:]<elapsed[-1]-2)
    predicted=v[:-1]/radius[:-1]/factor[:-1];error=np.rad2deg(rate-predicted)
    d=process(rows_at(a,ix,idx['metadata'],'DIAGNOSTIC_FAILED_SEGMENT'),prior,settings['bias'],a['door_dq'][ix]);g=d['valid'];X=np.c_[d['qdd'][g],d['qd'][g],d['direction'][g]]
    quant=lambda x:np.quantile(x,[.1,.5,.9]).tolist()
    result=dict(scope='post-exit diagnostic only; no parameter fit, model update or truth-based selection',segment=segment_id,result=s['result'],duration_s=float(t[-1]-t[0]),actual_gain_deg=float(np.rad2deg(q[-1]-q[0])),mean_actual_deg_s=float(np.rad2deg(q[-1]-q[0])/(t[-1]-t[0])),final_reference_deg=float(np.rad2deg(ref[-1])),
        active_lead_deg=float(np.rad2deg(lead)),reference_lead_P10_P50_P90_deg=quant(np.rad2deg(lag)),clock_rate_divisor_P10_P50_P90=quant(factor),effective_radius_P10_P50_P90_m=quant(radius),perpendicular_hinge_radius_P10_P50_P90_m=quant(np.linalg.norm(np.cross(offset,axis),axis=1)),nominal_speed_P10_P50_P90_mm_s=quant(v*1000),clock_rate_RMS_difference_deg_s=float(np.sqrt(np.mean(error[interior]**2))),
        active_code='Operations.interact passes 0.5deg overshoot, default lead stays 0.05deg; unchanged progress clock',regression_valid_samples=int(g.sum()),qdd_rms_rad_s2=float(np.sqrt(np.mean(d['qdd'][g]**2))),qd_rms_rad_s=float(np.sqrt(np.mean(d['qd'][g]**2))),compensated_torque_rms_nm=float(np.sqrt(np.mean(d['torque'][g]**2))),regressor_information=information(X),information_scope='scaled sample-level regressor, not online binned FIT information',why_no_online_fit='both result.ok=False segments excluded by frozen worker; no completed VALIDATION')
    (p/'POSTHOC_LIMITER_DIAGNOSTIC.json').write_text(json.dumps(result,indent=2));return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('episode',type=Path);parser.add_argument('--segment',type=int,required=True);a=parser.parse_args();print(json.dumps(diagnose(a.episode,a.segment),indent=2))
