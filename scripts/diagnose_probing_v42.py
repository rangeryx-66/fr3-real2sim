"""Archived transmission/command audit, no physics actor or hidden dynamics."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns,rows_at
from update_probing_v2_model import matrix
from xarm_ft_v31_estimator import process
from interaction_runtime.kinematics import FastNativeModel
from interaction_runtime.serialization import result_packet
from interaction_runtime.progress import hinge_radius
from interaction_runtime.segments import TransmissionMonitor
from interaction_runtime.probing_profiles_v3 import speed_fraction


def audit(p,out):
    started=time.perf_counter();a,idx=columns(p);j=json.loads((p/'job.json').read_text());c=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text());settings=json.loads((p/'identification_input.json').read_text());prior=settings['prior'];origin=np.asarray(prior['origin_world_m']);axis=np.asarray(prior['axis_world']);axis/=np.linalg.norm(axis)
    model=FastNativeModel(None,Path(j['asset_root']),json.loads((Path(j['source'])/'report.json').read_text()));base=json.loads((p/'visual_plan.json').read_text())['base'];arm=c['arm_indices'];fingers=c['finger_indices'];result=[]
    for s in idx['segments']:
        ix=np.flatnonzero(a['segment']==s['id']);t=a['t'][ix]-a['t'][ix[0]];q=a['door_q'][ix];direction=np.sign(np.deg2rad(s['target_deg'])-q[0]);T0=matrix(a['tcp_pose'][ix[0]]);r0=T0[:3,3]-origin;r0-=axis*(axis@r0);radius=hinge_radius(T0[:3,3],origin,axis)
        d=process(rows_at(a,ix,idx['metadata'],s['role']),prior,settings['bias'],a['door_dq'][ix]);take=np.arange(0,len(ix),8);data=[]
        for k in take:
            i=ix[k];P=model.poses(a['q'][i,arm],base,finger_q=a['q'][i,fingers])['tcp_link'];C=model.poses(a['command_q'][i],base,finger_q=a['q'][i,fingers])['tcp_link'];M=matrix(a['tcp_pose'][i]);r=M[:3,3]-origin;r-=axis*(axis@r);tcp_q=q[0]+np.arctan2(axis@np.cross(r0,r),r0@r)
            data.append([t[k],np.linalg.norm(C[:3,3]-M[:3,3]),np.linalg.norm(P[:3,3]-M[:3,3]),float(np.max(abs(a['command_q'][i]-a['q'][i,arm]))),direction*(tcp_q-q[k])])
        data=np.array(data);windows=[]
        for lo,hi in [(0,min(8,t[-1])),(max(0,t[-1]-10),t[-1])]:
            mask=(t>=lo)&(t<=hi);m=(data[:,0]>=lo)&(data[:,0]<=hi)
            windows.append(dict(interval_s=[float(lo),float(hi)],hinge_speed_median_deg_s=float(np.rad2deg(np.median(abs(a['door_dq'][ix][mask])))),reference_lead_median_deg=float(np.rad2deg(np.median(direction*(a['reference_q'][ix][mask]-q[mask])))),actual_TCP_door_lead_median_deg=float(np.rad2deg(np.median(data[m,4]))),command_TCP_error_P50_P95_mm=(np.quantile(data[m,1],[.5,.95])*1000).tolist(),actual_FK_TCP_error_P95_mm=float(np.quantile(data[m,2],.95)*1000),command_joint_error_P50_P95_rad=np.quantile(data[m,3],[.5,.95]).tolist(),relative_slip_P50_P95_mm=(np.quantile(a['relative_slip_m'][ix][mask],[.5,.95])*1000).tolist(),aperture_error_P50_mm=float(np.median((a['aperture'][ix]-a['command_aperture'][ix])[mask])*1000),pads_mean_n=np.mean(a['pad_load'][ix][mask],0).tolist(),compensated_hinge_torque_mean_nm=float(np.mean(d['torque'][mask]))))
        action=s['declared_action'];monitor=TransmissionMonitor(q[0],direction,action);trigger=None
        for k in range(len(ix)):
            i=ix[k];speed=min(.012,action.get('reference_speed_m_s',.012))*speed_fraction(t[k],action)
            state=dict(requested_rate_rad_s=direction*min(speed/radius,.08),lag_rad=direction*(a['reference_q'][i]-q[k]))
            if monitor.update(t[k],q[k],state,1/240,direction*(np.deg2rad(s['target_deg'])-q[k])):
                trigger=float(t[k]);break
        result.append(dict(id=s['id'],role=s['role'],original=s['result'],radius_m=radius,windows=windows,diagnostic_monitor_stop_s=trigger,diagnostic_valid_prefix_s=monitor.valid_until,diagnostic_last_window=monitor.last))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(11,9),constrained_layout=True)
    for n,s in enumerate(idx['segments']):
        ix=np.flatnonzero(a['segment']==s['id']);t=a['t'][ix]-a['t'][ix[0]];d=result[n]
        axes[n,0].plot(t,np.rad2deg(a['door_q'][ix]),label='measured door')
        axes[n,0].plot(t,np.rad2deg(a['reference_q'][ix]),label='reference',alpha=.7)
        axes[n,0].set(title=s['role'],ylabel='angle (deg)',xlabel='segment time (s)')
        axes[n,1].plot(t,a['relative_slip_m'][ix]*1000,label='relative slip (mm)')
        axes[n,1].plot(t,a['pad_load'][ix,0],label='L normal (N)',alpha=.7)
        axes[n,1].plot(t,a['pad_load'][ix,1],label='R normal (N)',alpha=.7)
        axes[n,1].set(xlabel='segment time (s)',ylabel='mm / N (labelled traces)')
        if d['diagnostic_monitor_stop_s'] is not None:
            for ax in axes[n]:ax.axvline(d['diagnostic_monitor_stop_s'],color='red',ls='--',label='diagnostic stop')
        for ax in axes[n]:ax.grid(alpha=.2);ax.legend(fontsize=8)
    out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(out.with_suffix('.png'),dpi=150);plt.close(fig)
    report=dict(source=str(p),archived_only=True,hidden_dynamics_used=False,FK_sample_hz=30,normal_report_absence_not_separation=True,segments=result,wall_s=time.perf_counter()-started);out.parent.mkdir(parents=True,exist_ok=True);report=result_packet(report);out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('output',type=Path);a=p.parse_args();audit(a.episode,a.output)
