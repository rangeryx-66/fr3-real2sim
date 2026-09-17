"""Offline synchronized empty-arm velocity semantics audit; no identification."""
import sys,json,argparse
from pathlib import Path
import numpy as np
from quasistatic_velocity_gate import configuration_velocity
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def stats(x):
 x=np.asarray(x);return {'mean':np.nanmean(x,axis=0).tolist(),'RMS':np.sqrt(np.nanmean(x*x,axis=0)).tolist(),'max_abs':np.nanmax(np.abs(x),axis=0).tolist()}
def main():
 p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();out=a.directory;res={};files=sorted(out.glob('pose_*.npz'));fig,axs=plt.subplots(len(files),3,figsize=(15,4*len(files)),squeeze=False)
 for row,path in enumerate(files):
  d=np.load(path);t=d['t'];dt=np.diff(t);mask=(t-t[0])>=1.;q=d['q_actual'];fd=d['dq_fd'];tp=d['tcp_position'];R=Rotation.from_quat(np.roll(d['tcp_quat_wxyz'],-1,axis=1));v=np.full_like(tp,np.nan);v[1:]=np.diff(tp,axis=0)/dt[:,None];omega=np.full_like(tp,np.nan);omega[1:]=(R[:-1].inv()*R[1:]).as_rotvec()/dt[:,None]
  raw=d['dq_reported'];direct=d['dq_physx'];target=d['dq_target'];result={'sample_count':len(t),'duration_s':float(t[-1]-t[0]),'step_increment_unique':np.unique(np.diff(d['step'])).tolist(),'dt_min_max': [float(dt.min()),float(dt.max())],'world_time_offset_range_s':[float((d['world_time']-t).min()),float((d['world_time']-t).max())],'q_vs_physx_max_rad':float(np.max(np.abs(q-d['q_physx']))),'reported_vs_physx_max_rad_s':float(np.max(np.abs(raw-direct))),'reported_vs_target_max_rad_s':float(np.max(np.abs(raw-target))),'reported_vs_fd_max_rad_s':float(np.nanmax(np.abs(raw[mask]-fd[mask]))),'stats':{k:stats(d[k][mask]) for k in ['dq_reported','dq_physx','dq_fd','dq_target','controller_velocity','tau']},'tcp_linear_velocity':stats(v[mask]),'tcp_angular_velocity':stats(omega[mask]),'q_total_span_rad':np.ptp(q[mask],axis=0).tolist(),'tau_half_window_drift_Nm':float(np.max(np.abs(d['tau'][mask][:sum(mask)//2].mean(0)-d['tau'][mask][sum(mask)//2:].mean(0)))),'old_gate_pass':bool(np.max(np.abs(raw[mask]))<.002),'fd_threshold_pass':bool(np.nanmax(np.abs(fd[mask]))<.002)}
  windows=[]
  for start in np.arange(1.,float(t[-1]-t[0])-.5,.5):
   sel=(t-t[0]>=start)&(t-t[0]<start+.5);vt=configuration_velocity(t[sel],q[sel]);tt=d['tau'][sel];half=len(tt)//2;drift=float(np.max(np.abs(tt[:half].mean(0)-tt[half:].mean(0))))
   windows.append({'start_s':float(start),'samples':int(sel.sum()),'old_pass':bool(np.max(np.abs(raw[sel]))<.002),'new_pass':bool(len(tt)>=96 and np.max(np.abs(vt))<.002 and drift<.05),'dq_fd_max':float(np.max(np.abs(vt))),'torque_drift_Nm':drift})
  result['settled_half_second_windows']=windows
  gate_v=configuration_velocity(t[mask],q[mask]);result['corrected_static_gate_pass']=bool(np.max(np.abs(gate_v))<.002 and result['tau_half_window_drift_Nm']<.05);result['corrected_gate_source']='ACTUAL_Q_TIMED_FINITE_DIFFERENCE';res[path.stem]=result
  axs[row,0].plot(t-t[0],np.max(np.abs(raw),axis=1),label='reported / PhysX');axs[row,0].plot(t-t[0],np.nanmax(np.abs(fd),axis=1),label='q finite difference');axs[row,0].plot(t-t[0],np.max(np.abs(target),axis=1),label='target');axs[row,0].axhline(.002,c='r',ls='--',label='0.002 threshold');axs[row,0].set_ylabel('rad/s');axs[row,0].set_title(path.stem+' max joint speed');axs[row,0].legend()
  axs[row,1].plot(t-t[0],np.linalg.norm(tp-tp[0],axis=1)*1e3);axs[row,1].set_ylabel('mm');axs[row,1].set_title('TCP translation relative to start')
  axs[row,2].plot(t-t[0],d['tau']-d['tau'][mask].mean(0));axs[row,2].set_ylabel('N m');axs[row,2].set_title('Torque deviation (7 joints)')
  for ax in axs[row]:ax.set_xlabel('simulation seconds');ax.grid(alpha=.2)
 fig.tight_layout();fig.savefig(out/'dq_timeseries.png',dpi=150);plt.close(fig)
 (out/'analysis.json').write_text(json.dumps(res,indent=2));print(json.dumps({k:{q:v[q] for q in ['duration_s','reported_vs_physx_max_rad_s','reported_vs_target_max_rad_s','reported_vs_fd_max_rad_s','old_gate_pass','fd_threshold_pass','tau_half_window_drift_Nm']} for k,v in res.items()},indent=2))
if __name__=='__main__':main()
