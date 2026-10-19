"""Post-exit retention timeline; native normal contacts are partial wrench data."""
import argparse,gzip,json,pickle
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);a=p.parse_args()
read=lambda p:json.loads(p.read_text())
def window(run,name,lo,hi):
 out=[];folder=run/(name+'_chunks')
 for c in read(folder/'index.json')['chunks']:
  rows=pickle.load(gzip.open(folder/c['file'],'rb'))
  if rows[0]['t']>hi:break
  if rows[-1]['t']<lo:continue
  out.extend(x for x in rows if lo<=x['t']<=hi)
 return out
run=a.root/'episode';report=read(run/'report.json');acq=report['grasp_acquisition'];t0=acq['t'];events=read(run/'events.json');events=events if isinstance(events,list) else events['events'];opening=next(x for x in events if x['event']=='SHORT_OPENING_START')
rows=window(run,'observations',t0-1,t0+2);phy=window(run,'physics_steps',t0-1,t0+2)
t=np.array([x['t'] for x in rows]);loads=np.array([[x['forces_n'][f] for f in ['gripper_link1','gripper_link2']] for x in rows]);states={}
for x in rows:
 e=x.get('force_event') or {};states.setdefault(e.get('status','missing'),x)
def brief(x):return dict(t=x['t'],phase=x['phase'],force_event=x.get('force_event'),filtered_pad_loads_n=x['forces_n'],aperture_mm=x['aperture_m']*1000,commanded_aperture_mm=x['telemetry']['command_aperture_m']*1000,door_angle_deg=x['door_angle_deg'])
rawzero=next(x for x in phy if x['t']>=t0 and not any(c.get('finger')=='gripper_link1' and c.get('allowed_pad_target') and c.get('force_n',0)>0 for c in x.get('contacts',[])))
hold=[x for x in rows if t0<=x['t']<opening['t']];q=np.array([x['q'][7:] for x in hold]);A=np.array([x['aperture_m'] for x in hold]);C=np.array([x['telemetry']['command_aperture_m'] for x in hold])
result=dict(first_retention_divergence=dict(threshold_event=next(x for x in events if x.get('event')=='PAD_CONTACT_LOST' and x.get('finger')=='gripper_link1' and x['t']>t0),first_zero_positive_normal_report_t=rawzero['t'],interpretation='Left unload starts during stationary post-verification hold, before opening; not automatically true grasp loss.'),first_force_policy_status={k:brief(x) for k,x in states.items()},last_force_policy=brief(rows[-1]),hold_aperture=dict(command_min_mm=float(C.min()*1000),command_max_mm=float(C.max()*1000),measured_min_mm=float(A.min()*1000),measured_max_mm=float(A.max()*1000),maximum_coupled_joint_change_from_hold_first_rad=float(np.max(abs(q-q[0])))),opening_start=opening,root_cause_limit='Observed retention/load-distribution failure. Exact material contact/settling cause is not identified by one episode; no new controller or geometry fix is inferred here.')
fig,axs=plt.subplots(4,1,figsize=(10,8),sharex=True)
for root,label,color in [(a.baseline,'a47fe09 physical baseline','#888888'),(a.root,'Incremental physical trial','#167ba5')]:
 b=root/'episode';r=read(b/'report.json');zero=r['grasp_acquisition']['t'];v=window(b,'observations',zero-1,zero+2);tv=np.array([x['t']-zero for x in v]);f=np.array([[x['forces_n'][z] for z in ['gripper_link1','gripper_link2']] for x in v])
 for i,side in enumerate(['left','right']):axs[i].plot(tv,f[:,i],label=label,color=color);axs[i].set_ylabel(side+' normal (N)')
axs[2].plot(t-t0,[x['aperture_m']*1000 for x in rows],label='measured');axs[2].plot(t-t0,[x['telemetry']['command_aperture_m']*1000 for x in rows],label='command');axs[2].set_ylabel('Aperture (mm)')
axs[3].plot(t-t0,[x['door_angle_deg']-acq['angle_deg'] for x in rows],label='actual net opening');axs[3].set(ylabel='Door gain (deg)',xlabel='Time after GRASP_VERIFIED (s)')
for ax in axs:ax.axvline(0,color='gray',ls=':');ax.axvline(opening['t']-t0,color='#bc452a',ls='--',label='new opening start' if ax==axs[0] else None);ax.grid(alpha=.2);ax.legend(fontsize=7)
fig.suptitle('Bilateral acquisition, then left unloading during unchanged 1-second hold\nOriginal sustained force stop after only 0.019 deg; normal data are not a full wrench');fig.tight_layout();fig.savefig(a.root/'analysis/HOLD_DIVERGENCE.png',dpi=150);plt.close(fig)
(a.root/'analysis/FIRST_DIVERGENCE.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
