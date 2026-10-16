"""Post-episode figures only: parameter updates and independent held-out motion.

Run only after independent hidden-truth evaluation has completed. No controller
imports, no fitting, and no trajectory forcing. Raw six-axis plots stay private.
"""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('case',type=Path);p.add_argument('--strict',action='store_true');a=p.parse_args()
e=json.loads((a.case/'INDEPENDENT_EVALUATION.json').read_text())
u=e['updates'];out=a.case/'evaluation_figures';out.mkdir(exist_ok=True)
if not u:raise SystemExit('No identified updates')
keys=['I_eff_kg_m2','b_nm_s_rad','tau_c_nm'];flags=['I_eff','b','tau_c']
fig,ax=plt.subplots(1,3,figsize=(13,3.8))
for axis,k,flag in zip(ax,keys,flags):
 vals=[v['estimated']['parameters'][k] for v in u];truth=u[-1]['authored_truth'][k]
 axis.plot(np.arange(len(u)),vals,'o-',label='provisional fit')
 axis.axhline(truth,color='black',ls='--',label='hidden authored truth (post hoc)')
 for j,v in enumerate(u):
  if not v['estimated']['observability'][flag]:axis.annotate('unsupported',(j,vals[j]),xytext=(0,8),textcoords='offset points',ha='center',fontsize=8)
 axis.set_title(k);axis.set_xlabel('cumulative segment update');axis.grid(alpha=.25)
ax[0].legend(fontsize=7)
fig.suptitle('Conditional fits, not calibrated confidence or recovered full object physics')
fig.tight_layout();fig.savefig(out/'PARAMETER_UPDATES.png',dpi=150);plt.close(fig)
last=u[-1];folder=a.case/'episode'/Path(last['path']).parent
d=np.load(folder/'measured_effort.npz',allow_pickle=True)
beta=(e['strict_posthoc_boundary_evaluation'][-1]['model'] if a.strict else last['estimated'])['parameters']
segments=d['segments'].astype(str);labels=[s for s in dict.fromkeys(segments) if s.startswith('VALIDATION')]
for label in labels:
 ix=np.flatnonzero(segments==label)
 if len(ix)<2:continue
 t=d['t'][ix];q=d['q'][ix];v=d['qd'][ix];tau=d['torque'][ix]
 I,b,c=[beta[k] for k in keys]
 if I<=1e-8:continue
 qp=[float(q[0])];vp=[float(v[0])]
 for j in range(1,len(ix)):
  dt=float(t[j]-t[j-1]);momentum=I*vp[-1]+dt*tau[j-1]
  vn=float(np.sign(momentum)*max(abs(momentum)-dt*c,0)/(I+dt*b))
  vp.append(vn);qp.append(qp[-1]+dt*vn)
 fig,ax=plt.subplots(3,1,figsize=(10,8),sharex=True)
 time=t-t[0];thin=np.arange(0,len(t),max(1,len(t)//1500))
 ax[0].plot(time[thin],np.rad2deg(q[thin]),label='measured angle');ax[0].plot(time[thin],np.rad2deg(np.asarray(qp)[thin]),label='independent prediction')
 ax[0].set_ylabel('angle deg');ax[0].legend()
 pred=I*d['qdd'][ix]+b*v+c*np.sign(v)
 ax[1].plot(time[thin],tau[thin],label='wrist F/T-derived torque',lw=.6);ax[1].plot(time[thin],pred[thin],label='model torque',lw=.6);ax[1].set_ylabel('Nm');ax[1].legend()
 ax[2].plot(time[thin],v[thin],label='measured velocity');ax[2].plot(time[thin],np.asarray(vp)[thin],label='predicted velocity');ax[2].set_ylabel('rad/s');ax[2].legend()
 for axis in ax:axis.grid(alpha=.25)
 ax[-1].set_xlabel('seconds from whole held-out closing segment start')
 fig.suptitle(('Strict post-hoc boundary exclusion / ' if a.strict else '')+'Independent dynamics: measured wrench + initial state only\nNo following-state correction; display subsampled, full-rate private evidence retained')
 suffix='_PREDICTION_STRICT.png' if a.strict else '_PREDICTION.png'
 fig.tight_layout();fig.savefig(out/(label+suffix),dpi=150);plt.close(fig)
print(str(out))
