from pathlib import Path
import json,gzip,pickle,sys
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(sys.argv[1]);O=R/'results/nero_actuator_stability_20261012';P=R/'results/nero_feasibility_20261012/integration_probe'
def load(d):
 rows=[]
 for f in sorted((d/'observations_chunks').glob('*.pkl.gz')):
  with gzip.open(f,'rb') as h:rows+=pickle.load(h)
 return rows
sets=[('Old explicit + false contact',load(P)),('Implicit + corrected proxy',load(O/'run_02')),('Implicit + gravity substeps',load(O/'run_03'))]
fig,ax=plt.subplots(3,2,figsize=(13,9))
for label,rows in sets:
 t=np.array([r['t'] for r in rows]);q=np.array([r['q'] for r in rows]);v=np.array([r['dq'] for r in rows]);goal=np.array([r['q_des_arm'] for r in rows]);tcp=np.array([r['T_tcp'] for r in rows]);dt=1/240;err=abs(q[:,:7]-goal).max(axis=1);speed=np.linalg.norm(np.diff(tcp[:,:3,3],axis=0),axis=1)/dt
 ax[0,0].plot(t,err,label=label);ix=t<=3;ax[0,1].plot(t[ix],err[ix],label=label)
 ax[1,0].plot(t[1:],speed*1000,label=label);i=t<=3;ax[1,1].plot(t[i],v[i,6],label=label)
 fd=np.diff(q[:,6])/dt;sel=t[1:]<=3;ax[2,0].plot(t[1:][sel],(v[1:,6]-fd)[sel],label=label)
 if 'actuator' in rows[0]:
  projected=np.array([r['solver_projected_effort'] for r in rows]);ax[2,1].plot(t,abs(projected[:,:7]).max(axis=1),label=label+' projected reaction')
 else:ax[2,1].plot(t,abs(np.array([r['tau_arm_nm'] for r in rows])).max(axis=1),label=label+' external command')
for a,title,y in zip(ax.flat,['Joint tracking, full tests','Same initial gravity hold','TCP position-difference velocity','Raw native joint7 velocity, hold','Native joint7 minus full-step velocity','Effort channels (different semantics)'],['rad','rad','mm/s','rad/s','rad/s','Nm']):
 a.set_title(title);a.set_xlabel('Simulation seconds');a.set_ylabel(y);a.grid(alpha=.25)
ax[0,0].set_yscale('symlog',linthresh=1e-4);ax[0,0].legend(fontsize=8);ax[2,1].legend(fontsize=7)
fig.suptitle('NERO robot-only actuator diagnostics — raw native values retained, no filtering');fig.tight_layout();fig.savefig(O/'ACTUATOR_COMPARISON.png',dpi=160)
