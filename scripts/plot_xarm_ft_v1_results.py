"""Post-outcome compact derived figures; full raw traces remain private."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from xarm_ft_v1_estimator import features
p=argparse.ArgumentParser();p.add_argument('case',type=Path);a=p.parse_args();e=json.loads((a.case/'INDEPENDENT_EVALUATION_V1.json').read_text());out=a.case/'evaluation_figures';out.mkdir(exist_ok=True)
updates=e['updates']
if updates:
 fig,axes=plt.subplots(1,3,figsize=(14,4))
 for kind,color in [('constant','#205a91'),('state_coulomb','#b7601b')]:
  for j,entry in enumerate(updates):
   m=entry['models'][kind]
   for k in [0,1]:axes[k].errorbar([j],[m['model']['parameters'][k]],yerr=[m['model']['conditional_std'][k]],color=color,fmt='o',capsize=3,label=kind if j==0 else None)
   axes[2].plot(m['curve_angle_deg'],m['estimated_c_nm'],label='%s update%d'%(kind,j),alpha=.45+.5*j/max(1,len(updates)-1))
 axes[0].axhline(e['authored_truth']['I_eff_kg_m2'],color='black',ls='--');axes[1].axhline(e['authored_truth']['b_nm_s_rad'],color='black',ls='--')
 last=updates[-1]['models']['constant'];axes[2].plot(last['curve_angle_deg'],last['authored_c_nm'],'k--',label='hidden authored law (post hoc only)')
 for ax,title in zip(axes,['I effective kgm2','viscous b Nms/rad','Coulomb resistance Nm']):ax.set_title(title);ax.grid(alpha=.25);ax.legend(fontsize=6)
 axes[0].set_xlabel('completed information block');axes[1].set_xlabel('completed information block');axes[2].set_xlabel('actually observed angle deg')
 fig.suptitle('Conditional parameter updates and observed-range function; not hardware confidence')
 fig.tight_layout();fig.savefig(out/'PARAMETER_FUNCTION_UPDATES.png',dpi=150);plt.close(fig)
for post in e['posthoc']:
 folder=a.case/'episode/posthoc_v1'/('attempt_%02d'%post['attempt']);data=dict(np.load(folder/'MEASUREMENTS.npz',allow_pickle=True));model=post['model'];samples=[]
 # Compact coverage plots are derived, not replacements for raw native records.
 t=data['t'];ids=np.arange(0,len(t),max(1,len(t)//2500));q=np.rad2deg(data['q'])
 fig,axes=plt.subplots(2,2,figsize=(12,8))
 scatter=axes[0,0].scatter(q[ids],data['qd'][ids],c=t[ids],s=2);axes[0,0].set_xlabel('angle deg');axes[0,0].set_ylabel('velocity rad/s')
 axes[0,1].scatter(q[ids],data['qdd'][ids],c=t[ids],s=2);axes[0,1].set_xlabel('angle deg');axes[0,1].set_ylabel('acceleration rad/s2')
 axes[1,0].scatter(q[ids],data['torque'][ids],c=np.sign(data['qd'][ids]),cmap='coolwarm',s=2);axes[1,0].set_xlabel('angle deg');axes[1,0].set_ylabel('measured wrist-derived effort Nm')
 axes[1,1].plot(t[ids],q[ids]);axes[1,1].axhspan(7,9,color='gold',alpha=.2,label='interior region excluded from fitting');axes[1,1].set_xlabel('simulation s');axes[1,1].set_ylabel('actual angle deg');axes[1,1].legend(fontsize=8)
 for ax in axes.flat:ax.grid(alpha=.25)
 fig.colorbar(scatter,ax=axes[0,:],label='simulation time s');fig.suptitle('Actual executed motion/excitation; display subsampled, full 240Hz private')
 fig.savefig(out/('COVERAGE_%02d.png'%post['attempt']),dpi=150,bbox_inches='tight');plt.close(fig)
 fig,ax=plt.subplots(3,1,figsize=(12,8),sharex=True)
 ax[0].plot(t[ids],data['pad_loads'][ids,0],label='left normal load');ax[0].plot(t[ids],data['pad_loads'][ids,1],label='right normal load');ax[0].set_ylabel('normal N');ax[0].legend()
 ax[1].plot(t[ids],data['relative_translation_proxy_m'][ids]*1000,label='TCP/body relative translation proxy');ax[1].set_ylabel('mm');ax[1].legend()
 ax[2].plot(t[ids],data['command_aperture_m'][ids]*1000,label='command aperture');ax[2].plot(t[ids],data['measured_aperture_m'][ids]*1000,label='measured aperture');ax[2].set_ylabel('mm');ax[2].legend();ax[2].set_xlabel('simulation s')
 for axis in ax:axis.grid(alpha=.25)
 fig.suptitle('Contact/holding diagnostics; single-pad zero is not proof of physical separation')
 fig.tight_layout();fig.savefig(out/('CONTACT_%02d.png'%post['attempt']),dpi=140);plt.close(fig)
 # Same frozen models evaluated only over the actually observed range.
 if model.get('models'):
  from evaluate_xarm_ft_v1 import curve
  angle=np.linspace(float(data['q'].min()),float(data['q'].max()),101);law=e['authored_truth']['Coulomb']
  authored=law['base_nm']+law['amplitude_nm']*np.exp(-.5*((angle-law['center_rad'])/law['width_rad'])**2)
  fig,axs=plt.subplots(1,3,figsize=(13,4))
  for n,(kind,m) in enumerate(model['models'].items()):
   for k in [0,1]:axs[k].errorbar([n],[m['parameters'][k]],yerr=[m['conditional_std'][k]],fmt='o',capsize=3,label=kind)
   axs[2].plot(np.rad2deg(angle),curve(angle,m),label=kind)
  axs[0].axhline(e['authored_truth']['I_eff_kg_m2'],color='black',ls='--');axs[1].axhline(e['authored_truth']['b_nm_s_rad'],color='black',ls='--');axs[2].plot(np.rad2deg(angle),authored,'k--',label='authored native law, post hoc only')
  for ax,title in zip(axs,['I kgm2 (conditional std)','b Nms/rad (conditional std)','Coulomb Nm / observed angles only']):ax.set_title(title);ax.grid(alpha=.2);ax.legend(fontsize=6)
  for ax in axs[:2]:ax.set_xticks([])
  axs[2].set_xlabel('actual observed angle deg')
  fig.suptitle('Post-episode frozen-estimator diagnosis, not an online update or full twin validation')
  fig.tight_layout();fig.savefig(out/('MODEL_COMPARISON_POSTHOC_%02d.png'%post['attempt']),dpi=150);plt.close(fig)
 # Independent native-initial-state simulation, no following measured states.
 for label in dict.fromkeys(data['segments']):
  if not str(label).startswith('VALIDATION'):continue
  ix=np.flatnonzero(data['segments']==label);runs=np.split(ix,np.flatnonzero(np.diff(t[ix])>.01)+1)
  for n,run in enumerate(runs):
   if len(run)<2:continue
   tt=t[run]-t[run[0]];fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
   axes[0].plot(tt,np.rad2deg(data['q'][run]),color='black',label='actual measured state')
   native=data['native_hinge_velocity'][run];axes[1].plot(tt,native,color='black',label='native measured hinge velocity')
   for kind,m in model.get('models',{}).items():
    I,b=m['parameters'][:2]
    if I<=1e-8:continue
    qp=[float(data['q'][run[0]])];vp=[float(native[0])]
    for j in range(1,len(run)):
     dt=tt[j]-tt[j-1];x=np.clip(qp[-1]/np.deg2rad(20),0,1);c=m['parameters'][2] if kind=='constant' else np.array([(1-x)**2,2*x*(1-x),x*x])@m['parameters'][2:]
     momentum=I*vp[-1]+dt*data['torque'][run[j-1]];vel=float(np.sign(momentum)*max(abs(momentum)-dt*c,0)/(I+dt*b));vp.append(vel);qp.append(qp[-1]+dt*vel)
    thin=np.arange(0,len(run),max(1,len(run)//1500));axes[0].plot(tt[thin],np.rad2deg(np.asarray(qp)[thin]),label=kind);axes[1].plot(tt[thin],np.asarray(vp)[thin],label=kind)
   for ax in axes:ax.grid(alpha=.25);ax.legend()
   axes[0].set_ylabel('angle deg');axes[1].set_ylabel('velocity rad/s');axes[1].set_xlabel('whole held-out interval seconds')
   fig.suptitle(str(label)+' / fixed model, measured effort + native initial state only\nNo GT trajectory following; unsupported inertia remains explicitly flagged')
   fig.tight_layout();fig.savefig(out/(str(label)+'_%d.png'%n),dpi=140);plt.close(fig)
print(out)
