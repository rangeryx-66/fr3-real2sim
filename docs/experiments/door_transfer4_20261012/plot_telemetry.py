from pathlib import Path
import json,pickle,gzip,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/door_transfer4_20261012')
fig,axs=plt.subplots(4,4,figsize=(16,10),sharex='row')
for y,ident in enumerate(['45415','45176','45134','45443']):
 p=root/ident/'physical_01';a=[];counter=0
 for f in sorted((p/'observations_chunks').glob('*.gz')):
  for r in pickle.load(gzip.open(f,'rb')):
   if counter%120==0:a.append([r['t'],r['door_angle_deg'],r['relative_translation_slip_m']*1000,*list(r['forces_n'].values()),r['margin_rad']])
   counter+=1
 a=np.asarray(a);t=a[:,0];axs[y,0].plot(t,a[:,1]);axs[y,0].axhline(30,c='k',ls='--',lw=.7);axs[y,1].plot(t,a[:,2]);axs[y,2].plot(t,a[:,3],label='L');axs[y,2].plot(t,a[:,4],label='R');axs[y,3].plot(t,a[:,5]);axs[y,3].axhline(.05,c='k',ls='--',lw=.7)
 for x in range(4):axs[y,x].grid(alpha=.2);axs[y,x].set_xlabel('simulation time (s)')
 axs[y,0].set_ylabel(ident+'\nangle (deg)');axs[y,1].set_ylabel('relative drift (mm)');axs[y,2].set_ylabel('pad load (N)');axs[y,3].set_ylabel('joint margin (rad)')
 axs[y,2].legend(loc='upper right')
 for e in json.loads((p/'CONTACT_ADAPTATION.json').read_text())['events']:
  if e['event']=='CONTACT_TRANSMISSION_STALL':
   for x in range(4):axs[y,x].axvline(e['t'],color='r',alpha=.6,ls=':')
fig.suptitle('Four-case ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT — sampled display of full 240 Hz evidence\nShared implementation frozen before outcomes; dashed angle = 30°, margin = unchanged 0.05 rad')
fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(root/'TELEMETRY_COMPARISON.png',dpi=150)
print(root/'TELEMETRY_COMPARISON.png')
