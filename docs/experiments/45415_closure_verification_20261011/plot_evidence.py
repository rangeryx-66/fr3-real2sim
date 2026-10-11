"""Static evidence figures from saved native/command records; no controller use."""
from pathlib import Path
import json,pickle,gzip,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/45415_closure_verification_20261011';p=out/'physical_01';old=ROOT/'results/oracle_door_development_20261011/45415/development_02'
def read(folder):
 rows=[]
 for f in sorted(folder.glob('*.gz')):rows.extend(pickle.load(gzip.open(f,'rb')))
 return rows
r=read(p/'observations_chunks');c=read(p/'closure_commands_chunks');before=read(old/'observations_chunks');report=json.loads((p/'report.json').read_text());g=report['grasp_acquisition'];first=min(x['t'] for x in r if max(x['forces_n'].values())>0)
fig,axes=plt.subplots(3,2,figsize=(12,10))
for rows,label in [(before,'old, uncorrected'),(r,'physical fix trial')]:
 x=[a for a in rows if 88<=a['t']<=106]
 axes[0,0].plot([a['t'] for a in x],[a['telemetry']['command_aperture_m']*1000 for a in x],label=label,lw=.7)
axes[0,0].set(title='Position-command continuity (same plan)',ylabel='Aperture command (mm)',xlabel='Simulation time (s)');axes[0,0].legend(fontsize=9)
x=[a for a in r if 88<=a['t']<=106]
for i in range(2):axes[0,1].plot([a['t'] for a in x],[a['telemetry']['pad_load_filtered_n'][i] for a in x],label=['left','right'][i],lw=.75)
axes[0,1].set(title='Bilateral load acquisition',ylabel='Filtered pad load (N)',xlabel='Simulation time (s)');axes[0,1].legend()
t=np.array([a['t'] for a in r]);angle=np.array([a['door_angle_deg'] for a in r]);positions=np.array([a['T_tcp'] for a in r])[:,:3,3];v=np.r_[0,np.linalg.norm(np.diff(positions,axis=0),axis=1)/np.diff(t)*1000]
axes[1,0].plot(t,angle);axes[1,0].axhline(30,ls='--',color='green',lw=.8);axes[1,0].set(title='Actual door angle',ylabel='Angle (degrees)',xlabel='Simulation time (s)')
if g:
 mask=t>=g['t'];axes[1,1].plot(t[mask],v[mask],lw=.6);axes[1,1].axhline(5,ls='--',color='red',lw=.8)
else:axes[1,1].text(.1,.5,'No valid grasp; no loaded-speed metric',transform=axes[1,1].transAxes)
axes[1,1].set(title='Measured loaded TCP velocity',ylabel='World speed (mm/s)',xlabel='Simulation time (s)')
axes[2,0].plot(t,[a['margin_rad'] for a in r]);axes[2,0].axhline(.05,ls='--',color='red',lw=.8);axes[2,0].set(title='Unchanged joint-margin protection',ylabel='Margin (rad)',xlabel='Simulation time (s)')
if g:axes[2,1].plot(t[mask],np.array([a['relative_translation_slip_m']*1000 for a in r])[mask],lw=.75)
axes[2,1].set(title='Handle-relative position drift after valid grasp',ylabel='Drift (mm)',xlabel='Simulation time (s)')
for ax in axes.ravel():
 ax.grid(alpha=.2)
 if g:ax.axvline(g['t'],ls=':',color='gray',lw=.7)
fig.suptitle('45415 / ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT — '+('PHYSICAL 30° PASS' if report['success'] else report['status']),fontsize=13);fig.tight_layout();fig.savefig(out/'PHYSICAL_EVIDENCE.png',dpi=160)
# Do not crop the real pre-contact transient out of the evidence.
fig,ax=plt.subplots(figsize=(9,3));m=(t>=5.35)&(t<=5.7);ax.plot(t[m],v[m]);ax.set(title='Unchanged pre-contact entry transient — zero pad load, report separately',xlabel='Simulation time (s)',ylabel='Measured TCP speed (mm/s)');ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'PRECONTACT_TRANSIENT.png',dpi=150)
