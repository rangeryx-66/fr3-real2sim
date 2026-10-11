from pathlib import Path
import json,numpy as np,gzip,pickle
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/oracle_door_development_20261011');fig,axs=plt.subplots(2,2,figsize=(12,8))
p=root/'45176/development_01';rows=[json.loads(x) for x in (p/'live.jsonl').read_text().splitlines()];t=[r['t'] for r in rows];a=[r['door_angle_deg'] for r in rows];axs[0,0].plot(t,a);axs[0,0].set(title='45176: correct handle, ineffective transmission',ylabel='Actual angle (deg)',xlabel='Simulation time (s)');axs[0,0].text(.04,.85,'1 Hz archive only; peak observed 1.307°\nFull 240 Hz buffer was not exported',transform=axs[0,0].transAxes,fontsize=10)
for finger in ['gripper_link1','gripper_link2']:axs[1,0].plot(t,[r['forces_n'][finger] for r in rows],label=finger)
axs[1,0].set(title='45176: sustained unilateral contact',ylabel='Pad load, 1 Hz samples (N)',xlabel='Simulation time (s)');axs[1,0].legend(fontsize=9)
p=root/'45415/development_02';rows=[]
for f in sorted((p/'observations_chunks').glob('*.gz')):rows.extend(pickle.load(gzip.open(f,'rb')))
r=[r for r in rows if r['t']>=98];t=[r['t'] for r in r];axs[0,1].plot(t,[r['telemetry']['command_aperture_m']*1000 for r in r],label='command');axs[0,1].plot(t,[r['aperture_m']*1000 for r in r],label='measured',alpha=.8);axs[0,1].set(title='45415: command reopening during centering',ylabel='Jaw opening (mm)',xlabel='Simulation time (s)');axs[0,1].legend()
for ix in [0,1]:axs[1,1].plot(t,[r['telemetry']['pad_load_filtered_n'][ix] for r in r],label=('left','right')[ix])
axs[1,1].axhline(.05,color='red',ls='--',lw=.8,label='existing 0.05 N');axs[1,1].set(title='45415: failed bilateral preload, repeated unchanged',ylabel='Filtered pad load (N)',xlabel='Simulation time (s)');axs[1,1].legend(fontsize=9)
for ax in axs.ravel():ax.grid(alpha=.2)
fig.suptitle('ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT — actual evidence, no new 30° PASS',fontsize=13);fig.tight_layout();fig.savefig(root/'PRESENTATION_EVIDENCE.png',dpi=160)
# Optional all-phase diagnostics for the no-grasp cases, explicitly separated.
for trial in ['development_01','development_02']:
 p=root/'45415'/trial;prev=None;vmax=0.;jmax=0.;steps=0
 for f in sorted((p/'observations_chunks').glob('*.gz')):
  for r in pickle.load(gzip.open(f,'rb')):
   if r['phase'] in ['SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD'] and prev is not None:
    vmax=max(vmax,float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(prev['T_tcp'])[:3,3])/(r['t']-prev['t'])));jmax=max(jmax,float(np.max(abs(np.array(r['telemetry']['joint_velocities'])[:6]))));steps+=1
   prev=r
 (p/'CLOSURE_MOTION_METRICS.json').write_text(json.dumps({'classification':'DIAGNOSTIC_ONLY_OFFLINE','valid_grasp_established':False,'maximum_TCP_speed_during_closure_m_s':vmax,'maximum_arm_joint_speed_during_closure_rad_s':jmax,'closure_observations':steps,'slip_metric_after_grasp':None,'reason':'No valid grasp, so zero-valued relative-slip field is not interpreted as stability.'},indent=2))
