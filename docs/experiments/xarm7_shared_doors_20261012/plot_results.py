from pathlib import Path
import json,gzip,pickle,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_shared_doors_20261012');fig,ax=plt.subplots(3,2,figsize=(12,9))
for j,a in enumerate(['45134','45443']):
 O=R/a/'physical_01';rows=[]
 for f in sorted((O/'observations_chunks').glob('*.gz')):
  with gzip.open(f,'rb') as s:rows+=pickle.load(s)
 t=np.array([r['t'] for r in rows]);theta=np.array([r['door_angle_deg'] for r in rows]);slip=np.array([r['relative_translation_slip_m'] for r in rows])*1000;forces=np.array([[r['forces_n'][n] for n in ['gripper_link1','gripper_link2']] for r in rows]);ax[0,j].plot(t,theta);ax[0,j].axhline(30,color='black',ls='--');ax[0,j].set_title(a+' actual physical joint angle');ax[0,j].set_ylabel('degrees');ax[1,j].plot(t,slip);ax[1,j].set_ylabel('relative motion (mm)');ax[2,j].plot(t,forces[:,0],label='left raw');ax[2,j].plot(t,forces[:,1],label='right raw');ax[2,j].legend();ax[2,j].set_ylabel('native pad/handle load (N)')
 for i in range(3):ax[i,j].set_xlabel('Runner simulation time (s), plus 1 s startup');ax[i,j].grid(alpha=.25)
fig.suptitle('xArm7 / oracle handle / known model / approximate finite actuator\nOne fixed-base physical attempt per case; raw load samples retained');fig.tight_layout();fig.savefig(R/'PHYSICAL_COMPARISON.png',dpi=160)
