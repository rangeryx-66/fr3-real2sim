"""Private 240Hz raw/compensated traces; frames are deliberately separate."""
from pathlib import Path
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg');matplotlib.rcParams['path.simplify']=False
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('packet',type=Path);a=p.parse_args();d=np.load(a.packet/'measured_effort.npz',allow_pickle=True)
fig,axes=plt.subplots(6,2,figsize=(15,12),sharex=True)
for j in range(6):
 axes[j,0].plot(d['t'],d['raw'][:,j],lw=.45,color='#285b91')
 axes[j,1].plot(d['t'],d['compensated_world'][:,j],lw=.45,color='#b16120')
 for ax in axes[j]:ax.grid(alpha=.25);ax.set_ylabel(['Fx N','Fy N','Fz N','Mx Nm','My Nm','Mz Nm'][j])
axes[0,0].set_title('RAW native parent-on-tool / local sensor frame')
axes[0,1].set_title('Robot-on-object / world axes / moment at SENSOR origin')
for ax in axes[-1]:ax.set_xlabel('simulation time s; all native 240Hz samples, no averaging')
fig.suptitle('Fixed-interface wrist F/T: raw and bias+gravity+inertia compensated\nFull wrench; hinge projection is a separate operation. Idealized sensor, not hardware precision.')
fig.tight_layout();fig.savefig(a.packet/'RAW_COMPENSATED_WRENCH.png',dpi=140);plt.close(fig)
