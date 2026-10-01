"""Plot measured door motion, contact and margins; distinguish historical trials."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--strict-replay-rejected',action='store_true');a=p.parse_args()
r=json.loads((a.folder/'report.json').read_text());rows=json.loads((a.folder/'observations.json').read_text())
t=np.array([v['t'] for v in rows]);active=next((v['t'] for v in rows if v['phase']=='OPEN_DOOR'),t[-1]);select=t>=max(0,active-4);t=t[select]-active;sample=[v for i,v in enumerate(rows) if select[i]]
fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True)
axes[0].plot(t,[v['door_angle_deg'] for v in sample]);axes[0].axhline(20,color='gray',ls='--');axes[0].set_ylabel('Actual door angle (deg)')
force=np.array([v['forces_n'] for v in sample]);axes[1].plot(t,force[:,0],label='finger 1');axes[1].plot(t,force[:,1],label='finger 2');axes[1].legend();axes[1].set_ylabel('Measured target contact (N)')
axes[2].plot(t,[v['margin_rad'] for v in sample]);axes[2].axhline(.05,color='red',ls='--');axes[2].set_ylabel('Minimum joint margin (rad)');axes[2].set_xlabel('Time from opening command (s)')
for ax in axes:ax.grid(alpha=.3)
title=f"PiPER fixed base: {r['status']}"
if a.strict_replay_rejected:title+='\nDIAGNOSTIC ONLY: strict mesh replay rejects closure; not a valid grasp'
fig.suptitle(title);fig.tight_layout();fig.savefig(a.folder/'trajectory.png',dpi=150)
