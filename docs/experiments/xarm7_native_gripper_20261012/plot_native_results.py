import json,collections
from pathlib import Path
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/xarm7_native_gripper_20261012';old=ROOT/'results/xarm7_shared_doors_20261012'
fig,axs=plt.subplots(1,3,figsize=(14,4.2))
for folder,label in [(old/'45134/physical_01','virtual PiPER hand'),(O/'45134/physical_01','native xArm G1')]:
 rows=[json.loads(l) for l in (folder/'live.jsonl').read_text().splitlines() if l.strip()]
 r=json.loads((folder/'report.json').read_text());start=r['grasp_acquisition']['t'];a=[x for x in rows if x['t']>=start];t=np.array([x['t']-start for x in a]);angle=np.array([x['door_angle_deg'] for x in a]);axs[0].plot(t,angle,label=label)
 for n,style in [('gripper_link1','-'),('gripper_link2','--')]:axs[1].plot(angle,[x['forces_n'][n] for x in a],style,label=label+' '+('left' if n.endswith('1') else 'right'))
 axs[2].plot(angle,[x['margin_rad'] for x in a],label=label)
axs[0].axhline(30,color='gray',linestyle=':');axs[0].set(xlabel='Simulation time after grasp (s)',ylabel='Actual opening (deg)')
axs[1].set(xlabel='Actual opening (deg)',ylabel='Raw finger/handle load (N)')
axs[2].set(xlabel='Actual opening (deg)',ylabel='Arm joint margin (rad)')
for ax in axs:ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('45134: actual contact execution / 1 Hz live-log visualization; metrics use full 240 Hz')
fig.tight_layout();fig.savefig(O/'45134_NATIVE_VIRTUAL_COMPARISON.png',dpi=150)
