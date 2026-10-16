"""Derived plot only; raw RGB-D is never exported by this script."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
f=a.root/'live/45936/episode/agent_geometry';d=json.loads((f/'observed_geometry.json').read_text());axis=np.asarray(d['axis_world']);normal=np.asarray(d['normal_world']);side=np.cross(normal,axis);origin=np.median(np.load(f/'points_world.npy'),axis=0)
bounds=(np.load(f/'points_world.npy')-origin)@axis*1000
ylim=(float(bounds.min()-10),float(bounds.max()+10))
fig,axs=plt.subplots(1,3,figsize=(11,4.6))
for ax,folder,name,color in [(axs[0],a.root/'offline/first_seed_replay','Frozen first-seed fusion','#c0752b'),(axs[1],f,'Live incremental fusion','#177ca6')]:
 P=np.load(folder/'points_world.npy');q=(P-origin)@np.column_stack((axis,side,normal))*1000
 ax.scatter(q[:,1],q[:,0],s=.6,c=color);ax.set(title=name,xlabel='Transverse (mm)',ylabel='Longitudinal (mm)',ylim=ylim,xlim=(-12,12));ax.grid(alpha=.2)
views=d['incremental_feedback']['views'];axs[2].plot(range(len(views)),[x['accumulated_extent_mm'] for x in views],'-o',color='#177ca6')
for i,v in enumerate(views):axs[2].annotate('%.2f mm\n+%d voxels'%(v['accumulated_extent_mm'],v['newly_observed_voxels']),(i,v['accumulated_extent_mm']),xytext=(0,-32 if i else 8),textcoords='offset points',ha='center',fontsize=8)
axs[2].set(title='Fresh-view contribution',xlabel='Observation index',ylabel='Accumulated measured extent (mm)',xticks=[0,1,2],ylim=(50,330),xlim=(-.35,2.35));axs[2].grid(alpha=.2)
fig.suptitle('Same fresh wrist RGB-D: newly exposed surfaces survive beyond the first footprint\nObservation/planning only; no physical grasp or opening',fontsize=11);fig.tight_layout();fig.savefig(a.output,dpi=150);plt.close(fig)
