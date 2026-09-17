"""Figures from recorded poses and physics telemetry, never synthesized outcomes."""
import gzip
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle
from scipy.spatial.transform import Rotation
from hand_geometry import SIZE
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/contact_margin1';OUT=ROOT/'results/contact_figures';OUT.mkdir(exist_ok=True)
fig,axes=plt.subplots(1,3,figsize=(12,4),constrained_layout=True)
for ax,seed in zip(axes,[0,3,18]):
    r=json.loads((RUN/f'B_seed_{seed:04d}.json').read_text());g=r['telemetry']['commanded_geometry'];T=np.array(g['T_TCP_target']);axis=g['closing_axis_target_axis'];other=[i for i in range(3) if i!=axis]
    for side,color in [(-1,'#4776d0'),(1,'#dc6756')]:
        points=[]
        for u,v in [(-1,-1),(1,-1),(1,1),(-1,1)]:
            p=np.zeros(3);p[axis]=side*SIZE[axis]/2;p[other]=[u*SIZE[other[0]]/2,v*SIZE[other[1]]/2];points.append((T[:3,:3]@p+T[:3,3])[[0,2]]*1000)
        ax.add_patch(Polygon(points,facecolor=color,alpha=.3,edgecolor=color,label=f'Target face {side:+d}'))
    ax.add_patch(Rectangle((-8.75,-9),17.5,18.5,fill=False,lw=2,color='#111111',label='Franka pad'))
    ax.plot(0,.25,'k+',ms=10);ax.set(xlim=(-40,40),ylim=(-35,45),xlabel='TCP x (mm)',ylabel='TCP z (mm)',aspect='equal')
    ax.set_title(f"Seed {seed} / {r['category']}\nMinimum pad coverage {100*g['min_pad_coverage']:.2f}%",fontsize=10);ax.grid(alpha=.15)
axes[0].legend(fontsize=7,loc='lower left');fig.suptitle('Commanded geometry: opposing target faces projected onto the actual pad rectangle')
fig.savefig(OUT/'pad_coverage.png',dpi=180);plt.close(fig)
fig,axes=plt.subplots(3,2,figsize=(11,8),sharex='col',constrained_layout=True)
for col,seed in enumerate([0,3]):
    r=json.loads((RUN/f'B_seed_{seed:04d}.json').read_text());d=json.load(gzip.open(r['trace']['path'],'rt'));samples=d['records'];t0=next(x['t'] for x in samples if x['phase']=='CLOSE');samples=[x for x in samples if x['t']>=t0];t=np.array([x['t']-t0 for x in samples]);idx=[d['names'].index('fr3_finger_joint'+str(i)) for i in [1,2]]
    axes[0,col].plot(t,[sum(x['q'][i] for i in idx)*1000 for x in samples],label='Actual width');axes[0,col].plot(t,[sum(x['command_q'][i] for i in idx)*1000 for x in samples],'--',label='Commanded width');axes[0,col].set_ylabel('Width (mm)');axes[0,col].set_title(f"Seed {seed}: {r['category']}");axes[0,col].legend(fontsize=8)
    for i in range(2):axes[1,col].plot(t,[x['forces'][i] for x in samples],label=['Left','Right'][i])
    axes[1,col].set_ylabel('Finger-target force (N)');axes[1,col].legend(fontsize=8)
    axes[2,col].plot(t,[(x['box'][2]-r['initial_target']['position'][2])*100 for x in samples]);axes[2,col].axhline(8,color='gray',ls='--',label='Required 8 cm');axes[2,col].set(ylabel='Actual target rise (cm)',xlabel='Seconds from CLOSE',ylim=(-.5,11));axes[2,col].legend(fontsize=8)
    for ax in axes[:,col]:ax.grid(alpha=.2)
fig.suptitle('1 mm margin, original selection: measured physics, unchanged success criteria');fig.savefig(OUT/'contact_physics.png',dpi=180);plt.close(fig)
print(OUT)
