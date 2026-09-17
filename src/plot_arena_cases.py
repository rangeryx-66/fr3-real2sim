"""Stored pointcloud, original top-K, selected pose, and measured contact figures."""
import json,gzip
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/arena_complex40';OUT=ROOT/'results/arena_complex_summary/figures';OUT.mkdir(exist_ok=True,parents=True)
for path in sorted(RUN.glob('B_seed*.json')):
    r=json.loads(path.read_text());seed=r['seed'];cloudpath=RUN/f'inputs/seed_{seed:04d}_cloud.npz';grasppath=RUN/f'inputs/seed_{seed:04d}_grasps.json'
    if not cloudpath.exists() or not grasppath.exists():continue
    cloud=np.load(cloudpath);raw=json.loads(grasppath.read_text());T=np.array(raw['T_B_C']);p=cloud['points']@T[:3,:3].T+T[:3,3];mask=cloud['mask'];ids=np.arange(0,len(p),max(1,len(p)//9000))
    fig=plt.figure(figsize=(15,5));ax=fig.add_subplot(131);ax.imshow(cloud['rgb']);ax.axis('off');ax.set_title(f'{r["target_class"]}, seed {seed}\nActual camera RGB')
    for panel,only_selected in [(132,False),(133,True)]:
        ax=fig.add_subplot(panel,projection='3d');ax.scatter(*p[ids].T,c=np.where(mask[ids],'#e9a23b','#9da3ab'),s=1,alpha=.5)
        for g in raw['grasps']:
            if only_selected and g['rank']!=r['selected_rank']:continue
            R=T[:3,:3]@g['rotation'];t=T[:3,:3]@g['translation']+T[:3,3];w=g['width'];d=g['depth']
            col='#15a167' if g['rank']==r['selected_rank'] else '#3473bc'
            lines=[[[-.02,-w/2,0],[d,-w/2,0]],[[-.02,w/2,0],[d,w/2,0]],[[-.02,-w/2,0],[-.02,w/2,0]]]
            for line in lines:
                pts=np.array(line)@R.T+t;ax.plot(*pts.T,c=col,lw=2 if only_selected else .8)
        if only_selected:
            trace=RUN/f'trace_seed_{seed:04d}.json.gz'
            if trace.exists():
                records=json.load(gzip.open(trace,'rt'))['records'];contactseq=[v for v in records if v['phase']=='CLOSE']
                for finger,col in [(0,'red'),(1,'blue')]:
                    cp=[x for v in contactseq[::12] for x in v['finger_contacts'][finger]['points_world_m']]
                    if cp:ax.scatter(*np.array(cp).T,c=col,s=7,label=f'finger {finger+1}')
        ax.set_xlim(.25,.75);ax.set_ylim(-.25,.25);ax.set_zlim(0,.3);ax.set_box_aspect((1,1,.6));ax.view_init(40,-65)
        ax.set_xlabel('base x / m');ax.set_ylabel('base y / m');ax.set_zlabel('z / m')
        ax.set_title(f'Selected rank {r["selected_rank"]} + CLOSE contacts' if only_selected else f'Unmodified AnyGrasp top-K ({len(raw["grasps"])})')
    fig.suptitle(f'{r["category"]} | contact {r["non_target_contact"]} | disturbance {r["non_target_disturbance"]}');fig.tight_layout();fig.savefig(OUT/f'seed_{seed}.png',dpi=140);plt.close(fig)
