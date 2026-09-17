import csv,gzip,json,sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(Path(__file__).resolve().parent))
OUT=ROOT/'results/settling_regrasp_summary';OUT.mkdir(parents=True,exist_ok=True)

def relative(rows):
    hp=np.array([x['tcp'] for x in rows]);op=np.array([x['box'] for x in rows])
    hq=Rotation.from_quat(np.array([x['tcp_quat'] for x in rows])[:,[1,2,3,0]])
    oq=Rotation.from_quat(np.array([x['box_quat'] for x in rows])[:,[1,2,3,0]])
    return hq.inv().apply(op-hp),hq.inv()*oq

fig,axes=plt.subplots(2,2,figsize=(11,7),sharex='col')
for col,name in enumerate(['bowl','mug']):
    result=json.loads((ROOT/f'results/settling_diagnostic/formal/{name}_settling_r00.json').read_text())
    rows=[x for x in json.load(gzip.open(result['trace']['path']))['records'] if x['phase']=='MICRO_LIFT']
    t=np.array([x['t'] for x in rows]);t-=t[0];p,q=relative(rows)
    trans=np.linalg.norm(p-p[0],axis=1)*1000;rot=(q[0].inv()*q).magnitude()*180/np.pi
    speed=np.r_[np.nan,np.linalg.norm(np.diff(p,axis=0),axis=1)/np.diff(t)*1000]
    angular=np.r_[np.nan,np.abs(np.diff(rot)/np.diff(t))]
    axes[0,col].plot(t,trans,label='relative translation mm');axes[0,col].plot(t,rot,label='relative rotation deg')
    axes[1,col].plot(t,speed,label='translation speed mm/s');axes[1,col].plot(t,angular,label='angular speed deg/s')
    axes[0,col].set_title(name+' | '+result['category']);axes[1,col].set_xlabel('sim time after micro start (s)')
    for ax in axes[:,col]:ax.grid(alpha=.25);ax.legend(fontsize=8)
axes[0,0].set_ylabel('cumulative motion');axes[1,0].set_ylabel('instantaneous speed')
fig.tight_layout();fig.savefig(OUT/'settling_dynamics.png',dpi=170);plt.close(fig)

summary=json.loads((OUT/'summary.json').read_text())
names=['soup','banana','bowl','mug'];x=np.arange(4);width=.35
fig,ax=plt.subplots(figsize=(8,4.5));ax.bar(x-width/2,[summary['stats'][n]['A']['success']/15 for n in names],width,label='A: original, one attempt');ax.bar(x+width/2,[summary['stats'][n]['B']['success']/15 for n in names],width,label='B: refinement + regrasp')
ax.set_xticks(x,names);ax.set_ylim(0,1.05);ax.set_ylabel('physical pick success rate');ax.grid(axis='y',alpha=.25);ax.legend();fig.tight_layout();fig.savefig(OUT/'formal_success_rates.png',dpi=170);plt.close(fig)
print('PLOTS_COMPLETE')
