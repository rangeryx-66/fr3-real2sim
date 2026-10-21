"""Derived engineering comparison, with online and replay results kept distinct."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plot(source, output):
    data=json.loads(Path(source).read_text())
    rows=[next(r for r in data['cases'] if r['id'].startswith(a) and r['model_online']) for a in ['45746','27044','19179','45290']]
    x=np.arange(4);fig,axs=plt.subplots(2,2,figsize=(11,7))
    for i,(ax,key,label,ref) in enumerate([(axs[0,0],0,'Effective mass (kg)',2.),(axs[0,1],1,'Damping (N s/m)',3.)]):
        ax.errorbar(x,[r['model_online']['parameters'][key] for r in rows],yerr=[r['model_online']['conditional_std'][key] for r in rows],fmt='o',color='tab:blue',capsize=4,label='Actual online estimate (conditional 1 SD)')
        for j,r in enumerate(rows):
            m=r.get('qualified_offline_replay_not_online_replacement')
            if m:ax.plot(j,m['parameters'][key],marker='D',markerfacecolor='none',color='tab:orange',label='Qualified offline replay' if j==0 else None)
        ax.axhline(ref,c='gray',ls='--',label='Post-episode authored reference');ax.set_ylabel(label)
    ax=axs[1,0]
    for j,r in enumerate(rows):
        m=r['model_online'];v=next(iter(m['heldout'].values()));ax.plot(j,1000*v['coordinate_rmse'],'o',color='tab:blue')
        m=r.get('qualified_offline_replay_not_online_replacement')
        if m:ax.plot(j,1000*next(iter(m['heldout'].values()))['coordinate_rmse'],marker='D',markerfacecolor='none',color='tab:orange')
    ax.set_ylabel('Independent forward position RMSE (mm)');ax.set_yscale('log')
    ax=axs[1,1]
    pull=[r['attempts'][0]['segments'][0]['gain_native']*1000 for r in rows];reverse=[-r['attempts'][0]['segments'][1]['gain_native']*1000 for r in rows]
    ax.bar(x-.16,pull,.3,label='Pull');ax.bar(x+.16,reverse,.3,label='Reverse');ax.set_ylabel('Actual physical travel (mm)');ax.legend(fontsize=8)
    for ax in axs.flat:ax.set_xticks(x,[r['id'].split('_')[0] for r in rows]);ax.grid(axis='y',alpha=.2)
    axs[0,0].legend(fontsize=7,loc='lower left');fig.suptitle('V7.3 short engineering controls — not a blind benchmark success rate')
    fig.text(.5,.012,'45746/27044: earlier online worker + separately labelled qualified replay. 19179/45290: qualified worker online.\nUncertainty excludes compensation/contact/model-selection systematics. All actions and raw evidence preserved.',ha='center',fontsize=8)
    fig.tight_layout(rect=[0,.06,1,.95]);fig.savefig(output,dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');a=p.parse_args();plot(a.source,a.output)
