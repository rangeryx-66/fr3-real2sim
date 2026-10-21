"""Post-seal derived plots only; the online estimator is never rerun/refit."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from xarm_ft_v7_estimator import basis,samples,prediction
from update_probing_v7_model import load_segments

def forward(piece,m):
    edge=piece['processing']['boundary_exclusion_samples'];ix=np.arange(edge,len(piece['t'])-edge)
    t=piece['t'][ix];q=piece['q'][ix];v=piece['native_qd'][ix];tau=piece['torque'][ix];mass,b=m['parameters'][:2]
    qp=[float(q[0])];vp=[float(v[0])]
    for j in range(1,len(t)):
        z=qp[-1];C=float(basis([z],m['domain_native'],m['intervals'])[0,m['active_controls']]@m['parameters'][2:]) if m['intervals'] else m['parameters'][2]
        dt=t[j]-t[j-1];p=mass*vp[-1]+dt*tau[j-1];vel=float(np.sign(p)*max(abs(p)-dt*C,0)/(mass+dt*b));vp.append(vel);qp.append(z+dt*vel)
    check=prediction(piece,m)
    if check.get('status')!='predicted' or abs(np.sqrt(np.mean((np.array(qp)-q)**2))-check['coordinate_rmse'])>1e-12:
        raise ValueError('Derived curve differs from frozen production forward prediction')
    return t-t[0],q,np.array(qp)

def plot(batch):
    batch=Path(batch)
    if not (batch/'TRUTH_ACCESS.json').exists():raise ValueError('Post-batch evaluation must finish first')
    rows=json.loads((batch/'public_derived/METRICS.json').read_text())['rows'];out=batch/'public_derived';x=np.arange(8);labels=[r['episode_id'].removeprefix('v74_') for r in rows]
    fig,axes=plt.subplots(2,2,figsize=(13,8))
    axes[0,0].bar(x-.17,[r['pull_travel_m']*1000 for r in rows],.32,label='Actual pull net travel');axes[0,0].bar(x+.17,[r['reverse_travel_m']*1000 for r in rows],.32,label='Actual reverse net travel');axes[0,0].set_ylabel('mm');axes[0,0].legend(fontsize=7)
    for ax,key,title in [(axes[0,1],'mass_error_percent','Mass error vs authored (%)'),(axes[1,0],'damping_error_percent','Damping error vs authored (%)')]:
        for j,r in enumerate(rows):
            m=r.get('comparison',{}).get('selected',{});value=m.get(key)
            if value is None:ax.text(j,0,'N/A',ha='center',fontsize=7)
            else:ax.plot(j,value,'o',c='tab:blue')
        ax.axhline(0,color='gray',lw=.7);ax.set_ylabel(title)
    ax=axes[1,1]
    for j,r in enumerate(rows):
        held=r.get('comparison',{}).get('selected',{}).get('heldout_motion',{});vals=[v['coordinate_rmse']*1000 for v in held.values() if v.get('status')=='predicted']
        if vals:ax.plot(np.repeat(j,len(vals)),vals,'o',c='tab:blue')
        else:ax.text(j,0,'N/A',ha='center',fontsize=7)
    ax.set_ylabel('Independent free position RMSE (mm)')
    for ax in axes.flat:ax.set_xticks(x,labels,rotation=35,ha='right',fontsize=8);ax.grid(axis='y',alpha=.2)
    fig.suptitle('V7.4 blind dynamics — all eight registered slots, failures retained');fig.tight_layout();fig.savefig(out/'benchmark_summary.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,4,figsize=(15,7))
    for r,ax in zip(rows,axes.flat):
        ax.set_title(r['episode_id'],fontsize=9);ax.set(xlabel='Drawer coordinate (mm)',ylabel='Resistance (N)')
        if not r.get('online_model'):
            ax.text(.5,.5,'No qualified model',transform=ax.transAxes,ha='center');continue
        m=r['online_model']['selected'];law=r['authored']['resistance_law']
        for lo,hi in m['support_intervals_native']:
            q=np.linspace(lo,hi,201);B=basis(q,m['domain_native'],m['intervals'])[:,m['active_controls']] if m['intervals'] else np.ones((len(q),1));C=B@m['parameters'][2:]
            cov=np.array(m['conditional_covariance'])[2:,2:];sd=np.sqrt(np.maximum(np.einsum('ij,jk,ik->i',B,cov,B),0));truth=law['base']+law['amplitude']*np.exp(-.5*((q-law['center'])/law['width'])**2)
            ax.plot(q*1000,truth,'k--',label='Authored post-hoc');ax.plot(q*1000,C,c='tab:blue',label='Frozen online');ax.fill_between(q*1000,C-sd,C+sd,alpha=.2,color='tab:blue')
        ax.grid(alpha=.2)
    fig.suptitle('Resistance only on measured FIT support; shading = conditional 1 SD, not total physical confidence');fig.tight_layout();fig.savefig(out/'resistance_support.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(4,2,figsize=(12,13))
    for r,ax in zip(rows,axes.flat):
        ax.set_title(r['episode_id'],fontsize=9);ax.set(xlabel='Retained prediction time (s)',ylabel='Drawer (mm)')
        if not r.get('online_model') or not r['complete_validation']:
            ax.text(.5,.5,'No complete independent VALIDATION',transform=ax.transAxes,ha='center',fontsize=8);continue
        pieces,settings,_=load_segments(batch/'episodes'/r['episode_id'],True);m=r['online_model']['selected']
        for d in pieces:
            if not d['segments'][0].startswith('VALIDATION'):continue
            samples(d,settings['estimation']['domain_native'],settings['estimation']['intervals'])
            if prediction(d,m).get('status')!='predicted':continue
            t,q,qp=forward(d,m);ax.plot(t,q*1000,c='k',label='Measured');ax.plot(t,qp*1000,'--',label='Free forward, one initial state');ax.text(.02,.02,f'{t[-1]:.3f}s / {len(t)} samples',transform=ax.transAxes,fontsize=7)
        ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('Independent held-out wrist-effort-driven predictions; no later state correction; SG boundaries excluded');fig.tight_layout();fig.savefig(out/'heldout_forward.png',dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('batch',type=Path);plot(p.parse_args().batch)
