"""Measured/proxy effort windows; no friction optimization or torque inference."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(root):
    root=Path(root);profile=json.loads((root/'effort_profile.json').read_text());samples=[json.loads(s) for s in (root/'effort_samples.jsonl').read_text().splitlines()]
    if not samples:return
    t=np.array([s['t'] for s in samples]);t-=t[0];P=np.array([s['T_ee'] for s in samples])[:,:3,3];distance=np.linalg.norm(P-P[0],axis=1)*1000
    verified=np.array([s.get('full_tangential_measurement',False) for s in samples]);force=np.array([s.get('effective_tangential_force_n',np.nan) if v else np.nan for s,v in zip(samples,verified)])
    fig,axes=plt.subplots(2,1,figsize=(9,5.5),sharex=True)
    axes[0].plot(t,force,lw=.5,label='Verified normal + friction contact force on handle')
    proxy=[]
    for s in samples:
        w=s.get('command_wrench_world');d=s.get('direction_world');proxy.append(np.nan if w is None or d is None else np.dot(w[:3],d))
    axes[0].plot(t,proxy,lw=.7,alpha=.7,label='Command proxy (not measured force)');axes[0].set_ylabel('Tangential force [N]');axes[0].legend(fontsize=8)
    axes[1].plot(t,distance,lw=1.);axes[1].set_ylabel('EE displacement [mm]');axes[1].set_xlabel('Time from probe start [s]')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('Task-opening effort: signed world-force projection; EE motion can include compliance',fontsize=10);fig.tight_layout();fig.savefig(root/'effort_timeseries.png',dpi=160);plt.close(fig)
    records=profile['moving_effort_vs_state'];states=[];means=[];std=[]
    for r in records:
        v=r.get('quasistatic_moving_tangential_force_n')
        if v:states.append(r['start_estimated_state']);means.append(v['mean']);std.append(v['std'])
    if states:
        fig,ax=plt.subplots(figsize=(8,4));ax.errorbar(states,means,yerr=std,fmt='.-',lw=.8,capsize=2);ax.set_xlabel('Estimated articulation state [capture units]');ax.set_ylabel('Measured signed tangential force [N]');ax.set_title('Low-speed window mean ± within-window std (not parameter uncertainty)');ax.grid(alpha=.2);fig.tight_layout();fig.savefig(root/'moving_effort_vs_state.png',dpi=160);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);plot(p.parse_args().capture)
