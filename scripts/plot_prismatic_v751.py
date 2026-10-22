"""Compact derived plots from sealed evidence; never exports numeric raw streams."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from diagnose_probing_v3 import columns,rows_at
from interaction_runtime.prismatic_unloading_v751 import causal_world_wrench


def plot(episode,output):
    p=Path(episode)
    if not (p/'ONLINE_SEAL_V751.json').exists():raise ValueError('SEALED_EPISODE_REQUIRED')
    a,index=columns(p);settings=json.loads((p/'identification_input.json').read_text())
    axis=np.asarray(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    fig,axs=plt.subplots(4,1,figsize=(11,9),constrained_layout=True)
    start=None
    for j,s in enumerate(index['segments']):
        ix=np.flatnonzero((a['step']>=s['first_step'])&(a['step']<=s['last_step']))
        if not len(ix):continue
        if start is None:start=a['t'][ix[0]] if 't'in a else a['step'][ix[0]]/240
        t=(a['t'][ix]if't'in a else a['step'][ix]/240)-start
        rows=rows_at(a,ix,index['metadata'],'SEALED_FIGURE')
        w=np.array([causal_world_wrench(r,settings['bias'])for r in rows])
        rail=w[:,:3]@axis;transverse=np.linalg.norm(w[:,:3]-rail[:,None]*axis,axis=1)
        axs[0].plot(t,1000*a['joint_q'][ix],label=f"{j+1}: {s['role']} [{s['result'].get('status')}]")
        axs[1].plot(t,rail,color='C0');axs[1].plot(t,transverse,color='C1')
        axs[2].plot(t,np.linalg.norm(w[:,3:],axis=1),color='C2')
        axs[3].plot(t,a['pad_load'][ix,0],color='C3');axs[3].plot(t,a['pad_load'][ix,1],color='C4')
    axs[0].legend(fontsize=7,loc='upper left')
    axs[0].set_ylabel('Drawer (mm)')
    axs[1].set_ylabel('Rail / transverse (N)')
    axs[1].plot([],[],color='C0',label='Signed rail effort');axs[1].plot([],[],color='C1',label='Transverse resultant');axs[1].legend(fontsize=8)
    axs[2].axhline(.6,color='red',ls='--',label='Original sustained guard (.15 s window)');axs[2].legend(fontsize=8)
    axs[2].set_ylabel('Sensor moment (N m)')
    axs[3].set_ylabel('Pad normal (N)');axs[3].set_xlabel('Simulation time from first probing segment (s); gaps include unloaded recovery')
    axs[3].plot([],[],color='C3',label='Left');axs[3].plot([],[],color='C4',label='Right');axs[3].legend(fontsize=8)
    for ax in axs:ax.grid(alpha=.2)
    fig.suptitle(p.name+'\nNative 240 Hz probing samples; causal bias + gravity load diagnostic; estimator separately compensates tool dynamics',fontsize=10)
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(out,dpi=140);plt.close(fig)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episode');ap.add_argument('--output',required=True);args=ap.parse_args();plot(args.episode,args.output)
