"""Compact derived contact plot from private measurements; no truth or fitting."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from diagnose_probing_v3 import columns


def plot(episode,output):
    arrays,index=columns(Path(episode))
    ids=np.unique(arrays['segment']);ids=ids[ids>0]
    if not len(ids):raise ValueError('No physical interaction segment')
    chosen=ids[0];i=arrays['segment']==chosen;t=arrays['t'][i];t=t-t[0]
    fig,axes=plt.subplots(4,1,figsize=(8,7),sharex=True)
    axes[0].plot(t,np.rad2deg(arrays['door_q'][i]),label='measured')
    axes[0].plot(t,np.rad2deg(arrays['reference_q'][i]),label='reference')
    axes[0].set_ylabel('Door (deg)')
    for j,name in enumerate(['left normal','right normal']):
        axes[1].plot(t,arrays['pad_load'][i,j],lw=.7,label=name)
    axes[1].set_ylabel('Pad load (N)')
    axes[2].plot(t,1000*arrays['aperture'][i],label='measured')
    axes[2].plot(t,1000*arrays['command_aperture'][i],label='command')
    axes[2].set_ylabel('Aperture (mm)')
    axes[3].plot(t,1000*arrays['relative_slip_m'][i]);axes[3].set_ylabel('Relative (mm)')
    axes[3].set_xlabel('Interaction segment time (s)')
    for ax in axes[:3]:ax.legend(fontsize=7)
    fig.suptitle('V4.3 first opening: positive normal reports persist as transmission stalls',fontsize=10)
    fig.tight_layout();fig.savefig(output,dpi=100);plt.close(fig)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('episode',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();plot(args.episode,args.output)
