"""Compact derived traction figure; preserves all recorded samples and force peaks."""
import argparse,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns

def plot(folders,output):
    fig,axes=plt.subplots(4,len(folders),figsize=(6*len(folders),10),squeeze=False)
    for col,folder in enumerate(folders):
        a,index=columns(Path(folder));ids=np.flatnonzero(a['segment']==1)
        if not len(ids):raise ValueError('No physical interaction segment: '+str(folder))
        import json
        prior=json.loads((Path(folder)/'ARTICULATION_INTERFACE.json').read_text())['prior'];axis=np.array(prior['axis_world']);axis/=np.linalg.norm(axis)
        t=a['t'][ids]-a['t'][ids[0]]
        axes[0,col].plot(t,a['pad_load'][ids,0],label='L raw normal');axes[0,col].plot(t,a['pad_load'][ids,1],label='R raw normal');axes[0,col].set_ylabel('N')
        for key,label in [('command_TCP_pose','Command TCP'),('tcp_pose','Measured TCP')]:
            x=a[key][ids,:3]@axis;axes[1,col].plot(t,(x-x[0])*1000,label=label)
        q=a['joint_q'][ids];axes[1,col].plot(t,(q-q[0])*1000,label='Actual drawer');axes[1,col].set_ylabel('Rail travel (mm)')
        f=-a['friction_force_on_fingers_world_N'][ids].sum(1);valid=np.all(a['friction_api_valid'][ids]>0,axis=1)&np.any(a['friction_anchor_counts'][ids]>0,axis=1);f[~valid]=np.nan
        axial=f@axis;cross=np.linalg.norm(f-axial[:,None]*axis,axis=1)
        axes[2,col].plot(t,axial,label='Native friction: rail');axes[2,col].plot(t,cross,label='Native friction: transverse');axes[2,col].set_ylabel('Reaction on object (N)')
        axes[3,col].plot(t,a['command_aperture'][ids]*1000,label='Command aperture');axes[3,col].plot(t,a['aperture'][ids]*1000,label='Measured aperture');axes[3,col].set_ylabel('Jaw aperture (mm)');axes[3,col].set_xlabel('Physical segment elapsed (s)')
        axes[0,col].set_title(Path(folder).name)
        for ax in axes[:,col]:ax.legend(fontsize=7);ax.grid(alpha=.2)
    fig.suptitle('Prismatic transmission: servo tracking and actual TCP/drawer motion are separate\nNative friction anchors are partial diagnostics; point reports are not pressure areas',fontsize=11)
    fig.tight_layout();fig.savefig(output,dpi=150);plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folders',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();plot(a.folders,a.output)
