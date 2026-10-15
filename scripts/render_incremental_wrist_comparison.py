"""Derived point-cloud figure only; no RGB/depth/GT assets are exported."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def render(root,output):
    data=json.loads((root/'B/observed_geometry.json').read_text())
    axis=np.asarray(data['axis_world']);normal=np.asarray(data['normal_world'])
    F=np.column_stack((axis,np.cross(normal,axis),normal))
    origin=np.asarray(data['legacy_estimate']['anchor_world_m'])
    A=np.load(root/'A/points_world.npy');B=np.load(root/'B/points_world.npy')
    last=len(data['incremental_feedback']['views'])-1
    delta=np.load(root/'B'/('new_voxel_points_%02d.npy'%last))
    qa=(A-origin)@F*1000;qb=(B-origin)@F*1000;qd=(delta-origin)@F*1000
    fig,axes=plt.subplots(1,3,figsize=(11,5),sharex=True,sharey=True)
    for ax,Q,title,color in zip(axes,[qa,qb,qd],
        ['Frozen first-seed fusion','Independent incremental fusion','New voxels from last view'],
        ['#537b9e','#2a8b66','#da8237']):
        ax.scatter(Q[:,1],Q[:,0],s=.55,c=color)
        ax.set(title=title+'\n%d points'%len(Q),xlabel='Transverse (mm)')
        ax.grid(alpha=.2)
    for ax,Q in zip(axes[:2],[qa,qb]):
        ax.text(.03,.98,'Extent %.2f mm'%np.ptp(Q[:,0]),transform=ax.transAxes,va='top',fontsize=9)
    axes[0].set_ylabel('Longitudinal (mm)')
    axes[0].set_xlim(qb[:,1].min()-2,qb[:,1].max()+2)
    axes[0].set_ylim(qb[:,0].min()-3,qb[:,0].max()+3)
    fig.suptitle(root.name+' | Same archived RGB-D and measured camera poses')
    fig.text(.5,.01,'Measured surfaces only. Unobserved rear remains uncertain. Voxel counts are not surface areas.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.03,1,.95));output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=160);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();render(a.root,a.output)
