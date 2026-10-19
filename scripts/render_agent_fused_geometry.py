"""Read-only fused geometry inspection, measured points versus declared priors."""
import argparse,json
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args()
    d=json.loads((a.folder/'observed_geometry.json').read_text());P=np.load(a.folder/'points_world.npy');axis=np.asarray(d['axis_world']);n=np.asarray(d['normal_world']);F=np.column_stack([axis,np.cross(n,axis),n]);center=np.median(P,axis=0);Q=(P-center)@F*1000
    fig,axes=plt.subplots(1,2,figsize=(12,4.5));axes[0].scatter(Q[:,0],Q[:,2],s=.5,c='#167bca',label='Fused measured points');axes[0].set(xlabel='Observed longitudinal axis (mm)',ylabel='Outward (mm)',title='Measured open surface; no GT');axes[0].legend()
    keep=abs(Q[:,0])<20;axes[1].scatter(Q[keep,1],Q[keep,2],s=1,c='#167bca',label='Measured middle +/-20mm')
    for i,scale in enumerate([.75,1.,1.25]):
        mesh=trimesh.load(a.folder/('uncertain_rear_%d.ply'%i),process=False)
        lines=trimesh.intersections.mesh_plane(mesh,axis,center)
        for j,line in enumerate(lines):
            q=(line-center)@F*1000;axes[1].plot(q[:,1],q[:,2],color='#da8316',alpha=.7,linestyle=[':', '-', '--'][i],label=str(scale)+'x assumed rear' if j==0 else None)
    axes[1].set(xlabel='Transverse (mm)',ylabel='Outward (mm)',title='Blue measured / orange hypothetical, not solid truth');axes[1].axis('equal');axes[1].legend(fontsize=7);fig.tight_layout();fig.savefig(a.folder/'AGENT_FUSED_INSPECTION.png',dpi=160)
