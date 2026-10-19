"""Inspection-only close-up of captured points; no executable geometry edit."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from native_g1_observed_regions import cloud

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args()
    info=json.loads((a.folder/'INSPECTION.json').read_text());meta,d,x,y,P,valid=cloud(a.folder)
    center=np.asarray(info['cue_geometry']['center_world_m']);n=np.asarray(info['observed_panel']['normal_world']);axis=np.asarray(info['cue_geometry']['PCA_axes_world'][0]);axis-=n*(axis@n);axis/=np.linalg.norm(axis);side=np.cross(n,axis);F=np.column_stack([axis,side,n])
    context=P[valid&(np.linalg.norm(P-center,axis=2)<.35)];cue=np.load(a.folder/'cue_points_world.npy');path=a.folder/'measured_geometry/preserved_points_world.npy';measured=np.load(path) if path.exists() else np.empty((0,3))
    convert=lambda Q:(Q-center)@F*1000
    fig=plt.figure(figsize=(13,5));ax=fig.add_subplot(121,projection='3d')
    for Q,c,label,size in [(context,'gray','Observed local context',.3),(cue,'#dd8b18','SAM cue points',.5),(measured,'#167bca','Frozen preserved surface',1)]:
        q=convert(Q[::max(1,int(np.ceil(len(Q)/16000)))]);ax.scatter(q[:,0],q[:,2],q[:,1],s=size,c=c,label=label,alpha=.4)
    ax.set(xlabel='Observed handle axis (mm)',ylabel='Outward from cue (mm)',zlabel='Transverse (mm)',title='Actual points only; no completed rear');ax.legend(fontsize=7);ax.view_init(elev=25,azim=-65)
    ax=fig.add_subplot(122)
    for Q,c,label in [(cue,'#dd8b18','SAM cue'),(measured,'#167bca','Frozen preserved surface')]:
        q=convert(Q);keep=abs(q[:,0])<20;ax.scatter(q[keep,1],q[keep,2],s=1,c=c,label=label,alpha=.5)
    ax.set(xlabel='Transverse (mm)',ylabel='Outward from cue (mm)',title='Measured middle section (+/-20mm along axis)');ax.axis('equal');ax.legend()
    fig.suptitle(a.folder.name+' | local inspection only; hidden surface not inferred');fig.tight_layout();fig.savefig(a.folder/'agent_local_geometry.png',dpi=160)
