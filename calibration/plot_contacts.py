"""Visualize GT mesh and recorded physical contacts in the actual TCP frame."""
import argparse,json,gzip
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('result');p.add_argument('output');a=p.parse_args();path=Path(a.result)
r=json.loads(path.read_text());d=json.load(gzip.open(path.with_suffix('.trace.json.gz')));m=np.load(ROOT/f"assets/arena_complex/{r['target']}_mesh.npz");rows=d['records']
phases=[v for v in ['CLOSE_HOLD','MICRO_LIFT','HOLD'] if any(x['phase']==v for x in rows)]
fig=plt.figure(figsize=(5*len(phases),5))
for i,phase in enumerate(phases):
    v=[x for x in rows if x['phase']==phase][-1];H=Rotation.from_quat(np.roll(v['tcp_quat'],-1));O=Rotation.from_quat(np.roll(v['box_quat'],-1));vertices=H.inv().apply(O.apply(m['vertices'])+v['box']-np.array(v['tcp']))*1000
    ax=fig.add_subplot(1,len(phases),i+1,projection='3d');faces=vertices[m['triangles']];step=max(1,len(faces)//8000)
    ax.add_collection3d(Poly3DCollection(faces[::step],facecolor='#dbc28e',edgecolor='none',alpha=.5))
    for f,finger in enumerate(v['finger_contacts']):
        q=v['q'][d['names'].index(f'fr3_finger_joint{f+1}')]*1000;y=(1 if f==0 else -1)*q
        pad=np.array([[-8.75,y,-9],[8.75,y,-9],[8.75,y,9.5],[-8.75,y,9.5]])
        ax.add_collection3d(Poly3DCollection([pad],facecolor=['#3178bb','#bf4e43'][f],alpha=.65))
        if finger['points_world_m']:
            pts=H.inv().apply(np.array(finger['points_world_m'])-v['tcp'])*1000;ns=H.inv().apply(finger['normals_world']);force=np.array(finger['normal_force_N'])
            ax.scatter(*pts.T,s=np.clip(force*7,10,100),c=['blue','red'][f]);ax.quiver(*pts.T,*ns.T,length=8,color=['blue','red'][f])
    center=(vertices.max(0)+vertices.min(0))/2;rad=max(np.ptp(vertices,axis=0).max()/2,45)
    ax.set(xlim=(center[0]-rad,center[0]+rad),ylim=(center[1]-rad,center[1]+rad),zlim=(center[2]-rad,center[2]+rad),xlabel='TCP x (mm)',ylabel='TCP y (mm)',zlabel='TCP z (mm)',title=phase);ax.set_box_aspect((1,1,1));ax.view_init(22,-55)
fig.suptitle(f"{r['id']} | {r['category']} | measured contact points / normals")
fig.tight_layout();fig.savefig(a.output,dpi=160);plt.close(fig)
