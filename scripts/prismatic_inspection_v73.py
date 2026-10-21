"""Small observed-surface/native-finger plan board, no privileged object mesh."""
import json
from pathlib import Path
import numpy as np


def plan_board(g):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    p=g['a'].output;c=json.loads((p/'visual_plan.json').read_text());m=g['model'];T=np.asarray(c['T'])
    width=c.get('contact_geometry_audit',{}).get('robust_contact',{}).get('chosen_expected_aperture_m',.02)
    P=m.poses(m.home,[0,0,0,0],width=float(width));D=T@np.linalg.inv(P['tcp_link'])
    points=np.load(p/'agent_geometry/contact_supported_points_world.npy');Q=points[::max(1,len(points)//5000)]
    fig=plt.figure(figsize=(10,5))
    for k,(el,az) in enumerate([(20,-65),(15,20)]):
        ax=fig.add_subplot(1,2,k+1,projection='3d');ax.scatter(*Q.T,s=.5,c='green',label='measured supported surfaces')
        for name,color in [('gripper_link1','blue'),('gripper_link2','orange')]:
            mesh=m.robot_hulls[name].copy();mesh.apply_transform(D@P[name]);ax.scatter(*mesh.vertices.T,s=4,c=color,label=name)
        center=T[:3,3];ax.scatter(*center,c='red',s=25)
        for key,v in zip(('xlim','ylim','zlim'),center):getattr(ax,'set_'+key)(v-.065,v+.065)
        ax.set(xlabel='World x m',ylabel='World y m',zlabel='World z m');ax.view_init(el,az)
    fig.suptitle('Automatic native-G1 grasp | measured surfaces only; unknown rear separate');fig.tight_layout()
    output=p/'AGENT_GRASP_PLAN_INSPECTION.png';fig.savefig(output,dpi=120);plt.close(fig);return str(output)
