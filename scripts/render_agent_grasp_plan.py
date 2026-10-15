"""Read-only observed surfaces and planned native fingers; no object GT."""
import argparse,json
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from xarm7_native_gripper import NativeGripperGeometry,FINGER_BODIES

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
    plan=json.loads((a.run/'visual_plan.json').read_text());T=np.asarray(plan['T']);P=np.load(a.run/'agent_geometry/points_world.npy');Q=(P-T[:3,3])@T[:3,:3]*1000
    geo=NativeGripperGeometry();fig,axes=plt.subplots(1,2,figsize=(12,5));width=float(plan['contact_geometry_audit']['robust_contact']['chosen_expected_aperture_m'])
    for ax,(i,j) in zip(axes,[(1,2),(0,2)]):
        ax.scatter(Q[:,i],Q[:,j],s=.4,c='#167bca',label='Measured wrist points')
        for w,alpha,label in [(.084,.25,'Open 84mm'),(width,.8,'Ranked aperture %.2fmm'%(width*1000))]:
            values=geo.values(geo.angle(w));root_to_tcp=geo.chain.root_to_link('tcp_link',values)
            poses={n:np.linalg.inv(root_to_tcp)@geo.chain.root_to_link(n,values) for n in FINGER_BODIES}
            for side,color in zip(FINGER_BODIES,['#df7b12','#a41bc7']):
                vertices=np.asarray(geo.meshes[side].vertices);X=poses[side]
                vertices=(vertices@X[:3,:3].T+X[:3,3])*1000
                ax.scatter(vertices[:,i],vertices[:,j],s=.6,color=color,alpha=alpha,label=side+' '+label)
        ax.scatter([0],[0],marker='+',color='black');ax.axis('equal');ax.set(xlabel='TCP axis %d mm'%i,ylabel='TCP axis %d mm'%j);ax.legend(fontsize=6)
    fig.suptitle('Fresh wrist geometry + automatic native G1 plan; rear remains assumed; no GT');fig.tight_layout();fig.savefig(a.run/'AGENT_GRASP_PLAN_INSPECTION.png',dpi=160)
