"""Private Agent plan board; observed geometry and official robot only, no GT object."""
import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from xarm7_native_gripper import NativeGripperGeometry,FINGER_BODIES
p=Path(sys.argv[1]);plan=json.loads((p/'visual_plan.json').read_text());g=p/'agent_geometry'
Q=np.load(g/'contact_supported_points_world.npy');Q=Q[::max(1,len(Q)//5000)]
T=np.asarray(plan['T']);hand=NativeGripperGeometry();width=float(sys.argv[2]) if len(sys.argv)>2 else .015
values=hand.values(hand.angle(width));D=T@np.linalg.inv(hand.chain.root_to_link('tcp_link',values))
fig=plt.figure(figsize=(12,5));axes=[fig.add_subplot(121,projection='3d'),fig.add_subplot(122,projection='3d')]
for ax in axes:
 ax.scatter(*Q.T,s=.5,c='#308f70',label='measured contact-supported RGB-D')
 for n,col in zip(FINGER_BODIES,['#397ac3','#df7c29']):
  M=hand.meshes[n].copy();M.apply_transform(D@hand.chain.root_to_link(n,values));ax.plot_trisurf(*M.vertices.T,triangles=M.faces,color=col,alpha=.35)
 ax.scatter(*T[:3,3],c='red',s=20);ax.set(xlabel='World X (m)',ylabel='World Y (m)',zlabel='World Z (m)')
 center=np.median(Q,axis=0);ax.set_xlim(center[0]-.09,center[0]+.09);ax.set_ylim(center[1]-.1,center[1]+.12);ax.set_zlim(center[2]-.18,center[2]+.18)
axes[0].view_init(20,-70);axes[1].view_init(15,20)
fig.suptitle('Fresh automatic grasp | measured surfaces + official G1 | display aperture %.1f mm; unknown rear omitted'%(1000*width));fig.tight_layout();fig.savefig(p/'agent_grasp_board.png',dpi=130);plt.close(fig)
print(p/'agent_grasp_board.png')
