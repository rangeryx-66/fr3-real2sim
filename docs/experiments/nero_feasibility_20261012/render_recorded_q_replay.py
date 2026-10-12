"""Visualization of recorded actual q only: NO new physics, NOT original video."""
from pathlib import Path
import json,gzip,pickle,subprocess,numpy as np,xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from articulated_demo.kinematics import URDFChain
from piper_mobile_demo.model import geometry
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/nero_feasibility_20261012';probe=O/'integration_probe';c=json.loads((O/'NERO_MODEL.json').read_text());names=json.loads((probe/'RUNTIME_MAPPING.json').read_text())['actual_native_joint_order'];chain=URDFChain(c['sim_urdf']);tree=ET.parse(c['sim_urdf']).getroot();meshes={}
for l in tree.findall('link'):
 e=l.find('collision')
 if e is not None:meshes[l.get('name')]=geometry(e,Path(c['sim_urdf']).parent).convex_hull
rows=[]
for p in sorted((probe/'observations_chunks').glob('*.gz')):rows.extend(pickle.load(gzip.open(p,'rb')))
fig=plt.figure(figsize=(9.6,7.2),dpi=100);ax=fig.add_subplot(111,projection='3d');ax.set_xlim(-.55,.75);ax.set_ylim(-.65,.65);ax.set_zlim(-.2,1.);ax.set_box_aspect([1.3,1.3,1.2]);ax.view_init(24,-55);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.set_zlabel('z (m)');artists={}
for n,m in meshes.items():
 color='orange' if n.startswith('gripper') else 'steelblue' if n.startswith('link') else 'gray';a=Poly3DCollection(m.triangles,facecolor=color,alpha=.8,linewidth=.2,edgecolor='gray');ax.add_collection3d(a);artists[n]=a
actual,=ax.plot([],[],[],'ro',label='measured-q TCP');wanted,=ax.plot([],[],[],'gx',label='command-q TCP');ax.legend(loc='upper right');title=fig.suptitle('')
ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg';out=probe/'recorded_q_replay.mp4';v=subprocess.Popen([ff,'-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x720','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23',str(out)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(probe/'replay_ffmpeg.log').open('w'))
for j,r in enumerate(rows[::12]):
 positions=dict(zip(names,r['q']));desired=positions.copy();desired.update(dict(zip(c['arm_joint_names'],r['q_des_arm'])))
 for n,m in meshes.items():
  T=chain.root_to_link(n,positions);points=m.vertices@T[:3,:3].T+T[:3,3];artists[n].set_verts(points[m.faces])
 A=chain.root_to_link('tcp_link',positions)[:3,3];B=chain.root_to_link('tcp_link',desired)[:3,3];actual.set_data_3d([A[0]],[A[1]],[A[2]]);wanted.set_data_3d([B[0]],[B[1]],[B[2]]);title.set_text('RECORDED Q REPLAY / simplified convex hull visuals / NO new physics\n'+r['phase']+' | recorded simulation time %.2f s | NOT original Isaac video'%r['t']);fig.canvas.draw();rgba=np.asarray(fig.canvas.buffer_rgba());v.stdin.write(np.ascontiguousarray(rgba[:,:,:3]).tobytes())
 if j==40:fig.savefig(O/'RECORDED_Q_REPLAY_FRAME.png')
v.stdin.close();v.wait(timeout=30);(probe/'VIDEO_EVIDENCE_LIMITATION.json').write_text(json.dumps({'original_video':'integration_continuous.mp4','original_camera':'misframed ground; robot not usefully visible','recorded_q_replay':'recorded_q_replay.mp4','replay_is_not_original_Isaac_video':True,'replay_uses_actual_saved_q':True,'replay_meshes':'simplified convex hulls; never collision evidence','no_new_simulation_or_door_episode':True},indent=2));print(out)
