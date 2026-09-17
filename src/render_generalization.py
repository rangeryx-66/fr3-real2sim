"""Render measured 240 Hz state logs; no inference, planning or new physical trial."""
from pathlib import Path
import os,gzip
source=Path(__file__).with_name('sim_server.py')
exec(compile(source.read_text().split('world.reset()')[0],str(source),'exec'))
import cv2
from scipy.spatial.transform import Rotation
from pxr import UsdLux,Gf
import omni.physx

OUT=ROOT/'videos/generalization';OUT.mkdir(parents=True,exist_ok=True)
light=UsdLux.DomeLight.Define(stage,'/World/video_dome');light.CreateIntensityAttr(700)
key=UsdLux.DistantLight.Define(stage,'/World/video_key');key.CreateIntensityAttr(1000);UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-35,25,15))
def look(cam,pos,at,fov):
 z=np.array(at)-pos;z/=np.linalg.norm(z);x=np.cross(z,[0,0,1]);x/=np.linalg.norm(x);y=np.cross(z,x)
 cam.set_world_pose(position=np.array(pos),orientation=np.roll(Rotation.from_matrix(np.column_stack([x,y,z])).as_quat(),1),camera_axes='ros')
 cam.set_focal_length(cam.get_horizontal_aperture()/(2*np.tan(np.deg2rad(fov/2))));cam.set_clipping_range(.02,10.)
views=[]
for name,res,pos,at,fov in [('overview',(768,600),[1.4,1.3,1.05],[.28,0,.28],65),('closeup',(512,600),[.85,.5,.38],[.49,0,.075],48)]:
 cam=Camera('/World/video_'+name,resolution=res,frequency=30);look(cam,np.array(pos),at,fov);views.append(cam)
world.reset()
for c in views:c.initialize()
for _ in range(45):world.render()
selected=[('clutter50',100),('clutter50',110),('clutter50',149),('pose30',200),('pose30',203),('pose30',229)]
if os.environ.get('FR3_RENDER_ONE'):selected=selected[:1]
manifest=[]
try:
 for group,seed in selected:
  directory=ROOT/'results'/('generalization_'+group);r=json.loads((directory/f'B_seed_{seed:04d}.json').read_text())
  data=json.loads(gzip.open(directory/f'trace_seed_{seed:04d}.json.gz','rt').read());records=data['records']
  for ob,body in zip(r['initial_layout'],clutter.bodies):body.set_world_pose(ob['position'],ob['quaternion_wxyz']);body.set_linear_velocity(np.zeros(3));body.set_angular_velocity(np.zeros(3))
  start=next((i for i,v in enumerate(records) if v['phase']=='PREGRASP'),0)
  samples=[records[max(0,start-1)]]*30+records[start::8]+[records[-1]]*45
  if os.environ.get('FR3_RENDER_TEST'):samples=[records[start],records[len(records)//2],records[-1]]
  writer=cv2.VideoWriter(str(OUT/f'{group}_seed_{seed}_raw.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),30,(1280,720));assert writer.isOpened()
  ix=[data['names'].index(n) for n in robot.dof_names];z0=r['initial_target']['position'][2];coverage=r['telemetry']['commanded_geometry']['min_pad_coverage']
  yaw=Rotation.from_quat(np.roll(r['initial_target']['quaternion_wxyz'],-1)).as_euler('xyz',degrees=True)[2]
  for frame_id,v in enumerate(samples):
   robot.set_joint_positions(np.array(v['q'])[ix]);robot.set_joint_velocities(np.zeros(len(ix)))
   box.set_world_pose(v['box'],v['box_quat']);box.set_linear_velocity(np.zeros(3));box.set_angular_velocity(np.zeros(3))
   # Render only. Recorded body and joint states are never re-integrated by physics.
   omni.physx.get_physx_interface().update_transformations(True,True,True,False)

   for _ in range(8):world.render()
   if os.environ.get('FR3_RENDER_TEST'):print('STATE_CHECK',frame_id,v['box'],box.get_world_pose()[0].tolist(),float(np.max(np.abs(robot.get_joint_positions()-np.array(v['q'])[ix]))),flush=True)
   pictures=[cv2.cvtColor(c.get_rgba()[:,:,:3].astype(np.uint8),cv2.COLOR_RGB2BGR) for c in views]
   frame=np.full((720,1280,3),(24,28,34),dtype=np.uint8);frame[70:670]=np.concatenate(pictures,axis=1)
   label=f'{group} | seed {seed} | original: {r["category"]} | rank {r["selected_rank"]} | pad {coverage*100:.1f}%'
   cv2.putText(frame,label,(18,27),cv2.FONT_HERSHEY_SIMPLEX,.66,(245,245,245),1,cv2.LINE_AA)
   cv2.putText(frame,'RECORDED STATE REPLAY - not a new physics trial | target yaw %.1f deg'%yaw,(18,55),cv2.FONT_HERSHEY_SIMPLEX,.55,(125,205,235),1,cv2.LINE_AA)
   info=f'{v["phase"]} | sim t={v["t"]-records[start]["t"]:.2f}s | lift {(v["box"][2]-z0)*100:.2f} cm | measured L/R {v["forces"][0]:.1f}/{v["forces"][1]:.1f} N'
   cv2.putText(frame,info,(18,701),cv2.FONT_HERSHEY_SIMPLEX,.64,(245,245,245),1,cv2.LINE_AA)
   assert np.max(np.abs(robot.get_joint_positions()-np.array(v['q'])[ix]))<1e-6
   assert np.linalg.norm(box.get_world_pose()[0]-np.array(v['box']))<1e-6
   writer.write(frame)
   if frame_id%200==0:print('FRAME',group,seed,frame_id,len(samples),flush=True)
   if frame_id in [0,len(samples)//2,len(samples)-1]:cv2.imwrite(str(OUT/f'{group}_{seed}_{frame_id}.jpg'),frame)
  writer.release();item=dict(group=group,seed=seed,frames=len(samples),fps=30,category=r['category'],source=str(directory),replay='measured joint and target states; no new physics; obstacles fixed at recorded initial pose (original disturbance zero)');manifest.append(item)
  (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2));print('VIDEO_COMPLETE',json.dumps(item),flush=True)
finally:app.close()
