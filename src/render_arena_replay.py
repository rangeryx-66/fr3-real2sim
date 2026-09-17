"""Render saved robot, target and ALL non-target states; no new physical trial."""
from pathlib import Path
import os,gzip,json
adapter=Path(__file__).with_name('sim_arena.py');text=adapter.read_text()
tail="exec(compile(text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))"
assert text.count(tail)==1
ns={'__file__':str(adapter)};exec(compile(text.replace(tail,''),str(adapter),'exec'),ns)
simtext=ns['text'].split('stage.Flatten().Export')[0]
exec(compile(simtext,str(ns['source']),'exec'))
import cv2,subprocess
from scipy.spatial.transform import Rotation
from pxr import UsdLux,Gf
import omni.physx,imageio_ffmpeg
OUT=ROOT/'videos/arena_complex';OUT.mkdir(parents=True,exist_ok=True)
UsdLux.DomeLight.Define(stage,'/World/video_dome').CreateIntensityAttr(700)
key=UsdLux.DistantLight.Define(stage,'/World/video_key');key.CreateIntensityAttr(1000);UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-35,25,15))
views=[]
for name,res,pos,at,fov in [('overview',(768,600),[1.4,1.3,1.05],[.32,0,.22],65),('closeup',(512,600),[.90,.50,.48],[.5,0,.10],53)]:
    cam=Camera('/World/video_'+name,resolution=res,frequency=30)
    z=np.array(at)-pos;z/=np.linalg.norm(z);x=np.cross(z,[0,0,1]);x/=np.linalg.norm(x);y=np.cross(z,x)
    cam.set_world_pose(position=np.array(pos),orientation=np.roll(Rotation.from_matrix(np.column_stack([x,y,z])).as_quat(),1),camera_axes='ros')
    cam.set_focal_length(cam.get_horizontal_aperture()/(2*np.tan(np.deg2rad(fov/2))));cam.set_clipping_range(.02,10.);views.append(cam)
world.reset()
for c in views:c.initialize()
for _ in range(45):world.render()
try:
    for seed in map(int,os.environ['FR3_RENDER_SEEDS'].split(',')):
        directory=ROOT/'results/arena_complex40';r=json.loads((directory/f'B_seed_{seed:04d}.json').read_text());assert r['target_class']==arena_scene.TARGET
        data=json.load(gzip.open(directory/f'trace_seed_{seed:04d}.json.gz','rt'));records=data['records'];start=next((i for i,v in enumerate(records) if v['phase']=='PREGRASP'),max(0,len(records)-240))
        samples=[records[start]]*30+records[start::8]+[records[-1]]*45
        if os.environ.get('FR3_RENDER_TEST'):samples=[records[start],records[(start+len(records))//2],records[-1]]
        raw=OUT/f'{r["target_class"]}_{seed}_raw.mp4';writer=cv2.VideoWriter(str(raw),cv2.VideoWriter_fourcc(*'mp4v'),30,(1280,720));assert writer.isOpened()
        ix=[data['names'].index(n) for n in robot.dof_names];z0=r['initial_target']['position'][2]
        for i,v in enumerate(samples):
            robot.set_joint_positions(np.array(v['q'])[ix]);robot.set_joint_velocities(np.zeros(len(ix)))
            box.set_world_pose(v['box'],v['box_quat']);box.set_linear_velocity(np.zeros(3));box.set_angular_velocity(np.zeros(3))
            for j,b in enumerate(clutter.bodies):
                poses=v['non_target_poses'];b.set_world_pose(poses['positions'][j],poses['quaternions_wxyz'][j]);b.set_linear_velocity(np.zeros(3));b.set_angular_velocity(np.zeros(3))
            omni.physx.get_physx_interface().update_transformations(True,True,True,False)
            for _ in range(8):world.render()
            pictures=[cv2.cvtColor(c.get_rgba()[:,:,:3].astype(np.uint8),cv2.COLOR_RGB2BGR) for c in views]
            frame=np.full((720,1280,3),(24,28,34),dtype=np.uint8);frame[70:670]=np.concatenate(pictures,axis=1)
            title=f'{r["target_class"]} | seed {seed} | {r["category"]} | rank {r["selected_rank"]} | NT contact {int(r["non_target_contact"])}'
            cv2.putText(frame,title,(18,27),cv2.FONT_HERSHEY_SIMPLEX,.72,(245,245,245),1,cv2.LINE_AA)
            cv2.putText(frame,'RECORDED STATE REPLAY | robot + all objects | not a new physics trial',(18,55),cv2.FONT_HERSHEY_SIMPLEX,.55,(125,205,235),1,cv2.LINE_AA)
            info=f'{v["phase"]} | t={v["t"]-records[start]["t"]:.2f}s | lift {(v["box"][2]-z0)*100:.2f} cm | L/R {v["forces"][0]:.1f}/{v["forces"][1]:.1f} N'
            cv2.putText(frame,info,(18,701),cv2.FONT_HERSHEY_SIMPLEX,.64,(245,245,245),1,cv2.LINE_AA)
            assert np.max(np.abs(robot.get_joint_positions()-np.array(v['q'])[ix]))<1e-6
            assert np.linalg.norm(box.get_world_pose()[0]-v['box'])<1e-6
            writer.write(frame)
            if i in [0,len(samples)//2,len(samples)-1]:cv2.imwrite(str(OUT/f'{r["target_class"]}_{seed}_{i}.jpg'),frame)
            if i%200==0:print('FRAME',seed,i,len(samples),flush=True)
        writer.release()
        dest=raw.with_name(raw.name.replace('_raw',''));subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-i',str(raw),'-c:v','libx264','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(dest)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        (OUT/f'{seed}_replay.json').write_text(json.dumps(dict(seed=seed,source=str(directory),frames=len(samples),fps=30,all_object_poses_replayed=True,new_physics=False),indent=2));print('COMPLETE',str(dest),flush=True)
finally:app.close()
