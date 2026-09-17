"""Render a measured stability trace without rerunning physics or control."""
from pathlib import Path
import os,json,gzip,subprocess
adapter=Path(__file__).with_name('sim_calibration.py')
source_text=adapter.read_text();tail="exec(compile(text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))"
assert source_text.count(tail)==1
ns={'__file__':str(adapter)};exec(compile(source_text.replace(tail,''),str(adapter),'exec'),ns)
simtext=ns['text'].split('world.reset()')[0];exec(compile(simtext,str(ns['source']),'exec'))
import cv2,numpy as np,omni.physx,imageio_ffmpeg
from scipy.spatial.transform import Rotation
from pxr import UsdLux,Gf

result_path=Path(os.environ['STABILITY_RENDER_RESULT']);result=json.loads(result_path.read_text())
trace_path=Path(result['trace']['path']);data=json.load(gzip.open(trace_path));records=data['records']
out=ROOT/'videos/settling_regrasp';out.mkdir(parents=True,exist_ok=True)
UsdLux.DomeLight.Define(stage,'/World/video_dome').CreateIntensityAttr(700)
key=UsdLux.DistantLight.Define(stage,'/World/video_key');key.CreateIntensityAttr(1000);UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-35,25,15))
def look(cam,pos,at,fov):
    z=np.array(at)-pos;z/=np.linalg.norm(z);x=np.cross(z,[0,0,1]);x/=np.linalg.norm(x);y=np.cross(z,x)
    cam.set_world_pose(position=np.array(pos),orientation=np.roll(Rotation.from_matrix(np.column_stack([x,y,z])).as_quat(),1),camera_axes='ros')
    cam.set_focal_length(cam.get_horizontal_aperture()/(2*np.tan(np.deg2rad(fov/2))));cam.set_clipping_range(.02,10.)
views=[]
for name,res,pos,at,fov in [('overview',(768,600),[1.3,1.15,.9],[.42,0,.18],62),('closeup',(512,600),[.82,.42,.34],[.5,0,.08],46)]:
    cam=Camera('/World/video_'+name,resolution=res,frequency=30);look(cam,np.array(pos),at,fov);views.append(cam)
world.reset()
for cam in views:cam.initialize()
for _ in range(120):world.render()
start=next((i for i,x in enumerate(records) if x['phase'] in ['PREGRASP','APPROACH']),0)
samples=[records[start]]*24+records[start::8]+[records[-1]]*45
ix=[data['names'].index(n) for n in robot.dof_names];z0=result.get('initial_target',{}).get('position',result.get('initial'))[2]
raw=out/(result['id']+'_raw.mp4');writer=cv2.VideoWriter(str(raw),cv2.VideoWriter_fourcc(*'mp4v'),30,(1280,720));assert writer.isOpened()
try:
    for frame_id,row in enumerate(samples):
        robot.set_joint_positions(np.asarray(row['q'])[ix]);robot.set_joint_velocities(np.zeros(len(ix)))
        box.set_world_pose(row['box'],row['box_quat']);box.set_linear_velocity(np.zeros(3));box.set_angular_velocity(np.zeros(3))
        omni.physx.get_physx_interface().update_transformations(True,True,True,False)
        for _ in range(2):world.render()
        rgba=[]
        for cam in views:
            picture=cam.get_rgba()
            # RTX/Replicator can return None for the first frame after a manual
            # articulation transform.  Wait for the same measured state to be
            # available instead of dropping or interpolating a trace sample.
            for _ in range(30):
                if picture is not None:
                    break
                world.render();picture=cam.get_rgba()
            if picture is None:
                raise RuntimeError(f'camera {cam.prim_path} produced no RGBA frame')
            rgba.append(picture)
        pictures=[cv2.cvtColor(p[:,:,:3].astype(np.uint8),cv2.COLOR_RGB2BGR) for p in rgba]
        frame=np.full((720,1280,3),(24,28,34),dtype=np.uint8);frame[70:670]=np.concatenate(pictures,axis=1)
        mode=result.get('mode','diagnostic');attempts=len(result.get('attempts',[]))
        title=f"{result['target']} | {mode} | {result['category']} | attempts {attempts or 1}"
        cv2.putText(frame,title,(18,27),cv2.FONT_HERSHEY_SIMPLEX,.72,(245,245,245),1,cv2.LINE_AA)
        cv2.putText(frame,'MEASURED STATE REPLAY | no new physics, weld or pose correction',(18,55),cv2.FONT_HERSHEY_SIMPLEX,.55,(125,205,235),1,cv2.LINE_AA)
        lift=(row['box'][2]-z0)*100;forces=row.get('forces',[0,0])
        info=f"{row['phase']} | t={row['t']-records[start]['t']:.2f}s | lift {lift:.2f} cm | L/R {forces[0]:.1f}/{forces[1]:.1f} N"
        cv2.putText(frame,info,(18,701),cv2.FONT_HERSHEY_SIMPLEX,.63,(245,245,245),1,cv2.LINE_AA)
        writer.write(frame)
finally:
    writer.release()
    destination=raw.with_name(raw.name.replace('_raw',''))
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-i',str(raw),'-c:v','libx264','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(destination)],check=True)
    (out/(result['id']+'.render.json')).write_text(json.dumps(dict(source_result=str(result_path),source_trace=str(trace_path),frames=len(samples),fps=30,replay_only=True),indent=2))
    print('VIDEO_COMPLETE',destination,flush=True)
    app.close()
