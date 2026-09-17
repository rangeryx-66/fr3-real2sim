"""Render measured states only; this never supplies a physics benchmark outcome."""
import os,json,gzip,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
adapter=ROOT/'calibration/sim_unseen.py'
code=adapter.read_text();tail="exec(compile('import os\\n'+text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))"
assert code.count(tail)==1
scope={'__file__':str(adapter)};exec(compile(code.replace(tail,''),str(adapter),'exec'),scope)
simtext='import os\n'+scope['text'].split('world.reset()')[0]
exec(compile(simtext,str(scope['source']),'exec'))
import cv2,numpy as np,omni.physx,imageio_ffmpeg
from scipy.spatial.transform import Rotation
from pxr import UsdLux,Gf

result_path=Path(os.environ['UNSEEN_RENDER_RESULT']);result=json.loads(result_path.read_text())
trace_path=Path(result['trace']['path']);data=json.load(gzip.open(trace_path));records=data['records']
out=Path(os.environ.get('UNSEEN_RENDER_OUTPUT',str(ROOT/'videos/unseen_v3')));out.mkdir(parents=True,exist_ok=True)
UsdLux.DomeLight.Define(stage,'/World/video_dome').CreateIntensityAttr(700)
key=UsdLux.DistantLight.Define(stage,'/World/video_key');key.CreateIntensityAttr(1000);UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-35,25,15))

def look(cam,pos,at,fov):
    z=np.array(at)-pos;z/=np.linalg.norm(z);x=np.cross(z,[0,0,1]);x/=np.linalg.norm(x);y=np.cross(z,x)
    cam.set_world_pose(position=np.array(pos),orientation=np.roll(Rotation.from_matrix(np.column_stack([x,y,z])).as_quat(),1),camera_axes='ros')
    cam.set_focal_length(cam.get_horizontal_aperture()/(2*np.tan(np.deg2rad(fov/2))));cam.set_clipping_range(.02,10.)

views=[]
for name,res,pos,at,fov in [('overview',(768,600),[1.3,1.15,.9],[.45,0,.18],62),('closeup',(512,600),[.94,.62,.49],[.5,0,.13],46)]:
    cam=Camera('/World/video_'+name,resolution=res,frequency=30);look(cam,np.array(pos),at,fov);views.append(cam)
world.reset()
for cam in views:cam.initialize()
for _ in range(100):world.render()
start=next((i for i,x in enumerate(records) if x['phase'] in ['PREGRASP','APPROACH']),0)
speed=int(os.environ.get('UNSEEN_RENDER_SPEED','1'));assert speed in [1,2,4]
samples=[records[start]]*20+records[start::10*speed]+[records[-1]]*30
ix=[data['names'].index(n) for n in robot.dof_names]
reference=np.array(result.get('height_reference_local_m',[0,0,0]))
z0=result.get('initial_height_reference_z_m',np.asarray(result['initial_target'])[2,3])
raw=out/(result['id']+'_raw.mp4');writer=cv2.VideoWriter(str(raw),cv2.VideoWriter_fourcc(*'mp4v'),24,(1280,720));assert writer.isOpened()
try:
    for frame_id,row in enumerate(samples):
        robot.set_joint_positions(np.asarray(row['q'])[ix]);robot.set_joint_velocities(np.zeros(len(ix)))
        box.set_world_pose(row['box'],row['box_quat']);box.set_linear_velocity(np.zeros(3));box.set_angular_velocity(np.zeros(3))
        if clutter and 'non_target_poses' in row:
            poses=row['non_target_poses']
            for body,position,quat in zip(clutter.bodies,poses['positions'],poses['quaternions_wxyz']):
                body.set_world_pose(position,quat);body.set_linear_velocity(np.zeros(3));body.set_angular_velocity(np.zeros(3))
        omni.physx.get_physx_interface().update_transformations(True,True,True,False)
        for _ in range(2):world.render()
        images=[]
        for cam in views:
            picture=cam.get_rgba()
            for _ in range(30):
                if picture is not None:break
                world.render();picture=cam.get_rgba()
            if picture is None:raise RuntimeError('No RGBA frame')
            images.append(cv2.cvtColor(picture[:,:,:3].astype(np.uint8),cv2.COLOR_RGB2BGR))
        frame=np.full((720,1280,3),(24,28,34),dtype=np.uint8);frame[70:670]=np.concatenate(images,axis=1)
        cv2.putText(frame,f"{result['target']} | {result['mode']} | {result['category']}",(18,27),cv2.FONT_HERSHEY_SIMPLEX,.64,(245,245,245),1,cv2.LINE_AA)
        cv2.putText(frame,f'MEASURED REPLAY | {speed}x simulation time | no new physics or weld',(18,55),cv2.FONT_HERSHEY_SIMPLEX,.55,(125,205,235),1,cv2.LINE_AA)
        R=Rotation.from_quat(np.roll(row['box_quat'],-1)).as_matrix();lift=(row['box'][2]+(R@reference)[2]-z0)*100
        forces=row.get('forces',[0,0]);bottom=row.get('mesh_bottom_z_m',float('nan'))*1000
        line=f"{row['phase']} | t={row['t']-records[start]['t']:.1f}s | lift {lift:.2f} cm | bottom {bottom:.1f} mm | L/R {forces[0]:.1f}/{forces[1]:.1f} N"
        cv2.putText(frame,line,(18,701),cv2.FONT_HERSHEY_SIMPLEX,.51,(245,245,245),1,cv2.LINE_AA)
        writer.write(frame)
finally:
    writer.release();destination=raw.with_name(raw.name.replace('_raw',''))
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-i',str(raw),'-c:v','libx264','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(destination)],check=True)
    (out/(result['id']+'.render.json')).write_text(json.dumps(dict(source_result=str(result_path),source_trace=str(trace_path),frames=len(samples),fps=24,speed=speed,replay_only=True),indent=2))
    print('VIDEO_COMPLETE',destination,flush=True);app.close()
