"""Record two views of a physics rerun of the saved, validated MoveIt trajectory."""
from pathlib import Path
# Reuse exactly the validated scene/drive configuration, not a second physics model.
source=Path(__file__).with_name('sim_server.py').read_text()
exec(compile(source.split('world.reset()')[0],str(Path(__file__).with_name('sim_server.py')),'exec'))
import cv2
from scipy.spatial.transform import Rotation
from pxr import UsdLux, Gf
OUT=ROOT/'videos';OUT.mkdir(exist_ok=True)
RUN=ROOT/'results/run_1788943864166403373'
trial=json.loads((RUN/'trial_01.json').read_text())
light=UsdLux.DomeLight.Define(stage,'/World/video_dome');light.CreateIntensityAttr(1100)
key=UsdLux.DistantLight.Define(stage,'/World/video_key');key.CreateIntensityAttr(1800)
UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-35,25,15))
def look(cam,pos,at):
    z=np.array(at)-pos;z=z/np.linalg.norm(z)
    x=np.cross(z,[0,0,1]);x=x/np.linalg.norm(x);y=np.cross(z,x)
    q=Rotation.from_matrix(np.column_stack([x,y,z])).as_quat()
    cam.set_world_pose(position=np.array(pos),orientation=np.roll(q,1),camera_axes='ros')
views=[]
for name,pos,at,focal in [('overview',[1.5,1.4,1.0],[.25,0,.30],68.),('closeup',[.91,.45,.35],[.49,0,.065],50.)]:
    cam=Camera('/World/video_'+name,resolution=(1280,720),frequency=30)
    look(cam,np.array(pos),at);cam.set_focal_length(cam.get_horizontal_aperture()/(2*np.tan(np.deg2rad(focal/2))));cam.set_clipping_range(.02,10.)
    views.append((name,cam))
world.reset()
for _,cam in views:cam.initialize()
names=robot.dof_names
arm=[names.index('fr3_joint'+str(i)) for i in range(1,8)]
fingers=[names.index('fr3_finger_joint'+str(i)) for i in [1,2]]
q=np.zeros(len(names));q[arm]=HOME;q[fingers]=.04
robot.set_joint_positions(q);robot.apply_action(ArticulationAction(joint_positions=q))
for i in range(240):world.step(render=i%8==0)
writers={name:cv2.VideoWriter(str(OUT/(name+'_raw.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),30,(1280,720)) for name,_ in views}
assert all(w.isOpened() for w in writers.values())
frames=0;step=0;telemetry=[];phase='START';initial_z=float(box.get_world_pose()[0][2])

def advance(target):
    global step,frames
    robot.apply_action(ArticulationAction(joint_positions=target))
    world.step(render=step%8==0)
    if step%8==0:
        bp,_=box.get_world_pose()
        force=np.linalg.norm(np.asarray(contacts.get_contact_force_matrix(dt=DT)).reshape(2,-1,3).sum(axis=1),axis=1)
        record=dict(t=step*DT,phase=phase,z=float(bp[2]),forces=force.tolist());telemetry.append(record)
        for name,cam in views:
            rgba=cam.get_rgba()
            if rgba is None or rgba.shape!=(720,1280,4):raise RuntimeError('camera not ready '+name)
            frame=cv2.cvtColor(rgba[:,:,:3].astype(np.uint8),cv2.COLOR_RGB2BGR)
            cv2.rectangle(frame,(0,0),(1280,91),(19,25,33),-1)
            cv2.putText(frame,'FR3 + Franka Hand | AnyGrasp + MoveIt 2',(26,34),cv2.FONT_HERSHEY_SIMPLEX,.82,(245,245,245),2,cv2.LINE_AA)
            cv2.putText(frame,'PHYSICS RERUN / saved trial 01 | '+name.upper()+' | '+phase,(27,70),cv2.FONT_HERSHEY_SIMPLEX,.6,(157,207,240),1,cv2.LINE_AA)
            cv2.rectangle(frame,(0,650),(1280,720),(19,25,33),-1)
            label=f'Sim {step*DT:5.2f}s    Object lift {(bp[2]-initial_z)*100:5.2f} cm    Contact L/R {force[0]:5.1f} / {force[1]:5.1f} N'
            cv2.putText(frame,label,(26,690),cv2.FONT_HERSHEY_SIMPLEX,.67,(245,245,245),1,cv2.LINE_AA)
            writers[name].write(frame)
            if frames in [0,210,330,450]:cv2.imwrite(str(OUT/f'{name}_{frames:04d}.jpg'),frame)
        frames+=1
    step+=1

def hold(duration):
    for _ in range(round(duration/DT)):advance(q)
def motion(execution):
    global q,phase
    phase=execution['stage']
    idx=[names.index(n) for n in execution['joint_names']]
    ts=np.array([p['t'] for p in execution['points']]);ps=np.array([p['q'] for p in execution['points']])
    if ts[0]>0:ts=np.r_[0,ts];ps=np.vstack([robot.get_joint_positions()[idx],ps])
    for j in range(int(np.ceil(ts[-1]/DT))+1):
        for k,index in enumerate(idx):q[index]=np.interp(j*DT,ts,ps[:,k])
        advance(q)
    hold(.35)
try:
    hold(1)
    motion(trial['executions'][0]);motion(trial['executions'][1])
    phase='CLOSE GRIPPER'
    for j in range(240):q[fingers]=.04*(1-(j+1)/240);advance(q)
    hold(.65)
    motion(trial['executions'][2])
    phase='HOLD 2+ SECONDS';hold(2.5)
    hs=[h for h in telemetry if h['phase']=='HOLD 2+ SECONDS']
    passed=hs[-1]['t']-hs[0]['t']>=2 and min(h['z'] for h in hs)>initial_z+.08 and min(min(h['forces']) for h in hs)>.1
    (OUT/'render_validation.json').write_text(json.dumps(dict(passed=bool(passed),source_trial=str(RUN/'trial_01.json'),frames=frames,fps=30,initial_z=initial_z,telemetry=telemetry),indent=2))
    print('RENDER_COMPLETE',frames,'PHYSICS_PASS',passed,flush=True)
    for name,cam in views:
        frame=cv2.cvtColor(cam.get_rgba()[:,:,:3].astype(np.uint8),cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(OUT/(name+'_final.jpg')),frame)
finally:
    for w in writers.values():w.release()
    app.close()
