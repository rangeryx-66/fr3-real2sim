"""One bounded no-object sensor commissioning scene; independent calibration loads only."""
import argparse, gzip, hashlib, json, os, pickle, signal, subprocess, time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from xarm_wrist_ft import prepare, WristFT, SENSOR_BODY, tool_gravity_world
ROOT=Path(__file__).resolve().parents[1]

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--gpu',type=int,default=5)
a=p.parse_args();O=Path(a.output);O.mkdir(parents=True,exist_ok=False)
c=prepare(O/'assembly');dt=1/240;rows=[];tick=0;phase='INIT';stop=False;video=None;world=None
start=time.perf_counter();status='STARTING';os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y'
signal.signal(signal.SIGTERM,lambda *args:globals().update(stop=True))
signal.signal(signal.SIGINT,lambda *args:globals().update(stop=True))
from isaacsim import SimulationApp
app=SimulationApp(dict(headless=True,active_gpu=a.gpu,physics_gpu=a.gpu,multi_gpu=False,anti_aliasing=0))
try:
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation,SingleXFormPrim,RigidPrim
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.asset.importer.urdf import URDFImporter,URDFImporterConfig
    from isaacsim.sensors.camera import Camera
    from pxr import UsdPhysics,PhysxSchema,Usd,PhysicsSchemaTools
    import omni.usd,omni.physx
    from xarm7_native_gripper import configure_stage,NativeDrive,GRIPPER_JOINTS
    from cross_robot_adapter import RobotPinkIK
    from precontact_trajectory import segment_samples
    from articulated_demo.kinematics import URDFChain
    solver=RobotPinkIK(c);chain=URDFChain(c['sim_urdf'])
    world=World(physics_dt=dt,rendering_dt=1/30,backend='numpy',device='cpu')
    world.scene.add_default_ground_plane(z_position=-.76)
    usd=URDFImporter(URDFImporterConfig(urdf_path=c['sim_urdf'],usd_path=str(O/'asset'),fix_base=True,
        allow_self_collision=True,merge_fixed_joints=False,joint_drive_type='force',joint_target_type='position')).import_urdf()
    add_reference_to_stage(usd,'/World/XArm7');stage=omni.usd.get_context().get_stage()
    configure_stage(stage,'/World/XArm7',c,O)
    robot=world.scene.add(SingleArticulation('/World/XArm7',name='xarm7'))
    camera=Camera('/World/diagnostic_camera',resolution=(960,720),frequency=30)
    eye=np.array([1.2,-1.35,1.]);f=np.array([0,0,.25])-eye;f/=np.linalg.norm(f)
    r=np.cross(f,[0,0,1]);r/=np.linalg.norm(r)
    camera.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(np.column_stack([r,np.cross(f,r),f])).as_quat(),1),camera_axes='ros')
    for prim in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
        if str(prim.GetPath()).startswith('/World/XArm7') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    world.reset();camera.initialize()
    names=list(robot.dof_names);arm=np.array([names.index(n) for n in c['arm_joint_names']])
    driver=NativeDrive(robot,c,O);q=np.zeros(len(names));q[arm]=c['home_q'];q[driver.fingers]=driver.geometry.angle(.08)
    robot.set_joint_positions(q);robot.set_joint_velocities(q*0);desired=q[arm].copy();velocity=np.zeros(7)
    driver.command_width(.08)
    paths={p.GetName():str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith('/World/XArm7')}
    views={n:SingleXFormPrim(path) for n,path in paths.items()}
    load_view=RigidPrim(paths[SENSOR_BODY],name='independent_calibration_load');load_view.initialize()
    sensor=WristFT(robot)
    urdf=ET.parse(c['sim_urdf']).getroot();properties={}
    for link in urdf.findall('link'):
        ins=link.find('inertial');n=link.get('name')
        if ins is None or n not in paths or n not in [SENSOR_BODY]+['%s_%s'%(s,b) for s in ['left','right'] for b in ['finger','outer_knuckle','inner_knuckle']]:continue
        origin=ins.find('origin');mass=float(ins.find('mass').get('value'));v=ins.find('inertia').attrib
        I=np.array([[float(v['ixx']),float(v['ixy']),float(v['ixz'])],[float(v['ixy']),float(v['iyy']),float(v['iyz'])],[float(v['ixz']),float(v['iyz']),float(v['izz'])]])
        R=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).as_matrix()
        properties[n]=dict(mass_kg=mass,com_body_m=list(map(float,origin.get('xyz','0 0 0').split())),inertia_body_kg_m2=(R@I@R.T).tolist())
    def pose(n):
        pos,quat=views[n].get_world_pose();T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.roll(quat,-1)).as_matrix();T[:3,3]=pos;return T
    contacts=[]
    def on_contact(headers,data):
        for h in headers:
            for d in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]:
                contacts.append(dict(collider0=str(PhysicsSchemaTools.intToSdfPath(h.collider0)),collider1=str(PhysicsSchemaTools.intToSdfPath(h.collider1)),impulse_world_ns=[d.impulse.x,d.impulse.y,d.impulse.z]))
    subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg'
    video=subprocess.Popen([ff,'-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x720','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23',str(O/'sensor_smoke.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(O/'ffmpeg.log').open('w'))
    load_force=np.zeros(3);load_torque=np.zeros(3);load_offset=np.zeros(3)
    def step():
        global tick
        if stop or (O/'STOP_AND_EXPORT').exists():raise KeyboardInterrupt('OWNER_STOP')
        driver.apply_action(ArticulationAction(joint_positions=desired,joint_velocities=velocity,joint_indices=arm));driver.hold()
        load_pose=pose(SENSOR_BODY);load_point=load_pose[:3,3]+load_offset
        if np.any(load_force) or np.any(load_torque):load_view.apply_forces_and_torques_at_pos(forces=load_force[None],torques=load_torque[None],positions=load_point[None],is_global=True)
        contacts.clear();world.step(render=False,update_fabric=True)
        T=pose(SENSOR_BODY);body_states=[]
        for n,b in properties.items():
            B=pose(n);com=B[:3,:3]@b['com_body_m']+B[:3,3]
            body_states.append(dict(name=n,mass_kg=b['mass_kg'],T_world_body=B.tolist(),com_world_m=com.tolist(),inertia_world_kg_m2=(B[:3,:3]@b['inertia_body_kg_m2']@B[:3,:3].T).tolist()))
        actual=np.asarray(robot.get_joint_positions());raw=sensor.read();fk=chain.root_to_link(SENSOR_BODY,dict(zip(names,actual)))
        rows.append(dict(t=float(world.current_time),step=tick,dt=dt,phase=phase,raw_wrench_sensor=raw.tolist(),valid=bool(np.isfinite(raw).all()),
            T_world_sensor=T.tolist(),tool_gravity_wrench_world=tool_gravity_world(body_states,T[:3,3]).tolist(),body_states=body_states,
            q=actual.tolist(),dq_native=robot.get_joint_velocities().tolist(),command_q_arm=desired.tolist(),actuator=driver.last.copy(),
            aperture=driver.feedback(),applied_efforts=robot.get_applied_joint_efforts().tolist(),projected_efforts=robot.get_measured_joint_efforts().tolist(),
            independently_applied_force_world_n=load_force.tolist(),independently_applied_torque_world_nm=load_torque.tolist(),independently_applied_position_world_m=load_point.tolist(),
            contacts=contacts.copy(),fk_position_error_m=float(np.linalg.norm(fk[:3,3]-T[:3,3])),fk_rotation_error_rad=float(Rotation.from_matrix(fk[:3,:3]@T[:3,:3].T).magnitude())))
        tick+=1
        if tick%8==0:
            world.render();frame=np.ascontiguousarray(camera.get_rgba()[:,:,:3]).astype(np.uint8);video.stdin.write(frame.tobytes())
            if not (O/'FIRST_FRAME.png').exists():
                from PIL import Image
                Image.fromarray(frame).save(O/'FIRST_FRAME.png')
        if tick%240==0:(O/'PROGRESS.json').write_text(json.dumps(dict(phase=phase,sim_s=tick*dt,steps=tick,pid=os.getpid())))
    def hold(s):
        for _ in range(round(s/dt)):step()
    def move(target):
        global desired,velocity
        begin=desired.copy()
        for ref,vel in segment_samples(solver,begin,np.asarray(target),4.,dt,.003):desired=ref;velocity=vel;step()
        velocity=np.zeros(7)
    (O/'RUNTIME_MAPPING.json').write_text(json.dumps(dict(**sensor.mapping,active_dof_names=names,body_paths=paths,tool_properties=properties,physics_dt_s=dt),indent=2))
    (O/'SOURCE_HASHES.json').write_text(json.dumps({str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),ROOT/'scripts/xarm_wrist_ft.py',ROOT/'scripts/xarm7_native_gripper.py',ROOT/'scripts/xarm7_runtime.py',Path(c['sim_urdf']),Path(c['dynamics_urdf'])]},indent=2))
    for _ in range(6):world.render()
    phase='SETTLE_INITIAL';hold(2.)
    home=np.asarray(c['home_q']);orientations=[home,home+np.array([0,0,0,0,0,.25,0]),home+np.array([0,0,0,0,0,.25,.4]),home+np.array([0,0,0,0,0,-.25,-.4])]
    for i,target in enumerate(orientations):
        if i:phase='UNLOADED_MOTION_%d'%i;move(target)
        phase='STATIC_ORIENTATION_%d'%i;hold(1.5)
    phase='UNLOADED_RETURN';move(home);phase='STATIC_REFERENCE';hold(1.5)
    # Independent known force and torque injections exist ONLY in this robot-only
    # calibration. No object is present. The sensor reads the mounting reaction.
    for axis in range(3):
        load_force=np.eye(3)[axis];load_torque=np.zeros(3);load_offset=np.array([.03,-.02,.04])
        phase='KNOWN_FORCE_%d'%axis;hold(1.)
        load_force=np.zeros(3);phase='ZERO_AFTER_FORCE_%d'%axis;hold(.5)
    for axis in range(3):
        load_torque=.05*np.eye(3)[axis];phase='KNOWN_TORQUE_%d'%axis;hold(1.)
        load_torque=np.zeros(3);phase='ZERO_AFTER_TORQUE_%d'%axis;hold(.5)
    phase='FINAL_HOLD';hold(1.);status='SENSOR_SMOKE_COMPLETE'
except BaseException as e:
    import traceback
    status=repr(e);(O/'ERROR.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
finally:
    if world is not None:
        try:world.pause()
        except Exception:pass
    dest=O/'physics_chunks';dest.mkdir(exist_ok=True);chunks=[]
    for i in range(0,len(rows),1024):
        file=dest/('%08d.pkl.gz'%i)
        with gzip.open(file,'wb',compresslevel=1) as stream:pickle.dump(rows[i:i+1024],stream,protocol=5)
        chunks.append(dict(file=file.name,records=min(1024,len(rows)-i),sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
    (dest/'index.json').write_text(json.dumps(dict(records=len(rows),chunks=chunks,lossless=True),indent=2))
    if video:
        try:video.stdin.close();video.wait(timeout=30)
        except Exception:pass
    (O/'RESULT.json').write_text(json.dumps(dict(status=status,sim_s=tick*dt,wall_s=time.perf_counter()-start,pid=os.getpid(),steps=len(rows),door_episodes=0,source='fixed-joint transmitted spatial wrench',shutdown='pause/export/video EOF/app.close'),indent=2))
    app.close()
