"""Real PiPER finger closure and passive-door interaction; no ideal attachment."""
import argparse,json,os,sys,hashlib,subprocess,shutil
from pathlib import Path
from datetime import datetime,timedelta
from zoneinfo import ZoneInfo
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def matrix(p,q):
    T=np.eye(4);T[:3,3]=p;T[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();return T


def bootstrap(a,base):
    # Only substitute robot-specific code in a guarded copy of the common loader.
    # Source file and all articulated-object definitions remain untouched.
    legacy=ROOT/'src/r1a7_articulated_sim_server.py';text=legacy.read_text();marker='commands = queue.Queue(); state = {}; lock = threading.Lock()'
    assert text.count(marker)==1;source=text.partition(marker)[0]
    substitutions=[("scripts/prepare_r1a7_description.py","scripts/prepare_piper_description.py"),
        ('nargs=7','nargs=6'),('default=(0.,1.3,1.,-1.3,0.,0.,0.)','default=(0.,1.15,-1.35,0.,.2,0.)'),
        ('config/r1a7_dex1.urdf','config/piper_sim.urdf'),('/World/R1A7','/World/Piper'),
        ('r1a7_dex1_filtered_asset_path_','piper_mobile_asset_path_'),
        ('dex1_Link1_3','gripper_link1'),('dex1_Link2_3','gripper_link2'),
        ('dex1_Link1_2','gripper_base'),('dex1_Link2_2','link6'),
        ('r1a7_tcp','tcp_link'),("names.index(f'J{i}') for i in range(1, 8)","names.index(f'joint{i}') for i in range(1, 7)"),
        ('dex1_Joint1_1','gripper_joint1'),('dex1_Joint2_1','gripper_joint2'),
        ('kp[arm] = [500., 500., 400., 400., 250., 150., 150.]','kp[arm] = 10000.'),
        ('kd[arm] = [40., 40., 32., 32., 22., 15., 15.]','kd[arm] = 400.'),
        ('kp[fingers] = 800.; kd[fingers] = 30.','kp[fingers] = 1000.; kd[fingers] = 40.'),
        ('q[fingers] = -.02','q[fingers] = [.05,-.05]'),
        ('fix_base=True, allow_self_collision=False, merge_fixed_joints=False,\n        joint_drive_type','fix_base=True, allow_self_collision=True, merge_fixed_joints=False,\n        joint_drive_type')]
    for old,new in substitutions:
        if old not in source:raise RuntimeError('scene loader changed: '+old)
        source=source.replace(old,new)
    source=source.replace("if prim.GetName() == name and str(prim.GetPath()).startswith(under)]", "if prim.GetName() == name and str(prim.GetPath()).startswith(under)\n               and (prim.HasAPI(UsdPhysics.RigidBodyAPI) or name == 'tcp_link')]")
    source=source.replace('save_to_usd=False','save_to_usd=True')
    # Fold the massless virtual opening command into the first real finger.
    # Preserve the official q2=-q1 mechanical coupling, not independent jaws.
    subprocess.run([sys.executable,str(ROOT/'scripts/prepare_piper_description.py')],check=True)
    robot=ET.parse(ROOT/'config/piper.urdf').getroot()
    for item in list(robot):
        if (item.tag=='link' and item.get('name')=='gripper_link') or (item.tag=='joint' and item.get('name')=='gripper'):robot.remove(item)
    for joint in robot.findall('joint'):
        for mimic in joint.findall('mimic'):joint.remove(mimic)
        if joint.get('name')=='gripper_joint2':ET.SubElement(joint,'mimic',joint='gripper_joint1',multiplier='-1.0',offset='0.0')
    ET.ElementTree(robot).write(ROOT/'config/piper_sim.urdf',encoding='unicode')
    # Importer proxy is prepared by us; original command URDF is preserved.
    source=source.replace("subprocess.run([sys.executable,str(ROOT/'scripts/prepare_piper_description.py')],check=True)","pass # already prepared")
    source=source.replace('usd_path=str(asset_cache)', 'usd_path=str(asset_cache / model_hash)')
    monitors="""scene_monitor_views=[]
for monitor_name in ['link1','link2','link3','link4','link5','link6','flange_link','gripper_base','gripper_link1','gripper_link2']:
    filters=list(diagnostic_contact_paths)
    if monitor_name not in ['gripper_link1','gripper_link2']:filters.append(contact_target_path)
    scene_monitor_views.append((monitor_name,world.scene.add(RigidPrim(
        prim_paths_expr=one_prim(monitor_name,'/World/Piper'),name='scene_contact_'+monitor_name,
        contact_filter_prim_paths_expr=filters,track_contact_forces=True,
        prepare_contact_sensors=True,max_contact_count=256))))
"""
    source=source.replace("tcp = SingleXFormPrim(one_prim('tcp_link', '/World/Piper'))",monitors+"tcp = SingleXFormPrim(one_prim('tcp_link', '/World/Piper'))")
    source=source.replace("robot = world.scene.add(SingleArticulation('/World/Piper', name='r1a7'))","robot = world.scene.add(SingleArticulation('/World/Piper', name='piper'))\n    robot.set_world_pose(BASE_POSE[:3], np.roll(Rotation.from_euler('z',BASE_POSE[3],degrees=True).as_quat(),1))")
    os.environ['R1A7_BASE_POSE']=','.join(map(str,base));os.environ['R1A7_SUPPORT_BOTTOM_Z']='-.56';os.environ['R1A7_PEDESTAL_SIZE']='.10,.10,.46'
    os.environ['R1A7_RUN_DIR']=str(a.output);os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y'
    s=json.loads((a.source/'report.json').read_text());place=s['asset_installation'];old=sys.argv
    sys.argv=[str(legacy),'--gpu',str(a.gpu),'--asset-root',str(a.asset_root),'--asset-x',str(place['x_m']),'--asset-y',str(place['y_m']),'--asset-yaw-deg',str(place['yaw_deg']),'--fixture-height-m',str(place['fixture_height_m']),'--record-overview','--enable-isolation-diagnostics']
    scene={'__name__':'piper_mobile_scene','__file__':str(legacy)}
    exec(compile(source,str(legacy),'exec'),scene);sys.argv=old;scene['bootstrap_hash']=hashlib.sha256(text.encode()).hexdigest();return scene


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--asset-root',type=Path,required=True);p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--gpu',type=int,default=6);p.add_argument('--contact-audit-only',action='store_true',help='Export measured closure cooking data and stop before any door-opening motion');p.add_argument('--overview-only',action='store_true');p.add_argument('--gripper-sanity',action='store_true');p.add_argument('--candidate-index',type=int);p.add_argument('--deadline-shanghai');p.add_argument('--geometry-python',default=os.environ.get('PIPER_GEOMETRY_PYTHON','/data1/home/rangeryx/.conda/envs/anygrasp/bin/python'))
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);report=json.loads(a.plan.read_text())
    if a.contact_audit_only and report['mode']=='mobile_recovery':raise RuntimeError('contact audit requires fixed base')
    chosen=report['best'] if a.candidate_index is None else report['rows'][a.candidate_index]
    original=json.loads((a.source/'report.json').read_text())['robot_base_pose'];base=original if report['mode']=='mobile_recovery' else (chosen['base'] if chosen else original)
    zone=ZoneInfo('Asia/Shanghai');now=datetime.now(zone);deadline=datetime.fromisoformat(a.deadline_shanghai) if a.deadline_shanghai else now.replace(hour=5,minute=0,second=0,microsecond=0)
    if deadline.tzinfo is None:deadline=deadline.replace(tzinfo=zone)
    if not a.deadline_shanghai and deadline<now:deadline+=timedelta(days=1)
    if now>=deadline:raise RuntimeError('deadline passed')
    scene=bootstrap(a,base);app=scene['app'];world=scene['world'];robot=scene['robot'];tcp=scene['tcp'];door=scene['articulation'];dt=scene['DT'];names=scene['names'];arm=scene['arm'];fingers=scene['fingers'];controller=scene['controller'];stage=scene['stage']
    from isaacsim.core.api.objects import FixedCuboid
    from isaacsim.core.prims import SingleXFormPrim
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import UsdPhysics,PhysxSchema,UsdGeom,Usd
    import cv2
    chassis=world.scene.add(FixedCuboid('/World/mobile_base',name='mobile_base',position=[base[0],base[1],-.66],orientation=np.roll(Rotation.from_euler('z',base[3],degrees=True).as_quat(),1),scale=[.34,.30,.20],physics_material=scene['mat']))
    pole=SingleXFormPrim('/World/r1a7_pedestal');base_link=SingleXFormPrim(scene['one_prim']('base_link','/World/Piper'))
    qtarget=np.array(robot.get_joint_positions()).copy();qtarget[fingers]=[.05,-.05]
    for _ in range(24):world.step(render=True)
    actual_base=matrix(*base_link.get_world_pose());expected=matrix(base[:3],np.roll(Rotation.from_euler('z',base[3],degrees=True).as_quat(),1))
    if np.linalg.norm(actual_base[:3,3]-expected[:3,3])>.002:raise RuntimeError('fixed-base world pose does not match installation: '+str(actual_base))
    # Contact surfaces are extracted from the official flat distal mesh faces.
    import trimesh
    pads={};pad_triangles={};finger_xforms={}
    for name in ('gripper_link1','gripper_link2'):
        mesh=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{name}.stl',force='mesh');inner=mesh.vertices[:,2].min();face=np.max(np.abs(mesh.triangles[:,:,2]-inner),axis=1)<1e-6;points=mesh.triangles[face].reshape(-1,3);pads[name]=np.array([points.min(0),points.max(0)]);pad_triangles[name]=mesh.triangles[face];finger_xforms[name]=SingleXFormPrim(scene['one_prim'](name,'/World/Piper'))
    from articulated_demo.kinematics import URDFChain
    chain=URDFChain(ROOT/'config/piper.urdf');q0=robot.get_joint_positions();expected_tcp=actual_base@chain.root_to_link('tcp_link',dict(zip(names,q0)))
    measured_tcp=matrix(*tcp.get_world_pose());position_error=float(np.linalg.norm(expected_tcp[:3,3]-measured_tcp[:3,3]));rotation_error=float(Rotation.from_matrix(expected_tcp[:3,:3]@measured_tcp[:3,:3].T).magnitude())
    if position_error>.001 or rotation_error>.001:raise RuntimeError('URDF/Isaac FK_TCP_MISMATCH')
    collision_audit=[]
    for prim in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
        if str(prim.GetPath()).startswith('/World/Piper') and prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
            mesh=UsdGeom.Mesh(prim);points=np.array(mesh.GetPointsAttr().Get());approx=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            collision_audit.append({'source':'authored USD mesh; see cooked_shapes.json for actual cooking output','path':str(prim.GetPath()),'approximation':str(approx),'local_bounds':np.array([points.min(0),points.max(0)]).tolist(),'world_transform':np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T.tolist()})
            if approx!='convexHull':raise RuntimeError('ISAAC_COLLISION_REPRESENTATION_MISMATCH:'+str(approx))
    (a.output/'collision_audit.json').write_text(json.dumps(collision_audit,indent=2))
    # Official 10 N finger effort limits: do not increase to conceal contact failure.
    for prim in stage.Traverse():
        if prim.GetName() in ('gripper_joint1','gripper_joint2') and prim.IsA(UsdPhysics.PrismaticJoint):
            limit=UsdPhysics.DriveAPI.Get(prim,'linear').GetMaxForceAttr().Get()
            if limit is None or abs(float(limit)-10.)>1e-6:raise RuntimeError('imported finger effort limit differs from official 10 N')
    # Re-authoring maxForce previously rebuilt runtime-only zero-gain drives.
    # Persist the existing gains and read-check the original effort limits.
    camera=scene['overview'][0];record=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(a.output/'execution.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
    rows=[];tick=0;phase='FIXED_BASE';status='STARTED';reference=None;loss_steps=0
    limits=ET.parse(ROOT/'config/piper.urdf').getroot();limits=np.array([[float(limits.find(f"joint[@name='joint{i}']/limit").get(k)) for k in ('lower','upper')] for i in range(1,7)])
    result={'kind':'real finger contact; passive door; no ideal attachment','mode':report['mode'],'base_initial':base,'base_final':base,'base_locked_during_manipulation':True,'bootstrap_sha256':scene['bootstrap_hash'],'finger_effort_limit_n':10.,'friction_unchanged':True,'asset_parameters_unchanged':True,'joint_names':names,'selected':chosen,'legal_real_grasp':False}
    result['fk_tcp_check']={'position_error_m':position_error,'rotation_error_rad':rotation_error}
    result['platform_assumption']={'type':'parameterized simulation chassis, not an official mobile robot','chassis_size_m':[.34,.30,.20],'chassis_center_z_m':-.66,'mast_size_m':[.10,.10,base[2]+.56],'floor_z_m':-.76,'movement':'kinematic SE2 reposition, then fixed root'}
    result['door_mass_model']='inherited baseline geometry COM/inertia approximation; source mass preserved; no new tuning'
    result['controller_gains']={'arm_kp':10000.,'arm_kd':400.,'finger_kp':1000.,'finger_kd':40.,'source':'existing PiPER simulation profile, not measured hardware gains'}
    result['gripper_mechanical_coupling']='official mimic folded to q2=-q1; virtual command w=2*q1'
    result['simulation_urdf_sha256']=hashlib.sha256((ROOT/'config/piper_sim.urdf').read_bytes()).hexdigest()
    def step():
        nonlocal tick,loss_steps
        if datetime.now(zone)>=deadline:raise RuntimeError('CUTOFF_05_00')
        controller.apply_action(ArticulationAction(joint_positions=qtarget));world.step(render=tick%8==0)
        actual_mount=matrix(*base_link.get_world_pose());mount_error=float(np.linalg.norm(actual_mount[:3,3]-np.asarray(base[:3])))
        if phase!='REPOSITION' and mount_error>.001:raise RuntimeError('BASE_NOT_LOCKED')
        q=np.asarray(robot.get_joint_positions());margin=float(np.min(np.minimum(q[arm]-limits[:,0],limits[:,1]-q[arm])))
        forces=[];contacts=[];nonpad=0.
        for name,view in zip(('gripper_link1','gripper_link2'),scene['finger_views']):
            forces.append(float(np.linalg.norm(view.get_contact_force_matrix(dt=dt))))
            force,point,normal,separation,count,index=[np.asarray(x) for x in view.get_contact_force_data(dt=dt)]
            start=int(index[0,0]);amount=int(count[0,0]);T=matrix(*finger_xforms[name].get_world_pose());inverse=np.linalg.inv(T);bound=pads[name]
            for k in range(start,start+amount):
                f=float(np.asarray(force[k]).reshape(-1)[0]);local=(inverse@np.r_[point[k],1])[:3]
                projected=local.copy();projected[2]=bound[0,2];triangles=pad_triangles[name];nearest=trimesh.triangles.closest_point(triangles,np.tile(projected,(len(triangles),1)))
                pad=bool(np.min(np.linalg.norm(nearest-projected,axis=1))<.00005 and abs(local[2]-bound[0,2])<.001)
                contacts.append({'finger':name,'point_world':point[k].tolist(),'point_local':local.tolist(),'normal_world':normal[k].tolist(),'separation_m':float(np.asarray(separation[k]).reshape(-1)[0]),'force_n':f,'pad':pad})
                if not pad:nonpad+=abs(f)
        bodyforce=sum(float(np.linalg.norm(v.get_contact_force_matrix(dt=dt))) for v in scene['body_views'])
        scene_forces={name:float(np.linalg.norm(view.get_contact_force_matrix(dt=dt))) for name,view in scene['scene_monitor_views']}
        T=matrix(*tcp.get_world_pose());L=matrix(*scene['door_link'].get_world_pose());rel=np.linalg.inv(L)@T
        slip=0. if reference is None else float(np.linalg.norm(rel[:3,3]-reference[:3,3]))
        state={'t':tick*dt,'phase':phase,'robot_q':q.tolist(),'command_q':qtarget.tolist(),'actual_aperture_m':float(q[fingers[0]]-q[fingers[1]]),'door_angle_deg':float(np.rad2deg(door.get_joint_positions()[0])),'margin_rad':margin,'forces_n':forces,'nonpad_force_n':nonpad,'palm_wrist_force_n':bodyforce,'relative_slip_m':slip,'T_tcp':T.tolist(),'T_moving_link':L.tolist(),'contacts':contacts,
               'T_base':actual_mount.tolist(),'base_position_error_m':mount_error,
               'finger_world_poses':{n:matrix(*f.get_world_pose()).tolist() for n,f in finger_xforms.items()},
               'all_contact_force_by_finger_n':[float(np.linalg.norm(v.get_net_contact_forces(dt=dt))) for v in scene['finger_views']],
               'door_fixture_static_contact_n':float(np.linalg.norm(scene['door_contacts'].get_contact_force_matrix(dt=dt))),
               'scene_contact_by_robot_link_n':scene_forces,
               'explicit_dof_actuation_efforts_api':np.asarray(robot._articulation_view._physics_view.get_dof_actuation_forces())[0].tolist()};rows.append(state)
        if tick%8==0:
            image=np.asarray(camera.get_rgba())[:,:,:3].copy();cv2.rectangle(image,(0,0),(1280,80),(10,10,10),-1)
            cv2.putText(image,f'PiPER {report["mode"]} | {phase} | REAL CONTACT / PASSIVE DOOR',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
            cv2.putText(image,f'door={state["door_angle_deg"]:.2f} deg  margin={margin:.3f} rad  slip={slip*1000:.1f} mm',(12,61),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1);record.stdin.write(np.ascontiguousarray(image).tobytes())
        tick+=1
        if margin<=.05:raise RuntimeError('LOW_JOINT_MARGIN')
        if phase in ('PREGRASP','APPROACH','CLOSE','OPEN_DOOR','HOLD') and (nonpad>.01 or bodyforce>.01):raise RuntimeError('NON_PAD_CONTACT')
        if phase in ('PREGRASP','APPROACH','CLOSE','OPEN_DOOR','HOLD') and max(scene_forces.values(),default=0)>.01:raise RuntimeError('SCENE_COLLISION')
        if phase in ('OPEN_DOOR','HOLD'):
            loss_steps=loss_steps+1 if min(forces)<.2 else 0
            if loss_steps>24:raise RuntimeError('CONTACT_LOSS')
            if slip>.005:raise RuntimeError('GRASP_SLIP')
        return state
    def move(q,duration=2.):
        old=qtarget[arm].copy()
        for f in np.linspace(0,1,int(duration/dt)):
            blend=f*f*(3-2*f);qtarget[arm]=old+blend*(np.asarray(q)-old);step()
    try:
        if a.gripper_sanity:
            phase='GRIPPER_SANITY'
            for opening in [.1,0,.1]:
                qtarget[fingers]=[opening/2,-opening/2]
                for _ in range(480):step()
            result['gripper_sanity_endpoints']=[{'command':opening,'q':rows[(i+1)*480-1]['robot_q'][6:],'forces':rows[(i+1)*480-1]['all_contact_force_by_finger_n']} for i,opening in enumerate([.1,0,.1])]
            status='GRIPPER_SANITY_COMPLETE' if all(np.max(np.abs(np.array(v['q'])-np.array([v['command']/2,-v['command']/2])))<.002 for v in result['gripper_sanity_endpoints']) else 'GRIPPER_CONTROL_ERROR'
        elif a.overview_only:
            phase='OVERVIEW_ONLY'
            for _ in range(240):step()
            status='OVERVIEW_ONLY'
        elif chosen is None:status='NO_SAFE_GRASP_PATH'
        else:
            if report['mode']=='mobile_recovery':
                request={'kind':'base_reposition','source':str(a.source),'asset_root':str(a.asset_root),'target_points':str(a.plan.parent/'target_points.npy'),'selected':chosen,'actual_q':rows[-1]['robot_q'] if rows else robot.get_joint_positions().tolist(),'actual_door_angle_deg':0.,'initial_base':base}
                (a.output/'base_route_request.json').write_text(json.dumps(request))
                subprocess.run(['env','-u','PYTHONPATH',a.geometry_python,str(ROOT/'scripts/check_piper_closed_path.py'),str(a.output/'base_route_request.json'),str(a.output/'base_route_check.json')],check=True)
                result['base_route_check']=json.loads((a.output/'base_route_check.json').read_text())
                if result['base_route_check']['status']!='BASE_ROUTE_VALID':raise RuntimeError(result['base_route_check']['status'])
                phase='REPOSITION';end=np.asarray(chosen['base']);begin=np.asarray(base)
                end[3]=begin[3]+(end[3]-begin[3]+180)%360-180
                for f in np.linspace(0,1,480):
                    B=begin+f*(end-begin);quat=np.roll(Rotation.from_euler('z',B[3],degrees=True).as_quat(),1)
                    robot.set_world_pose(B[:3],quat);chassis.set_world_pose([B[0],B[1],-.66],quat);pole.set_world_pose([B[0],B[1],(B[2]-.56)/2],quat);step()
                base=end.tolist();result['base_final']=base;phase='BASE_LOCKED'
                for _ in range(240):step()
                if np.linalg.norm(base_link.get_world_pose()[0]-np.asarray(base[:3]))>.002:raise RuntimeError('BASE_REPOSITION_FAILED')
            phase='PREGRASP'
            for q in chosen['preplan'][1:]:move(q,2.)
            phase='APPROACH'
            for q in chosen['approach']:move(q,.12)
            phase='CLOSE'
            for opening in np.linspace(.1,0,480):qtarget[fingers]=[opening/2,-opening/2];step()
            for _ in range(240):closed=step()
            result['actual_closure']=closed
            if a.contact_audit_only:
                from piper_mobile_demo.cooked_geometry import export_cooked
                export_cooked(stage,a.output/'cooked_shapes.json')
                subprocess.run(['env','-u','PYTHONPATH',a.geometry_python,str(ROOT/'scripts/validate_piper_cooked_contact.py'),str(a.output/'cooked_shapes.json'),str(a.output/'contact_validation.json')],check=True)
                result['contact_validation']=json.loads((a.output/'contact_validation.json').read_text())
                (a.output/'closure_observations.json').write_text(json.dumps(rows))
                subprocess.run(['env','-u','PYTHONPATH',a.geometry_python,str(ROOT/'scripts/replay_piper_contact_geometry.py'),str(a.output/'cooked_shapes.json'),str(a.output/'closure_observations.json'),str(a.output/'closure_replay.json')],check=True)
                result['closure_replay']=json.loads((a.output/'closure_replay.json').read_text())['summary']
                result['pad_only_contact_verified']=min(closed['forces_n'])>=.2 and closed['nonpad_force_n']<=.01 and closed['palm_wrist_force_n']<=.01 and all(result['closure_replay']['invalid_samples_by_mode'].get(mode,1)==0 for mode in ['physx_cooked','raw_finger_raw_target'])
                result['contact_point_label_provenance']='legacy application projection within 1 mm of original flat patch; not a PhysX material/shape identity'
                result['door_opening_executed']=False
                raise RuntimeError('CONTACT_AUDIT_COMPLETE_NO_OPENING')
            if min(closed['forces_n'])<.2:raise RuntimeError('BAD_CONTACT')
            if abs(closed['robot_q'][fingers[0]]+closed['robot_q'][fingers[1]])>.0002:raise RuntimeError('GRIPPER_COUPLING_ERROR')
            result['legacy_contact_point_pad_label']=True
            result['contact_point_label_provenance']='application projection within 1 mm; not native PhysX pad classification'
            result['physx_pad_only_closure']=True # retained legacy key; strict offline audit still controls legality
            request={'source':str(a.source),'asset_root':str(a.asset_root),'target_points':str(a.plan.parent/'target_points.npy'),'selected':chosen,'actual_q':closed['robot_q'],'actual_door_angle_deg':closed['door_angle_deg']}
            (a.output/'closed_path_request.json').write_text(json.dumps(request))
            subprocess.run(['env','-u','PYTHONPATH',a.geometry_python,str(ROOT/'scripts/check_piper_closed_path.py'),str(a.output/'closed_path_request.json'),str(a.output/'closed_path_check.json')],check=True)
            result['closed_path_check']=json.loads((a.output/'closed_path_check.json').read_text())
            result['legal_real_grasp']=bool(result['closed_path_check'].get('initial_state_valid'))
            if result['closed_path_check']['status']!='CLOSED_PATH_VALID':raise RuntimeError(result['closed_path_check']['status'])
            reference=np.linalg.inv(np.asarray(closed['T_moving_link']))@np.asarray(closed['T_tcp'])
            phase='OPEN_DOOR'
            for waypoint in chosen['arc']:move(waypoint['q'],.3)
            phase='HOLD'
            hold_start=len(rows)
            for _ in range(480):step()
            result['minimum_held_door_angle_deg']=min(r['door_angle_deg'] for r in rows[hold_start:])
            status='SUCCESS' if result['minimum_held_door_angle_deg']>=20 else 'PARTIAL_OPENING'
    except BaseException as e:
        status=str(e);result['exception_type']=type(e).__name__;print('STOP',status,flush=True)
    finally:
        result.update(status=status,sample_count=len(rows),max_actual_door_angle_deg=max((r['door_angle_deg'] for r in rows),default=0),minimum_joint_margin_rad=min((r['margin_rad'] for r in rows),default=None),maximum_relative_slip_m=max((r['relative_slip_m'] for r in rows),default=None))
        (a.output/'report.json').write_text(json.dumps(result,indent=2));(a.output/'observations.json').write_text(json.dumps(rows));record.stdin.close();record.wait(timeout=30);print(json.dumps(result,indent=2),flush=True);app.close()

if __name__=='__main__':main()
