"""Canonical-gated physical closure test of the frozen formal grasp; no opening arc."""
from piper_mobile_execute import *
from piper_mobile_demo.contact_ownership import NativeOwnershipReports
from piper_mobile_demo.cooked_geometry import export_cooked
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ownership',type=Path,default=ROOT/'config/piper_contact_ownership.json');p.add_argument('--canonical-validation',type=Path,required=True);p.add_argument('--pull-plan',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--asset-root',type=Path,required=True);p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--gpu',type=int,default=6);p.add_argument('--contact-audit-only',action='store_true',help='Export measured closure cooking data and stop before any door-opening motion');p.add_argument('--overview-only',action='store_true');p.add_argument('--gripper-sanity',action='store_true');p.add_argument('--candidate-index',type=int);p.add_argument('--deadline-shanghai');p.add_argument('--geometry-python',default=os.environ.get('PIPER_GEOMETRY_PYTHON','/data1/home/rangeryx/.conda/envs/anygrasp/bin/python'))
    a=p.parse_args();canonical=json.loads(a.canonical_validation.read_text());
    if not canonical.get('canonical_acceptance') or canonical.get('ownership_manifest_sha256')!=hashlib.sha256(a.ownership.read_bytes()).hexdigest():raise RuntimeError('canonical full-trajectory test has not passed')
    a.output.mkdir(parents=True,exist_ok=True);report=json.loads(a.plan.read_text())
    if report['mode']=='mobile_recovery':raise RuntimeError('contact audit requires fixed base')
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
    pole=SingleXFormPrim('/World/r1a7_pedestal',reset_xform_properties=False);base_link=SingleXFormPrim(scene['one_prim']('base_link','/World/Piper'),reset_xform_properties=False)
    export_cooked(stage,a.output/'cooked_initial.json')
    subprocess.run(['env','-u','PYTHONPATH',a.geometry_python,str(ROOT/'scripts/associate_piper_target_collider.py'),str(a.output/'cooked_initial.json'),str(a.plan.parent/'target_points.npy'),str(a.output/'target_collider_association.json')],check=True)
    association=json.loads((a.output/'target_collider_association.json').read_text())
    qtarget=np.array(robot.get_joint_positions()).copy();qtarget[fingers]=[.05,-.05]
    for _ in range(24):world.step(render=True)
    actual_base=matrix(*base_link.get_world_pose());expected=matrix(base[:3],np.roll(Rotation.from_euler('z',base[3],degrees=True).as_quat(),1))
    if np.linalg.norm(actual_base[:3,3]-expected[:3,3])>.002:raise RuntimeError('fixed-base world pose does not match installation: '+str(actual_base))
    finger_xforms={name:SingleXFormPrim(scene['one_prim'](name,'/World/Piper'),reset_xform_properties=False) for name in ('gripper_link1','gripper_link2')}
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
    camera=scene['overview'][0];record=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','16','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(a.output/'execution.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
    rows=[];tick=0;phase='FIXED_BASE';status='STARTED';reference=None;loss_steps=0
    limits=ET.parse(ROOT/'config/piper.urdf').getroot();limits=np.array([[float(limits.find(f"joint[@name='joint{i}']/limit").get(k)) for k in ('lower','upper')] for i in range(1,7)])
    result={'kind':'real finger contact; passive door; no ideal attachment','mode':report['mode'],'base_initial':base,'base_final':base,'base_locked_during_manipulation':True,'bootstrap_sha256':scene['bootstrap_hash'],'finger_effort_limit_n':10.,'friction_unchanged':True,'asset_parameters_unchanged':True,'joint_names':names,'selected':chosen,'legal_real_grasp':False}
    result['fk_tcp_check']={'position_error_m':position_error,'rotation_error_rad':rotation_error}
    result['platform_assumption']={'type':'parameterized simulation chassis, not an official mobile robot','chassis_size_m':[.34,.30,.20],'chassis_center_z_m':-.66,'mast_size_m':[.10,.10,base[2]+.56],'floor_z_m':-.76,'movement':'kinematic SE2 reposition, then fixed root'}
    result['door_mass_model']='inherited baseline geometry COM/inertia approximation; source mass preserved; no new tuning'
    result['controller_gains']={'arm_kp':10000.,'arm_kd':400.,'finger_kp':1000.,'finger_kd':40.,'source':'existing PiPER simulation profile, not measured hardware gains'}
    result['gripper_mechanical_coupling']='official mimic folded to q2=-q1; virtual command w=2*q1'
    result['simulation_urdf_sha256']=hashlib.sha256((ROOT/'config/piper_sim.urdf').read_bytes()).hexdigest()
    def sample_physics():
        q=np.asarray(robot.get_joint_positions());poses={}
        for n,view in zip(('gripper_link1','gripper_link2'),scene['finger_views']):
            p,r=view.get_world_poses();poses[n]=matrix(p[0],r[0]).tolist()
        p,r=scene['door_contacts'].get_world_poses()
        return {'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':float(np.min(np.minimum(q[arm]-limits[:,0],limits[:,1]-q[arm]))),'finger_world_poses':poses,'T_moving_link':matrix(p[0],r[0]).tolist()}
    native=NativeOwnershipReports(stage,[scene['contact_target_path']],dt,association['allowed_pad_targets'],world=world,sample_provider=sample_physics)
    def step():
        nonlocal tick,loss_steps
        if datetime.now(zone)>=deadline:raise RuntimeError('CUTOFF_05_00')
        pre_poses={n:matrix(*frame.get_world_pose()).tolist() for n,frame in finger_xforms.items()};pre_object_pose=matrix(*scene['door_link'].get_world_pose()).tolist();native.clear();controller.apply_action(ArticulationAction(joint_positions=qtarget));world.step(render=tick%8==0)
        actual_mount=matrix(*base_link.get_world_pose());mount_error=float(np.linalg.norm(actual_mount[:3,3]-np.asarray(base[:3])))
        if phase!='REPOSITION' and mount_error>.001:raise RuntimeError('BASE_NOT_LOCKED')
        q=np.asarray(robot.get_joint_positions());margin=float(np.min(np.minimum(q[arm]-limits[:,0],limits[:,1]-q[arm])))
        ownership=native.state();contacts=ownership['contacts'];forces=[ownership['pad_forces_n'][n] for n in ('gripper_link1','gripper_link2')];nonpad=sum(x['force_n'] for x in contacts if x['owner']!='pad')
        bodyforce=sum(float(np.linalg.norm(v.get_contact_force_matrix(dt=dt))) for v in scene['body_views'])
        scene_forces={name:float(np.linalg.norm(view.get_contact_force_matrix(dt=dt))) for name,view in scene['scene_monitor_views']}
        T=matrix(*tcp.get_world_pose());L=matrix(*scene['door_link'].get_world_pose());rel=np.linalg.inv(L)@T
        slip=0. if reference is None else float(np.linalg.norm(rel[:3,3]-reference[:3,3]))
        state={'t':tick*dt,'physics_time_s':ownership['physics_time_s'],'phase':phase,'robot_q':q.tolist(),'command_q':qtarget.tolist(),'actual_aperture_m':float(q[fingers[0]]-q[fingers[1]]),'door_angle_deg':float(np.rad2deg(door.get_joint_positions()[0])),'margin_rad':margin,'forces_n':forces,'nonpad_force_n':nonpad,'palm_wrist_force_n':bodyforce,'relative_slip_m':slip,'T_tcp':T.tolist(),'T_moving_link':L.tolist(),'contacts':contacts,'ownership':ownership,'pre_step_finger_world_poses':pre_poses,'pre_step_moving_link_world_pose':pre_object_pose,'geometry_pose_time':'after physics step; native contact generation precedes integration','aperture_m':float(q[fingers[0]]-q[fingers[1]]),
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
        if ownership['metal_contacts'] or ownership['pad_target_violations']:raise RuntimeError('NATIVE_FORBIDDEN_CONTACT')
        if phase in ('PREGRASP','APPROACH','CLOSE','PULL_DIAGNOSTIC','HOLD') and bodyforce>.01:raise RuntimeError('NON_PAD_CONTACT')
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
            phase='PREGRASP'
            for q in chosen['preplan'][1:]:move(q,2.)
            phase='APPROACH'
            for q in chosen['approach']:move(q,.12)
            phase='CLOSE'
            for opening in np.linspace(.1,0,480):qtarget[fingers]=[opening/2,-opening/2];step()
            for _ in range(240):closed=step()
            result['actual_closure']=closed
            export_cooked(stage,a.output/'cooked_closed.json')
            subprocess.run(['env','-u','PYTHONPATH',a.geometry_python,str(ROOT/'scripts/validate_piper_owned_contact.py'),str(a.output/'cooked_closed.json'),str(a.output/'closure_validation.json'),'--ownership',str(a.ownership)],check=True)
            validation=json.loads((a.output/'closure_validation.json').read_text())
            result['legal_real_grasp']=min(closed['forces_n'])>=.2 and closed['ownership']['metal_contacts']==0 and closed['ownership']['pad_target_violations']==0 and validation['geometry_safe']
            if not result['legal_real_grasp']:raise RuntimeError('FORMAL_CLOSURE_GEOMETRY_OR_CONTACT_FAILURE')
            closure_rows=[r for r in native.physics_steps if r['phase']=='CLOSE'];(a.output/'closure_observations.json').write_text(json.dumps(closure_rows))
            subprocess.run(['env','-u','PYTHONPATH',a.geometry_python,str(ROOT/'scripts/validate_piper_owned_contact.py'),str(a.output/'cooked_closed.json'),str(a.output/'closure_replay.json'),'--ownership',str(a.ownership),'--observations',str(a.output/'closure_observations.json')],check=True)
            replay=json.loads((a.output/'closure_replay.json').read_text())
            if not replay['unified_safe_all_samples']:raise RuntimeError('FORMAL_CLOSURE_REPLAY_FORBIDDEN_CONTACT')
            phase='PULL_DIAGNOSTIC';pull=json.loads(a.pull_plan.read_text())
            for waypoint in pull['waypoints']:move(waypoint['q'],.25)
            phase='HOLD'
            for _ in range(240):step()
            status='FORMAL_NATIVE_PAD_ONLY_AND_SMALL_PULL_PENDING_REPLAY'
    except BaseException as e:
        status=str(e);result['exception_type']=type(e).__name__;print('STOP',status,flush=True)
    finally:
        export_cooked(stage,a.output/'cooked_final.json')
        result['door_opening_arc_executed']=False;result['target_collider_association']=association;result['owned_colliders']=scene['owned_contact_colliders'];result['native_contact_classification']='native collider identity only';result['native_metal_contact_samples']=sum(r['ownership']['metal_contacts']>0 for r in rows);result['pad_wrong_target_samples']=sum(r['ownership']['pad_target_violations']>0 for r in rows)
        result['physics_step_samples']=len(native.physics_steps);(a.output/'physics_steps.json').write_text(json.dumps(native.physics_steps))
        result.update(status=status,sample_count=len(rows),max_actual_door_angle_deg=max((r['door_angle_deg'] for r in rows),default=0),minimum_joint_margin_rad=min((r['margin_rad'] for r in rows),default=None),maximum_relative_slip_m=max((r['relative_slip_m'] for r in rows),default=None))
        (a.output/'report.json').write_text(json.dumps(result,indent=2));(a.output/'observations.json').write_text(json.dumps(rows));record.stdin.close();record.wait(timeout=30);print(json.dumps(result,indent=2),flush=True);app.close()

if __name__=='__main__':main()
