"""Isolated Isaac instrumentation for ObjectScan and PayloadID.

The frozen grasp simulator is source-adapted in a separate entry point.  No
executor, grasp ranking, planning rule, or success threshold is changed.
"""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'calibration'))
adapter=(ROOT/'calibration/sim_unseen.py').read_text()
end="exec(compile('import os\\n'+text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))"
assert adapter.count(end)==1
ns={'__file__':str(ROOT/'calibration/sim_unseen.py')}
exec(compile(adapter.replace(end,''),str(ROOT/'calibration/sim_unseen.py'),'exec'),ns)
source=ns['source'];text=ns['text']
text=text.replace(
    '                    results.clear();calib.phase="RESET";support.reset()',
    '''                    results.clear()
                    if diagnostic_fixed_joint is not None:
                        stage.RemovePrim('/World/diagnostic_fixed_joint')
                        diagnostic_fixed_joint=None
                    diagnostic_attachment_mode='NORMAL'
                    calib.phase="RESET";support.reset()''',
    1,
)

camera_hook="camera.set_clipping_range(.05,2.)"
camera_code=camera_hook+r'''
# Independent close-range scan camera.  The AnyGrasp overhead camera remains
# unchanged.  A runner may provide comma-separated values for the held-object
# station; the default preserves the earlier ObjectScan acquisition.
def _scan_vec(name, default):
    raw=os.environ.get(name)
    if not raw:return np.asarray(default,dtype=float)
    try:
        value=np.asarray([float(x.strip()) for x in raw.split(',')],dtype=float)
        if value.shape!=(3,) or not np.isfinite(value).all():raise ValueError
        return value
    except Exception as exc:
        raise RuntimeError(f'invalid {name}: {raw}') from exc
SCAN_CAMERA_POSITION=_scan_vec('REAL2SIM_SCAN_CAMERA_POSITION',[.88,-.58,.48])
SCAN_LOOK_AT=_scan_vec('REAL2SIM_SCAN_LOOK_AT',[.50,0.,.32])
scan_z=(SCAN_LOOK_AT-SCAN_CAMERA_POSITION);scan_z/=np.linalg.norm(scan_z)
scan_x=np.cross(scan_z,np.array([0.,0.,1.]));scan_x/=np.linalg.norm(scan_x);scan_y=np.cross(scan_z,scan_x)
T_B_SCAN_C=np.eye(4);T_B_SCAN_C[:3,:3]=np.column_stack([scan_x,scan_y,scan_z]);T_B_SCAN_C[:3,3]=SCAN_CAMERA_POSITION
scan_camera=Camera('/World/scan_camera',position=SCAN_CAMERA_POSITION,resolution=(640,480),frequency=30)
scan_camera.set_world_pose(position=SCAN_CAMERA_POSITION,orientation=np.roll(Rotation.from_matrix(T_B_SCAN_C[:3,:3]).as_quat(),1),camera_axes='ros')
scan_camera.set_focal_length(1.8);scan_camera.set_horizontal_aperture(1.44);scan_camera.set_vertical_aperture(1.08);scan_camera.set_clipping_range(.05,2.)
'''
assert text.count(camera_hook)==1;text=text.replace(camera_hook,camera_code)
init_hook="camera.add_instance_id_segmentation_to_frame()"
init_code=init_hook+"\nscan_camera.initialize();scan_camera.add_distance_to_image_plane_to_frame();scan_camera.add_instance_id_segmentation_to_frame()"
assert text.count(init_hook)==1;text=text.replace(init_hook,init_code)

old="commands=queue.Queue(); state={}; lock=threading.Lock(); tick=240; active=None; target=q.copy(); results={}; history=[]"
new=old+"""
payload_records=[];payload_recording=False;payload_record_mode=None;payload_guard_reference=None;payload_guard_abort=None
payload_motion_events=[];payload_motion_violation_active=False
MASS_COM_ONLY = os.environ.get('PAYLOAD_MASS_COM_ONLY','0') == '1'
# A static gravity regressor needs the TCP Jacobian, but the pose is constant
# during each hold.  Refresh at a fixed 5 Hz (default) and reuse the measured
# value between refreshes; q/dq/torque telemetry still records every 240 Hz
# tick.  This is a calibration recording optimization, not a grasp change.
jacobian_refresh_period=max(1,int(os.environ.get('PAYLOAD_JACOBIAN_REFRESH_TICKS','48')))
jacobian_cache=np.zeros((6,7),dtype=float);jacobian_tick=-1
diagnostic_fixed_joint=None;diagnostic_attachment_mode=os.environ.get('PAYLOAD_ATTACHMENT_MODE','NORMAL').upper()
articulation_view=robot._articulation_view
body_names=list(articulation_view.body_names)
hand_body_index=next(i for i,n in enumerate(body_names) if n=='fr3_hand')
import xml.etree.ElementTree as _payload_xml
hand_com_local=np.fromstring(_payload_xml.parse(str(ROOT/'config/fr3.urdf')).getroot().find("link[@name='fr3_hand']/inertial/origin").attrib['xyz'],sep=' ')
print('REAL2SIM_JACOBIAN_BODY',hand_body_index,body_names,flush=True)
"""
assert text.count(old)==1;text=text.replace(old,new)

hook="                elif op=='capture':"
commands=r'''                elif op=='diagnostic_fixed_joint':
                    # Calibration-only rigid attachment.  The joint frame is
                    # placed at the current object pose, so this preserves
                    # the measured hand/object transform at the instant of
                    # attachment.  It is never enabled by the production
                    # Real2Sim pipeline.
                    from pxr import Sdf,Gf,UsdPhysics
                    if diagnostic_fixed_joint is not None:
                        stage.RemovePrim('/World/diagnostic_fixed_joint')
                    hand_path=finger_paths[0].rsplit('/',1)[0]
                    pp,pq=palm.get_world_pose(); bp0,bq0=box.get_world_pose()
                    H0=np.eye(4);H0[:3,:3]=Rotation.from_quat(np.roll(pq,-1)).as_matrix();H0[:3,3]=pp
                    H1=np.eye(4);H1[:3,:3]=Rotation.from_quat(np.roll(bq0,-1)).as_matrix();H1[:3,3]=bp0
                    T01=np.linalg.inv(H0)@H1
                    q01=Rotation.from_matrix(T01[:3,:3]).as_quat()
                    diagnostic_fixed_joint=UsdPhysics.FixedJoint.Define(stage,Sdf.Path('/World/diagnostic_fixed_joint'))
                    diagnostic_fixed_joint.CreateBody0Rel().SetTargets([Sdf.Path(hand_path)])
                    diagnostic_fixed_joint.CreateBody1Rel().SetTargets([Sdf.Path('/World/box')])
                    diagnostic_fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(x) for x in T01[:3,3]]))
                    diagnostic_fixed_joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(q01[3]),Gf.Vec3f(*[float(x) for x in q01[:3]])))
                    diagnostic_fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.,0.,0.))
                    diagnostic_fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.,Gf.Vec3f(0.,0.,0.)))
                    diagnostic_fixed_joint.CreateBreakForceAttr().Set(1.0e12)
                    diagnostic_fixed_joint.CreateBreakTorqueAttr().Set(1.0e12)
                    diagnostic_attachment_mode='FIXED'
                    results[token]={'ok':True,'mode':'FIXED','body0':hand_path,'body1':'/World/box','T_hand_object':T01.tolist()}
                elif op=='payload_guard':
                    qguard=support.query()
                    # A FixedJoint replay is a calibration-only rigid-body
                    # diagnostic.  It may have no physical finger contact
                    # after direct placement, so the ordinary bilateral
                    # contact gate is bypassed only for that explicit mode;
                    # production payload captures remain fail-closed.
                    stable=bool((qguard.get('passed') and qguard.get('currently_clear') and min(f)>.1)
                                or (diagnostic_attachment_mode=='FIXED' and diagnostic_fixed_joint is not None))
                    results[token]={'ok':True,'free_space_stable':stable,'support':qguard,'forces_N':f.tolist(),'abort':payload_guard_abort}
                elif op=='payload_record_start':
                    mode=cmd.get('mode','payload')
                    qguard=support.query()
                    if mode!='baseline' and not ((qguard.get('passed') and qguard.get('currently_clear') and min(f)>.1)
                                                  or (diagnostic_attachment_mode=='FIXED' and diagnostic_fixed_joint is not None)):
                        raise RuntimeError('PAYLOAD_ID_REQUIRES_FREE_SPACE_STABLE')
                    payload_records=[];payload_recording=True;payload_record_mode=mode;payload_guard_abort=None
                    payload_motion_events=[];payload_motion_violation_active=False
                    Hguard=np.eye(4);Hguard[:3,:3]=Rotation.from_quat(np.roll(tq,-1)).as_matrix();Hguard[:3,3]=tp
                    Oguard=np.eye(4);Oguard[:3,:3]=Rotation.from_quat(np.roll(bq,-1)).as_matrix();Oguard[:3,3]=bp
                    payload_guard_reference=np.linalg.inv(Hguard)@Oguard if mode!='baseline' else None
                    results[token]={'ok':True,'mode':mode,'start_t':tick*DT}
                elif op=='payload_record_stop':
                    payload_recording=False
                    if not payload_records:raise RuntimeError('NO_PAYLOAD_SAMPLES')
                    out=Path(cmd['path']);out.parent.mkdir(parents=True,exist_ok=True)
                    keys=payload_records[0].keys();arrays={k:np.asarray([x[k] for x in payload_records]) for k in keys}
                    arrays['guard_passed']=np.asarray([payload_guard_abort is None],dtype=bool)
                    arrays['mode']=np.asarray([payload_record_mode])
                    arrays['attachment_mode']=np.asarray([diagnostic_attachment_mode])
                    arrays['motion_events_json']=np.asarray([json.dumps(payload_motion_events)])
                    np.savez_compressed(out,**arrays)
                    results[token]={'ok':payload_guard_abort is None,'path':str(out),'samples':len(payload_records),'abort':payload_guard_abort,'motion_events':payload_motion_events}
                elif op=='real2sim_capture':
                    for _ in range(8):world.step(render=True);tick+=1
                    out=Path(cmd['output']);fid=int(cmd['pass_id'])*10000+int(cmd['frame_id']);stem=f'{fid:06d}'
                    for folder in ['rgb','depth','masks','gripper_masks','poses','eval_gt']:(out/folder).mkdir(parents=True,exist_ok=True)
                    rgba=np.asarray(scan_camera.get_rgba());depth=np.asarray(scan_camera.get_depth())
                    if rgba.size==0 or depth.size==0:raise RuntimeError('RGBD_UNAVAILABLE')
                    seg=scan_camera.get_current_frame()['instance_id_segmentation'];raw=np.asarray(seg['data']).reshape(depth.shape)
                    labels=seg['info']['idToLabels']
                    object_ids=[int(k) for k,v in labels.items() if str(v)=='/World/box' or str(v).startswith('/World/box/')]
                    gripper_ids=[int(k) for k,v in labels.items() if str(v).startswith('/World/FR3/') and any(x in str(v).lower() for x in ['finger','hand'])]
                    if not object_ids:raise RuntimeError('OBJECT_INSTANCE_MASK_MISSING')
                    object_mask=np.isin(raw,object_ids);gripper_mask=np.isin(raw,gripper_ids)
                    import cv2
                    cv2.imwrite(str(out/'rgb'/f'{stem}.png'),rgba[...,:3][...,[2,1,0]])
                    depth_mm=np.where(np.isfinite(depth)&(depth>0),np.clip(np.rint(depth*1000),0,65535),0).astype(np.uint16)
                    cv2.imwrite(str(out/'depth'/f'{stem}.png'),depth_mm)
                    cv2.imwrite(str(out/'masks'/f'{stem}.png'),object_mask.astype(np.uint8)*255)
                    cv2.imwrite(str(out/'gripper_masks'/f'{stem}.png'),gripper_mask.astype(np.uint8)*255)
                    K=scan_camera.get_intrinsics_matrix();np.savetxt(out/'cam_K.txt',K)
                    tp0,tq0=tcp.get_world_pose();bp0,bq0=box.get_world_pose()
                    H=np.eye(4);H[:3,:3]=Rotation.from_quat(np.roll(tq0,-1)).as_matrix();H[:3,3]=tp0
                    np.savetxt(out/'poses'/f'{stem}_T_B_TCP.txt',H)
                    # GT object pose is sequestered for post-estimation validation only.
                    G=np.eye(4);G[:3,:3]=Rotation.from_quat(np.roll(bq0,-1)).as_matrix();G[:3,3]=bp0
                    np.savetxt(out/'eval_gt'/f'{stem}_T_B_object.txt',G)
                    record={'ok':True,'timestamp_s':tick*DT,'rgb':str(out/'rgb'/f'{stem}.png'),'depth':str(out/'depth'/f'{stem}.png'),
                      'object_mask':str(out/'masks'/f'{stem}.png'),'gripper_mask':str(out/'gripper_masks'/f'{stem}.png'),
                      'T_B_camera':T_B_SCAN_C.tolist(),'T_B_TCP':H.tolist(),'intrinsics':K.tolist(),
                      'object_pixels':int(object_mask.sum()),'gripper_pixels':int(gripper_mask.sum())}
                    (out/'poses'/f'{stem}.json').write_text(json.dumps(record,indent=2));results[token]=record
'''+hook
assert text.count(hook)==1;text=text.replace(hook,commands)

sample_hook="        sample=dict(t=tick*DT"
telemetry=r'''        if payload_recording:
            q_now=np.asarray(robot.get_joint_positions());dq_now=np.asarray(robot.get_joint_velocities())
            tau_now=np.asarray(robot.get_measured_joint_efforts())
            pp_now,pq_now=palm.get_world_pose()
            if MASS_COM_ONLY and jacobian_tick >= 0 and tick % jacobian_refresh_period != 0:
                # Mass/CoM still needs the TCP Jacobian for the gravity
                # regressor.  Reuse a measured Jacobian for at most
                # ``jacobian_refresh_period`` ticks (5 Hz by default at the
                # 240 Hz recorder rate); q is static in
                # these windows, so this preserves the regressor while
                # avoiding a costly PhysX query on every sample.
                J_tcp=jacobian_cache
            else:
                try:
                    J_all=np.asarray(articulation_view.get_jacobians())
                    jac_row=hand_body_index-1 if J_all.shape[-3]==len(body_names)-1 else hand_body_index
                    J_hand=np.asarray(J_all[jac_row] if J_all.ndim==3 else J_all[0,jac_row])
                    hand_com_world=np.asarray(pp_now)+Rotation.from_quat(np.roll(pq_now,-1)).as_matrix()@hand_com_local
                    r_tcp=np.asarray(tp)-hand_com_world;J_tcp=J_hand.copy()
                    J_tcp[:3,:]=J_hand[:3,:]-np.array([[0,-r_tcp[2],r_tcp[1]],[r_tcp[2],0,-r_tcp[0]],[-r_tcp[1],r_tcp[0],0]])@J_hand[3:,:]
                    jacobian_cache=J_tcp.copy();jacobian_tick=tick
                except Exception as exc:
                    payload_guard_abort={'category':'JACOBIAN_UNAVAILABLE','error':str(exc),'t':tick*DT}
                    payload_recording=False
                    if active:results[active['id']]={'ok':False,'reason':'PAYLOAD_SAFETY_ABORT','detail':payload_guard_abort};active=None;target=q_now.copy()
            relative_motion_violation=False
            if payload_recording:
                Hlive=np.eye(4);Hlive[:3,:3]=Rotation.from_quat(np.roll(tq,-1)).as_matrix();Hlive[:3,3]=tp
                Olive=np.eye(4);Olive[:3,:3]=Rotation.from_quat(np.roll(bq,-1)).as_matrix();Olive[:3,3]=bp
                T_TCP_object=np.linalg.inv(Hlive)@Olive
                if payload_record_mode!='baseline':
                    delta=np.linalg.inv(payload_guard_reference)@T_TCP_object
                    drift_t=float(np.linalg.norm(delta[:3,3]));drift_r=float(Rotation.from_matrix(delta[:3,:3]).magnitude())
                    # An explicit FixedJoint replay is a calibration-only
                    # rigid-attachment diagnostic.  It deliberately has no
                    # bilateral finger contact, so the ordinary contact
                    # gate must not abort its recording stream.  Physical
                    # payload captures remain fail-closed here.
                    fixed_diagnostic = (diagnostic_attachment_mode=='FIXED'
                                        and diagnostic_fixed_joint is not None)
                    contact_lost=(min(f)<=.1 or support.query().get('currently_clear') is not True) and not fixed_diagnostic
                    relative_motion_violation=drift_t>.003 or drift_r>np.deg2rad(5)
                    # A transient pose-dependent slip invalidates only the
                    # affected static window; the full stream is retained so
                    # the estimator can reject that window and use the other
                    # identical-pose holds.  Loss of bilateral contact or
                    # table support remains a fail-closed safety abort.
                    if contact_lost:
                        payload_guard_abort={'category':'CONTACT_LOSS','translation_m':drift_t,'rotation_rad':drift_r,'t':tick*DT}
                        payload_recording=False
                        if active:results[active['id']]={'ok':False,'reason':'PAYLOAD_SAFETY_ABORT','detail':payload_guard_abort};active=None;target=q_now.copy()
                    elif relative_motion_violation and not payload_motion_violation_active:
                        payload_motion_events.append({'category':'RELATIVE_SLIP_WINDOW','translation_m':drift_t,'rotation_rad':drift_r,'t':tick*DT})
                    payload_motion_violation_active=relative_motion_violation
            if payload_recording:
                payload_records.append({'t':tick*DT,'q':q_now[arm],'dq':dq_now[arm],'tau':tau_now[arm],
                    'T_B_TCP':Hlive,'T_B_hand':np.block([[Rotation.from_quat(np.roll(pq_now,-1)).as_matrix(),np.asarray(pp_now).reshape(3,1)],[np.zeros((1,3)),np.ones((1,1))]]),
                    'T_TCP_object':T_TCP_object,'relative_motion_violation':relative_motion_violation,
                    'jacobian_TCP':J_tcp[:,:7],'jacobian_convention':'TCP_ORIGIN_BASE_AXES_V2',
                    'finger_positions':q_now[fingers],'actual_opening_m':float(np.sum(q_now[fingers]))})
'''+sample_hook
assert text.count(sample_hook)==1;text=text.replace(sample_hook,telemetry)

# scipy Rotation is already imported by unseen_scene but not in the generated global scope.
exec(compile('import os\nfrom scipy.spatial.transform import Rotation\n'+text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))
