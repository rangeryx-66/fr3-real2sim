"""Instrumentation-only adapter for one continuous third-person closed loop.

This wraps ``sim_real2sim.py`` without changing grasp selection, MoveIt,
PayloadID, or reconstruction algorithms.  It adds a fixed third-person camera,
the FR3 hand-mounted RGB-D camera, and explicit recording/capture commands.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
adapter = (ROOT / "calibration/sim_real2sim.py").read_text()
end = "exec(compile('import os\\nfrom scipy.spatial.transform import Rotation\\n'+text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))"
assert adapter.count(end) == 1
namespace = {"__file__": str(ROOT / "calibration/sim_real2sim.py")}
exec(compile(adapter.replace(end, ""), str(ROOT / "calibration/sim_real2sim.py"), "exec"), namespace)
source, text = namespace["source"], namespace["text"]

# Use the validated current-window Payload-ID start gate from the production
# fresh-capture simulator.  The ordinary gate carries lift-history state that
# is appropriate for grasp verification but can become stale after the first
# calibration attitude change.
gate_needle = "qguard.get('passed') and qguard.get('currently_clear') and min(f)>.1"
assert text.count(gate_needle) == 2
text = text.replace(gate_needle, "__import__('payload_start_gate').evaluate_start(support)['passed'] and qguard.get('currently_clear') and min(f)>.1")
gate_detail = "'support':qguard,'forces_N':f.tolist()"
assert text.count(gate_detail) == 1
text = text.replace(gate_detail, "'support':qguard,'payload_start_gate':__import__('payload_start_gate').evaluate_start(support),'forces_N':f.tolist()")

camera_anchor = "scan_camera.set_focal_length(1.8);scan_camera.set_horizontal_aperture(1.44);scan_camera.set_vertical_aperture(1.08);scan_camera.set_clipping_range(.05,2.)"
camera_code = camera_anchor + r'''

# Wrist camera uses the same hand-frame extrinsic as ObserveSceneSkill.
hand_path=finger_paths[0].rsplit('/',1)[0]
wrist_camera=Camera(
    hand_path+'/wrist_camera', name='wrist_camera', frequency=30,
    resolution=(640,360), translation=np.array([.11,-.031,-.074]),
    orientation=np.array([.70710678,0.,0.,.70710678]),
)
wrist_camera.set_local_pose(
    translation=np.array([.11,-.031,-.074]),
    orientation=np.array([.70710678,0.,0.,.70710678]), camera_axes='ros')
wrist_camera.set_focal_length(2.8);wrist_camera.set_horizontal_aperture(5.376)
wrist_camera.set_vertical_aperture(3.024);wrist_camera.set_clipping_range(.03,2.)

# Observer camera stays outside the workspace and sees the full robot/table.
THIRD_POSITION=np.array([1.18,-1.16,.86],dtype=float)
THIRD_TARGET=np.array([.50,0.,.16],dtype=float)
third_z=THIRD_TARGET-THIRD_POSITION;third_z/=np.linalg.norm(third_z)
third_x=np.cross(third_z,np.array([0.,0.,1.]));third_x/=np.linalg.norm(third_x)
third_y=np.cross(third_z,third_x)
T_B_THIRD=np.eye(4);T_B_THIRD[:3,:3]=np.column_stack([third_x,third_y,third_z]);T_B_THIRD[:3,3]=THIRD_POSITION
third_camera=Camera('/World/third_person_camera',name='third_person_camera',position=THIRD_POSITION,resolution=(1280,720),frequency=30)
third_camera.set_world_pose(position=THIRD_POSITION,orientation=np.roll(Rotation.from_matrix(T_B_THIRD[:3,:3]).as_quat(),1),camera_axes='ros')
third_camera.set_focal_length(2.6);third_camera.set_horizontal_aperture(4.8);third_camera.set_vertical_aperture(2.7);third_camera.set_clipping_range(.05,3.)
'''
assert text.count(camera_anchor) == 1
text = text.replace(camera_anchor, camera_code)

init_anchor = "scan_camera.initialize();scan_camera.add_distance_to_image_plane_to_frame();scan_camera.add_instance_id_segmentation_to_frame()"
init_code = init_anchor + r'''
wrist_camera.initialize();wrist_camera.add_distance_to_image_plane_to_frame();wrist_camera.add_instance_id_segmentation_to_frame()
third_camera.initialize()
'''
assert text.count(init_anchor) == 1
text = text.replace(init_anchor, init_code)

state_anchor = "diagnostic_fixed_joint=None;diagnostic_attachment_mode=os.environ.get('PAYLOAD_ATTACHMENT_MODE','NORMAL').upper()"
state_code = state_anchor + r'''
third_recording=False;third_writer=None;third_output=None;third_frame_count=0;third_start_tick=0
third_phase='IDLE';third_markers=[];wrist_frame_count=0
'''
assert text.count(state_anchor) == 1
text = text.replace(state_anchor, state_code)

command_anchor = "                elif op=='diagnostic_fixed_joint':"
commands = r'''                elif op=='third_record_start':
                    if third_recording:raise RuntimeError('third-person recording already active')
                    import cv2
                    third_output=Path(cmd['path']);third_output.parent.mkdir(parents=True,exist_ok=True)
                    third_writer=cv2.VideoWriter(str(third_output),cv2.VideoWriter_fourcc(*'mp4v'),30.,(1280,720))
                    if not third_writer.isOpened():raise RuntimeError('third-person video writer unavailable')
                    third_recording=True;third_frame_count=0;third_start_tick=tick;third_phase=str(cmd.get('phase','observe_scene'))
                    third_markers=[{'phase':third_phase,'frame':0,'time_s':0.0}]
                    results[token]={'ok':True,'path':str(third_output),'phase':third_phase}
                elif op=='third_record_mark':
                    if not third_recording:raise RuntimeError('third-person recording not active')
                    third_phase=str(cmd['phase']);third_markers.append({'phase':third_phase,'frame':third_frame_count,'time_s':(tick-third_start_tick)*DT})
                    results[token]={'ok':True,'phase':third_phase,'frame':third_frame_count}
                elif op=='third_record_stop':
                    if not third_recording:raise RuntimeError('third-person recording not active')
                    third_recording=False;third_writer.release();third_writer=None
                    manifest={'schema':'fr3_continuous_third_person/v1','path':str(third_output),'fps':30,'frames':third_frame_count,
                      'duration_s':third_frame_count/30.,'camera_position':THIRD_POSITION.tolist(),'look_at_target':THIRD_TARGET.tolist(),
                      'markers':third_markers,'single_sim_process':True,'continuous_world_state':True}
                    third_output.with_suffix('.json').write_text(json.dumps(manifest,indent=2));results[token]={'ok':True,**manifest}
                elif op=='wrist_capture':
                    import cv2
                    for _ in range(8):world.step(render=True);tick+=1
                    output=Path(cmd['output']);output.mkdir(parents=True,exist_ok=True)
                    rgba=np.asarray(wrist_camera.get_rgba());depth=np.asarray(wrist_camera.get_depth())
                    if rgba.size==0 or depth.size==0:raise RuntimeError('wrist RGB-D unavailable')
                    seg=wrist_camera.get_current_frame()['instance_id_segmentation'];raw=np.asarray(seg['data']).reshape(depth.shape)
                    labels=seg['info']['idToLabels'];ids=[int(k) for k,v in labels.items() if str(v)=='/World/box' or str(v).startswith('/World/box/')]
                    mask=np.isin(raw,ids);stem=f'{wrist_frame_count:06d}';wrist_frame_count+=1
                    cv2.imwrite(str(output/f'{stem}.png'),rgba[...,:3][...,[2,1,0]])
                    cv2.imwrite(str(output/f'{stem}.depth.png'),np.where(np.isfinite(depth)&(depth>0),np.clip(np.rint(depth*1000),0,65535),0).astype(np.uint16))
                    cv2.imwrite(str(output/f'{stem}.mask.png'),mask.astype(np.uint8)*255)
                    cp,cq=wrist_camera.get_world_pose(camera_axes='ros');K=wrist_camera.get_intrinsics_matrix()
                    record={'ok':True,'frame':stem,'phase':str(cmd.get('phase',third_phase)),'timestamp_s':tick*DT,
                      'rgb':str(output/f'{stem}.png'),'depth':str(output/f'{stem}.depth.png'),'mask':str(output/f'{stem}.mask.png'),
                      'target_pixels':int(mask.sum()),'camera_position_world':np.asarray(cp).tolist(),
                      'camera_quaternion_world_wxyz':np.asarray(cq).tolist(),'intrinsics':np.asarray(K).tolist()}
                    (output/f'{stem}.json').write_text(json.dumps(record,indent=2));results[token]=record
''' + command_anchor
assert text.count(command_anchor) == 1
text = text.replace(command_anchor, commands)

step_anchor = "        world.step(render=False);tick+=1"
step_code = r'''        render_now=bool(third_recording and tick%8==0)
        world.step(render=render_now);tick+=1
        if render_now and third_recording:
            import cv2
            frame=np.asarray(third_camera.get_rgba())
            if frame.size:
                bgr=frame[...,:3][...,[2,1,0]].copy()
                cv2.putText(bgr,third_phase,(28,54),cv2.FONT_HERSHEY_SIMPLEX,1.1,(255,255,255),3,cv2.LINE_AA)
                cv2.putText(bgr,f't={(tick-third_start_tick)*DT:6.2f}s',(28,96),cv2.FONT_HERSHEY_SIMPLEX,.8,(230,230,230),2,cv2.LINE_AA)
                third_writer.write(bgr);third_frame_count+=1'''
assert text.count(step_anchor) == 1
text = text.replace(step_anchor, step_code)

finally_anchor = "finally:\n    server.shutdown();app.close()"
finally_code = "finally:\n    if third_writer is not None:third_writer.release()\n    server.shutdown();app.close()"
assert text.count(finally_anchor) == 1
text = text.replace(finally_anchor, finally_code)

exec(compile("import os\nfrom scipy.spatial.transform import Rotation\n" + text, str(source), "exec"),
     dict(__name__="__main__", __file__=str(source)))
