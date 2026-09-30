"""Watermarked visualization of saved planned IK waypoints, never physics execution."""
import hashlib
import json
import math
import shutil
import subprocess
import numpy as np


def render_preview(report_path,video_path,world,robot,door,views,q,arm,fingers,base_pose,args):
    import cv2
    report=json.loads(report_path.read_text())
    if not np.allclose(report['robot_base_pose'],base_pose,atol=1e-6):
        raise ValueError('preview base differs from planning base')
    placement=report['asset_installation']
    if not np.allclose([placement[k] for k in ('x_m','y_m','yaw_deg')],
                       [args.asset_x,args.asset_y,args.asset_yaw_deg],atol=1e-6):
        raise ValueError('preview asset placement differs from planning placement')
    selected=max(report['candidate_results'],key=lambda r:sum(a['status']=='PLANNED' for a in r.get('arc',[])))
    arc=[]
    for row in selected.get('arc',[]):
        if row['status']!='PLANNED':break
        if row.get('all_joint_min_margin_rad',0.)<=.05:
            raise ValueError('preview waypoint has insufficient all-joint margin')
        arc.append(row)
    if not arc:raise ValueError('no planned prefix to visualize')
    angles=np.array([selected['arc_start_deg'],*[r['door_angle_deg'] for r in arc]])
    qs=np.array([selected['grasp_joint_q'],*[r['q'] for r in arc]])
    from isaacsim.core.utils.types import ArticulationAction
    initial=q.copy();initial[arm]=qs[0];initial[fingers]=0.
    robot.apply_action(ArticulationAction(joint_positions=initial))
    # Flush camera/Fabric history before frame zero; otherwise the first
    # camera buffers still show the parked home pose from scene initialization.
    for _ in range(32):
        robot.set_joint_positions(initial);robot.set_joint_velocities(np.zeros_like(initial))
        door.set_joint_positions(np.array([math.radians(angles[0])]))
        door.set_joint_velocities(np.zeros(1));world.step(render=True)
    video_path.parent.mkdir(parents=True,exist_ok=True)
    fps=24;duration=9.;frames=int(duration*fps)
    ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg:raise RuntimeError('ffmpeg unavailable')
    encoder=subprocess.Popen([ffmpeg,'-hide_banner','-loglevel','error','-y',
        '-f','rawvideo','-pixel_format','rgb24','-video_size','1920x720',
        '-framerate',str(fps),'-i','-','-an','-c:v','libx264','-crf','18',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(video_path)],stdin=subprocess.PIPE)
    try:
        # This isolated renderer prescribes both articulations every frame.
        # A short world step synchronizes PhysX/Fabric transforms to RTX;
        # these forced states cannot count as a contact/physics experiment.
        for index in range(frames):
            time=index/fps
            angle=float(angles[0]+np.clip((time-1.)/6.,0.,1.)*(angles[-1]-angles[0]))
            qr=np.array([np.interp(angle,angles,qs[:,j]) for j in range(7)])
            full=q.copy();full[arm]=qr;full[fingers]=0.
            robot.apply_action(ArticulationAction(joint_positions=full))
            for _ in range(2):
                robot.set_joint_positions(full);robot.set_joint_velocities(np.zeros_like(full))
                door.set_joint_positions(np.array([math.radians(angle)]))
                door.set_joint_velocities(np.zeros(1))
                world.step(render=True)
            panels=[cv2.resize(np.asarray(v.get_rgba())[:,:,:3],(960,720)) for v in views]
            frame=np.ascontiguousarray(np.hstack(panels),dtype=np.uint8)
            cv2.rectangle(frame,(0,0),(1920,100),(15,15,15),-1)
            cv2.putText(frame,'PLANNED IK WAYPOINT PREVIEW - NOT PHYSICAL EXECUTION',
                (25,35),cv2.FONT_HERSHEY_SIMPLEX,.85,(255,220,80),2,cv2.LINE_AA)
            cv2.putText(frame,f'Door: {angle:.1f} deg / {angles[-1]:.1f} deg | Table-edge mount: 10 cm outside, 10 cm below',
                (25,75),cv2.FONT_HERSHEY_SIMPLEX,.8,(245,245,245),2,cv2.LINE_AA)
            cv2.rectangle(frame,(0,678),(1920,720),(15,15,15),-1)
            cv2.putText(frame,'Waypoint interpolation; gripper pose illustrative. Contact and physical opening remain unverified.',
                (25,705),cv2.FONT_HERSHEY_SIMPLEX,.7,(245,245,245),1,cv2.LINE_AA)
            encoder.stdin.write(frame.tobytes())
            if index in (0,frames-1):
                cv2.imwrite(str(video_path.with_name(f'{video_path.stem}_{index}.jpg')),cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
        encoder.stdin.close()
        if encoder.wait(timeout=60)!=0:raise RuntimeError('preview encoding failed')
    finally:
        if encoder.poll() is None:encoder.kill();encoder.wait()
    video_path.with_suffix('.json').write_text(json.dumps({
        'kind':'planned IK waypoint visualization; not physical execution',
        'source_report':str(report_path),'source_report_sha256':hashlib.sha256(report_path.read_bytes()).hexdigest(),
        'raw_rank':selected['raw_rank'],'variant':selected['variant'],
        'planned_prefix_end_deg':float(angles[-1]),'planned_waypoints':len(arc),
        'min_all_joint_margin_planned_rad':min(r['all_joint_min_margin_rad'] for r in arc),
        'interpolation':'linear joint waypoint interpolation; not the exact stored OMPL trajectory',
        'physics_execution':False,'scripted_door_visualization':True,
        'render_sync':'two prescribed-state simulation steps per frame; not free physical opening',
        'contact_verified':False,'success_claim':False,'frames':frames,'fps':fps,
        'robot_base_pose':base_pose.tolist()},indent=2))
