"""Unknown-joint physical point-wrench probe with an ideal TCP attachment.

No real Dex1 grasp. The massless ideal TCP transfers force to the moving link;
the arm follows the observed link pose. No object joint target is issued.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from interaction_identification.fitting import fit_articulation,evaluate


def matrix(position,quat_wxyz):
    T=np.eye(4);T[:3,3]=position;T[:3,:3]=Rotation.from_quat(np.roll(quat_wxyz,-1)).as_matrix();return T


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',required=True,type=Path)
    p.add_argument('--initial-state',required=True,type=Path)
    p.add_argument('--asset-root',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--gpu',type=int,default=6)
    p.add_argument('--force-n',type=float,default=.05)
    p.add_argument('--duration-s',type=float,default=6.5)
    p.add_argument('--motion-cap-m',type=float,default=.025)
    p.add_argument('--deadline-shanghai',help='ISO timestamp; defaults to the next 05:00 Shanghai time')
    a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    zone=ZoneInfo('Asia/Shanghai');now=datetime.now(zone)
    deadline=datetime.fromisoformat(a.deadline_shanghai) if a.deadline_shanghai else now.replace(hour=5,minute=0,second=0,microsecond=0)
    if deadline.tzinfo is None:deadline=deadline.replace(tzinfo=zone)
    if not a.deadline_shanghai and deadline<=now:deadline+=timedelta(days=1)
    if now>=deadline:raise RuntimeError('experiment deadline already passed')
    source=json.loads((a.source/'report.json').read_text());initial=json.loads(a.initial_state.read_text())
    state=initial['isolation']['isolated_grasp']['actual_closure_samples'][0]
    home=[state['robot_q'][state['names'].index(f'J{i}')] for i in range(1,8)]
    placement=source['asset_installation'];os.environ['R1A7_BASE_POSE']=','.join(map(str,source['robot_base_pose']))
    os.environ['R1A7_SUPPORT_BOTTOM_Z']=str(source['support_bottom_z_m'])
    os.environ['R1A7_PEDESTAL_SIZE']='0.10,0.10,0.20';os.environ['R1A7_RUN_DIR']=str(out)
    os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y'
    # Reuse the trusted scene bootstrap exactly, stopping before HTTP/control.
    # No existing grasp/contact/planning source is edited or executed here.
    legacy=ROOT/'src/r1a7_articulated_sim_server.py';text=legacy.read_text();marker='commands = queue.Queue(); state = {}; lock = threading.Lock()'
    if text.count(marker)!=1:raise RuntimeError('scene bootstrap marker changed; inspect before running')
    bootstrap=text.partition(marker)[0]
    old_argv=sys.argv;sys.argv=[str(legacy),'--gpu',str(a.gpu),'--asset-root',str(a.asset_root),
        '--asset-x',str(placement['x_m']),'--asset-y',str(placement['y_m']),
        '--asset-yaw-deg',str(placement['yaw_deg']),'--fixture-height-m',str(placement['fixture_height_m']),
        '--home-q',*[str(q) for q in home],'--record-overview',
        '--camera-offset',*[str(q) for q in source['camera_offset_m']]]
    scene={'__name__':'act2see_scene_bootstrap','__file__':str(legacy)}
    exec(compile(bootstrap,str(legacy),'exec'),scene);sys.argv=old_argv
    app=scene['app'];world=scene['world'];robot=scene['robot'];tcp=scene['tcp'];link=scene['door_link'];dt=scene['DT'];names=scene['names'];arm=scene['arm'];fingers=scene['fingers'];controller=scene['controller']
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import UsdGeom,Gf
    body=RigidPrim(prim_paths_expr=scene['contact_target_path'],name='act2see_external_wrench');body.initialize()
    import xml.etree.ElementTree as ET
    model=ET.parse(ROOT/'config/r1a7_dex1.urdf').getroot();limits=np.array([[float(model.find(f"joint[@name='J{i}']/limit").get(k)) for k in ('lower','upper')] for i in range(1,8)])
    for _ in range(24):world.step(render=True)
    T0=matrix(*tcp.get_world_pose());L0=matrix(*link.get_world_pose());attachment=np.linalg.inv(L0)@T0
    # Force direction depends only on the observed initial grasp approach.
    direction=-T0[:3,2];direction/=np.linalg.norm(direction)
    view=robot._articulation_view;index=view.get_body_index('dex1_base_link')-1
    from isaacsim.core.prims import SingleXFormPrim
    wrist=SingleXFormPrim(scene['one_prim']('dex1_base_link','/World/R1A7'))
    mark=UsdGeom.Sphere.Define(scene['stage'],'/World/act2see_ideal_tcp');mark.CreateRadiusAttr(.004);mark.CreateDisplayColorAttr([(.95,.65,.1)])
    marker_pose=SingleXFormPrim('/World/act2see_ideal_tcp')
    qcmd=np.asarray(robot.get_joint_positions()).copy();qcmd[fingers]=-.02
    rows=[];previous=T0.copy();v_filtered=np.zeros(3);reason='PROBE_COMPLETE';record=None
    import subprocess,cv2,shutil
    camera=scene['overview'][0];video=out/'probe.mp4'
    record=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-',
                             '-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(video)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(out/'ffmpeg.log','w'))
    metadata={'kind':'ideal kinematic grasp; physical unknown-joint point-force probe; no real contact success',
              'robot_joint_names':list(names),'physics_dt_s':dt,
              'base_pose':source['robot_base_pose'],'scene_source':str(a.source),'initial_state':str(a.initial_state),
              'bootstrap_sha256':hashlib.sha256(text.encode()).hexdigest(),'force_direction_world':direction.tolist(),
              'force_n':a.force_n,'duration_s':a.duration_s,'motion_cap_m':a.motion_cap_m,
              'deadline_shanghai':deadline.astimezone(zone).isoformat(),
              'gt_used_for_control':False,'gt_used_for_fit':False,'physical_parameters_changed':False,
              'attachment':'massless ideal TCP rigidly attached to observed moving-link frame; force forwarded to that body; finite-tracking R1 follower',
              'robot_drive_gains_unchanged':True,'object_joint_targets_issued':False,'measured_wrench_tcp':None}
    try:
        for step in range(int(a.duration_s/dt)):
            wall=datetime.now(ZoneInfo('Asia/Shanghai'))
            if wall>=deadline:reason='CUTOFF_05_00';break
            t=step*dt;L=matrix(*link.get_world_pose());desired=L@attachment;actual=matrix(*tcp.get_world_pose())
            velocity=(desired[:3,3]-previous[:3,3])/dt;v_filtered=.8*v_filtered+.2*velocity;previous=desired.copy()
            sign=0. if t<1. or t>=4. else (1. if t<2.5 else -1.)
            F=sign*a.force_n*direction-3.*v_filtered
            norm=np.linalg.norm(F)
            if norm>a.force_n:F*=a.force_n/norm
            if np.linalg.norm(desired[:3,3]-T0[:3,3])>a.motion_cap_m:reason='SMALL_MOTION_CAP';break
            q=np.asarray(robot.get_joint_positions());margin=float(np.min(np.minimum(q[arm]-limits[:,0],limits[:,1]-q[arm])))
            if margin<=.05:reason='LOW_JOINT_MARGIN';break
            J=np.asarray(view.get_jacobians())[0,index,:,:][:,arm].copy()
            wp=np.asarray(wrist.get_world_pose()[0]);offset=actual[:3,3]-wp
            J[:3]+=np.cross(J[3:].T,offset).T
            error=np.r_[desired[:3,3]-actual[:3,3],Rotation.from_matrix(desired[:3,:3]@actual[:3,:3].T).as_rotvec()]
            # Feedback follows observed motion, never an asset-derived arc.
            delta=J.T@np.linalg.solve(J@J.T+1e-5*np.eye(6),error)
            qcmd[arm]=np.clip(q[arm]+np.clip(delta,-.015,.015),limits[:,0]+.05,limits[:,1]-.05)
            controller.apply_action(ArticulationAction(joint_positions=qcmd))
            gravity=np.asarray(view.get_generalized_gravity_forces())[0]
            robot.set_joint_efforts(gravity)
            body.apply_forces_and_torques_at_pos(forces=F[None,:],positions=desired[None,:3,3],is_global=True)
            marker_pose.set_world_pose(desired[:3,3],np.roll(Rotation.from_matrix(desired[:3,:3]).as_quat(),1))
            rows.append({'t':t,'T_ee_ideal':desired.tolist(),'T_ee_robot':actual.tolist(),'robot_q':q.tolist(),
                         'T_moving_link':L.tolist(),'applied_wrench_tcp_world':np.r_[F,np.zeros(3)].tolist(),
                         'measured_wrench_tcp_world':None,'joint_margin_rad':margin,'tracking_error_m':float(np.linalg.norm(error[:3]))})
            world.step(render=step%8==0)
            if step%8==0:
                image=np.asarray(camera.get_rgba())[:,:,:3].copy()
                if image.shape!=(960,1280,3):image=cv2.resize(image,(1280,960))
                cv2.rectangle(image,(0,0),(1280,88),(15,15,15),-1)
                cv2.putText(image,'IDEAL ATTACHMENT | UNKNOWN JOINT FORCE PROBE | NOT REAL DEX1 GRASP',(15,30),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
                cv2.putText(image,f't={t:.2f}s  motion={np.linalg.norm(desired[:3,3]-T0[:3,3])*1000:.1f}mm  tracking={np.linalg.norm(error[:3])*1000:.1f}mm',(15,63),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
                record.stdin.write(np.ascontiguousarray(image).tobytes())
        metadata['stop_reason']=reason;metadata['sample_count']=len(rows)
        (out/'observations.json').write_text(json.dumps({'metadata':metadata,'samples':rows}))
        # Fit is completed BEFORE accessing the hidden simulator joint geometry.
        fit=fit_articulation([r['T_ee_ideal'] for r in rows]);robot_fit=fit_articulation([r['T_ee_robot'] for r in rows])
        (out/'estimated_articulation.json').write_text(json.dumps({'ideal_observation_fit':fit,'robot_ee_fit':robot_fit},indent=2))
        joint=scene['asset_chain'].joints[scene['moving_link']]
        B=np.eye(4);B[:3,:3]=scene['asset_rotation'];B[:3,3]=scene['asset_xyz']
        H=B@scene['asset_chain'].root_to_link(joint.parent,{})@joint.origin
        gt={'joint_type':joint.kind,'axis_world':(H[:3,:3]@joint.axis).tolist(),'origin_world':H[:3,3].tolist()}
        evaluation=evaluate(fit,gt,np.array(rows[0]['T_ee_ideal'])[:3,3],[r['T_ee_ideal'] for r in rows])
        robot_evaluation=evaluate(robot_fit,gt,np.array(rows[0]['T_ee_robot'])[:3,3],[r['T_ee_robot'] for r in rows])
        metadata['min_observed_joint_margin_rad']=min(r['joint_margin_rad'] for r in rows)
        metadata['zero_input_baseline_motion_m']=max(np.linalg.norm(np.array(r['T_ee_ideal'])[:3,3]-T0[:3,3]) for r in rows if r['t']<1.)
        metadata['max_tracking_error_m']=max(r['tracking_error_m'] for r in rows)
        metadata['max_observed_displacement_m']=max(np.linalg.norm(np.array(r['T_ee_ideal'])[:3,3]-T0[:3,3]) for r in rows)
        result={'metadata':metadata,'estimate':fit,'robot_ee_estimate':robot_fit,'ground_truth_evaluation_only':gt,'evaluation':evaluation,'robot_ee_evaluation':robot_evaluation}
        (out/'report.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
    except BaseException as error:
        import traceback
        traceback.print_exc()
        (out/'failure.json').write_text(json.dumps({'error':repr(error),'sample_count':len(rows)},indent=2))
        (out/'observations.json').write_text(json.dumps({'metadata':metadata,'samples':rows}))
        raise
    finally:
        if record:record.stdin.close();record.wait(timeout=30)
        app.close()
if __name__=='__main__':main()
