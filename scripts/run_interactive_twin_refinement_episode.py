"""Independent bounded refinement: identical physical-contact baseline.

Door drives/forces/attachments are never commanded during execution.
"""
from piper_mobile_execute import *

def parser():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--job',type=Path,required=True);return p

def main():
 import time
 from types import SimpleNamespace
 from interactive_twin.plant import bootstrap_job
 from interactive_twin.execution import save_observable_logs
 from interactive_twin_refinement.protocol import PhysicsProtocol
 job=json.loads(parser().parse_args().job.read_text())
 from interactive_twin_refinement.assembly import install_setup_capture
 install_setup_capture()
 if job.get('mobile_platform'):
  from interactive_twin_recovery.native import install
  install()
  from interactive_twin.plant import bootstrap_job
 a=SimpleNamespace(**{k:Path(job[k]) for k in ('source','asset_root','plan','output')},gpu=job.get('gpu',1),candidate=job.get('candidate',0),ownership=None,no_operation=False,deadline_shanghai=job['deadline_shanghai'])
 a.output.mkdir(parents=True,exist_ok=True)
 experiment_files=sorted((ROOT/'interactive_twin').glob('*.py'))+[Path(__file__).resolve()]
 (a.output/'experiment_code_hashes.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in experiment_files},indent=2))
 if datetime.now(ZoneInfo('Asia/Shanghai'))>=datetime.fromisoformat(a.deadline_shanghai):
  (a.output/'report.json').write_text(json.dumps({'status':'CUTOFF_05_00','success':False}));return
 wall_start=time.monotonic();wall_budget=job.get('wall_clock_budget_s',1200)
 tape=json.loads(Path(job['replay_commands']).read_text()) if job.get('replay_commands') else None
 replay_safety=None
 if tape is not None and any(f.get('cartesian_input') is not None and f['phase'] in ('P1','P2','P3','P4') for f in tape):
  from interactive_twin.replay import saved_safety_model
  replay_safety=saved_safety_model(job.get('reference_safety_memory',Path(job['replay_commands']).parent/'structured_memory.json'))
 protocol=None;protocol_completed=False;issued=[];replay_last_compliant=None;calibration_comp_fraction=1.
 expected_hashes={'scripts/run_piper_contact_baseline.py': '3abbd9c4d2d3ad0c7206ec6f5d225bdbd8f010b9c7e710993be8231d9f058476', 'articulated_interaction/physical_baseline.py': '9993be2ce0ff3cd697bb1922b72a3bbfc203d9d0478aa90a624df5aea5bf2eb6', 'articulated_interaction/control.py': '2643be70fea569f4e95621bdbd6ea63aa79dcd97f2e69f9734917356fcb30b13', 'scripts/prepare_piper_description.py': '65dcd081aa6eecd43083eb185221b87d47c14f5f65a2af4989c60fa90ad3d66c', 'config/semantic_interaction.json': '2163f398eaf9e42c3e167f11d1b17f41849037089a7a4a91a3b425aac59259c8', 'piper_mobile_demo/model.py': '92a8f6ec04a0741a0469f44161961cc424583cc347a20ab489f1d541ad472d3d'}
 expected_hashes.update({'interaction_identification/act2see_loop.py': '7ad9091d6f5ce1315f5bade70bf57d6402d375abaa1354c2ef6a2d3c4cd4166f', 'interaction_identification/fitting.py': 'f945972d037f0786cb092a37a3f6d024af8c1872c335481d159f0606f9101c09', 'interaction_identification/contact_probe.py': '58588a536ab8dab7a918c99ea26ce56a2236112f3ae391c5f938274ed1bd1089'})
 frozen_hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in expected_hashes}
 if frozen_hashes!=expected_hashes:raise RuntimeError('FROZEN_BASELINE_CHANGED')
 (a.output/'frozen_baseline.json').write_text(json.dumps(frozen_hashes,indent=2))
 source=json.loads((a.source/'report.json').read_text());base=source['robot_base_pose'];plan=json.loads(a.plan.read_text()) if a.plan.exists() else {};chosen=plan['trial_candidates'][a.candidate] if plan else {'base':base,'T':np.eye(4).tolist()}
 if base!=chosen['base']:raise RuntimeError('BASE_CHANGED')
 a.ownership=None;scene=bootstrap_job(a,base,job);world=scene['world'];app=scene['app'];robot=scene['robot'];stage=scene['stage'];arm=scene['arm'];fingers=scene['fingers'];names=scene['names'];controller=scene['controller'];dt=scene['DT']
 (a.output/'native_robot_dofs.json').write_text(json.dumps({'joint_names':list(names),'arm_indices':[int(x) for x in arm],'finger_indices':[int(x) for x in fingers]},indent=2))
 from isaacsim.core.utils.types import ArticulationAction
 from isaacsim.sensors.camera import Camera
 from interaction_identification.contact_probe import robot_only_model,GroundTruthGate
 from interaction_identification.act2see_loop import ConstrainedDrive,InteractionMemory,TactileRetention
 from interaction_identification.fitting import fit_articulation,evaluate
 from piper_mobile_demo.cooked_geometry import export_cooked
 from articulated_interaction.physical_baseline import PhysicalScene,WholeFingerReports,FINGERS
 from articulated_interaction.control import JawCenteredClosure
 from articulated_demo.kinematics import transform
 import cv2
 from interactive_twin_refinement.assembly import capture_assembly
 capture_assembly(scene,a.output/'native_assembly_private.json')
 (a.output/'setup_warmup_evaluation_private.json').write_text(json.dumps({'setup_only':True,'controller_access':False,'joint_angles_rad':scene.get('startup_angles',[])},indent=2))
 if job.get('mode')=='assembly_audit':
  world.pause();(a.output/'report.json').write_text(json.dumps({'status':'ASSEMBLY_AUDIT_COMPLETE','success':False,'interaction_executed':False,'no_physics_or_geometry_modified':True},indent=2));app.close();return
 gt_gate=GroundTruthGate();gt_gate.protect(scene)
 deadline=datetime.fromisoformat(a.deadline_shanghai)
 model=robot_only_model(ROOT/'config/piper.urdf');model.deadline_timestamp=deadline.timestamp()
 export_cooked(stage,a.output/'cooked_initial.json');export=json.loads((a.output/'cooked_initial.json').read_text())
 allowed=[e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]
 if len(allowed)!=3:raise RuntimeError('FROZEN_PROXY_ASSOCIATION_FAILED')
 collision=PhysicalScene(export,model,allowed)
 if job.get('mode')=='plan':
  from interactive_twin.planning import plan_grasps
  visual=json.loads(Path(job['initial_visual']).read_text())
  result=plan_grasps(model,collision,collision.moving_reference,visual,base,seed=job.get('seed',732061660),budget=12,output=a.output/'plan.json',wall_clock_s=job.get('planning_budget_s',600))
  (a.output/'report.json').write_text(json.dumps({'status':'PLAN_COMPLETE' if result['trial_candidates'] else 'NO_LEGAL_APPROACH','episode_id':job['episode_id'],'success':False,'candidate_count':len(result['trial_candidates'])},indent=2))
  world.pause();world.render()
  for i,view in enumerate(scene['overview']):
   rgb=np.asarray(view.get_rgba())[:,:,:3]
   cv2.imwrite(str(a.output/f'preflight_view_{i}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
  app.close();return
 T=np.asarray(chosen['T']);slide=depth=roll=0.;normal=-T[:3,2];anchor_world=T[:3,3].copy()
 moving_initial=collision.moving_reference
 anchor_link=(np.linalg.inv(moving_initial)@np.r_[anchor_world,1])[:3]
 fingerprint={'manifest_sha256':hashlib.sha256((a.asset_root/'manifest.json').read_bytes()).hexdigest(),'geometry':'unchanged frozen semantic proxy','baseline_commit':'f5dcc6f'}
 if fingerprint['manifest_sha256']!=job['frozen_proxy_sha256']:raise RuntimeError('FROZEN_PROXY_CHANGED')
 (a.output/'frozen_proxy.json').write_text(json.dumps(fingerprint,indent=2))
 actual_views=dict(scene['scene_monitor_views']);B=transform(base[:3],[0,0,np.deg2rad(base[3])]);chain=model.chain
 positions=dict(zip(names,robot.get_joint_positions()));offset=np.linalg.inv(chain.root_to_link('gripper_base',positions))@chain.root_to_link('tcp_link',positions)
 probe_start=0;probe_start_time=0.;probe=None;memory=None;drive=None;fit=None;fit_poses=[];fit_saved=False;following=False;probe_metrics={};evaluation={};grasp_tcp=None;compliant=False;torque_diagnostics={};retention=TactileRetention(FINGERS)
 def moving():
  # Collision prediction assumes retained grasp; native actual contacts remain authoritative.
  return moving_initial if grasp_tcp is None else tcp()@np.linalg.inv(grasp_tcp)@moving_initial
 def tcp():
  p,q=actual_views['gripper_base'].get_world_poses();return matrix(p[0],q[0])@offset
 def poses(q):return model.poses(np.asarray(q)[arm],base,finger_q=np.asarray(q)[fingers])
 qtarget=np.asarray(robot.get_joint_positions(),dtype=float).copy();qtarget[fingers]=[.05,-.05];qvelocity=np.zeros(6);phase='SETTLE';rows=[];tick=0;reference=None;legal=False;success=False;pull2=False;status='STARTED';loss_s=0.;slip_s=0.;preflight={}
 urdf=ET.parse(ROOT/'config/piper.urdf').getroot();vel=np.array([float(urdf.find(f"joint[@name='joint{i}']/limit").get('velocity')) for i in range(1,7)]);limits=model.limits
 effort_limits=np.array([float(urdf.find(f"joint[@name='joint{i}']/limit").get('effort')) for i in range(1,7)]);policy=json.loads((ROOT/'config/semantic_interaction.json').read_text());policy['slow_closure_m_s']=.001;mode='position';effort=0.
 near=scene['overview'][1];eye=anchor_world+normal*.45+np.array([0,0,.20]);focus=anchor_world+np.array([0,0,.02]);forward=(focus-eye)/np.linalg.norm(focus-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward));near.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros');near.set_clipping_range(.02,3.);near.add_distance_to_image_plane_to_frame()
 video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(a.output/'contact_baseline.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
 def sample():
  q=np.asarray(robot.get_joint_positions());P=poses(q);E=tcp()
  return {'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':model.margin(q[arm]),'finger_world_poses':{n:P[n].tolist() for n in FINGERS},'T_tcp':E.tolist(),'T_predicted_collision_body':moving().tolist(),'relative_translation_slip_m':retention.drift_m,'estimated_angle_deg':0. if memory is None else memory.angle_deg(E),'articulation_consistency_error_m':None if memory is None else memory.consistency_error(E)}
 native=WholeFingerReports(stage,export,allowed,world,dt,sample)
 def window(seconds=.3):return rows[-max(1,int(seconds/dt)):]
 def filtered():return np.mean([list(s['forces_n'].values()) for s in window(.08)],axis=0) if rows else np.zeros(2)
 def grip_window():
  w=window(.5);loads=np.array([list(s['forces_n'].values()) for s in w]);opening=np.array([s['aperture_m'] for s in w]);q=np.asarray(robot.get_joint_positions());P=poses(q);centers=np.array([(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]);center=(moving()@np.r_[anchor_link,1])[:3];axis_jaw=centers[0]-centers[1];axis_jaw/=np.linalg.norm(axis_jaw);interval=np.sort(centers@axis_jaw);between=interval[0]<=center@axis_jaw<=interval[1]
  good=len(w)>=int(.45/dt) and between and np.all(np.mean(loads>.05,axis=0)>=.6) and np.all(np.mean(loads,axis=0)>=.1) and np.ptp(opening)<.0005
  return good,{'between_jaws':bool(between),'force_duty':np.mean(loads>.05,axis=0).tolist(),'mean_forces_n':np.mean(loads,axis=0).tolist(),'aperture_range_m':float(np.ptp(opening))}
 def step():
  nonlocal tick,loss_s,slip_s,torque_diagnostics,phase,mode,effort,compliant,reference,grasp_tcp,replay_last_compliant,drive,legal,memory,fit_poses,probe_start,probe_start_time
  if datetime.now(ZoneInfo('Asia/Shanghai'))>=deadline:raise RuntimeError('CUTOFF_05_00')
  if time.monotonic()-wall_start>wall_budget:raise RuntimeError('EPISODE_WALL_CLOCK_BUDGET')
  native.clear()
  replay_frame=None;cartesian_input=None
  if tape is not None:
   if replay_safety is not None and memory is None and tick*dt>=replay_safety['activation_time_s']-1e-9:
    memory=InteractionMemory(replay_safety['initial_ee'],a.output)
    memory.estimate=replay_safety['fit'];memory.follow_sign=replay_safety['follow_sign'];memory.observe(tcp());fit_poses=memory.poses;probe_start=len(rows);probe_start_time=tick*dt;memory.save()
   replay_frame=tape[tick];phase=replay_frame['phase'];mode=replay_frame['finger_mode'];effort=replay_frame['finger_effort']
   compliant=replay_frame['compliant'];qtarget[arm]=replay_frame['arm_position'];qtarget[fingers]=replay_frame['finger_position'];qvelocity[:]=replay_frame['arm_velocity']
   gain_state=(compliant,mode)
   if gain_state!=replay_last_compliant:
    kp=np.zeros(len(names));kd=kp.copy();kp[arm]=0. if compliant else 10000.;kd[arm]=0. if compliant else 400.;kp[fingers]=1000. if mode=='position' else 0.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=False);replay_last_compliant=gain_state
   if replay_frame.get('retention_armed') and reference is None:
    ready,detail=grip_window()
    if not ready:raise RuntimeError('REPLAY_BILATERAL_HOLD_NOT_ESTABLISHED')
    retention.arm();grasp_tcp=tcp().copy();reference=True;legal=True
  if compliant and (tape is None or replay_frame.get('cartesian_input') is not None):
   if replay_frame is not None:
    command=replay_frame['cartesian_input']
    if drive is None:drive=ConstrainedDrive(tcp(),command['direction_world'])
    # Replay external drive input, not the reference robot's feedback torque.
    # The frozen servo uses this twin's robot encoders and robot-only dynamics.
    drive.direction=np.asarray(command['direction_world'],dtype=float)
    drive.reference=np.asarray(command['reference_world_m'],dtype=float).copy()
    drive.active=bool(command['active']);drive.speed_m_s=0.
   view=robot._articulation_view;E=tcp();native_qdot=np.asarray(robot.get_joint_velocities(),dtype=float)[arm]
   q_measured=np.asarray(robot.get_joint_positions(),dtype=float)[arm]
   qdot=(q_measured-np.asarray(rows[-2]['q'])[arm])/dt if len(rows)>1 else native_qdot
   ee_velocity=(E[:3,3]-np.asarray(rows[-2]['T_tcp'])[:3,3])/dt if len(rows)>1 else np.zeros(3)
   J=measured_jacobian(E)
   M=np.asarray(view.get_mass_matrices())[0][np.ix_(arm,arm)]
   gravity=np.asarray(view.get_generalized_gravity_forces())[0,arm]
   coriolis=np.asarray(view.get_coriolis_and_centrifugal_forces())[0,arm]
   tau=drive.command(E,J,M,qdot,gravity,coriolis,dt)
   if phase=='ROBOT_CALIBRATION_TRANSITION':tau*=calibration_comp_fraction
   cartesian_input={'direction_world':drive.direction.tolist(),'reference_world_m':drive.reference.tolist(),'active':bool(drive.active)}
   torque_diagnostics={**drive.last,'native_joint_velocity':native_qdot.tolist(),'encoder_difference_joint_velocity':qdot.tolist(),'EE_difference_velocity_m_s':ee_velocity.tolist(),'robot_gravity_effort':gravity.tolist(),'command_arm_effort':tau.tolist()}
   if phase=='COMPLIANT_SETTLE' and not (a.output/'compliance_transition.json').exists():(a.output/'compliance_transition.json').write_text(json.dumps(torque_diagnostics,indent=2))
   if not np.all(np.isfinite(tau)):raise RuntimeError('INVALID_TASK_TORQUE')
   if np.any(np.abs(tau)>effort_limits):raise RuntimeError('EXISTING_ARM_EFFORT_LIMIT')
   if np.any(np.abs(qdot)>vel/3):raise RuntimeError('EXISTING_ARM_SPEED_LIMIT')
   if np.linalg.norm(ee_velocity)>.005:raise RuntimeError('PROBE_CARTESIAN_SPEED_LIMIT')
   controller.apply_action(ArticulationAction(joint_efforts=tau,joint_indices=arm))
  elif compliant:
   tau=np.asarray(replay_frame['arm_effort']);torque_diagnostics=replay_frame.get('diagnostics',{})
   if np.any(np.abs(tau)>effort_limits):raise RuntimeError('EXISTING_ARM_EFFORT_LIMIT')
   if len(rows)>1:
    velocity=(np.asarray(robot.get_joint_positions())[arm]-np.asarray(rows[-2]['q'])[arm])/dt
    if np.any(np.abs(velocity)>vel/3):raise RuntimeError('EXISTING_ARM_SPEED_LIMIT')
    if np.linalg.norm(tcp()[:3,3]-np.asarray(rows[-2]['T_tcp'])[:3,3])/dt>.005:raise RuntimeError('PROBE_CARTESIAN_SPEED_LIMIT')
   controller.apply_action(ArticulationAction(joint_efforts=tau,joint_indices=arm))
  else:
   controller.apply_action(ArticulationAction(joint_positions=qtarget[arm],joint_velocities=qvelocity,joint_indices=arm))
  controller.apply_action(ArticulationAction(joint_positions=qtarget[fingers],joint_indices=fingers) if mode=='position' else ArticulationAction(joint_efforts=np.array([-effort,effort]),joint_indices=fingers))
  issued.append({'t':tick*dt,'phase':phase,'compliant':compliant,'arm_position':qtarget[arm].tolist(),'arm_velocity':qvelocity.tolist(),'arm_effort':np.asarray(tau).tolist() if compliant else [0.]*6,'finger_mode':mode,'finger_position':qtarget[fingers].tolist(),'finger_effort':effort,'retention_armed':reference is not None,'diagnostics':dict(torque_diagnostics),'cartesian_input':cartesian_input})
  world.step(render=False,update_fabric=True)
  if tick%8==0:
   world.render()
  s=sample();contacts=native.state();s.update(retention.observe(tick*dt,np.asarray(s['T_tcp']),contacts['contacts']));s['relative_translation_slip_m']=retention.drift_m;s['constrained_drive']=torque_diagnostics;s.update(t=tick*dt,forces_n=contacts['finger_handle_forces_n'],contacts=contacts['contacts'],mode=mode,closing_effort_n=effort,command_q_arm=qtarget[arm].tolist());rows.append(s);tick+=1
  if abs(contacts['physics_window_dt_s']-dt)>1e-6:raise RuntimeError('PHYSICS_CLOCK_MISMATCH')
  if s['margin_rad']<=.05:raise RuntimeError('LOW_JOINT_MARGIN')
  P=poses(np.asarray(s['q']));ok,why=collision.contact_guard(s['contacts'],P)
  if not ok:raise RuntimeError(why)
  if tick%8==0:
   ok,why=collision.check(P,np.asarray(s['T_predicted_collision_body']),allow_handle=phase not in ('SETTLE','PREGRASP','APPROACH','NO_OPERATION'))
   if not ok:raise RuntimeError(why)
   image=np.asarray(near.get_rgba())[:,:,:3].copy();overview=np.asarray(scene['overview'][0].get_rgba())[:,:,:3];image[-240:,-320:]=cv2.resize(overview,(320,240));cv2.rectangle(image,(0,0),(1280,80),(12,12,12),-1)
   cv2.putText(image,f'INTERACTIVE TWIN / SIM SURROGATE | {phase}',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1);cv2.putText(image,f'EE estimate={s["estimated_angle_deg"]:.2f} deg | tactile drift={s["relative_translation_slip_m"]*1000:.2f} mm | margin={s["margin_rad"]:.3f}',(12,62),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1);video.stdin.write(np.ascontiguousarray(image).tobytes())
  if tick%240==0:
   (a.output/'progress.json').write_text(json.dumps({'phase':phase,'t':s['t'],'estimated_angle_deg':s['estimated_angle_deg'],'attempt_history':[] if memory is None else memory.attempts,'ee_travel_m':0. if memory is None else memory.travel(),'aperture_m':s['aperture_m'],'forces_n':s['forces_n'],'margin_rad':s['margin_rad'],'relative_translation_slip_m':s['relative_translation_slip_m']},indent=2))
   cv2.imwrite(str(a.output/'latest_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR))
  if phase in ('CLOSE','SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD','SMALL_PULL','OPEN_5_DEG','EXPLORATORY','ESTIMATED_FOLLOW','ZERO_PROBE','FINAL_HOLD','COMPLIANT_SETTLE','PROBE_HOLD','P1','P2','P3','P4','PHYSICS_HOLD','ROBOT_CALIBRATION') and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']:raise RuntimeError('EXISTING_LOW_PRELOAD_FORCE_LIMIT')
  if reference is not None:
   loss_s=loss_s+dt if np.min(filtered())<.05 else 0.;slip_s=slip_s+dt if max(s['relative_translation_slip_m'],s['articulation_consistency_error_m'] or 0.)>policy['max_slip_m'] else 0.
   if loss_s>.15:raise RuntimeError('SUSTAINED_CONTACT_LOSS')
   if slip_s>.1:raise RuntimeError('SUSTAINED_RELATIVE_SLIP')
  return s
 def move(q,duration):
  nonlocal qvelocity
  old=qtarget[arm].copy();delta=np.asarray(q)-old;duration=max(duration,float(np.max(1.5*np.abs(delta)/(vel/3))))
  for f in np.linspace(0,1,max(2,int(duration/dt))):qtarget[arm]=old+f*f*(3-2*f)*delta;qvelocity=6*f*(1-f)*delta/duration;step()
  qvelocity=np.zeros(6)
 def ik(target,seed):return model.ik(target,base,seed,starts=1)
 def hold(seconds):
  for _ in range(int(seconds/dt)):step()
 view=robot._articulation_view;body_index=view.get_body_index('gripper_base');jacobian_index=body_index-1
 com_local=np.asarray(view.get_body_coms()[0],dtype=float)[0,body_index]
 def measured_jacobian(E):
  J=np.asarray(view.get_jacobians(),dtype=float)[0,jacobian_index,:,:][:,arm].copy()
  p,q=actual_views['gripper_base'].get_world_poses();W=matrix(p[0],q[0])
  # PhysX body Jacobians refer to COM, not the URDF link origin.
  center_of_mass=(W@np.r_[com_local,1])[:3]
  J[:3]+=np.cross(J[3:].T,E[:3,3]-center_of_mass).T
  return J
 def validate_robot_jacobian(label):
  q=np.asarray(robot.get_joint_positions(),dtype=float)[arm];J=measured_jacobian(tcp())
  numerical=np.zeros((6,6));epsilon=1e-5;F=B@model.fk(q)
  for column in range(6):
   offset_q=q.copy();offset_q[column]+=epsilon;G=B@model.fk(offset_q)
   numerical[:,column]=np.r_[(G[:3,3]-F[:3,3])/epsilon,Rotation.from_matrix(G[:3,:3]@F[:3,:3].T).as_rotvec()/epsilon]
  jacobian_error=float(np.max(np.abs(J-numerical)))
  (a.output/('robot_control_check_'+label+'.json')).write_text(json.dumps({'jacobian_max_abs_error':jacobian_error,'COM_local_m':com_local.tolist(),'native':J.tolist(),'numeric':numerical.tolist()},indent=2))
  if jacobian_error>.002:raise RuntimeError('ROBOT_JACOBIAN_FRAME_MISMATCH')
 try:
  validate_robot_jacobian('initial')
  phase='SETTLE';hold(.5);initial_angle=0.
  pass # RGB-D is logged only, never a probe feedback gate
  if tape is not None:
   for _ in range(len(tape)-tick):step()
   ready,detail=grip_window();legal=bool(ready)
   if not ready:raise RuntimeError('REPLAY_FINAL_HOLD_LOST')
   status='REPLAY_COMPLETE';success=False
  elif job.get('mode')=='robot_calibration':
   E0=tcp();drive=ConstrainedDrive(E0,-E0[:3,2]);drive.active=True;effort_limits=np.array([float(urdf.find(f"joint[@name='joint{i}']/limit").get('effort')) for i in range(1,7)])
   kp=np.zeros(len(names));kd=kp.copy();kp[fingers]=1000.;kd[fingers]=40.;compliant=True;drive.active=False;phase='ROBOT_CALIBRATION_TRANSITION'
   # Free-space setup avoids releasing stored position-servo effort in one step.
   # This transition is only robot-only calibration, never object fitting or the
   # frozen contact/probe controller. End gains equal its zero-stiffness mode.
   for fraction in np.linspace(0.,1.,int(1./dt)):
    calibration_comp_fraction=float(fraction);kp[arm]=10000.*(1.-fraction);kd[arm]=400.*(1.-fraction);controller.set_gains(kps=kp,kds=kd,save_to_usd=False);step()
   phase='ROBOT_CALIBRATION'
   for j in range(int(10/dt)):
    drive.active=(j*dt<3 or 5<j*dt<8);step()
   status='ROBOT_CALIBRATION_COMPLETE'
  else:
   phase='PREFLIGHT';P=poses(robot.get_joint_positions());pre=T.copy();pre[:3,3]-=.04*T[:3,2]
   approach=[];seed=np.asarray(chosen['approach'][0]);seed=ik(pre,seed)
   if seed is None:raise RuntimeError('NO_PREGRASP_IK')
   for f in np.linspace(0,1,21):
    target=pre.copy();target[:3,3]=pre[:3,3]*(1-f)+T[:3,3]*f;seed=ik(target,seed)
    if seed is None:raise RuntimeError('NO_APPROACH_IK')
    P=model.poses(seed,base,width=.1);ok,why=collision.check(P,moving(),False)
    if not ok:raise RuntimeError('APPROACH_PREFLIGHT_'+why)
    approach.append(seed.tolist())
   path=chosen['preplan'];last=np.asarray(path[-1]);qpre=np.asarray(approach[0])
   for x,y in zip(path+[qpre.tolist()],(path+[qpre.tolist()])[1:]):
    for q in np.linspace(x,y,max(2,int(np.ceil(np.max(np.abs(np.asarray(y)-x))/.025))+1)):
     ok,why=collision.check(model.poses(q,base,width=.1),moving(),False)
     if not ok:raise RuntimeError('HOME_PREFLIGHT_'+why)
   phase='PREGRASP'
   for q in path[1:]:move(q,1.5)
   move(qpre,.8);phase='APPROACH'
   for q in approach[1:]:move(q,.12)
   closure=JawCenteredClosure(tcp(),policy);phase='CLOSE'
   for _ in range(int(100/dt)):
    state=rows[-1];P=poses(np.asarray(state['q']));centers=[(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]
    cmd=closure.update(tcp(),state['aperture_m'],filtered(),centers,dt);phase=cmd['state']
    if cmd['mode']=='position':qtarget[fingers]=[cmd['opening_m']/2,-cmd['opening_m']/2]
    else:
     if mode!='effort':
      kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True);mode='effort'
     effort=cmd['closing_effort_n']
    q=ik(cmd['T'],np.asarray(state['q'])[arm])
    if q is None:raise RuntimeError('CLOSURE_CENTERING_IK_FAILURE')
    qvelocity=np.clip((q-qtarget[arm])/dt,-vel/3,vel/3);qtarget[arm]=q;step()
    ready,detail=grip_window()
    if closure.state=='FORCE_HOLD' and ready:legal=True;break
   if not legal:raise RuntimeError('BILATERAL_HOLD_NOT_ESTABLISHED')
   qvelocity=np.zeros(6);phase='FORCE_HOLD';grasp_tcp=tcp().copy();retention.arm();reference=True;hold(.5)
   E0=tcp();memory=InteractionMemory(E0,a.output);probe=memory;memory.observe(E0);fit_poses=memory.poses;probe_start=len(rows);probe_start_time=tick*dt
   validate_robot_jacobian('grasp')
   effort_limits=np.array([float(urdf.find(f"joint[@name='joint{i}']/limit").get('effort')) for i in range(1,7)])
   drive=ConstrainedDrive(E0,memory.directions[0]);drive.active=False
   kp=np.zeros(len(names));kd=kp.copy();kd[fingers]=40.
   # Only the new post-grasp primitive uses Cartesian compliance. The baseline
   # approach/closure/hold functions, geometry and all force caps are untouched.
   controller.set_gains(kps=kp,kds=kd,save_to_usd=False);compliant=True
   phase='COMPLIANT_SETTLE';hold(.5)
   # Identification starts after the passive controller transition. Its small
   # elastic orientation relaxation is logged, not mixed into probe excitation.
   E0=tcp();memory=InteractionMemory(E0,a.output);probe=memory;memory.observe(E0);fit_poses=memory.poses;probe_start=len(rows);probe_start_time=tick*dt
   if a.no_operation:
    phase='ZERO_PROBE';hold(max(25.,a.control_duration-tick*dt));fit_poses=[r['T_tcp'] for r in rows[probe_start:]][::8];memory.poses=fit_poses;fit=memory.final_fit();memory.save();status='ZERO_PROBE_'+fit['joint_type']
   else:
    if job.get('initial_estimate'):
     # Only a previously saved sensor-only model is loaded; scene joint data
     # remain sealed behind GroundTruthGate until the experiment is stopped.
     estimate_path=Path(job['initial_estimate']);loaded=json.loads(estimate_path.read_text())
     if loaded.get('joint_type')!='revolute' or loaded.get('confidence',0)<=.9 or 'revolute' not in loaded:raise RuntimeError('INVALID_SAVED_ESTIMATE')
     evidence_path=Path(job.get('initial_estimate_memory',estimate_path.parent/'structured_memory.json'))
     evidence=json.loads(evidence_path.read_text())
     if evidence.get('GT_inputs') is not False:raise RuntimeError('ESTIMATE_PROVENANCE_NOT_SENSOR_ONLY')
     accepted=[item for item in evidence.get('fit_history',[]) if item.get('accepted') and item.get('fit')==loaded]
     if not accepted:raise RuntimeError('SAVED_ESTIMATE_SUPPORT_MISMATCH')
     count=int(accepted[-1]['observation_count']);observations=np.asarray(evidence['supporting_observations'][:count])
     if count<12 or len(observations)!=count or np.max(np.linalg.norm(observations[:,:3,3]-observations[0,:3,3],axis=1))<.005:raise RuntimeError('SAVED_ESTIMATE_EXCITATION_INSUFFICIENT')
     axis=np.asarray(loaded['revolute']['axis'])
     observed_rotation=Rotation.from_matrix(observations[-1,:3,:3]@observations[0,:3,:3].T).as_rotvec()
     memory.estimate=loaded;memory.follow_sign=1. if observed_rotation@axis>=0 else -1.
     memory.estimates.append({'fit':loaded,'accepted':True,'observation_count':0,'source':'saved_EE_only_estimate','source_path':str(estimate_path)})
     # Prior D support is evidence from a completed experiment. Keep it separate
     # from this run's measured trajectory; use the current EE as safety anchor.
     (a.output/'adopted_estimate.json').write_text(json.dumps({
      'schema':'adopted-ee-articulation-v1','fit':loaded,'GT_inputs':False,
      'supporting_observations':observations.tolist(),'support_scope':'prior_completed_D_before_current_physics',
      'activation_time_s':tick*dt,'initial_ee':E0.tolist(),
      'source_estimate_path':str(estimate_path.resolve()),'source_estimate_sha256':hashlib.sha256(estimate_path.read_bytes()).hexdigest(),
      'source_memory_path':str(evidence_path.resolve()),'source_memory_sha256':hashlib.sha256(evidence_path.read_bytes()).hexdigest()},indent=2))
     memory.save();following=True
    for attempt in range(0 if following else 4):
     direction=memory.begin_attempt(attempt,tick*dt);drive.set_direction(direction,tcp());drive.active=True
     phase='EXPLORATORY';last_fit=0.;attempt_start=tick*dt;reason='UNOBSERVABLE_ATTEMPT_TIMEOUT'
     for j in range(int(35/dt)):
      step()
      if j%8==0:
       memory.observe(tcp());elapsed=tick*dt-attempt_start
       if elapsed-last_fit>=1.:
        last_fit=elapsed;memory.try_fit()
        if memory.estimate is not None:
         following=True;reason='RELIABLE_REVOLUTE_ESTIMATE';break
       if elapsed>=12. and memory.travel(memory.start_index)<.002:
        reason='SAFE_LOW_EXCITATION';break
       if memory.travel()>.020:
        reason='TOTAL_PROBE_EXCITATION_BUDGET';break
     memory.finish_attempt(tick*dt,reason)
     if following or reason=='TOTAL_PROBE_EXCITATION_BUDGET':break
     phase='PROBE_HOLD';drive.active=False;hold(1.)
    if not following:raise RuntimeError('UNOBSERVABLE')
    fit=memory.estimate;probe_metrics={'distance_m':memory.travel(),'duration_s':tick*dt-probe_start_time,'fit_input':'measured robot TCP SE(3) only','fit_saved_before_follow':True}
    (a.output/'estimated_articulation.json').write_text(json.dumps(fit,indent=2));fit_saved=True
    if job.get('mode') in ('physics_reference','sensitivity_reference'):
     protocol=PhysicsProtocol(memory,drive,job.get('physics_protocol'))
     for segment in protocol.segments:
      phase=segment['probe_id'];protocol.start(segment,tcp());segment_start=tcp()[:3,3].copy()
      for j in range(int(segment['duration_s']/dt)):
       protocol.update(segment,j*dt,tcp());step()
       if j%8==0:memory.observe(tcp())
     protocol.finish();protocol_completed=True
    drive.active=True
    phase='ESTIMATED_FOLLOW';follow_start=tick*dt;segment_start=tcp()[:3,3].copy();last_refine=tick*dt;drive.set_direction(memory.tangent(tcp()),tcp())
    for j in range(int(((180 if job.get('refinement_once') else 90) if job.get('continue_manipulation',True) else 0)/dt)):
     step()
     if j%8==0:
      E=tcp();memory.observe(E)
      if np.linalg.norm(E[:3,3]-segment_start)>=.001:
       drive.refresh_tangent(memory.tangent(E),E);segment_start=E[:3,3].copy()
      if tick*dt-last_refine>=2.:
       memory.try_fit();last_refine=tick*dt
      if memory.angle_deg(E)>=float(job.get('refinement_target_deg',5.5)):break
    if job.get('continue_manipulation',True) and memory.angle_deg(tcp())<float(job.get('refinement_target_deg',5.5)):raise RuntimeError('ESTIMATED_FOLLOW_TIMEOUT')
    if job.get('refinement_once'):
     # One reversal, same compliant controller and all existing safety guards.
     # Reference uses saved/EE-estimated geometry; no simulator object state.
     phase='REFINEMENT_REVERSAL_SETTLE';drive.active=False;hold(1.)
     phase='REFINEMENT_REVERSE';reverse_start=memory.angle_deg(tcp());segment_start=tcp()[:3,3].copy()
     drive.set_direction(-memory.tangent(tcp()),tcp());drive.active=True
     for j in range(int(65/dt)):
      step()
      if j%8==0:
       E=tcp();memory.observe(E)
       if np.linalg.norm(E[:3,3]-segment_start)>=.001:
        drive.refresh_tangent(-memory.tangent(E),E);segment_start=E[:3,3].copy()
       if reverse_start-memory.angle_deg(E)>=2.5:break
     (a.output/'refinement_actual.json').write_text(json.dumps({'estimated_opening_deg':reverse_start,'estimated_reverse_deg':reverse_start-memory.angle_deg(tcp()),'GT_used':False,'supplemental_trajectory_budget':1},indent=2))
    phase='FINAL_HOLD';drive.active=False;hold(2.);ready,detail=grip_window();success=bool(ready and retention.drift_m<=policy['max_slip_m']);status='ESTIMATED_OPENING_HOLD_COMPLETE' if success else 'ESTIMATED_OPENING_HOLD_FAILED'
    if protocol_completed and not job.get('continue_manipulation',True):
     status='PHYSICS_PROTOCOL_COMPLETE' if success else 'PHYSICS_FINAL_HOLD_FAILED';success=False

 except BaseException as error:
  import traceback
  status=str(error) if isinstance(error,RuntimeError) else 'IMPLEMENTATION_ERROR:'+str(error);print(traceback.format_exc(),flush=True)
 finally:
  world.pause()
  if memory is not None and len(fit_poses)<2 and len(rows)>probe_start:
   fit_poses.extend([r['T_tcp'] for r in rows[probe_start:]][::8])
  if probe is not None and fit_poses and not probe_metrics:
   points=np.asarray(fit_poses);rotation=Rotation.from_matrix(points[:,:3,:3]@points[0,:3,:3].T).as_rotvec()
   probe_metrics={'duration_s':max(0.,rows[-1]['t']-rows[probe_start]['t']) if len(rows)>probe_start else 0.,'distance_m':float(np.max(np.linalg.norm(points[:,:3,3]-points[0,:3,3],axis=1))),'observed_rotation_span_deg':float(np.rad2deg(np.max(np.linalg.norm(rotation,axis=1)))),'sample_count':len(points),'fit_input':'measured robot TCP SE(3) only','failure_phase':phase}
  (a.output/'ee_probe_trajectory.json').write_text(json.dumps(fit_poses))
  if memory is not None and memory.estimate is not None:
   fit=memory.estimate;(a.output/'estimated_articulation.json').write_text(json.dumps(fit,indent=2));fit_saved=True
  if not fit_saved:
   fit=memory.final_fit() if memory is not None else {'joint_type':'UNOBSERVABLE','confidence':0.,'reason':'no post-grasp EE observations'}
   (a.output/'estimated_articulation.json').write_text(json.dumps(fit,indent=2));fit_saved=True
  if memory is not None:
   if memory.attempts and memory.attempts[-1]['result']=='RUNNING':memory.finish_attempt(tick*dt,status)
   memory.save()
  gt_gate.open=True
  # First access to simulator joint geometry/state AFTER interaction stopped
  # and the sensor-only fit was written. Nothing below drives the robot.
  joint=scene['asset_chain'].joints[scene['moving_link']];asset_world=np.eye(4);asset_world[:3,:3]=scene['asset_rotation'];asset_world[:3,3]=scene['asset_xyz'];H=asset_world@scene['asset_chain'].root_to_link(joint.parent,{})@joint.origin
  gt={'joint_type':joint.kind,'axis_world':(H[:3,:3]@joint.axis).tolist(),'origin_world':H[:3,3].tolist()}
  actual_angle=float(np.rad2deg(scene['articulation'].get_joint_positions()[0]));evaluation=evaluate(fit,gt,np.asarray(fit_poses[0])[:3,3],fit_poses) if fit_poses else {'type_correct':False}
  displacement=actual_angle-float(np.rad2deg(job.get('initial_articulation_rad',0.)))
  evaluation.update(actual_door_displacement_deg=displacement,actual_final_door_angle_deg=actual_angle,gt_access_after_saved_fit=True,gt_access_after_world_pause=True)
  if grasp_tcp is not None:
   final_object=matrix(*scene['door_link'].get_world_pose());before=np.linalg.inv(moving_initial)@grasp_tcp;after=np.linalg.inv(final_object)@tcp()
   evaluation['final_true_relative_translation_slip_m']=float(np.linalg.norm(before[:3,3]-after[:3,3]))
   evaluation['final_true_relative_rotation_slip_deg']=float(np.rad2deg(Rotation.from_matrix(after[:3,:3]@before[:3,:3].T).magnitude()))
  if success:
   success=displacement>=5. and evaluation.get('final_true_relative_translation_slip_m',float('inf'))<=policy['max_slip_m']
   status='SUCCESS' if success else ('FINAL_TRUE_RELATIVE_SLIP' if evaluation.get('final_true_relative_translation_slip_m',0)>policy['max_slip_m'] else 'ESTIMATED_GOAL_ACTUAL_OPENING_BELOW_5_DEG')
  world.render();cv2.imwrite(str(a.output/'final_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR));video.stdin.close();video.wait(timeout=30);export_cooked(stage,a.output/'cooked_final.json');(a.output/'observations.json').write_text(json.dumps(rows));(a.output/'physics_steps.json').write_text(json.dumps(native.physics_steps));(a.output/'evaluation_only.json').write_text(json.dumps({'ground_truth':gt,'evaluation':evaluation},indent=2))
  report={'status':status,'success':success,'mode':'Act2See-style EE-only constrained real-contact interaction','zero_probe_uses_same_compliance':True,'baseline_commit':'f5dcc6f','candidate':a.candidate,'base_fixed':base,'official_unsplit_finger_counts':{n:sum(s.body==n for s in collision.robot) for n in FINGERS},'grasp_contact_implementation_unchanged':True,'gt_control_inputs':False,'gt_fit_inputs':False,'gt_runtime_object_pose_reads':False,'observed_handle_motion':'not used online; initial geometry and grasp only','attempt_history':[] if memory is None else memory.attempts,'accepted_estimate_count':0 if memory is None else sum(r['accepted'] for r in memory.estimates),'estimated_model_refined':False if memory is None else sum(r['accepted'] for r in memory.estimates)>1,'estimate':fit,'probe':probe_metrics,'evaluation':evaluation,'bilateral_hold_established':legal,'followed_estimated_articulation':following,'minimum_joint_margin_rad':min((s['margin_rad'] for s in rows),default=None),'slip_measurement':'contact-plane normal drift and tilt only; tangential slip unobservable, not maximum GT relative slip','collision_prediction':'measured EE transform under retained-grasp assumption; native actual contact guard remains active','probe_controller':'robot gravity/Coriolis + Jacobian-transpose directional impedance, free orthogonal and rotational stiffness' ,'full_6D_pose_tracking_during_probe':False,'full_relative_slip_observable_online':False,'maximum_true_relative_slip_m':None,'peak_estimated_model_consistency_error_m':max((s['articulation_consistency_error_m'] or 0. for s in rows),default=None),'maximum_detected_contact_surface_drift_m':max((s['relative_translation_slip_m'] for s in rows),default=None),'peak_finger_handle_force_n':max((max(s['forces_n'].values()) for s in rows),default=0.),'duration_s':len(rows)*dt,'first_failure_state':rows[-1] if rows and not success and not a.no_operation else None,'no_operation':a.no_operation,'video':'contact_baseline.mp4','attachments':False,'object_actuation':False,'GT_access_log_after_fit':gt_gate.accesses,'GT_scope':'scene assembly only; static geometry and final joint state read only after saved fit and stopped interaction'}
  report.update(experiment='interactive_twin',episode_id=job.get('episode_id'),mode=job.get('observation_mode','SIM_TO_SIM_BLIND_SYSID'),role=job.get('role','reference'),physics_protocol_complete=protocol_completed,simulator_only_safety_supervisor=True,hardware_deployable_contact_supervisor=False,replay_uses_same_estimated_consistency_safety=replay_safety is not None)
  if status not in ('SUCCESS','PHYSICS_PROTOCOL_COMPLETE','REPLAY_COMPLETE','ROBOT_CALIBRATION_COMPLETE'):report['failure_phase']=phase
  (a.output/'command_tape.json').write_text(json.dumps(issued))
  save_observable_logs(a.output,rows,issued,arm,job,report)
  from interactive_twin_refinement.data import save_actions
  save_actions(a.output,rows,report)
  (a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='first_failure_state'},indent=2),flush=True);app.close()

if __name__=='__main__':main()
