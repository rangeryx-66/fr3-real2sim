"""Unknown-articulation physical-contact probe; frozen grasp and safety baseline.

Door drives/forces/attachments are never commanded during execution.
"""
from piper_mobile_execute import *

def parser():
 p=argparse.ArgumentParser(description=__doc__)
 for k in ('source','asset-root','plan','output'):p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--gpu',type=int,default=6);p.add_argument('--repeat',type=int,default=3)
 p.add_argument('--trial',action='store_true');p.add_argument('--candidate',type=int,default=0,choices=[0]);p.add_argument('--no-operation',action='store_true');p.add_argument('--control-duration',type=float,default=100.)
 now=datetime.now(ZoneInfo('Asia/Shanghai'));cutoff=now.replace(hour=5,minute=0,second=0,microsecond=0)
 if cutoff<=now:cutoff+=timedelta(days=1)
 p.add_argument('--deadline-shanghai',default=cutoff.isoformat())
 return p

def main():
 a=parser().parse_args();a.output.mkdir(parents=True,exist_ok=True)
 if not a.trial:
  runs=[];success=None
  (a.output/'summary.json').write_text(json.dumps({'mode':'unknown-articulation real-contact RGB-D probe','runs':[],'successes':0,'complete':False},indent=2))
  def run(index,name,control=False,duration=100):
   if datetime.now(ZoneInfo('Asia/Shanghai'))>=datetime.fromisoformat(a.deadline_shanghai):
    return {'status':'CUTOFF_05_00','success':False}
   out=a.output/name;cmd=[sys.executable,__file__,'--trial','--source',str(a.source),'--asset-root',str(a.asset_root),'--plan',str(a.plan),'--output',str(out),'--candidate',str(index),'--gpu',str(a.gpu),'--deadline-shanghai',a.deadline_shanghai]
   if control:cmd+=['--no-operation','--control-duration',str(duration)]
   with open(a.output/(name+'.log'),'w') as f:ret=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
   r=json.loads((out/'report.json').read_text()) if (out/'report.json').exists() else {'status':'PROCESS_FAILED','exit_code':ret.returncode,'success':False}
   runs.append({'name':name,'candidate':index,'report':r});print(name,r.get('status'),flush=True)
   (a.output/'summary.json').write_text(json.dumps({'mode':'unknown-articulation real-contact RGB-D probe','runs':runs,'successes':sum(x['report'].get('success',False) for x in runs),'complete':False},indent=2))
   return r
  for index in range(1):
   if datetime.now(ZoneInfo('Asia/Shanghai'))>=datetime.fromisoformat(a.deadline_shanghai):break
   r=run(index,f'candidate_{index:02d}')
   if r.get('status')=='PROCESS_FAILED' or r.get('status','').startswith('IMPLEMENTATION_ERROR:'):raise RuntimeError('IMPLEMENTATION_ERROR: stop before testing further grasps')
   if r.get('success'):success=(index,r);break
  if success:
   index,first=success
   for i in range(a.repeat):
    if datetime.now(ZoneInfo('Asia/Shanghai'))>=datetime.fromisoformat(a.deadline_shanghai):break
    run(index,f'repeat_{i+1:02d}')
   run(index,'no_operation',True,first['duration_s'])
  else:
   # Even a failed contact run gets a passive initial-state control.
   run(0,'no_operation',True,max((x['report'].get('duration_s',5.) for x in runs),default=5.))
  d=json.loads((a.output/'summary.json').read_text());d.update(complete=True,deadline_reached=datetime.now(ZoneInfo('Asia/Shanghai'))>=datetime.fromisoformat(a.deadline_shanghai),first_success_candidate=None if not success else success[0],attempted_grasp_candidates=sum(x['name'].startswith('candidate') for x in runs),actual_contact_success_count=sum(x['report'].get('success',False) for x in runs if x['name']!='no_operation'),grasp_run_count=sum(x['name']!='no_operation' for x in runs));(a.output/'summary.json').write_text(json.dumps(d,indent=2));return
 if datetime.now(ZoneInfo('Asia/Shanghai'))>=datetime.fromisoformat(a.deadline_shanghai):
  (a.output/'report.json').write_text(json.dumps({'status':'CUTOFF_05_00','success':False},indent=2));return
 source=json.loads((a.source/'report.json').read_text());base=source['robot_base_pose'];plan=json.loads(a.plan.read_text());chosen=plan['trial_candidates'][0]
 if base!=chosen['base']:raise RuntimeError('BASE_CHANGED')
 a.ownership=None;scene=bootstrap(a,base);world=scene['world'];app=scene['app'];robot=scene['robot'];stage=scene['stage'];arm=scene['arm'];fingers=scene['fingers'];names=scene['names'];controller=scene['controller'];dt=scene['DT']
 from isaacsim.core.utils.types import ArticulationAction
 from isaacsim.sensors.camera import Camera
 from interaction_identification.contact_probe import robot_only_model,DepthMotion,ObservedCompliantPull,high_quality_revolute,GroundTruthGate
 from interaction_identification.fitting import fit_articulation,evaluate
 from piper_mobile_demo.cooked_geometry import export_cooked
 from articulated_interaction.physical_baseline import PhysicalScene,WholeFingerReports,FINGERS
 from articulated_interaction.control import JawCenteredClosure
 from articulated_demo.kinematics import transform
 import cv2
 gt_gate=GroundTruthGate();gt_gate.protect(scene)
 deadline=datetime.fromisoformat(a.deadline_shanghai)
 model=robot_only_model(ROOT/'config/piper.urdf');model.deadline_timestamp=deadline.timestamp()
 export_cooked(stage,a.output/'cooked_initial.json');export=json.loads((a.output/'cooked_initial.json').read_text())
 allowed=[e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]
 if len(allowed)!=3:raise RuntimeError('FROZEN_PROXY_ASSOCIATION_FAILED')
 collision=PhysicalScene(export,model,allowed)
 T=np.asarray(chosen['T']);slide=depth=roll=0.;normal=-T[:3,2];anchor_world=T[:3,3].copy()
 moving_initial=collision.moving_reference
 anchor_link=(np.linalg.inv(moving_initial)@np.r_[anchor_world,1])[:3]
 fingerprint={'manifest_sha256':hashlib.sha256((a.asset_root/'manifest.json').read_bytes()).hexdigest(),'geometry':'unchanged frozen semantic proxy','baseline_commit':'f5dcc6f'}
 if fingerprint['manifest_sha256']!='a1bc615255468175c665311a47c798d63826fc12e5bc87821f3ea04ebd7c0cb4':raise RuntimeError('FROZEN_PROXY_CHANGED')
 (a.output/'frozen_proxy.json').write_text(json.dumps(fingerprint,indent=2))
 actual_views=dict(scene['scene_monitor_views']);B=transform(base[:3],[0,0,np.deg2rad(base[3])]);chain=model.chain
 positions=dict(zip(names,robot.get_joint_positions()));offset=np.linalg.inv(chain.root_to_link('gripper_base',positions))@chain.root_to_link('tcp_link',positions)
 observer=None;probe=None;fit=None;fit_poses=[];fit_saved=False;following=False;probe_metrics={};evaluation={}
 def moving():
  return (np.eye(4) if observer is None else observer.delta)@moving_initial
 def tcp():
  p,q=actual_views['gripper_base'].get_world_poses();return matrix(p[0],q[0])@offset
 def poses(q):return model.poses(np.asarray(q)[arm],base,finger_q=np.asarray(q)[fingers])
 qtarget=np.asarray(robot.get_joint_positions(),dtype=float).copy();qtarget[fingers]=[.05,-.05];qvelocity=np.zeros(6);phase='SETTLE';rows=[];tick=0;reference=None;legal=False;success=False;pull2=False;status='STARTED';loss_s=0.;slip_s=0.;preflight={}
 urdf=ET.parse(ROOT/'config/piper.urdf').getroot();vel=np.array([float(urdf.find(f"joint[@name='joint{i}']/limit").get('velocity')) for i in range(1,7)]);limits=model.limits
 policy=json.loads((ROOT/'config/semantic_interaction.json').read_text());policy['slow_closure_m_s']=.001;mode='position';effort=0.
 near=scene['overview'][1];eye=anchor_world+normal*.45+np.array([0,0,.20]);focus=anchor_world+np.array([0,0,.02]);forward=(focus-eye)/np.linalg.norm(focus-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward));near.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros');near.set_clipping_range(.02,3.);near.add_distance_to_image_plane_to_frame();camera_world=matrix(*near.get_world_pose(camera_axes='ros'));observer=DepthMotion(near,camera_world,near.get_intrinsics_matrix(),T,model)
 video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(a.output/'contact_baseline.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
 def sample():
  q=np.asarray(robot.get_joint_positions());P=poses(q);D=moving();E=tcp();rel=np.linalg.inv(D)@E
  return {'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':model.margin(q[arm]),'finger_world_poses':{n:P[n].tolist() for n in FINGERS},'T_tcp':E.tolist(),'T_observed_collision_body':D.tolist(),'relative_translation_slip_m':0. if reference is None else float(np.linalg.norm(rel[:3,3]-reference[:3,3])),'relative_rotation_slip_deg':0. if reference is None else float(np.rad2deg(Rotation.from_matrix(rel[:3,:3]@reference[:3,:3].T).magnitude())),'estimated_angle_deg':0. if probe is None or probe.estimate is None else probe.angle_deg(E), 'handle_observation':{} if observer is None else observer.status.copy()}
 native=WholeFingerReports(stage,export,allowed,world,dt,sample)
 def window(seconds=.3):return rows[-max(1,int(seconds/dt)):]
 def filtered():return np.mean([list(s['forces_n'].values()) for s in window(.08)],axis=0) if rows else np.zeros(2)
 def grip_window():
  w=window(.5);loads=np.array([list(s['forces_n'].values()) for s in w]);opening=np.array([s['aperture_m'] for s in w]);q=np.asarray(robot.get_joint_positions());P=poses(q);centers=np.array([(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]);center=(moving()@np.r_[anchor_link,1])[:3];axis_jaw=centers[0]-centers[1];axis_jaw/=np.linalg.norm(axis_jaw);interval=np.sort(centers@axis_jaw);between=interval[0]<=center@axis_jaw<=interval[1]
  good=len(w)>=int(.45/dt) and between and np.all(np.mean(loads>.05,axis=0)>=.6) and np.all(np.mean(loads,axis=0)>=.1) and np.ptp(opening)<.0005
  return good,{'between_jaws':bool(between),'force_duty':np.mean(loads>.05,axis=0).tolist(),'mean_forces_n':np.mean(loads,axis=0).tolist(),'aperture_range_m':float(np.ptp(opening))}
 def step():
  nonlocal tick,loss_s,slip_s
  if datetime.now(ZoneInfo('Asia/Shanghai'))>=deadline:raise RuntimeError('CUTOFF_05_00')
  native.clear();controller.apply_action(ArticulationAction(joint_positions=qtarget[arm],joint_velocities=qvelocity,joint_indices=arm))
  controller.apply_action(ArticulationAction(joint_positions=qtarget[fingers],joint_indices=fingers) if mode=='position' else ArticulationAction(joint_efforts=np.array([-effort,effort]),joint_indices=fingers))
  world.step(render=False,update_fabric=True)
  if tick%8==0:
   world.render();observer.update(poses(robot.get_joint_positions()),tick*dt)
  s=sample();contacts=native.state();s.update(t=tick*dt,forces_n=contacts['finger_handle_forces_n'],contacts=contacts['contacts'],mode=mode,closing_effort_n=effort,command_q_arm=qtarget[arm].tolist());rows.append(s);tick+=1
  if abs(contacts['physics_window_dt_s']-dt)>1e-6:raise RuntimeError('PHYSICS_CLOCK_MISMATCH')
  if s['margin_rad']<=.05:raise RuntimeError('LOW_JOINT_MARGIN')
  P=poses(np.asarray(s['q']));ok,why=collision.contact_guard(s['contacts'],P)
  if not ok:raise RuntimeError(why)
  if tick%8==0:
   ok,why=collision.check(P,np.asarray(s['T_observed_collision_body']),allow_handle=phase not in ('SETTLE','PREGRASP','APPROACH','NO_OPERATION'))
   if not ok:raise RuntimeError(why)
   image=np.asarray(near.get_rgba())[:,:,:3].copy();overview=np.asarray(scene['overview'][0].get_rgba())[:,:,:3];image[-240:,-320:]=cv2.resize(overview,(320,240));cv2.rectangle(image,(0,0),(1280,80),(12,12,12),-1)
   cv2.putText(image,f'UNKNOWN ARTICULATION / REAL CONTACT | {phase}',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1);cv2.putText(image,f'EE estimate={s["estimated_angle_deg"]:.2f} deg | slip={s["relative_translation_slip_m"]*1000:.2f} mm | margin={s["margin_rad"]:.3f}',(12,62),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1);video.stdin.write(np.ascontiguousarray(image).tobytes())
  if tick%240==0:
   (a.output/'progress.json').write_text(json.dumps({'phase':phase,'t':s['t'],'estimated_angle_deg':s['estimated_angle_deg'],'rgbd_tracker':observer.status,'aperture_m':s['aperture_m'],'forces_n':s['forces_n'],'margin_rad':s['margin_rad'],'relative_translation_slip_m':s['relative_translation_slip_m']},indent=2))
   cv2.imwrite(str(a.output/'latest_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR))
  if phase in ('CLOSE','SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD','SMALL_PULL','OPEN_5_DEG','EXPLORATORY','ESTIMATED_FOLLOW','ZERO_PROBE','FINAL_HOLD') and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']:raise RuntimeError('EXISTING_LOW_PRELOAD_FORCE_LIMIT')
  if reference is not None:
   loss_s=loss_s+dt if np.min(filtered())<.05 else 0.;slip_s=slip_s+dt if s['relative_translation_slip_m']>policy['max_slip_m'] else 0.
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
 try:
  phase='SETTLE';hold(.5);initial_angle=0.
  if not observer.status.get('valid'):raise RuntimeError('RGBD_OBSERVATION_UNAVAILABLE')
  if True:
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
   qvelocity=np.zeros(6);phase='FORCE_HOLD';observer.initialize_grasp(poses(robot.get_joint_positions()),tick*dt);reference=np.linalg.inv(moving())@tcp();hold(.5)
   if not observer.status.get('valid'):raise RuntimeError('RGBD_OBSERVATION_UNAVAILABLE')
   E0=tcp();probe=ObservedCompliantPull(E0,observer.delta,policy);fit_poses=[E0.tolist()];probe_start=len(rows);probe_duration=0.;last_fit=0.
   if a.no_operation:
    phase='ZERO_PROBE';hold(25.);fit_poses=[r['T_tcp'] for r in rows[probe_start:]][::8];fit=fit_articulation(fit_poses);(a.output/'estimated_articulation.json').write_text(json.dumps(fit,indent=2));fit_saved=True;status='ZERO_PROBE_'+fit['joint_type']
   else:
    for j in range(int(130/dt)):
     phase='ESTIMATED_FOLLOW' if following else 'EXPLORATORY'
     if j%8==0:
      if not observer.status.get('valid'):raise RuntimeError('RGBD_TRACKING_LOSS')
      E=tcp();P=poses(robot.get_joint_positions());centers=[(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]
      cmd=probe.update(E,observer.delta,filtered(),centers,8*dt)
      # Keep the frozen hold command's measured tracking bias. Sending the
      # sagged measured pose as an absolute setpoint on every update would
      # ratchet gravity deflection into a spurious downward probe.
      command_pose=B@model.fk(qtarget[arm])
      ik_target=cmd@np.linalg.inv(E)@command_pose
      q=ik(ik_target,np.asarray(robot.get_joint_positions())[arm])
      if q is None:raise RuntimeError('PROBE_IK_CONTROL_FAILURE')
      if np.max(np.abs(q-np.asarray(robot.get_joint_positions())[arm]))>.05:raise RuntimeError('IK_BRANCH_JUMP')
      qvelocity=np.clip((q-qtarget[arm])/(8*dt),-vel/3,vel/3);qtarget[arm]=q
     step()
     if j%8==0:
      fit_poses.append(tcp().tolist());probe_duration=(j+1)*dt
      if not following and probe_duration-last_fit>=1. and len(fit_poses)>12:
       last_fit=probe_duration;candidate_fit=fit_articulation(fit_poses);(a.output/'fit_progress.json').write_text(json.dumps(candidate_fit,indent=2))
       if high_quality_revolute(candidate_fit):
        fit=candidate_fit;(a.output/'estimated_articulation.json').write_text(json.dumps(fit,indent=2));fit_saved=True;probe.set_estimate(fit,tcp());following=True
        probe_metrics={'duration_s':probe_duration,'distance_m':fit['travel_m'],'observed_angle_span_deg':float(np.rad2deg(fit['revolute']['angle_span_rad'])),'fit_input':'measured robot TCP SE(3) only','fit_saved_before_follow':True}
      if not following and (probe_duration>=40 or np.linalg.norm(tcp()[:3,3]-E0[:3,3])>.015):raise RuntimeError('UNOBSERVABLE')
      if following and probe.angle_deg(tcp())>=5.5:break
    if not following:raise RuntimeError('UNOBSERVABLE')
    if probe.angle_deg(tcp())<5.5:raise RuntimeError('ESTIMATED_FOLLOW_TIMEOUT')
    phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(2.);ready,detail=grip_window();success=ready and rows[-1]['relative_translation_slip_m']<=policy['max_slip_m'];status='ESTIMATED_OPENING_HOLD_COMPLETE' if success else 'ESTIMATED_OPENING_HOLD_FAILED'

 except BaseException as error:
  import traceback
  status=str(error) if isinstance(error,RuntimeError) else 'IMPLEMENTATION_ERROR:'+str(error);print(traceback.format_exc(),flush=True)
 finally:
  world.pause()
  if probe is not None and fit_poses and not probe_metrics:
   points=np.asarray(fit_poses);rotation=Rotation.from_matrix(points[:,:3,:3]@points[0,:3,:3].T).as_rotvec()
   probe_metrics={'duration_s':max(0.,rows[-1]['t']-rows[probe_start]['t']) if len(rows)>probe_start else 0.,'distance_m':float(np.max(np.linalg.norm(points[:,:3,3]-points[0,:3,3],axis=1))),'observed_rotation_span_deg':float(np.rad2deg(np.max(np.linalg.norm(rotation,axis=1)))),'sample_count':len(points),'fit_input':'measured robot TCP SE(3) only','failure_phase':phase}
  (a.output/'ee_probe_trajectory.json').write_text(json.dumps(fit_poses))
  if not fit_saved:
   fit=fit_articulation(fit_poses) if len(fit_poses)>=12 else {'joint_type':'UNOBSERVABLE','confidence':0.,'reason':'insufficient observed samples'}
   (a.output/'estimated_articulation.json').write_text(json.dumps(fit,indent=2));fit_saved=True
  gt_gate.open=True
  # First access to simulator joint geometry/state AFTER interaction stopped
  # and the sensor-only fit was written. Nothing below drives the robot.
  joint=scene['asset_chain'].joints[scene['moving_link']];asset_world=np.eye(4);asset_world[:3,:3]=scene['asset_rotation'];asset_world[:3,3]=scene['asset_xyz'];H=asset_world@scene['asset_chain'].root_to_link(joint.parent,{})@joint.origin
  gt={'joint_type':joint.kind,'axis_world':(H[:3,:3]@joint.axis).tolist(),'origin_world':H[:3,3].tolist()}
  actual_angle=float(np.rad2deg(scene['articulation'].get_joint_positions()[0]));evaluation=evaluate(fit,gt,np.asarray(fit_poses[0])[:3,3],fit_poses) if fit_poses else {'type_correct':False}
  evaluation.update(actual_final_door_angle_deg=actual_angle,gt_access_after_saved_fit=True,gt_access_after_world_pause=True)
  if success:success=actual_angle>=5.;status='SUCCESS' if success else 'ESTIMATED_GOAL_ACTUAL_OPENING_BELOW_5_DEG'
  world.render();cv2.imwrite(str(a.output/'final_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR));video.stdin.close();video.wait(timeout=30);export_cooked(stage,a.output/'cooked_final.json');(a.output/'observations.json').write_text(json.dumps(rows));(a.output/'physics_steps.json').write_text(json.dumps(native.physics_steps));(a.output/'rgbd_motion.json').write_text(json.dumps(observer.frames));(a.output/'evaluation_only.json').write_text(json.dumps({'ground_truth':gt,'evaluation':evaluation},indent=2))
  report={'status':status,'success':success,'mode':'unknown-articulation real-contact RGB-D probe','baseline_commit':'f5dcc6f','candidate':0,'base_fixed':base,'official_unsplit_finger_counts':{n:sum(s.body==n for s in collision.robot) for n in FINGERS},'grasp_contact_implementation_unchanged':True,'gt_control_inputs':False,'gt_fit_inputs':False,'gt_runtime_object_pose_reads':False,'observed_handle_motion':'initial calibrated RGB-D points + optical PnP, free SE3; no GT segmentation/link pose','estimate':fit,'probe':probe_metrics,'evaluation':evaluation,'bilateral_hold_established':legal,'followed_estimated_articulation':following,'minimum_joint_margin_rad':min((s['margin_rad'] for s in rows),default=None),'slip_measurement':'RGB-D observed handle motion relative to measured robot TCP; not simulator-GT slip','command_tracking_bias_preserved':True,'maximum_relative_translation_slip_m':max((s['relative_translation_slip_m'] for s in rows),default=None),'peak_finger_handle_force_n':max((max(s['forces_n'].values()) for s in rows),default=0.),'duration_s':len(rows)*dt,'first_failure_state':rows[-1] if rows and not success and not a.no_operation else None,'no_operation':a.no_operation,'video':'contact_baseline.mp4','attachments':False,'object_actuation':False,'GT_access_log_after_fit':gt_gate.accesses,'GT_scope':'scene assembly only; static geometry and final joint state read only after saved fit and stopped interaction'}
  (a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='first_failure_state'},indent=2),flush=True);app.close()

if __name__=='__main__':main()
