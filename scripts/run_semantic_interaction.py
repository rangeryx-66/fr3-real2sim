"""Independent real-contact probe. Controller receives robot FK, pad positions and left/right pad loads only.

Object joint/link truth is recorded in evaluation records, never controller inputs.
No attachments, object actuation, hinge-arc commands, or base reposition.
"""
from piper_mobile_execute import *
from articulated_interaction.contact_validation import audit_envelope,NonpadClassifier
from articulated_interaction.control import JawCenteredClosure,PadCompliantPull
from piper_mobile_demo.contact_ownership import NativeOwnershipReports
from piper_mobile_demo.cooked_geometry import export_cooked
from articulated_demo.kinematics import URDFChain,transform
from scipy.optimize import least_squares

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for n in ['source','asset-root','output']:p.add_argument('--'+n,type=Path,required=True)
 p.add_argument('--policy','--calibration',dest='calibration',type=Path,default=ROOT/'config/semantic_interaction.json');p.add_argument('--plan',type=Path);p.add_argument('--candidate-index',type=int,default=0);p.add_argument('--gpu',type=int,default=6);p.add_argument('--ownership',type=Path,default=ROOT/'config/piper_contact_ownership.json');p.add_argument('--export-only',action='store_true');p.add_argument('--diagnose-first-nonpad',action='store_true');p.add_argument('--deadline-shanghai');p.add_argument('--max-probe-mm',type=float,default=2.)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);cal=json.loads(a.calibration.read_text());meta=json.loads((a.asset_root/'manifest.json').read_text());source=json.loads((a.source/'report.json').read_text());base=source['robot_base_pose'];now=datetime.now(ZoneInfo('Asia/Shanghai'));deadline=datetime.fromisoformat(a.deadline_shanghai) if a.deadline_shanghai else now.replace(hour=5,minute=0,second=0,microsecond=0)
 if deadline.tzinfo is None:deadline=deadline.replace(tzinfo=ZoneInfo('Asia/Shanghai'))
 if not a.deadline_shanghai and deadline<now:deadline+=timedelta(days=1)
 if now>=deadline:raise RuntimeError('CUTOFF_05_00')
 if cal['ownership_sha256']!=hashlib.sha256(a.ownership.read_bytes()).hexdigest():raise RuntimeError('UNCALIBRATED_OWNERSHIP')
 plan=json.loads(a.plan.read_text()) if a.plan else None;chosen=None if plan is None else plan['trial_candidates'][a.candidate_index]
 if chosen is not None and chosen['base']!=base:raise RuntimeError('BASE_CHANGED')
 scene=bootstrap(a,base);world=scene['world'];robot=scene['robot'];controller=scene['controller'];arm=scene['arm'];fingers=scene['fingers'];names=scene['names'];dt=scene['DT'];stage=scene['stage'];app=scene['app']
 from isaacsim.core.utils.types import ArticulationAction
 import cv2
 chain=URDFChain(ROOT/'config/piper.urdf');B=transform(base[:3],[0,0,np.deg2rad(base[3])]);root=ET.parse(ROOT/'config/piper.urdf').getroot();limits=np.array([[float(root.find(f"joint[@name='joint{i}']/limit").get(k)) for k in ('lower','upper')] for i in range(1,7)])
 def poses(q):return {n:B@chain.root_to_link(n,dict(zip(names,q))) for n in ['tcp_link','gripper_link1','gripper_link2','link6','gripper_base']}
 def ik(T,seed):
  def residual(q):
   # PhysX states are float32. Preserve the solver's float64 perturbations;
   # assigning them into a float32 buffer silently zeroes finite differences.
   full=np.asarray(robot.get_joint_positions(),dtype=np.float64).copy();full[arm]=q;P=poses(full)['tcp_link'];return np.r_[P[:3,3]-T[:3,3],.1*Rotation.from_matrix(P[:3,:3]@T[:3,:3].T).as_rotvec()]
  out=least_squares(residual,np.clip(seed,limits[:,0]+.050001,limits[:,1]-.050001),bounds=(limits[:,0]+.050001,limits[:,1]-.050001),max_nfev=90,ftol=1e-7,xtol=1e-7,gtol=1e-7)
  r=residual(out.x)
  return out.x if np.linalg.norm(r[:3])<.0008 and np.linalg.norm(r[3:])<.0004 else None
 qtarget=robot.get_joint_positions().copy();qtarget[fingers]=[.05,-.05];qvelocity=np.zeros(6);velocity_limits=np.array([float(root.find(f"joint[@name='joint{i}']/limit").get('velocity')) for i in range(1,7)])
 for _ in range(24):world.step(render=True)
 expected_tcp=poses(robot.get_joint_positions())['tcp_link'];measured_tcp=matrix(*scene['tcp'].get_world_pose());fk_error=float(np.linalg.norm(expected_tcp[:3,3]-measured_tcp[:3,3]))
 if fk_error>.001 or Rotation.from_matrix(expected_tcp[:3,:3]@measured_tcp[:3,:3].T).magnitude()>.001:raise RuntimeError('FK_TCP_MISMATCH')
 export_cooked(stage,a.output/'cooked_initial.json');export=json.loads((a.output/'cooked_initial.json').read_text())
 allowed=[e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]
 if len(allowed)!=len(meta['interaction_geometry']['pieces']):raise RuntimeError('NATIVE_HANDLE_PIECE_ASSOCIATION:'+str((len(allowed),len(meta['interaction_geometry']['pieces']))))
 association={'allowed_pad_targets':allowed,'method':'explicit generated semantic handle collision piece identity; no contact projection','expected_piece_count':len(allowed)};(a.output/'association.json').write_text(json.dumps(association,indent=2))
 ownership=json.loads(a.ownership.read_text());envelope=audit_envelope(ROOT,export,ownership);(a.output/'ownership_envelope_audit.json').write_text(json.dumps(envelope,indent=2))
 if not envelope['passed']:
  world.pause();app.close();raise RuntimeError('OWNERSHIP_COOKED_EXTERIOR_MISMATCH')
 validator=NonpadClassifier(ROOT,export,ownership,allowed)
 phase='SETTLE';tick=0;rows=[];evaluation=[];mode='position';effort=0.;status='STARTED';legal=False;pull=False
 def sample():
  q=np.asarray(robot.get_joint_positions());P=poses(q)
  return {'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':float(np.min(np.minimum(q[arm]-limits[:,0],limits[:,1]-q[arm]))),'finger_world_poses':{n:P[n].tolist() for n in ['gripper_link1','gripper_link2']},'T_tcp':P['tcp_link'].tolist()}
 def sample_physics():
  state=sample()
  pL,qL=scene['door_contacts'].get_world_poses();state['diagnostic_target_world_pose_pre']=matrix(pL[0],qL[0]).tolist()
  return state
 native=NativeOwnershipReports(stage,[scene['contact_target_path']]+list(scene['diagnostic_contact_paths']),dt,allowed,world=world,sample_provider=sample_physics)
 video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(a.output/'probe.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
 def step():
  nonlocal tick
  if datetime.now(ZoneInfo('Asia/Shanghai'))>=deadline:raise RuntimeError('CUTOFF_05_00')
  native.clear();controller.apply_action(ArticulationAction(joint_positions=qtarget[arm],joint_velocities=qvelocity,joint_indices=arm))
  controller.apply_action(ArticulationAction(joint_positions=qtarget[fingers],joint_indices=fingers) if mode=='position' else ArticulationAction(joint_efforts=np.array([-effort,effort]),joint_indices=fingers))
  begin=len(native.physics_steps);world.step(render=False,update_fabric=True)
  if tick%8==0:world.render()
  state=sample();contacts=native.state()
  if abs(contacts['physics_window_dt_s']-dt)>1e-6:raise RuntimeError('CONTROL_PHYSICS_CLOCK_MISMATCH')
  state.update(t=tick*dt,ownership=contacts,control_mode=mode,closing_effort_n=effort,command_q_arm=qtarget[arm].tolist(),command_velocity_arm=qvelocity.tolist(),max_joint_tracking_error_rad=float(np.max(np.abs(np.asarray(state['q'])[arm]-qtarget[arm]))));rows.append(state)
  # Simulator truth is isolated in this evaluation collector.
  pL,qL=scene['door_contacts'].get_world_poses();L=matrix(pL[0],qL[0]);evaluation.append({'t':state['t'],'joint_rad':float(scene['articulation'].get_joint_positions()[0]),'T_moving_link':L.tolist()})
  if tick%8==0:
   image=np.asarray(scene['overview'][0].get_rgba())[:,:,:3].copy();cv2.rectangle(image,(0,0),(1280,70),(10,10,10),-1);force=list(contacts['pad_forces_n'].values());cv2.putText(image,f'SENSOR-ONLY INTERACTION | {phase} | pad={force} | nonpad reports={contacts["metal_contacts"]}',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1);cv2.putText(image,f'aperture={state["aperture_m"]*1000:.1f} mm | margin={state["margin_rad"]:.3f} rad | no GT motion commands',(12,57),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1);video.stdin.write(np.ascontiguousarray(image).tobytes())
  tick+=1
  if state['margin_rad']<=.05:raise RuntimeError('LOW_JOINT_MARGIN')
  for substep in native.physics_steps[begin:]:
   classification=validator.classify(substep['contacts'],substep['finger_world_poses'],np.asarray(substep['diagnostic_target_world_pose_pre']),substep['physics_dt_s'])
   substep['nonpad_classification']=classification
   state['nonpad_classification']=classification
   if substep['ownership']['pad_target_violations']:raise RuntimeError('WRONG_PAD_TARGET_CONTACT')
   if classification['confirmed_forbidden']:raise RuntimeError(classification['events'][-1]['classification'])
   if a.diagnose_first_nonpad and classification['events']:raise RuntimeError('NONPAD_IMPULSE_PAUSE_FOR_DISTANCE_CLASSIFICATION')
  if phase in ('PREGRASP','APPROACH') and any(c['force_n']>0 for c in contacts['contacts']):raise RuntimeError('APPROACH_CONTACT')
  if phase not in ('SETTLE','EXPORT'):
   forces={n:float(np.linalg.norm(v.get_contact_force_matrix(dt=contacts['physics_window_dt_s']))) for n,v in scene['scene_monitor_views']};body=sum(float(np.linalg.norm(v.get_contact_force_matrix(dt=contacts['physics_window_dt_s']))) for v in scene['body_views'])
   state['scene_contact_by_robot_link_n']=forces;state['palm_wrist_force_n']=body
   if max(forces.values(),default=0)>0 or body>0:raise RuntimeError('ROBOT_SCENE_OR_NONPAD_CONTACT')
  return state
 def move(q,duration):
  nonlocal qvelocity
  old=qtarget[arm].copy()
  delta=np.asarray(q)-old;duration=max(duration,float(np.max(1.5*np.abs(delta)/(velocity_limits/3))))
  for f in np.linspace(0,1,max(2,int(duration/dt))):qtarget[arm]=old+(f*f*(3-2*f))*delta;qvelocity=6*f*(1-f)*delta/duration;step()
  qvelocity=np.zeros(6)
 try:
  if a.export_only:
   phase='EXPORT'
   for _ in range(60):step()
   status='NATIVE_GEOMETRY_EXPORTED'
  elif chosen is None:raise RuntimeError('NO_PRIMITIVE_PLAN')
  else:
   phase='PREGRASP'
   for q in chosen['preplan'][1:]:move(q,1.5)
   phase='APPROACH'
   for q in chosen['approach']:move(q,.08)
   state=rows[-1];closure=JawCenteredClosure(np.asarray(state['T_tcp']),cal)
   def pad_centers(state):
    return [(np.asarray(state['finger_world_poses'][n])@np.r_[np.asarray(ownership['fingers'][n]['pad_vertices']).mean(0),1])[:3] for n in ('gripper_link1','gripper_link2')]
   def pad_forces(state):return [state['ownership']['pad_forces_n'][n] for n in ('gripper_link1','gripper_link2')]
   for _ in range(int(600/dt)):
    cmd=closure.update(np.asarray(state['T_tcp']),state['aperture_m'],pad_forces(state),pad_centers(state),dt);phase=cmd['state']
    if max(pad_forces(state))>cal['max_pad_load_n']:raise RuntimeError('LOW_PRELOAD_FORCE_LIMIT')
    if cmd['mode']=='position':qtarget[fingers]=[cmd['opening_m']/2,-cmd['opening_m']/2]
    else:
     if mode!='effort':
      kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True);mode='effort'
     effort=cmd['closing_effort_n']
    q=ik(cmd['T'],np.asarray(state['q'])[arm])
    if q is None:raise RuntimeError('CLOSURE_CENTERING_IK_FAILURE')
    qvelocity=np.clip((q-qtarget[arm])/dt,-velocity_limits/3,velocity_limits/3);qtarget[arm]=q;state=step()
    if closure.state=='FORCE_HOLD':legal=True;break
   if not legal:raise RuntimeError('BILATERAL_PRELOAD_NOT_ESTABLISHED')
   qvelocity=np.zeros(6);phase='FORCE_HOLD'
   for _ in range(120):
    if min(pad_forces(state))<.1:raise RuntimeError('HOLD_CONTACT_LOSS')
    cmd=closure.update(np.asarray(state['T_tcp']),state['aperture_m'],pad_forces(state),pad_centers(state),dt);effort=cmd['closing_effort_n'];state=step()
   T0=np.asarray(state['T_tcp']);probe=PadCompliantPull(T0,cal);phase='BLIND_PROBE'
   for _ in range(int(8/dt)):
    state=rows[-1]
    if min(pad_forces(state))<.1:raise RuntimeError('CONTACT_LOSS')
    if max(pad_forces(state))>cal['max_pad_load_n']:raise RuntimeError('LOW_PRELOAD_FORCE_LIMIT')
    effort=closure.update(np.asarray(state['T_tcp']),state['aperture_m'],pad_forces(state),pad_centers(state),dt)['closing_effort_n']
    T=probe.update(np.asarray(state['T_tcp']),pad_forces(state),pad_centers(state),dt);q=ik(T,np.asarray(state['q'])[arm])
    if q is None:raise RuntimeError('PROBE_IK_FAILURE')
    if np.max(np.abs(q-np.asarray(state['q'])[arm]))>.05:raise RuntimeError('IK_BRANCH_JUMP')
    qvelocity=np.clip((q-qtarget[arm])/dt,-velocity_limits/3,velocity_limits/3);qtarget[arm]=q;state=step();travel=float(np.linalg.norm(np.asarray(state['T_tcp'])[:3,3]-T0[:3,3]));pull=pull or travel>=.002
    if travel>=a.max_probe_mm/1000:break
   status='PROBE_RECORDED_PENDING_EVALUATION'
 except BaseException as e:
  import traceback
  status=str(e);print(traceback.format_exc(),flush=True)
 finally:
  world.pause();export_cooked(stage,a.output/'cooked_final.json');(a.output/'observations.json').write_text(json.dumps(rows));(a.output/'physics_steps.json').write_text(json.dumps(native.physics_steps));(a.output/'evaluation_gt.json').write_text(json.dumps(evaluation));video.stdin.close();video.wait(timeout=30)
  report={'status':status,'asset_id':meta['asset_id'],'base_fixed':base,'selected':chosen,'legal_preload_native_only_pending_offline_audit':legal,'ee_reached_2mm':pull,'minimum_joint_margin_rad':min((r['margin_rad'] for r in rows),default=None),'metal_contact_samples':sum(r['ownership']['metal_contacts']>0 for r in rows),'controller_information':['robot q/FK','left/right pad load','pad centers from official robot geometry','initial handle primitive pose'],'GT_scope':'evaluation_gt.json only; no GT joint type/axis/angle in controller','calibration':cal,'probe_geometry':'separate PCA semantic bar/support proxies; official robot and visual geometry unchanged','closure':'slow close -> pause unilateral -> geometry-directed centering -> bilateral low preload; no fixed aperture','wrist_wrench_used':False,'raw_triangle_policy':'diagnostic only; no penetration threshold used'}
  report.update(deadline_shanghai=deadline.isoformat(),fk_tcp_position_error_m=fk_error,physics_dt_s=dt,video_fps=30,arm_gains_unchanged=[10000,400],force_hold_finger_kp=0,finger_kd_unchanged=40,friction_unchanged=True,object_actuation='none; initial closed reset in inherited scene setup only')
  if evaluation:report['actual_joint_displacement_rad']=evaluation[-1]['joint_rad']-evaluation[0]['joint_rad']
  report['nonpad_events']=validator.events
  report['nonpad_impulse_samples']=report['metal_contact_samples']
  report['metal_contact_samples_scope']='native non-pad impulse count; NOT confirmed illegal contact count'
  report['representation_warning_events']=sum(e['classification']=='PROXIMITY_OR_REPRESENTATION_WARNING' for e in validator.events)
  report['confirmed_forbidden_events']=sum(e['classification'].startswith('CONFIRMED_FORBIDDEN') for e in validator.events)
  report['ownership_envelope_audit']=envelope
  (a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='selected'},indent=2),flush=True);app.close()
if __name__=='__main__':main()
