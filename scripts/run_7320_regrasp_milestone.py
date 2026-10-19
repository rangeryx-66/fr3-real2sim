"""7320 only: the existing two-station contact sequence, with persistent failures.
Known-model diagnostic. Real closure/contact; no object state commands.
"""
from piper_mobile_execute import *
from wrist_reconstruction.planner import camera_clearance
from wrist_reconstruction.geometry import calibration
import time
from types import SimpleNamespace
p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);args=p.parse_args();job=json.loads(args.job.read_text());whole=json.loads(Path(job['whole_task_plan']).read_text())
a=SimpleNamespace(source=Path(job['source']),asset_root=Path(job['asset_root']),plan=Path(job['plan']),output=Path(job['output']),gpu=job['gpu'],trial=True,candidate=0,no_operation=False,deadline_shanghai=job['deadline_shanghai'],ownership=None)
a.output.mkdir(parents=True,exist_ok=True)
kind=job['skill']['joint_type'];base_moves=0;regrasp_count=0;initial_base=list(whole['base']);max_state=0.;first_divergence=None;command_state=0.;planned_reference=None

source=json.loads((a.source/'report.json').read_text());base=source['robot_base_pose'];plan=json.loads(a.plan.read_text());chosen=plan['trial_candidates'][0]
if base!=chosen['base']:raise RuntimeError('BASE_CHANGED')
a.ownership=None;from articulated_interaction_skill.scene import install
install()
from interactive_twin_refinement.assembly import install_setup_capture
install_setup_capture()
from interactive_twin.plant import bootstrap_job
scene=bootstrap_job(a,base,job);world=scene['world'];app=scene['app'];robot=scene['robot'];stage=scene['stage'];arm=scene['arm'];fingers=scene['fingers'];names=scene['names'];controller=scene['controller'];dt=scene['DT']
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from piper_mobile_demo.model import Model
from piper_mobile_demo.cooked_geometry import export_cooked
from articulated_interaction.physical_baseline import PhysicalScene,WholeFingerReports,FINGERS
from articulated_interaction.control import JawCenteredClosure
from articulated_demo.kinematics import transform
import cv2
deadline=datetime.fromisoformat(a.deadline_shanghai)
model=Model(ROOT/'config/piper.urdf',a.asset_root,source);model.deadline_timestamp=deadline.timestamp()
export_cooked(stage,a.output/'cooked_initial.json');export=json.loads((a.output/'cooked_initial.json').read_text())
allowed=[e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]
if len(allowed)!=len(model.manifest['interaction_geometry']['pieces']):raise RuntimeError('FROZEN_PROXY_ASSOCIATION_FAILED')
collision=PhysicalScene(export,model,allowed)
meta=model.manifest['interaction_geometry']['selection'];normal=model.asset_T[:3,:3]@np.asarray(meta['outward_normal_root']);axis=model.asset_T[:3,:3]@np.asarray(meta['axis_root'])
section=min(meta['sections'],key=lambda s:abs(s['fraction']));anchor_world=(model.asset_T@np.r_[section['anchor_root_m'],1])[:3]
moving_initial=collision.moving_reference;anchor_link=(np.linalg.inv(moving_initial)@np.r_[anchor_world,1])[:3]
# Freeze and record the existing approximation; no proxy regeneration.
fingerprint={'manifest_sha256':hashlib.sha256((a.asset_root/'manifest.json').read_bytes()).hexdigest(),'asset_urdf_sha256':hashlib.sha256(model.asset_urdf.read_bytes()).hexdigest(),'geometry':'existing bar box plus two support primitives; approximate, not calibrated real geometry','selection':meta}
(a.output/'frozen_proxy.json').write_text(json.dumps(fingerprint,indent=2))
# At most 12 local changes in this same bar-side-pinch family.
variants=[(0,0,0)]
slide,depth,roll=variants[a.candidate];T=np.asarray(chosen['T']);T[:3,3]+=slide*axis+depth*T[:3,2];T[:3,:3]=Rotation.from_rotvec(axis*np.deg2rad(roll)).as_matrix()@T[:3,:3]
actual_views=dict(scene['scene_monitor_views']);B=transform(base[:3],[0,0,np.deg2rad(base[3])]);chain=model.chain
positions=dict(zip(names,robot.get_joint_positions()));offset=np.linalg.inv(chain.root_to_link('gripper_base',positions))@chain.root_to_link('tcp_link',positions)
def moving():
 p,q=scene['door_contacts'].get_world_poses();return matrix(p[0],q[0])
def tcp():
 p,q=actual_views['gripper_base'].get_world_poses();return matrix(p[0],q[0])@offset
def poses(q):return model.poses(np.asarray(q)[arm],base,finger_q=np.asarray(q)[fingers])
qtarget=np.asarray(robot.get_joint_positions(),dtype=float).copy();qtarget[fingers]=[.05,-.05];qvelocity=np.zeros(6);phase='SETTLE';rows=[];tick=0;reference=None;legal=False;success=False;pull2=False;status='STARTED';loss_s=0.;slip_s=0.;preflight={}
urdf=ET.parse(ROOT/'config/piper.urdf').getroot();vel=np.array([float(urdf.find(f"joint[@name='joint{i}']/limit").get('velocity')) for i in range(1,7)]);limits=model.limits
from wrist_reconstruction.force_policy import TemporalForceGuard
force_guard=TemporalForceGuard(job['wrist_experiment']['force_policy'])
policy=json.loads((ROOT/'config/semantic_interaction.json').read_text());policy['slow_closure_m_s']=.001;mode='position';effort=0.
near=scene['overview'][1];eye=anchor_world+normal*.45+np.array([0,0,.20]);focus=anchor_world+np.array([0,0,.02]);forward=(focus-eye)/np.linalg.norm(focus-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward));near.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros');near.set_clipping_range(.02,3.)
video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(a.output/'contact_baseline.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
def sample():
 q=np.asarray(robot.get_joint_positions());P=poses(q);D=moving();E=tcp();rel=np.linalg.inv(D)@E
 return {'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':model.margin(q[arm]),'finger_world_poses':{n:P[n].tolist() for n in FINGERS},'T_tcp':E.tolist(),'T_moving_link':D.tolist(),'relative_translation_slip_m':0. if reference is None else float(np.linalg.norm(rel[:3,3]-reference[:3,3])),'relative_rotation_slip_deg':0. if reference is None else float(np.rad2deg(Rotation.from_matrix(rel[:3,:3]@reference[:3,:3].T).magnitude())),'door_angle_deg':float(np.rad2deg(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])) if kind=='revolute' else float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])}
native=WholeFingerReports(stage,export,allowed,world,dt,sample)
def window(seconds=.3):return rows[-max(1,int(seconds/dt)):]
def filtered():return np.mean([list(s['forces_n'].values()) for s in window(.08)],axis=0) if rows else np.zeros(2)
def grip_window():
 w=window(.5);loads=np.array([list(s['forces_n'].values()) for s in w]);opening=np.array([s['aperture_m'] for s in w]);q=np.asarray(robot.get_joint_positions());P=poses(q);centers=np.array([(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]);center=(moving()@np.r_[anchor_link,1])[:3];axis_jaw=centers[0]-centers[1];axis_jaw/=np.linalg.norm(axis_jaw);interval=np.sort(centers@axis_jaw);between=interval[0]<=center@axis_jaw<=interval[1]
 good=len(w)>=int(.45/dt) and np.all(np.mean(loads>.05,axis=0)>=.6) and np.all(np.mean(loads,axis=0)>=.1) and np.ptp(opening)<.0005
 return good,{'between_jaws':bool(between),'force_duty':np.mean(loads>.05,axis=0).tolist(),'mean_forces_n':np.mean(loads,axis=0).tolist(),'aperture_range_m':float(np.ptp(opening))}
def step():
 global tick,loss_s,slip_s,max_state,first_divergence
 if datetime.now(ZoneInfo('Asia/Shanghai'))>=deadline:raise RuntimeError('CUTOFF_05_00')
 native.clear();controller.apply_action(ArticulationAction(joint_positions=qtarget[arm],joint_velocities=qvelocity,joint_indices=arm))
 controller.apply_action(ArticulationAction(joint_positions=qtarget[fingers],joint_efforts=np.zeros(2),joint_indices=fingers) if mode=='position' else ArticulationAction(joint_efforts=np.array([-effort,effort]),joint_indices=fingers))
 world.step(render=False,update_fabric=True)
 if tick%8==0:world.render()
 s=sample();s.update(base=list(base),base_moves=base_moves,regrasp_count=regrasp_count);contacts=native.state();s.update(t=tick*dt,forces_n=contacts['finger_handle_forces_n'],contacts=contacts['contacts'],mode=mode,closing_effort_n=effort,command_q_arm=qtarget[arm].tolist());s['force_event']=force_guard.update(s['forces_n'],dt,s['t']);s['planned_articulation_state']=command_state;s['planned_T_tcp']=None if planned_reference is None else planned_reference.tolist();rows.append(s);tick+=1
 max_state=max(max_state,s['door_angle_deg'])
 if planned_reference is not None and first_divergence is None:
  error=float(np.linalg.norm(np.asarray(s['T_tcp'])[:3,3]-planned_reference[:3,3]));state_error=abs(s['door_angle_deg']-(np.rad2deg(command_state) if kind=='revolute' else command_state))
  if error>.003 or state_error>(2. if kind=='revolute' else .003):first_divergence={'t':s['t'],'phase':phase,'planned_state':command_state,'actual_state':s['door_angle_deg'],'TCP_position_error_m':error,'definition':'diagnostic first >3mm TCP or >2deg/3mm object-state discrepancy; never a manipulation veto'}
 if abs(contacts['physics_window_dt_s']-dt)>1e-6:raise RuntimeError('PHYSICS_CLOCK_MISMATCH')
 if s['margin_rad']<=.05:raise RuntimeError('LOW_JOINT_MARGIN')
 P=poses(np.asarray(s['q']));ok,why=collision.contact_guard(s['contacts'],P)
 if not ok:raise RuntimeError(why)
 if tick%8==0:
  ok,why=collision.check(P,np.asarray(s['T_moving_link']),allow_handle=phase not in ('SETTLE','PREGRASP','APPROACH','NO_OPERATION','CLEARANCE_RETREAT','BASE_ROUTE','SECOND_APPROACH'))
  if not ok:raise RuntimeError(why)
  image=np.asarray(near.get_rgba())[:,:,:3].copy();overview=np.asarray(scene['overview'][0].get_rgba())[:,:,:3];image[-240:,-320:]=cv2.resize(overview,(320,240));cv2.rectangle(image,(0,0),(1280,80),(12,12,12),-1)
  cv2.putText(image,f'KNOWN_MODEL_DIAGNOSTIC / PHYSICAL CONTACT | {phase}',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1);cv2.putText(image,f'door={s["door_angle_deg"]:.2f} deg | slip={s["relative_translation_slip_m"]*1000:.2f} mm | margin={s["margin_rad"]:.3f}',(12,62),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1);video.stdin.write(np.ascontiguousarray(image).tobytes())
 if tick%240==0:
  with (a.output/'live.jsonl').open('a') as live:live.write(json.dumps({k:s[k] for k in ('t','phase','door_angle_deg','aperture_m','forces_n','base','base_moves','regrasp_count','margin_rad')})+'\n')
  (a.output/'progress.json').write_text(json.dumps({'pid':os.getpid(),'base_moves':base_moves,'regrasp_count':regrasp_count,'base':base,'phase':phase,'t':s['t'],'door_angle_deg':s['door_angle_deg'],'aperture_m':s['aperture_m'],'forces_n':s['forces_n'],'margin_rad':s['margin_rad'],'relative_translation_slip_m':s['relative_translation_slip_m']},indent=2))
  cv2.imwrite(str(a.output/'latest_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR))
 if s['force_event']['status'].startswith('HARD_FORCE_STOP'):raise RuntimeError(s['force_event']['status'])
 if reference is not None:
  if np.any(abs(np.asarray(robot.get_joint_velocities())[arm])>vel/3):raise RuntimeError('EXISTING_ARM_SPEED_LIMIT')
  if len(rows)>1 and np.linalg.norm(np.asarray(rows[-1]['T_tcp'])[:3,3]-np.asarray(rows[-2]['T_tcp'])[:3,3])/dt>.005:raise RuntimeError('PROBE_CARTESIAN_SPEED_LIMIT')
  loss_s=loss_s+dt if np.min(filtered())<.05 else 0.;slip_s=slip_s+dt if s['relative_translation_slip_m']>policy['max_slip_m'] else 0.
  if loss_s>.15:
   s['grasp_event']='GRASP_LOAD_ASYMMETRY';s['effort_valid']=False
   if np.max(filtered())<.05:raise RuntimeError('CONTACT_LOST_PAUSE_IN_SAME_SCENE')
  if slip_s>.1:s['relative_motion_event']='GRASP_RELATIVE_MOTION' # diagnostic only
 return s
def move(q,duration):
 global qvelocity
 old=qtarget[arm].copy();delta=np.asarray(q)-old;duration=max(duration,float(np.max(1.5*np.abs(delta)/(vel/3))))
 for f in np.linspace(0,1,max(2,int(duration/dt))):qtarget[arm]=old+f*f*(3-2*f)*delta;qvelocity=6*f*(1-f)*delta/duration;step()
 qvelocity=np.zeros(6)
def ik(target,seed):return model.ik(target,base,seed,starts=1)
def hold(seconds):
 for _ in range(int(seconds/dt)):step()
try:
 phase='SETTLE';hold(.5);initial_angle=rows[-1]['door_angle_deg']
 if a.no_operation:phase='NO_OPERATION';hold(a.control_duration);status='NO_OPERATION_CONTROL_COMPLETE'
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
  qvelocity=np.zeros(6);phase='FORCE_HOLD';reference=np.linalg.inv(moving())@tcp();hold(.5)
  E0=tcp();D0=moving();theta0=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]);seed=np.asarray(robot.get_joint_positions())[arm];command_bias=qtarget[arm]-seed
  (a.output/('command_bias_'+str(regrasp_count)+'.json' if 'regrasp_count' in locals() else 'command_bias.json')).write_text(json.dumps({'q_measured':seed.tolist(),'command_q':qtarget[arm].tolist(),'bias':command_bias.tolist(),'purpose':'retain the loaded position-command equilibrium at path transition; unchanged gains and jaw preload'},indent=2))
  origin=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta0});arc=[]
  # Reuse the saved path, aligning the grasp after real physical closure.
  for saved in whole['path']:
   s=max(theta0,float(saved['state']));body=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:s});D=body@np.linalg.inv(origin)
   target=D@E0;q=ik(target,seed)
   if q is None:raise RuntimeError('ALIGNED_WHOLE_PATH_NO_IK')
   arc.append({'q':q.tolist(),'state':s,'T_tcp':target.tolist(),'margin_rad':model.margin(q)});seed=q
  preflight={'known_model_whole_path':arc,'min_joint_margin_rad':min(x['margin_rad'] for x in arc),'planning_goal':whole['goal_state'],'GT_planning':True,'end_retreat':whole['end_retreat']};(a.output/'aligned_whole_path.json').write_text(json.dumps(preflight,indent=2))
  previous=E0.copy()
  for waypoint in arc:
   phase='OPEN_1' if regrasp_count==0 else 'OPEN_2';command_state=waypoint['state'];planned_reference=np.asarray(waypoint['T_tcp'])
   distance=np.linalg.norm(planned_reference[:3,3]-previous[:3,3]);duration=max(.25,1.5*distance/.003)
   if force_guard.pending:
    phase='FORCE_HOLD';hold(.25)
    if not force_guard.settled():raise RuntimeError('HARD_FORCE_STOP_SUSTAINED')
    force_guard.pending=False;duration*=2
   command=np.asarray(waypoint['q'])+command_bias
   if model.margin(command)<=.05:raise RuntimeError('COMMAND_MARGIN_BELOW_EXISTING_LIMIT')
   move(command,duration);previous=planned_reference.copy()
  # Execute the already-saved switch; retain the same physical scene.
  phase='SWITCH_SAFE_HOLD';qvelocity=np.zeros(6);hold(2.)
  switch_actual=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
  print('STATE_DIAGNOSTIC',phase,rows[-1]['door_angle_deg'],flush=True)
  reference=None;planned_reference=None;phase='RELEASE';effort=0.;mode='position'
  kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kp[fingers]=1000.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True)
  opening=float(whole['switch_retreat']['opening_m']);present=float(rows[-1]['aperture_m'])
  # Release slowly; no stale closed-position command replaces the preload hold.
  for w in np.linspace(present,opening,max(2,int(abs(opening-present)/(.001*dt))+1)):
   qtarget[fingers]=[w/2,-w/2];step()
  hold(.3)
  (a.output/'release_measurement.json').write_text(json.dumps({'target_m':opening,'actual_m':rows[-1]['aperture_m'],'error_m':rows[-1]['aperture_m']-opening,'pad_loads_n':filtered().tolist(),'classification':'DIAGNOSTIC','closing_effort_explicitly_cleared':True},indent=2))
  if np.any(filtered()>.05):hold(1.)
  if np.any(filtered()>.05):raise RuntimeError('RELEASE_CONTACT_REMAINS_PAUSE_IN_SAME_SCENE')
  phase='CLEARANCE_RETREAT'
  for q in whole['switch_retreat']['q_path'][1:]:
   distance=np.linalg.norm(model.poses(q,base,width=opening)['tcp_link'][:3,3]-tcp()[:3,3]);move(q,max(.25,1.5*distance/.003))
  hold(.3)
  released_state=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
  print('STATE_DIAGNOSTIC',phase,rows[-1]['door_angle_deg'],flush=True)
  from isaacsim.core.prims import SingleXFormPrim
  from interactive_twin_recovery.mobile import scene_at
  support=SingleXFormPrim('/World/r1a7_pedestal');chassis=SingleXFormPrim('/World/mobile_chassis');phase='BASE_ROUTE'
  for target_base in whole['base_route']['waypoints'][1:]:
   old_base=np.asarray(base).copy();delta_base=np.asarray(target_base)-old_base;duration=max(np.linalg.norm(delta_base[:2])/.01,abs(delta_base[3])/5.,dt)
   for f in np.linspace(0.,1.,max(2,int(duration/dt)))[1:]:
    pose=old_base+f*delta_base;quat=np.roll(Rotation.from_euler('z',pose[3],degrees=True).as_quat(),1)
    # Existing kinematic SE(2) platform, only after verified release/clearance.
    robot.set_world_pose(pose[:3],quat);support.set_world_pose([pose[0],pose[1],-.33],quat);chassis.set_world_pose([pose[0],pose[1],-.66],quat)
    base[:]=pose.tolist();B=transform(base[:3],[0,0,np.deg2rad(base[3])]);
    if tick%8==0:collision=scene_at(ROOT,export,model,initial_base,base)
    step()
  base_moves+=1;hold(.3)
  now=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
  print('STATE_DIAGNOSTIC',phase,rows[-1]['door_angle_deg'],flush=True)
  phase='SECOND_APPROACH'
  for q in whole['second_station']['preplan'][1:]:move(q,1.5)
  for q in whole['second_station']['approach'][1:]:move(q,.15)
  # Reclose with the original bilateral verification and same preload command.
  legal=False;loss_s=0.;slip_s=0.
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
  qvelocity=np.zeros(6);phase='FORCE_HOLD';reference=np.linalg.inv(moving())@tcp();hold(.5)
  regrasp_count+=1;regrasp_angle=rows[-1]['door_angle_deg']
  (a.output/'regrasp_contact.json').write_text(json.dumps({'angle_deg':regrasp_angle,'pad_loads_n':filtered().tolist(),'contact_window':detail,'base':base,'physical_closure':True},indent=2))
  E0=tcp();D0=moving();theta0=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]);seed=np.asarray(robot.get_joint_positions())[arm];command_bias=qtarget[arm]-seed
  (a.output/('command_bias_'+str(regrasp_count)+'.json' if 'regrasp_count' in locals() else 'command_bias.json')).write_text(json.dumps({'q_measured':seed.tolist(),'command_q':qtarget[arm].tolist(),'bias':command_bias.tolist(),'purpose':'retain the loaded position-command equilibrium at path transition; unchanged gains and jaw preload'},indent=2))
  origin=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta0});arc=[]
  # Reuse the saved path, aligning the grasp after real physical closure.
  for saved in whole['second_station']['path']:
   if float(saved['state'])>theta0+np.deg2rad(12.):break
   s=max(theta0,float(saved['state']));body=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:s});D=body@np.linalg.inv(origin)
   target=D@E0;q=ik(target,seed)
   if q is None:raise RuntimeError('ALIGNED_WHOLE_PATH_NO_IK')
   arc.append({'q':q.tolist(),'state':s,'T_tcp':target.tolist(),'margin_rad':model.margin(q)});seed=q
  preflight={'known_model_whole_path':arc,'min_joint_margin_rad':min(x['margin_rad'] for x in arc),'planning_goal':whole['goal_state'],'GT_planning':True,'end_retreat':whole['end_retreat']};(a.output/'aligned_second_leg.json').write_text(json.dumps(preflight,indent=2))
  previous=E0.copy()
  for waypoint in arc:
   phase='OPEN_1' if regrasp_count==0 else 'OPEN_2';command_state=waypoint['state'];planned_reference=np.asarray(waypoint['T_tcp'])
   distance=np.linalg.norm(planned_reference[:3,3]-previous[:3,3]);duration=max(.25,1.5*distance/.003)
   if force_guard.pending:
    phase='FORCE_HOLD';hold(.25)
    if not force_guard.settled():raise RuntimeError('HARD_FORCE_STOP_SUSTAINED')
    force_guard.pending=False;duration*=2
   command=np.asarray(waypoint['q'])+command_bias
   if model.margin(command)<=.05:raise RuntimeError('COMMAND_MARGIN_BELOW_EXISTING_LIMIT')
   move(command,duration);previous=planned_reference.copy()
  # Smooth spline ends at zero arm velocity. Gripper effort/gains are retained.
  phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(2.);ready,detail=grip_window()
  success=bool(np.rad2deg(switch_actual)>=40. and base_moves==1 and regrasp_count==1 and rows[-1]['door_angle_deg']-regrasp_angle>=10.);status='SUCCESS_PHYSICAL_REGRASP_7320' if success else 'MILESTONE_NOT_YET_REACHED'
  if not success:raise RuntimeError(status)

except BaseException as error:
 import traceback
 status=str(error) if isinstance(error,RuntimeError) else 'IMPLEMENTATION_ERROR:'+str(error);print(traceback.format_exc(),flush=True)
 world.pause();world.render()
 (a.output/'paused.json').write_text(json.dumps({'status':status,'phase':phase,'pid':os.getpid(),'max_angle_deg':max_state,'base_moves':base_moves,'regrasp_count':regrasp_count,'scene_preserved':True},indent=2))
 (a.output/'observations.json').write_text(json.dumps(rows))
 while app.is_running() and not success:
  app.update();time.sleep(.1)
  continuation=a.output/'continue_same_scene.py'
  if continuation.exists():
   command=continuation.read_text();continuation.rename(a.output/f'continuation_{time.time_ns()}.py')
   try:
    world.play();exec(compile(command,str(continuation),'exec'),globals())
   except Exception:
    world.pause();print(traceback.format_exc(),flush=True)
    (a.output/'paused.json').write_text(json.dumps({'status':traceback.format_exc(),'phase':phase,'pid':os.getpid(),'max_angle_deg':max_state,'base_moves':base_moves,'regrasp_count':regrasp_count,'scene_preserved':True},indent=2))
    (a.output/'observations.json').write_text(json.dumps(rows))

finally:
 world.pause();world.render();cv2.imwrite(str(a.output/'final_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR));video.stdin.close();video.wait(timeout=30);export_cooked(stage,a.output/'cooked_final.json');(a.output/'observations.json').write_text(json.dumps(rows));(a.output/'physics_steps.json').write_text(json.dumps(native.physics_steps))
 report={'status':status,'success':success,'mode':'KNOWN_MODEL_DIAGNOSTIC','GT_planning':True,'GT_collision_scene':True,'unknown_structure_identification_success':False,'autonomous_reconstruction_success':False,'maximum_actual_state':max_state,'actual_units':'degrees' if kind=='revolute' else 'meters','first_planned_actual_divergence':first_divergence,'whole_task_plan':job['whole_task_plan'],'grip_control_continuous':True,'loaded_arm_command_continuous':True,'end_release_retreat_planned':True,'base_prepositioned_before_grasp':True,'candidate':a.candidate,'local_variant':{'slide_m':slide,'depth_m':depth,'roll_deg':roll},'T_grasp':T.tolist(),'base_fixed':initial_base,'final_base':base,'base_moves':base_moves,'regrasp_count':regrasp_count,'regrasp_angle_deg':globals().get('regrasp_angle'),'before_release_angle_deg':float(np.rad2deg(switch_actual)) if 'switch_actual' in globals() else None,'bounded_station_switches':1,'official_unsplit_finger_counts':{n:sum(s.body==n for s in collision.robot) for n in FINGERS},'ownership_gate_used':False,'raw_triangle_execution_veto':False,'proxy_approximate_and_frozen':True,'finger_effort_limit_n':10.,'preload_target_n':policy['preload_n'],'friction_unchanged':True,'object_actuation':'none during execution; loader closed reset only','attachments':False,'bilateral_hold_established':legal,'actual_2mm_tcp_motion_completed':pull2,'preflight':preflight,'initial_door_angle_deg':rows[0]['door_angle_deg'] if rows else None,'final_door_angle_deg':rows[-1]['door_angle_deg'] if rows else None,'actual_door_displacement_deg':rows[-1]['door_angle_deg']-rows[0]['door_angle_deg'] if rows else None,'peak_finger_handle_force_n':max((max(s['forces_n'].values()) for s in rows),default=0.),'minimum_joint_margin_rad':min((s['margin_rad'] for s in rows),default=None),'maximum_relative_translation_slip_m':max((s['relative_translation_slip_m'] for s in rows),default=None),'final_relative_translation_slip_m':rows[-1]['relative_translation_slip_m'] if rows else None,'duration_s':len(rows)*dt,'first_failure_state':rows[-1] if rows and not success and not a.no_operation else None,'video':'contact_baseline.mp4','physics_dt_s':dt,'render_fps':30,'no_operation':a.no_operation,'deadline_shanghai':a.deadline_shanghai};(a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('first_failure_state','preflight')},indent=2),flush=True);app.close()
