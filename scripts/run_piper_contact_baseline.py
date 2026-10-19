"""Known-model physical-contact baseline; bounded search, repetitions and control.

Door drives/forces/attachments are never commanded during execution.
"""
from piper_mobile_execute import *

def parser():
 p=argparse.ArgumentParser(description=__doc__)
 for k in ('source','asset-root','plan','output'):p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--gpu',type=int,default=6);p.add_argument('--max-candidates',type=int,default=12);p.add_argument('--repeat',type=int,default=3)
 p.add_argument('--trial',action='store_true');p.add_argument('--candidate',type=int,default=0);p.add_argument('--no-operation',action='store_true');p.add_argument('--control-duration',type=float,default=100.)
 p.add_argument('--deadline-shanghai',default='2026-10-03T05:00:00+08:00')
 return p

def main():
 a=parser().parse_args();a.output.mkdir(parents=True,exist_ok=True)
 if not a.trial:
  runs=[];success=None
  def run(index,name,control=False,duration=100):
   out=a.output/name;cmd=[sys.executable,__file__,'--trial','--source',str(a.source),'--asset-root',str(a.asset_root),'--plan',str(a.plan),'--output',str(out),'--candidate',str(index),'--gpu',str(a.gpu),'--deadline-shanghai',a.deadline_shanghai]
   if control:cmd+=['--no-operation','--control-duration',str(duration)]
   with open(a.output/(name+'.log'),'w') as f:ret=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
   r=json.loads((out/'report.json').read_text()) if (out/'report.json').exists() else {'status':'PROCESS_FAILED','exit_code':ret.returncode,'success':False}
   runs.append({'name':name,'candidate':index,'report':r});print(name,r.get('status'),flush=True)
   (a.output/'summary.json').write_text(json.dumps({'mode':'known-model physical-contact baseline','runs':runs,'successes':sum(x['report'].get('success',False) for x in runs),'complete':False},indent=2))
   return r
  for index in range(min(12,a.max_candidates)):
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
  d=json.loads((a.output/'summary.json').read_text());d.update(complete=True,first_success_candidate=None if not success else success[0],attempted_grasp_candidates=sum(x['name'].startswith('candidate') for x in runs),actual_contact_success_count=sum(x['report'].get('success',False) for x in runs if x['name']!='no_operation'),grasp_run_count=sum(x['name']!='no_operation' for x in runs));(a.output/'summary.json').write_text(json.dumps(d,indent=2));return
 source=json.loads((a.source/'report.json').read_text());base=source['robot_base_pose'];plan=json.loads(a.plan.read_text());chosen=plan['trial_candidates'][0]
 if base!=chosen['base']:raise RuntimeError('BASE_CHANGED')
 a.ownership=None;scene=bootstrap(a,base);world=scene['world'];app=scene['app'];robot=scene['robot'];stage=scene['stage'];arm=scene['arm'];fingers=scene['fingers'];names=scene['names'];controller=scene['controller'];dt=scene['DT']
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
 variants=[(0,0,0),(0,-.004,0),(0,.004,0),(-.01,0,0),(.01,0,0),(0,0,-5),(0,0,5),(-.01,-.004,0),(.01,-.004,0),(-.01,.004,0),(.01,.004,0),(0,-.004,5)]
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
 policy=json.loads((ROOT/'config/semantic_interaction.json').read_text());policy['slow_closure_m_s']=.001;mode='position';effort=0.
 near=scene['overview'][1];eye=anchor_world+normal*.45+np.array([0,0,.20]);focus=anchor_world+np.array([0,0,.02]);forward=(focus-eye)/np.linalg.norm(focus-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward));near.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros');near.set_clipping_range(.02,3.)
 video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(a.output/'contact_baseline.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
 def sample():
  q=np.asarray(robot.get_joint_positions());P=poses(q);D=moving();E=tcp();rel=np.linalg.inv(D)@E
  return {'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':model.margin(q[arm]),'finger_world_poses':{n:P[n].tolist() for n in FINGERS},'T_tcp':E.tolist(),'T_moving_link':D.tolist(),'relative_translation_slip_m':0. if reference is None else float(np.linalg.norm(rel[:3,3]-reference[:3,3])),'relative_rotation_slip_deg':0. if reference is None else float(np.rad2deg(Rotation.from_matrix(rel[:3,:3]@reference[:3,:3].T).magnitude())),'door_angle_deg':float(np.rad2deg(scene['articulation'].get_joint_positions()[0]))}
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
  if tick%8==0:world.render()
  s=sample();contacts=native.state();s.update(t=tick*dt,forces_n=contacts['finger_handle_forces_n'],contacts=contacts['contacts'],mode=mode,closing_effort_n=effort,command_q_arm=qtarget[arm].tolist());rows.append(s);tick+=1
  if abs(contacts['physics_window_dt_s']-dt)>1e-6:raise RuntimeError('PHYSICS_CLOCK_MISMATCH')
  if s['margin_rad']<=.05:raise RuntimeError('LOW_JOINT_MARGIN')
  P=poses(np.asarray(s['q']));ok,why=collision.contact_guard(s['contacts'],P)
  if not ok:raise RuntimeError(why)
  if tick%8==0:
   ok,why=collision.check(P,np.asarray(s['T_moving_link']),allow_handle=phase not in ('SETTLE','PREGRASP','APPROACH','NO_OPERATION'))
   if not ok:raise RuntimeError(why)
   image=np.asarray(near.get_rgba())[:,:,:3].copy();overview=np.asarray(scene['overview'][0].get_rgba())[:,:,:3];image[-240:,-320:]=cv2.resize(overview,(320,240));cv2.rectangle(image,(0,0),(1280,80),(12,12,12),-1)
   cv2.putText(image,f'KNOWN-MODEL / PHYSICAL CONTACT | {phase}',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1);cv2.putText(image,f'door={s["door_angle_deg"]:.2f} deg | slip={s["relative_translation_slip_m"]*1000:.2f} mm | margin={s["margin_rad"]:.3f}',(12,62),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1);video.stdin.write(np.ascontiguousarray(image).tobytes())
  if tick%240==0:
   (a.output/'progress.json').write_text(json.dumps({'phase':phase,'t':s['t'],'door_angle_deg':s['door_angle_deg'],'aperture_m':s['aperture_m'],'forces_n':s['forces_n'],'margin_rad':s['margin_rad'],'relative_translation_slip_m':s['relative_translation_slip_m']},indent=2))
   cv2.imwrite(str(a.output/'latest_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR))
  if phase in ('CLOSE','SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD','SMALL_PULL','OPEN_5_DEG','FINAL_HOLD') and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']:raise RuntimeError('EXISTING_LOW_PRELOAD_FORCE_LIMIT')
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
   E0=tcp();D0=moving();theta0=float(scene['articulation'].get_joint_positions()[0]);theta_ref=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta0});seed=np.asarray(robot.get_joint_positions())[arm];arc=[]
   for angle in np.linspace(theta0,theta0+np.deg2rad(5.5),45)[1:]:
    target=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:float(angle)})@np.linalg.inv(theta_ref)@E0;q=ik(target,seed)
    if q is None:raise RuntimeError('OPENING_PREFLIGHT_NO_IK')
    modelD=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:float(angle)})@np.linalg.inv(theta_ref)@D0
    ok,why=collision.check(model.poses(q,base,finger_q=robot.get_joint_positions()[fingers]),modelD,True)
    if not ok:raise RuntimeError('OPENING_PREFLIGHT_'+why)
    arc.append({'q':q.tolist(),'reference_angle_rad':float(angle),'target_tcp':target.tolist()});seed=q
   preflight={'known_model_arc':arc,'min_joint_margin_rad':min(model.margin(np.asarray(x['q'])) for x in arc)};(a.output/'arc.json').write_text(json.dumps(preflight,indent=2))
   for waypoint in arc:
    phase='SMALL_PULL' if not pull2 else 'OPEN_5_DEG';move(waypoint['q'],.25)
    travel=float(np.linalg.norm(tcp()[:3,3]-E0[:3,3]))
    if not pull2 and travel>=.002:
     pull2=True;preflight['small_pull_actual_tcp_displacement_m']=travel;preflight['small_pull_actual_door_angle_deg']=rows[-1]['door_angle_deg'];phase='FORCE_HOLD';hold(.3)
    if rows[-1]['door_angle_deg']-initial_angle>=5.:break
   phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(2.);ready,detail=grip_window();success=ready and rows[-1]['door_angle_deg']-initial_angle>=5. and rows[-1]['relative_translation_slip_m']<=policy['max_slip_m'];status='SUCCESS' if success else 'ACTUAL_OPENING_OR_HOLD_CRITERION_NOT_MET'
 except BaseException as error:
  import traceback
  status=str(error) if isinstance(error,RuntimeError) else 'IMPLEMENTATION_ERROR:'+str(error);print(traceback.format_exc(),flush=True)
 finally:
  world.pause();world.render();cv2.imwrite(str(a.output/'final_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR));video.stdin.close();video.wait(timeout=30);export_cooked(stage,a.output/'cooked_final.json');(a.output/'observations.json').write_text(json.dumps(rows));(a.output/'physics_steps.json').write_text(json.dumps(native.physics_steps))
  report={'status':status,'success':success,'mode':'known-model physical-contact baseline','candidate':a.candidate,'local_variant':{'slide_m':slide,'depth_m':depth,'roll_deg':roll},'T_grasp':T.tolist(),'base_fixed':base,'official_unsplit_finger_counts':{n:sum(s.body==n for s in collision.robot) for n in FINGERS},'ownership_gate_used':False,'raw_triangle_execution_veto':False,'proxy_approximate_and_frozen':True,'finger_effort_limit_n':10.,'preload_target_n':policy['preload_n'],'friction_unchanged':True,'object_actuation':'none during execution; loader closed reset only','attachments':False,'bilateral_hold_established':legal,'actual_2mm_tcp_motion_completed':pull2,'preflight':preflight,'initial_door_angle_deg':rows[0]['door_angle_deg'] if rows else None,'final_door_angle_deg':rows[-1]['door_angle_deg'] if rows else None,'actual_door_displacement_deg':rows[-1]['door_angle_deg']-rows[0]['door_angle_deg'] if rows else None,'peak_finger_handle_force_n':max((max(s['forces_n'].values()) for s in rows),default=0.),'minimum_joint_margin_rad':min((s['margin_rad'] for s in rows),default=None),'maximum_relative_translation_slip_m':max((s['relative_translation_slip_m'] for s in rows),default=None),'final_relative_translation_slip_m':rows[-1]['relative_translation_slip_m'] if rows else None,'duration_s':len(rows)*dt,'first_failure_state':rows[-1] if rows and not success and not a.no_operation else None,'video':'contact_baseline.mp4','physics_dt_s':dt,'render_fps':30,'no_operation':a.no_operation,'deadline_shanghai':a.deadline_shanghai};(a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('first_failure_state','preflight')},indent=2),flush=True);app.close()

if __name__=='__main__':main()
