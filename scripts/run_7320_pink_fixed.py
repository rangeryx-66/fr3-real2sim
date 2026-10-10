"""Known-model, Pinocchio/Pink, one stationary base and one physical grasp.
Existing frozen setup/closure/guards/telemetry reused. No mobile/regrasp sequence.
"""
import sys,json,time
from pathlib import Path
from piper_pink_fixed import PinkIK
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());baseline=Path(spec['baseline_runner']);text=baseline.read_text();setup,sequence=text.split("try:\n phase='SETTLE';hold(.5)",1)
exec(compile(setup,str(baseline)+'::frozen_setup','exec'),globals())
solver=PinkIK(ROOT/'config/piper.urdf',model.home);model.ik=solver.solve
# Only the IK source changes. Contact, jaw, plant and guards remain frozen.
original_step=step;grasp_relative=None;ref=None;twist=np.zeros(6);goal=np.deg2rad(90.);angle_reference=0.;bias=None;seed=None;opening=False

def body_at(theta):
 j=model.manifest['joint_name'];link=model.manifest['moving_link'];F=model.asset_T@model.asset.root_to_link(link,{j:float(theta)});F0=model.asset_T@model.asset.root_to_link(link,{j:0.})
 return F@np.linalg.inv(F0)@moving_initial

def actual():return float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])

def step():
 global ref,twist,angle_reference,qvelocity,seed,planned_reference,command_state
 if opening:
  theta=actual();lag=max(0.,angle_reference-theta);radius=max(1e-12,float(np.linalg.norm(tcp()[:3,3]-body_at(theta)[:3,3])))
  rate=.0015/radius/(1+lag/np.deg2rad(.05));angle_reference=min(goal+np.deg2rad(.5),max(theta+np.deg2rad(.05),angle_reference+rate*dt))
  desired=body_at(angle_reference)@np.linalg.inv(body_at(theta))@moving()@grasp_relative
  dp=desired[:3,3]-ref[:3,3];rv=Rotation.from_matrix(desired[:3,:3]@ref[:3,:3].T).as_rotvec();error=np.r_[dp,rv*radius];distance=np.linalg.norm(error);acc=.002
  wanted=error/max(distance,1e-12)*min(.0015,np.sqrt(2*acc*distance));change=wanted-twist;twist+=change*min(1.,acc*dt/max(np.linalg.norm(change),1e-12));inc=twist*dt
  if np.dot(inc,error)>0 and np.linalg.norm(inc)>distance:inc=error.copy();twist=inc/dt
  target=ref.copy();target[:3,3]+=inc[:3];target[:3,:3]=Rotation.from_rotvec(inc[3:]/radius).as_matrix()@ref[:3,:3]
  q=solver.velocity_step(target,base,seed,dt)
  if q is None:raise RuntimeError('FIXED_BASE_PINK_REACHABLE_INTERVAL_END')
  command=q+bias
  if model.margin(command)<=.05:raise RuntimeError('EXISTING_COMMAND_JOINT_MARGIN')
  delta=command-qtarget[arm];fraction=min(1.,float(np.min((vel/3)/np.maximum(np.abs(delta)/dt,1e-12))))
  qtarget[arm]+=fraction*delta;qvelocity=fraction*delta/dt;seed=qtarget[arm]-bias
  ref[:3,3]+=fraction*inc[:3];ref[:3,:3]=Rotation.from_rotvec(fraction*inc[3:]/radius).as_matrix()@ref[:3,:3];planned_reference=ref.copy();command_state=angle_reference
 return original_step()

initial_grasp=False
try:
 initial="if True:\n phase='SETTLE';hold(.5)"+sequence.split("  profile_mark('OPEN_1')",1)[0]
 exec(compile(initial,str(baseline)+'::unchanged_initial_grasp','exec'),globals());initial_grasp=legal
 grasp_relative=np.linalg.inv(moving())@tcp();reference=grasp_relative.copy();ref=tcp().copy();seed=np.asarray(robot.get_joint_positions())[arm];bias=qtarget[arm]-seed;angle_reference=actual();opening=True
 diagnostic_write(a.output/'acquired_grasp_reference.json',json.dumps({'T_body_tcp':grasp_relative.tolist(),'base':base,'q':seed.tolist(),'t':tick*dt}))
 while actual()<goal:
  phase='FIXED_BASE_PINK_OPEN';step()
 opening=False;qvelocity=np.zeros(6);phase='FINAL_HOLD';hold(.5);status='FIXED_BASE_SINGLE_GRASP_90_SUCCESS';success=True
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True);opening=False;qvelocity=np.zeros(6)
world.pause()
for label,data in [('observations',rows),('physics_steps',native.physics_steps)]:
 try:export_raw_records(label,data)
 except Exception as error:diagnostic_issue(label,error)
try:video.stdin.close();video.wait(timeout=30)
except Exception as error:diagnostic_issue('video',error)
result={'classification':'KNOWN_MODEL_DIAGNOSTIC_PINOCCHIO_PINK_FIXED_BASE','success':success,'status':status,'max_actual_angle_deg':max_state,'final_actual_angle_deg':float(np.rad2deg(actual())),'base_fixed':list(base),'initial_grasp':initial_grasp,'base_moves_after_grasp':0,'regrasps':0,'intentional_releases':0,'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'peak_pad_load_n':max((max(r['forces_n'].values()) for r in rows),default=0),'minimum_joint_margin_rad':min((r['margin_rad'] for r in rows),default=None),'physics_steps':len(native.physics_steps),'IK':'Pinocchio3.9.0/Pink3.3.0/DAQP','solver_calls':solver.calls,'last_solver_issue':solver.last_error,'scene_preserved':True,'pid':os.getpid(),'preposition':'normal closed setup initialized at selected station; no base actuation during grasp','GT_dependencies':['known first-grasp template','hinge geometry','actual moving-body and articulation feedback','collision model']}
diagnostic_write(a.output/'report.json',json.dumps(result,indent=2));diagnostic_write(a.output/'events.json',json.dumps(telemetry_events,indent=2));print('PHYSICAL_RESULT',json.dumps(result),flush=True)
while app.is_running():
 if (a.output/'close_completed_scene').exists():app.close();break
 app.update();time.sleep(.1)
