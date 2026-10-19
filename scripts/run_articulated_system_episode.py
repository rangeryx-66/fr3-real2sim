"""Independent continuation hooks; original successful physical prefix frozen."""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_articulated_skill_episode import source as original

def source():
    text=original()
    def replace(a,b):
        nonlocal text
        if text.count(a)!=1:raise RuntimeError('SYSTEM_HOOK_CHANGED:'+a[:90])
        text=text.replace(a,b)
    replace('from articulated_interaction_skill.capture import CaptureRecorder','from articulated_system.capture import MultiviewRecorder as CaptureRecorder')
    replace('from articulated_interaction_skill.session import run as run_skill','from articulated_system.session import run as run_skill')
    marker="   skill_result=run_skill(runtime);"
    insertion='''   from articulated_system.recovery import Recovery
   saved_memory=None;retreat_path=None;retreat_edge=None;release_opening=None;original_moving_reference=collision.moving_reference.copy()
   def system_gains(position=True):
    kps=np.zeros(len(names));kds=kps.copy();kps[arm]=10000. if position else 0.;kds[arm]=400. if position else 0.;kps[fingers]=1000. if mode=='position' else 0.;kds[fingers]=40.
    controller.set_gains(kps=kps,kds=kds,save_to_usd=False)
    if position:
     # PhysX retains actuation forces when a later action only sets positions.
     # Release/retreat must clear the previous grasp/task effort, not fight it
     # with changed gains or a relaxed aperture acceptance threshold.
     controller.apply_action(ArticulationAction(joint_efforts=np.zeros(len(arm)),joint_indices=arm))
     controller.apply_action(ArticulationAction(joint_efforts=np.zeros(len(fingers)),joint_indices=fingers))
   def release_begin(D):
    nonlocal reference,grasp_tcp,compliant,mode,memory,saved_memory,moving_initial
    saved_memory=memory;memory=None;reference=None;retention.reference=None;retention.history.clear();retention.drift_m=0.;drive.active=False;compliant=False
    qtarget[:]=np.asarray(robot.get_joint_positions());mode='position';system_gains();moving_initial=D@original_moving_reference;grasp_tcp=None
   def release_increment(amount):
    opening=min(float(qtarget[fingers[0]]-qtarget[fingers[1]])+amount,.1);qtarget[fingers]=[opening/2,-opening/2];runtime.phase('SYSTEM_RELEASE')
   def released():return release_opening is not None and max(filtered())<.02 and float(rows[-1]['aperture_m'])>=release_opening-.0005
   def close_current():
    nonlocal mode,effort,legal,reference,grasp_tcp,compliant
    compliant=False;mode='position';system_gains();closure=JawCenteredClosure(tcp(),policy);legal=False
    for _ in range(int(100/dt)):
     state=rows[-1];P=poses(np.asarray(state['q']));centers=[(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]
     cmd=closure.update(tcp(),state['aperture_m'],filtered(),centers,dt);phase_name=cmd['state'];runtime.phase(phase_name)
     if cmd['mode']=='position':qtarget[fingers]=[cmd['opening_m']/2,-cmd['opening_m']/2]
     else:
      if mode!='effort':mode='effort';system_gains()
      effort=cmd['closing_effort_n']
     q=ik(cmd['T'],np.asarray(state['q'])[arm])
     if q is None:raise RuntimeError('RECOVERY_CLOSURE_IK_FAILURE')
     qvelocity[:]=np.clip((q-qtarget[arm])/dt,-vel/3,vel/3);qtarget[arm]=q;step();ready,detail=grip_window()
     if closure.state=='FORCE_HOLD' and ready:legal=True;break
    if not legal:raise RuntimeError('RECOVERY_BILATERAL_GRASP_FAIL')
    qvelocity[:]=0.;grasp_tcp=tcp().copy();runtime.grasp=grasp_tcp.copy();retention.arm();reference=True;runtime.phase('FORCE_HOLD');hold(.5)
   def plan_retreat(D):
    nonlocal retreat_path,retreat_edge,release_opening
    observed_moving=D@original_moving_reference
    from piper_mobile_demo.model import Model
    target=tcp().copy();target[:3,3]-=.04*target[:3,2];qstart=np.asarray(robot.get_joint_positions())[arm];release_opening=min(.1,float(rows[-1]['aperture_m'])+job['system_capture'].get('release_extra_aperture_m',.020))
    def check(q,_base):
     if model.margin(np.asarray(q))<=.05:return False,'LOW_JOINT_MARGIN',None
     ok,why=collision.check(model.poses(np.asarray(q),base,width=release_opening),observed_moving,False);return ok,why,None
    class PlanningView:
     def __getattr__(self,name):return getattr(model,name)
     def check(self,q,_base):return check(q,_base)
    # A collision-checked arm retreat need not preserve the initial TCP
    # orientation along a straight Cartesian line. Keep the SAME existing
    # joint planner, margin and collision checks for a direct home escape.
    if check(qstart,base)[0]:
     retreat_path=Model.joint_plan(PlanningView(),qstart,np.asarray(model.home),base,iterations=1500)
     if retreat_path is not None:
      retreat_edge=[qstart];return
    qpre=ik(target,qstart)
    if qpre is None:raise RuntimeError('NO_SAFE_RELEASE_RETREAT_IK')
    edge=list(np.linspace(qstart,qpre,max(2,int(np.max(abs(qpre-qstart))/.025)+2)))
    if any(not check(q,base)[0] for q in edge):raise RuntimeError('RELEASE_RETREAT_COLLISION')
    retreat_path=Model.joint_plan(PlanningView(),qpre,np.asarray(model.home),base,iterations=1500)
    if retreat_path is None:raise RuntimeError('NO_SAFE_ARM_HOME_RETREAT')
    retreat_edge=edge
   def retreat_home(choice):
    if retreat_edge is None or retreat_path is None:raise RuntimeError('RETREAT_NOT_PREPLANNED')
    runtime.phase('SYSTEM_RETREAT')
    for q in retreat_edge[1:]:move(q,.12)
    for q in retreat_path[1:]:move(q,1.5)
    qtarget[fingers]=[.05,-.05];hold(1.)
   def system_at_base():
    from interactive_twin_recovery.mobile import at_base
    data=at_base(export,original_base,base);delta=transform(base[:3],[0,0,np.deg2rad(base[3])])@np.linalg.inv(transform(original_base[:3],[0,0,np.deg2rad(original_base[3])]))
    for e in data['shapes']:
     if '/World/mobile_chassis' in e['path']:e['world_transform']=(delta@np.asarray(e['world_transform'])).tolist()
    return data
   def base_route(choice):
    nonlocal collision,B
    from isaacsim.core.prims import SingleXFormPrim
    from interactive_twin_recovery.mobile import at_base
    support=SingleXFormPrim('/World/r1a7_pedestal');chassis=SingleXFormPrim('/World/mobile_chassis');old=list(base)
    route=choice['route'];runtime.phase('SYSTEM_BASE_ROUTE')
    for target in route['waypoints'][1:]:
     start=np.asarray(base).copy();delta=np.asarray(target)-start;duration=max(float(np.linalg.norm(delta[:2]))/.01,abs(float(delta[3]))/5.,dt)
     for f in np.linspace(0.,1.,max(2,int(duration/dt)))[1:]:
      pose=start+f*delta;quat=np.roll(Rotation.from_euler('z',pose[3],degrees=True).as_quat(),1)
      robot.set_world_pose(pose[:3],quat);support.set_world_pose(np.array([pose[0],pose[1],-.33]),quat);chassis.set_world_pose(np.array([pose[0],pose[1],-.66]),quat)
      base[:]=pose.tolist();B=transform(base[:3],[0,0,np.deg2rad(base[3])])
      if tick%8==0:
       data=system_at_base();collision=PhysicalScene(data,model,allowed);collision.moving_reference=moving_initial
      step()
    runtime.phase('SYSTEM_BASE_LOCK');hold(1.)
   def regrasp(choice,observed_correction):
    nonlocal collision,moving_initial,anchor_link,anchor_world
    from interactive_twin_recovery.mobile import at_base
    moving_initial=observed_correction@moving_initial
    collision=PhysicalScene(system_at_base(),model,allowed);collision.moving_reference=moving_initial
    candidates=choice['plan']['trial_candidates'][:job['system_capture']['regrasp_budget']]
    if not candidates:raise RuntimeError('NO_REGRASP_PLAN')
    # New RGB-D correction must remain within the prechecked pose; otherwise
    # rebuild a plan rather than executing an obsolete closed-state target.
    if np.linalg.norm(observed_correction[:3,3])>.001:raise RuntimeError('REGRASP_POSE_CHANGED_REPLAN_REQUIRED')
    trial=candidates[0];runtime.phase('SYSTEM_REGRASP_PREGRASP')
    for q in trial['preplan'][1:]:move(q,1.5)
    runtime.phase('APPROACH')
    for q in trial['approach']:move(q,.12)
    close_current()
   def reset_memory():
    nonlocal memory,drive,compliant,fit_poses
    memory=InteractionMemory(tcp(),a.output);memory.estimate=json.loads(json.dumps(saved_memory.estimate));memory.follow_sign=saved_memory.follow_sign;memory.observe(tcp());fit_poses=memory.poses
    runtime.memory=memory;drive=ConstrainedDrive(tcp(),memory.tangent(tcp()));drive.active=False;runtime.drive=drive;compliant=True;system_gains(False);runtime.phase('COMPLIANT_SETTLE');hold(.5)
    memory.initial=tcp().copy();memory.monitor_anchor=tcp().copy();memory.save()
   original_base=list(base);runtime.deadline=deadline.timestamp();runtime.margin=lambda:model.margin(np.asarray(robot.get_joint_positions())[arm]);runtime.state_offset=0.
   runtime.release_begin=release_begin;runtime.release_increment=release_increment;runtime.released=released;runtime.reclose_at_current_pose=close_current;runtime.plan_retreat=plan_retreat;runtime.retreat_home=retreat_home;runtime.base_route=base_route;runtime.regrasp=regrasp;runtime.reset_grasp_memory=reset_memory
   runtime.initial_visual={'T_world_handle':T.tolist(),'anchor_world_m':anchor_world.tolist(),'axis_world':T[:3,0].tolist(),'outward_normal_world':normal.tolist(),'dimensions_m':json.loads((a.asset_root/'manifest.json').read_text())['interaction_geometry']['selection']['dimensions_m']}
   recovery=Recovery(runtime,ROOT,job,export,model,allowed);runtime.recover=recovery.run
'''
    replace(marker,insertion+marker)
    # Add new orchestration phases to the SAME existing load cap; no threshold changes.
    text=text.replace("if phase in ('CLOSE'","if (phase.startswith('SYSTEM_') or phase in ('CLOSE'")
    text=text.replace(") and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']:",")) and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']:")
    ast.parse(text);return text

if __name__=='__main__':
    import argparse,json,hashlib
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--job',type=Path,required=True);args,_=parser.parse_known_args()
    job=json.loads(args.job.read_text());output=Path(job['output']);output.mkdir(parents=True,exist_ok=True);expanded=source()
    paths=[Path(__file__),ROOT/'articulated_system/recovery.py',ROOT/'articulated_system/session.py',ROOT/'articulated_system/capture.py']
    (output/'orchestration_provenance.json').write_text(json.dumps({'job_sha256':hashlib.sha256(args.job.read_bytes()).hexdigest(),
        'expanded_program_sha256':hashlib.sha256(expanded.encode()).hexdigest(),
        'sources':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        'release_transition':'clear previous actuator effort; unchanged position gains/preload/limits'},indent=2))
    (output/'frozen_execution_job.json').write_text(json.dumps(job,indent=2))
    (output/'expanded_program_provenance.py').write_text(expanded)
    exec(compile(expanded,str(ROOT/'scripts/run_interactive_twin_refinement_episode.py'),'exec'),globals())
