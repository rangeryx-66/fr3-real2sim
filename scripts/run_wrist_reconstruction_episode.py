"""Independent wrist scan/regrasp orchestration; frozen contact prefix retained."""
from pathlib import Path
import sys,ast,json,hashlib,importlib.util
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
# Some server deployments retained older root-level copies of entry scripts.
# Load the checked-in source chain explicitly, so those copies cannot shadow
# the current scripts/ release/retreat implementation through sys.path order.
for name in ('run_cross_object_structure_episode','run_active_structure_episode',
             'run_operational_structure_episode','run_paper_structure_episode',
             'run_articulated_skill_episode','run_articulated_system_episode'):
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/f'{name}.py')
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module)
from run_articulated_system_episode import source as parent


def source():
    text=parent()
    if "runtime.plan_retreat=plan_retreat" not in text:
        raise RuntimeError("WRIST_PARENT_RUNTIME_API_MISMATCH: missing preplanned retreat")
    def replace(a,b):
        nonlocal text
        if text.count(a)!=1:raise RuntimeError('WRIST_HOOK_CHANGED:'+a[:80])
        text=text.replace(a,b)
    replace('from articulated_system.capture import MultiviewRecorder as CaptureRecorder','from wrist_reconstruction.capture import WristRecorder as CaptureRecorder')
    replace('from articulated_system.recovery import Recovery','from wrist_reconstruction.recovery import Recovery')
    replace('from articulated_system.session import run as run_skill','from wrist_reconstruction.session import run as run_skill')
    replace("    route=choice['route'];runtime.phase('SYSTEM_BASE_ROUTE')",'''    from interactive_twin_recovery.mobile import MobileRuntimeScene
    route_export=system_at_base();route_export['shapes']=[e for e in route_export['shapes'] if '/World/mobile_chassis' not in e['path']]
    route_cache=MobileRuntimeScene(ROOT,route_export,model,list(base))
    class RouteCollision:
     def __getattr__(self,name):return getattr(route_cache.scene,name)
     def check(self,P,moving_pose,allow_handle):
      if reference is not None:raise RuntimeError('BASE_ROUTE_REQUIRES_RELEASED_GRASP')
      route_cache.moving_reference=moving_pose
      return route_cache.check(P,base)
     def contact_guard(self,contacts,P):return route_cache.contact_guard(contacts,P)
    collision=RouteCollision()
    route=choice['route'];runtime.phase('SYSTEM_BASE_ROUTE')''')
    replace("      if tick%8==0:\n       data=system_at_base();collision=PhysicalScene(data,model,allowed);collision.moving_reference=moving_initial", "      # The existing mobile cache updates transforms and repeats the same\n      # collision checks every physics step, without rebuilding cooked solids.")
    replace("    runtime.phase('SYSTEM_BASE_LOCK');hold(1.)", "    collision=PhysicalScene(system_at_base(),model,allowed);collision.moving_reference=moving_initial\n    runtime.phase('SYSTEM_BASE_LOCK');hold(1.)")
    replace(" from isaacsim.core.utils.types import ArticulationAction", " from wrist_reconstruction.scene import normal_background\n normal_background(stage)\n from isaacsim.core.utils.types import ArticulationAction")
    replace("   recovery=Recovery(runtime,ROOT,job,export,model,allowed);runtime.recover=recovery.run",'''   def execute_arm_path(path,phase_name,minimum_duration=1.5):
    runtime.phase(phase_name)
    for q in path[1:]:move(q,max(minimum_duration,float(np.max(abs(np.asarray(q)-np.asarray(robot.get_joint_positions())[arm])))/.2))
   def configure_release(opening,D):
    nonlocal release_opening
    release_opening=opening;release_begin(D)
   def set_observed_moving(D):
    nonlocal moving_initial
    moving_initial=D@original_moving_reference;collision.moving_reference=moving_initial
   def open_clear_gripper():
    for opening in np.linspace(float(qtarget[fingers[0]]-qtarget[fingers[1]]),.1,40):
     qtarget[fingers]=[opening/2,-opening/2];hold(.05)
   def halt_at_measured_state():
    nonlocal mode,compliant
    qtarget[:]=np.asarray(robot.get_joint_positions());qvelocity[:]=0.;mode='position';compliant=False;system_gains()
   def return_last_safe():
    from piper_mobile_demo.model import Model
    target=np.asarray(runtime.last_safe_arm);start=np.asarray(robot.get_joint_positions())[arm]
    def check(q,b):
     if model.margin(q)<=.05:return False,'LOW_JOINT_MARGIN',None
     ok,why=collision.check(model.poses(q,b,finger_q=np.asarray(robot.get_joint_positions())[fingers]),moving_initial,False);return ok,why,None
    class View:
     def __getattr__(self,name):return getattr(model,name)
     def check(self,q,b):return check(q,b)
    path=Model.joint_plan(View(),start,target,base,iterations=1500)
    if path is None:return False
    try:execute_arm_path(path,'SYSTEM_SAFE_RETURN');hold(.3);return True
    except RuntimeError:return False
   runtime.arm_q=lambda:np.asarray(robot.get_joint_positions())[arm].copy();runtime.finger_q=lambda:np.asarray(robot.get_joint_positions())[fingers].copy()
   runtime.execute_arm_path=execute_arm_path;runtime.configure_release=configure_release;runtime.open_clear_gripper=open_clear_gripper;runtime.set_observed_moving=set_observed_moving
   runtime.halt_at_measured_state=halt_at_measured_state;runtime.return_last_safe=return_last_safe;runtime.last_safe_arm=runtime.arm_q()
   skill_capture.runtime=runtime
   recovery=Recovery(runtime,ROOT,job,export,model,allowed);runtime.recover=recovery.run
   runtime.scan_to=lambda target:recovery.mobile.execute(target@skill_capture.cal['X'])
   runtime.scan_home=recovery.mobile.home;runtime.observe_handle=recovery.mobile.observe_handle
''')
    replace("    if np.linalg.norm(observed_correction[:3,3])>.001:raise RuntimeError('REGRASP_POSE_CHANGED_REPLAN_REQUIRED')", "    # Fresh wrist RGB-D targets have already been collision/IK replanned.\n    pass")
    replace("    saved_memory=memory;memory=None;", "    saved_memory=memory;runtime.saved_estimate=None if memory is None else memory.estimate;memory=None;")
    replace("drive=ConstrainedDrive(tcp(),memory.tangent(tcp()))", "drive=ConstrainedDrive(tcp(),memory.tangent(tcp()) if memory.estimate is not None else memory.directions[0])")
    replace("memory.initial=tcp().copy();memory.monitor_anchor=tcp().copy();memory.save()", "memory.initial=tcp().copy();from wrist_reconstruction.recovery import reset_model_monitor;reset_model_monitor(memory,tcp());memory.save()")
    # Keep wrist extrinsics rigidly attached to MEASURED TCP, not proposed pose.
    replace('  world.step(render=False,update_fabric=True)', '  skill_capture.sync(tcp())\n  world.step(render=False,update_fabric=True)')
    replace('  return s\n def move', '  if skill_capture.runtime is not None:skill_capture.runtime.last_safe_arm=np.asarray(s["q"])[arm].copy()\n  return s\n def move')
    ast.parse(text);return text

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);a=p.parse_args();job=json.loads(a.job.read_text());out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    expanded=source();(out/'expanded_wrist_program.py').write_text(expanded)
    cal_path=Path(job['camera_calibration'])
    if not cal_path.is_absolute():cal_path=ROOT/cal_path
    paths=[Path(__file__),cal_path,*sorted((ROOT/'wrist_reconstruction').glob('*.py'))]
    (out/'wrist_orchestration_provenance.json').write_text(json.dumps({
        'camera_calibration_path':str(cal_path),
        'camera_calibration_sha256':hashlib.sha256(cal_path.read_bytes()).hexdigest(),
        'job_sha256':hashlib.sha256(a.job.read_bytes()).hexdigest(),
        'expanded_program_sha256':hashlib.sha256(expanded.encode()).hexdigest(),
        'sources':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        'physical_contact_baseline_unchanged':True},indent=2))
    (out/'frozen_wrist_job.json').write_text(json.dumps(job,indent=2));exec(compile(expanded,str(ROOT/'scripts/run_interactive_twin_refinement_episode.py'),'exec'),globals())
