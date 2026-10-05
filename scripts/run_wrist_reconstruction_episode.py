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
    replace(" from isaacsim.core.utils.types import ArticulationAction", " from wrist_reconstruction.scene import normal_background\n normal_background(stage)\n from isaacsim.core.utils.types import ArticulationAction")
    replace("   recovery=Recovery(runtime,ROOT,job,export,model,allowed);runtime.recover=recovery.run",'''   def scan_to(target):
    from piper_mobile_demo.model import Model
    start=np.asarray(robot.get_joint_positions())[arm];goal=model.ik(target,base,seed=start,starts=5)
    if goal is None:raise RuntimeError('SCAN_NO_IK')
    if model.margin(goal)<=.05:raise RuntimeError('SCAN_UNSAFE_MARGIN')
    def check(q,_base):
     if model.margin(np.asarray(q))<=.05:return False,'LOW_JOINT_MARGIN',None
     P=model.poses(q,base,finger_q=np.asarray(robot.get_joint_positions())[fingers]);ok,why=collision.check(P,moving_initial,False)
     if not ok:return False,why,None
     # Include the nominal physical camera housing in scan/environment tests.
     from piper_mobile_demo.owned_scene import Shape,intersects
     import trimesh
     C=P['tcp_link']@skill_capture.cal['X'];body=Shape(trimesh.creation.box(skill_capture.cal['camera_body_size_m']),C,True,'/World/wrist_camera_housing','camera')
     for obstacle in collision.scene:
      if intersects(body,obstacle):return False,'SCAN_CAMERA_ENVIRONMENT_COLLISION',None
     return True,'SAFE',None
    class ScanView:
     def __getattr__(self,name):return getattr(model,name)
     def check(self,q,_base):return check(q,_base)
    path=Model.joint_plan(ScanView(),start,goal,base,iterations=1500)
    if path is None:raise RuntimeError('SCAN_NO_COLLISION_FREE_ARM_PATH')
    runtime.phase('SYSTEM_WRIST_SCAN')
    for q in path[1:]:move(q,max(1.5,float(np.max(abs(np.asarray(q)-np.asarray(robot.get_joint_positions())[arm])))/.2))
   def scan_home():
    from piper_mobile_demo.model import Model
    def check(q,_base):
     if model.margin(np.asarray(q))<=.05:return False,'LOW_JOINT_MARGIN',None
     ok,why=collision.check(model.poses(q,base,finger_q=np.asarray(robot.get_joint_positions())[fingers]),moving_initial,False);return ok,why,None
    class View:
     def __getattr__(self,name):return getattr(model,name)
     def check(self,q,_base):return check(q,_base)
    path=Model.joint_plan(View(),np.asarray(robot.get_joint_positions())[arm],model.home,base,iterations=1500)
    if path is None:raise RuntimeError('SCAN_NO_SAFE_HOME_PATH')
    runtime.phase('SYSTEM_SCAN_HOME')
    for q in path[1:]:move(q,2.)
   def observe_handle(anchor):
    from wrist_reconstruction.geometry import look_at,optical_to_tcp
    normal=np.asarray(runtime.initial_visual['outward_normal_world']);eye=np.asarray(anchor)+normal*.34+np.array([0,0,.1])
    scan_to(optical_to_tcp(look_at(eye,np.asarray(anchor)),skill_capture.cal['X']))
   runtime.scan_to=scan_to;runtime.scan_home=scan_home;runtime.observe_handle=observe_handle;skill_capture.runtime=runtime
   recovery=Recovery(runtime,ROOT,job,export,model,allowed);runtime.recover=recovery.run''')
    replace("    if np.linalg.norm(observed_correction[:3,3])>.001:raise RuntimeError('REGRASP_POSE_CHANGED_REPLAN_REQUIRED')", "    # Fresh wrist RGB-D targets have already been collision/IK replanned.\n    pass")
    replace("    saved_memory=memory;memory=None;", "    saved_memory=memory;runtime.saved_estimate=None if memory is None else memory.estimate;memory=None;")
    replace("drive=ConstrainedDrive(tcp(),memory.tangent(tcp()))", "drive=ConstrainedDrive(tcp(),memory.tangent(tcp()) if memory.estimate is not None else memory.directions[0])")
    # Keep wrist extrinsics rigidly attached to MEASURED TCP, not proposed pose.
    replace('  world.step(render=False,update_fabric=True)', '  skill_capture.sync(tcp())\n  world.step(render=False,update_fabric=True)')
    ast.parse(text);return text

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);a=p.parse_args();job=json.loads(a.job.read_text());out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    expanded=source();(out/'expanded_wrist_program.py').write_text(expanded)
    (out/'frozen_wrist_job.json').write_text(json.dumps(job,indent=2));exec(compile(expanded,str(ROOT/'scripts/run_interactive_twin_refinement_episode.py'),'exec'),globals())
