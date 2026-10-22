"""Robot-only archived-candidate diagnostic; no collision/execution approval.

Compile the exact frozen numeric Pink class without importing a USD scene.
Online station forecasts use the actual installed runtime class and colliders.
This tool never creates a simulator, grasp transform or physical action.
"""
import argparse,ast,hashlib,json,sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def run(episode,bases,goal):
    import numpy as np,pinocchio as pin,pink
    from pink.tasks import FrameTask,PostureTask
    from scipy.spatial.transform import Rotation
    from articulated_demo.kinematics import transform
    from interaction_runtime.articulation_v7 import Articulation
    from .station_forecast import check_arc
    source=ROOT/'scripts/cross_robot_adapter.py'
    node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='RobotPinkIK')
    scope=dict(np=np,pin=pin,pink=pink,FrameTask=FrameTask,PostureTask=PostureTask,Rotation=Rotation,transform=transform)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),scope)
    episode=Path(episode);plan=json.loads((episode/'visual_plan.json').read_text())
    c=json.loads((episode/'robot_io/NATIVE_CONFIG.json').read_text())
    settings=json.loads((episode/'identification_input.json').read_text());p=settings['prior']
    limits=json.loads((episode/'job.json').read_text())['articulation_v7']['limits_native']
    motion=Articulation(p['joint_type'],tuple(p['axis_world']),tuple(p['origin_world_m']),tuple(limits))
    motion.validate_target(dict(target_q=goal,coordinate_unit=motion.units['coordinate']))
    results=[]
    for xy in bases:
        if not all(__import__('math').isfinite(v) for v in xy):raise ValueError('FINITE_XY_REQUIRED')
        base=np.asarray(plan['base']).copy();base[:2]=xy
        solver=scope['RobotPinkIK'](c)
        result=check_arc(motion,np.asarray(plan['T']),np.eye(4),base,
            float(plan['path'][0]['state']),goal,float(plan['contact_geometry_audit']['physical_aperture_m']),
            solver,lambda *a,**k:{},SimpleNamespace(check=lambda *a:(True,'NO_COLLISION_CHECK')))
        results.append(dict(base_xy_m=xy,**result))
    return dict(scope='IK_ONLY_DIAGNOSTIC_NOT_COLLISION_OR_EXECUTION',results=results,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        selected_plan_sha256=hashlib.sha256((episode/'visual_plan.json').read_bytes()).hexdigest(),
        archived_observed_candidate_not_online_pose_fallback=True,hidden_dynamics_used=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('episode',type=Path)
    parser.add_argument('--base-xy',nargs=2,type=float,action='append',required=True)
    parser.add_argument('--goal-q',type=float,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.write_text(json.dumps(run(args.episode,args.base_xy,args.goal_q),indent=2))
