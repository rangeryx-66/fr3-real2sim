"""Archived-pose engineering reach/exit check only; never online grasp input."""
import sys,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def run(p,out):
    from interaction_runtime.kinematics import FastNativeModel
    from cross_robot_adapter import RobotPinkIK
    from xarm7_native_execution import mapped_export
    from interactive_twin_recovery.mobile import physical_scene_class
    from revolute10_fixed import authored_handle_colliders
    from native_g1_execution_candidates import current_export
    from native_g1_safe_park import plan_exit
    from types import SimpleNamespace
    began=time.perf_counter();job=json.loads((p/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());cfg=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text());model=FastNativeModel(None,Path(job['asset_root']),source);solver=RobotPinkIK(cfg['gripper_config']);base=source['robot_base_pose'];prior=json.loads((p/'identification_input.json').read_text())['prior'];axis=np.array(prior['axis_world']);axis/=np.linalg.norm(axis);origin=np.array(prior['origin_world_m']);export=json.loads((p/'planning_current_scene.json').read_text());allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection']);scene=physical_scene_class(ROOT)(mapped_export(export),model,allowed)
    def D(deg):
        R=Rotation.from_rotvec(axis*np.deg2rad(deg)).as_matrix();T=np.eye(4);T[:3,:3]=R;T[:3,3]=origin-R@origin;return T
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text());attempts=[e['result'] for e in trace if e['command']['operation']=='attempt_grasp_and_probe'];recorded_start=attempts[0]['safe_exit']['park']['door_angle_deg'];Mzero=np.linalg.inv(D(recorded_start))@scene.moving_reference
    cases=[]
    for k,start in [(0,0.),(1,recorded_start)]:
        plan=json.loads((p/('agent_inspected/g%02d_plan.json'%k)).read_text());T0=np.linalg.inv(D(start))@np.asarray(plan['T']);q=np.asarray(plan['approach'][-1]);path=[];reason=None
        for theta in np.linspace(start,45,181):
            qn=solver.solve(D(theta)@T0,base,q,starts=1)
            if qn is None:reason='CONTINUOUS_IK';break
            margin=model.margin(qn)
            if margin<=.05:reason='JOINT_MARGIN';break
            ok,why=scene.check(model.poses(qn,base,width=.015),D(theta)@Mzero,True)
            if not ok:reason=why;break
            path.append(dict(angle_deg=float(theta),q=qn.tolist(),margin_rad=float(margin)));q=qn
        record=dict(archived_generation=k,start_deg=start,max_kinematic_deg=path[-1]['angle_deg'] if path else start,limiting_reason=reason,min_margin_rad=min([s['margin_rad'] for s in path],default=None),path=path,parking_checks=[])
        # Check released, current-angle exit at largest supported targets only.
        for target in [20.,30.,45.]:
            eligible=[s for s in path if s['angle_deg']>=target]
            if not eligible:continue
            s=eligible[0];arm=np.arange(7);fingers=np.arange(7,13);qall=np.r_[s['q'],np.full(6,model.native.angle(.084))];moving=D(s['angle_deg'])@Mzero
            g=dict(model=model,solver=solver,collision=scene,robot=SimpleNamespace(get_joint_positions=lambda qall=qall:qall),arm=arm,fingers=fingers,base=base,moving=lambda moving=moving:moving,tcp=lambda s=s:D(s['angle_deg'])@T0,qtarget=qall,normal=np.asarray(T0)[:3,2],ik=lambda T,seed:solver.solve(T,base,seed,starts=1))
            try:e=plan_exit(g);record['parking_checks'].append(dict(angle_deg=s['angle_deg'],valid=e['valid'],reason=e.get('reason')))
            except Exception as exc:record['parking_checks'].append(dict(angle_deg=s['angle_deg'],error=repr(exc)))
        cases.append(record)
    result=dict(scope='OFFLINE_ARCHIVED_ENGINEERING_ONLY; never live plan/view/grasp input',no_physics=True,cases=cases,wall_s=time.perf_counter()-began)
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2));print(json.dumps(dict(cases=[{k:v for k,v in c.items() if k!='path'} for c in cases],wall_s=result['wall_s'])))
if __name__=='__main__':run(*map(Path,sys.argv[1:3]))
