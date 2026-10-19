"""Offline anticipated exit of a fresh visual plan; no motion and no dynamics truth."""
import json,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.kinematics import FastNativeModel
from cross_robot_adapter import RobotPinkIK
from xarm7_native_execution import mapped_export
from interactive_twin_recovery.mobile import physical_scene_class
from revolute10_fixed import authored_handle_colliders
from native_g1_safe_park import plan_exit
p=Path(sys.argv[1]);targets=list(map(float,sys.argv[2:]));job=json.loads((p/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());model=FastNativeModel(None,Path(job['asset_root']),source);config=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text())['gripper_config'];solver=RobotPinkIK(config);base=source['robot_base_pose'];export=json.loads((p/'planning_current_scene.json').read_text());allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection']);scene=physical_scene_class(ROOT)(mapped_export(export),model,allowed);plan=json.loads((p/'visual_plan.json').read_text());audit=json.loads((p/'EXECUTION_CANDIDATE_AUDIT.json').read_text());start=audit['candidates'][-1]['actual_door_state_rad'];prior=json.loads((p/'identification_input.json').read_text())['prior'];axis=np.array(prior['axis_world']);axis/=np.linalg.norm(axis);origin=np.array(prior['origin_world_m']);T=np.array(plan['T']);q=np.array(plan['approach'][-1]);results=[]
for target in targets:
    for a in np.linspace(0,np.deg2rad(target)-start,61):
        R=Rotation.from_rotvec(axis*a).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=origin-R@origin;qn=solver.solve(D@T,base,q,starts=1)
        if qn is None:break
        q=qn
    if qn is None:results.append(dict(target_deg=target,valid=False,reason='ANTICIPATED_IK'));continue
    moving=D@scene.moving_reference;qall=np.r_[q,np.full(6,model.native.angle(.084))];arm=np.arange(7);fingers=np.arange(7,13)
    g=dict(model=model,solver=solver,collision=scene,robot=SimpleNamespace(get_joint_positions=lambda:qall),arm=arm,fingers=fingers,base=base,moving=lambda:moving,tcp=lambda:D@T,qtarget=qall,normal=T[:3,2],ik=lambda M,seed:solver.solve(M,base,seed,starts=1))
    route=plan_exit(g);results.append(dict(target_deg=target,valid=route['valid'],reason=route.get('reason'),region=route.get('selected_region')))
r=dict(source='fresh visual plan, permitted hinge/state prior and native safety geometry; engineering prediction only',requires_measured_state_revalidation=True,targets=results);(p/'FRESH_PLAN_EXIT_CHECK.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
