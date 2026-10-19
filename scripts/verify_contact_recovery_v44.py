"""Offline archived-state planning regression, no Isaac actor or new grasp pose."""
import json,shutil,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def state(p,cooked,out):
    from interaction_runtime.kinematics import FastNativeModel
    from cross_robot_adapter import RobotPinkIK
    from xarm7_native_execution import mapped_export
    from interactive_twin_recovery.mobile import physical_scene_class
    from revolute10_fixed import authored_handle_colliders
    j=json.loads((p/'job.json').read_text());c=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text());source=json.loads((Path(j['source'])/'report.json').read_text());m=FastNativeModel(None,Path(j['asset_root']),source);solver=RobotPinkIK(c['gripper_config']);export=json.loads(cooked.read_text());allowed=authored_handle_colliders(export,m.manifest['interaction_geometry']['selection']);scene=physical_scene_class(ROOT)(mapped_export(export),m,allowed)
    check=json.loads((p/'agent_responses/014.json').read_text())['result'];Q=np.asarray(json.loads((p/'rgbd/generation_01_agent_02/camera.json').read_text())['robot_q']);target=Q.copy();target[c['arm_indices']]=check['return_path'][1];park=json.loads((p/'agent_responses/006.json').read_text())['result']['safe_exit']['park'];M=np.asarray(park['moving_pose']);out.mkdir(parents=True,exist_ok=True);shutil.copyfile(cooked,out/'cooked_initial.json')
    g=dict(a=SimpleNamespace(output=out),ROOT=ROOT,job=j,model=m,robot=SimpleNamespace(get_joint_positions=lambda:Q),qtarget=target,
           arm=c['arm_indices'],fingers=c['finger_indices'],allowed=allowed,collision=scene,base=source['robot_base_pose'],moving=lambda:M,actual=lambda:np.deg2rad(park['door_angle_deg']),ik=lambda T,q:solver.solve(T,source['robot_base_pose'],q),wrist_routes=[dict(q_path=list(reversed(check['return_path'][2:-1])))],execution_measured_fingers=True)
    return g,c,solver


def run(p,cooked,out):
    from native_g1_execution_candidates import CandidateCheck,context
    from interaction_runtime.policy import Policy
    from interaction_runtime.segments import classify_outcome
    import native_g1_multiview_input as multi
    from agent_command_protocol_v12 import atomic_json
    g,c,solver=state(p,cooked,out);policy=Policy.load(ROOT/'configs/interaction_runtime_v1.json')
    old=json.loads((p/'visual_plan.json').read_text());before=CandidateCheck(g,policy,c['gripper_config'],solver)(old)
    assert not before['valid'] and 'original39' in before['reason']
    # Same existing episode-local failed-family memory as the live adapter.
    failed=tuple(json.loads((p/'agent_responses/006.json').read_text())['result']['family']);rescore=multi.rescore
    def remaining(*args,**kwargs):
        return [x for x in rescore(*args,**kwargs) if tuple(float(x[k]) for k in ['section','insertion_m','roll','pitch'])!=failed]
    with context(g,policy,c['gripper_config'],solver),patch.object(multi,'rescore',remaining):
        candidate,geometry=multi.plan_observed(g,p/'agent_geometry')
    result=dict(archived_fixture_only=True,old_candidate=before,new_candidate=dict(family=[candidate[k] for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']],T=candidate['T'],feasibility=candidate['execution_feasibility']),
                classification_replay=classify_outcome(False,True,'ADAPTIVE_HORIZON_WITHOUT_HEALTHY_PROGRESS',False,1.),safety_or_score_changes=False)
    assert result['classification_replay']=='TRANSMISSION_FAILURE';assert result['new_candidate']['feasibility']['valid']
    atomic_json(out/'OFFLINE_RESULT.json',result);print(json.dumps(result,indent=2));return result

if __name__=='__main__':run(*map(Path,sys.argv[1:4]))
