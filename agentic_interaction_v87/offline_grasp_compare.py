"""Closed-episode, same-wrist-frame native-planner comparison; no Isaac imports."""
import argparse,copy,json,math,shutil,types
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np


def compare(episode,frame,inference,output,mode,scored_cache=None):
    episode,frame,output=map(Path,(episode,frame,output))
    if output.exists():raise ValueError('PRESERVE_PREVIOUS_COMPARISON')
    output.mkdir(parents=True)
    # Load the installed USD PhysX schema without starting Kit/World. The
    # frozen collision module imports this schema also for its unused report
    # class; no placeholder or modified collision predicate is substituted.
    import pxr
    package=Path('/data1/home/rangeryx/isaaclab-arena/.venv/lib/python3.12/site-packages/isaacsim')
    schemas=list(package.glob('extscache/omni.usd.schema.physx-*/pxr'))
    if len(schemas)!=1:raise ValueError('UNAMBIGUOUS_INSTALLED_PHYSX_SCHEMA_REQUIRED')
    usd=list(package.glob('extscache/omni.usd.libs-*/pxr'))
    if len(usd)!=1:raise ValueError('UNAMBIGUOUS_INSTALLED_USD_REQUIRED')
    pxr.__path__[:]=[str(usd[0]),str(schemas[0])]
    from pxr import Usd,UsdPhysics,PhysxSchema
    job=json.loads((episode/'job.json').read_text());camera=json.loads((frame/'camera.json').read_text())
    import xarm7_native_gripper as hand
    hand.OUT=Path(job['v8_robot_io'])
    from interaction_runtime.kinematics import FastNativeModel
    from cross_robot_adapter import RobotPinkIK
    from agentic_interaction_v81.collision import MobileScene
    from revolute10_fixed import authored_handle_colliders
    from interaction_runtime.policy import Policy
    from native_g1_execution_candidates import CandidateCheck,context
    from agentic_interaction_v81.replanning import candidate_gate
    import native_g1_multiview_input as multi
    from .grasp_candidates import geometry_candidates,novel_shortlist,pose_id
    config=json.loads((Path(job['v8_robot_io'])/'NATIVE_CONFIG.json').read_text())
    source=json.loads((Path(job['source'])/'report.json').read_text())
    model=FastNativeModel(None,Path(job['asset_root']),source);solver=RobotPinkIK(config);model.ik=solver.solve
    collision_export=episode/'cooked_initial.json'
    if not collision_export.exists():collision_export=episode/'planning_current_scene.json'
    export=json.loads(collision_export.read_text())
    allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection'])
    base=np.asarray(camera['base']);collision=MobileScene(export,model,allowed,source['robot_base_pose'])
    measured=np.asarray(camera['robot_q']);names=camera['robot_joint_names']
    arm=np.array([names.index(n)for n in config['arm_joint_names']])
    fingers=np.array([names.index(n)for n in hand.GRIPPER_JOINTS])
    link=model.manifest['moving_link'];joint=model.manifest['joint_name']
    if json.loads((episode/'PROCESS_COMPLETION.json').read_text()).get('actor_exited')is not True:
        raise ValueError('CLOSED_SOURCE_REQUIRED')
    progress=json.loads((episode/'TASK_PROGRESS.json').read_text())
    if abs(progress['maximum_measured_native']-progress['minimum_measured_native'])>1e-8:
        raise ValueError('FRAME_TIMESTAMPED_NATIVE_OBJECT_STATE_REQUIRED_FOR_MOVING_EPISODE')
    object_q=float(progress['current_measured_native'])
    # Native contact telemetry uses the physical door_link, not the abstract
    # articulation child. Substituting the latter changes the hinge prior and
    # collision transform despite an identical coordinate value.
    entries=[e for e in export['shapes']if e.get('rigid_body_path','').split('/')[-1]==model.manifest['door_link']]
    if not entries:raise ValueError('RECORDED_NATIVE_CONTACT_BODY_REQUIRED')
    M=np.asarray(entries[0]['rigid_body_world_transform'])
    trace=json.loads((episode/'AGENT_ACTION_TRACE.json').read_text());matching=[]
    for entry in trace:
        result=entry.get('result',{});observation=result.get('observation',{}).get('observation',{})
        if observation.get('t_sim')==camera['t_sim']and result.get('evaluation',{}).get('feasible'):
            matching.append(entry)
    if len(matching)!=1:raise ValueError('UNAMBIGUOUS_RECORDED_WRIST_COMMAND_ENDPOINT_REQUIRED')
    endpoint=np.asarray(matching[0]['result']['evaluation']['route']['q_path'][-1],dtype=np.float32)
    equilibrium=measured.copy();equilibrium[arm]=endpoint
    policy=Policy.load(Path(__file__).resolve().parents[1]/'configs/interaction_runtime_v1.json')
    source_dir=output/'source';source_dir.mkdir();source=dict(source,robot_base_pose=base.tolist())
    (source_dir/'report.json').write_text(json.dumps(source))
    job=copy.deepcopy(job);job.update(output=str(output),source=str(source_dir),benchmark_goal_rad=math.radians(3),visual_planning_wall_s=900)
    shutil.copy2(collision_export,output/'cooked_initial.json')
    from native_g1_metric_rgbd_geometry import preserve
    data=multi.build_geometry(frame,[],output/'agent_geometry')
    g=dict(ROOT=Path(__file__).resolve().parents[1],a=SimpleNamespace(output=output),job=job,
        model=model,solver=solver,base=base,robot=SimpleNamespace(get_joint_positions=lambda:measured.copy()),
        qtarget=equilibrium,arm=arm,fingers=fingers,FINGERS=['gripper_link1','gripper_link2'],
        moving=lambda:M.copy(),actual=lambda:object_q,collision=collision,allowed=allowed,
        dt=1/240,wrist_routes=[],execution_measured_fingers=True,
        ik=lambda T,q:solver.solve(T,base,q),normal=np.asarray(data['normal_world']))
    g['rows']=[dict(T_moving_link=M.tolist())]
    g['tcp']=lambda:np.asarray(camera['T_world_tcp_measured']).copy()
    from native_g1_ft_identification import kinematic_prior
    recorded_prior=json.loads((episode/'ARTICULATION_INTERFACE.json').read_text())['prior']
    reproduced_prior=kinematic_prior(g)
    if not all(np.allclose(reproduced_prior[k],recorded_prior[k],atol=1e-6,rtol=0)
            for k in ['origin_world_m','axis_world']):
        raise ValueError('RECONSTRUCTED_NATIVE_HINGE_PRIOR_MISMATCH')
    (output/'STATE_RECONSTRUCTION.json').write_text(json.dumps(dict(
        moving_body=model.manifest['door_link'],native_pose_source=str(collision_export),
        wrist_command_sequence=matching[0]['command']['sequence'],command_precision='float32',
        retained_command_bias_rad=(equilibrium[arm]-measured[arm]).tolist(),
        native_hinge_prior_matched=True,zero_bias_substitution=False),indent=2)+'\n')
    g['policy']=json.loads((Path(__file__).resolve().parents[1]/'config/semantic_interaction.json').read_text())
    g['runtime_policy']=policy
    actions=[dict(role='FIT_FORWARD',coordinate_unit='rad',target_q=math.radians(3),
        reference_speed_m_s=.008,reference_acceleration_m_s2=.025,profile='constant',
        fit_intervals_s=[[.6,12.]],allow_target_truncation=False,hold=True)]
    original=multi.rescore;proposal_audit=[]
    def rescore(geometries,model,surfaces,base,origin,axis,config,out):
        if scored_cache:
            cached=json.loads(Path(scored_cache).read_text())
            for item in cached:item['T']=np.asarray(item['T'])
            return cached
        if mode=='rule':return original(geometries,model,surfaces,base,origin,axis,config,out)
        generated=geometry_candidates(json.loads(Path(inference).read_text()),camera,data,model,
            surfaces,multi.closing_scene(model,output/'agent_geometry'),base,np.asarray(origin),axis)
        proposal_audit.extend(dict(pose_id=c['pose_id'],generator_rank=c['generator_rank'],
            contact_rank=c['contact']['rank'],observed_closing_blocker=c['contact']['non_handle_closing_collision'])for c in generated)
        (output/'GENERATED_CANDIDATES.json').write_text(json.dumps(proposal_audit,indent=2))
        import native_g1_robust_contact_ranking as ranking
        with patch.object(ranking,'diverse_shortlist',novel_shortlist):
            return original(generated,model,surfaces,base,origin,axis,config,out)
    candidate=None;error=None
    with patch.object(multi,'rescore',rescore),context(g,policy,config,solver)as checker,candidate_gate(actions,math.radians(3)):
        try:candidate,_=multi.plan_observed(g,output/'agent_geometry')
        except Exception as failure:error=repr(failure)
    result=dict(mode=mode,source_episode=episode.name,frame=str(frame),
        generated_count=len(json.loads(Path(inference).read_text())['grasps'])if mode=='graspgenx'else len(proposal_audit),
        scored_cache=str(scored_cache)if scored_cache else None,checked_candidates=len(checker.audit),
        safe_executable_candidates=sum(bool(c.get('valid')and c.get('v81_revolute_chain',{}).get('valid'))for c in checker.audit),
        selected=bool(candidate),failure=error,complete_native_checker_audit=checker.audit,
        scope='offline same-frame native geometry, approach, timed motion, closure and stopped-exit checks; no physical grasp',
        no_GT_physics_read=True,physical_motion_executed=False)
    (output/'RESULT.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items()if k!='complete_native_checker_audit'}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode');p.add_argument('frame');p.add_argument('inference');p.add_argument('output');p.add_argument('--mode',choices=['rule','graspgenx'],required=True)
    p.add_argument('--scored-cache')
    a=p.parse_args();compare(a.episode,a.frame,a.inference,a.output,a.mode,a.scored_cache)
