"""Explicit archived recovery diagnostic; no actor, motion, or online approval."""
import json,sys,time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
stage,ep,out=map(Path,sys.argv[1:4]);sys.path[:0]=[str(stage),str(stage/'scripts')]
out.mkdir(parents=True,exist_ok=False)
def read(p):return json.loads(Path(p).read_text())
import xarm7_native_gripper as hand
hand.OUT=ep/'robot_io'
import native_g1_multiview_input as multi
import xarm7_native_planning as native
import native_g1_robust_contact_ranking as frozen
from native_g1_prismatic_ranking import rescore
from interaction_runtime.kinematics import FastNativeModel
from interaction_runtime.prismatic_planning_v73 import native_precision_ik_class
from interaction_runtime.prismatic_candidate_v75 import PrismaticCandidateCheckV75
from interaction_runtime.policy import Policy
job=read(ep/'job.json');job.update(source=str(ep/'station_states/01'),benchmark_goal_rad=.05,visual_planning_wall_s=240)
source=read(Path(job['source'])/'report.json');base=np.array(source['robot_base_pose'])
model=FastNativeModel(None,job['asset_root'],source)
solver=native_precision_ik_class()(hand.configuration());model.ik=solver.solve
meta=read(ep/'rgbd/generation_04_agent_02/camera.json');actual=float(read(ep/'DECISION_PACKET.json')['joint_coordinate_m'])
runtime=read(ep/'EFFECTIVE_RUNTIME.json');arm=np.array(runtime['arm_indices']);fingers=np.array(runtime['finger_indices'])
nearest=None
for path in sorted((ep/'telemetry').glob('*.npz')):
    with np.load(path)as z:
        i=int(np.argmin(abs(z['t']-meta['t_sim'])));error=float(abs(z['t'][i]-meta['t_sim']))
        if nearest is None or error<nearest[0]:nearest=(error,z['q'][i].copy(),z['command_q'][i].copy(),path.name,int(i))
error,q_numeric,command,chunk,row=nearest
q=np.array(meta['robot_q'])
with np.load(ep/'telemetry'/chunk)as z:
    bracket=np.flatnonzero(abs(z['t']-meta['t_sim'])<=.101)
    stable_command=bool(len(bracket)>=2 and np.max(np.ptp(z['command_q'][bracket],axis=0))<1e-12)
state_error=float(np.max(abs(q-q_numeric)))
if error>.051 or state_error>1e-6 or not stable_command:raise ValueError('ARCHIVED_STATE_TIME_ALIGNMENT_UNAVAILABLE')
qt=q.copy();qt[arm]=command
allowed=read(ep/'station_states/01/prior_station_visual_plan.json')['allowed_authored_colliders']
export=read(ep/'planning_current_scene.json');M=np.array(next(e['rigid_body_world_transform']for e in export['shapes']if e['path']in allowed))
collision=native.scene_factory(stage,export,model,allowed)
policy=Policy(runtime['policy'],runtime['policy_sha256'])
closure=read(ep/'PRELOAD_CONFIGURATION.json')['active_policy'];closure['preload_n']=1.5
actions=read(ep/'agent_commands/013.json')['planned_segments']
(out/'agent_geometry').symlink_to(ep/'agent_geometry',target_is_directory=True)
g=dict(model=model,job=job,allowed=allowed,a=SimpleNamespace(output=out),base=base,
       planning_collision_export=ep/'planning_current_scene.json',planning_start_state_rad=actual,
       robot=SimpleNamespace(get_joint_positions=lambda:q.copy()),arm=arm,fingers=fingers,
       qtarget=qt,moving=lambda:M,actual=lambda:actual,collision=collision,solver=solver,
       ik=lambda T,seed:solver.solve(T,base,seed,starts=1),wrist_routes=[],
       tcp=lambda:model.poses(q[arm],base,finger_q=q[fingers])['tcp_link'],
       execution_measured_fingers=True,policy=closure,runtime_policy=policy,FINGERS=native.LOGICAL,
       v75_motion_prior=read(ep/'ARTICULATION_INTERFACE.json')['prior'],
       v75_declared_chain=actions,dt=1/240,v75_periodic_arm_indices=list(range(7)),
       v75_arm_speed_limits=runtime['gripper_config']['arm_speed_limits'])
checker=PrismaticCandidateCheckV75(g,policy,runtime['gripper_config'],solver)
g['execution_candidate_check']=checker
from agentic_interaction_v81.recovery_search import scope,skip_dominated_checks
ops=SimpleNamespace(r=SimpleNamespace(output=out,base_generation=1,base=base),
    g={'agent_observations':read(ep/'AGENT_OBSERVATIONS.json')},acquisitions=1)
selection={'production_scope':True}
# Closed episodes compress raw .npy checkpoints. This diagnostic does NOT
# pretend its archive identity is a live command/observation authorization.
archive_identity=dict(scope='archived offline fixture only',
    observation_catalog_sha256=read(ep/'AGENT_CANDIDATE_CATALOG.json')['observation_sha256'])
began=time.perf_counter();result={}
with patch('agentic_interaction_v81.recovery_search.observation_identity',return_value=archive_identity),scope(ops,1),skip_dominated_checks(),patch.object(multi,'rescore',rescore),patch.object(native,'NativeModel',FastNativeModel),patch.object(native,'RobotPinkIK',native_precision_ik_class()):
    try:
        candidate,data=multi.plan_observed(g,ep/'agent_geometry')
        result.update(found=True,family=[candidate[k]for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']],
                      selected_full_native_chain=candidate['execution_feasibility'])
    except RuntimeError as e:result.update(found=False,reason=str(e))
result.update(scope='ARCHIVED OFFLINE FULL CHAIN; NOT A FRESH GRASP OR PHYSICAL RECOVERY',
    no_Isaac_actor=True,hidden_dynamics_read=False,actual_base=base.tolist(),actual_object_q_m=actual,
    measured_state_source=dict(robot_q='exact camera checkpoint',command_chunk=chunk,row=row,command_time_error_s=error,
        stationary_state_difference_rad=state_error,unchanged_bracketing_command=stable_command),
    return_route='original checker: archived measured state and command directly to native home; no replayed camera waypoints',
    selection=selection,wall_s=time.perf_counter()-began)
(out/'REPLAY_RESULT.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items()if k not in ['selected_full_native_chain','selection']},indent=2))
