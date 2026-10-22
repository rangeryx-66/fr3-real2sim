"""Offline archived terminal state, actual released fingers/native colliders.

Never an online grasp or sensing input. Endpoint equivalence alone does not
approve any path. Both alternatives use the original native collision checks.
"""
import argparse,json,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def run(episode,job_path,output,chain_only=False,attempt=1):
    from interaction_runtime.kinematics import FastNativeModel
    from interaction_runtime.prismatic_planning_v73 import native_precision_ik_class
    from interaction_runtime.prismatic_park_v75 import plan_exit as winding_exit
    from xarm7_native_execution import mapped_export
    from interactive_twin_recovery.mobile import physical_scene_class
    from revolute10_fixed import authored_handle_colliders
    from native_g1_safe_park import plan_exit
    from diagnose_probing_v3 import columns
    from update_probing_v2_model import matrix
    episode=Path(episode);began=time.perf_counter();job=json.loads(Path(job_path).read_text())
    source=json.loads((Path(job['source'])/'report.json').read_text());cfg=json.loads((episode/'EFFECTIVE_RUNTIME.json').read_text())
    model=FastNativeModel(None,Path(job['asset_root']),source);solver=native_precision_ik_class()(cfg['gripper_config'])
    export=json.loads((episode/'planning_current_scene.json').read_text());allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection'])
    scene=physical_scene_class(ROOT)(mapped_export(export),model,allowed);a,index=columns(episode)
    arm=np.asarray(cfg['arm_indices']);fingers=np.asarray(cfg['finger_indices']);qall=a['q'][-1].astype(float)
    command=qall.copy();command[arm]=a['command_q'][-1];M=matrix(a['moving_link_pose'][-1]);T=matrix(a['tcp_pose'][-1])
    base=source['robot_base_pose'];original_normal=np.asarray(json.loads((episode/'agent_geometry/observed_geometry.json').read_text())['normal_world'])
    # visual.prepare assigns this observed normal before physical execution.
    # The initial runtime default is not the post-grasp exit context.
    g=dict(model=model,solver=solver,collision=scene,robot=SimpleNamespace(get_joint_positions=lambda:qall.copy()),
        arm=arm,fingers=fingers,base=base,moving=lambda:M,tcp=lambda:T,qtarget=command,normal=original_normal,
        ik=lambda target,seed:solver.solve(target,base,seed,starts=1),
        v75_periodic_arm_indices=[i for i,n in enumerate(model.names)if any(j.name==n and j.kind in ('revolute','continuous')for j in model.chain.joints.values())],
        v75_arm_speed_limits=cfg['gripper_config']['arm_speed_limits'])
    result=dict(kind='archived offline native collision only',hidden_dynamics_read=False,physics_started=False,
        measured_final_q=qall.tolist(),command_final_q=command.tolist(),moving_pose=M.tolist(),TCP=T.tolist(),
        existing_runtime_retreat_normal=original_normal.tolist(),observed_normal=json.loads((episode/'agent_geometry/observed_geometry.json').read_text())['normal_world'],
        current_state_check=scene.check(model.poses(qall[arm],base,finger_q=qall[fingers]),M,False))
    if chain_only:
        from interaction_runtime.prismatic_chain_v75 import evaluate_chain
        from interaction_runtime.policy import Policy
        trace=json.loads((episode/'AGENT_ACTION_TRACE.json').read_text())
        entries=[e for e in trace if e['command']['operation']=='attempt_grasp_and_probe']
        entry=entries[attempt-1];first=entry['result']['first_record'];i=int(np.searchsorted(a['step'],first))
        selected=json.loads((episode/'attempt_inputs'/('%02d.json'%(attempt-1))).read_text())['selected']
        qall=a['q'][i].astype(float);command=qall.copy();command[arm]=a['command_q'][i]
        M=matrix(a['moving_link_pose'][i]);T=matrix(a['tcp_pose'][i]);start=float(a['joint_q'][i])
        closure=json.loads((episode/'PRELOAD_CONFIGURATION.json').read_text())['active_policy']
        closure=dict(closure,preload_n=entry['command'].get('native_g1_preload_request_n',1.))
        g.update(job=job,policy=closure,FINGERS=['gripper_link1','gripper_link2'],dt=1/240,qtarget=command,
            actual=lambda:start,v75_motion_prior=json.loads((episode/'identification_input.json').read_text())['prior'])
        chain=evaluate_chain(g,selected,entry['command']['segments'],Policy.load(ROOT/'configs/interaction_runtime_v1.json'),cfg['gripper_config'],solver)
        result.update(chain=chain,scope='OFFLINE_ARCHIVED_POSE_DIAGNOSTIC_ONLY; never executable Agent input',wall_s=time.perf_counter()-began)
        Path(output).parent.mkdir(parents=True,exist_ok=True);Path(output).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:chain.get(k)for k in ('valid','reason','role','coordinate_m','center_m','checked_native_configurations')}));return result
    for name,planner in [('original',plan_exit),('equivalent_winding',winding_exit)]:
        start=time.perf_counter()
        try:r=planner(g);result[name]=dict(r,wall_s=time.perf_counter()-start)
        except Exception as e:result[name]=dict(valid=False,error=repr(e),wall_s=time.perf_counter()-start)
    result['wall_s']=time.perf_counter()-began
    Path(output).parent.mkdir(parents=True,exist_ok=True);Path(output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:{f:v.get(f)for f in ('valid','reason','error','parking_region','wall_s')}for k,v in result.items()if k in ('original','equivalent_winding')}))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--chain-only',action='store_true');p.add_argument('--attempt',type=int,default=1);a=p.parse_args();run(a.episode,a.job,a.output,a.chain_only,a.attempt)
