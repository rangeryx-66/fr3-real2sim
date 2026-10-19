"""Private post-run evidence -> compact per-attempt development result.

No simulation or execution imports. Original streams are never rewritten.
"""
import argparse,gzip,hashlib,json,pickle,subprocess
from pathlib import Path
import numpy as np


def read(path):return json.loads(path.read_text())
def stream(ep,name):
    root=ep/(name+'_chunks')
    for part in read(root/'index.json')['chunks']:
        yield from pickle.load(gzip.open(root/part['file'],'rb'))
def spans(t,missing,dt=1/240):
    d=np.diff(np.r_[False,missing,False].astype(int));return [[float(t[i]),float(t[j-1]+dt)] for i,j in zip(np.flatnonzero(d==1),np.flatnonzero(d==-1))]


def analyze(root,render_contact=False):
    ep=root/'episode';result=read(ep/'RECOVERY_EPISODE_RESULT.json');events=read(ep/'events.json');obs=list(stream(ep,'observations'));rows=[]
    for attempt in result['attempts']:
        states=obs[attempt['first_row']:attempt['last_row']];acq=attempt['acquisition'];t0=acq['t'] if acq else (states[0]['t'] if states else 0.)
        relevant=[r for r in states if r['t']>=t0];t=np.array([r['t']-t0 for r in relevant]);raw=np.array([[r['telemetry']['pad_load_raw_n'][n] for n in ['gripper_link1','gripper_link2']] for r in relevant]);filtered=np.array([r['telemetry']['pad_load_filtered_n'] for r in relevant])
        opening=next((e for e in events if e.get('event')=='SHORT_OPENING_START' and e.get('attempt_index')==attempt['index']),None)
        hold=(t>=0)&(t<opening['t']-t0) if opening else np.zeros(len(t),bool)
        caps=[];body_available=0;targets_available=0
        for r in states:
            f=r.get('native_g1_interface') or {};cap=f.get('runtime_max_efforts',{})
            if cap.get('available'):
                v=np.asarray(cap['value']).reshape(-1);caps.append(float(v[7]))
            if all(f.get('actual_finger_body_world_poses',{}).get(n,{}).get('available') for n in ['left_finger','right_finger']):body_available+=1
            if f.get('applied_action_readback',{}).get('joint_positions',{}).get('available'):targets_available+=1
        relative=[np.linalg.inv(np.asarray(r['T_moving_link']))@np.asarray(r['T_tcp']) for r in relevant];relative_origin=relative[0] if relative else np.eye(4)
        relative_mm=[float(np.linalg.norm(T[:3,3]-relative_origin[:3,3])*1000) for T in relative]
        relative_deg=[float(np.degrees(np.arccos(np.clip((np.trace(relative_origin[:3,:3].T@T[:3,:3])-1)/2,-1,1)))) for T in relative]
        pos=np.array([r['T_tcp'] for r in states])[:,:3,3] if states else np.zeros((0,3));ts=np.array([r['t'] for r in states]);speed=np.linalg.norm(np.diff(pos,axis=0),axis=1)/np.diff(ts)*1000 if len(states)>1 else np.array([])
        summary=dict(index=attempt['index'],family=attempt['family'],physical_grasp_started=attempt['physical_grasp_started'],bilateral_acquisition=acq is not None,acquisition=acq,
            hold_simulation_s=None if not opening or not acq else opening['t']-t0,actual_post_acquisition_gain_deg=attempt['opening_gain_deg'],net_post_acquisition_gain_deg=(states[-1]['door_angle_deg']-acq['angle_deg']) if states and acq else None,controller_target_reached=attempt['controller_target_reached'],short_success=attempt['short_success'],
            termination_reason=attempt['termination_reason'],safe_exit=attempt['safe_exit']['safe'],safe_exit_reason=attempt['safe_exit'].get('reason'),
            max_world_TCP_speed_during_attempt_mm_s=float(speed.max()) if len(speed) else None,
            max_contact_positive_TCP_speed_mm_s=max((float(v) for v,r in zip(speed,states[1:]) if max(r['telemetry']['pad_load_raw_n'].values())>0),default=None),peak_raw_post_acquisition_n=raw.max(0).tolist() if len(raw) else None,peak_filtered_post_acquisition_n=filtered.max(0).tolist() if len(filtered) else None,
            single_pad_positive_normal_absence_after_acquisition_s={n:spans(t,raw[:,i]==0) for i,n in enumerate(['left','right'])} if len(raw) else {},
            hold_raw_positive_normal_absence_s={n:spans(t,(raw[:,i]==0)&hold) for i,n in enumerate(['left','right'])} if len(raw) else {},
            first_positive_contact={n:next((dict(t=r['t'],aperture_mm=r['aperture_m']*1000,commanded_aperture_mm=r['telemetry']['command_aperture_m']*1000,raw_n=r['telemetry']['pad_load_raw_n'][n]) for r in states if r['telemetry']['pad_load_raw_n'][n]>0),None) for n in ['gripper_link1','gripper_link2']},maximum_relative_translation_mm=max(relative_mm,default=None),maximum_relative_rotation_deg=max(relative_deg,default=None),relative_motion_anchor='first full-resolution acquired sample; fixed across hold/opening; no controller-reference reset',minimum_joint_margin_rad=attempt['minimum_joint_margin_rad'],physical_attempt_wall_s=attempt['physical_attempt_wall_s'],safe_exit_simulation_s=attempt['safe_exit'].get('end_sim_s',0)-attempt['safe_exit']['start_sim_s'] if attempt['safe_exit'].get('end_sim_s') is not None else None,
            interface=dict(states=len(states),actual_finger_body_pose_records=body_available,position_target_readback_records=targets_available,running_master_max_effort_range_nm=[min(caps),max(caps)] if caps else None,independent_drive_force_available=False),
            private_evidence=attempt['folder'])
        plan=read(Path(attempt['folder'])/'selected_plan.json')
        summary['predicted_first_contacts_before_refinement']=plan.get('contact_geometry_audit',{}).get('first_contacts',{})
        summary['release_and_clearance_completed']='unloaded_return_check' in attempt['safe_exit']
        if render_contact:
            import native_g1_recovery_telemetry as passive
            post=ep/'postrun_contact_inspection'/('%02d'%attempt['index']);post.mkdir(parents=True,exist_ok=True)
            packet=passive.inspect_result(dict(rows=obs,telemetry_events=events,FINGERS=['gripper_link1','gripper_link2']),dict(attempt,folder=str(post)))
            packet.update(offline_after_actor_export=True,inspected_only_after_unloaded_safe_exit=False,original_runner_safe_exit=attempt['safe_exit']['safe'],source_attempt_folder=attempt['folder'],wrist_image=str(Path(attempt['folder'])/'stopped_wrist.png'),diagnostic_image=str(Path(attempt['folder'])/'stopped_diagnostic.png'))
            (post/'CONTACT_INSPECTION.json').write_text(json.dumps(packet,indent=2))
        rows.append(summary)
    process=read(root/'PROCESS_EXIT.json') if (root/'PROCESS_EXIT.json').exists() else {}
    sizes={name:dict(records=sum(c['records'] for c in read(ep/(name+'_chunks/index.json'))['chunks']),chunks=len(read(ep/(name+'_chunks/index.json'))['chunks'])) for name in ['physics_steps','observations','closure_commands']}
    definition=read(ep/'NATIVE_G1_INTERFACE_DEFINITION.json')
    output=dict(final_short_transmission_achieved=any(a['short_success'] for a in result['attempts']),runner_completion_success=result['success'],asset_id=read(root/'job.json')['asset_id'],success=result['success'],status=result['status'],attempts=rows,
        physical_grasp_attempts=result['physical_grasp_attempts'],recovery_grasps=result['recovery_grasps'],recovery_triggered=result['recovery_grasps']>0,
        Agent_adjustments=result['Agent_adjustments'],observation_count=sum(len(g) for g in result['observation_generations']),simulation_s=result['simulation_s'],wall_s=result['wall_s'],supervised_actor_wall_s=process.get('end_unix_s',0)-process.get('start_unix_s',0) if process else None,
        telemetry=sizes,interface_startup_metadata_max_efforts=definition['runtime_max_efforts'],
        sensor_force_limitations=definition['independent_motor_drive_force'],raw_evidence_public=False,source_sha256=read(ep/'RECOVERY_EXECUTABLE_HASH.json')['sources'],
        known_dependencies=['registered station','known hinge/dynamic state','original scene collisions','simulated wrist mount without validated camera housing','approximate finite-effort actuator'],
        experiment_type='two-case paired development; not a population robustness estimate')
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--render-contact',action='store_true');a=p.parse_args();out=analyze(a.root,a.render_contact);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k not in ['source_sha256','attempts']},indent=2))
