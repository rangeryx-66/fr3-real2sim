"""Offline V1.1 evidence summary. Never imported by the physical actor."""
import argparse,json
from pathlib import Path
import numpy as np
from analyze_native_g1_contact_recovery import analyze,stream,read


def summarize(root):
    output=analyze(root);ep=root/'episode';run=read(ep/'RECOVERY_EPISODE_RESULT.json')
    output['experiment_type']='one-case bounded safe-termination development; not robustness or hardware validation'
    output['exit_version']='V1.1'
    output['manipulation_success']=run['manipulation_success']
    output['full_lifecycle_success']=run['full_lifecycle_success']
    for result,attempt in zip(output['attempts'],run['attempts']):
        exit=attempt['safe_exit'];plan=exit.get('joint_exit_plan',{})
        result.update(release_completed=exit.get('released',False),clearance_completed=exit.get('clearance_reached',False),
            release_and_clearance_completed=bool(exit.get('released') and exit.get('clearance_reached')),
            safe_park_reached=exit.get('safe_park_reached',False),terminal_state=exit.get('terminal_state'),
            parking_verification=exit.get('parking_verification'),exit_planning_wall_s=plan.get('planning_wall_s'),
            route_candidates=plan.get('candidates'),selected_parking_region=plan.get('parking_region'),
            route_method=plan.get('route_method'),executed_segment_revalidations=len(exit.get('executed_segment_checks',[])),
            post_clearance_return_audit=exit.get('post_clearance_return_audit'),
            post_clearance_local_replan=exit.get('post_clearance_local_replan'))
    last=None;phase={};minimum=float('inf');previous=None;maximum_tcp=0.;force_events=[]
    for row in stream(ep,'observations'):
        last=row;minimum=min(minimum,row['margin_rad'])
        p=phase.setdefault(row['phase'],dict(samples=0,start_sim_s=row['t'],end_sim_s=row['t'],maximum_pad_n=0.))
        p['samples']+=1;p['end_sim_s']=row['t'];p['maximum_pad_n']=max(p['maximum_pad_n'],max(row['forces_n'].values()))
        if row.get('force_event',{}).get('status','NORMAL')!='NORMAL':
            force_events.append(dict(t=row['t'],phase=row['phase'],event=row['force_event']))
        if previous is not None:
            speed=np.linalg.norm(np.asarray(row['T_tcp'])[:3,3]-np.asarray(previous['T_tcp'])[:3,3])/(row['t']-previous['t'])
            maximum_tcp=max(maximum_tcp,float(speed))
        previous=row
    for p in phase.values():p['recorded_simulation_duration_s']=p['samples']/240.
    output.update(final_measured_state={k:last[k] for k in ['q','door_angle_deg','aperture_m','forces_n','margin_rad','T_tcp','T_moving_link','phase','t']},
        phase_simulation=phase,whole_episode_minimum_margin_rad=minimum,whole_episode_maximum_tcp_mm_s=maximum_tcp*1000,
        final_filtered_pad_loads_n=last['telemetry']['pad_load_filtered_n'],
        final_joint_velocity_rad_s=last['telemetry']['joint_velocities'],
        final_commanded_aperture_m=last['telemetry']['command_aperture_m'],
        all_phase_peak_pad_load_n=max(p['maximum_pad_n'] for p in phase.values()),
        all_phase_non_normal_force_events=force_events,
        terminal_events=[e for e in read(ep/'events.json') if e.get('event') in ['GRASP_VERIFIED','SHORT_OPENING_START','RECOVERY_PHYSICAL_RELEASE','CLEARANCE_REACHED','SAFE_PARK_REACHED']])
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=summarize(a.root);a.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:result[k] for k in ['manipulation_success','full_lifecycle_success','physical_grasp_attempts','simulation_s','wall_s']},indent=2))
