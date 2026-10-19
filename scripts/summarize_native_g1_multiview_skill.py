"""Passive post-run metrics/plots. Not imported by the physical runner."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from analyze_native_g1_visual_contact_mismatch import records,summarize

def intervals(t,missing,dt):
    spans=[];start=None
    for i,v in enumerate(missing):
        if v and start is None:start=i
        if start is not None and (not v or i==len(missing)-1):
            end=i if not v else i+1
            spans.append(dict(start_s=float(t[start]),duration_s=float(t[end-1]-t[start]+dt)));start=None
    return spans

def analyze(folder,output):
    report=json.loads((folder/'report.json').read_text());physics=records(folder,'physics_steps');obs=records(folder,'observations');detail=summarize(folder)
    metadata=json.loads((folder/'sensor_metadata.json').read_text());arm=metadata['arm_indices'];dt=float(metadata['physics_dt_s'])
    t=np.array([r['t'] for r in physics]);P=np.array([r['T_tcp'] for r in physics]);angle=np.array([r['door_angle_deg'] for r in physics]);speed=np.r_[0,np.linalg.norm(np.diff(P[:,:3,3],axis=0),axis=1)/np.diff(t)]
    loads=np.array([[sum(c.get('force_n',0.) for c in r.get('contacts',[]) if c.get('finger')==n and c.get('allowed_pad_target')) for n in ['gripper_link1','gripper_link2']] for r in physics])
    acquisition=report.get('grasp_acquisition');post=t>=acquisition['t'] if acquisition else np.zeros(len(t),bool)
    first={n:None if r is None else {k:v for k,v in r.items() if k!='contacts'} for n,r in detail['first_actual_positive_normal_contact'].items()}
    tracking=[float(np.max(abs(np.asarray(r['q'])[arm]-np.asarray(r['command_q_arm'])))) for r in obs if r.get('command_q_arm') is not None]
    target_error=[float(np.linalg.norm(np.asarray(r['T_tcp'])[:3,3]-np.asarray(r['planned_T_tcp'])[:3,3])) for r in obs if r.get('planned_T_tcp') is not None]
    raw_keys=['raw_contact_records','pad_load_raw_n','pad_load_filtered_n','joint_velocities','applied_joint_efforts','measured_joint_efforts','measured_joint_forces','native_gripper_feedback']
    completeness={k:sum(r.get('telemetry',{}).get(k) is not None for r in obs) for k in raw_keys}
    commands=records(folder,'closure_commands');closure_t=np.array([r['t'] for r in commands])
    result=dict(report=report,first_contacts=first,actual_gain_after_acquisition_deg=float(report['max_actual_angle_after_valid_grasp_deg']-acquisition['angle_deg']) if post.any() else None,
        physics_pre_step_gain_after_acquisition_deg=float(angle[post].max()-acquisition['angle_deg']) if post.any() else None,
        angle_sampling_note='Primary gain uses post-step observation/report maximum minus acquisition event angle; native contact geometry snapshots precede integration by one step.',
        closure_relative_motion_from_first_contact=detail.get('closure_relative_motion_from_first_contact'),
        closure_state_first_records=detail.get('first_closure_state_records'),final_closure=detail.get('final_closure'),
        actual_gain_after_opening_start_deg=float(report['max_actual_angle_after_valid_grasp_deg']-report['opening_start_angle_deg']) if post.any() and report.get('opening_start_angle_deg') is not None else None,
        peak_raw_allowed_normal_load_n=float(loads.max()),max_all_phase_TCP_mm_s=float(speed.max()*1000),
        max_post_acquisition_TCP_mm_s=float(speed[post].max()*1000) if post.any() else None,
        max_post_acquisition_relative_translation_mm=max((r['relative_translation_slip_m']*1000 for r in physics if acquisition and r['t']>=acquisition['t']),default=None),
        max_post_acquisition_relative_rotation_deg=max((r['relative_rotation_slip_deg'] for r in physics if acquisition and r['t']>=acquisition['t']),default=None),
        max_joint_command_tracking_rad=max(tracking,default=None),max_planned_TCP_tracking_mm=max(target_error,default=0)*1000,
        positive_normal_absence_after_acquisition={n:intervals(t[post],loads[post,i]<=0,dt) for i,n in enumerate(['left','right'])} if post.any() else {},
        telemetry_records=dict(physics_steps=len(physics),observations=len(obs),closure_commands=len(commands)),
        telemetry_fields_present_records=completeness,expected_native_rate_hz=1/dt,physics_timestamps_uniform=bool(np.allclose(np.diff(t),dt)),
        positive_normal_absence_is_not_true_grasp_loss=True,normal_loads_are_partial_not_complete_wrenches=True)
    output.mkdir(parents=True,exist_ok=True);(output/'METRICS.json').write_text(json.dumps(result,indent=2))
    fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True);stride=24;ii=np.arange(0,len(t),stride)
    axes[0].plot(t[ii],angle[ii]);axes[0].set_ylabel('Actual angle (deg)')
    for i,n in enumerate(['left','right']):
        maxima=np.array([loads[j:min(j+stride,len(t)),i].max() for j in ii])
        axes[1].plot(t[ii],maxima,label=n)
    axes[1].set_ylabel('Normal load (N)\n100ms maxima');axes[1].legend()
    axes[2].plot([r['t'] for r in obs],[r.get('telemetry',{}).get('command_aperture_m',np.nan)*1000 for r in obs],label='command')
    axes[2].plot(t[ii],np.array([r['aperture_m']*1000 for r in physics])[ii],label='measured')
    axes[2].set_ylabel('Aperture (mm)');axes[2].legend()
    axes[3].plot(t[ii],np.array([r['relative_translation_slip_m']*1000 for r in physics])[ii]);axes[3].set_ylabel('Post-acquisition\nrelative motion (mm)');axes[3].set_xlabel('Main episode simulation time (s)')
    if acquisition:
        for ax in axes:ax.axvline(acquisition['t'],color='gray',ls='--')
    fig.suptitle(folder.parent.name+' | multi-view visual input / known hinge-state / approximate actuator');fig.tight_layout();fig.savefig(output/'trace.png',dpi=160);plt.close(fig)
    print(json.dumps({k:result[k] for k in ['actual_gain_after_acquisition_deg','peak_raw_allowed_normal_load_n','max_post_acquisition_TCP_mm_s','telemetry_records']}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();analyze(a.run,a.output)
