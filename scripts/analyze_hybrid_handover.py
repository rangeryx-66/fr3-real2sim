"""Post-hoc native-target FK versus measured TCP, no controller changes.

planned_T_tcp is stale while the frozen controller has opening=False during
local centering. Actual joint targets, rather than that diagnostic field, are
the command-continuity evidence. No object GT geometry/dynamics are loaded.
"""
import argparse, json, pickle
from pathlib import Path
import numpy as np
from articulated_demo.kinematics import URDFChain, transform


def analyze(episode, segment, output):
    with (segment/'measurements.pkl').open('rb') as f:rows=pickle.load(f)
    config=json.loads((episode/'sensor_assembly/SENSOR_CONFIG.json').read_text())
    metadata=json.loads((episode/'sensor_metadata.json').read_text())
    names=metadata['joint_names'];arm=config['arm_joint_names'];indices=[names.index(n) for n in arm]
    chain=URDFChain(episode/'sensor_assembly/sim_urdf.urdf');base=rows[0]['base']
    B=transform(base[:3],[0,0,np.deg2rad(base[3])])
    def fk(q):return B@chain.root_to_link('tcp_link',dict(zip(arm,q)))
    cmd=np.array([fk(r['command_q_arm']) for r in rows])
    joint=np.array([fk(np.asarray(r['q'])[indices]) for r in rows])
    actual=np.array([r['T_tcp'] for r in rows]);t=np.array([r['t'] for r in rows]);dt=np.diff(t)
    speed=lambda T:np.linalg.norm(np.diff(T[:,:3,3],axis=0),axis=1)/dt*1000
    vcmd,vjoint,vactual=map(speed,[cmd,joint,actual]);i=int(vactual.argmax())+1
    result=dict(peak_index=i,t=float(t[i]),previous_phase=rows[i-1]['phase'],current_phase=rows[i]['phase'],
        commanded_joint_FK_speed_at_stop_mm_s=float(vcmd[i-1]),measured_joint_FK_speed_at_stop_mm_s=float(vjoint[i-1]),
        measured_body_TCP_speed_at_stop_mm_s=float(vactual[i-1]),max_command_FK_speed_mm_s=float(vcmd.max()),
        command_joint_step_rad=(np.asarray(rows[i]['command_q_arm'])-rows[i-1]['command_q_arm']).tolist(),
        FK_to_body_translation_before_mm=float(np.linalg.norm(joint[i-1,:3,3]-actual[i-1,:3,3])*1000),
        FK_to_body_translation_at_stop_mm=float(np.linalg.norm(joint[i,:3,3]-actual[i,:3,3])*1000),
        maximum_recorded_handle_relative_translation_mm=float(max(r['relative_translation_slip_m'] for r in rows)*1000),
        maximum_recorded_handle_relative_rotation_deg=float(max(r['relative_rotation_slip_deg'] for r in rows)),
        planned_pose_field_not_used_as_command_evidence=True,raw_240hz_peaks_preserved=True,
        object_truth_or_hidden_parameters_loaded=False,control_code_modified=False)
    output.mkdir(parents=True,exist_ok=True);(output/'HANDOVER.json').write_text(json.dumps(result,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    x=t-t[0];fig,axes=plt.subplots(3,2,figsize=(13,10))
    loads=np.array([list(r['forces_n'].values()) for r in rows])
    axes[0,0].plot(x,loads[:,0],label='left native normal');axes[0,0].plot(x,loads[:,1],label='right native normal')
    axes[0,0].set_ylabel('N (normal reports, not complete wrench)')
    axes[0,1].plot(x,[r['aperture_m']*1000 for r in rows],label='measured aperture')
    axes[0,1].plot(x,[r['telemetry']['command_aperture_m']*1000 for r in rows],label='commanded aperture')
    axes[0,1].set_ylabel('mm')
    axes[1,0].plot(x,[r['door_angle_deg']-rows[0]['door_angle_deg'] for r in rows],label='actual gain')
    axes[1,0].set_ylabel('deg after opening start')
    axes[1,1].plot(x[1:],vcmd,label='joint target FK');axes[1,1].plot(x[1:],vjoint,label='measured joint FK')
    axes[1,1].plot(x[1:],vactual,label='measured body TCP');axes[1,1].axhline(5,color='red',ls='--',label='unchanged measured 5 mm/s limit')
    axes[1,1].set_ylabel('world TCP mm/s')
    axes[2,0].plot(x,[r['relative_translation_slip_m']*1000 for r in rows],label='recorded TCP/body relative change')
    axes[2,0].set_ylabel('mm; relative-motion proxy')
    tracking=np.array([np.asarray(r['command_q_arm'])-np.asarray(r['q'])[indices] for r in rows])
    for j,n in enumerate(arm):axes[2,1].plot(x,tracking[:,j],lw=.6,label=n)
    axes[2,1].set_ylabel('joint command minus measured rad')
    for ax in axes.flat:
        ax.axvline(x[i],c='red',lw=.7);ax.grid(alpha=.2);ax.legend(fontsize=6);ax.set_xlabel('seconds after opening start')
    fig.suptitle('One physical attempt: original local-centering / opening handover\nNative 240 Hz peaks retained; no new simulation or control modification')
    fig.tight_layout();fig.savefig(output/'HANDOVER.png',dpi=150);plt.close(fig)
    print(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--segment',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();analyze(a.episode,a.segment,a.output)
